#!/usr/bin/env python3
"""Verify the real image -> detection/debug/TF path, including empty observations."""

import copy
import threading
import time
import unittest

import cv2
import numpy as np
import rospy
import rostest
from ft_detection.msg import DetectionFrame
from sensor_msgs.msg import CameraInfo, CompressedImage, Image
from tf2_msgs.msg import TFMessage


class DetectionPipelineTest(unittest.TestCase):
    def setUp(self):
        self.lock = threading.Lock()
        self.frames, self.images, self.transforms = {}, {}, {}
        self.index = 0
        self.image_topic = rospy.get_param('/ft_detector/image_topic') + '/compressed'
        self.camera_topic = rospy.get_param('/ft_detector/camera_info_topic')
        self.image_pub = rospy.Publisher(self.image_topic, CompressedImage, queue_size=1)
        self.camera_pub = rospy.Publisher(self.camera_topic, CameraInfo, queue_size=1)
        self.subscribers = [
            rospy.Subscriber(rospy.get_param('/ft_detector/detection_topic'), DetectionFrame, self.receive_frame),
            rospy.Subscriber(rospy.get_param('/ft_detector/debug_image_topic'), Image, self.receive_image,
                             queue_size=5, buff_size=16*1024*1024),
            rospy.Subscriber('/tf', TFMessage, self.receive_tf),
        ]
        self.wait(lambda: self.image_pub.get_num_connections() > 0 and self.camera_pub.get_num_connections() > 0
                  and all(s.get_num_connections() > 0 for s in self.subscribers))
        self.info = CameraInfo()
        self.info.header.frame_id = 'pipeline_camera'
        self.info.width, self.info.height = 1440, 1080
        self.info.distortion_model = 'plumb_bob'
        self.info.D = [0.0]*5
        self.info.K = [2000.,0.,720.,0.,2000.,540.,0.,0.,1.]
        self.source = np.zeros((1080,1440,3),np.uint8)
        # Known synthetic geometry from pipeline.test at Z=2 m; not a real-target measurement.
        for left, right, y, hue in [(300,435,400,110),(850,1080,600,5)]:
            colour = cv2.cvtColor(np.uint8([[[hue,230,250]]]),cv2.COLOR_HSV2BGR)[0,0].tolist()
            for x in [left,right]:
                cv2.rectangle(self.source,(x-2,y-28),(x+2,y+28),colour,-1)

    def tearDown(self):
        for subscriber in self.subscribers:
            subscriber.unregister()
        self.image_pub.unregister()
        self.camera_pub.unregister()

    @staticmethod
    def key(header):
        return header.stamp.secs, header.stamp.nsecs, header.frame_id

    def receive_frame(self,message):
        with self.lock:
            self.frames[self.key(message.header)] = message

    def receive_image(self,message):
        with self.lock:
            self.images[self.key(message.header)] = message

    def receive_tf(self,message):
        with self.lock:
            for transform in message.transforms:
                self.transforms.setdefault(self.key(transform.header),[]).append(transform)

    def wait(self,predicate,timeout=10):
        deadline = time.monotonic()+timeout
        while time.monotonic()<deadline and not rospy.is_shutdown():
            with self.lock:
                if predicate():
                    return
            time.sleep(.02)
        self.fail('Timed out waiting for pipeline output')

    def publish_camera(self,info):
        self.camera_pub.publish(info)
        time.sleep(.15)

    def frame(self,pixels,expected_ids,seconds=None,parent='pipeline_camera'):
        self.index += 1
        message = CompressedImage()
        message.header.stamp = rospy.Time(100+self.index if seconds is None else seconds,123456789)
        message.header.frame_id = parent
        message.format = 'png'
        message.data = cv2.imencode('.png',pixels)[1].tobytes()
        key = self.key(message.header)
        self.image_pub.publish(message)
        self.wait(lambda: key in self.frames and key in self.images)
        if expected_ids:
            self.wait(lambda: len(self.transforms.get(key,[])) == len(expected_ids))
        else:
            time.sleep(.08)
        with self.lock:
            frame, image = self.frames[key], self.images[key]
            transforms = list(self.transforms.get(key,[]))
        self.assertEqual(self.key(frame.header),key)
        self.assertEqual(self.key(image.header),key)
        self.assertEqual((image.width,image.height,image.encoding),(1440,1080,'bgr8'))
        self.assertEqual([d.id for d in frame.detections],expected_ids)
        self.assertEqual(sorted(t.child_frame_id for t in transforms),['armor_'+str(i) for i in expected_ids])
        for detection in frame.detections:
            tf = next(t for t in transforms if t.child_frame_id == 'armor_'+str(detection.id))
            p,q = detection.pose.position,detection.pose.orientation
            values = [p.x,p.y,p.z,q.x,q.y,q.z,q.w,detection.confidence]
            values += [value for corner in detection.corners for value in (corner.x,corner.y,corner.z)]
            self.assertTrue(np.isfinite(values).all())
            self.assertAlmostEqual(p.z,2.0,delta=.02)
            self.assertAlmostEqual(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w,1.0,places=9)
            self.assertEqual((p.x,p.y,p.z),(tf.transform.translation.x,tf.transform.translation.y,tf.transform.translation.z))
            self.assertEqual(q,tf.transform.rotation)
            self.assertTrue(all(c.z == 0 for c in detection.corners))
            self.assertEqual((detection.is_red,detection.is_large_armor),(detection.id == 1,detection.id == 1))

    def test_detection_debug_and_tf_stay_aligned(self):
        self.frame(self.source,[])  # First image before calibration still publishes an empty frame.
        self.publish_camera(self.info)
        self.frame(self.source,[0,1])
        self.frame(np.zeros_like(self.source),[])
        invalid = copy.deepcopy(self.info)
        invalid.K = [0.,0.,720.,0.,2000.,540.,0.,0.,1.]
        self.publish_camera(invalid)
        self.frame(self.source,[])
        self.publish_camera(self.info)
        self.frame(self.source,[0,1])
        self.frame(self.source,[],parent='unmatched_image_frame')
        self.frame(self.source,[0,1],seconds=90)  # Rewind uses image time and fresh pose history.


if __name__ == '__main__':
    rospy.init_node('detection_pipeline_test')
    rostest.rosrun('ft_bringup','detection_pipeline',DetectionPipelineTest)
