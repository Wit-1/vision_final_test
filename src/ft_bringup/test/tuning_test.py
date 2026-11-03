#!/usr/bin/env python3
"""Check YAML precedence, actual effects of all 21 live fields, and atomic rejection."""

from pathlib import Path
import copy
import math
import subprocess
import threading
import time
import unittest

import cv2
import numpy as np
import rospy
import rospkg
import rostest
import yaml
from dynamic_reconfigure.client import Client
from dynamic_reconfigure.encoding import decode_config
from dynamic_reconfigure.msg import ConfigDescription
from ft_detector.cfg import DetectorConfig
from ft_detection.msg import DetectionFrame
from sensor_msgs.msg import CameraInfo, CompressedImage, Image


class DetectorTuningTest(unittest.TestCase):
    def setUp(self):
        self.lock = threading.Lock()
        self.frames, self.blue, self.red = {}, {}, {}
        self.index = 0
        config = Path(rospkg.RosPack().get_path('ft_bringup')) / 'config/detector.yaml'
        self.baseline = yaml.safe_load(config.read_text())['tuning']
        self.client = Client('/ft_detector/tuning', timeout=8)
        self.image_pub = rospy.Publisher('/hk_camera/image_raw/compressed', CompressedImage, queue_size=1)
        self.camera_pub = rospy.Publisher('/hk_camera/camera_info', CameraInfo, queue_size=1)
        self.subscribers = [
            rospy.Subscriber('/ft/detection', DetectionFrame, self.receive_frame),
            rospy.Subscriber('/ft/debug/mask/blue', Image, self.receive_blue, queue_size=5, buff_size=4000000),
            rospy.Subscriber('/ft/debug/mask/red', Image, self.receive_red, queue_size=5, buff_size=4000000),
        ]
        self.wait(lambda: self.image_pub.get_num_connections() > 0 and self.camera_pub.get_num_connections() > 0
                  and all(s.get_num_connections() > 0 for s in self.subscribers))
        self.K = np.array([[2000., 0, 720.], [0, 2000., 540.], [0, 0, 1.]])
        info = CameraInfo()
        info.header.frame_id = 'tuning_camera'
        info.width, info.height = 1440, 1080
        info.distortion_model, info.D = 'plumb_bob', [0.0] * 5
        info.K = self.K.ravel().tolist()
        self.camera_pub.publish(info)
        time.sleep(.15)
        self.samples = {
            'blue': self.board(110, 200, .135, (280, 300)),
            'red_low': self.board(5, 250, .135, (650, 550)),
            'red_high': self.board(170, 250, .230, (1120, 820), (.3, .5, -.12)),
        }
        self.scene = np.bitwise_or.reduce(list(self.samples.values()))

    def tearDown(self):
        self.client.close()
        self.image_pub.unregister()
        self.camera_pub.unregister()
        for subscriber in self.subscribers:
            subscriber.unregister()

    def board(self, hue, value, width, centre, rotation=(0., 0., 0.)):
        image = np.zeros((1080, 1440, 3), np.uint8)
        objects = np.array([[-width/2, -.028, 0], [width/2, -.028, 0],
                            [width/2, .028, 0], [-width/2, .028, 0]], dtype=float)
        translation = np.array([(centre[0]-720)*2/2000, (centre[1]-540)*2/2000, 2.])
        pixels = cv2.projectPoints(objects, np.array(rotation), translation, self.K, np.zeros(5))[0].reshape(4, 2)
        colour = cv2.cvtColor(np.uint8([[[hue, 230, value]]]), cv2.COLOR_HSV2BGR)[0, 0].tolist()
        for top, bottom in [(pixels[0], pixels[3]), (pixels[1], pixels[2])]:
            direction = bottom-top
            normal = np.array([direction[1], -direction[0]]) / np.linalg.norm(direction) * 2
            polygon = np.rint([top-normal, top+normal, bottom+normal, bottom-normal]).astype(np.int32)
            cv2.fillConvexPoly(image, polygon, colour)
        return image

    @staticmethod
    def key(header):
        return header.stamp.secs, header.stamp.nsecs, header.frame_id

    def receive_frame(self, message):
        with self.lock:
            self.frames[self.key(message.header)] = message

    def receive_blue(self, message):
        with self.lock:
            self.blue[self.key(message.header)] = int(np.count_nonzero(np.frombuffer(message.data, np.uint8)))

    def receive_red(self, message):
        with self.lock:
            self.red[self.key(message.header)] = int(np.count_nonzero(np.frombuffer(message.data, np.uint8)))

    def wait(self, predicate):
        end = time.monotonic() + 8
        while time.monotonic() < end and not rospy.is_shutdown():
            with self.lock:
                if predicate():
                    return
            time.sleep(.01)
        self.fail('Timed out waiting for tuning output')

    def frame(self, pixels):
        self.index += 1
        message = CompressedImage()
        message.header.frame_id = 'tuning_camera'
        message.header.stamp = rospy.Time(200 + self.index, 12345)
        message.format = 'png'
        message.data = cv2.imencode('.png', pixels)[1].tobytes()
        key = self.key(message.header)
        self.image_pub.publish(message)
        self.wait(lambda: key in self.frames and key in self.blue and key in self.red)
        with self.lock:
            return self.frames[key], self.blue[key], self.red[key]

    def test_yaml_seed_live_effects_and_rejection(self):
        self.assertEqual(len(self.baseline), 21)
        for name, value in self.baseline.items():
            self.assertEqual(DetectorConfig.defaults[name], value, name)
        expected = dict(self.baseline, blue_v_min=220)
        initial = self.client.get_configuration(timeout=5)
        description = rospy.wait_for_message('/ft_detector/tuning/parameter_descriptions', ConfigDescription, timeout=5)
        defaults = decode_config(description.dflt)
        for name, value in expected.items():
            self.assertEqual(initial[name], value, name)
            self.assertEqual(defaults[name], value, name)

        # A bad startup parameter set must fail, rather than be silently replaced by cfg defaults.
        for mode, expected_error in [('missing', 'Missing or wrongly typed'),
                                     ('interval', 'Invalid HSV interval'),
                                     ('ui_range', 'outside the Detector.cfg UI range')]:
            invalid = copy.deepcopy(rospy.get_param('/ft_detector'))
            if mode == 'missing':
                del invalid['tuning']['red_high_v_min']
            elif mode == 'interval':
                invalid['tuning']['blue_h_min'] = 150
            else:
                invalid['tuning']['min_contour_area'] = 20000.
            rospy.set_param('/invalid_detector', invalid)
            try:
                result = subprocess.run(['rosrun', 'ft_detector', 'ft_detector_node', '__name:=invalid_detector'],
                                        capture_output=True, text=True, timeout=8)
                self.assertEqual(result.returncode, 1, result.stdout + result.stderr)
                self.assertIn(expected_error, result.stdout + result.stderr)
            finally:
                rospy.delete_param('/invalid_detector')

        first, blue, red = self.frame(self.scene)
        self.assertEqual(blue, 0)
        self.assertGreater(red, 0)
        self.assertEqual(len(first.detections), 2)

        self.client.update_configuration(self.baseline)
        restored, blue, red = self.frame(self.scene)
        self.assertGreater(blue, 0)
        self.assertEqual(len(restored.detections), 3)

        # Each of the 18 HSV endpoints must change the actual mask, not just the parameter server.
        excluded = {
            'blue': dict(h_min=120, h_max=100, s_min=240, s_max=200, v_min=240, v_max=180),
            'red_low': dict(h_min=10, h_max=4, s_min=240, s_max=200, v_min=255, v_max=240),
            'red_high': dict(h_min=175, h_max=165, s_min=240, s_max=200, v_min=255, v_max=240),
        }
        for prefix, fields in excluded.items():
            for suffix, value in fields.items():
                self.client.update_configuration(self.baseline)
                name = prefix + '_' + suffix
                applied = self.client.update_configuration({name: value})
                self.assertEqual(applied[name], value)
                frame, blue, red = self.frame(self.samples[prefix])
                self.assertEqual(blue if prefix == 'blue' else red, 0, name)
                self.assertFalse(frame.detections, name)

        # Remaining live fields must change pairing or contour acceptance while masks stay nonempty.
        for name, value, count in [('min_contour_area', 1000., 0),
                                   ('max_pair_spacing_ratio', 3.5, 2),
                                   ('max_pair_angle_deg', .1, 2)]:
            self.client.update_configuration(self.baseline)
            self.client.update_configuration({name: value})
            frame, blue, red = self.frame(self.scene)
            self.assertGreater(blue, 0); self.assertGreater(red, 0)
            self.assertEqual(len(frame.detections), count, name)

        self.client.update_configuration(self.baseline)
        for invalid in [{'blue_h_min': 150, 'min_contour_area': 100.},
                        {'max_pair_spacing_ratio': 3.0, 'blue_v_min': 250},
                        {'min_contour_area': float('nan')}]:
            result = self.client.update_configuration(invalid)
            for name, value in self.baseline.items():
                self.assertEqual(result[name], value, name)
                self.assertEqual(rospy.get_param('/ft_detector/tuning/' + name), value, name)
                self.assertTrue(math.isfinite(result[name]))
            frame, _, _ = self.frame(self.scene)
            self.assertEqual(len(frame.detections), 3)
        self.assertEqual(rospy.get_param('/ft_detector/traditional/min_light_length'), 10.)


if __name__ == '__main__':
    rospy.init_node('detector_tuning_test')
    rostest.rosrun('ft_bringup', 'detector_tuning', DetectorTuningTest)
