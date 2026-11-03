#!/usr/bin/env python3
"""Exercise tracking, ID lifetime, image time and marker deletion through the actual ROS node."""

import math
import threading
import time
import unittest

import rospy
import rostest
from ft_detection.msg import Detection, DetectionFrame
from ft_msgs.msg import Track
from visualization_msgs.msg import Marker


class TrackerPipelineTest(unittest.TestCase):
    def setUp(self):
        self.lock = threading.Lock()
        self.tracks, self.markers = {}, {}
        self.received = 0
        self.publisher = rospy.Publisher('/ft/detection', DetectionFrame, queue_size=1)
        self.subscribers = [rospy.Subscriber('/ft/track', Track, self.receive_track),
                            rospy.Subscriber('/ft/debug/marker', Marker, self.receive_marker)]
        self.wait(lambda: self.publisher.get_num_connections() > 0
                  and all(s.get_num_connections() > 0 for s in self.subscribers))
        self.lead = rospy.get_param('/ft_tracker/prediction_time_s')

    def tearDown(self):
        self.publisher.unregister()
        for subscriber in self.subscribers:
            subscriber.unregister()

    @staticmethod
    def key(header):
        return header.stamp.secs, header.stamp.nsecs, header.frame_id

    def receive_track(self, message):
        with self.lock:
            self.tracks[self.key(message.header)] = message
            self.received += 1

    def receive_marker(self, message):
        with self.lock:
            self.markers.setdefault(self.key(message.header), {})[message.id] = message

    def wait(self, predicate):
        deadline = time.monotonic() + 8
        while time.monotonic() < deadline and not rospy.is_shutdown():
            with self.lock:
                if predicate():
                    return
            time.sleep(.01)
        self.fail('Timed out waiting for tracker output')

    def send(self, stamp, positions, tracking, expected_id=None, parent='tracker_camera', marker_parent=None):
        message = DetectionFrame()
        message.header.stamp = rospy.Time.from_sec(stamp)
        message.header.frame_id = parent
        for index, x in enumerate(positions):
            detection = Detection()
            detection.id = 50 + index  # Deliberately unrelated to track IDs.
            detection.pose.position.x, detection.pose.position.z = x, 3.0
            detection.pose.orientation.w = 1.0
            detection.confidence = 1.0
            message.detections.append(detection)
        key = self.key(message.header)
        marker_key = key if marker_parent is None else (key[0], key[1], marker_parent)
        with self.lock:
            self.tracks.pop(key, None)
            self.markers.pop(marker_key, None)
        self.publisher.publish(message)
        self.wait(lambda: key in self.tracks and len(self.markers.get(marker_key, {})) == 3)
        with self.lock:
            track, markers = self.tracks[key], dict(self.markers[marker_key])
        self.assertEqual(self.key(track.header), key)
        self.assertEqual(track.tracking, tracking)
        if expected_id is not None:
            self.assertEqual(track.id, expected_id)
        values = [getattr(vector, axis) for vector in (track.position, track.velocity, track.position_predict)
                  for axis in ('x', 'y', 'z')]
        self.assertTrue(all(math.isfinite(value) for value in values))
        for axis in ('x', 'y', 'z'):
            self.assertAlmostEqual(getattr(track.position_predict, axis),
                                   getattr(track.position, axis) + self.lead * getattr(track.velocity, axis), places=10)
        for marker in markers.values():
            self.assertEqual(marker.ns, 'ft_tracker')
            self.assertEqual(marker.action, Marker.ADD if track.id else Marker.DELETE)
        if track.id:
            self.assertEqual(markers[0].pose.position, track.position)
            self.assertEqual(markers[1].pose.position, track.position_predict)
            self.assertEqual(markers[2].points, [track.position, track.position_predict])
        else:
            self.assertTrue(all(value == 0 for value in values))
        return track

    def test_observation_loss_reacquisition_and_frame_time_resets(self):
        self.send(10.0, [0.0], True, 1)
        seen = self.send(10.1, [.04, 2.0], True, 1)
        first = self.send(10.2, [], False, 1)
        second = self.send(10.3, [], False, 1)
        self.assertGreater(first.position.x, seen.position.x)
        self.assertGreater(second.position.x, first.position.x)
        self.send(10.4, [.16], True, 1)
        with self.lock:
            count = self.received
        time.sleep(.2)  # No input: wall time does not advance the track.
        with self.lock:
            self.assertEqual(count, self.received)
        self.send(10.91, [], False, 0)
        self.send(10.92, [.2], True, 2)
        self.send(10.93, [2.0], False, 2)
        self.send(9.0, [.2], True, 3)
        self.send(9.01, [.2], True, 4, parent='other_camera')
        self.send(9.02, [float('nan')], False, 4, parent='other_camera')
        self.send(9.03, [], False, 0, parent='', marker_parent='other_camera')
        self.send(9.04, [.2], True, 5)
        self.send(9.04, [.2], True, 6)


if __name__ == '__main__':
    rospy.init_node('tracker_pipeline_test')
    rostest.rosrun('ft_bringup', 'tracker_pipeline', TrackerPipelineTest)
