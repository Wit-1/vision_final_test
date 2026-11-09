#!/usr/bin/env python3
"""Join two debug image topics at exactly the same image timestamp."""

import math

import cv2
from cv_bridge import CvBridge, CvBridgeError
import message_filters
import rospy
from sensor_msgs.msg import Image


class DetectionComparison:
    def __init__(self):
        first = rospy.get_param("~traditional_topic")
        second = rospy.get_param("~neural_topic")
        output = rospy.get_param("~output_topic")
        self.scale = rospy.get_param("~scale")
        queue = rospy.get_param("~queue_size")
        if not all(isinstance(topic, str) and topic for topic in (first, second, output)):
            raise ValueError("Comparison topics must be nonempty strings")
        if len({first, second, output}) != 3 or queue <= 0 or not math.isfinite(self.scale) or not 0 < self.scale <= 1:
            raise ValueError("Comparison topics, queue or scale are invalid")
        self.bridge = CvBridge()
        self.publisher = rospy.Publisher(output, Image, queue_size=1)
        self.inputs = [message_filters.Subscriber(first, Image, queue_size=queue),
                       message_filters.Subscriber(second, Image, queue_size=queue)]
        self.synchronizer = message_filters.TimeSynchronizer(self.inputs, queue, reset=True)
        self.synchronizer.registerCallback(self.receive)

    def receive(self, traditional, neural):
        if traditional.header.frame_id != neural.header.frame_id:
            rospy.logwarn_throttle(1.0, "Comparison skipped: image frame_ids differ")
            return
        try:
            left = self.bridge.imgmsg_to_cv2(traditional, "bgr8")
            right = self.bridge.imgmsg_to_cv2(neural, "bgr8")
            if left.shape != right.shape or left.size == 0:
                rospy.logwarn_throttle(1.0, "Comparison skipped: image sizes differ or are empty")
                return
            height, width = left.shape[:2]
            size = (max(1, math.floor(width * self.scale + 0.5)),
                    max(1, math.floor(height * self.scale + 0.5)))
            comparison = cv2.hconcat((cv2.resize(left, size), cv2.resize(right, size)))
            message = self.bridge.cv2_to_imgmsg(comparison, "bgr8")
            message.header = traditional.header
            self.publisher.publish(message)
        except (CvBridgeError, cv2.error) as error:
            rospy.logwarn_throttle(1.0, "Comparison image conversion failed: %s", error)


def main():
    rospy.init_node("compare_detections")
    try:
        comparison = DetectionComparison()
        rospy.spin()
    except (KeyError, TypeError, ValueError) as error:
        rospy.logfatal("Comparison configuration failed: %s", error)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
