#!/usr/bin/env python3
"""Draw the paired linear and rotating prediction endpoints in RViz."""

import math

from ft_msgs.msg import Target, Track
import message_filters
import rospy
from visualization_msgs.msg import Marker


class PredictionComparison:
    def __init__(self):
        track = rospy.get_param("~linear/track_topic")
        target = rospy.get_param("~rotating/target_topic")
        output = rospy.get_param("~output_topic")
        queue = rospy.get_param("~queue_size")
        self.width = rospy.get_param("~line_width_m")
        self.lifetime = rospy.get_param("~lifetime_s")
        if not all(isinstance(topic, str) and topic for topic in (track, target, output)):
            raise ValueError("Comparison topics must be nonempty strings")
        if len({track, target, output}) != 3 or queue <= 0 or not all(
                math.isfinite(value) and value > 0 for value in (self.width, self.lifetime)):
            raise ValueError("Comparison topics, queue or marker dimensions are invalid")
        self.publisher = rospy.Publisher(output, Marker, queue_size=1)
        self.inputs = [message_filters.Subscriber(track, Track, queue_size=queue),
                       message_filters.Subscriber(target, Target, queue_size=queue)]
        self.synchronizer = message_filters.TimeSynchronizer(self.inputs, queue, reset=True)
        self.synchronizer.registerCallback(self.receive)

    def receive(self, track, target):
        if not target.header.frame_id:
            return
        marker = Marker()
        marker.header = target.header
        marker.ns = "ft_prediction_comparison"
        marker.id = 0
        marker.type = Marker.LINE_LIST
        marker.action = Marker.DELETE
        marker.pose.orientation.w = 1.0
        marker.lifetime = rospy.Duration(self.lifetime)
        paired = track.header.frame_id == target.header.frame_id
        points = (track.position_predict, target.position_predict)
        finite = all(math.isfinite(value) for point in points for value in (point.x, point.y, point.z))
        if paired and track.id and track.tracking and target.target_id and target.valid and finite:
            marker.action = Marker.ADD
            marker.scale.x = self.width
            marker.color.r = marker.color.g = marker.color.b = 0.8
            marker.color.a = 0.8
            marker.points = list(points)
        self.publisher.publish(marker)


def main():
    rospy.init_node("compare_predictions")
    try:
        comparison = PredictionComparison()
        rospy.spin()
    except (KeyError, TypeError, ValueError) as error:
        rospy.logfatal("Comparison configuration failed: %s", error)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
