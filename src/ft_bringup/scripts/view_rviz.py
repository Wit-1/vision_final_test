#!/usr/bin/env python3
"""Start RViz with the frame supplied by CameraInfo, independent of dataset names."""

import os
from pathlib import Path
import signal
import subprocess
import threading

import rospy
from sensor_msgs.msg import CameraInfo


def stop_viewer(process):
    if process is None or process.poll() is not None:
        return
    try:
        os.killpg(process.pid, signal.SIGINT)
    except ProcessLookupError:
        return
    try:
        process.wait(timeout=5)
    except subprocess.TimeoutExpired:
        process.kill()
        process.wait()


def main():
    rospy.init_node("ft_rviz_launcher")
    topic = rospy.get_param("/ft_detector/camera_info_topic")
    config = rospy.get_param("~config")
    if not isinstance(topic, str) or not topic or not Path(config).is_file():
        rospy.logerr("RViz requires a CameraInfo topic and an existing configuration file")
        return 1

    wake = threading.Event()
    lock = threading.Lock()
    latest_frame = None

    def receive_camera_info(message):
        nonlocal latest_frame
        frame = message.header.frame_id
        if not frame:
            rospy.logwarn_throttle(1.0, "CameraInfo has no frame_id; waiting to start RViz")
            return
        with lock:
            latest_frame = frame
        wake.set()

    subscriber = rospy.Subscriber(topic, CameraInfo, receive_camera_info, queue_size=1)
    rospy.on_shutdown(wake.set)
    rospy.loginfo("Waiting for CameraInfo on %s to select the RViz fixed frame", topic)
    process = None
    active_frame = None
    try:
        while not rospy.is_shutdown():
            with lock:
                frame = latest_frame
            if frame and frame != active_frame:
                stop_viewer(process)
                rospy.loginfo("Starting RViz with fixed frame %s", frame)
                process = subprocess.Popen(
                    ["rosrun", "rviz", "rviz", "-d", config, "-f", frame, "__name:=ft_rviz"],
                    start_new_session=True,
                )
                active_frame = frame
            if process is not None and process.poll() is not None:
                return process.returncode
            # Viewer lifetime must still be monitored while bag/ROS time is paused.
            wake.wait(timeout=0.1)
            wake.clear()
    finally:
        subscriber.unregister()
        stop_viewer(process)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
