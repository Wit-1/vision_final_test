#!/usr/bin/env python3
"""Open the parameter editor after the configured dynamic server is ready."""

import os

import rospy


def main():
    rospy.init_node('ft_reconfigure')
    server = rospy.get_param('~server')
    if not isinstance(server, str) or not server.startswith('/') or server == '/':
        rospy.logerr('An absolute dynamic reconfigure server namespace is required')
        return 1
    server = server.rstrip('/')
    rospy.loginfo('Waiting for dynamic reconfigure server %s', server)
    try:
        # wait_for_service uses wall time; no bag /clock is needed to open this window.
        rospy.wait_for_service(server + '/set_parameters')
    except rospy.ROSInterruptException:
        return 0
    if rospy.is_shutdown():
        return 0
    rospy.signal_shutdown('Starting parameter editor')
    # Replace the helper: roslaunch continues to own the GUI process and its exit status.
    os.execvp('rosrun', ['rosrun', 'rqt_reconfigure', 'rqt_reconfigure', server])
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
