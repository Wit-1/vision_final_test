#!/usr/bin/env bash
set -eo pipefail

# roslaunch loads the detector's parameters before starting these processes.
# Read the same topic as the detector instead of repeating its value here.
debug_topic="$(rosparam get /ft_detector/debug_image_topic)"
exec rosrun rqt_image_view rqt_image_view "$debug_topic" "$@"
