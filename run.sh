#!/usr/bin/env bash
set -eo pipefail

project_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
command_name="${1:-help}"
if [ "$#" -gt 0 ]; then shift; fi

usage() {
    cat <<'USAGE'
在 ROS Noetic 环境中使用：
  ./run.sh check
  ./run.sh build [软件包名或 catkin build 参数]
  ./run.sh test
  ./run.sh launch [dataset:=blue_1] [gui:=true] [rviz:=false]
  ./run.sh play <blue_1|blue_guard|red_1_3|red_4> [rosbag play 参数]

示例（两个终端使用同一数据集）：
  ./run.sh launch dataset:=red_4
  ./run.sh play red_4 --pause -r 0.2
USAGE
}

case "$command_name" in
    help|-h|--help) usage; exit 0 ;;
    check|build|test|launch|play) ;;
    *) usage >&2; exit 2 ;;
esac

if [ ! -f /opt/ros/noetic/setup.bash ]; then
    echo '未找到 /opt/ros/noetic/setup.bash，请在已配置 ROS Noetic 的环境中运行。' >&2
    exit 1
fi

# Start with Noetic alone, even if the shell previously sourced the parent workspace.
unset CMAKE_PREFIX_PATH ROS_PACKAGE_PATH ROSLISP_PACKAGE_DIRECTORIES
unset PYTHONPATH LD_LIBRARY_PATH PKG_CONFIG_PATH
source /opt/ros/noetic/setup.bash
cd -- "$project_root"

load_workspace() {
    if [ ! -f "$project_root/devel/setup.bash" ]; then
        echo '本项目尚未构建，请先运行 ./run.sh build。' >&2
        exit 1
    fi
    source "$project_root/devel/setup.bash"
}

case "$command_name" in
    check)
        command -v catkin >/dev/null
        python3 - <<'PY'
import cv2
import rospkg

packages = ["roscpp", "rospy", "sensor_msgs", "cv_bridge", "image_transport",
            "compressed_image_transport", "message_generation", "message_runtime",
            "rosbag", "roslaunch", "rqt_image_view", "tf2", "tf2_ros", "rviz"]
rospack = rospkg.RosPack()
for name in packages:
    rospack.get_path(name)
print("ROS Noetic 基础依赖可用，OpenCV " + cv2.__version__)
PY
        ;;
    build)
        # catkin init normally searches upward; create metadata at this exact root.
        python3 - "$project_root" <<'PY'
from pathlib import Path
import sys
from catkin_tools.metadata import find_enclosing_workspace, init_metadata_root, set_active_profile

root = Path(sys.argv[1])
(root / "src").mkdir(exist_ok=True)
if not (root / ".catkin_tools/profiles/default/config.yaml").is_file():
    (root / ".catkin_tools/profiles").mkdir(parents=True, exist_ok=True)
    init_metadata_root(str(root))
    set_active_profile(str(root), "default")
assert find_enclosing_workspace(str(root)) == str(root)
PY
        catkin config --workspace "$project_root" --extend /opt/ros/noetic -j2 \
            --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
        exec catkin build --workspace "$project_root" "$@"
        ;;
    test)
        load_workspace
        catkin build --workspace "$project_root" ft_detector --no-deps --no-status \
            --catkin-make-args run_tests
        exec catkin_test_results "$project_root/build/ft_detector/test_results"
        ;;
    launch)
        load_workspace
        exec roslaunch ft_bringup demo.launch "$@"
        ;;
    play)
        if [ "$#" -lt 1 ]; then usage >&2; exit 2; fi
        dataset="$1"
        shift
        load_workspace
        playback_details="$(python3 - "$project_root" "$dataset" <<'PY'
from pathlib import Path
import sys
import yaml

root = Path(sys.argv[1])
profile = root / "src/ft_bringup/config/datasets" / (sys.argv[2] + ".yaml")
if not profile.is_file():
    raise SystemExit("未知数据集：" + sys.argv[2])
# Match detector.launch: common settings first, then dataset overrides.
config = yaml.safe_load((root / "src/ft_bringup/config/detector.yaml").read_text())
config.update(yaml.safe_load(profile.read_text()))
bag = root / "task" / config["bag_file"]
if not bag.is_file():
    raise SystemExit("找不到素材：" + str(bag))
print(bag)
print(config["image_topic"] + "/compressed")
print(config["camera_info_topic"])
PY
)"
        mapfile -t playback_args <<< "$playback_details"
        # Only replay camera inputs and /clock; retain terminal input for pause/resume.
        exec rosbag play --clock "${playback_args[0]}" \
            --topics "${playback_args[1]}" "${playback_args[2]}" "$@"
        ;;
esac
