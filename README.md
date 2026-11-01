# 视觉组最终考核：装甲板自瞄感知链路

当前实现第 1 点图像接入：读取原始 bag 的压缩图像和 CameraInfo，缓存相机参数，并发布 `/ft/debug/image`。识别、PnP、跟踪和进阶功能按 [PLAN.md](PLAN.md) 继续实现。

本项目是独立 catkin 工作空间和 Git 仓库，基于 Ubuntu 20.04、ROS Noetic 和 OpenCV 4.x 开发。[验收记录](docs/acceptance.md)记录当前实现的检查依据。

## 环境与构建

准备好 Ubuntu 20.04 和 ROS Noetic 环境后，在项目根目录执行：

```bash
./run.sh check
./run.sh build
```

`run.sh` 会清理旧工作空间的搜索路径，从 `/opt/ros/noetic` 加载基础环境，再加载本项目的构建结果。构建输出保存在本项目内。首次在嵌套目录构建时，脚本会建立明确的工作空间标记，避免 catkin 向上找到父工作空间。

主要依赖为 catkin tools、roscpp、sensor_msgs、cv_bridge、image_transport、compressed_image_transport、OpenCV 4.x、roslaunch、rosbag、rqt_image_view 和 Python YAML。项目已在 OpenCV 4.2.0 上验证；原生或容器化的 Noetic 环境均需提供这些依赖。

## 两个终端运行

两个终端都在已配置 ROS Noetic 的环境中，从同一项目根目录操作，并使用相同的数据集名称。

终端一启动节点和 rqt：

```bash
./run.sh launch dataset:=red_4
```

终端二回放素材：

```bash
./run.sh play red_4 --pause -r 0.2
```

回放最初处于暂停状态，在回放终端按空格开始。rqt 会自动选择节点配置的调试图话题。按空格可暂停或继续，Ctrl+C 结束对应终端的程序。

`-r 0.2` 用于方便观察。需要原速时去掉这一项，需要循环时加入 `-l`。没有桌面窗口时使用：

```bash
./run.sh launch dataset:=red_4 gui:=false
```

数据集名称与素材对应：

| 数据集参数 | 本地素材 | 相机话题前缀 |
|---|---|---|
| `blue_1` | `task/blue_1.bag` | `/hk_camera/` |
| `blue_guard` | `task/blue_guard.bag` | `/hk_camera/` |
| `red_1_3` | `task/red_1&3.bag` | `/hk_camera/` |
| `red_4` | `task/red_4.bag` | `/hk_camera_ca/` |

切换数据集时，停止旧回放和启动进程，再在两个终端使用新的数据集名称。图像接入代码保持不变。

## 参数与数据流

- [detector.yaml](src/ft_bringup/config/detector.yaml)：公共话题默认值、压缩 transport 和三个队列长度，队列默认均为 1。
- [datasets](src/ft_bringup/config/datasets)：每份素材的 bag 文件名、图像基础话题和 CameraInfo 话题。
- [demo.launch](src/ft_bringup/launch/demo.launch)：设置 `/use_sim_time`，启动检测节点，并按 `gui` 参数启动 rqt。
- [detector.launch](src/ft_bringup/launch/detector.launch)：先加载公共 YAML，再加载数据集 YAML，后者覆盖相同键。

参数放在 `/ft_detector` 私有命名空间。修改 YAML 后重新启动；识别参数在线调节将在第 5 点加入。

图像路径：

```text
基础图像话题 + compressed transport
  → /compressed 消息
  → 插件解码为 sensor_msgs/Image
  → cv_bridge::toCvShare 获取 cv::Mat
  → /ft/debug/image
```

CameraInfo 独立订阅，缓存完整 K、D、尺寸和 frame_id。图像输出使用输入图像的时间戳和坐标系。压缩解码和生成输出消息需要分配数据，编码匹配时 `toCvShare` 共享解码后的像素。

`run.sh play` 默认只回放图像、CameraInfo 和 `/clock`；历史目标 TF 和参考检测不参与当前链路。

## 脚本背后的命令

在项目根目录的 ROS Noetic 终端中，启动命令等价于：

```bash
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch ft_bringup demo.launch dataset:=red_4
```

`red_4` 回放命令等价于：

```bash
rosbag play --clock task/red_4.bag \
  --topics /hk_camera_ca/image_raw/compressed /hk_camera_ca/camera_info \
  --pause -r 0.2
```

如果 rqt 没有画面，先确认回放是否仍暂停、两个终端的数据集是否一致，再看节点是否收到 CameraInfo 和图像。如果提示无法连接显示服务器，检查当前环境的 DISPLAY 和图形显示权限；使用容器时，还需配置图形转发。无图形环境可使用 `gui:=false`。

## 素材与生成文件

原始素材放在 `task/`，保持原件，不进入 Git。`src/ft_detection` 来自随包消息定义，保留原样。

`build/`、`devel/`、`.catkin_tools/`、运行日志、bag、派生模型、视频和打包结果均由忽略规则排除。
