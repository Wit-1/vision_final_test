# 视觉组最终考核：装甲板自瞄感知链路

基于 ROS Noetic 和 OpenCV 的独立 catkin 工作空间，实现图像接入、传统装甲板识别、PnP、跨帧选解、检测消息发布及可视化。后续任务见 [PLAN.md](PLAN.md)。

## 环境与构建

环境：Ubuntu 20.04、ROS Noetic、OpenCV 4.x（已验证 4.2.0）及 catkin tools。需安装 compressed_image_transport、cv_bridge、tf2_ros、rqt_image_view、RViz、Python YAML；测试另需 gtest、rostest 和 Python OpenCV/NumPy。包级依赖见各包的 package.xml。

在项目根目录执行：

```bash
./run.sh check
./run.sh build
./run.sh test
```

脚本自动加载 Noetic 和本项目环境，构建结果保存在本项目。测试包含 71 项 C++ 单元用例和 1 项端到端测试，验证范围见[验收记录](docs/acceptance.md)。

## 运行

将原始 bag 放入 task/，在同一 Noetic 环境的两个终端中，从项目根目录执行：

终端一，启动检测、rqt 和 RViz：

```bash
./run.sh launch dataset:=red_4 rviz:=true
```

终端二，开始回放：

```bash
./run.sh play red_4 -r 0.2
```

RViz 收到首份有效 CameraInfo 帧名后才打开，固定坐标系自动选择。`-r 0.2` 为慢速回放，去掉即可原速；可加 `--pause` 暂停启动或 `-l` 循环。回放终端按空格暂停/继续，Ctrl+C 结束。

| 数据集参数 | 素材文件 |
|---|---|
| blue_1 | task/blue_1.bag |
| blue_guard | task/blue_guard.bag |
| red_1_3 | task/red_1&3.bag |
| red_4 | task/red_4.bag |

两个终端的数据集须一致；切换时先停止旧启动和回放。`gui:=false` 关闭 rqt，`rviz:=false` 关闭 RViz，均关闭可用于无桌面运行。

<details>
<summary>不使用 run.sh 时的启动与回放命令</summary>

两个终端均先加载环境：

```bash
source /opt/ros/noetic/setup.bash
source devel/setup.bash
```

分别执行：

```bash
# 终端一
roslaunch ft_bringup demo.launch dataset:=red_4 rviz:=true
# 终端二
rosbag play --clock task/red_4.bag --topics /hk_camera/image_raw/compressed /hk_camera/camera_info -r 0.2
```

</details>

## 参数与输出

公共话题、HSV、几何及 PnP 参数位于 [detector.yaml](src/ft_bringup/config/detector.yaml)，bag 文件名位于 [datasets/](src/ft_bringup/config/datasets)。公共参数先加载，数据集参数覆盖同名键；修改后重启。`debug_details: true` 可显示灯条下标和拒绝原因，在线调参将在第 5 点实现。

四份素材共用 `/hk_camera/image_raw/compressed` 和 `/hk_camera/camera_info`；回放仅提供相机输入和 `/clock`。

| 输出 | 内容 |
|---|---|
| `/ft/detection` | ft_detection/DetectionFrame，有效检测及带图像 header 的空检测帧 |
| `/ft/debug/image` | 框、角点 0～3、帧内 ID、颜色、大小、位姿轴及深度/重投影误差 |
| `/ft/debug/mask/blue`、`/ft/debug/mask/red` | 蓝色、红色 HSV 掩膜 |
| `/tf` | 相机坐标系到 `armor_<帧内ID>` 的变换 |

另开终端查看一帧非空消息：

```bash
source /opt/ros/noetic/setup.bash
source devel/setup.bash
rostopic echo --filter 'len(m.detections) > 0' -n 1 /ft/detection
```

四点采用灯条端点，顺序为左上、右上、右下、左下。角点 x/y 单位像素、z 固定为 0；位姿和 TF 平移单位米，沿用图像时间戳。ID 仅在帧内有效，confidence 为传统几何评分。完整接口、坐标和选解约定见[设计说明](docs/design.md)。

## 当前限制与排查

端点尺寸暂用小板 135×56 mm、大板 230×56 mm，来源为 Alliance2026 开源项目（据本人提供），尚未与素材实物核对。YAML 保留 `confirmed: false` 和临时尺寸开关；`provisional` 提示不阻止运行，真实米制尺度仍待验证。

- rqt 无画面或 RViz 未打开：确认回放已开始、数据集一致，CameraInfo 有输入。容器运行还需配置 DISPLAY 和图形访问权限。
- 空检测时仍看到旧坐标轴：TF 缓存不会由空帧主动删除，应按图像时间戳判断当前观测。
- 反光、遮挡、强透视可能误检或漏检；跨帧选解仍有初始姿态歧义，已测范围见[验收记录](docs/acceptance.md)。

原始 task/、派生资源及 build/、devel/ 等生成物不进入 Git；提供的 ft_detection 消息包保持原样。
