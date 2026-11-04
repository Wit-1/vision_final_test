# 视觉组最终考核：装甲板自瞄感知链路

基于 ROS Noetic 和 OpenCV 的独立 catkin 工作空间，实现图像接入、传统装甲板识别、PnP、跨帧选解、检测消息、单板平移跟踪及可视化。后续任务见 [PLAN.md](PLAN.md)。

## 仓库结构

当前源码与文档结构如下：

```text
vision_final_test/
├── src/
│   ├── ft_detection/         # 素材提供的检测消息包
│   ├── ft_msgs/              # 自定义 Track 消息
│   ├── ft_detector/          # 图像接入、传统检测、PnP、TF 和调参
│   ├── ft_tracker/           # 单板匀速跟踪与提前量预测
│   └── ft_bringup/
│       ├── config/           # 检测、跟踪和数据集参数
│       ├── launch/           # 统一启动及各模块入口
│       ├── scripts/          # 图像、RViz 和调参窗口启动器
│       └── rviz/             # 三维显示配置
├── docs/
│   ├── design.md             # 数据流、消息、坐标和算法约定
│   └── acceptance.md         # 功能检查项、结果和验证范围
├── PLAN.md                   # 实施计划与后续任务
├── README.md                 # 构建、运行和使用说明
├── run.sh                    # check/build/launch/play 入口
└── .gitignore                # 素材和生成物的忽略规则
```

每个 ROS 包包含自己的 package.xml 和 CMakeLists.txt；算法包的 include/ 声明接口，src/ 实现算法和 ROS 节点。原始素材放入 task/，视频放入 video/，与 assets/、build/、devel/ 等本地资源和生成物一起由 Git 忽略。

## 环境与构建

环境：Ubuntu 20.04、ROS Noetic、OpenCV 4.x（开发环境为 4.2.0）及 catkin tools。需安装 compressed_image_transport、cv_bridge、tf2_ros、dynamic_reconfigure、rqt_image_view、rqt_reconfigure、RViz、Python OpenCV/YAML。包级依赖见各包的 package.xml。

在项目根目录执行：

```bash
./run.sh check
./run.sh build
```

脚本自动加载 Noetic 和本项目环境，构建结果保存在本项目。实际验证结果及限制见[验收记录](docs/acceptance.md)。

## 运行

将原始 bag 放入 task/，在同一 Noetic 环境的两个终端中，从项目根目录执行：

终端一，启动检测、rqt 和 RViz：

```bash
./run.sh launch dataset:=red_4 rviz:=true reconfigure:=true
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

两个终端的数据集须一致；切换时先停止旧启动和回放。`gui` 控制图像窗口，`rviz` 控制 RViz，`reconfigure` 控制调参窗口；三者均为 false 时可用于无桌面运行。调参窗口等动态服务就绪后打开，不需要开始回放。

默认同时启动跟踪；加 `tracker:=false` 可单独运行检测。RViz 的绿色球是当前估计，橙色球是 T 秒后的预测位置，纯预测时标记变淡。

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
roslaunch ft_bringup demo.launch dataset:=red_4 rviz:=true reconfigure:=true
# 终端二
rosbag play --clock task/red_4.bag --topics /hk_camera/image_raw/compressed /hk_camera/camera_info -r 0.2
```

</details>

## 参数与输出

公共话题、HSV、几何及 PnP 参数位于 [detector.yaml](src/ft_bringup/config/detector.yaml)，bag 文件名位于 [datasets/](src/ft_bringup/config/datasets)。公共参数先加载，数据集参数覆盖同名键。修改 YAML 后重启；其中 tuning 的 21 项也可在线调整。`debug_details: true` 可显示灯条下标和拒绝原因。

四份素材共用 `/hk_camera/image_raw/compressed` 和 `/hk_camera/camera_info`；回放仅提供相机输入和 `/clock`。

| 输出 | 内容 |
|---|---|
| `/ft/detection` | ft_detection/DetectionFrame，有效检测及带图像 header 的空检测帧 |
| `/ft/track` | ft_msgs/Track，轨迹 ID、命中标志、位置、速度和提前量位置 |
| `/ft/debug/marker` | 跟踪当前位置、未来位置和连线 |
| `/ft/debug/image` | 框、角点 0～3、帧内 ID、颜色、大小、位姿轴及深度/重投影误差 |
| `/ft/debug/mask/blue`、`/ft/debug/mask/red` | 蓝色、红色 HSV 掩膜 |
| `/tf` | 相机坐标系到 `armor_<帧内ID>` 的变换 |

另开终端查看一帧非空消息：

```bash
source /opt/ros/noetic/setup.bash
source devel/setup.bash
rostopic echo --filter 'len(m.detections) > 0' -n 1 /ft/detection
```

四点采用灯条端点，顺序为左上、右上、右下、左下。角点 x/y 单位像素、z 固定为 0；位姿和 TF 平移单位米，沿用图像时间戳。Detection.id 仅在帧内有效，confidence 为传统几何评分。完整接口、坐标和选解约定见[设计说明](docs/design.md)。

## 跟踪参数

[tracker.yaml](src/ft_bringup/config/tracker.yaml) 配置六维匀速卡尔曼滤波：状态为位置和速度，dt 来自图像时间戳。Q 采用随 dt 积分的白加速度噪声，q=2 m²/s³；R 为 0.0025 m²（位置标准差 5 cm），关联距离门限为 0.5 m。

首次选最近目标，之后在预测位置附近关联并保持锁定；漏检时 `tracking=false`，超过 0.5 s 删除。`id=0` 表示无轨迹，正 ID 表示已有轨迹；轨迹 ID 与帧内检测 ID 不同。`position_predict=position+T*velocity`，默认 T=0.1 s；模型、参数单位和已知限制见[设计说明](docs/design.md#单装甲板平移跟踪)。

## 在线调参

统一启动的 `reconfigure:=true` 会打开调参窗口。也可在另一个 Noetic 终端手动打开：

```bash
source /opt/ros/noetic/setup.bash
source devel/setup.bash
rosrun rqt_reconfigure rqt_reconfigure
```

选择 `/ft_detector/tuning`，可调整蓝色、红色低/高区间的 HSV 上下限、最小轮廓面积、配对最大归一化间距和角度差。滑块释放后生效；不合法组合会恢复上一次有效值。当前值及“Set to Default”来自启动时的 YAML，在线修改不会写回文件，需保留时手动更新 tuning 分组。其他参数（含跟踪 Q/R、关联门限和 T）修改 YAML 后重启。

## 尺寸约定

端点尺寸暂用小板 135×56 mm、大板 230×56 mm，参考 Alliance2026 开源参数，尚未与素材实物核对。YAML 保留 `confirmed: false` 和临时尺寸开关；`provisional` 表示采用临时尺寸模型。

## 开发简概

按 Git 提交顺序概述已有功能：

| 提交 | 开发内容 |
|---|---|
| `00018e4` | 建立图像输入与 CameraInfo 缓存，接通基础运行入口。 |
| `1783867` | 加入传统装甲板识别、灯条配对和调试绘图。 |
| `fd87594` | 加入 PnP 位姿解算、跨帧选解和 TF 可视化。 |
| `6401c18` | 统一相机输入配置，由 CameraInfo 自动选择 RViz 固定坐标系。 |
| `4db06c4` | 完成检测消息发布、空检测帧和可视化输出对照。 |
| `31abdf0` | 加入单板匀速跟踪、轨迹生命周期和提前量预测。 |
| `e0d5eba` | 加入动态调参、参数校验和 YAML 默认值管理。 |
