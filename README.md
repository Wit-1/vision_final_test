# 视觉组最终考核：装甲板自瞄感知链路

基于 ROS Noetic、OpenCV 和 ONNX Runtime 的独立 catkin 工作空间，实现图像接入、传统/神经网络装甲板识别、PnP、跨帧选解、单板平移跟踪及可视化。后续任务见 [PLAN.md](PLAN.md)。

## 仓库结构

当前源码与文档结构如下：

```text
vision_final_test/
├── src/
│   ├── ft_detection/         # 素材提供的检测消息包
│   ├── ft_msgs/              # 自定义 Track 消息
│   ├── ft_detector/          # 图像接入、两种识别、PnP、TF 和调参
│   ├── ft_tracker/           # 单板匀速跟踪与提前量预测
│   └── ft_bringup/
│       ├── config/           # 检测、跟踪和数据集参数
│       ├── launch/           # 统一启动及各模块入口
│       ├── scripts/          # 图像、RViz 和调参窗口启动器
│       └── rviz/             # 三维显示配置
├── tools/
│   ├── setup_inference.sh    # 准备本地 C++ 推理依赖
│   └── prepare_model.py     # 保留原模型、排序节点并准备资源
├── docs/
│   ├── design.md             # 数据流、消息、坐标和算法约定
│   └── acceptance.md         # 功能检查项、结果和验证范围
├── PLAN.md                   # 实施计划与后续任务
├── README.md                 # 构建、运行和使用说明
├── run.sh                    # 依赖/模型准备、构建和运行入口
└── .gitignore                # 素材和生成物的忽略规则
```

每个 ROS 包包含自己的 package.xml 和 CMakeLists.txt；算法包的 include/ 声明接口，src/ 实现算法和 ROS 节点。原始素材放入 task/，视频放入 video/，与 assets/、build/、devel/ 等本地资源和生成物一起由 Git 忽略。

## 环境与构建

环境：Ubuntu 20.04、ROS Noetic、OpenCV 4.2.0、catkin tools，以及 curl、Python pip/OpenCV/YAML。推理使用 ONNX Runtime 1.22.0 CPU C++ 包，神经网络库及节点使用 C++17，其余算法库使用 C++14。ROS 依赖见各包的 package.xml。

在项目根目录执行：

```bash
./run.sh setup-inference
./run.sh prepare-model
./run.sh check
./run.sh build
```

前两条仅需在首次准备或资源缺失时运行：依赖保存在 .deps/，模型及样例保存在 assets/。脚本自动加载 Noetic 和本项目环境，构建结果保存在本项目。实际验证结果及限制见[验收记录](docs/acceptance.md)。

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

默认 `mode:=tradition`。神经网络模式及同帧对比：

```bash
./run.sh launch dataset:=red_1_3 mode:=nn compare_modes:=true rviz:=true
./run.sh play red_1_3 -s 2 -r 0.15
```

两条命令分别在两个终端运行。rqt 选择 `/ft/debug/comparison`，左侧传统、右侧神经网络，均来自同一帧；`/ft/debug/image` 显示选中模式。关闭 `compare_modes` 后仅执行选中模式，模式切换需重启 launch。

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

话题、传统识别及 PnP 参数位于 [detector.yaml](src/ft_bringup/config/detector.yaml)，神经网络协议、阈值和对比设置位于 [model.yaml](src/ft_bringup/config/model.yaml)，bag 文件名位于 [datasets/](src/ft_bringup/config/datasets)。按 detector→model→数据集顺序加载，后加载的同名键覆盖前值。tuning 的 25 项可在线调整，其余 YAML 参数修改后重启。`debug_details: true` 可显示灯条下标和拒绝原因。

四份素材共用 `/hk_camera/image_raw/compressed` 和 `/hk_camera/camera_info`；回放仅提供相机输入和 `/clock`。

| 输出 | 内容 |
|---|---|
| `/ft/detection` | ft_detection/DetectionFrame，有效检测及带图像 header 的空检测帧 |
| `/ft/track` | ft_msgs/Track，轨迹 ID、命中标志、位置、速度和提前量位置 |
| `/ft/debug/marker` | 跟踪当前位置、未来位置和连线 |
| `/ft/debug/image` | 框、角点 0～3、帧内 ID、颜色、大小、位姿轴及深度/重投影误差 |
| `/ft/debug/mask/blue`、`/ft/debug/mask/red` | 蓝色、红色 HSV 掩膜 |
| `/ft/debug/comparison` | compare_modes 开启时的同帧双模式对比图 |
| `/tf` | 相机坐标系到 `armor_<帧内ID>` 的变换 |

另开终端查看一帧非空消息：

```bash
source /opt/ros/noetic/setup.bash
source devel/setup.bash
rostopic echo --filter 'len(m.detections) > 0' -n 1 /ft/detection
```

四点顺序为左上、右上、右下、左下；角点 x/y 单位像素、z 固定为 0，位姿和 TF 平移单位米。Detection.id 仅在帧内有效；confidence 在传统模式为几何评分，在神经网络模式为颜色分数×编号分数。神经网络编号另显示为 #1、#3 等，不替代帧内 ID。HSV 掩膜只在执行传统识别时发布。完整约定见[设计说明](docs/design.md)。

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

选择 `/ft_detector/tuning`：传统模式有 21 项 HSV/几何字段，神经网络有颜色分数、编号分数、联合分数和 NMS IoU 共 4 项。滑块释放后生效；不合法组合整组恢复上一次有效配置。当前值及“Set to Default”来自启动 YAML，在线修改不会写回文件。其他参数（含跟踪 Q/R、关联门限和 T）修改 YAML 后重启。

## 神经网络协议与计时

输入为 RGB float32 NCHW `[1,3,480,640]`：等比缩放、居中补边 124、除以 255。输出 `[1,21,6300]` 按颜色、编号和四点解码，不再 sigmoid；角点反变换后接入共用 PnP 和跟踪。

默认颜色/编号分数门限为 0.5/0.5，乘积门限 0.25；取最高分的至多 300 个合法候选，按四点外接框做跨类别 NMS，IoU 门限 0.5。大小分类使用公共几何规则。模型原件、派生模型及哈希清单位于 assets/models/，派生仅改变节点顺序。

`FT_TIMING` 日志给出传统检测、NN 预处理/推理/后处理和图像回调的累计平均毫秒数及样本数；正常退出打印 `FINAL FT_TIMING` 完整汇总。回调耗时包含选中识别、PnP、绘图和发布准备，开启对比时包含两条识别分支；不含压缩图传输解码及下游跟踪。后端和详细协议见[设计说明](docs/design.md#神经网络识别)。

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
