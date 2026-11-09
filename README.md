# 视觉组最终考核：装甲板自瞄感知链路

基于 ROS Noetic、OpenCV 和 ONNX Runtime 的独立 catkin 工作空间，实现图像接入、传统/神经网络装甲板识别、PnP、单板平移跟踪、车体旋转估计和装甲板选择。实施安排见 [PLAN.md](PLAN.md)，完整目录与文件用途见[文件介绍](docs/file_guide.md)。

## 仓库结构

当前源码与文档结构如下：

```text
vision_final_test/
├── src/
│   ├── ft_detection/    # 素材提供的检测消息
│   ├── ft_msgs/         # Track、Target 消息
│   ├── ft_detector/     # 图像接入、识别、PnP、TF 和调参
│   ├── ft_tracker/      # 单板匀速跟踪与提前量
│   ├── ft_selector/     # 旋转车体估计与未来选板
│   └── ft_bringup/      # 参数、启动和可视化配置
├── tools/              # 资源准备、性能统计和辅助对比
├── docs/               # 文件介绍、学习问答和结果配图
├── PLAN.md             # 实施计划
├── ai_usage.md         # AI 辅助范围与本人决策
├── README.md           # 构建与使用入口
└── run.sh              # 环境准备、构建、启动和回放
```

每个 ROS 包包含自己的 package.xml 和 CMakeLists.txt；算法包的 include/ 声明接口，src/ 实现算法和 ROS 节点。原始素材放入 task/，视频放入 video/，与 assets/、build/、devel/ 等本地资源和生成物一起由 Git 忽略。

## 环境与构建

环境为 Ubuntu 20.04、ROS Noetic、OpenCV 4.2.0、Eigen3、catkin tools、工具所需的 libyaml-cpp-dev，以及 curl、Python pip/OpenCV/YAML。推理使用 ONNX Runtime 1.22.0 CPU C++ 包。神经网络与旋转选板使用 C++17，其余算法库使用 C++14。ROS 依赖见各包的 package.xml。

在项目根目录执行：

```bash
./run.sh setup-inference
./run.sh prepare-model
./run.sh check
./run.sh build
```

前两条仅需在首次准备或资源缺失时运行：依赖保存在 .deps/，模型及样例保存在 assets/。脚本自动加载 Noetic 和本项目环境，构建结果保存在本项目。各模块的接口与算法约定按下面的题目编号说明。

## 运行

将原始 bag 放入 task/，在同一 Noetic 环境的两个终端中，从项目根目录执行：

终端一启动检测、跟踪、选板和可视化。

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

默认启动检测、跟踪和选板。用 `tracker:=false` 或 `selector:=false` 分别关闭对应节点，同时设置两者为 false 可单独运行检测。RViz 的绿色球是单板当前估计，橙色球是 T 秒后的线性预测，纯预测时标记变淡。

默认 `mode:=tradition`，用 `mode:=nn` 选择神经网络。独立对比工具同时运行两种检测。

```bash
./run.sh compare dataset:=red_1_3 mode:=nn
./run.sh play red_1_3 -s 2 -r 0.15
```

两条命令分别在两个终端运行。rqt 选择 `/ft/debug/comparison`，左侧传统、右侧 NN，按图像时间戳和坐标系对齐。mode 指定正式链路使用的模式，辅助检测通过独立话题供拼图使用。RViz 灰线由预测对照工具发布。正常运行使用 launch，模式切换需重启。

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

## 第一点：图像接入与相机参数

使用 image_transport 的 compressed 插件和 cv_bridge 接入图像，缓存最新有效 CameraInfo，从中读取 K、D、图像尺寸和实际 frame_id。相机内参不写死，业务消息沿用输入图像时间戳与坐标系。

四份素材共用 `/hk_camera/image_raw/compressed` 和 `/hk_camera/camera_info`；回放仅提供相机输入和 `/clock`。输入话题与小队列长度在 [detector.yaml](src/ft_bringup/config/detector.yaml) 中配置，bag 文件名在 [datasets/](src/ft_bringup/config/datasets) 中配置。

## 第二点：传统装甲板识别

传统模式依次进行 HSV 颜色分割、轮廓筛选和灯条配对。蓝色使用单区间，红色合并两段区间；根据灯条长度、角差、高差和间距筛选候选，拒绝共享灯条的明显劣配与歧义配对，再按归一化间距区分大小板。

一帧中的多个候选均保留，按位置排序分配帧内 ID。识别框、ID 和大小显示在 `/ft/debug/image`；`debug_details: true` 可显示灯条下标和拒绝原因。传统模式另发布 `/ft/debug/mask/blue`、`/ft/debug/mask/red` 两路 HSV 掩膜。

## 第三点：位姿解算与 TF

采用定义一：一对灯条的四个端点，顺序为左上、右上、右下、左下。装甲板原点在四点中心，X 沿板向右、Y 沿板向下、Z=X×Y 指向板背后；相机光学系为 x 右、y 下、z 前，均为右手系。

PnP 使用 IPPE 求平面候选，检查正深度和重投影误差，并在有效历史存在时辅助消歧。变换为 P_camera=R×P_armor+t，TF 以相机为父系、armor_<帧内ID> 为子系，直接使用 R 和 t。重投影坐标轴为 X 红、Y 绿、Z 蓝。

端点尺寸暂用小板 135×56 mm、大板 230×56 mm，参考 Alliance2026 开源参数，尚未与素材实物核对。YAML 保留 `confirmed: false` 和临时尺寸开关；`provisional` 表示采用临时尺寸模型。重投影检查通过仍不能证明实物米制精度，首次朝向仍可能存在平面双解歧义。

下图为原始 red_4 回放时的 RViz TF 界面，左侧显示 armor_0 的父坐标系及位置，右侧显示对应的三条坐标轴。截图仅在界面中调整了视角和显示项。

![RViz 中装甲板的 TF 位姿及父坐标系](docs/images/tf_pose_rviz.png)

## 第四点：消息发布与可视化

| 输出 | 内容 |
|---|---|
| `/ft/detection` | ft_detection/DetectionFrame，有效检测及带图像 header 的空检测帧 |
| `/ft/debug/image` | 框、角点 0～3、帧内 ID、颜色、大小、位姿轴及深度/重投影误差 |
| `/tf` | 相机坐标系到 armor_<帧内ID> 的变换 |

四点顺序为左上、右上、右下、左下；角点 x/y 单位像素、z 固定为 0，位姿和 TF 平移单位米。Detection.id 仅在帧内有效，位姿过滤后可不连续。confidence 在传统模式为几何评分，在神经网络模式为颜色分数×编号分数；模型编号另显示为 #1、#3 等，不替代帧内 ID。

每张成功处理的图像均发布检测帧，无有效位姿时 detections 为空。另开终端查看一帧非空消息：

```bash
source /opt/ros/noetic/setup.bash
source devel/setup.bash
rostopic echo --filter 'len(m.detections) > 0' -n 1 /ft/detection
```

用 rqt_image_view 选择 `/ft/debug/image`，可以看到识别框、角点编号、帧内 ID、大小、位姿轴和重投影信息。下图为原始 red_4 回放中的实际窗口。

![rqt_image_view 查看装甲板 debug 图](docs/images/debug_rqt_image_view.png)

## 第五点：参数化与动态调参

话题、传统识别及 PnP 参数位于 [detector.yaml](src/ft_bringup/config/detector.yaml)，神经网络协议、阈值和线程设置位于 [model.yaml](src/ft_bringup/config/model.yaml)。按 detector→model→数据集顺序加载，后加载的同名键覆盖前值。

统一启动的 `reconfigure:=true` 会打开调参窗口。也可在另一个 Noetic 终端手动打开：

```bash
source /opt/ros/noetic/setup.bash
source devel/setup.bash
rosrun rqt_reconfigure rqt_reconfigure
```

选择 `/ft_detector/tuning`：传统模式有 21 项 HSV/几何字段，神经网络有颜色分数、编号分数、联合分数和 NMS IoU 共 4 项。滑块释放后生效；不合法组合整组恢复上一次有效配置。当前值及“Set to Default”来自启动 YAML，在线修改不会写回文件。其他参数（含跟踪 Q/R、关联门限和 T）修改 YAML 后重启。

调参对照：在 blue_1 的同一帧中，将 blue_v_min 从 130 提高到 240，蓝色掩膜白像素从 2795 减少到 1686，检测数从 2 变为 0；恢复为 130 后结果恢复。提高亮度下限会收紧筛选，也可能漏掉原来能识别的目标。

参数窗口选中 `/ft_detector/tuning` 后，可看到在线字段及其滑块。下图显示启动 YAML 加载后的当前值，包含 21 项传统字段和 4 项 NN 字段。

![rqt_reconfigure 中的检测参数滑块](docs/images/tuning_rqt_reconfigure.png)

## 第六点：单目标平移跟踪与提前量

[tracker.yaml](src/ft_bringup/config/tracker.yaml) 配置六维匀速卡尔曼滤波：状态为位置和速度，dt 来自图像时间戳。Q 采用随 dt 积分的白加速度噪声，q=2 m²/s³；R 为 0.0025 m²（位置标准差 5 cm），关联距离门限为 0.5 m。

这些是试验初值：q 用于允许短时加减速，R 以 5 cm 的位置噪声作初始假设，0.5 m 门限用于容纳相邻帧预测误差，同时按颜色与大小限制关联。它们尚未由实测噪声或标注数据标定；误关联时应结合目标速度与检测抖动收紧门限。

首次选最近目标，之后在预测位置附近关联并保持锁定；漏检时 `tracking=false`，超过 0.5 s 删除。`id=0` 表示无轨迹，正 ID 表示已有轨迹；轨迹 ID 与帧内检测 ID 不同。`position_predict=position+T*velocity`，默认 T=0.1 s。

`/ft/track` 发布 ft_msgs/Track，包含轨迹 ID、命中标志、位置、速度和提前量位置。`/ft/debug/marker` 显示绿色当前位置、橙色未来位置及连线，纯预测时标记变淡。位置单位为 m，速度为 m/s，T 为 s。相机运动、同类目标交叉和旋转换板可能影响关联。

下图为 red_4 运动片段的 RViz Marker 界面，订阅 `/ft/debug/marker`。绿色球为当前估计位置，橙色球为 T=0.1 s 的预测位置，橙线连接两者；按当前临时尺寸模型，这一帧两点间距约 7.8 cm。

![RViz 中的当前位置、提前量预测位置及连线](docs/images/prediction_marker_rviz.png)

## 第七点：基础链路跑通

统一入口为 demo.launch，上文的 run.sh launch 会加载检测、跟踪和可视化配置。图像、RViz 和动态调参窗口分别由 gui、rviz、reconfigure 控制；RViz 从有效 CameraInfo 自动选择固定坐标系。

回放后用 rqt 查看 `/ft/debug/image`，在 RViz 查看装甲板 TF 与提前量 marker，并通过动态调参观察识别效果变化。

## 第八点：神经网络推理

使用 ONNX Runtime 1.22.0 CPU C++ 后端，`mode:=nn` 选择神经网络，`mode:=tradition` 选择传统识别，切换需重启。

输入 images 为 RGB float32 NCHW `[1,3,480,640]`：等比缩放、居中补边 124、除以 255。输出 output0 为 `[1,21,6300]`，按 channel×6300+candidate 读取，不再 sigmoid。

| 输出通道 | 含义 |
|---|---|
| 0～3 | 蓝、红、熄灭、其他分数 |
| 4～12 | 九项编号分数：哨兵、1、2、3、4、5、前哨站、基地、基地 |
| 13～20 | 四个角点，模型点序为左上、左下、右下、右上 |

颜色和编号分别取最高分类，只有胜出颜色为红或蓝才继续筛选。四点按 0/3/2/1 重排为公共点序，减去补边后除以实际横纵缩放比例还原到原图，再接入共用 PnP 和跟踪。

默认颜色/编号分数门限为 0.5/0.5，乘积门限 0.25；取最高分的至多 300 个合法候选，按四点外接框做跨类别 NMS，IoU 门限 0.5。大小分类使用公共几何规则。模型原件、派生模型及哈希清单位于 assets/models/，派生仅改变节点顺序。

上文的 run.sh compare 同时启动两种检测，发布 `/ft/debug/comparison` 同帧拼图，左传统、右 NN；正式链路只使用 mode 选中的结果。耗时统计见[性能工具说明](tools/README.md)。

素材包两张样例图在默认阈值下的结果如下。这里统计通过 PnP 的有效检测数，仅说明这两张图的输出，不作为全段识别准确率。

| 样例 | 传统方法 | 神经网络 |
|---|---:|---:|
| sample_1.jpg | 1 | 2 |
| sample_2.jpg | 1 | 3 |

下图来自原始 red_1_3 第 2 秒起的 0.6 s 回放片段，是 compare 工具实际发布的一帧：左侧传统方法得到 1 个有效检测，右侧 NN 得到 2 个，均使用同一输入图像。画面保留四点、帧内 ID、位姿轴和临时尺寸标记。平均处理耗时与测量范围见[第十点](#第十点全链路集成与性能)。

![同帧检测对比：左传统、右神经网络](docs/images/detection_comparison.png)

## 第九点：旋转目标建模与装甲板选择

用两个终端启动选板并回放：

```bash
./run.sh launch dataset:=blue_guard mode:=nn rviz:=true
./run.sh play blue_guard -u 3 -r 0.15
```

旋转 EKF 使用八维状态 [cx,cy,cz,vx,vy,vz,θ,ω]，分别表示车体中心、平移速度、相位和角速度；用图像时间差预测，角度及角残差环绕到 ±π，通过换板关联保持车体 ID。半径、槽位数和门限在 [selector.yaml](src/ft_bringup/config/selector.yaml) 中设置，提前量沿用 [tracker.yaml](src/ft_bringup/config/tracker.yaml)。默认半径 0.25 m 为试验值；模型假设相机稳定、装甲板等高等半径且等角分布，几何朝向检查不包含实际遮挡和视野边界。

首次选最近的有效板；已有轨迹时，将各模型槽位与同色观测配对，位置差门限为 0.40 m、角差门限为 0.70 rad。通过门限后，以归一化位置误差和角误差的平方和最小的一组校正，每帧只用一组观测，换槽位不更换车体 ID。

未来选板先按 c+vT 和 θ+ωT 推进圆心与相位，再遍历各槽位。预测坐标须有限、深度为正，板的朝外法向与指向相机方向的点积须大于 0.20，取其中最高分槽位。本帧有匹配观测、当前板深度为正且未来选板成功时 valid=true。

`/ft/target` 发布 ft_msgs/Target，包含车体 ID、有效标志、当前板和未来选中板的位置。未来板由模型计算，可属于当前不可见的另一槽位。短时漏检时 `valid=false`，保留车体 ID 并以半透明 marker 显示预测；失联超过 0.5 s 时清除车体 ID 和 marker。

`/ft/debug/selector` 的青色方块表示车体中心，紫色球是当前板，蓝色球是 T 秒后最朝向相机的板。橙色球仍为单板匀速预测。运行 compare 工具时，`/ft/debug/prediction_comparison` 在同时间戳、同坐标系且两路有有效观测时，用灰线连接两个预测点。

蓝紫用于看提前选板，蓝橙用于比较两种预测。判断偏差时，将同一帧发出的预测与 T 秒后的可见板对应。相机运动、突然变速或模型几何不符都可能使旋转预测失效。

在 red_1_3 的一个换板样例中，NN 模式使用 T=0.1 s、半径 0.25 m，将约 3.68 s 发出的两路预测投到约 3.77 s 的图像。旋转预测已转向新面，距新可见板四点中心约 55 px；线性预测仍向旧板一侧外推，相距约 327 px。下图为局部放大，距离按原始 1440×1080 图像计算，是单个样例的二维对照。

![旋转换板预测对照：当前图像与后续图像](docs/images/rotation_prediction.png)

固定半径、姿态噪声、突然变速，以及未建模的遮挡和视野边界都是模型的限制。

## 第十点：全链路集成与性能

默认统一启动检测、平移跟踪和旋转选板，支持 tradition / nn 模式。tracker、selector 分别控制两个下游节点，两者均直接订阅检测结果。

独立基准复用现有算法库，统计各阶段耗时的平均值与 P95。构建、运行、CSV 字段和计时范围见 [tools/README.md](tools/README.md)。

实测环境为 Intel Core i7-14650HX、Ubuntu 20.04 Noetic 容器、OpenCV 4.2.0 和 ONNX Runtime 1.22.0 CPU。OpenCV 与 ORT intra-op 各 2 线程，ORT inter-op 为 1；原始 red_4 从第 20 秒起处理 1 s，每种模式 100 帧，首帧计入。

| 模式 | 检测平均 ms | 检测 P95 ms | 串行总平均 ms | 串行总 P95 ms |
|---|---:|---:|---:|---:|
| tradition | 4.713 | 4.992 | 9.054 | 9.447 |
| nn | 40.005 | 41.314 | 47.050 | 48.438 |

串行总用时包含 JPEG 解码、检测、PnP/观测转换、单板跟踪、旋转选板和调试绘图；模型加载、磁盘读取、CameraInfo 缓存、CSV 写入及 ROS 通信/排队在计时之外。P95 用最近秩法直接统计逐帧总用时，结果表示离线算法开销，不能直接当作在线链路帧率。

传统模式的主要开销为检测 52.0% 和 JPEG 解码 43.0%；NN 模式为检测 85.0% 和解码 11.4%，两种跟踪算法更新占比较小。NN 检测包含预处理、推理、解码与去重，85.0% 不能全部归于推理引擎。后续优化应先细分检测阶段，确认主要耗时后再选择方案。

保持完整链路和 bag 回放运行，在第三个 Noetic 终端打开话题图：

```bash
rosrun rqt_graph rqt_graph
```

选择 Nodes/Topics (all)，可以看到 /ft/detection 分别连接跟踪与选板，两者发布 /ft/track 和 /ft/target。对比工具运行时会增加辅助检测和对比节点。

下图为正常 NN 链路运行时导出的实际 rqt_graph，筛选了相机输入与核心业务话题。它展示发布/订阅关系，耗时由上面的基准测量。

![实际 ROS 话题连接图](docs/images/ros_graph.png)

## 待确认问题

- **素材车辆的灯条尺寸：** 小板 135×56 mm、大板 230×56 mm 目前参考 Alliance2026 参数，尚未核对素材实物的灯条中心距和发光长度，真实米制精度仍待验证。
- **旋转模型的车辆几何：** 半径 0.25 m 为未实测的试验值，四槽位、等高、等半径和等角分布是否符合素材车辆，仍需核实。
