# 感知链路接口与坐标约定

本文件记录实现采用的约定；实际验证范围及尚未确认的内容见[验收记录](acceptance.md)。

## 数据流与模块边界

```text
原始 compressed 图像 + CameraInfo
  → detector_node：压缩解码、标定缓存和参数
  → TraditionalDetector 或 NnDetector：共用 ArmorCandidate
  → PoseHistory.references：上一帧中唯一匹配的位姿参考
  → PnpSolver：几何校验、平面候选、带条件的跨帧消歧与重投影
  → PoseHistory.store：整帧求解结束后保存本帧合格目标
  → buildDetectionOutput：单次姿态转换 → /ft/detection 与匹配的 /tf
  → drawDetections / drawPoses：独立副本上的框、四点编号和坐标轴
  → /ft/debug/image、两个 mask
```

检测、PnP、历史和绘图位于无 ROS 依赖的 ft_detector_core。ft_detector_output 是独立 ROS 适配库，负责生成 DetectionFrame 与 TransformStamped，节点负责发布。每个二维候选对应一个 PnpResult；未求解时保持 valid=false，不以默认零位姿填充检测消息。

## 检测消息协议

沿用素材 ft_detection 包，不修改消息定义或 MD5。DetectionFrame 只有一份 header 和一个 detections 数组，Detection 元素本身没有 header。

| 字段 | 当前含义 |
|---|---|
| header.stamp / frame_id | 输入图像时间和坐标系；有效位姿需图像与 CameraInfo 匹配 |
| detections[].id | 当前帧唯一候选编号，过滤后可不连续，不代表跨帧身份 |
| is_red / is_large_armor | 当前候选的颜色与大小分类 |
| pose.position | 该装甲板原点在相机系的位置，直接使用 tvec，单位米 |
| pose.orientation | rvec→旋转矩阵→归一化四元数，直接表示装甲板到相机的旋转 |
| corners[4] | 原图像像素的 TL/TR/BR/BL；Point32 的 z 固定为 0 |
| confidence | 传统为 1/(1+cost)，NN 为颜色分数×编号分数；均非标定概率 |

buildDetectionOutput 检查数组对应、有效标志、候选/位姿 ID、重复 ID、有限值和评分范围；不合格候选不进入消息及其 TF。消息和 TF 共用同一个 geometry_msgs/Pose，不分别计算四元数。Point32 的二维像素不能按位置米制字段解释。三维尺寸仍是暂定来源，消息没有另加尺寸确认字段。

每张成功解码并处理的图像发布一份 DetectionFrame，无目标或所有位姿失败时 detections 为空。缺少匹配标定、PnP 关闭也发布空帧，供后续跟踪按图像时间推进丢失状态；空帧不重复发送旧位姿或旧 TF。积压输入或损坏图像被跳过时不伪造处理结果。可用位姿的 TF 发布仍由 pnp.publish_tf 独立控制。

## 像素点、物体点和单位

采用灯条长轴端点，图像点始终为：

| 索引 | 图像点 | 物体坐标（米） |
|---|---|---|
| 0 | 左灯上端 | (-W/2, -L/2, 0) |
| 1 | 右灯上端 | (+W/2, -L/2, 0) |
| 2 | 右灯下端 | (+W/2, +L/2, 0) |
| 3 | 左灯下端 | (-W/2, +L/2, 0) |

W 是灯条中心距，L 是对应发光段的长度。坐标原点为两灯中心连线中点；X 沿板向右，Y 沿板向下，Z=X×Y，在正面观察时指向板背后，构成右手系。相机采用光学坐标：x 右、y 下、z 前。旋转后物体轴无需再与相机轴平行。

图像点单位为像素；尺寸、平移、TF 和坐标轴长度统一为米；内部旋转向量为弧度。当前采用灯条端点模型，YAML 中小板 W/L 为 0.135/0.056 m、大板为 0.230/0.056 m，尺寸参考 Alliance2026 开源参数，尚未与本素材实物核对。

YAML 中 small、large 分别保存 width_m、height_m、confirmed。confirmed 表示实际端点尺寸是否核实；小板和大板分别确认，不能用一个类别的确认替代另一个。

allow_provisional_dimensions 默认在算法参数中为 false；当该开关关闭，未确认类别返回 dimensions_unconfirmed。当前 YAML 显式开启临时开关，允许未确认但为正数的尺寸进入 PnP；零尺寸仍拒绝。PnpResult.provisional 标明本次使用了临时尺寸；valid 只代表软件几何检查合格，不代表物理尺度已核实。调试图的状态与每个合格位姿标签都会标注 provisional，TF 则使用同一个临时模型结果。

## 相机模型

从 CameraInfo 获取原始图像的 K、D、尺寸及 frame_id，不使用 rectified 图像对应的投影矩阵 P。图像必须与缓存的标定尺寸和 frame_id 一致。

当前支持 plumb_bob（D 为空或 4/5 个系数）、rational_polynomial（8 个系数）；未知模型、非有限数据、无效焦距或 K 结构均拒绝。非平凡 ROI 和 binning>1 的坐标换算尚未实现，因此明确拒绝这类标定。无效 CameraInfo 清空缓存，不能继续用先前标定做三维解算；二维识别仍运行。

四份原始素材当前均为 1440×1080、plumb_bob、5 个 D 系数，无 ROI 或 binning 换算。它们共用 `/hk_camera/image_raw/compressed` 和 `/hk_camera/camera_info`，公共入口定义在 detector.yaml，各数据集 YAML 仅保存 bag 文件名。detector.launch 和 run.sh play 均先读公共参数再应用数据集覆盖。

话题名称与消息 header.frame_id 独立。CameraInfo 将 frame_id 定义为相机光学坐标系，节点始终保留实际消息中的名称，不据话题名推断坐标轴或重命名。

## 位姿求解与筛选

PnpSolver 使用 solvePnPGeneric + SOLVEPNP_IPPE 取得平面候选解，逐解检查：

1. 输入四点有限、位于图内、凸、面积与边长足够，且符合约定的左右及上下顺序；不替错误输入自动重排。
2. rvec/tvec 有限，原点及全部物体角点的相机深度均在配置范围内；板法向与中心位置的点积必须为正，以保持约定的正面朝向。
3. 用 projectPoints 和原 K/D 重投影四点，计算每点二维欧氏误差的 RMS，记录通过物理检查的全部候选及最小误差 e_min；最小误差也超过 3 px 时拒绝。
4. 若有有效的上一帧参考，只在 e≤3 px 且 e≤e_min+error_margin_px 的候选内，选择与参考旋转距离最小者。没有参考时选择最小误差。相对旋转角为 acos(clamp((trace(R_ref^T R)-1)/2))，不直接相减旋转向量或四元数分量。
5. 用选中候选自己的完整 rvec/tvec 投影原点及三个正轴端点，检查正深度、有限性及绘制数值范围。图像和 TF 都使用这个候选，没有平均姿态、固定坐标轴或用历史位姿代替当前观测。

IPPE_SQUARE 有特定正方形点序要求，本项目不使用它。PnpSolver 本身不存时间状态；solve 的第三个参数是可选的 ArmorPose 指针，由节点取得外部历史参考。近正视平面和远距离小目标存在姿态歧义，微小像素误差可能引起分支切换。当前已实现带条件的连续性选解，但它不能确定首次初始化的分支是真实朝向，也不能承诺所有素材连续无跳变。

PoseHistory 只保存上一帧解算成功的候选和位姿，不生成跨帧 ID 或运动预测。references 对当前与历史候选检查颜色相同、大小相同、对应四端点的位移 RMS 足够小；位移除以两帧中较短的平均灯条长度，默认门限 0.5。双方在门控内都只有一个匹配时才提供参考，一对多、多对一均不继承。该短时几何门控不等同于完整跟踪器，交叉、遮挡和外观相同目标仍有身份关联局限。

节点先取得整帧参考并逐候选 solve，之后再 store，避免更新容器使参考指针失效。store 只保留本帧有效位姿，丢失或解算失败的目标不保留旧参考；无唯一匹配时当前帧独立初始化。图像时间重复、倒退或间隔超过 max_gap_s=0.1 会清空参考；标定 K/D、模型、尺寸或父坐标系改变、标定无效、图像与标定不匹配，以及图像处理异常也清空参考。只有 CameraInfo 时间戳更新而实际标定不变时不清空。

重投影误差小只能检查像素对应的一致性，不能证明尺寸尺度正确。把 W、L 同时扩大一倍，平移也会扩大一倍而投影保持相同。

若只改 W 或 L，或将本体宽高暂配给灯条端点导致宽高比不符，影响还可能包括倾角与双解选择，不能视为统一的距离缩放。当前使用暂定端点尺寸；跨帧选解的验证范围见[验收记录](acceptance.md)，真实角度和米制尺度仍待核实。

## 变换与 TF

求解得到：

```text
P_camera = R_camera_from_armor × P_armor + t_camera_from_armor
```

因此 TF 的 parent 为该帧匹配的 CameraInfo.header.frame_id，child 为配置前缀 armor_ 加本帧候选 ID。直接使用 R 和 t，不对 R 转置或把 t 取负。节点用 Rodrigues 得到 R，再由 tf2::Matrix3x3 转换并归一化四元数，发布动态 TF。

TF 和三张图像的 stamp、frame_id 来自输入图像及其匹配标定，不使用系统当前时间代替。ROS 的 header.seq 由各发布者维护，不作为跨话题对齐键。时间回退时清空姿态参考；TF 查询端也可能清理自身时间缓存。

ID 是逐帧候选编号，armor_0 下一帧可能代表另一块板。失败或无目标时不发送新的该帧 TF；旧 TF 不会由发布端主动删除。RViz 配置把 TF Frame Timeout 设为当前 RViz 支持的 1 秒，查询时仍须使用目标图像时间，不把最后一次旧 TF 当作当前检测。

## 调试和参数

X/Y/Z 投影轴分别画红/绿/蓝，使用与位姿相同的 K/D 和轴长；标注 Z（米）、重投影误差（像素）及 PnP 状态。绘制在已 clone 的标注图上，输入像素和 mask 保持独立。

PnpResult.continuity_selected 表示参考历史后选择了 RMS 略大的当前帧合格分支，图中追加 (prior)。它不表示每个有历史参考的帧，也不表示实际朝向已核实。provisional 标记继续独立表示尺寸尚未实测确认。

参数 pnp/continuity 包含 enabled、error_margin_px=1.0、max_gap_s=0.1、max_corner_shift_ratio=0.5。error_margin_px 是相对当帧最优误差允许增加的量，不能替代原来的绝对 3 px 门限。PnpParams 的算法默认 margin 为 0，节点明确从 YAML 加载。关闭 enabled 后不传入历史参考，恢复单帧最小误差选择；修改参数后重启。

pnp.enabled 关闭全部三维求解；pnp.publish_tf 仅关闭 TF 广播，合格位姿仍可投影显示。全部参数由 YAML 加载；tuning 中的 21 项可通过 dynamic_reconfigure 在线调整，其余参数修改后重启，加载及校验规则见下文。

RViz 通过 demo.launch 的 rviz 参数按需启动。view_rviz.py 从检测节点参数读取同一个 CameraInfo 话题，收到首个非空 frame_id 后以该固定帧启动 RViz；帧名改变时先停止原查看器再启动新查看器，相同帧名不会重启。关闭 RViz 后启动器退出，关闭 launch 时清理子进程。监控使用墙上时间，暂停 bag 的模拟时间不会阻塞清理。

脚本不包含任何数据集名称分支，也不改写 CameraInfo、图像或 TF。固定帧不再通过额外的 launch 参数或 RViz 配置中的常量指定；真实相机坐标系由消息决定。

统一 demo.launch 另有 reconfigure 参数，默认 false，控制 view_reconfigure.py。该脚本等待配置的动态服务后打开 rqt_reconfigure，并传入服务器命名空间供自动选择；等待服务使用墙上时间，初始没有 /clock 时仍可打开调参窗口。进入 GUI 前关闭辅助 rospy 节点，再用 exec 替换进程，roslaunch 继续管理该 GUI 的进程生命周期。gui、rviz、reconfigure 分别控制图像、三维和参数窗口；检测和默认启用的跟踪不依赖这些窗口。

## 接口依据

- [OpenCV 4.2 标定、PnP 与投影接口](https://docs.opencv.org/4.2.0/d9/d0c/group__calib3d.html)。
- [ROS CameraInfo 定义](https://github.com/ros/common_msgs/blob/noetic-devel/sensor_msgs/msg/CameraInfo.msg)。
- [ROS REP-103 单位与坐标约定](https://github.com/ros-infrastructure/rep/blob/master/rep-0103.rst)。

## 单装甲板平移跟踪

`ft_tracker_core` 不依赖 ROS，使用 OpenCV `cv::KalmanFilter` 的双精度六维匀速模型。ROS 节点只负责读取 YAML、转换观测、处理坐标系变化及发布 Track/Marker。输入是自写检测链路的 `/ft/detection`，不使用参考 bag 的检测或历史目标 TF。

状态为 `[x,y,z,vx,vy,vz]`，位置单位米、速度单位 m/s，观测为装甲板中心 `[x,y,z]`。每一帧先预测位置 `p_pre=p_post+v*dt`，再用关联到的当前观测校正；`dt` 只取相邻输入图像时间差。H 取状态的前三维，F 的位置/速度交叉块为 `dt*I`。

Q 来自连续白加速度模型，三个轴互相独立；每个轴的位置/速度块为 `q*[[dt^3/3,dt^2/2],[dt^2/2,dt]]`，因此 Q 会随真实 dt 改变。`process_noise_density` 是 q，单位 m²/s³；R 为 `measurement_noise_variance*I`，单位 m²。初始位置与速度方差分别由 YAML 配置，首帧速度为零，不凭一帧推断速度。

当前试验初值 q=2、R=0.0025（位置标准差 5 cm）、初始位置方差 0.01、速度方差 4。它们是可调起点，未用距离真值标定；增大 q 允许速度更快变化，增大 R 降低对位置观测的信任。关联门限 0.5 m 取题面建议 0.3～0.5 m 的较宽端，避免初始速度未收敛时立即丢锁；它不是实测最优值，后续需结合像素/PnP 抖动和目标速度调整。

首次从有效观测中选择距离相机原点最近者，等距时用帧内检测 ID 决定顺序。之后只在预测位置 0.5 m 内选择最近且颜色/大小兼容的观测；颜色和大小门控可分别关闭。检测 ID 不参与跨帧身份匹配。锁定后不因另一个目标更靠近相机而重选，但同色同尺寸目标交叉仍可能错误关联，不能保证车体身份。

没有关联观测时，只推进预测状态和协方差，保留轨迹 ID，`tracking=false`；有观测命中（含首次建立）才为 true。丢失时长从最后一次命中算起，严格超过 `lost_timeout_s=0.5` 才清除。清除后同一输入帧如有有效观测，可立即以新 ID 建轨。长输入间隔同样触发超时；暂停回放而没有输入不会自行更新。时间重复、倒退或 frame_id 改变重置状态，进程内 ID 计数保持递增。

`ft_msgs/Track` 沿用输入 header，包含 id、tracking、position、velocity、position_predict。`id=0` 表示当前无轨迹，其数值字段为零占位；`id>0 && tracking=false` 表示已有轨迹的纯预测，不能把无轨迹的零位置当测量。非有限值及非正观测深度不会建轨或参与校正。`position_predict=position+T*velocity`，默认 T=0.1 s；它不推进下一帧的滤波状态，不包含弹道、旋转或相机运动补偿。

`/ft/debug/marker` 使用三个固定 namespace/id：绿色球（0）为当前估计，橙色球（1）为未来位置，橙线（2）连接两者；纯预测时降低透明度。轨迹删除时发送三个 DELETE，避免把旧 marker 当当前轨迹。marker 时间与坐标系和 Track 一致，生命周期由 YAML 配置；空 frame_id 时用上一个有效坐标系删除旧标记。RViz 的 Tracking 显示订阅该话题。

跟踪处理的是相机坐标系中的相对平移。车体旋转换面、相机运动、大小误分类和暂定 PnP 尺寸都可能影响它；第 9 点再处理车体旋转与选板。滤波接口依据：[OpenCV 4.2 KalmanFilter](https://docs.opencv.org/4.2.0/dd/d6a/classcv_1_1KalmanFilter.html)。

## 参数加载与在线调参

| YAML 位置 / 参数命名空间 | 内容 | 生效方式 |
|---|---|---|
| detector.yaml 根 /ft_detector | 相机和输出话题、队列、transport、debug_details | 启动读取 |
| detector.yaml tuning /ft_detector/tuning | 蓝色与两段红色共 18 项 HSV 边界、最小面积、最大配对间距/角度差 | 启动读取及 dynamic_reconfigure |
| model.yaml tuning /ft_detector/tuning | NN 颜色/编号/联合分数和 NMS IoU 门限 | 启动读取及 dynamic_reconfigure |
| model.yaml nn /ft_detector/nn | 模型路径、输入尺寸、补边值、线程数和候选数量上限 | 启动读取 |
| detector.yaml traditional /ft_detector/traditional | 灯条长度、长宽比、填充率、倾角、配对长度/高度差、最小间距、大小分界及歧义规则 | 启动读取 |
| detector.yaml pnp /ft_detector/pnp | 大小板端点尺寸及确认标记、深度/重投影门限、历史消歧与 TF 参数 | 启动读取 |
| tracker.yaml /ft_tracker | Q 的 q、R、初始协方差、关联/属性/丢失门限、T、话题/队列及 marker | 启动读取 |
| datasets/*.yaml | bag 文件名；公共参数之后加载，同名键由数据集覆盖 | 启动/回放读取 |

CameraInfo 继续提供 K、D、图像尺寸及实际 frame_id，不将内参写入 YAML。launch 通过 `$(find ft_bringup)` 找配置；run.sh 使用自身项目根定位。tuning 分组保存 18 项 HSV 边界、最小轮廓面积、最大配对间距和角度差；其余几何参数保存在 traditional 分组。

Detector.cfg 声明 21 项传统字段及 4 项 NN 字段，生成 DetectorConfig.h/Python 配置。H 范围 0～179，S/V 为 0～255；面积 0.1～10000 px²，最大归一化间距 0.1～20，最大角度差 0.1～90°；NN 分数及 IoU 为 0～1。构建默认值分别与 detector.yaml、model.yaml 保持一致。启动时先验证 YAML，再写入动态配置；超出 UI 范围的启动值报错退出。

节点在 `/ft_detector/tuning` 建立动态服务，先 updateConfig 写入实际 YAML 初值，setConfigDefault 更新本实例的默认描述，再注册回调。这样数据集覆盖后的初值与 rqt 的当前值、恢复默认值一致，不被构建默认值或上次运行的参数服务器残留覆盖。服务器使用外部递归互斥锁。

tuning_parameters.hpp/cpp 在算法参数与生成配置之间转换。paramsWithTuning 先复制现有 TraditionalParams，再改 21 项，因此未暴露的几何参数保留。ft_detector_tuning 是依赖生成配置的适配库，原 ft_detector_core 仍不依赖 ROS。CMake 显式依赖 gencfg，生成物保留在构建目录。

当前节点用单线程 ros::spin，图像和配置回调依次执行。在线回调先验证传统与 NN 的完整候选配置，再替换各自算法参数；HSV 下限大于上限、配对关系非法或 NaN 等会使 25 项整组恢复上一次有效配置。单值越界由动态服务先限制到 cfg 范围，组合合法性仍由算法校验。有效变化清空旧 PnP 参考；NN 阈值更新不重新加载模型。

在线修改更新内存对象和参数服务器，不写 YAML。仅 rosparam set 或修改磁盘 YAML 不会自动重建算法；需调用动态服务/rqt，或改 YAML 后重启。恢复默认指本次启动加载的 YAML 值；要跨次运行保留调整，手动写回 YAML。该行为依据已安装 Noetic 实现及 [dynamic_reconfigure Server 源码](https://github.com/ros/dynamic_reconfigure/blob/noetic-devel/include/dynamic_reconfigure/server.h)。

## 神经网络识别

### 运行库与模型资源（8.1）

在线推理由 ONNX Runtime 1.22.0 的 C++ CPU Session 执行，相关实现位于 nn_detector.hpp/cpp。该库使用 C++17，公共头文件通过 Impl 隐藏第三方类型；CMake 从 ONNXRUNTIME_ROOT 查找 SDK，默认是项目 .deps/onnxruntime。SDK 安装入口为 setup_inference.sh，下载官方 Linux x86_64 包并核对 SHA-256；不改变系统 OpenCV。参考：[官方发布](https://github.com/microsoft/onnxruntime/releases/tag/v1.22.0)、[Session API](https://onnxruntime.ai/docs/api/c/struct_ort_1_1_session.html)。

prepare_model.py 使用本地 ONNX 1.17.0、NumPy 1.24.4、protobuf 4.25.3，读取 task/nn_model.zip，保存原模型 armor.original.onnx、运行模型 armor.onnx、两张原始样例及 manifest.json。模型 IR=10、opset=22，内部包含 FP16 权重，输入输出为 float32。原模型图有 250 个节点，派生模型按依赖稳定排序；不改变节点属性、权重数值或精度，准备时逐项核对序列化内容并执行 ONNX checker。哈希与工具版本保存于清单。图的节点须满足依赖顺序，见 [ONNX IR](https://onnx.ai/onnx/repo-docs/IR.html#graphs)。

节点仅在 mode=nn 或 compare_modes=true 时创建 Session，启动时检查输入/输出名称、类型、数量和形状。缺模型或协议不符明确退出，不切换为其他算法。Python 只参与资源准备，在线没有 Python 推理或现成检测框架。

### 预处理和坐标恢复（8.2）

给定原图 W×H，以 min(640/W,480/H) 缩放，取整后居中补齐到 640×480，像素值为 124。奇数补边的多余像素放在右侧/下侧。保存实际 resized_size、scale_x/scale_y 和左/上补边；取整后的两轴实际比例用于反变换，避免奇数尺寸的舍入偏差。

BGR 转 RGB，像素除以 255，并将交错的 HWC 像素拆入连续的 NCHW `[1,3,480,640]`。NnInput 持有这段 float 内存，生命周期覆盖同步 Session::Run。逆变换为 x=(u-left)/scale_x、y=(v-top)/scale_y，不裁剪角点以伪造完整板。

### 输出解码与去重（8.3）

输出为 `[1,21,6300]`，读取索引 channel×6300+candidate，不将其当作 6300×21 的连续候选行。

| 通道 | 含义 |
|---|---|
| 0～3 | 蓝、红、熄灭、其他的分数；只保留胜出类别为蓝/红的候选 |
| 4～12 | 哨兵、1、2、3、4、5、前哨站、基地、基地的分数 |
| 13～20 | 模型画布四点，顺序左上、左下、右下、右上 |

分数不再 sigmoid。颜色与编号分别 argmax，再检查颜色≥0.5、编号≥0.5、乘积≥0.25。拒绝非有限/越范围分数、非有限/越界角点、非凸或反向四边形，以及不满足公共最小面积/边长的候选。角点按 0/3/2/1 转为共用 TL/TR/BR/BL 并反变换。

以分数乘积降序排序，相同分数用原输出索引决定顺序，最多取 300 个合法候选；对四点外接框做跨颜色/编号的贪心 NMS，IoU>0.5 时抑制后来的低分候选。保留结果按中心 x/y 排序，重新赋帧内 ID。四项分数/IoU 门限支持在线调整，候选上限启动读取。

模型没有大小输出。大小判别沿用中心距/平均灯条长度的几何比值，分界为 traditional/large_armor_min_spacing_ratio（默认 3.4）；透视压缩会影响分类。number_class 保留模型的 0～8 类别，调试图显示 #1、#3 等；原消息没有编号字段，因此不改写 Detection.id 或消息定义。

### 共用位姿与对比（8.4、8.5）

两种识别返回 ArmorCandidate，选中模式共用 PnpSolver、PoseHistory、buildDetectionOutput 和跟踪节点。DetectionFrame、TF、debug、comparison 沿用输入图像 header；模型未检出时发布空检测帧。HSV 掩膜只在执行传统识别时发布。

mode 为启动参数 tradition/nn，compare_modes 默认 false。开启时同一原图执行两种识别，在 /ft/debug/comparison 左右显示传统和 NN 的四点、编号、位姿；默认各缩放 0.5。只有选中模式发布业务检测和 TF 并更新位姿历史；另一侧使用同一 PnP 的单帧解，不更新历史或创建另一条跟踪链路。

所有耗时用 steady_clock。FT_TIMING 给出各阶段累计平均值与各自样本数，退出时输出带 FINAL 前缀的完整汇总；回调均值包括解码后图像的接入、识别、PnP、绘图和发布准备，对比开启时还包含另一分支和拼图。它不含 compressed 插件解码、网络传输、排队及下游跟踪。NN pre/infer/post 均值分别覆盖张量准备、同步 Run、解码去重；不包括模型加载，不剔除首帧。OpenCV 和 ORT intra-op 均配置为 2 线程，ORT inter-op 为 1。
