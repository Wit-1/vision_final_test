# 感知链路接口与坐标约定

本文件记录实现采用的约定；实际验证范围及尚未确认的内容见[验收记录](acceptance.md)。

## 数据流与模块边界

```text
原始 compressed 图像 + CameraInfo
  → detector_node：压缩解码、标定缓存和参数
  → TraditionalDetector：二维灯条与 ArmorCandidate
  → PoseHistory.references：上一帧中唯一匹配的位姿参考
  → PnpSolver：几何校验、平面候选、带条件的跨帧消歧与重投影
  → PoseHistory.store：整帧求解结束后保存本帧合格目标
  → drawDetections / drawPoses：独立副本上的框、标记和坐标轴
  → /ft/debug/image、两个 mask；合格位姿 → /tf
```

检测、PnP 和绘图位于无 ROS 依赖的 ft_detector_core。节点负责 ROS 接口和 TF。第 4 点将增加检测消息；目前没有把默认或无效位姿发布成检测结果。

## 像素点、物体点和单位

采用灯条长轴端点，图像点始终为：

| 索引 | 图像点 | 物体坐标（米） |
|---|---|---|
| 0 | 左灯上端 | (-W/2, -L/2, 0) |
| 1 | 右灯上端 | (+W/2, -L/2, 0) |
| 2 | 右灯下端 | (+W/2, +L/2, 0) |
| 3 | 左灯下端 | (-W/2, +L/2, 0) |

W 是灯条中心距，L 是对应发光段的长度。坐标原点为两灯中心连线中点；X 沿板向右，Y 沿板向下，Z=X×Y，在正面观察时指向板背后，构成右手系。相机采用光学坐标：x 右、y 下、z 前。旋转后物体轴无需再与相机轴平行。

图像点单位为像素；尺寸、平移、TF 和坐标轴长度统一为米；内部旋转向量为弧度。题面列出大板 235×127 mm、小板 120×125 mm，但未明确标注它们对应端点矩形 W×L 还是板本体 Wb×Hb。2026-11-02 本人先选择临时按题面尺寸使用，随后指定试改为小板 135×56 mm、大板 230×56 mm，并说明当前尺寸来源为 Alliance2026 开源项目。当前 YAML 为小板 0.135/0.056 m、大板 0.230/0.056 m；来源据本人提供，仍保留定义一及未与本素材实物核对的标记。

YAML 中 small、large 分别保存 width_m、height_m、confirmed。confirmed 表示实际端点尺寸是否核实；小板和大板分别确认，不能用一个类别的确认替代另一个。

allow_provisional_dimensions 默认在算法参数中为 false；当该开关关闭，未确认类别返回 dimensions_unconfirmed。当前 YAML 按本人选择显式开启临时开关，允许未确认但为正数的尺寸进入 PnP；零尺寸仍拒绝。PnpResult.provisional 标明本次使用了临时尺寸；valid 只代表软件几何检查合格，不代表物理尺度已核实。调试图的状态与每个合格位姿标签都会标注 provisional，TF 则使用同一个临时模型结果。

## 相机模型

从 CameraInfo 获取原始图像的 K、D、尺寸及 frame_id，不使用 rectified 图像对应的投影矩阵 P。图像必须与缓存的标定尺寸和 frame_id 一致。

当前支持 plumb_bob（D 为空或 4/5 个系数）、rational_polynomial（8 个系数）；未知模型、非有限数据、无效焦距或 K 结构均拒绝。非平凡 ROI 和 binning>1 的坐标换算尚未实现，因此明确拒绝这类标定。无效 CameraInfo 清空缓存，不能继续用先前标定做三维解算；二维识别仍运行。

四份原始素材当前均为 1440×1080、plumb_bob、5 个 D 系数，无 ROI 或 binning 换算。CameraInfo 将 frame_id 定义为相机光学坐标系；red_4 的名字 hk_camera_ca 不含 optical 字样，但不据名称擅自改轴或重命名。

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

重投影误差小只能检查像素对应的一致性，不能证明尺寸尺度正确。把 W、L 同时扩大一倍，平移也会扩大一倍而投影保持相同；测试中明确验证此行为。

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

pnp.enabled 关闭全部三维求解；pnp.publish_tf 仅关闭 TF 广播，合格位姿仍可投影显示。全部几何尺寸、深度、面积、边长、重投影误差和轴长由 YAML 加载，修改后需重启。在线调参仍在第 5 点实现。

RViz 通过 demo.launch 的 rviz 参数按需启动，rviz_fixed_frame 仅配置查看器，不改变实际 TF 父坐标系。前三份素材使用 camera_optical_frame，red_4 使用 hk_camera_ca；其他输入须依据实际 CameraInfo 设置查看器。

## 接口依据

- [OpenCV 4.2 标定、PnP 与投影接口](https://docs.opencv.org/4.2.0/d9/d0c/group__calib3d.html)。
- [ROS CameraInfo 定义](https://github.com/ros/common_msgs/blob/noetic-devel/sensor_msgs/msg/CameraInfo.msg)。
- [ROS REP-103 单位与坐标约定](https://github.com/ros-infrastructure/rep/blob/master/rep-0103.rst)。
