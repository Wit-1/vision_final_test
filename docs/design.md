# 感知链路设计

本文说明数据接口、坐标和算法约定。运行命令见 [README](../README.md)，验证条件与结果见[验收记录](acceptance.md)。

## 数据流与接口

```text
压缩图像 + CameraInfo
  → image_transport 解码 → 检测节点
  → 传统识别 / NN → ArmorCandidate → PnP 与历史选解
  → /ft/detection ┬→ 单板平移跟踪 → /ft/track
                  └→ 旋转 EKF 与未来选板 → /ft/target
  → TF、调试图、同帧对比图和 RViz marker
```

算法库处理几何、滤波和选板，ROS 节点负责参数、消息转换和发布。两种识别共用 ArmorCandidate、PnP 与检测输出。

| 接口 | 关键约定 |
|---|---|
| DetectionFrame | 沿用素材消息定义，一份 header 和 detections 数组。每张成功处理的图像发布一帧，缺标定、无目标或位姿全部失败时发布空数组 |
| Detection | id 是帧内编号，PnP 过滤后可不连续。pose 是板原点在相机系的位姿。corners 为原图像素，z 固定为 0。confidence 为传统几何评分或 NN 颜色分与编号分的乘积 |
| Track | id 是单板轨迹 ID，tracking 表示本帧有观测命中。position、velocity 和 position_predict 为当前位置、速度与提前量位置。id=0 表示无轨迹 |
| Target | target_id 是车体轨迹 ID。position 是当前板估计，position_predict 是未来选中板位置，两者可属于不同槽位。valid 表示当前观测与未来几何选板均满足条件 |

检测消息和对应 TF 共用一次位姿转换的结果。发布前检查有效标志、ID 对应、有限值及评分范围。位姿失败的候选被过滤，空检测帧供跟踪推进丢失状态。

所有业务消息、TF 和调试图沿用图像时间戳与相机 frame_id，跨话题按二者对齐。header.seq 由发布者维护。帧内 Detection.id、模型编号、单板 Track.id 和车体 Target.target_id 分别使用。

## 坐标、尺寸与相机

相机光学坐标为 x 向右、y 向下、z 向前。装甲板坐标原点在灯条四端点中心，X 沿板向右、Y 沿板向下，Z=X×Y 指向板背后。距离和尺寸用米，速度用 m/s，角度用弧度，图像坐标用像素。

采用灯条端点模型，二维点与三维物体点固定对应。

| 索引 | 图像点 | 物体点（m） |
|---|---|---|
| 0 | 左上 | (-W/2, -L/2, 0) |
| 1 | 右上 | (+W/2, -L/2, 0) |
| 2 | 右下 | (+W/2, +L/2, 0) |
| 3 | 左下 | (-W/2, +L/2, 0) |

W 为灯条中心距，L 为发光段长度。当前小板为 0.135×0.056 m，大板为 0.230×0.056 m，来源为本人提供的 Alliance2026 开源参数，尚待核对素材实物。YAML 中大小板分别保留 confirmed=false，allow_provisional_dimensions=true 允许临时解算，并在图上标注 provisional。关闭临时开关后，未确认类别停止输出位姿。

CameraInfo 提供 K、D、图像尺寸和实际 frame_id。图像须与缓存标定的尺寸、帧名一致。支持 plumb_bob 的 0/4/5 个畸变系数和 rational_polynomial 的 8 个系数。未知模型、无效内参、非平凡 ROI 或 binning>1 的标定会被拒绝并清空缓存。四份原始素材均为 1440×1080，共用 /hk_camera/image_raw/compressed 和 /hk_camera/camera_info。

## 识别与位姿

传统识别依次进行 HSV 筛色、灯条轮廓与长轴筛选、灯条配对和冲突处理。红色使用两段 H 区间。几何距离以平均灯条长度归一化，默认间距比达到 3.4 判大板。共享灯条的明显较差配对被拒绝，代价接近的歧义配对放弃。confidence=1/(1+cost)。

PnP 使用 solvePnPGeneric 与 SOLVEPNP_IPPE 求平面候选解，检查四点几何、正深度、正面朝向及重投影误差。默认深度范围 0.05～30 m，四点重投影 RMS 上限 3 px。有效历史存在时，在当帧最小误差加 1 px 范围内，选与参考旋转最接近的合格解。

PoseHistory 只保存上一帧有效位姿。颜色、大小和归一化端点位移通过门控，且双方互为唯一匹配时才提供参考，默认位移门限 0.5。整帧解算后统一保存，避免参考失效。时间重复、回退或间隔超过 0.1 s，标定、尺寸、帧名或有效调参变化，以及处理异常，都会清理参考。首帧与失去参考时选最小重投影误差分支，首次朝向仍可能有平面双解歧义。

变换关系为 P_camera=R×P_armor+t。TF 以相机为父系，以 armor_<帧内ID> 为子系，直接使用 R 和 t。坐标轴投影与 TF 使用同一个完整解，X/Y/Z 分别绘成红/绿/蓝。空帧停止发送该帧装甲板 TF，RViz 仍可能保留历史坐标轴。

重投影误差用于检查像素对应。将 W、L 同比例放大，求得的平移也随之放大，投影仍可相同，因此米制精度需要真实端点尺寸支撑。

## 单装甲板平移跟踪

使用双精度六维匀速卡尔曼滤波，状态为 [x,y,z,vx,vy,vz]，观测为板中心 [x,y,z]。每帧用图像 dt 预测，再用关联观测校正。各轴过程噪声为 q×[[dt³/3,dt²/2],[dt²/2,dt]]，默认 q=2 m²/s³，位置观测方差 R=0.0025 m²。

首次选择最近的有效板，随后在预测位置 0.5 m 内关联最近且颜色、大小兼容的观测。锁定后保持轨迹 ID。无匹配时保留预测并置 tracking=false，自最后观测严格超过 0.5 s 后清除，可在清除的同一帧用有效观测建立新 ID。时间重复、回退或相机帧名变化会重置状态，进程内 ID 继续递增。

position_predict=position+velocity×T，默认 T=0.1 s。预测使用相机系中的相对运动，相机运动、同类目标交叉和旋转换板都会影响关联。

## 神经网络识别

在线使用 ONNX Runtime 1.22.0 C++ CPU Session，默认两线程。SDK 保存在 .deps/onnxruntime，prepare_model.py 保留原模型，只对运行模型重排节点并验证内容，将哈希和版本写入 manifest.json。模型为 IR 10、opset 22，内部含 FP16 权重，输入输出为 float32。Session 在启动时检查协议，随后逐帧复用。

| 项目 | 协议 |
|---|---|
| 输入 | images，RGB float32 NCHW [1,3,480,640] |
| 输出 | output0，[1,21,6300]，按 channel×6300+candidate 读取 |
| 通道 0～3 | 蓝、红、熄灭、其他分数 |
| 通道 4～12 | 哨兵、1、2、3、4、5、前哨站、基地、基地分数 |
| 通道 13～20 | 模型画布四点，左上、左下、右下、右上 |

原图等比缩放并居中补齐到 640×480，补边值 124。BGR 转 RGB、除以 255，再按通道拆入连续张量。保存取整后的实际横纵比例和补边，用 x=(u-left)/scale_x、y=(v-top)/scale_y 还原角点。

直接使用输出分数。颜色和编号各取最高分类，仅保留胜出颜色为红或蓝、颜色及编号分均≥0.5、乘积≥0.25 的候选。角点按 0/3/2/1 重排为公共点序，再做边界、凸性、面积和边长检查。至多保留分数最高的 300 个合法候选，对外接矩形执行跨类别贪心 NMS，IoU>0.5 时抑制低分候选。剩余候选按中心排序并分配帧内 ID。

大小分类沿用几何间距比。模型编号在调试图中显示为 #1、#3 等。mode 为 tradition 或 nn，compare_modes 开启后同帧绘制左传统、右 NN。选中模式发布业务检测、TF 并更新位姿历史，另一侧用单帧 PnP 绘图。

## 旋转车体与装甲板选择

用相机 X-Z 平面近似旋转平面，各板等高、等半径、等角分布。八维状态为 [cx,cy,cz,vx,vy,vz,θ,ω]。默认四槽位、半径 r=0.25 m，这是缺乏实测依据的试验值。实际半径应为旋转轴到灯条端点中心的水平距离，偏差会影响圆心、未来板位置及关联。

槽位 i 的相位 θᵢ=wrapAngle(θ+2πi/N)，朝外法向 nᵢ=(sin θᵢ,0,cos θᵢ)，板中心 pᵢ=c+r×nᵢ。PnP 局部 +Z 取反得到朝外法向，投到 X-Z 平面后用 atan2(nx,nz) 提取角度，初始化用 c=p-r×n 反推圆心。

EKF 用图像 dt 推进圆心和相位，观测为匹配槽位的三维板中心和法向角。位置对相位的雅可比为 r(cos θᵢ,0,-sin θᵢ)，相位和角残差均环绕到 ±π，协方差使用 Joseph 形式更新。

首次选最近板，随后检查各槽位与同色观测的位置差和角差，默认门限为 0.40 m、0.70 rad，每帧用归一化误差平方和最小的一组校正。换板保留车体 ID。短时无匹配时继续预测并置 valid=false，严格超过 0.5 s 的首帧清空，后续观测可建立新 ID。时间回退或帧名变化清理状态，重复图像帧跳过。

提前量沿用 tracker.yaml。对 t+T 的各槽位，检查有限坐标、正深度以及朝外法向与指向相机方向的点积，选择得分最高且严格超过 0.20 的槽位。未来板可以当前不可见。本帧有匹配、当前板深度为正且选出合格未来槽位时 valid=true，失联输出 ID 0 和零占位。

旋转模型可同时表达平移和转动。相机运动、突然变速、姿态噪声、不同板高度或半径，以及遮挡和视野边界，都可能使几何选板与实际可见板不一致。

## 参数与可视化

检测参数按 detector.yaml→model.yaml→数据集 YAML 加载，同名键由后加载的值覆盖。CameraInfo 提供相机内参，tracker.yaml 和 selector.yaml 分别配置两种运动模型，选板读取 tracker.yaml 的同一提前量。

/ft_detector/tuning 支持 21 项传统字段和 4 项 NN 字段在线调整。启动 YAML 决定初值与实例默认值，非法组合整组回退，合法变化清空 PnP 历史。在线值保存在内存和参数服务器，跨次运行保存需写回 YAML。其他配置修改后重启。

| RViz 标记 | 含义 |
|---|---|
| 绿色球、橙色球和橙线 | 单板当前位置、线性未来位置及连线 |
| 青色方块、紫色球、蓝色球 | 车体中心、当前板和未来选中板 |
| 灰线 | 同时间戳、同坐标系且两路均有观测时，连接两种预测位置 |

短时纯预测的标记变淡，轨迹清除时发送 DELETE。两路独立锁定，比较时需确认是同一车辆。蓝紫用于看提前选板，蓝橙用于比较预测，精度需对应 T 秒后的实际画面。

图像、RViz 和调参窗口分别由 gui、rviz、reconfigure 控制，选板由 selector 开启。RViz 根据 CameraInfo 自动选择固定坐标系。耗时使用单调时钟，FT_TIMING 报告检测、NN 三阶段和图像回调的累计平均毫秒数。回调包括 PnP、绘图和发布准备，对比开启时包括两条分支。网络传输、输入排队和 compressed 解码在回调计时之前，下游跟踪在其之后。

## 参考接口

- [OpenCV 4.2 PnP 与投影](https://docs.opencv.org/4.2.0/d9/d0c/group__calib3d.html)
- [OpenCV 4.2 KalmanFilter](https://docs.opencv.org/4.2.0/dd/d6a/classcv_1_1KalmanFilter.html)
- [ROS CameraInfo](https://github.com/ros/common_msgs/blob/noetic-devel/sensor_msgs/msg/CameraInfo.msg)
- [ROS REP-103](https://github.com/ros-infrastructure/rep/blob/master/rep-0103.rst)
- [ONNX 图结构规范](https://onnx.ai/onnx/repo-docs/IR.html#graphs)
- [ONNX Runtime 1.22.0](https://github.com/microsoft/onnxruntime/releases/tag/v1.22.0)
