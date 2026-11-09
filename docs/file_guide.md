# 文件介绍

本文集中说明提交到 Git 的项目结构、文件用途和任务归属。学习问答见 [learning_log.md](learning_log.md)。

## 项目结构

目录仅列出纳入版本管理的源码、配置、工具和文档。

```text
vision_final_test/                     # 独立 Git 仓库、独立 catkin 工作空间
├── PLAN.md                            # 实施步骤与验收方式
├── README.md                          # 项目入口：环境、构建、运行、参数、限制
├── ai_usage.md                        # 关键 AI 辅助和本人项目决策
├── .gitignore                         # 排除素材、依赖、缓存和生成结果
├── run.sh                             # 环境检查、构建、启动、回放入口
│
├── src/
│   ├── ft_detection/                  # 素材提供的检测消息包，保留定义兼容
│   │   ├── CMakeLists.txt
│   │   ├── package.xml
│   │   ├── README.md
│   │   └── msg/
│   │       ├── Detection.msg
│   │       └── DetectionFrame.msg
│   │
│   ├── ft_msgs/                       # 自定义跟踪、目标消息
│   │   └── msg/
│   │       ├── Track.msg
│   │       └── Target.msg
│   │
│   ├── ft_detector/                   # 图像接入、两种识别方式、PnP、检测输出
│   │   ├── include/ft_detector/
│   │   │   ├── armor_candidate.hpp    # 两种识别方式共用的内部候选类型
│   │   │   ├── traditional_detector.hpp
│   │   │   ├── pnp_solver.hpp
│   │   │   ├── pose_history.hpp       # 上一帧位姿参考和唯一几何匹配
│   │   │   ├── detection_output.hpp   # ROS 检测消息和 TF 的共同输出接口
│   │   │   ├── tuning_parameters.hpp  # 算法参数与动态配置的转换
│   │   │   ├── debug_draw.hpp
│   │   │   └── nn_detector.hpp
│   │   ├── src/
│   │   │   ├── detector_node.cpp      # ROS 回调、CameraInfo、模式选择与发布
│   │   │   ├── traditional_detector.cpp
│   │   │   ├── pnp_solver.cpp
│   │   │   ├── pose_history.cpp       # 双解消歧所用历史的匹配与失效清理
│   │   │   ├── detection_output.cpp   # 字段检查、共享四元数与消息构造
│   │   │   ├── tuning_parameters.cpp  # 保留静态字段、转换 21 项在线参数
│   │   │   ├── debug_draw.cpp
│   │   │   └── nn_detector.cpp
│   │   └── cfg/
│   │       └── Detector.cfg           # 在线调参字段、范围和默认值
│   │
│   ├── ft_tracker/                    # 单装甲板匀速跟踪
│   │   ├── include/ft_tracker/
│   │   │   └── linear_tracker.hpp
│   │   └── src/
│   │       ├── linear_tracker.cpp     # 滤波、关联、轨迹生命周期、提前量
│   │       └── tracker_node.cpp       # ROS 输入输出、参数和跟踪 marker
│   │
│   ├── ft_selector/                   # 车体旋转估计与未来装甲板选择
│   │   ├── include/ft_selector/
│   │   │   ├── rotating_tracker.hpp
│   │   │   └── target_selector.hpp
│   │   └── src/
│   │       ├── rotating_tracker.cpp   # 车体状态、旋转 EKF、换板关联
│   │       ├── target_selector.cpp    # 未来可见性、选板和有效性
│   │       └── selector_node.cpp      # ROS 输入输出及车体、装甲板 marker
│   │
│   └── ft_bringup/                    # 集中管理启动、参数和可视化
│       ├── launch/
│       │   ├── demo.launch           # 对外统一启动入口
│       │   ├── detector.launch
│       │   ├── tracker.launch
│       │   ├── selector.launch
│       │   └── visualization.launch
│       ├── config/
│       │   ├── detector.yaml         # 传统识别、几何、尺寸和队列参数
│       │   ├── tracker.yaml          # Q/R、关联、丢失门限和 T
│       │   ├── selector.yaml         # 旋转模型、关联和选板参数
│       │   ├── model.yaml            # 模型路径、筛选和去重参数
│       │   └── datasets/
│       │       ├── blue_1.yaml
│       │       ├── blue_guard.yaml
│       │       ├── red_1_3.yaml
│       │       └── red_4.yaml
│       ├── scripts/
│       │   ├── view_debug_image.sh    # 读取节点配置的调试图话题并启动 rqt
│       │   ├── view_reconfigure.py    # 动态服务就绪后打开并选择调参窗口
│       │   └── view_rviz.py           # 从 CameraInfo 自动选取 RViz 固定坐标系
│       └── rviz/
│           └── vision.rviz
│
├── tools/                            # 开发辅助工具，不参与在线算法
│   ├── README.md                    # 独立性能工具的构建、参数和统计范围
│   ├── setup_inference.sh            # 安装本地 C++ 推理与模型准备依赖
│   ├── prepare_model.py              # 原模型检查、节点排序、哈希记录
│   ├── CMakeLists.txt                # 独立工具构建入口
│   ├── benchmark_pipeline.cpp       # 原始 bag 算法基准
│   ├── summarize_timings.py         # 平均值与 P95 汇总
│   ├── compare_detections.py        # 同帧检测拼图
│   ├── compare_predictions.py       # 线性/旋转预测对照
│   └── comparison.launch           # 对比工具启动入口
│
└── docs/
    ├── file_guide.md                 # 项目结构、文件用途和任务归属
    ├── images/
    │   ├── nn_corner_definition_check.png # NN 选点定义的临时实验结果图
    │   ├── tf_pose_rviz.png          # 装甲板 TF 位姿
    │   ├── debug_rqt_image_view.png  # rqt 中的识别调试图
    │   ├── tuning_rqt_reconfigure.png # 在线参数滑块
    │   ├── prediction_marker_rviz.png # 单板提前量位置标记
    │   ├── detection_comparison.png  # 原始 bag 的同帧检测对比
    │   ├── rotation_prediction.png   # 换板预测的投影对照
    │   └── ros_graph.png             # 实际话题图导出
    └── learning_log.md               # 项目相关的实际学习问答及必要纠正
```

结构约定：

- **每个 ROS 包均有自己的 `CMakeLists.txt` 和 `package.xml`**，树中除提供的消息包外省略重复展示。
- `.hpp` 声明类型和接口，`.cpp` 实现算法；节点文件负责 ROS 通信和组织调用。
- `ft_bringup` 管理配置与启动；检测、滤波和选板算法放在各自业务包内。
- 两种检测方式返回相同的 `ArmorCandidate`，随后使用同一个 PnP 和消息发布流程。
- 正式文档和功能所需辅助工具随模块增长；每次创建时解释用途。自动验证材料临时使用后清理，不加入项目目录蓝图和学习安排。

工作空间与环境约定：

- 直接在本项目根目录构建，保持工作空间与其他项目独立。
- 在已配置的 Noetic 环境中构建，保持本项目与其他 catkin 工作空间独立。
- `run.sh` 明确加载 Noetic 和本项目环境，避免父工作空间的自动 source 干扰。
- 本项目使用独立 Git 仓库，按自身忽略规则管理素材和生成文件。
- `run.sh` 提供入口，README 同时写明实际执行的命令，便于理解和排错。
- README 等提交文档使用通用 Noetic 环境说明和项目相对路径，不依赖个人绝对路径或外层环境的专用启动脚本。

## 文件用途与任务归属

| 文件 | 用途 | 任务归属 |
|---|---|---|
| [PLAN.md](../PLAN.md) | 实施顺序、学习步骤与验收方式 | 全程 |
| [README.md](../README.md) | 环境、构建、运行、仓库结构和按题目组织的实现说明 | 全程 |
| [ai_usage.md](../ai_usage.md) | 关键 AI 辅助和本人项目决策 | 全程 |
| [.gitignore](../.gitignore) | 忽略素材、依赖缓存和生成结果 | 1.1 |
| [run.sh](../run.sh) | 加载本项目环境，检查依赖，提供主工程与工具构建、基准、对比、完整链路启动和回放入口 | 1.1、1.4、7.1、10.1～10.3 |
| [docs/file_guide.md](file_guide.md) | 项目结构、当前文件用途及任务归属 | 全程 |
| [docs/learning_log.md](learning_log.md) | 项目相关的实际学习问答及必要纠正 | 全程 |
| [docs/images/nn_corner_definition_check.png](images/nn_corner_definition_check.png) | 模型输出四点与灯条端点的放大对照，供 Task8 项目决策说明使用 | 8.4 |
| [docs/images/tf_pose_rviz.png](images/tf_pose_rviz.png) | RViz 中装甲板的父坐标系、位置及 TF 坐标轴 | 3.4 |
| [docs/images/debug_rqt_image_view.png](images/debug_rqt_image_view.png) | rqt_image_view 中的识别框、角点、ID 和位姿轴 | 4.3 |
| [docs/images/tuning_rqt_reconfigure.png](images/tuning_rqt_reconfigure.png) | rqt_reconfigure 中的检测参数及滑块 | 5.2 |
| [docs/images/prediction_marker_rviz.png](images/prediction_marker_rviz.png) | RViz 中单板的当前位置、预测位置和连线 | 6.5 |
| [docs/images/detection_comparison.png](images/detection_comparison.png) | compare 工具从原始 red_1_3 回放导出的同帧检测图，左传统、右 NN | 8.5 |
| [docs/images/rotation_prediction.png](images/rotation_prediction.png) | 将同一时刻的旋转与线性预测投到后续图像，展示换板样例 | 9.5 |
| [docs/images/ros_graph.png](images/ros_graph.png) | 正常 NN 链路实际 rqt_graph 的核心话题导出图 | 10.3 |
| [src/ft_detection/CMakeLists.txt](../src/ft_detection/CMakeLists.txt) | 提供者原始消息构建声明 | 1.2、4.1 |
| [src/ft_detection/package.xml](../src/ft_detection/package.xml) | 提供者原始消息包依赖 | 1.2、4.1 |
| [src/ft_detection/README.md](../src/ft_detection/README.md) | 提供者原始消息说明 | 1.2、4.1 |
| [src/ft_detection/msg/Detection.msg](../src/ft_detection/msg/Detection.msg) | 单块装甲板的字段定义 | 4.1 |
| [src/ft_detection/msg/DetectionFrame.msg](../src/ft_detection/msg/DetectionFrame.msg) | 图像 header 和整帧检测数组定义 | 4.1、4.2 |
| [src/ft_msgs/CMakeLists.txt](../src/ft_msgs/CMakeLists.txt) | 生成并导出 Track 和 Target 消息 | 6.2、9.5 |
| [src/ft_msgs/package.xml](../src/ft_msgs/package.xml) | 自定义消息包依赖 | 6.2 |
| [src/ft_msgs/msg/Track.msg](../src/ft_msgs/msg/Track.msg) | 轨迹 ID、命中标志、位置、速度和未来位置 | 6.2、6.5 |
| [src/ft_detector/CMakeLists.txt](../src/ft_detector/CMakeLists.txt) | 构建算法、消息输出、调参适配、NN 库及检测节点 | 1～5、8.1 |
| [src/ft_detector/package.xml](../src/ft_detector/package.xml) | 检测节点、图像插件及推理资源准备依赖 | 1～5、8.1 |
| [src/ft_detector/cfg/Detector.cfg](../src/ft_detector/cfg/Detector.cfg) | 声明 21 项传统及 4 项 NN 在线字段、范围和默认值 | 5.2、5.3 |
| [src/ft_detector/include/ft_detector/armor_candidate.hpp](../src/ft_detector/include/ft_detector/armor_candidate.hpp) | 共用二维候选、四点与帧内 ID 类型 | 2.3、2.4 |
| [src/ft_detector/include/ft_detector/traditional_detector.hpp](../src/ft_detector/include/ft_detector/traditional_detector.hpp) | 传统检测参数、灯条及检测接口声明 | 2.1～2.4 |
| [src/ft_detector/src/traditional_detector.cpp](../src/ft_detector/src/traditional_detector.cpp) | HSV、轮廓筛选、灯条配对和冲突处理 | 2.1～2.4 |
| [src/ft_detector/include/ft_detector/pnp_solver.hpp](../src/ft_detector/include/ft_detector/pnp_solver.hpp) | 相机模型、物体点和位姿解算接口 | 3.1～3.4 |
| [src/ft_detector/src/pnp_solver.cpp](../src/ft_detector/src/pnp_solver.cpp) | IPPE、合理性筛选、参考选解和轴投影 | 3.1～3.4 |
| [src/ft_detector/include/ft_detector/pose_history.hpp](../src/ft_detector/include/ft_detector/pose_history.hpp) | 上一帧位姿参考的保存与查询接口 | 3.3 |
| [src/ft_detector/src/pose_history.cpp](../src/ft_detector/src/pose_history.cpp) | 唯一几何匹配与参考状态失效处理 | 3.3 |
| [src/ft_detector/include/ft_detector/detection_output.hpp](../src/ft_detector/include/ft_detector/detection_output.hpp) | 检测消息和 TF 的共享输出接口 | 4.1、4.2、4.4 |
| [src/ft_detector/src/detection_output.cpp](../src/ft_detector/src/detection_output.cpp) | 检查输出字段，构造一致的检测消息与 TF | 4.2、4.4 |
| [src/ft_detector/include/ft_detector/tuning_parameters.hpp](../src/ft_detector/include/ft_detector/tuning_parameters.hpp) | 算法参数与动态配置的转换接口 | 5.2、5.3 |
| [src/ft_detector/src/tuning_parameters.cpp](../src/ft_detector/src/tuning_parameters.cpp) | 保留静态字段并转换 21 项在线参数 | 5.2、5.3 |
| [src/ft_detector/include/ft_detector/debug_draw.hpp](../src/ft_detector/include/ft_detector/debug_draw.hpp) | 调试绘图接口声明 | 2.2～2.4、3.4、4.3 |
| [src/ft_detector/src/debug_draw.cpp](../src/ft_detector/src/debug_draw.cpp) | 在副本绘制灯条、候选、四点编号和有效位姿轴 | 2.2～2.4、3.4、4.3 |
| [src/ft_detector/src/detector_node.cpp](../src/ft_detector/src/detector_node.cpp) | 接收图像/标定，选择识别模式，共用 PnP/发布与当前模式调试图 | 1～5、8.4、8.5 |
| [src/ft_tracker/CMakeLists.txt](../src/ft_tracker/CMakeLists.txt) | 构建匀速跟踪库与 ROS 节点 | 6.2 |
| [src/ft_tracker/package.xml](../src/ft_tracker/package.xml) | 跟踪节点、OpenCV 与消息依赖 | 6.2 |
| [src/ft_tracker/include/ft_tracker/linear_tracker.hpp](../src/ft_tracker/include/ft_tracker/linear_tracker.hpp) | 滤波状态、观测、参数和更新接口 | 6.1～6.5 |
| [src/ft_tracker/src/linear_tracker.cpp](../src/ft_tracker/src/linear_tracker.cpp) | 预测/校正、关联、生命周期和未来位置 | 6.1～6.5 |
| [src/ft_tracker/src/tracker_node.cpp](../src/ft_tracker/src/tracker_node.cpp) | 读取参数、接收检测并发布 Track 和 marker | 6.2～6.5 |
| [src/ft_bringup/CMakeLists.txt](../src/ft_bringup/CMakeLists.txt) | 安装配置、RViz、查看器和 tools 下的 ROS 对比脚本 | 1.4、7.1、10.2 |
| [src/ft_bringup/package.xml](../src/ft_bringup/package.xml) | 启动、选板、可视化、话题图、消息同步与独立工具依赖 | 1.4、7.1、9.5、10.2、10.3 |
| [src/ft_bringup/launch/demo.launch](../src/ft_bringup/launch/demo.launch) | 默认启动检测、跟踪和选板，分别控制各节点与可视化窗口 | 7.1、8.5、9.5、10.1 |
| [src/ft_bringup/launch/detector.launch](../src/ft_bringup/launch/detector.launch) | 依次加载公共/数据集配置并启动检测节点 | 1.4 |
| [src/ft_bringup/launch/tracker.launch](../src/ft_bringup/launch/tracker.launch) | 加载跟踪参数并启动跟踪节点 | 6.5 |
| [src/ft_bringup/launch/visualization.launch](../src/ft_bringup/launch/visualization.launch) | 分别控制图像、RViz 和动态调参窗口 | 1.4、7.1 |
| [src/ft_bringup/config/detector.yaml](../src/ft_bringup/config/detector.yaml) | 公共话题、队列、HSV、几何、尺寸和 PnP 参数 | 1～5 |
| [src/ft_bringup/config/tracker.yaml](../src/ft_bringup/config/tracker.yaml) | Q/R、初始方差、关联/丢失门限、T 和 marker 参数 | 5.1、6.1～6.5 |
| [src/ft_bringup/config/datasets/blue_1.yaml](../src/ft_bringup/config/datasets/blue_1.yaml) | blue_1 的 bag 文件名，沿用公共相机入口 | 1.4 |
| [src/ft_bringup/config/datasets/blue_guard.yaml](../src/ft_bringup/config/datasets/blue_guard.yaml) | blue_guard 的 bag 文件名，沿用公共相机入口 | 1.4 |
| [src/ft_bringup/config/datasets/red_1_3.yaml](../src/ft_bringup/config/datasets/red_1_3.yaml) | red_1_3 的 bag 文件名，沿用公共相机入口 | 1.4 |
| [src/ft_bringup/config/datasets/red_4.yaml](../src/ft_bringup/config/datasets/red_4.yaml) | red_4 的 bag 文件名，沿用公共相机入口 | 1.4 |
| [src/ft_bringup/scripts/view_debug_image.sh](../src/ft_bringup/scripts/view_debug_image.sh) | 读取配置的调试图话题并打开图像窗口 | 1.4 |
| [src/ft_bringup/scripts/view_rviz.py](../src/ft_bringup/scripts/view_rviz.py) | 等待 CameraInfo 并依据实际 frame_id 管理 RViz | 3.4、7.1 |
| [src/ft_bringup/scripts/view_reconfigure.py](../src/ft_bringup/scripts/view_reconfigure.py) | 等待动态服务后启动并定位调参窗口 | 7.1 |
| [src/ft_bringup/rviz/vision.rviz](../src/ft_bringup/rviz/vision.rviz) | 装甲板 TF、单板跟踪及旋转选板的 marker 显示配置 | 3.4、6.5、9.5 |
| [tools/setup_inference.sh](../tools/setup_inference.sh) | 本地 C++ SDK 下载/校验及模型准备依赖安装 | 8.1 |
| [tools/prepare_model.py](../tools/prepare_model.py) | 原模型保留、稳定节点排序、协议检查与哈希清单 | 8.1 |
| [src/ft_detector/include/ft_detector/nn_detector.hpp](../src/ft_detector/include/ft_detector/nn_detector.hpp) | NN 参数、张量/变换、解码结果及推理接口 | 8.1～8.4 |
| [src/ft_detector/src/nn_detector.cpp](../src/ft_detector/src/nn_detector.cpp) | ORT Session、RGB/NCHW、坐标反变换、分类/几何筛选和 NMS | 8.1～8.4 |
| [src/ft_bringup/config/model.yaml](../src/ft_bringup/config/model.yaml) | 模型协议、4 项动态门限和线程参数 | 8.1～8.5 |
| [src/ft_selector/CMakeLists.txt](../src/ft_selector/CMakeLists.txt) | 构建 Eigen 旋转算法库和 ROS 节点 | 9.3～9.5 |
| [src/ft_selector/package.xml](../src/ft_selector/package.xml) | 旋转选板包与 ROS、Eigen 依赖 | 9.3～9.5 |
| [src/ft_selector/include/ft_selector/rotating_tracker.hpp](../src/ft_selector/include/ft_selector/rotating_tracker.hpp) | 圆周模型、观测、八维状态和滤波接口 | 9.1～9.4 |
| [src/ft_selector/src/rotating_tracker.cpp](../src/ft_selector/src/rotating_tracker.cpp) | 圆周几何、角度环绕、旋转 EKF、换板关联和生命周期 | 9.1～9.4 |
| [src/ft_selector/include/ft_selector/target_selector.hpp](../src/ft_selector/include/ft_selector/target_selector.hpp) | 选板接口、当前位置和未来位置 | 9.5 |
| [src/ft_selector/src/target_selector.cpp](../src/ft_selector/src/target_selector.cpp) | 用未来相位和朝向选板，设置目标有效标志 | 9.5 |
| [src/ft_selector/src/selector_node.cpp](../src/ft_selector/src/selector_node.cpp) | 从检测位姿提取朝向，发布 Target 和 RViz marker | 9.1、9.4、9.5 |
| [src/ft_msgs/msg/Target.msg](../src/ft_msgs/msg/Target.msg) | 车体 ID、有效标志、当前板及未来板位置 | 9.5 |
| [src/ft_bringup/config/selector.yaml](../src/ft_bringup/config/selector.yaml) | 圆周半径、槽位、EKF 噪声、关联和选板参数 | 9.1～9.5 |
| [src/ft_bringup/launch/selector.launch](../src/ft_bringup/launch/selector.launch) | 启动选板并读取与单板跟踪相同的提前量 | 9.5 |
| [tools/CMakeLists.txt](../tools/CMakeLists.txt) | 链接现有算法库，独立构建性能基准 | 10.2 |
| [tools/README.md](../tools/README.md) | 性能工具的构建、参数、CSV 字段和统计范围 | 10.2 |
| [tools/benchmark_pipeline.cpp](../tools/benchmark_pipeline.cpp) | 顺序读取原始 bag 并测量算法阶段耗时，输出逐帧 CSV | 10.2 |
| [tools/summarize_timings.py](../tools/summarize_timings.py) | 按数据集和模式计算平均值、最近秩 P95 和样本数 | 10.2 |
| [tools/compare_detections.py](../tools/compare_detections.py) | 按时间戳与坐标系同步两路调试图并拼接 | 8.5、10.2 |
| [tools/compare_predictions.py](../tools/compare_predictions.py) | 同步 Track/Target 并发布预测端点对照灰线 | 9.5、10.2 |
| [tools/comparison.launch](../tools/comparison.launch) | 启动正常链路、隔离的辅助检测和两种对比工具 | 10.2 |
