# 视觉组最终考核：项目结构、逐步实施与提交计划

## 1．目标与推进规则

完成题目**第 1～7 点必做，以及第 8～10 点全部选做**，暂不安排附加题。任务编号依据本地素材 `task/视觉组最终考核.pdf`。

以下文件路径均相对项目根目录 `vision_final_test/`。

本计划保存在项目根目录，说明项目结构、实施步骤及验收方式。目录蓝图包含尚待实现的后续模块。

推进方式：

1. 每个题目点可集中搭建所需文件，再按小步骤依次讲解；先说明目标和涉及的文件。
2. 展示改动，解释文件用途、调用关系和关键代码。
3. 运行验证，对照预期与实际结果。
4. 通过问答或参数实验确认理解，等本人确认后继续。
5. **完成一个题目点后集中验收，再展示改动清单和英文 commit 说明，得到本人明确确认后提交。commit 说明首字母大写，以句号结尾。**

小步骤不单独 commit。理解确认与 commit 授权分别处理，不能相互代替。PLAN 只保留实施计划；项目学习问答保存在 docs/learning_log.md，关键 AI 辅助和本人项目决策保存在 ai_usage.md，提交历史由 Git 保存。

`ai_usage.md`、`docs/learning_log.md` 和 `docs/file_index.md` 在开发期间保留本地，项目最终整理时统一提交。learning_log 只保留项目相关的实际学习问答及必要纠正；ai_usage 简要记录关键辅助内容和本人项目决策，不记录 AI 自行排错的过程。

自动化验证文件按临时材料处理，不作为学习内容或提交内容；此前的测试源码、测试配置及构建注册一并清理。后续验证脚本放在 `/tmp`，验证结束后清理，必要的结果与限制保留在验收记录。本人只需学习实际功能代码、配置和运行方式。

## 2．最终项目结构

以下是最终目录蓝图，文件随对应任务逐步创建。

```text
vision_final_test/                     # 独立 Git 仓库、独立 catkin 工作空间
├── PLAN.md                            # 项目结构、实施步骤与验收方式
├── README.md                          # 项目入口：环境、构建、运行、参数、限制
├── ai_usage.md                        # 关键 AI 辅助和本人项目决策
├── .gitignore                         # 排除素材、依赖、缓存和生成结果
├── run.sh                             # 环境检查、构建、启动、回放入口
│
├── task/                              # 已有原始素材，保留原件，不进入 Git
│   ├── 视觉组最终考核.pdf
│   ├── ft_detection.zip
│   ├── nn_model.zip
│   ├── blue_1.bag
│   ├── blue_1_with_detection.bag
│   ├── blue_guard.bag
│   ├── blue_guard_with_detection.bag
│   ├── red_1&3.bag
│   ├── red_1&3_with_detection.bag
│   ├── red_4.bag
│   └── red_4_with_detection.bag
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
│   │       └── selector_node.cpp      # ROS 输入输出及旋转对比 marker
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
│   ├── setup_inference.sh            # 安装本地 C++ 推理与模型准备依赖
│   ├── prepare_model.py              # 原模型检查、节点排序、哈希记录
│   └── summarize_timings.py          # 处理耗时数据，计算平均和 P95
│
├── docs/
│   ├── file_index.md                 # 每个文件的用途和任务归属
│   ├── design.md                     # 数据流、接口、坐标、尺寸和模型协议
│   ├── learning_log.md               # 项目相关的实际学习问答及必要纠正
│   └── acceptance.md                 # 原题第 1～10 点验收表和复现命令
│
├── assets/                           # 解压素材与派生模型，不进入 Git
│   ├── models/
│   └── samples/
├── .deps/                            # 本地第三方依赖缓存，不进入 Git
├── runs/                             # 日志、截图、计时 CSV，不进入 Git
├── video/                            # 演示视频，单独交付
├── dist/                             # 最终提交包，不进入 Git
└── build/、devel/、install/、logs/、
    .catkin_tools/                    # catkin 自动生成，不进入 Git
```

结构约定：

- **每个 ROS 包均有自己的 `CMakeLists.txt` 和 `package.xml`**，树中除提供的消息包外省略重复展示。
- `.hpp` 声明类型和接口，`.cpp` 实现算法；节点文件负责 ROS 通信和组织调用。
- `ft_bringup` 管理配置与启动；检测、滤波和选板算法放在各自业务包内。
- 两种检测方式返回相同的 `ArmorCandidate`，随后使用同一个 PnP 和消息发布流程。
- 素材目录与解压后的运行资源分开，原件保持完整。
- 正式文档和功能所需辅助工具随模块增长；每次创建时解释用途。自动验证材料临时使用后清理，不加入项目目录蓝图和学习安排。

工作空间与环境约定：

- 直接在本项目根目录构建，生成自己的 `build/` 和 `devel/`。
- 在已配置的 Noetic 环境中构建，保持本项目与其他 catkin 工作空间独立。
- `run.sh` 明确加载 Noetic 和本项目环境，避免父工作空间的自动 source 干扰。
- 本项目使用独立 Git 仓库，按自身忽略规则管理素材和生成文件。
- `run.sh` 提供入口，README 同时写明实际执行的命令，便于理解和排错。
- README 等提交文档使用通用 Noetic 环境说明和项目相对路径，不依赖个人绝对路径或外层环境的专用启动脚本。

## 3．任务点、学习步骤和文件归属

基础执行顺序为 **1→2→3→4→6→5→7**。第 5 点包含跟踪参数，因此在第 6 点完成后集中验收；前面各点从实现开始就使用 YAML。

| 任务点 | 分步实施 | 主要文件归属与验收 |
|---|---|---|
| **第 1 点：图像接入与相机参数** | **1.1** 初始化独立工作空间、Git、计划及忽略规则；**1.2** 解释素材与依赖，准备消息包；**1.3** 接入压缩图像、缓存 CameraInfo、保留 header；**1.4** 配置不同 bag 的入口并在 rqt 看图 | 基础构建文件、`run.sh`、`detector_node.cpp`、数据集配置和最小 launch。四种原始 bag 均可接入，相机参数来自 CameraInfo。确认后 commit |
| **第 2 点：传统装甲板识别** | **2.1** 蓝色 HSV，再加红色双区间；**2.2** 灯条轮廓与方向；**2.3** 配对、冲突处理和大小判别；**2.4** 多目标、帧内 ID、遮挡及反光错例 | `armor_candidate.hpp`、传统检测、调试绘图和检测参数。能解释每个候选为何保留或拒绝。确认后 commit |
| **第 3 点：位姿解算与 TF** | **3.1** 确认四点及对应物理尺寸；**3.2** 固定顶点顺序、原点、右手坐标轴和米制单位；**3.3** PnP、正深度、合理性及带条件的跨帧选解；**3.4** 重投影坐标轴和 TF | PnP、PoseHistory 和设计文档。检查点对应、尺度、变换方向、连续性及历史清理；尺寸未确认时暂停米制验收。确认后 commit |
| **第 4 点：消息发布与可视化** | **4.1** 逐字段解释随包检测消息；**4.2** 发布全部有效检测和空检测帧；**4.3** 完成框、四点、大小、ID 和坐标轴显示；**4.4** 对照消息、图像和 TF | 完善检测节点、调试绘图与 RViz 配置。检测消息无 NaN，时间戳、frame 和单位一致。确认后 commit |
| **第 6 点：平移跟踪与提前量** | **6.1** 用可控数据理解预测与校正；**6.2** 六维匀速卡尔曼滤波；**6.3** 首次选择、关联与保持锁定；**6.4** 丢检预测、超时删除、重捕获；**6.5** T 秒预测及 marker | 新建 `ft_msgs/Track.msg`、`ft_tracker`、跟踪配置和 launch。验证真实 `red_4`、短漏检、超过 0.5 秒丢失及 ID 生命周期。确认后 commit |
| **第 5 点：参数化与动态调参** | **5.1** 汇总检测、尺寸、队列、滤波、关联和提前量；**5.2** 实现要求的在线参数；**5.3** 对齐 YAML 与动态初始值；**5.4** 做单参数对照实验 | `Detector.cfg`、参数加载与回调、全部 YAML，以及验收记录中的参数实验。重启加载和在线修改均实际生效。确认后 commit |
| **第 7 点：基础跑通标准** | **7.1** 统一基础启动；**7.2** 完善基础说明和验收记录；**7.3** 录制 2～3 分钟演示 | `demo.launch`、可视化入口、README 和验收记录。学习实际运行与观察结果，自动验证脚本不纳入学习或提交。完整展示自写检测、跟踪、TF、marker 和动态调参。确认后 commit |
| **第 8 点：神经网络推理** | **8.1** 验证后端与模型兼容，处理节点顺序；**8.2** 缩放、补边、RGB、归一化与张量；**8.3** 解码分数、编号和四点并去重；**8.4** 还原坐标，接入共同 PnP；**8.5** 模式切换、同画面对比和计时 | 神经网络检测文件、模型准备工具和模型配置。两张样例与至少一段 bag 跑通，记录版本、阈值、耗时和失败案例。确认后 commit |
| **第 9 点：旋转建模与装甲板选择** | **9.1** 解释车体中心、板中心、半径、相位与角速度；**9.2** 验证圆周模型及角度环绕；**9.3** 实现旋转 EKF；**9.4** 多板关联、换面与车体 ID；**9.5** 未来选板、有效标志及线性预测对照 | 新增 `Target.msg`、`ft_selector`、旋转配置和 launch。验证换板、遮挡、失联与其他车辆干扰，说明模型限制。确认后 commit |
| **第 10 点：全链路集成与性能** | **10.1** 统一完整链路和两种模式；**10.2** 分阶段测量平均与 P95；**10.3** rqt_graph 与瓶颈分析；**10.4** 全题回归验收和材料整理 | 完善统一启动、计时统计工具、README、AI 记录及演示。性能范围明确，说明与实际行为一致；临时验证材料检查后清理。确认后 commit |

第 1 点提前准备消息包，第 4 点完成对应发布行为；参数也会随所属算法一起加入，第 5 点负责完整检查与在线调参。文件可以跨模块更新，每次更新都记录对应题号。

## 4．节点、接口与素材适配

| 节点 | 输入 | 输出 |
|---|---|---|
| 检测节点 | 压缩图像、CameraInfo | `/ft/detection`、`/ft/debug/image`、装甲板 TF |
| 平移跟踪节点 | `/ft/detection` | `/ft/track`、跟踪和提前量 marker |
| 旋转与选板节点 | `/ft/detection`；同时间戳的 `/ft/track` 用于对照 | `/ft/target`、车体中心、当前板和预测板 marker |
| 可视化工具 | 调试图、消息、TF、marker | rqt 与 RViz 显示 |

消息约定：

- 检测沿用提供的 `ft_detection/DetectionFrame`，保持字段兼容。来源：本地素材 `task/ft_detection.zip`。
- `Track.msg` 包含 `header、id、tracking、position、velocity、position_predict`。
- `Target.msg` 包含 `header、valid、target_id、position、position_predict`。
- 帧内检测 ID、神经网络编号、跨帧轨迹 ID 分开使用。
- 每帧发布检测结果；无目标时发布空数组，供跟踪节点推进丢失状态。
- 消息和 TF 使用图像时间戳，滤波使用图像时间差；时间倒退重置，耗时使用单调时钟。

素材适配已经明确：

- 更新后的四份原始 bag 共用 `/hk_camera/` 相机入口，话题集中在公共配置；各数据集 YAML 只保存 bag 文件名。图像与 CameraInfo 的 frame_id 按消息原值使用，RViz 自动读取 CameraInfo 选取固定坐标系。
- 模型输入 `[1,3,480,640]`，输出 `[1,21,6300]`；补边值 124，分数不再 sigmoid；模型四点顺序转换为左上、右上、右下、左下。来源：本地素材 `task/nn_model.zip`。
- 模型没有单独的大小输出，大小分类沿用可配置几何规则。
- 原模型保留；派生模型只做必要的节点顺序整理并记录哈希。ONNX 要求节点符合依赖顺序。[ONNX 图结构规范](https://onnx.ai/onnx/repo-docs/IR.html#graphs)
- 基础跟踪采用六维匀速卡尔曼滤波，默认丢失门限 0.5 秒、提前量 0.1 秒。
- 旋转部分先使用配置半径的简化模型，在相机稳定片段验证，明确相机运动等失效情况。
- 历史目标 TF 和参考检测不作为真值；完整验收由原始图像驱动自写检测链路。

## 5．验证、文档和交付

验证按三层进行：

1. **算法检查：**角度边界、四点对应、单位、匀速运动、关联、丢失、时间回退、相位环绕和换板。
2. **真实素材：**红蓝、大小、多目标、反光、遮挡和旋转，保留实际失败案例。
3. **完整链路：**发布者、消息字段、时间戳、TF、marker、参数修改、模式切换和性能统计。

文档分工：

- `file_index.md`：每个文件的用途、所属题号和调用位置。
- `design.md`：数据流、消息、坐标轴、四点尺寸、模型协议和运动假设。
- `acceptance.md`：原题第 1～10 点的要求、操作、结果和提交记录。
- `learning_log.md`：项目相关的实际学习问答、本人回答及必要纠正。
- `ai_usage.md`：关键 AI 辅助范围和本人项目决策。

全部模块完成后，核对源码、演示和文档，准备最终 tag 与对应 zip。远程推送、评审访问及邮件发送按后续明确授权执行。
