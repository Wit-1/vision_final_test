# 验收记录

本文件记录实际功能检查依据及人工确认结果。

## 第 1 点：图像接入与相机参数

实现步骤为 1.1～1.4。自动功能检查通过，rqt 已由本人在桌面确认正常可见。

| 原题要求 | 实现与证据 | 状态 |
|---|---|---|
| 独立 catkin 包与依赖声明 | ft_detector、ft_bringup 和随包 ft_detection，已在独立工作空间构建 | 自动检查通过 |
| 从无缓存目录初始化和构建 | 在本地全新副本运行 run.sh build，明确只扩展 Noetic，原配置哈希不变 | 自动检查通过 |
| 订阅 compressed 图像与 CameraInfo | image_transport compressed 插件与独立 CameraInfo 订阅 | 自动检查通过 |
| 使用 cv_bridge 获取 cv::Mat | 编码匹配时 toCvShare 共享解码后的像素 | 已实现，真实图像检查通过 |
| 读取 K、D 和尺寸，缓存 CameraInfo | 完整消息保存在 camera_info_；1.3 只发送一次 CameraInfo 后继续处理图像 | 自动检查通过 |
| 保留图像时间戳和 frame_id | 对收到的输出逐帧匹配输入并比较像素 | 自动检查通过 |
| 换 bag 无需修改代码 | 四份独立数据集 YAML，经公开 run.sh launch/play 验证 | 自动检查通过 |
| 小订阅队列 | 图像、CameraInfo 和调试图队列均由 YAML 配置，默认 1 | 已核对 |
| rqt 能显示图像 | 实际 ImageView 插件离屏渲染 red_4，正常关闭；本人反馈“rqt正常，可见” | 自动检查与本人确认通过 |

### 四份原始 bag 功能检查

通过 `run.sh launch dataset:=... gui:=false` 与 `run.sh play ... --duration 0.6 -r 0.2` 执行。以下数量为本次短片段实际收到的不同时间戳，不是完整 bag 的消息总数。

| 数据集 | 输入时间戳数 | 输出时间戳数 | 匹配数 | 输出 frame_id |
|---|---:|---:|---:|---|
| blue_1 | 94 | 93 | 93 | camera_optical_frame |
| blue_guard | 65 | 63 | 63 | camera_optical_frame |
| red_1_3 | 49 | 49 | 49 | camera_optical_frame |
| red_4 | 63 | 62 | 62 | hk_camera_ca |

所有收到的输出均为 1440×1080、bgr8，时间戳、frame_id 和解码像素与对应输入一致。小队列允许丢弃积压帧；本次低速回放用于功能检查，不作为实时性能结论。

消息包 MD5 已与四份参考 bag 匹配，原始五个消息包文件保持不变。参考目标结果不用于上述图像接入检查。

### 本地证据与复现

- `runs/2026-11-01-1.3/result.json`：单次标定缓存与图像检查。
- `runs/2026-11-01-1.4/result.json`：四份数据与 rqt 检查。
- `runs/2026-11-01-1.4/rqt-red_4.png`：rqt 实际离屏渲染图。
- `runs/2026-11-01-1.4/public_gui_result.json`：公开 gui:=true 启动链路，真实查看器订阅调试图，启动、回放与结束均通过。
- `runs/2026-11-01-1.4/fresh_bootstrap.json`：从未构建目录初始化成功，未改动原项目配置。
- 验证辅助代码及日志在对应 `runs/` 目录下保留，原始素材与渲染图不进入 Git。

公开运行方式见 [README](../README.md)。本人已反馈“rqt正常，可见”；其余接入行为的自动证据见上表。
