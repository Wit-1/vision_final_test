# 独立性能工具

基准按原始 bag 的图像顺序调用主工程算法库，测量单帧处理开销，无需运行 ROS master 或播放 bag。以下命令均在项目根目录、已配置的 Noetic 环境中执行。

| 文件 | 作用 |
|---|---|
| [CMakeLists.txt](CMakeLists.txt) | 独立构建 benchmark_pipeline，链接现有检测、跟踪和选板库 |
| [benchmark_pipeline.cpp](benchmark_pipeline.cpp) | 读取原始图像与 CameraInfo，输出逐帧耗时 CSV |
| [summarize_timings.py](summarize_timings.py) | 按数据集、模式和阶段汇总样本数、平均值与 P95 |

## 构建与运行

先按[项目说明](../README.md#环境与构建)准备依赖和模型，再构建主工程与工具：

```bash
./run.sh build
./run.sh build-tools
mkdir -p runs
```

使用相同片段分别测量两种模式，再合并汇总：

```bash
./run.sh benchmark --dataset red_4 --mode tradition --start 20 --duration 1 > runs/red_4_tradition.csv
./run.sh benchmark --dataset red_4 --mode nn --start 20 --duration 1 > runs/red_4_nn.csv
python3 tools/summarize_timings.py runs/red_4_tradition.csv runs/red_4_nn.csv --output runs/red_4_summary.csv
```

基准的 CSV 写到标准输出，诊断信息写到标准错误。汇总文件须使用新文件名，已有同名文件时拒绝覆盖；不传 --output 则输出到终端。运行数据保存在 Git 忽略的 runs/。

## 参数与配置

| 参数 | 默认值 | 含义 |
|---|---|---|
| --dataset | blue_1 | blue_1、blue_guard、red_1_3 或 red_4 |
| --mode | tradition | tradition 或 nn |
| --start | 0 | 距 bag 起点的秒数，必须非负 |
| --duration | 1 | 处理片段的秒数，必须大于零 |

run.sh 自动传入项目根目录。参数按 detector.yaml→model.yaml→数据集 YAML 合并，跟踪与选板分别读取 tracker.yaml、selector.yaml。基准使用启动配置，不读取在线调参界面的临时值。

片段范围按 bag 记录时间选择；状态更新使用图像时间戳。工具缓存起点之前的最新有效 CameraInfo，跳过重复图像帧；时间回退或坐标系变化时重置算法状态。

## CSV 与统计范围

每行对应一张处理后的图像。dataset、mode、stamp_ns、frame_id 标识输入；candidates、detections、track_id、tracking、target_id、target_valid 记录本帧输出状态。

| 耗时字段 | 测量内容 |
|---|---|
| decode_ms | JPEG 解码 |
| detect_ms | 选中模式的检测；NN 包含预处理、推理、解码与 NMS |
| pose_ms | PnP、历史选解、检测消息构造和两路跟踪观测转换 |
| track_ms | 单板匀速跟踪更新 |
| selector_ms | 旋转跟踪与未来选板 |
| draw_ms | 检测与位姿调试图绘制 |
| total_ms | 从 JPEG 解码开始到调试绘图结束的整个串行调用 |

耗时使用单调时钟，单位 ms，首帧计入。模型及 bag 加载、磁盘读取、CameraInfo 缓存和 CSV 写入位于计时之外。结果表示离线串行算法开销，ROS 通信、排队、在线回调调度和窗口显示不在此范围内。

汇总输出 dataset、mode、stage、samples、mean_ms、p95_ms。P95 使用最近秩法，取升序样本的第 ceil(0.95×N) 项；各阶段分别统计，总 P95 直接计算逐帧 total_ms，不将各阶段 P95 相加。

比较时使用相同素材范围、参数和线程配置，并避免同时运行其他推理任务。离线串行耗时不能直接当作完整在线链路的帧率，阶段数据用于定位主要开销。
