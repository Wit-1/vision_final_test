# 视觉组最终考核：装甲板自瞄感知链路

当前实现第 1 点图像接入及第 2 点传统装甲板识别：读取原始 bag 的压缩图像和 CameraInfo，通过 HSV、轮廓和灯条配对识别红蓝、大小及多个候选，并发布标注图和两个颜色掩膜。第 2 点已完成分步讲解、理解问答与桌面显示验收。第 3 点已搭建 PnP、跨帧选解、重投影坐标轴与 TF 接入，定义一的 W/L 按本人试验要求暂定，实际尺寸口径仍待复核；检测消息、跟踪和进阶功能按 [PLAN.md](PLAN.md) 继续实现。

本项目是独立 catkin 工作空间和 Git 仓库，基于 Ubuntu 20.04、ROS Noetic 和 OpenCV 4.x 开发。[验收记录](docs/acceptance.md)记录当前实现的检查依据。

## 环境与构建

准备好 Ubuntu 20.04 和 ROS Noetic 环境后，在项目根目录执行：

```bash
./run.sh check
./run.sh build
./run.sh test
```

`run.sh` 会清理旧工作空间的搜索路径，从 `/opt/ros/noetic` 加载基础环境，再加载本项目的构建结果。构建输出保存在本项目内。首次在嵌套目录构建时，脚本会建立明确的工作空间标记，避免 catkin 向上找到父工作空间。

主要依赖为 catkin tools、roscpp、sensor_msgs、cv_bridge、image_transport、compressed_image_transport、OpenCV 4.x、roslaunch、rosbag、rqt_image_view、tf2、tf2_ros、RViz 和 Python YAML。几何测试另需 gtest。项目已在 OpenCV 4.2.0 上验证；原生或容器化的 Noetic 环境均需提供这些依赖。

## 两个终端运行

两个终端都在已配置 ROS Noetic 的环境中，从同一项目根目录操作，并使用相同的数据集名称。

终端一启动节点和 rqt：

```bash
./run.sh launch dataset:=red_4
```

终端二回放素材：

```bash
./run.sh play red_4 --pause -r 0.2
```

回放最初处于暂停状态，在回放终端按空格开始。rqt 会自动选择节点配置的调试图话题。按空格可暂停或继续，Ctrl+C 结束对应终端的程序。

`-r 0.2` 用于方便观察。需要原速时去掉这一项，需要循环时加入 `-l`。没有桌面窗口时使用：

```bash
./run.sh launch dataset:=red_4 gui:=false
```

数据集名称与素材对应：

| 数据集参数 | 本地素材 | 相机话题前缀 |
|---|---|---|
| `blue_1` | `task/blue_1.bag` | `/hk_camera/` |
| `blue_guard` | `task/blue_guard.bag` | `/hk_camera/` |
| `red_1_3` | `task/red_1&3.bag` | `/hk_camera/` |
| `red_4` | `task/red_4.bag` | `/hk_camera_ca/` |

切换数据集时，停止旧回放和启动进程，再在两个终端使用新的数据集名称。图像接入代码保持不变。

## 参数与数据流

- [detector.yaml](src/ft_bringup/config/detector.yaml)：公共话题、压缩 transport、三个队列长度，以及第 2 点所有 HSV 和几何参数；队列默认均为 1。
- [datasets](src/ft_bringup/config/datasets)：每份素材的 bag 文件名、图像基础话题和 CameraInfo 话题。
- [demo.launch](src/ft_bringup/launch/demo.launch)：设置 `/use_sim_time`，启动检测节点，并按 `gui` 参数启动 rqt。
- [detector.launch](src/ft_bringup/launch/detector.launch)：先加载公共 YAML，再加载数据集 YAML，后者覆盖相同键。

参数放在 `/ft_detector` 私有命名空间。修改 YAML 后重新启动；识别参数在线调节将在第 5 点加入。

图像路径：

```text
基础图像话题 + compressed transport
  → /compressed 消息
  → 插件解码为 sensor_msgs/Image
  → cv_bridge::toCvShare 获取 cv::Mat
  → TraditionalDetector：HSV → 轮廓 → 灯条 → 配对
  → PnpSolver：端点几何与匹配标定 → 合格位姿 / TF
  → drawDetections / drawPoses：复制后画框及投影轴
  → /ft/debug/image（bgr8）和 /ft/debug/mask/blue、red（mono8）
```

CameraInfo 独立订阅，缓存完整 K、D、尺寸和 frame_id。标注图和两个掩膜均使用输入图像的时间戳和坐标系。二维识别不依赖标定；PnP 需要匹配的标定和正数端点尺寸，未确认尺寸须显式开启临时模型模式。压缩解码和生成输出消息需要分配数据，编码匹配时 `toCvShare` 共享解码后的像素。

`run.sh play` 默认只回放图像、CameraInfo 和 `/clock`；历史目标 TF 和参考检测不参与当前链路。

## 传统检测约定与调试

检测和绘图位于 `ft_detector_core`，节点与测试调用同一份算法库。流程为 `detect → findLights → pairLights → drawDetections`。验证范围见[验收记录](docs/acceptance.md)，检测规则和已知限制见下文。

在 rqt 话题框选择：

- `/ft/debug/image`：识别框、四个灯条端点、帧内 ID、RED/BLUE 和 LARGE/SMALL。
- `/ft/debug/mask/blue`、`/ft/debug/mask/red`：颜色筛选后的单通道掩膜。

将 YAML 的 `debug_details` 改为 true 并重新启动，可显示灯条下标和拒绝原因；看完恢复 false。参数当前在启动时读取，在线动态调参在第 5 点实现。

`./run.sh test` 运行 59 个独立用例（传统检测 20、PnP 30、姿态历史 9）；测试代码调用与节点相同的检测库，无需 ROS master。原始 bag 的使用方法与上面的两个终端命令一致。

候选四点定义为左灯上端、右灯上端、右灯下端、左灯下端，单位是原图像像素；图像 x 向右、y 向下。灯条长轴统一向下，再计算上下端点。它们不是板本体四角，后续 PnP 的物体点尺寸必须与这些端点对应，不能直接套用板本体尺寸。

大小按灯条间距与平均灯长的比值分类，分界由 YAML 配置，强透视可能判错。ID 逐帧编号，不是板上数字或跨帧轨迹 ID；传统候选的 confidence 只是 `1/(1+cost)` 几何评分，不是正确率。当前阶段不发布未解算的 pose。

面积门限单位为像素平方，长度和边缘距离为像素，角度为度，其余几何门限为无量纲比例或代价。配对代价为长度不一致度、归一化中心高度差和归一化角度差之和；每根灯条分别记录最低配对代价，任意一端存在明显更好的配对即拒绝当前组，共享配对代价过近时放弃歧义组。

不通过闭运算跨越遮挡补断条，拒绝边缘截断、明显长度不匹配、跨过第三灯条等候选；两条同时变短或很亮且形状相似的倒影仍可能误检。绘图先 clone 输入像素，保持共享原图及颜色掩膜不被标注污染。

## 位姿、TF 与 RViz

正式的点对应、右手坐标轴、单位、相机模型、变换方向与失败处理见 [设计约定](docs/design.md)。

当前保留定义一，使用以下 W/L。尺寸来源为 Alliance2026 开源项目（据本人于 2026-11-02 提供），暂用于本素材：

```yaml
pnp:
  allow_provisional_dimensions: true
  small: {width_m: 0.135, height_m: 0.056, confirmed: false}
  large: {width_m: 0.230, height_m: 0.056, confirmed: false}
```

这是明确采用的临时模型，尚未核实为灯条端点的真实尺寸。`allow_provisional_dimensions` 允许正数临时尺寸进入求解，`confirmed` 继续为 false。几何检查合格时绘制坐标轴并发布 TF，调试图显示 `PnP: provisional`，位姿标签也标记 provisional。

临时开关不会绕过标定、四点、深度或重投影检查，也不会允许零尺寸。关闭该开关后，未确认类别恢复为 dimensions_unconfirmed。获得可靠的端点尺寸后，将对应类别 W/L 从毫米除以 1000 填入，核实后将 confirmed 设为 true，并关闭临时开关；修改参数后重启节点。

当前可检查实际图像驱动的软件链路，但真实尺度准确性仍待后续验证或调整。

本人回放后反馈末尾静止段仍有方向跳变。当前已接入跨帧选解：在原有物理检查和 3 px 门限内，两个解误差接近时优先选择接近上一帧姿态的分支。颜色、大小及四端点位置必须形成唯一的一对一匹配；帧内 ID 不参与匹配。丢失、匹配不明确、标定变化或时间异常后重新初始化。

`pnp/continuity` 中 enabled 控制开关，error_margin_px 默认 1.0，max_gap_s 为 0.1，max_corner_shift_ratio 为 0.5；修改后重启。关闭 enabled 可恢复原单帧选择，用于对照。图中 (prior) 表示本帧参考历史选中了误差略大的合格分支；(provisional) 继续表示尺寸临时。

修改后的 C++ 库及实际 ROS 节点在 red_4 最后三秒均未出现超过 30° 的相邻跳变，最大仍约 6.99°。这是局部连续性结果，初始分支不保证是真实朝向。具体条件见[验收记录](docs/acceptance.md)。

坐标轴颜色为 X 红、Y 绿、Z 蓝；Z 标签是相机前向深度，e 标签是四角重投影 RMS。默认 TF 为 `CameraInfo.frame_id → armor_<帧内ID>`，平移单位米，stamp 使用图像时间戳。帧内 ID 不保证跨帧身份，旧 TF 不能当作当前检测。

启用 RViz 的方式：

```bash
./run.sh launch dataset:=blue_1 rviz:=true
# red_4 的光学坐标系名字不同，需要同步设置查看器：
./run.sh launch dataset:=red_4 rviz:=true rviz_fixed_frame:=hk_camera_ca
```

以上是两种独立启动示例，先停止上一组程序再切换。`gui:=false rviz:=true` 可只打开 RViz。rviz_fixed_frame 仅改变查看器，不改变节点读取的标定或 TF 父坐标系；不允许临时尺寸且尺寸未确认时，没有 armor TF 是预期行为。

合成测试验证软件实现和显示，不代替原始素材的实际尺寸与距离验收。实验脚本、截图和中间日志已清理；正式测试保留在 `src/ft_detector/test/`。

## 脚本背后的命令

在项目根目录的 ROS Noetic 终端中，启动命令等价于：

```bash
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch ft_bringup demo.launch dataset:=red_4
```

`red_4` 回放命令等价于：

```bash
rosbag play --clock task/red_4.bag \
  --topics /hk_camera_ca/image_raw/compressed /hk_camera_ca/camera_info \
  --pause -r 0.2
```

如果 rqt 没有画面，先确认回放是否仍暂停、两个终端的数据集是否一致，再看节点是否收到 CameraInfo 和图像。如果提示无法连接显示服务器，检查当前环境的 DISPLAY 和图形显示权限；使用容器时，还需配置图形转发。无图形环境可使用 `gui:=false`。

## 素材与生成文件

原始素材放在 `task/`，保持原件，不进入 Git。`src/ft_detection` 来自随包消息定义，保留原样。

`build/`、`devel/`、`.catkin_tools/`、运行日志、bag、派生模型、视频和打包结果均由忽略规则排除。
