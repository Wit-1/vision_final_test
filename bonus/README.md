# 附加题

| 模块 | 内容 |
|---|---|
| [GUI 调试控制台](gui/README.md) | 素材回放、动态调参、检测模式和工作区对比、双 RViz 视角联动 |
| [旋转预测改进](rotation_prediction/README.md) | 在线估计半径与两组高度差、预测板重投影、T 秒后的观测对照 |
| [装甲板识别改进](detection_improvements/README.md) | NN 编号辅助大小板、两种模式的动态 ROI、CPU 线程测量与优化 |

GUI 源码位于 `gui/ft_rqt_gui/`，旋转算法和投影工具位于 `rotation_prediction/`，NN 判型、共享 ROI 策略、传统裁剪适配及独立线程工具位于 `detection_improvements/`。前两个 ROS 包通过 `src/` 中的相对符号链接接入，检测扩展由原检测包编译；构建和运行仍使用项目根目录的 `run.sh`。
