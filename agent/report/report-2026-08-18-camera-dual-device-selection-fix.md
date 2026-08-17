# 摄像头面板双设备切换修复简报

日期：2026-08-18

## 问题与根因

HP Ubuntu 24.04 笔记本同时连接两个 V4L2 摄像头：

- HP 5MP Camera：`/dev/video0`，稳定路径为 Quanta `video-index0`。
- Wasintek camera：`/dev/video4`，稳定路径为 Wasintek `video-index0`。

面板选择 Wasintek 后，后台确实按 Wasintek 读取了模式，并在 `probe` 响应中返回正确的
`selected_device`。但 `_apply_probe()` 重建设备下拉列表时忽略该字段，强制恢复服务端磁盘中
原来保存的 HP 设备。此时画质列表仍属于 Wasintek，最终形成“HP 路径 + Wasintek 模式”的
混合配置，表现为设备自动跳回 HP，启动时被后台正确拒绝为“不支持所选组合”。

## 修复内容

- `video-service/camera_app/panel.py`
  - 设备列表刷新后以本次 `probe.selected_device` 为选择权威，不再用旧持久化配置覆盖用户选择。
  - 若设备在枚举和能力读取之间消失，仍保留实际探测路径，以便显示对应错误而非悄悄切换设备。
  - V4L2 能力探测期间禁用设备、画质、保存与启动控件，避免用户提交上一设备遗留的模式。
  - 探测成功或失败后统一恢复控件门控。
- `tests/test_camera_service.py`
  - 新增 HP 为旧配置、Wasintek 为本次探测设备的双摄像头回归。
  - 断言下拉框、路径显示和最终提交配置均保持 Wasintek，且提交的是同一探测结果中的模式。

## 本机验证

- `pytest -q tests/test_camera_service.py`：15 passed。
- 加载 ROS Jazzy 和本仓库 overlay 后执行 `pytest -q tests`：136 passed。
- `git diff --check`：通过。
- 未加载 overlay 时全量 `tests` 有 3 项因找不到 `guided_interfaces` 失败；加载项目要求的 overlay
  后全部通过，属于测试环境差异，不是本次代码问题。
- 仓库根目录直接收集还会遇到用户现有 `integration/websocket_test_demo` 缺少 `ws_demo` 的独立
  收集错误，因此正式全量范围使用项目 `tests/`。

## HP 笔记本真实验证

- 远端：`scq@192.168.112.101`，Ubuntu 24.04.4、x86-64。
- 远端摄像头模块定向回归：15 passed。
- 使用真实后台 `probe` 数据驱动修复后的 Qt 面板，依次选择 HP、Wasintek：
  - HP 保持 HP 稳定路径，提交 MJPEG 2560×1920@30。
  - Wasintek 保持 Wasintek 稳定路径，提交 MJPEG 1280×720@120。
  - 两次设备路径一致性断言均通过，没有自动跳回。
- Wasintek MJPEG 1280×720@120 + MKV：约38.35秒、4602帧、约118.98 fps；FFprobe
  识别为 MJPEG、1280×720、120/1。
- HP MJPEG 1920×1080@30 + MP4：约34.44秒、1033帧、约29.67 fps；FFprobe识别为
  MJPEG、1920×1080、30/1。
- 两段录像均出现该类摄像头已知的非致命 JPEG APP 字段告警，但流参数可识别且服务正常运行、
  停止和封装。

## 远端收尾

- 删除本次生成的两段明确测试录像：约520 MB MKV 和约274 MB MP4；它们是临时验证数据，
  删除后不可恢复。
- 最终配置恢复为 Wasintek、MJPEG、1280×720@120、MP4，服务状态为 stopped。
- 保留用户原先已运行的 `camera_service.py serve` 待命进程；FFmpeg、MediaMTX 和面板无残留。
- 没有连接飞控、运行 ROS 飞行链路、解锁或起飞。
