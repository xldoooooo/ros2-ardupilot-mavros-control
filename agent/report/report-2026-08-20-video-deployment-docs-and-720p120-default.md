# 视频部署文档与 720p120 默认值调整报告

- 日期：2026-08-20
- 真机：Jetson Orin NX，Ubuntu 24.04 / ROS 2 Jazzy / aarch64
- 飞机地址：`nvidia@192.168.112.169`
- 安全边界：全程未解锁、未起飞，最终 `armed=false`

## 完成内容

1. 根 `README.md` 增加新飞机部署索引，修正当前 Jazzy、工作区路径、串口权限和飞控/视频独立
   systemd 边界；详细步骤分别指向机载部署指导与视频 README。
2. `video_service/README.md` 增加经过当前 Jetson 验证的完整部署流程：
   - 原生 sparse checkout 与构建；
   - FFmpeg、v4l-utils、用户组和 V4L2 模式检查；
   - MediaMTX v1.20.0 ARM64 官方归档地址与 SHA-256 校验；
   - `/etc/ros2-ardupilot` 配置、`/home/share` 权限；
   - 可替换占位符的独立 systemd unit 安装；
   - 未武装 ROS/RTSP/资源释放验收与独立更新回退方式。
3. 文档明确飞机虽同步完整 `video_service/`，磁盘上包含 Qt 面板源文件，但机载 unit 只运行
   `onboard_video_node.py`。机载导入共享后端，不导入 PySide6 或面板，也不创建窗口。
4. `CameraConfig`、机载 INI fallback、面板 fallback 与示例 `camera.conf` 的默认值统一改为：

   ```text
   codec=h264
   width=1280
   height=720
   fps=120
   container=mp4
   ```

   现有地面端用户 JSON 配置仍按持久化值运行，不被源码默认值静默覆盖。当前飞机
   `/etc/ros2-ardupilot/camera.conf` 已同步，MediaMTX 路径继续使用 ARM64
   `/usr/local/bin/mediamtx`。
5. `video-service.service.example` 现显式要求替换用户、home 与工作区三个占位符，避免新飞机只改
   `User=` 却遗漏 `ONBOARD_WORKSPACE`。

## 飞机 720p120 实测

部署前复核：FCU connected、`armed=false`、STABILIZE，视频 stopped。只开启独立视频链后：

```text
RTSP: rtsp://192.168.112.169:8554/camera
codec: H.264 High
resolution: 1280x720
r_frame_rate: 120/1
avg_frame_rate: 120/1
ground decode: 600 frames, 5.05 s media time, success
recording: 28.1 s, 73,466,260 bytes, MP4
```

关闭后录像正常封装并移至：

```text
/home/nvidia/task22_5-bench-artifacts-20260819/video/
```

最终无 FFmpeg/MediaMTX、8554 监听或摄像头占用；视频为 stopped，飞控仍 connected、
`armed=false`、STABILIZE。

## H.264 与 MJPEG 结论

- 真实光学运动模糊主要由曝光时间、运动速度和成像几何决定；同一曝光下，H.264 与 MJPEG 不会
  产生不同的传感器积分模糊。
- MJPEG 每帧独立编码，通常更适合逐帧抓拍，快速运动时不会出现跨帧预测残留，但带宽和存储
  成本很高，仍可能有 JPEG 块效应和振铃。
- H.264 帧间压缩显著节省带宽和存储，适合飞机持续 720p120 推流/录像；码率不足或场景突变时，
  运动区域可能出现压缩涂抹、块效应或短时预测残留。
- 当前默认保留 H.264。若未来以单帧取证清晰度优先，必须用同一运动标靶、同一曝光同时做
  H.264/MJPEG A/B；本轮没有把静态场景包装成清晰度定量排名。

原理依据：

- Basler 曝光与运动模糊：<https://docs.baslerweb.com/optimizing-image-quality>
- RFC 2435 Motion-JPEG 独立帧：<https://www.rfc-editor.org/rfc/rfc2435.html>
- RFC 6184 H.264 IDR 与帧间预测：<https://www.rfc-editor.org/rfc/rfc6184.html>

## 回归结果

```text
tests/: 161 passed in 40.52 s
targeted camera/deploy: 42 passed
colcon: 19 tests, 0 errors, 0 failures, 0 skipped
modified production Python flake8 F/I: passed
bash syntax: passed
git diff --check: passed
```

没有 reset、clean、commit、pull 或 push；用户既有 dirty worktree 保持不变。
