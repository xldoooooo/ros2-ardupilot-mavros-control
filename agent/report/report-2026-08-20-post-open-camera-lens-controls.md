# 摄像头开启后应用镜头参数修复简报

## 问题与结论

Wasintek 摄像头在视频采集进程打开设备时会重置或忽略此前写入的曝光和增益。旧逻辑在
FFmpeg 打开设备前批量设置 `lens.conf`，因此命令虽成功，实际采集仍可能恢复成较长曝光，
表现为画面更亮且运动模糊。

本次把编码、分辨率、帧率等采集配置保留在 FFmpeg 打开设备的原有阶段；镜头控制则改为
RTSP 被 FFprobe 确认可读后等待 1 秒，再按顺序逐项写入。默认值为手动曝光模式
`auto_exposure=1`、曝光时间 `25`、增益 `200`。

## 实现内容

- 删除启动前的镜头参数写入，在 FFmpeg 和 RTSP 均已就绪后调用；
- 流就绪后固定等待 1 秒，确认采集进程仍存活；
- 每个 V4L2 控制使用独立 `v4l2-ctl --set-ctrl` 命令，自动曝光和曝光时间写入后各等待
  200 ms，复现人工验证成功的硬件时序；
- 写入结束后一次读取全部控制值并逐项校验；超时、驱动拒绝、读回不一致或采集进程提前退出
  均使本次视频启动失败并清理视频进程，但不会影响飞控；
- 更新视频 README、默认镜头配置注释和单元测试。

## 真实摄像头验证

验证设备：`/dev/v4l/by-id/usb-Wasintek_Wasintek_camera_00.00.01-video-index0`。
测试前故意把曝光设为约 78、增益设为 40，随后使用当前保存的 H.264 1920×1080@60 配置启动
本机摄像头服务。启动返回后读回结果为：

```text
gain: 200
auto_exposure: 1 (Manual Mode)
exposure_time_absolute: 25
```

FFprobe 从 `rtsp://127.0.0.1:8554/camera` 实测得到 H.264、1920×1080、60/1。录像正常封装为
`/home/nvidia/Videos/ros2-ardupilot-camera/recording-20260820-202755-123311.mp4`，时长约
15.62 秒。停止后 FFmpeg、MediaMTX、8554 端口和 `/dev/video0`、`/dev/video1` 均无残留占用。

由于用户指定在摄像头打开并稳定 1 秒后才写镜头参数，自动录像开头约 1 秒可能保留设备刚
打开时的默认曝光；随后录制和推流使用已校验的 25/200。

## 自动测试

- `pytest -q tests/test_camera_service.py`：29 passed；
- source ROS 2 Jazzy 与工作区 overlay 后，`pytest -q tests`：165 passed；
- `git diff --check`：通过。

本次未执行任何真机解锁或起飞操作。
