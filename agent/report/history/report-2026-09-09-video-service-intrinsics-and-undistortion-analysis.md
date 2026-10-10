# 视频服务内参归档与直播去畸变算力分析简报

日期：2026-09-09

## 完成情况

- 已新增 `video_service/config/intrinsics.yaml`，保存 Wasintek 下视相机 1920×1080 OpenCV
  `plumb_bob` 内参、5 项畸变系数和 RMS 重投影误差。
- 新文件与 `correction_service/config/intrinsics.yaml` 内容逐字节一致；来源仍为飞机
  `/home/nvidia/camera_calib/camera_calibration.yaml`（2026-08-27）。
- 已在 `video_service/README.md` 记录文件用途，并明确当前视频流水线尚未读取该文件或执行去畸变。
- 按用户暂停指令，本次没有实现直播画面去畸变，也没有修改 FFmpeg 命令或启停真机视频服务。
- 本机 YAML 字段校验通过；`tests/test_camera_service.py` 与 `tests/test_onboard_deploy.py` 共 50 项
  测试全部通过。

## 真机安全状态与测试方法

真机为 Jetson Orin NX、Ubuntu 24.04/Jazzy、aarch64。测试前只读确认：

- MAVROS：`connected=true`、`armed=false`、`guided=false`、`mode=STABILIZE`；
- `video-service.service` 与飞控服务均为 active；
- `/video_service/status`：`running=false`、`state=stopped`。

算力测试只读取真机已有的 H.264 1920×1080、约 60 fps MP4，输出到 FFmpeg `null` muxer；
没有打开摄像头、没有发送 ROS 控制消息、没有解锁或起飞、没有停止或重启任何服务。

## 实测结果

测试使用 FFmpeg `lenscorrection` 的双线性插值代表直播去畸变所需的逐像素几何变换。CPU 核数按
`(user + sys) / 媒体时长` 估算，适合判断持续实时负载量级。

| 路径 | 媒体时长 | 墙钟时间 | user+sys | 折算持续 CPU 占用 |
| --- | ---: | ---: | ---: | ---: |
| 软件 H.264 解码，不校正、不编码 | 10 s | 2.222 s | 9.531 s | 约 0.95 核 |
| 软件解码 + `lenscorrection` 双线性校正 | 15 s | 6.680 s | 36.415 s | 约 2.43 核 |
| NVIDIA 硬件解码 + CPU 双线性校正 | 10 s | 6.927 s | 18.635 s | 约 1.86 核 |
| 软件解码 + 校正 + `libx264 ultrafast` | 5 s | 3.716 s | 19.056 s | 约 3.81 核 |

当前生产链直接复制摄像头原生 H.264，同时送给 RTSP 和录像，不做逐像素处理。加入去畸变后必须
解码；若直播仍输出 H.264，还必须重新编码，因此不能再维持现有低开销的 stream-copy 路径。

## 真机硬件能力边界

- 真机 FFmpeg 8.0.1 提供 `h264_nvv4l2dec`，硬件解码可以工作。
- FFmpeg 虽列出 `h264_nvenc`，实际启动失败：缺少 `libnvidia-encode.so.1`，并报告驱动版本要求；
  `h264_v4l2m2m` 也因找不到有效编码设备而失败。
- GStreamer 存在 `nvv4l2decoder`、`nvvidconv` 和 `nvv4l2h264enc`，说明 Jetson 硬件编解码能力可用，
  但没有现成 `cudawarp` 元件。若继续实现，需另行验证 CPU 校正与 GStreamer 硬件编解码之间的
  内存转换、端到端延迟、60 fps 稳定性和与 MediaMTX/录像分支的集成。

## 结论与未完成项

直播去畸变会消耗显著算力。直接沿用 FFmpeg CPU 路径时，完整实时链约需 3.8 个 CPU 核；即使
使用已可工作的硬件解码，CPU 双线性校正本身仍有明显负担。当前不应在生产视频服务中默认开启。

建议后续单独原型化“保留原始 H.264 录像分支 + 硬件解码 + GPU/CPU 精确 OpenCV 模型去畸变 +
GStreamer 硬件 H.264 编码 + RTSP”链路，并实测 CPU/GPU/内存带宽、温度、延迟、帧率和画质后再
决定是否上线。`videoservice直播画面去畸变` 继续保持未完成，不把本次算力分析当作实现结果。
