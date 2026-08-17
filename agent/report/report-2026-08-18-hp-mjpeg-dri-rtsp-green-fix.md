# HP MJPEG DRI RTSP 纯绿色修复简报

日期：2026-08-18

## 结论

HP Quanta 5MP 内置摄像头的纯绿色画面不是代码未同步、Linux 驱动或 Qt 控件问题。
摄像头输出的 MJPEG 含 JPEG DRI/restart markers；FFmpeg 6.1/7.1 的 RTP/JPEG 发送端
没有把 restart interval 写入 RTP，导致同一 RTSP 流在系统 FFmpeg 和 Qt FFmpeg 后端中
均解码失败。

服务现会在 MJPEG 启动前读取一帧头部并按设备自动选择路径：

- 不含 DRI：保持原单 FFmpeg、RTSP 与录像双路 stream-copy；
- 含 DRI：录像仍 stream-copy 保存原始码流，只将 RTSP 分支规范化为高质量、RFC 2435
  兼容的 MJPEG；
- H.264：完全保持原零转码路径。

## 代码改动

- `video-service/camera_app/controller.py`
  - 新增 MJPEG DRI 头检测；检测失败会在启动前返回明确错误。
  - 新增只作用于 RTSP 分支的 MJPEG 兼容输出：`yuvj420p`、质量 2、标准 Huffman、
    duplicated quantization matrix。
  - 录像分支继续 `-c:v copy`，并保留原有固定时间戳、分片 MP4/MKV/AVI 行为。
  - 状态新增 `rtsp_mjpeg_normalization`，便于确认当前是否启用兼容分支。
- `tests/test_camera_service.py`
  - 覆盖 DRI 仅在 JPEG 扫描头前识别、压缩数据中的相同字节不会误判。
  - 覆盖兼容路径仍只有一次 V4L2 输入、录像 copy、仅 RTSP 重编码及 RFC 参数。
- `video-service/README.md`
  - 补充 DRI 原因、自动兼容行为及对其他摄像头的影响边界。

## HP 笔记本实测

远端：`scq@192.168.112.101`，Ubuntu 24.04，项目路径
`/home/scq/ros2-ardupilot-mavros-control`。

### 故障复现

- 三个核心文件 SHA-256 与开发机一致，排除代码未同步。
- HP 直接 V4L2 抓帧：1920×1080，画面正常。
- 原始 JPEG：SOF 为 4:2:2，存在 `FF DD 00 04 00 78`，即 DRI=120。
- 原零转码 RTSP：系统 FFmpeg 报 `mjpeg_decode_dc: bad vlc` 并输出纯绿色。
- Qt 7.1.3：得到 1920×1080 YUYV 帧，但五个采样点全部为 RGB `(0,134,0)`。

### 修复验证

- 状态自动显示 `rtsp_mjpeg_normalization: true`。
- Qt 7.1.3 收到 1920×1080、30 fps、NV12 正常彩色帧，无解码错误。
- 服务截图 JPG 目视正常。
- 持续 68 秒：2023 帧，实测约 29.80 fps，速度 1.00x。
- 录像仍为 MJPEG、`yuvj422p`、1920×1080、30/1，证明录像未被兼容转码。
- HP 兼容分支实测 FFmpeg 约 43.2% CPU、410988 KiB RSS；该成本只发生在含 DRI
  且选择 MJPEG 的摄像头上。

### Wasintek 回归

- Wasintek 1920×1080 原始 MJPEG 头不含 DRI。
- 1280×720@120 启动状态为 `rtsp_mjpeg_normalization: false`。
- 仍使用原 `-c:v copy` tee 路径；Qt 得到正常彩色 1280×720 YUYV 帧。
- 持续约36.6秒：4361帧，实测约119.55 fps。
- Wasintek H.264 及所有其他 H.264 摄像头不执行 MJPEG 头检测与兼容转码。

## 自动测试

- 远端摄像头模块：19 passed。
- 本机摄像头模块：19 passed。
- 本机完整 `tests/`：先加载 `/opt/ros/jazzy/setup.bash` 与 `install/setup.bash` 后，
  140 passed。
- 未加载项目 overlay 的首次全量测试有3项因找不到 `guided_interfaces` 失败；加载正确
  工程环境后全部通过，该结果未被隐瞒。
- `git diff --check` 与 Python 语法编译通过。

## 清理与远端状态

- 已删除本轮诊断生成的9段短录像、1张远端测试截图和 `/tmp` 诊断文件，合计释放约
  7.5 GiB；删除不可恢复。用户原有录像未修改。
- 已终止本轮手工测试遗留的暂停态 FFmpeg PID 28464。
- 用户在验证期间自行打开了地面站和摄像头面板并继续切换配置；未强制关闭用户 GUI，
  摄像头后台保持独立可用。
- 远端仓库历史原本为 ahead 2/behind，不执行 merge、rebase 或 reset；仅同步本次实际文件。

## 已知代价与边界

- 含 DRI 的 MJPEG RTSP 分支需要一次软件解码/重编码，并从原始 4:2:2 转为 RFC 2435
  可稳定承载的 4:2:0；原始录像质量不变。
- MJPEG RTP 的2040像素宽高上限仍存在，超限模式继续在面板隐藏并由后端拒绝。
- `461 Unsupported Transport` 是 Qt 先尝试 UDP、被仅开放 TCP 的 MediaMTX 拒绝后自动回退
  TCP 的提示；实际播放成功时不是故障。
