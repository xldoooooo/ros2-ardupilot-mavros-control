# HP 高分辨率 MJPEG RTSP 绿屏修复简报

日期：2026-08-18

## 现象

HP Ubuntu 24.04 笔记本选择内置 HP 5MP Camera 的 MJPEG 2560×1920@30 后，面板预览为绿色，
Qt Multimedia 的 FFmpeg 后端持续输出 `Missing packets; dropping frame`。MediaMTX 同期记录
`reader is too slow, discarding ... frames`。

## 根因证据

- 直接从 HP V4L2 节点提取的 2560×1920 MJPEG 帧画面正常。
- 同一次采集零转码保存的 2560×1920 MP4 首帧画面正常，视频约128.09 Mbps。
- 经 RTSP 后，Qt/FFmpeg 把同一视频识别成错误的 `512x1920`，并持续丢弃不完整帧。
- RFC 2435 第3.1.5和3.1.6节规定 RTP/JPEG 宽高为8 bit、以8像素为单位，最大只能表示
  2040像素。2560超过协议字段上限；`2560 / 8` 写入8 bit后回绕，接收端得到
  `(320 mod 256) * 8 = 512`，与实测错误宽度完全一致。
- MediaMTX 官方文档说明 `reader is too slow` 表示读取端出站队列不足，建议把
  `writeQueueSize` 提高到1024。该问题会加重高码率流的 RTP 分片缺失，但提高队列无法解决
  2560本身超出RFC字段上限的问题。

官方依据：

- RFC 2435：<https://datatracker.ietf.org/doc/rfc2435/>
- MediaMTX Decrease packet loss：<https://mediamtx.org/docs/features/decrease-packet-loss>

## 修复内容

- 新增 `RTP_JPEG_MAX_DIMENSION = 2040`，集中表达 RTP/JPEG 的协议边界。
- 后台 `probe` 将宽或高超过2040的MJPEG模式移入 `excluded_modes`，不再提供给画质下拉框；
  H.264等编码不受这一限制。
- 面板明确显示隐藏了多少个超过 RTSP/JPEG 限制的模式。
- 后台 `_preflight()` 再次校验，绕过GUI手工提交高分辨率MJPEG也会在启动 MediaMTX、FFmpeg
  或占用摄像头前收到明确错误。
- 生成的 MediaMTX 配置加入 `writeQueueSize: 1024`，改善合法高码率模式的突发包承载。
- README 补充 RTP/JPEG 2040像素限制和高分辨率替代路线。

## HP 真实验证

- HP 探测结果保留 MJPEG 1920×1080、1280×720和640系列模式，隐藏4个2560宽模式。
- 手工提交 MJPEG 2560×1920@30 被明确拒绝，且没有启动媒体进程。
- 正式服务启动 HP MJPEG 1920×1080@30 + MP4：
  - 10秒 Qt Multimedia/QVideoSink 收到284个有效帧，最终尺寸始终为1920×1080。
  - 媒体进度约29.92 fps。
  - MediaMTX 无 `reader is too slow`、lost 或 discard 记录。
  - 录像为 MJPEG、1920×1080、30/1 fps、11.2秒，可读取。
- Qt 首先尝试被服务禁用的 UDP 时仍会输出一次 `461 Unsupported Transport`，随后自动使用TCP，
  这与绿屏和丢帧无关。

## 自动验证与收尾

- 摄像头模块定向回归：16 passed。
- 加载 ROS Jazzy 和项目 overlay 后完整 `tests/` 回归：137 passed。
- Python compileall、致命级 flake8 和 `git diff --check`：通过。项目完整 flake8 会报告既有的
  docstring、引号和导入风格规则，本任务没有借机重排无关代码。
- HP 最终配置为内置摄像头、MJPEG、1920×1080@30、MP4；状态 stopped，只保留独立后台
  `camera_service.py serve`，无 FFmpeg、MediaMTX 或面板残留。
- 删除本次生成的两段1080p诊断录像以及两张临时诊断JPEG；文件仅用于本次验证，删除后不可恢复。
- 没有运行飞行链路、连接飞控、解锁或起飞。
