# USB 摄像头 RTSP、录像、截图与地面站集成执行报告

日期：2026-08-17

## 结果摘要

已在 `video-service/` 下完成一套可脱离地面站运行的单 USB 摄像头服务，并在当前
Wasintek UVC 摄像头上完成实机验证：

- 支持摄像头原生 H.264 或 MJPEG，真实枚举分辨率/帧率，包括
  1920×1080@30 与 1280×720@120；
- 必选 RTSP 使用 MediaMTX 对局域网发布，客户端从
  `rtsp://IP:端口/路径` 拉流；
- 同一个 FFmpeg 进程只打开一次摄像头，以 tee 同时发布 RTSP 并零转码封装
  MP4、MKV 或 AVI；
- IPC 命令和 `SIGUSR1` 均可从本机 RTSP 流保存 JPG，不会第二次打开摄像头；
- 新增独立 PySide6 摄像头面板，包含 RTSP 实时预览、设备、IP、端口、路径、
  原生编码、真实画质组合、录像格式、视频/图片路径、启停、截图和运行指标；
- 地面站右上角新增“摄像头配置面板”按钮。地面站只分离启动该窗口，不拥有
  摄像头进程；关闭面板或地面站不停止正在进行的推流/录像。

没有连接、修改或控制飞控，没有解锁、起飞或发送任何飞行命令。

## 实现结构与边界

链路保持为最小的三个运行角色：

1. `camera_service.py` 是独立状态机和当前用户 Unix Socket 控制端；
2. FFmpeg 独占一个 V4L2 采集节点，原码复制到 RTSP 和录像两个输出；
3. MediaMTX 是局域网多客户端 RTSP 服务端，只允许回环地址发布、允许局域网读取。

面板通过换行 JSON 的本地 Unix Socket 调用服务，视频数据不经过 Python IPC。
地面站、ROS、仿真初始化器和飞控清理器都不管理摄像头后台。摄像头配置保存在
`~/.config/ros2-ardupilot-camera/config.json`；Socket/PID 使用 XDG 运行目录或
当前用户私有 `/tmp` 目录；外部进程日志保存在
`~/.local/state/ros2-ardupilot-camera/`。

实现没有引入 GStreamer、OpenCV、Web 后端、数据库或新的 Python 依赖。系统仍只需
FFmpeg/FFprobe、`v4l2-ctl` 和项目已有 PySide6。MediaMTX 是必要的 RTSP 服务端：
FFmpeg 负责采集、复制和封装，MediaMTX 负责稳定监听端口以及同时服务多个拉流客户端。

## MediaMTX 更新

通过官方 GitHub 最新发行版接口确认并更新为
[MediaMTX v1.20.0](https://github.com/bluenviron/mediamtx/releases/tag/v1.20.0)，
发行日期为 2026-08-05。生产副本位于 `video-service/bin/mediamtx/`，旧
`video-service/demo/` 未修改。

- 官方 Linux amd64 压缩包 SHA-256：
  `952d5f7d31d1b448ab4da4509550594c511d42636db9d7bb175d377f4ede81df`
- 随项目保存的可执行文件 SHA-256：
  `25947caac403f37ec881c9be213af2cad67e344a6c7098905b0d31c17f40e336`
- 本机执行 `mediamtx --version`：`v1.20.0`
- 生成的最小配置已由该二进制真实启动验证：只开启 RTSP/TCP，关闭 RTMP、
  HLS、WebRTC、SRT、MoQ、API、metrics、pprof 和 playback。

## H.264 时间戳问题结论

用户提醒的问题在原始采集层确实有可观察迹象：该摄像头原生 H.264 的 USB 到达时间
呈成对/突发分布，直接依赖设备时间戳时还会出现初始未设置或非单调时间戳警告。码流
内容和帧数本身没有损坏。

既定 H.264 路线没有改变。生产命令仍为摄像头原生 H.264 零转码，只增加 FFmpeg
`setts=pts=N:dts=N:duration=1:time_base=1/FPS`，按所选帧率为每个实际收到的包重建
单调固定帧率时间轴。实测结果：

- 1080p30 MP4：2246 帧、74.866667 秒、平均和声明帧率均为 30/1，约
  20.53 Mbps；相邻 PTS 只出现 33.333/33.334 ms；
- 720p120 MP4：2180 帧、18.166667 秒、平均和声明帧率均为 120/1，约
  20.97 Mbps；相邻 PTS 只出现 8.333/8.334 ms；
- H.264 的 MKV 与 AVI 也分别完成 3.6 秒真实封装，均为 30/1；
- H.264 服务日志没有丢帧、非单调时间戳或解码错误。

因此保留 H.264 为默认、MP4 为推荐。这个修正不解码、不重编码，不增加有损处理；
面板同时显示由媒体进度与墙钟采样计算的吞吐帧率，以便发现摄像头实际供帧不足。

## 实机功能验证

当前摄像头为 `2aad:6373` Wasintek UVC，稳定设备路径为
`/dev/v4l/by-id/usb-Wasintek_Wasintek_camera_00.00.01-video-index0`，USB 链路
为 480 Mbps。`v4l2-ctl` 解析得到 23 个 H.264/MJPEG 离散模式，目标四种组合均
存在。

### RTSP、录像与帧率

| 编码/画质 | 保存格式 | 实测录像 | 结果 |
| --- | --- | --- | --- |
| H.264 1080p30 | MP4 | 2246 帧 / 74.866667 s | 精确 30 fps，192.2 MB |
| H.264 720p120 | MP4 | 2180 帧 / 18.166667 s | 精确 120 fps，47.7 MB |
| MJPEG 1080p30 | MP4 | 390 帧 / 13.000000 s | 精确 30 fps，93.4 MB |
| MJPEG 720p120 | MKV | 427 帧 / 3.558 s | 约 120 fps，57.0 MB |
| MJPEG 720p120 | AVI | 403 帧 / 3.358333 s | 精确 120 fps，54.0 MB |

局域网地址 `rtsp://192.168.112.176:18554/camera` 真实拉流通过。FFmpeg 客户端
在 1080p30 和 720p120 窗口分别完成 143 帧和 593 帧解码，`dup_frames=0`、
`drop_frames=0`。Qt 面板的实际 `QVideoSink` 在 720p120 流上 5 秒收到 556 个
有效 1280×720 视频帧；面板关闭后后台仍处于运行态。安全停止并封装一次用时约
0.09 秒。

Qt FFmpeg 后端会先尝试 UDP，因服务固定为 TCP 而收到一次 `461 Unsupported
Transport`，随后自动回退 TCP 并正常播放。这是控制台级兼容探测，不会让面板或
推流进入错误态，未因此增加 UDP 传输面。

### JPG 与信号

- IPC `snapshot` 保存 1920×1080 baseline JPG 成功；
- 向服务 PID 发送 `SIGUSR1` 后保存第二张 1920×1080 JPG 成功；
- 图片分别为 264446 与 275838 字节，已目视确认内容正常；
- 信号处理器只设置事件，截图在普通线程执行；连续信号会合并。

### MJPEG 说明

MJPEG 的 RTSP、MP4、MKV、AVI 和截图均成功。该摄像头 JPEG APP 扩展会让 FFmpeg
偶发输出 `unable to decode APP fields`，本轮所有帧仍能计数和解码，没有丢帧或
输出失败。MJPEG MP4 可用，但 AVI 对传统 MJPEG 播放器兼容性更直接。

MJPEG 文件显著大于 H.264。当前画面下 720p120 AVI 约 129 Mbps，半小时可能接近
29 GB；H.264 720p120 约 21 Mbps，半小时约 4.7 GB。生产使用前应按所选编码检查
保存盘余量。

## GUI 与独立性

- 面板使用与地面站相同的浅色工程配色、边框、圆角、状态色和字体；已在停止态和
  运行态完成 1180×780 视觉检查；
- 真实 V4L2 模式而非硬编码画质列表；切换 H.264/MJPEG 后只展示设备实际支持项；
- 运行时锁定会影响采集链的配置控件，但预览重连、截图、关闭摄像头保持可用；
- 配置保存不自动启动或重启摄像头；窗口打开也不自动占用摄像头；
- 多个面板共享同一后台，摄像头仍只被一个 FFmpeg 采集进程占用；
- 地面站按钮通过 `QProcess.startDetached` 启动面板，地面站退出清理不会触碰它。

## 自动化与静态验证

- 新增摄像头配置、V4L2 解析、主路由 IP、三种封装命令、时间戳、MediaMTX
  配置、Unix Socket IPC、面板生命周期和地面站分离启动测试；
- 摄像头相关定向测试：13 passed；
- 首轮未 source 本地 ROS overlay 的全量测试为 116 passed、3 failed，三项均因
  `guided_interfaces` 无法导入；没有把该环境失败记为通过；
- source `/opt/ros/jazzy/setup.bash` 与 `install/setup.bash` 后正式全量结果：
  119 passed；
- `ground_station.py --check-environment`：通过；
- `py_compile`、致命 Flake8（E9/F63/F7/F82）、88 字符检查和 `git diff --check`：
  全部通过。

## 未做事项与边界

- 本轮最长连续录像约 75 秒，没有执行接近 30 分钟的满时长浸泡测试；已使用分片
  MP4 降低异常退出时整段文件不可读的风险；
- 当前随项目提供的是 Linux amd64 MediaMTX；其他架构需替换相同版本对应资产；
- 需求未包含音频，本实现只处理视频；摄像头的 USB 音频描述符警告不进入视频链；
- 没有设置系统开机自启。面板会按需启动后台，也可通过 CLI 独立启动；
- 所有本轮大体积临时录像、临时下载和测试后台均在验证结束后清理，8554/18554
  最终无监听；正式代码、测试、报告和版本元数据保留。
