# 独立摄像头服务

该目录提供一条单摄像头链路：FFmpeg 只打开一次 USB 摄像头，将摄像头原生
H.264 或 MJPEG 同时交给 MediaMTX 发布 RTSP，并直接封装为本地录像。截图也从
本机 RTSP 流读取，不会再次占用摄像头。

## 图形界面

地面站右上角的“摄像头配置面板”会启动独立窗口。以下命令均从项目根目录
执行；也可以脱离地面站直接运行：

```bash
./.venv/bin/python video-service/camera_panel.py
```

面板关闭只关闭自身 RTSP 预览，不停止推流或录像。摄像头必须通过面板中的
“关闭摄像头”安全停止；停止后录像从 `.partial.<格式>` 原子改为最终文件名。

## 命令行与信号截图

面板会按需自动拉起后台，也可以手工运行和控制：

```bash
./.venv/bin/python video-service/camera_service.py serve
./.venv/bin/python video-service/camera_service.py status
./.venv/bin/python video-service/camera_service.py probe
./.venv/bin/python video-service/camera_service.py start
./.venv/bin/python video-service/camera_service.py snapshot
./.venv/bin/python video-service/camera_service.py stop
./.venv/bin/python video-service/camera_service.py shutdown
```

`status` 返回 `service_pid`。向该 PID 发送 `SIGUSR1` 会异步保存一张 JPG：

```bash
kill -USR1 <service_pid>
```

连续信号会合并，避免并发解码争抢资源。Socket 和 PID 位于当前用户私有运行
目录；配置默认保存到 `~/.config/ros2-ardupilot-camera/config.json`，外部进程
日志保存到 `~/.local/state/ros2-ardupilot-camera/`。

## 编码与封装

- H.264 是默认路线：摄像头原生码流零转码，并用 FFmpeg `setts` 按选定帧率
  重建稳定时间戳，推荐保存为分片 MP4。
- MJPEG 是兼容回退，同样零转码；MP4、MKV、AVI 均可用，AVI 对传统 MJPEG
  播放器兼容性最好，但文件通常明显更大。
- RTSP 固定使用 TCP；MediaMTX 只允许本机发布，局域网客户端可以读取所配置
  的 `rtsp://IP:端口/路径`。

运行依赖为系统 `ffmpeg`、`ffprobe`、`v4l2-ctl` 和项目现有 PySide6。MediaMTX
Linux amd64 可执行文件已经随目录固定版本提供，不需要安装额外 Python 包或
引入 ROS、GStreamer、OpenCV、Web 服务与数据库。
