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
- 标准 RTP/JPEG 的宽和高最多表示 2040 像素，因此面板会隐藏 MJPEG 2560、3840
  等无法可靠通过 RTSP 传输的模式；MJPEG 1080p 及以下不受影响。高于该尺寸只能
  使用摄像头原生 H.264 等其他 RTSP 可承载编码，不能用零转码 MJPEG 路线。
- RTSP 固定使用 TCP；MediaMTX 只允许本机发布，局域网客户端可以读取所配置
  的 `rtsp://IP:端口/路径`。

运行依赖为系统 `ffmpeg`、`ffprobe`、`v4l2-ctl` 和项目现有 PySide6。MediaMTX
Linux amd64 可执行文件已经随目录固定版本提供，不需要安装额外 Python 包或
引入 ROS、GStreamer、OpenCV、Web 服务与数据库。

## Ubuntu 24.04 部署

摄像头服务本身是 Python 程序，不需要 CMake、`colcon build` 或其他编译步骤，
也不依赖 ROS 2。克隆或更新仓库后，先从项目根目录安装系统依赖：

```bash
sudo apt update
sudo apt install -y ffmpeg v4l-utils python3-venv
```

后端只使用 Python 标准库，不需要安装额外 Python 包。按照本项目约定创建独立
环境后，即可在一个终端启动服务：

```bash
python3 -m venv .venv
./.venv/bin/python video-service/camera_service.py serve
```

如果需要独立摄像头面板或地面站内的实时预览，还需安装 PySide6。运行完整
地面站时直接安装项目现有依赖；只使用摄像头面板时可以仅安装 PySide6：

```bash
# 完整地面站
./.venv/bin/python -m pip install -r requirements-gui.txt

# 或者：只运行独立摄像头面板
./.venv/bin/python -m pip install 'PySide6>=6.7,<7'
./.venv/bin/python video-service/camera_panel.py
```

若项目已有 `.venv`，不要重复创建，直接补装依赖即可。完整地面站和 ROS 功能
仍需按项目要求构建 ROS 工作空间；这不是摄像头服务自身的运行条件。

### CPU 架构

仓库内的 `video-service/bin/mediamtx/mediamtx` 是 Linux x86-64 静态可执行文件。
先确认目标机器架构：

```bash
uname -m
file video-service/bin/mediamtx/mediamtx
```

- 输出 `x86_64` 时可以直接使用仓库内文件，无需另外安装 MediaMTX。
- 输出 `aarch64` 或 `arm64` 时，必须下载相同版本的 Linux ARM64 MediaMTX，
  替换上述可执行文件并执行
  `chmod +x video-service/bin/mediamtx/mediamtx`；Python 代码无需修改或编译。

### 首次运行检查

插入摄像头后先确认 V4L2 设备和可用模式：

```bash
v4l2-ctl --list-devices
v4l2-ctl --list-formats-ext -d /dev/video0
ffmpeg -version
ffprobe -version
```

实际设备可能是 `/dev/video1`，也可能位于稳定的
`/dev/v4l/by-id/...-video-index0` 路径；以配置面板“检测设备”的结果为准。
如果打开设备时报权限不足，可将当前用户加入 `video` 组，然后注销并重新登录：

```bash
sudo usermod -aG video "$USER"
```

新机器不会自动继承另一台机器的用户配置。首次打开面板后，应重新选择设备、
分辨率、帧率、编码和保存目录。配置保存在
`~/.config/ros2-ardupilot-camera/config.json`；默认录像和截图目录会自动创建。
对已发现 H.264 输出不稳定的 Wasintek 摄像头，优先选择摄像头原生 MJPEG 模式，
例如 `1920x1080@30` 或 `1280x720@120`。

RTSP 默认监听 TCP 8554 端口。启动后可用面板或命令行 `probe` 验证：

```bash
./.venv/bin/python video-service/camera_service.py probe
```

如果目标机器启用了 UFW，并且需要其他局域网设备拉流，再开放对应 TCP 端口：

```bash
sudo ufw allow 8554/tcp
```

局域网播放器使用面板显示的完整地址，例如
`rtsp://192.168.1.10:8554/camera`。RTSP 固定走 TCP，因此客户端也应优先选择
RTSP-over-TCP。
