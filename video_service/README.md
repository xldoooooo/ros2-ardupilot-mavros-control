<!-- Independent video service operation, configuration and deployment. -->
# 独立视频服务

`video_service/` 复用一条经过本机摄像头验证的链路：FFmpeg 只打开一次 USB
摄像头，把原生 H.264/MJPEG 同时交给 MediaMTX 发布 RTSP 并封装录像；JPG
从本机 RTSP 读取，不会第二次占用设备。它是独立进程，摄像头、FFmpeg、
MediaMTX 或磁盘失败只进入 `VideoStatus`，不能停止或改变飞控任务。

## 地面去畸变中继

`ground_relay.py` 独立处理一条视频，再向已有 MediaMTX 发布；客户端共用处理结果。
默认 `--backend auto` 按 Intel、NVIDIA、CPU 顺序选择，先使用本地短样本实际探测硬件链路。
指定 `--backend intel|nvidia` 时，该 GPU 不可用也会回退 CPU；`--backend cpu` 可直接使用软件路线。

| 后端 | 处理路线 | 依赖与限制 |
| --- | --- | --- |
| Intel | VAAPI 解码 → OpenCL 去畸变 → VAAPI 编码 | Intel OpenCL ICD、VAAPI 驱动与共享表面支持；保留原有低拷贝路线 |
| NVIDIA | NVDEC 解码 → OpenCL 去畸变 → NVENC 编码 | NVIDIA 驱动提供 CUDA、OpenCL、NVENC；FFmpeg 支持对应接口；含 GPU/主存搬运，不是零拷贝 CUDA 滤镜 |
| CPU | 软件解码 → OpenCV 去畸变 → libx264 编码 | 项目 Python 的 PyAV、OpenCV、NumPy、PyYAML；无需 GPU 驱动 |

```bash
# setup_ground_station.sh 也会安装此依赖清单；现有地面站升级时执行：
./.venv/bin/python -m pip install -r requirements-gui.txt

./.venv/bin/python video_service/ground_relay.py \
  --input rtsp://127.0.0.1:8554/camera-raw \
  --output rtsp://127.0.0.1:8554/camera-rectified
```

Ubuntu 24.04 的 Intel 后端需 `intel-opencl-icd` 和可访问的 `/dev/dri/renderD128`。
启动时优先使用系统/当前环境；如果探测失败，会自动尝试用户目录
`~/.local/share/ros2-ardupilot-video/intel-opencl`（设置了 `XDG_DATA_HOME` 时跟随该目录）。
这里存放已解包的 Intel OpenCL 及依赖，保留 `usr/lib/<架构>/intel-opencl/libigdrcl.so` 布局；
也可用 `--intel-opencl-runtime <目录>` 指定。程序自动为 Intel 子进程设置库路径和 ICD，
无需手工 export，不改变 NVIDIA/CPU 的环境。mini-ground 已在此固定目录部署并验证自动加载。
新机器可优先通过系统包安装 `intel-opencl-icd`；自动发现不会自行下载或安装驱动。
NVIDIA 后端需匹配显卡的 NVIDIA 驱动及其 OpenCL ICD，无需为该路线安装 CUDA Toolkit、CuPy。
CPU 依赖中的 OpenCV/NumPy/PyYAML沿用项目环境；缺失时安装系统
`python3-opencv python3-numpy python3-yaml`，项目 `.venv` 需能访问系统包。
媒体工具 `ffmpeg`、`ffprobe` 和已配置允许发布的 MediaMTX 须提前就绪。

默认标定为 `config/video/Wasintek/intrinsics.yaml`：完整五参数 `plumb_bob`、alpha=0。
输入尺寸须与标定一致；尺寸/标定错误会明确报错，CPU 回退不会修复错误的相机标定。
目标是8位 YUV420 视频；默认色度位置 `left`，中心采样使用 `--chroma-location center`。
所有路线保留相对输入时间戳，源视频有时间缺口时不会自动补成完整录像。

常用配置：`--backend`、`--intrinsics`、`--bitrate`（默认8M）、`--gop`（默认60）、
`--cpu-threads`（默认4）。Intel设备用 `--device`；NVIDIA用 `--cuda-device`，
OpenCL设备默认选 NVIDIA GPU，多个设备时可用 `--opencl-device 0.0` 指定对应平台/设备。
`--duration`、`--frames` 用于有限测试，`--progress` 可输出处理进度；`--ffmpeg` / `--ffprobe`
用于选择工具版本。输出也可直接指定 `.mp4` / `.mkv` 文件；已有文件不会被覆盖。

GPU 不可用或运行中退出时，终端打印原因并转入 CPU。运行中切换会重连输入、重建编码器，
存在短暂中断，客户端可能需要重新播放。若输出为文件，失败的 GPU 文件保留为
`*.gpu-failed-<时间戳>.*`，再创建 CPU 输出；本地文件输入从头重做，实时输入只能从重连后继续。
失败文件可能因异常中止而无法解码。CPU 速度取决于机器，可能低于源帧率、增加延迟，不能据此宣称录像完整。
CPU 也失败时明确报错退出，不反复重试掩盖输入、网络或磁盘故障。

**用户提示：** 摄像头面板的拉流地址下方会显示本机中继的 CPU 回退/处理中断/失败提醒，
完整原因在悬浮提示中；提示按当前实际播放地址关联，`localhost`、`127.0.0.1`、`::1` 等价。
局域网地址与回环地址须使用一致写法才能关联。状态文件位于
`$XDG_RUNTIME_DIR/ros2-ardupilot-relay/`，无该变量时在 `/tmp/ros2-ardupilot-relay-<uid>/`。
面板与中继须由同一用户运行；远端观看者目前不会收到这些本机状态。
本入口仍独立启动，尚未接入 GUI 启停或自启服务；原有机载采集链不变。

维护入口：`ground_relay.py` 管理选择、探测与进程；`relay_calibration.py` 共享镜头几何，
`relay_cpu.py` 实现保留时间戳的软件处理，`shaders/undistort_nv12.cl` 同时支持 NV12 与 YUV420P。
面板提示逻辑在 `camera_app/relay_status.py`。

## 地面面板

从项目根目录独立打开：

```bash
./.venv/bin/python video_service/camera_panel.py
```

面板的 frameless 标题栏、窗口缩放和 Wayland 自绘阴影复用
`ground_station_core/qt_ui/window_chrome.py`，入口会自动把项目根目录加入 Python 搜索路径；因此
应继续使用上述项目根目录命令启动。该显示层复用不改变摄像头后台与飞控生命周期隔离。

面板有三个正交来源：

- **本机摄像头**：本机设备、推流、画质与存储配置可编辑，启停和抓拍走私有
  Unix Socket；
- **真机摄像头**：配置只读，状态和 RTSP 地址从独立 ROS `VideoStatus` 获取，
  启停经 `SetVideoState` 发布期望状态，人工抓拍逐条发布 `VideoCapture`；
- **指定 RTSP**：只做查看器，不发送任何摄像头命令。

拉流地址始终是独立的手填字段，不由推流 IP/端口/路径拼接，也不会被后台轮询
覆盖。播放/暂停按钮与空格键等价；改变地址后再次播放会创建全新的 RTSP 会话。
播放时长和平均帧数只统计面板真正处于播放状态时收到的画面。切换来源或关闭
面板只卸载预览和 ROS 客户端，不会向本机或真机发送关闭命令。

### 地面站视频依赖（Ubuntu 24.04 / amd64）

MediaMTX 是与 CPU 架构相关的系统依赖，不随 Git 仓库分发。全新 x86_64 地面站先安装发行版
媒体工具，再安装并校验官方 MediaMTX v1.20.0 amd64 文件：

```bash
sudo apt update
sudo apt install -y ffmpeg v4l-utils ca-certificates curl

mediamtx_stage="$(mktemp -d)"
curl -fL \
  https://github.com/bluenviron/mediamtx/releases/download/v1.20.0/mediamtx_v1.20.0_linux_amd64.tar.gz \
  -o "${mediamtx_stage}/mediamtx_v1.20.0_linux_amd64.tar.gz"
echo '952d5f7d31d1b448ab4da4509550594c511d42636db9d7bb175d377f4ede81df  mediamtx_v1.20.0_linux_amd64.tar.gz' \
  | (cd "${mediamtx_stage}" && sha256sum -c -)
tar -xzf "${mediamtx_stage}/mediamtx_v1.20.0_linux_amd64.tar.gz" \
  -C "${mediamtx_stage}"
sudo install -m 0755 "${mediamtx_stage}/mediamtx" /usr/local/bin/mediamtx
file /usr/local/bin/mediamtx
/usr/local/bin/mediamtx --version
rm -rf -- "${mediamtx_stage}"
```

应看到 `x86-64` 与 `v1.20.0`。从项目根目录执行 `./scripts/ground/setup_ground_station.sh` 时也会检查
`ffmpeg`、`ffprobe`、`v4l2-ctl` 和该固定路径，缺失时直接给出安装提示。若面板曾在升级前
启动过旧 `camera_service.py serve`，先执行
`./.venv/bin/python video_service/camera_service.py shutdown`；下一次点击“开启本机摄像头”时
面板会按当前代码重新启动后台，避免旧进程继续持有已经删除的仓库内路径。

## 本机 Socket 服务

面板会按需分离启动后台，也可手工控制：

```bash
./.venv/bin/python video_service/camera_service.py serve
./.venv/bin/python video_service/camera_service.py status
./.venv/bin/python video_service/camera_service.py probe
./.venv/bin/python video_service/camera_service.py start
./.venv/bin/python video_service/camera_service.py snapshot
./.venv/bin/python video_service/camera_service.py stop
./.venv/bin/python video_service/camera_service.py shutdown
```

`status` 中的 `service_pid` 也可接收 `SIGUSR1` 人工截图。信号事件允许合并；
ROS 航点抓拍不使用这条事件路径，而是逐条排队、逐条发布成功或失败结果。

本机配置默认保存到
项目内 `config/ground/video.json`，运行文件位于 XDG runtime，日志
位于 `~/.local/state/ros2-ardupilot-camera/`。停止时先让 FFmpeg 封装录像，再把
`.partial.<格式>` 原子改名为最终时间文件。

## 机载进程与 ROS 接口 3.2

机载配置直接读取项目 `config/aircraft/<AIRCRAFT_ID>/video/camera.conf` 和
`lens.conf`，由 `config/aircraft.local.env` 的 `AIRCRAFT_ID` 选择飞机。
视频流只使用 Wasintek；标定基线保存在 `config/video/Wasintek/intrinsics.yaml`，
后缀 `.old` 表示旧镜头，视频流水线不读取标定内参或执行去畸变。
默认媒体目录为 `/home/share`，JPG 目录为 `/home/share/jpg`；采集格式、分辨率与帧率
必须是目标摄像头真实声明的组合。

### 飞机上为什么也有 Qt 文件

机载 sparse checkout 会同步整个 `video_service/`，因此飞机磁盘上也能看到
`camera_panel.py` 和 `camera_app/panel.py`。这是源码随包存在，不代表飞机运行 Qt：

- 飞机的 `video-service.service` 只执行 `onboard_video_node.py`；
- 机载节点只导入共享的 `config.py`、`controller.py` 和 `onboard_config.py`；
- `PySide6` 只由地面端 `camera_panel.py` 导入，飞机无需安装 PySide6，也不会创建窗口；
- 保留完整目录可以让地面与机载共用同一套 FFmpeg、MediaMTX、录像和截图实现，避免复制两套
  容易漂移的后端代码。

### 新飞机快速部署（Ubuntu 24.04 / Jazzy / ARM64）

以下步骤是当前 Jetson Orin NX 真机验证过的可重复基线。全程保持飞控未解锁；视频 unit 必须
独立于飞控 unit，不得加入 `scripts/onboard/start_onboard_control.sh`。

先按下文第 2～3 步安装一次系统媒体依赖；完成机载源码同步后，视频服务本身只需一条命令：

```bash
cd /home/<机载用户>/ros2-ardupilot-mavros-control
./video_service/deploy/install_onboard_video_service.sh
```

安装器验证 FFmpeg、v4l2-ctl 和系统 `/usr/local/bin/mediamtx`，在首次需要时构建 ROS 接口，然后
创建媒体目录与独立 systemd unit，并执行 `enable --now`。项目内逐机配置不会被覆盖。它不会操作
`ros2-ardupilot-onboard.service`，开机启动的也只是等待 ROS 命令的视频节点，摄像头仍保持关闭。

下面第 2～3 步是新系统必须手工完成的一次性系统依赖；第 4～5 步是安装器内部操作的展开说明，
只用于定制路径和故障排查。

1. **准备原生工作区。** 按
   [机载部署指导](../src/onboard_control/deploy/ONBOARD_DEPLOYMENT.md) 完成 sparse checkout，
   确认其中包含 `video_service/`，然后在飞机上原生构建和验证：

   ```bash
   cd /home/<机载用户>/ros2-ardupilot-mavros-control
   ./src/onboard_control/deploy/build_onboard_control.sh --verify
   ```

   不要从 x86 地面机复制 `build/`、`install/` 或 MediaMTX 二进制。

2. **手工安装视频系统依赖。** Ubuntu 默认 apt 源不提供 MediaMTX 本体，但 FFmpeg、摄像头工具
   和下载校验工具可直接安装：

   ```bash
   sudo apt update
   sudo apt install -y ffmpeg v4l-utils ca-certificates curl
   ```

   视频 unit 通过 `SupplementaryGroups=video` 取得摄像头权限，不需要为此重新登录。连接摄像头后
   先确认稳定设备路径和真实模式：

   ```bash
   v4l2-ctl --list-devices
   v4l2-ctl --list-formats-ext \
     -d /dev/v4l/by-id/<camera>-video-index0
   ```

   当前 Wasintek 支持 H.264/MJPEG 1280×720@120；更换摄像头后必须以新设备输出为准。

3. **手工安装匹配架构的 MediaMTX。** Git 仓库不再携带 MediaMTX 二进制；Ubuntu 24.04 默认
   apt 源也没有 `mediamtx` 包。ARM64 飞机使用已验证的官方 v1.20.0 ARM64 归档：

   ```bash
   mediamtx_stage="$(mktemp -d)"
   curl -fL \
     https://github.com/bluenviron/mediamtx/releases/download/v1.20.0/mediamtx_v1.20.0_linux_arm64.tar.gz \
     -o "${mediamtx_stage}/mediamtx_v1.20.0_linux_arm64.tar.gz"
   echo '6aa3c03da7b6477f1e110c8e18e819cf9ef121e8981b52b8f8219982dae35f2f  mediamtx_v1.20.0_linux_arm64.tar.gz' \
     | (cd "${mediamtx_stage}" && sha256sum -c -)
   tar -xzf "${mediamtx_stage}/mediamtx_v1.20.0_linux_arm64.tar.gz" \
     -C "${mediamtx_stage}"
   sudo install -m 0755 "${mediamtx_stage}/mediamtx" /usr/local/bin/mediamtx
   file /usr/local/bin/mediamtx
   /usr/local/bin/mediamtx --version
   ```

   若飞机不能直连 GitHub，应在地面机下载并校验同一归档后用 `scp` 传入；不要关闭摘要校验，
   不要跳过摘要校验，也不要从其他机器复制架构不明的可执行文件。

4. **选择逐机配置并准备媒体目录。** 在 `config/aircraft.local.env` 设置 `AIRCRAFT_ID=uav001`
   （1 号新飞机）或 `AIRCRAFT_ID=uav002`（2 号翻新飞机），核对对应 `video/camera.conf`
   的设备路径、发布 IP 和媒体目录。安装器只创建媒体目录，不复制或覆盖参数。
   镜头与采集参数在下一次摄像头“关闭→开启”时读取，无需重新构建。

5. **安装独立 systemd unit（安装器自动完成）。** 若需人工排障，可从模板生成实际 unit；三个
   占位符都必须替换：

   ```bash
   cd /home/<机载用户>/ros2-ardupilot-mavros-control
   video_workspace="$(pwd -P)"
   video_user="$(id -un)"
   video_home="$(getent passwd "${video_user}" | cut -d: -f6)"
   video_unit_stage="$(mktemp)"
   sed \
     -e "s|ONBOARD_USER|${video_user}|g" \
     -e "s|ONBOARD_WORKSPACE_PATH|${video_workspace}|g" \
     -e "s|ONBOARD_HOME_PATH|${video_home}|g" \
     system/systemd/video-service.service > "${video_unit_stage}"
   sudo install -m 0644 "${video_unit_stage}" \
     /etc/systemd/system/video-service.service
   sudo systemd-analyze verify /etc/systemd/system/video-service.service
   sudo systemctl daemon-reload
   sudo systemctl enable --now video-service.service
   ```

   禁止给该 unit 添加飞控服务的 `Requires=`、`PartOf=` 或 `BindsTo=`。摄像头故障只允许
   `video-service.service` 自己失败或重启。

6. **未武装验收。** 视频节点启动后摄像头默认保持关闭，不会占用设备：

   ```bash
   systemctl is-enabled video-service.service
   systemctl is-active video-service.service
   journalctl -u video-service.service -b --no-pager -n 100
   source /opt/ros/jazzy/setup.bash
   source /home/<机载用户>/ros2-ardupilot-mavros-control/install/setup.bash
   ROS_DOMAIN_ID=0 ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET \
     ros2 topic echo --once /video_service/status
   ```

   应看到接口 3.2、`service_available=true`、`running=false`、`state=stopped` 和正确 RTSP
   地址。随后可由地面摄像头面板测试开启、跨机拉流、抓拍、停止；停止后确认没有
   FFmpeg/MediaMTX、8554 监听或摄像头占用。整个过程不得申请飞行租约、解锁或起飞。

7. **更新与回退。** 更新源码后先运行 `./src/onboard_control/deploy/build_onboard_control.sh --verify`。配置更新直接修改对应逐机目录，使用 Git 提交同步；只需重启视频
   unit 时执行 `sudo systemctl restart video-service.service`，不要重启或停止飞控 unit。

独立启动命令：

```bash
./scripts/onboard/start_onboard_video.sh
```

手工执行与 systemd unit 读取同一份逐机配置，不回退系统目录或安装副本。
直接运行脚本时仍是前台普通进程，生命周期由当前终端管理。

彻底清理独立 unit、残留机载视频节点、配置的 RTSP 端口和真实摄像头占用者：

```bash
./scripts/onboard/stop_onboard_video.sh
```

清理后立即重新启动独立 systemd unit：

```bash
./scripts/onboard/stop_onboard_video.sh --restart
```

停止脚本不会停止或重启 `ros2-ardupilot-onboard.service`。它会结束所有占用已配置真机摄像头
设备的进程；执行前应确认没有需要保留的人工 qv4l2/FFmpeg 调试会话。

不要把它添加成 `scripts/onboard/start_onboard_control.sh` 的第五个受监督子进程；该飞控脚本会在任一
子进程退出时清理整套飞行栈，违反视频故障隔离要求。参考
`system/systemd/video-service.service` 建立单独 unit，且不要声明对飞控 unit 的
`Requires` 或 `PartOf`。当前 Jetson 已按这一边界部署并启用 `video-service.service`。

接口 QoS 与语义：

- `/video_service/control`：reliable + transient-local，深度 1，只表示最新期望；
- `/video_service/capture`：reliable + volatile，深度 256，旧航点绝不在节点重启后
  补拍；
- `/video_service/status`：reliable + transient-local，周期发布 RTSP 和实际媒体
  路径，消费者超过 3 秒未收到必须判陈旧；
- `/video_service/capture_result`：每条抓拍都返回成功或失败及任务/航点关联字段；
- `/video_service/set_video_state`：地面摄像头面板直接调用的独立启停服务，不要求
  onboard_control、飞行租约或 FCU 在线，响应只表示期望状态已进入视频队列；

飞行相关视频动作只能由 onboard_control 发布 `/video_service/control` 或
`/video_service/capture`；纯视频手动启停与抓拍则由地面摄像头面板直接连接 video_service。
onboard_control 不提供纯视频手动启停代理，避免两条业务链混用。

机载控制节点从 MAVROS `ExtendedState` 的起飞/空中边沿自动开启视频，在可靠
落地或解除武装边沿关闭；这覆盖遥控器起飞而不依赖某一种地面命令。航点只有在
机载到达判定真实成立后才发抓拍事件，失败不改变航点或飞行终态。
机载节点会通过 MAVROS `MessageInterval` 自动请求 MAVLink
`EXTENDED_SYS_STATE(245)` 2 Hz；当前 ArduPilot 若不显式请求不会持续产生
`/mavros/extended_state` 样本。

命名均包含本机时间和微秒：录像为 `recording-<time>.<format>`；人工图片含
`manual-N`，本地 GUI 航点含 `gcs-N`，上位机航点含其原始 `photoNo` 的可逆 URL
转义。上位机的 `pointNo` 始终仍来自 `taskPoints.index`，两者不可混用。

## 镜头配置

服务每次从关闭变为开启时先让 FFmpeg 按摄像头配置打开设备，并用 FFprobe 确认
RTSP 已经可读；流稳定 1 秒后才重新读取并分步应用 `lens.conf`。本任务指定：手动曝光
`auto_exposure=1`、曝光时间 25、增益 200；自动曝光、曝光时间和增益分别调用
`v4l2-ctl`，前两项之间各等待 200 ms，最后逐项读回全部值。其余值使用同型号摄像头
默认值。配置文件逐项记录了驱动实测范围和默认值。设置失败、读回不一致或应用期间
FFmpeg 退出都只报告视频启动失败，不影响飞控。

## FFmpeg、MediaMTX 与架构

H.264 和普通 MJPEG 都保持摄像头原生码流零转码。固定时间戳表达式只使用
FFmpeg 4.4 已支持的 `setts` `pts/dts` 选项，截图使用 `-vsync passthrough`。
开发机以源码构建的 FFmpeg 4.4.5 和系统 FFmpeg 6.1 做过兼容回归；当前 Jetson
Ubuntu 24.04/aarch64 使用 NVIDIA FFmpeg 8.0.1，已用真实同型号摄像头跑通 H.264
1080p30/MP4 与 MJPEG 720p30/MKV 的 RTSP、录像和 JPG。含 DRI 的 MJPEG 只重编码
RTSP 分支，录像仍为 stream copy。若恢复旧 Ubuntu 22.04/Humble 目标，其发行版
FFmpeg 4.4.2 仍须在该目标上单独复验，不能用当前 Jetson 结果替代。

MediaMTX 是系统依赖，Git 仓库不再保存预编译文件。所有目标机都必须安装与自身架构匹配的
v1.20.0，并由 `camera.conf` 使用 `/usr/local/bin/mediamtx`。当前 Jetson 已安装并实测该路径的
ARM64 版本；地面开发机也必须安装自己的 amd64 版本，不能跨架构复制。

## H.264 与 MJPEG 的运动画面取舍

编码格式本身不决定传感器的光学运动模糊。同一分辨率、帧率、曝光时间、镜头和运动速度下，
两种编码在曝光期间形成的模糊基本相同；要真正减轻运动模糊，应优先缩短
`exposure_time_absolute` 并用光照或增益补偿。较长曝光会增加运动模糊，这一点与编码器无关。

编码后观感和抓拍仍有差别：

- MJPEG 把每帧作为独立 JPEG，单帧不依赖前后画面；在快速运动、需要逐帧取证且带宽足够时，
  抓拍通常更稳定，主要缺陷是单帧 JPEG 的块效应/振铃；
- H.264 使用帧内/帧间预测，低得多的带宽和存储成本更适合飞机持续推流录像；码率不足、
  GOP 较长或场景突变时，运动区域可能出现块效应、细节涂抹或短暂预测残留；
- 因此“真实运动模糊谁更严重”的答案是两者相同；“编码后单帧谁通常更利于抓拍”的答案是
  MJPEG，但不能脱离该摄像头内部 JPEG 质量与 H.264 码率做绝对保证。当前生产默认选择 H.264
  1080p60，优先保证画面细节、跨机带宽和长时间录像的平衡；若任务把单帧证据质量置于带宽之上，应在同一运动
  标靶、同一曝光下实拍 A/B 后再切 MJPEG。

原理参考：[Basler 曝光与运动模糊说明](https://docs.baslerweb.com/optimizing-image-quality)、
[RFC 2435 Motion-JPEG 独立帧定义](https://www.rfc-editor.org/rfc/rfc2435.html)、
[RFC 6184 H.264 IDR/帧间预测定义](https://www.rfc-editor.org/rfc/rfc6184.html)。

基础依赖：ROS 2 Python、`ffmpeg`、`ffprobe`、`v4l2-ctl`。面板额外需要项目既有
PySide6。首次部署应确认：

```bash
uname -m
ffmpeg -version
v4l2-ctl --list-formats-ext -d /dev/v4l/by-id/<camera>-video-index0
file /usr/local/bin/mediamtx
```

## 当前验证边界

当前 Jetson Orin NX（Ubuntu 24.04/Jazzy/aarch64）已完成真实 Wasintek 摄像头、
ARM64 MediaMTX、NVIDIA FFmpeg 8.0.1、`/home/share` 权限、跨机 RTSP、录像、三类
JPG、镜头参数、地面 Qt 播放、ROS 直连、状态陈旧和视频故障隔离验证；全过程真实
FCU 保持 `armed=false`。隔离 domain 中的假 `ExtendedState` 已验证
`ON_GROUND -> IN_AIR -> ON_GROUND` 会产生关闭、开启、关闭期望。

默认值改为 H.264 1920×1080@60 后，当前飞机又完成一次真实复验：FFprobe 报告
1920×1080、60/1，录像为 32.17 秒、15,471,611 bytes，人工抓拍为 1920×1080 JPG；开启
一秒后读回手动曝光 25、增益 200。停止后 FFmpeg、MediaMTX、8554 和摄像头均释放，
`video-service.service` 仍独立运行且没有重启。该测试证明当前 H.264 默认链可用，但没有使用
受控运动标靶同时录制 MJPEG，因此不把静态场景结果包装成两种编码的清晰度定量排名。

尚未执行且不能由自动代理执行的只剩真实飞行态验证：用户人工起飞后的自动开启、真实落地后的
自动关闭，以及实际飞抵航点后的自动抓拍与上位机 08/09 联动。它们必须在未来经独立飞行安全
评审并由用户人工解锁、起飞时补验；隔离边沿测试不能替代真实飞行结论。



# 调试参数

```bash
qv4l2
```
