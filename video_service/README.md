# 独立视频服务

`video_service/` 复用一条经过本机摄像头验证的链路：FFmpeg 只打开一次 USB
摄像头，把原生 H.264/MJPEG 同时交给 MediaMTX 发布 RTSP 并封装录像；JPG
从本机 RTSP 读取，不会第二次占用设备。它是独立进程，摄像头、FFmpeg、
MediaMTX 或磁盘失败只进入 `VideoStatus`，不能停止或改变飞控任务。

## 地面面板

从项目根目录独立打开：

```bash
./.venv/bin/python video_service/camera_panel.py
```

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

应看到 `x86-64` 与 `v1.20.0`。执行根目录 `./setup_ground_station.sh` 时也会检查
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
`~/.config/ros2-ardupilot-camera/config.json`，运行文件位于 XDG runtime，日志
位于 `~/.local/state/ros2-ardupilot-camera/`。停止时先让 FFmpeg 封装录像，再把
`.partial.<格式>` 原子改名为最终时间文件。

## 机载进程与 ROS 接口 3.2

机载配置是带完整注释的 [camera.conf](config/camera.conf)，镜头配置是
[lens.conf](config/lens.conf)。默认媒体目录为 `/home/share`，JPG 目录为
`/home/share/jpg`，默认视频模式为原生 H.264 1280×720@120 fps；格式、分辨率和帧率
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
独立于飞控 unit，不得加入 `start_onboard_control.sh`。

先按下文第 2～3 步安装一次系统媒体依赖；完成机载源码同步后，视频服务本身只需一条命令：

```bash
cd /home/<机载用户>/ros2-ardupilot-mavros-control
./video_service/deploy/install_onboard_video_service.sh
```

安装器验证 FFmpeg、v4l2-ctl 和系统 `/usr/local/bin/mediamtx`，在首次需要时构建 ROS 接口，然后
创建配置、媒体目录与独立 systemd unit，并执行 `enable --now`。已存在的
`/etc/ros2-ardupilot/camera.conf` 和 `lens.conf` 不会被覆盖。它不会操作
`ros2-ardupilot-onboard.service`，开机启动的也只是等待 ROS 命令的视频节点，摄像头仍保持关闭。

下面第 2～3 步是新系统必须手工完成的一次性系统依赖；第 4～5 步是安装器内部操作的展开说明，
只用于定制路径和故障排查。

1. **准备原生工作区。** 按
   [机载部署指导](../src/onboard_control/deploy/ONBOARD_DEPLOYMENT.md) 完成 sparse checkout，
   确认其中包含 `video_service/`，然后在飞机上原生构建和验证：

   ```bash
   cd /home/<机载用户>/ros2-ardupilot-mavros-control
   ./build_onboard_control.sh --verify
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

4. **安装配置和媒体目录（安装器自动完成）。**

   ```bash
   cd /home/<机载用户>/ros2-ardupilot-mavros-control
   sudo install -d -m 0755 /etc/ros2-ardupilot
   sudo install -m 0644 video_service/config/camera.conf \
     /etc/ros2-ardupilot/camera.conf
   sudo install -m 0644 video_service/config/lens.conf \
     /etc/ros2-ardupilot/lens.conf
   sudo install -d -o <机载用户> -g <机载用户> -m 0755 \
     /home/share /home/share/jpg
   sudoedit /etc/ros2-ardupilot/camera.conf
   ```

   至少核对 `device`、`advertise_ip`、保存目录和
   `mediamtx_binary=/usr/local/bin/mediamtx`。生产环境推荐填写
   `/dev/v4l/by-id/...-video-index0` 稳定路径；`auto` 只适合唯一摄像头。默认配置是 H.264
   1280×720@120、MP4。配置只在下一次摄像头“关闭→开启”时生效，不会改飞控参数。

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
     video_service/deploy/video-service.service.example > "${video_unit_stage}"
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

7. **更新与回退。** 更新源码后先运行 `./build_onboard_control.sh --verify`。配置更新用
   `install` 覆盖前先自行保留 `/etc/ros2-ardupilot/camera.conf`、`lens.conf`；只需重启视频
   unit 时执行 `sudo systemctl restart video-service.service`，不要重启或停止飞控 unit。

独立启动命令：

```bash
./start_onboard_video.sh
```

手工执行与 systemd unit 使用相同的生产配置：若未显式设置环境变量，脚本优先读取
`/etc/ros2-ardupilot/camera.conf` 和 `/etc/ros2-ardupilot/lens.conf`；仅在系统配置不存在时
回退 `video_service/config/`。直接运行脚本时仍是前台普通进程，生命周期由当前终端管理；配置、
摄像头行为和 MediaMTX 路径则与 systemd 启动一致。

彻底清理独立 unit、残留机载视频节点、配置的 RTSP 端口和真实摄像头占用者：

```bash
./stop_onboard_video.sh
```

清理后立即重新启动独立 systemd unit：

```bash
./stop_onboard_video.sh --restart
```

停止脚本不会停止或重启 `ros2-ardupilot-onboard.service`。它会结束所有占用已配置真机摄像头
设备的进程；执行前应确认没有需要保留的人工 qv4l2/FFmpeg 调试会话。

不要把它添加成 `start_onboard_control.sh` 的第五个受监督子进程；该飞控脚本会在任一
子进程退出时清理整套飞行栈，违反视频故障隔离要求。参考
`deploy/video-service.service.example` 建立单独 unit，且不要声明对飞控 unit 的
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
- `/onboard_control/set_video_state`：不申请飞行租约，只确认期望状态已发布，不
  伪称硬件已经启动；保留给飞控侧代理和兼容调用，地面摄像头面板不再依赖它。

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
- H.264 使用帧内/帧间预测，低得多的带宽和存储成本更适合飞机持续 120 fps 推流录像；码率不足、
  GOP 较长或场景突变时，运动区域可能出现块效应、细节涂抹或短暂预测残留；
- 因此“真实运动模糊谁更严重”的答案是两者相同；“编码后单帧谁通常更利于抓拍”的答案是
  MJPEG，但不能脱离该摄像头内部 JPEG 质量与 H.264 码率做绝对保证。当前生产默认选择 H.264
  720p120，优先保证跨机带宽和长时间录像；若任务把单帧证据质量置于带宽之上，应在同一运动
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
JPG、镜头参数、地面 Qt 播放、ROS 代理、状态陈旧和视频故障隔离验证；全过程真实
FCU 保持 `armed=false`。隔离 domain 中的假 `ExtendedState` 已验证
`ON_GROUND -> IN_AIR -> ON_GROUND` 会产生关闭、开启、关闭期望。

默认值改为 H.264 1280×720@120 后，当前飞机又完成一次真实复验：FFprobe 报告 120/1，地面机
连续实时解码 600 帧成功，录像为 28.1 秒、73,466,260 bytes，停止后 FFmpeg、MediaMTX、
8554 和摄像头均释放。该测试证明当前 H.264 默认链可用，但没有使用受控运动标靶同时录制
MJPEG，因此不把静态场景结果包装成两种编码的清晰度定量排名。

尚未执行且不能由自动代理执行的只剩真实飞行态验证：用户人工起飞后的自动开启、真实落地后的
自动关闭，以及实际飞抵航点后的自动抓拍与上位机 08/09 联动。它们必须在未来经独立飞行安全
评审并由用户人工解锁、起飞时补验；隔离边沿测试不能替代真实飞行结论。



# 调试参数

```bash
qv4l2
```
