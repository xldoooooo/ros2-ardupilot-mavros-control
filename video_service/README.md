<!-- Independent video service operation, configuration and deployment. -->
# 独立视频服务

`video_service/` 复用一条经过本机摄像头验证的链路：FFmpeg 只打开一次 USB
摄像头，把原生 H.264/MJPEG 同时交给 MediaMTX 发布 RTSP 并封装录像；JPG
从本机 RTSP 读取，不会第二次占用设备。它是独立进程，摄像头、FFmpeg、
MediaMTX 或磁盘失败只进入 `VideoStatus`，不能停止或改变飞控任务。

## 地面集中媒体服务

机载采集和缓存原片，主动向地面推送一条原始RTSP；照片正常上传，整段视频仅在地面明确请求时通过rsync/SSH上传。
机载MediaMTX只监听127.0.0.1，供本机截图；外部只能读取地面`<飞机编号>/rectified`流。
地面`ground_media.py`独立于GUI运行，复用下述中继处理一次，同一编码器输出同时写入MKV和RTSP。
GPU使用FFmpeg tee封装，CPU将同一批编码包分发给两路容器；从首批编码包开始录像，避免再次拉流接入的等待。
GPU运行中失败会保留已有片段，再创建CPU片段继续；结束后合并可播放片段并保留缺段警告，不补帧或声称连续。
原片、上传暂存、处理暂存与FTP公开目录分开。

地面部署：

```bash
./video_service/deploy/install_ground_media.sh
# 主窗口“文件管理”可离线浏览已接收媒体；也可独立打开：
./.venv/bin/python video_service/file_manager.py
```

配置示例为`config/ground/media.example.conf`，安装时生成并保留本机`config/ground/media.json`。
其中 `gpu.intel`、`gpu.nvidia`、`gpu.amd` 可分别设置本机设备，Intel 可另设 OpenCL 运行时目录；
所有字段均可省略，Intel/AMD 默认自动发现设备，保留 CPU 回退。配置中的文件路径支持相对项目根目录、
`~` 和环境变量；MediaMTX 优先从 `PATH` 查找，也可指定相对或绝对路径。参数及生效方式见
[GPU 配置说明](deploy/GROUND_VIDEO_DEPLOYMENT.md#本机-gpu-参数配置)。
可用`GROUND_MEDIA_CONFIG`指定另一份文件。修改后重启`ground-media.service`。
顶层 `bitrate` 统一控制对外去畸变直播和最终去畸变 MP4 的目标码率，1080p60 清晰度优先建议设为 `"20M"`，
带宽受限可设为 `"16M"`；`gop` 保持 `60`。新安装示例使用 `20M`，已有机器需自行修改本机 `media.json`。
例如将现有字段改为 `"bitrate": "20M"`，无活动录像时执行 `sudo systemctl restart ground-media.service`，
下一次新会话使用新值；历史会话的重建沿用其已保存的编码参数，不会随配置自动提高码率。
`root`指定存储根，`public_host`指定客户端可达地址，`max_bytes`为总媒体容量上限；
统计internal/public/inbox/staging，包含活动文件，不含索引和日志。超限按最旧记录清理，
跳过录像/处理/传输及刚入库等待ACK的原片；若活动文件已占满空间，界面报错而不删除唯一机载副本。
FTP只读公开`root/public`，默认端口2121、被动端口30000～30019；原片和未完成文件不可下载。
HTTP 8090提供文件列表和处理版下载，本机GUI可删除或整段重建；RTSP默认8554。

每架飞机的`config/aircraft/<AIRCRAFT_ID>/video/camera.conf`中`[media]`配置
`push_url/public_url/ground_api/upload_target/upload_root/cache_directory`；上传root须与地面root一致。
先为机载用户配置到`upload_target`的免密SSH（正常host-key校验），再启用视频节点。
上传使用持久SQLite队列，图片优先，默认2048 KiB/s；网络恢复后rsync差分重传。
地面核对size和SHA-256并持久入库后发ACK，飞机从ACK开始保留86400秒再清理；
未确认文件继续保存。缓存清理、上传重试和处理失败分别显示，不影响飞行结果。
旧飞机FTP需停用，`/home/share`不再被任何新媒体入口使用；旧数据迁移由部署者确认后清理。

各会话固定保存五参数内参与编码配置，照片采用同一alpha=0投影；video记录关联飞机、
会话和任务，photo_no与waypoint_index分列。文件管理可筛选、预览/打开、另存、删除和从完整原片重建。
新会话目录为`mission-YYYYMMDD-HHMMSS`（飞机本地时间）；照片保留
`snapshot-时间-来源/编号.jpg`命名，视频使用`recording-时间.mp4/mkv`。
同秒新会话使用下一个可用秒数，重名成品追加序号；内部传输与数据库仍使用独立媒体ID。
地面实时处理版与原片重建版分开保存。冷启动、断流或GPU回退可能造成开头/中间缺失；
界面显示已知中断，封装后检查完整解码及大致时长，不声称逐帧完整。
建议飞行前手动开启摄像头，待地面显示录像运行后再开始任务；媒体异常不阻止飞控。
关闭预览/文件管理窗口不会终止录像和上传。
摄像头面板顶部只显示媒体摘要和重要异常，完整记录通过“详情”展开，预览会随窗口尺寸缩放。

巡检落地后优先交付地面实时去畸变录像，缺头或缺段但可播放时照常发送08，只记录警告。
多个可播放片段优先直接封装合并，不重新编码；只有地面没有可播放录像时，才请求机载原片并去畸变生成后备MP4。
明确需要人工处理、两端均无可用视频来源或后备处理失败时，立即终止该轮，发送videoPath/JPGPath均为空字符串的08并释放后轮。
`delivery_timeout_seconds`配置在地面`media.json`，默认600秒，从可靠降落成功事件开始计时（提前停录像不会启动超时）；GUI从可靠降落终态计时兜底。
超时同样以空路径08结束，不再自动上传/处理；终态跨服务重启保留，原始文件仍保留供检查。
正常视频08不等待迟到照片；09仍按本轮点位顺序独立报告，失败终止的轮次不再补09。
08的`data`仅含`videoPath/JPGPath`，09仅含`pointNo/pointName/pointPic`；不增加任务ID、媒体状态或补发字段。
路径均相对FTP虚拟根，例如`/uav002/mission-20261027-163300/recording-20261027-163300-000001.mp4`，生产环境使用已有FTP主机/端口直接下载。
照片失败和媒体错误写入本地日志；已发送的成功或空路径08不会因轮询、文件恢复或目录变化再次发送。
GUI文件管理仍使用HTTP下载，FTP只读根仍为`root/public`。

维护入口：`onboard_media.py`处理主动推流和任务关联，`media_transfer.py`维护机载队列，
`media_store.py`负责核验、照片和容量策略，`ground_media.py`负责服务与录像/重建，
`file_manager_app.py`与`camera_app/media_status.py`只负责GUI。

## 地面去畸变中继

新机安装、Intel自动加载、NVIDIA驱动/FFmpeg兼容要求及验收命令，见
[地面视频部署说明](deploy/GROUND_VIDEO_DEPLOYMENT.md)。

`ground_relay.py` 独立处理一条视频，再向已有 MediaMTX 发布；客户端共用处理结果。
默认 `--backend auto` 按 Intel、NVIDIA、AMD、CPU 顺序选择，先使用本地短样本实际探测硬件链路。
指定 `--backend intel|nvidia|amd` 时，该 GPU 不可用也会回退 CPU；`--backend cpu` 可直接使用软件路线。

| 后端 | 处理路线 | 依赖与限制 |
| --- | --- | --- |
| Intel | VAAPI 解码 → OpenCL 去畸变 → VAAPI 编码 | Intel OpenCL ICD、VAAPI 驱动与共享表面支持；保留原有低拷贝路线 |
| NVIDIA | NVDEC 解码 → OpenCL 去畸变 → NVENC 编码 | NVIDIA 驱动提供 CUDA、OpenCL、NVENC；FFmpeg 支持对应接口；含 GPU/主存搬运，不是零拷贝 CUDA 滤镜 |
| AMD | VAAPI 解码 → Vulkan/libplacebo 去畸变 → VAAPI 编码 | Mesa VAAPI/Vulkan、FFmpeg libplacebo；含主存搬运，无需 ROCm/OpenCL；已验证 laptop Radeon 780M |
| CPU | 软件解码 → OpenCV 去畸变 → libx264 编码 | 项目 Python 的 PyAV、OpenCV、NumPy、PyYAML；无需 GPU 驱动 |

```bash
# setup_ground_station.sh 也会安装此依赖清单；现有地面站升级时执行：
./.venv/bin/python -m pip install -r requirements-gui.txt

./.venv/bin/python video_service/ground_relay.py \
  --input rtsp://127.0.0.1:8554/camera-raw \
  --output rtsp://127.0.0.1:8554/camera-rectified
```

Ubuntu 24.04 的 Intel 后端需 `intel-opencl-icd` 和可访问的 render 设备，默认自动枚举并实际探测。
启动时优先使用系统/当前环境；如果探测失败，会自动尝试用户目录
`~/.local/share/ros2-ardupilot-video/intel-opencl`（设置了 `XDG_DATA_HOME` 时跟随该目录）。
这里存放已解包的 Intel OpenCL 及依赖，保留 `usr/lib/<架构>/intel-opencl/libigdrcl.so` 布局；
也可用 `--intel-opencl-runtime <目录>` 指定。程序自动为 Intel 子进程设置库路径和 ICD，
无需手工 export，不改变 NVIDIA/AMD/CPU 的环境。mini-ground 已在此固定目录部署并验证自动加载。
新机器可优先通过系统包安装 `intel-opencl-icd`；自动发现不会自行下载或安装驱动。
NVIDIA 后端需支持 NVDEC 与 H.264 NVENC 的显卡、匹配 FFmpeg 的 NVIDIA 驱动及其 OpenCL ICD；
不限定 RTX 3080，驱动最低版本取决于 FFmpeg 使用的编解码 API。无需另装 CUDA Toolkit、CuPy。
AMD 后端需 `mesa-va-drivers`、`mesa-vulkan-drivers` 和支持 `libplacebo` 的 FFmpeg；
laptop 已用现有驱动验证，无需安装 ROCm，其他机器以实际探测结果为准。
CPU 依赖中的 OpenCV/NumPy/PyYAML沿用项目环境；缺失时安装系统
`python3-opencv python3-numpy python3-yaml`，项目 `.venv` 需能访问系统包。
媒体工具 `ffmpeg`、`ffprobe` 和已配置允许发布的 MediaMTX 须提前就绪。

默认标定为 `config/video/Wasintek/intrinsics.yaml`：完整五参数 `plumb_bob`、alpha=0；
输出投影保持水平和垂直相同的缩放比例，避免画面变形，边缘按无黑边要求裁切。
输入尺寸须与标定一致；尺寸/标定错误会明确报错，CPU 回退不会修复错误的相机标定。
目标是8位 YUV420 视频；默认色度位置 `left`，中心采样使用 `--chroma-location center`。
所有路线保留相对输入时间戳，源视频有时间缺口时不会自动补成完整录像。

常用配置：`--backend`、`--intrinsics`、`--bitrate`（默认8M）、`--gop`（默认60）、
`--cpu-threads`（默认4）。Intel/AMD设备用 `--device`，也可分别用 `--intel-device` / `--amd-device` 覆盖；NVIDIA用 `--cuda-device`，
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
该中继由地面媒体后台管理，也可独立启动进行诊断；摄像头面板同时显示后台录像和文件状态。

维护入口：`ground_relay.py` 管理选择、探测与进程；`relay_calibration.py` 共享镜头几何，
`relay_cpu.py` 实现保留时间戳的软件处理，`shaders/undistort_nv12.cl` 同时支持 NV12 与 YUV420P。
AMD 着色器为 `shaders/undistort.hook`；面板提示逻辑在 `camera_app/relay_status.py`。
`relay_config.py` 将媒体配置中的可选 GPU 参数传给中继；设备设置使用当前主机配置，不随历史录像固定。

## 公网直播与 FTP

公网地址及凭据仅写入不提交 Git 的 `config/ground/media.json`。局域网原始流、
机载上传及本机 HTTP 文件管理继续使用现有地址。`fps` 省略或为 null 时保留源帧率；
例如设置为 `60` 并将 `bitrate` 设为 `"15M"`，使地面直播与录像使用该编码档位。
源视频也需达到 60 fps；提高配置值不会补出真实运动帧。仿真直播需在
`config/ground/simulation_media.json` 中设置相同的 `fps`、`bitrate`。
实时处理只抽帧，不填补原视频时间缺口。

```json
"public_stream": {
  "enabled": true,
  "streams": {"uav002": "rtsp://video.example:18554/live/uav002?secret=<TOKEN>"}
},
"remote_ftp": {
  "enabled": true,
  "url": "ftp://<USER>:<PASSWORD>@files.example:2121/",
  "directory": ".",
  "timeout": 60
},
"local_ftp_enabled": false
```

以上为对象内字段片段。公网转推从本机 `<编号>/rectified` 复制同一批 H.264 编码包，
不再去畸变或编码，断线独立重试。仅支持显式映射的 `uav001/uav002/simulation`，
`loop/rectified` 全天轮播始终不参与公网转推。`/api/media` 的 `public_streams` 提供每路状态。

远程 FTP 保留 `public` 根内目录；服务端需将账号根目录对应到媒体根。
先上传临时文件，再校验大小及正式回执路径，成功后才开放 08/09 的 FTP 路径。
上传失败在后台重试，本地预览和下载仍可使用；录像沿用既有落地交付超时。
上传结果保存在现有媒体索引的元数据，重启后不重复上传未变化的已确认文件。
仿真返回现有样例路径，启用公网前须先将配置中的样例目录同步到同一 FTP 根；
更换样例后也需同步，不会把样例文件改成实时编码档位。

恢复局域网时关闭 `public_stream.enabled`、`remote_ftp.enabled`，恢复本机 FTP 开关及原编码值；
如文件中保留 `_local_configuration`，按其中原值恢复。仿真 `fps` 恢复 null 即使用原复制路线。
修改后在无活动录像时重启 `ground-media.service`，并重新启动仿真媒体。
维护入口为 `public_stream.py`（直播转推）、`remote_ftp.py`（单文件发布），
调度与回执门控仍由 `ground_media.py` 负责。

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
后缀 `.old` 表示旧镜头；机载保留原片，地面使用当前内参去畸变。
默认原片位于 `~/.local/share/ardupilot-media/cache/videos` 和 `cache/images`；采集格式、分辨率与帧率
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

先按[机载部署](../src/onboard_control/deploy/ONBOARD_DEPLOYMENT.md)准备原生工作区和
[依赖](../src/onboard_control/deploy/ONBOARD_DEPENDENCIES.md)。前置依赖安装器包含FFmpeg、
V4L2工具及固定版本/架构的MediaMTX；机载视频另需rsync/SSH及项目 `.venv`。
不复制开发机 `build/install` 或架构不明的媒体二进制。

1. 选择 `config/aircraft.local.env` 的 `AIRCRAFT_ID`，核对对应 `video/camera.conf` 的稳定
   设备路径、采集格式、主动推流及地面上传参数。用 `v4l2-ctl --list-devices` 和
   `--list-formats-ext -d <实际设备>` 确认设备声明的组合。
2. 先配置到地面媒体主机的免密SSH和上传目录，再执行原生构建与安装：

   ```bash
   ./src/onboard_control/deploy/build_onboard_control.sh --verify
   ./video_service/deploy/install_onboard_video_service.sh
   ```

   安装器核验工具、MediaMTX v1.20.0和配置，创建媒体目录及独立 `video-service.service`，
   执行 `enable --now`；保留逐机参数。硬件未齐时加 `--install-only`，不启用或启动。
3. 保持未解锁，检查服务及状态：

   ```bash
   systemctl status video-service.service
   journalctl -u video-service.service -b --no-pager -n 100
   ros2 topic echo --once /video_service/status
   ```

   ROS终端先加载Jazzy和项目overlay，使用飞机相同domain。应看到接口3.2、服务可用且
   摄像头尚未运行；再从地面面板检查开启、RTSP、抓拍及停止后释放设备/端口。

前台及清理入口：

```bash
./scripts/onboard/start_onboard_video.sh
./scripts/onboard/stop_onboard_video.sh
./scripts/onboard/stop_onboard_video.sh --restart
```

手工和systemd共用逐机配置；镜头/采集配置在摄像头下次关闭→开启时读取。
停止脚本清理独立unit及占用配置相机的进程，执行前确认无需保留人工qv4l2/FFmpeg会话。
更新源码后重建，只重启视频unit；不能把视频加入飞控受监督子进程，不能对飞控unit声明
`Requires/PartOf/BindsTo`。视频异常不能停止或重启飞控链。

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
v1.20.0；机载默认路径为 `/usr/local/bin/mediamtx`，也可在所选 `camera.conf` 的
`mediamtx_binary` 中指定其他可执行路径，安装器与运行时读取同一值。当前 Jetson 已安装并实测默认路径的
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

## 验证边界与诊断

硬件验收应核对真实摄像头声明格式、录像可完整解码、抓拍尺寸、曝光读回、RTSP 可达性，
以及停止后设备/端口释放和服务故障隔离。可使用 `qv4l2` 查看设备控制项。
历史静态台架结果不能代替受控运动画面的 H.264/MJPEG 对比，也不能证明媒体逐帧完整。

用户人工起飞后的自动开启、真实落地后的关闭、飞抵航点抓拍和上位机08/09联动，
须结合实际飞行及媒体结果验收；隔离假状态测试只验证事件链。实机解锁与起飞只能由用户手动执行。
