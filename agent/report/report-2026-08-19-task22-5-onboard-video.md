# 任务 22.5：机载视频、地面面板与上位机媒体回传执行报告

- 日期：2026-08-19
- 任务文件：`agent/task/22.5-onboard-video.md`
- 开发机：Ubuntu 24.04 / ROS 2 Jazzy / x86_64
- 真机状态：本次未连接飞机，未访问飞控，未解锁、未起飞
- 结论：本机同型号摄像头、隔离 ROS 仿真和自动化能够覆盖的部分已完成；飞机
  Ubuntu 22.04 / Humble / aarch64 台架验收明确留待下次连接真机后补全。

## 一、完成情况

| 范围 | 结果 | 说明 |
|---|---|---|
| 独立机载视频模块 | 完成 | 位于根目录 `video_service/`，单独进程与单独 systemd 示例，不进入飞控共同故障域 |
| 起飞自动开启、落地自动关闭 | 完成并仿真 | 使用 MAVROS `ExtendedState` 空中/落地边沿，解除武装作为关闭备份，覆盖非地面站起飞来源 |
| 地面站直接启停真机视频 | 完成并仿真 | 独立 `SetVideoState` 服务，不需要飞行租约，响应只表示期望状态已发布 |
| 航点与人工抓拍 | 完成并仿真 | 航点到达后逐条排队，人工抓拍独立编号；成功或失败均逐条回报 |
| `photoNo` 原值传递 | 完成 | 不校正类型、顺序、重复或负数；仅用于可逆文件名，不与 `pointNo` 混用 |
| 媒体目录与甲方字段 | 完成 | 默认视频 `/home/share/`、图片 `/home/share/jpg/`；08/09 使用真实完成结果，失败或超时不伪造路径 |
| 摄像头与镜头配置 | 完成 | `camera.conf`、`lens.conf` 均有参数用途、同型设备组合/范围和默认值说明 |
| 地面摄像头面板三模式 | 完成 | 本机摄像头、真机摄像头、指定 RTSP；查看器与摄像头服务状态相互独立 |
| 面板交互优化 | 完成 | 手填 URL、播放/暂停、空格、复制图标、真机地址读取、播放统计、全下拉/数字框禁滚轮 |
| `video-service` 重命名 | 完成 | 生产路径、测试、地面站入口及 sparse 部署均改为 `video_service` |
| 本机同型号摄像头实测 | 完成 | H.264、MJPEG、RTSP、录像、JPG、镜头参数与资源释放均实测 |
| 飞机 ARM64 实测 | 待补 | 当前无真机，未部署 ARM64 MediaMTX，未验证飞机 FFmpeg 4.4.2、目录权限与真实局域网 |

## 二、实现说明

### 2.1 独立视频故障域

视频服务复用原地面端的 FFmpeg + MediaMTX 技术链，摄像头只由一个 V4L2 采集
进程打开一次，再同时输出 RTSP 和录像；JPG 从本机 RTSP 抓取，不会第二次占用
USB 摄像头。

机载入口为 `video_service/start_onboard_video.sh`，systemd 参考文件为
`video_service/deploy/video-service.service.example`。视频没有加入
`start_drone_all.sh`：该脚本的任一受监督子进程退出都会清理飞行栈，把视频加入
其中会违反“视频故障不得影响飞控”的要求。示例 unit 也没有对飞控服务声明
`Requires` 或 `PartOf`。

摄像头、FFmpeg、MediaMTX、磁盘或 ROS 视频节点故障只写入 `VideoStatus` 和抓拍
结果，不会改变 `active_task`、控制模式、无人机异常、failsafe 或飞行命令结果。

### 2.2 ROS 2 接口 3.2

共享接口和 `onboard_control` 包版本均升级为 3.2.0，线协议升级为 3.2：

- `VideoControl.msg`：reliable + transient-local + depth 1，只保留最新期望开关，
  视频节点晚启动或重启仍能收到当前期望；
- `VideoCapture.msg`：reliable + volatile + depth 256，航点抓拍逐条投递，节点重启
  后不会补拍历史航点；
- `VideoCaptureResult.msg`：每条请求都返回成功或失败、实际 JPG 路径及任务关联；
- `VideoStatus.msg`：reliable + transient-local，周期发布服务、推流、录像与错误状态；
- `SetVideoState.srv`：地面站直接控制视频期望，不进入飞行租约和飞行控制门控；
- `Waypoint.msg`：增加 `has_photo_no` 与 `photo_no`，保存甲方原始编号。

`VideoStatus` 的 transient 样本可能在节点退出后残留，所以地面客户端按本地单调
时间将超过 3 秒未更新的状态判为陈旧，不能仅凭缓存中的 `service_available=true`
宣称真机服务在线。

地面摄像头面板每次进程启动使用新的 UUID `source_id`，避免面板重启后从序号 1
开始时被机载端误判成旧请求。

### 2.3 飞行边沿与抓拍时点

`onboard_control` 新增 `/mavros/extended_state` 订阅：

- 首次进入 `TAKEOFF`/`IN_AIR` 发布视频开启期望；
- 曾经在空中后进入 `ON_GROUND` 发布关闭期望；
- `armed -> disarmed` 边沿提供可靠关闭备份。

因此逻辑不依赖某个 `start_takeoff()` 调用，也能覆盖遥控器等其他方式触发的实际
起飞。自动抓拍的落点位于机载航点到达判定 `kReached` 成立之后、航点索引递增
之前，使用 1-based 航点序号；抓拍失败不撤销已经完成的航点。

### 2.4 文件名、配置与媒体路径

所有媒体名都含本地时间和微秒：

- 录像：`recording-<timestamp>.<format>`；
- 面板人工抓拍：包含 `manual-N`；
- 我方地面站航点：包含 `gcs-N`；
- 甲方航点：包含原始 `photoNo` 的可逆 URL 编码。

甲方的重复、负数、字符串或非递增 `photoNo` 均原样保存，不参与 `pointNo` 计算。
`cameraAngle` 按任务要求继续忽略。

`video_service/config/camera.conf` 默认配置 RTSP、设备、分辨率、帧率、H.264/MJPEG、
图片/视频目录与格式；`video_service/config/lens.conf` 只包含任务要求的十项镜头参数。
每次摄像头从关闭变为开启时，服务会重新读取两个配置，并在 FFmpeg 打开设备前
应用镜头值。代码不夹紧用户配置；设备驱动拒绝时只令视频启动失败并如实报告。

任务指定的镜头值为：

```text
auto_exposure=1
exposure_time_absolute=25
gain=200
brightness=6
contrast=6
saturation=6
hue=0
sharpness=6
power_line_frequency=1
zoom_absolute=10
```

上位机 09 现在由真实 `VideoCaptureResult` 驱动，`pointPic` 是实际完成的 JPG 路径；
08 在巡检航点和降落均到达可靠终态后等待各点图片结果及录像封装，再发送实际
`videoPath`/`JPGPath`。等待超时、抓拍失败或录像仍未封装时发送空路径，不把当前
`.partial` 或尚在写入的录像伪称成已完成文件，也不反向把媒体故障判成飞行失败。
返航任务继续不发 08/09。

### 2.5 地面摄像头面板

面板保留独立 detached 进程边界，提供三种来源：

1. **本机摄像头**：允许设备探测、推流地址、画质、存储、启停和人工抓拍；
2. **真机摄像头**：本地配置整组禁用，可读取新鲜 `VideoStatus` 中的 RTSP 地址并
   发送独立视频启停/人工抓拍请求；
3. **指定 RTSP**：只允许查看任意 RTSP，不发送本机或真机摄像头命令。

拉流输入框在所有模式下均可手填，只接受有主机名的 `rtsp://` URL。IP、端口、
路径、本机状态轮询和真机后台状态都不会自动覆盖用户输入；只有用户明确点击
“读取真机地址”才写入真机地址。播放键、暂停键和空格使用同一状态机，修改 URL
后再次播放会销毁旧播放器并创建新会话；复制键写入输入框原值。

“当前录像”和“已采集帧”已删除；“播放时长”只累计 `QMediaPlayer` 真正处于
Playing 的区间，“平均帧数”只统计 `QVideoSink` 收到的有效画面，暂停期间两者
冻结。所有 `QComboBox` 和数字输入框都忽略滚轮事件。切换来源或关闭面板只卸载
预览和自身 ROS context，不会向本机或真机发送 stop。

## 三、验证结果

### 3.1 正式自动化与构建

执行：

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select guided_interfaces onboard_control \
  --symlink-install --event-handlers console_direct+ \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
.venv/bin/python -m pytest -q tests
colcon test --packages-select guided_interfaces onboard_control
colcon test-result --verbose
python ground_station.py --check-environment
```

结果：

- 两个 ROS 包 Release 构建成功；
- Python 正式测试范围：`160 passed in 39.12s`；
- ROS/C++：`19 tests, 0 errors, 0 failures, 0 skipped`；
- 干净环境下 `ground_station.py --check-environment` 成功；
- 新增测试覆盖三模式门控、URL 不被覆盖、播放/暂停/换源、图标与剪贴板、播放
  统计、禁滚轮、关闭不发 stop、视频 QoS、逐条抓拍、stale 状态、任意起降边沿、
  `photoNo` 原值、实际 08/09 路径以及独立部署边界；
- 修改范围的生产 Python 编译、flake8 `F/I`、shell `bash -n` 与 `git diff --check`
  均通过。

### 3.2 无真机 ROS 隔离仿真

所有模拟均使用非零 domain 和 localhost-only 发现，没有 FCU、飞行租约、解锁或
飞行指令：

- domain 229：假机载控制端与真实视频节点验证 start/stop、状态发布、逐条航点/人工
  抓拍队列及一条失败结果；
- domain 228：编译后的 `onboard_control_node` 配合假 `ExtendedState`，验证
  `ON_GROUND -> IN_AIR -> ON_GROUND` 自动开关，并验证无飞行租约的
  `SetVideoState` 代理；
- domain 227：真实面板 ROS 客户端验证自动读取地址、3 秒陈旧判定，以及关闭客户端
  不发布 stop。

### 3.3 本机同型号 Wasintek 摄像头实测

稳定设备路径为：

```text
/dev/v4l/by-id/usb-Wasintek_Wasintek_camera_00.00.01-video-index0
```

H.264 1920×1080@30：

- RTSP 探测为 H.264、1920×1080、30 fps；
- `manual-1`、`manual-2`、`gcs-1` 和上位机原值 `客户/A 01` 四类 JPG 均成功，
  上位机编号在文件名中可逆编码，图片约 362～363 KB；
- 十项镜头参数读回值与配置完全一致；
- MP4 样本 6.5 秒、15,299,594 bytes，H.264 1080p30 可正常读取；
- 停止后无 `.partial` 文件，摄像头和 RTSP 端口均释放。

MJPEG 1280×720@30：

- RTSP 探测为 MJPEG、1280×720、30 fps；
- JPG 为 143,903 bytes；
- MKV 样本 2.766 秒、10,724,289 bytes，可正常读取；
- 停止后无 `.partial` 文件，设备和端口均释放。

测试结束后没有遗留 FFmpeg、MediaMTX、摄像头服务或测试 ROS 进程。

### 3.4 FFmpeg 4.4 兼容验证

Ubuntu 22.04 的 FFmpeg 4.4 不支持新版本 `setts` 的 `duration/time_base` 参数，也
不支持后来的 `-fps_mode passthrough`。生产命令因此改为 4.4 可用的：

```text
setts=pts=N/(FPS*TB):dts=N/(FPS*TB)
-vsync passthrough
```

本机从官方源码临时构建 FFmpeg 4.4.5，确认其 `setts` 仅有 `ts/pts/dts`，并用该
二进制跑通真实 Wasintek H.264：RTSP 为 1080p30，JPG 为 279,852 bytes，MP4 为
2.833 秒、7,728,792 bytes；停止后无 partial 和端口残留。临时源码与构建目录已
删除。

这只能证明同一 4.4 系列命令和 x86_64 实流可用，不能替代飞机 Ubuntu 22.04
发行版 FFmpeg 4.4.2 的 aarch64 验收。

## 四、未完成项与已知限制

下列内容因当前没有真机而没有伪造完成：

1. 飞机 aarch64 上替换为 MediaMTX v1.20.0 Linux ARM64；仓库当前二进制是 x86-64；
2. 飞机发行版 FFmpeg 4.4.2 上的 H.264/MJPEG、截图和最终封装；
3. 飞机 `/home/share`、`/home/share/jpg` 的属主、权限、空间和掉电行为；
4. 真实摄像头稳定 `/dev/v4l/by-id` 路径及十项 V4L2 参数读回；
5. 飞机真实 LAN IP、RTSP/TCP 端口、防火墙和地面 Qt 跨机拉流；
6. 真实 MAVROS `ExtendedState` 的起飞/落地边沿和飞行期间资源负载；
7. 飞机 source、install 与运行进程升级为接口 3.2 后的跨 Humble/Jazzy 通讯。

额外说明：仓库正式测试命令限定为 `pytest tests/`，其 160 项全部通过。直接从仓库
根运行无范围的 `pytest -q` 会额外收集历史
`integration/websocket_test_demo/tests/test_protocol.py`，因该独立 demo 缺少
`ws_demo` 模块而在收集阶段失败；这不是本次生产测试回归。对整个 `video_service`
无差别 `compileall` 还会碰到未改动历史 demo `demo5.py` 的 GBK 编码声明问题，生产
入口和新增模块的定向编译已通过。这两项均未通过删除 demo 或降低正式测试标准来
掩盖。

## 五、下次真机补测顺序

下次连接飞机后应按以下顺序进行，且默认全程保持 `armed=false`：

1. 只读确认 `uname -m`、Ubuntu/ROS/FFmpeg 版本、飞控 `armed=false`、已有开机服务和
   源码/install/runtime 接口版本；
2. 停止可能冲突的视频实例，但不干扰飞控服务；安装并校验 MediaMTX v1.20.0 ARM64；
3. 创建并核对 `/home/share`、`/home/share/jpg` 权限与磁盘空间，确认摄像头 by-id 和能力；
4. 构建并 source 接口 3.2，两包测试和 localhost-only smoke 通过后再启动独立视频 unit；
5. 在未武装台架完成启停、RTSP、三类 JPG、录像封装、服务故障隔离和地面面板拉流；
6. 检查 CPU、内存、磁盘吞吐和视频节点崩溃不会停止飞控栈；
7. 如需验证真实起飞/落地自动边沿，只能由用户本人决定并手动解锁、起飞；代理不会执行
   解锁或起飞操作。

## 六、主要文件

- `video_service/onboard_video_node.py`
- `video_service/camera_app/controller.py`
- `video_service/camera_app/panel.py`
- `video_service/camera_app/ros_client.py`
- `video_service/config/camera.conf`
- `video_service/config/lens.conf`
- `video_service/README.md`
- `src/guided_interfaces/msg/Video*.msg`
- `src/guided_interfaces/srv/SetVideoState.srv`
- `src/onboard_control/src/onboard_control_node.cpp`
- `ground_station_core/upstream/status_projector.py`
- `tests/test_camera_service.py`
- `tests/test_onboard_video_service.py`

