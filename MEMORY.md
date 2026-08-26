# 项目重要记忆

本文件只维护当前真实基线、长期有效的工程约束和少量关键历史节点。逐任务过程、实验数据、
临时路径和旧版本结论统一查阅 `agent/report/`，不再在本文件重复堆叠。

当本文件与源码、包清单或最新验证报告冲突时，以当前源码和实际运行时检查为准，并及时修正
本文件。当前基线日期为 2026-08-26，仓库线协议为 3.2。

## 绝对安全边界

- 严禁代理自行解锁或起飞实机；实机解锁与起飞只能由用户人工完成。
- 默认实机排查必须保持 `armed=false`，优先使用只读状态、被动 DDS 检测、隔离 smoke 和无桨
  台架检查。不得把“进程启动”“服务 ACK”或“话题可见”扩大解释为可实飞。
- 正式“连接实机服务”不是纯只读操作：它会申请控制租约、续发心跳、确认消息频率并写入人工
  确认的 GPS/EKF 原点。齿轮右侧 Wi-Fi 检测才是只订阅状态与日志的被动入口。
- 修改或部署机载代码后，必须核对源码、`install/`、运行进程报告的接口版本一致；禁止让旧安装
  产物静默运行。
- 对飞机、云服务器或其他远端主机的服务进行停止、重启、覆盖或删除前，必须确认目标和影响；
  不得干扰与当前任务无关的既有服务。

## 当前环境、入口与验证命令

- 开发机：Ubuntu 24.04、ROS 2 Jazzy，已安装 MAVROS 与 ArduPilot SITL。
- 当前真机伴随计算机已更换为 Jetson Orin NX、Ubuntu 24.04、ROS 2 Jazzy、aarch64；SSH 为
  `nvidia@192.168.112.169`，工作区为
  `/home/nvidia/ros2-ardupilot-mavros-control`。旧 `xld@192.168.112.186` 的
  Ubuntu 22.04/Humble 结果只属于历史基线，不得当作当前飞机状态。
- Python 必须使用项目 `.venv`。地面站入口为 `ground_station.py`，推荐通过
  `./start_ground_all.sh` 启动；`--check-environment` 只检查环境，不创建飞行会话。
- 当前 ROS 工作区包含：
  - `src/guided_interfaces`：地面站与机载端共享的唯一高层协议；
  - `src/onboard_control`：机载 C++ 控制与安全状态机；
  - `src/guided_sim`：URDF、RViz 与预览/TF 可视化，不含第二套控制器。
- 常用构建与验证：

  ```bash
  source /opt/ros/jazzy/setup.bash
  colcon build --packages-select guided_interfaces onboard_control guided_sim
  source install/setup.bash
  QT_QPA_PLATFORM=offscreen .venv/bin/python -m pytest -q tests
  colcon test-result --verbose
  ```

- 地面端或飞机只重建两包机载代码可运行 `./build_onboard_control.sh`；`--verify` 追加依赖、
  ROS/C++ 测试和 localhost 隔离 smoke。构建不会自动重启运行中的机载服务。
- `README.md` 当前存在并维护常用启动/停止说明；Ubuntu 22.04 通用部署见
  `DEPLOY_UBUNTU_2204.md`，机载最小部署见
  `src/onboard_control/deploy/ONBOARD_DEPLOYMENT.md`。

## 当前架构边界

### 地面站

- GUI 为 PySide6/Qt 6，本项目没有浏览器 Web GUI。`ground_station.py` 只负责环境自举和 Qt
  入口；主要代码位于 `ground_station_core/`。
- `ground_station_core/ros_controller.py` 是地面端 ROS 客户端门面：按需创建独立
  `rclpy.Context`，发布心跳/运动意图，调用高层服务，聚合 `ControlStatus`。它不得创建
  MAVROS 姿态 setpoint 发布者或保存安全关键的持续控制算法。
- `ground_station_core/environment.py` 编排本地 SITL 或实机会话；实机路径只连接远端机载服务，
  不启动或终止远端 MAVROS、Odin、extnav 或 onboard 节点。
- `ground_station_core/process_manager.py` 只管理本项目明确启动的本地进程组。退出、仿真终止和
  异常退出兜底必须清理 SITL、仿真 MAVROS、仿真 onboard 与 RViz，不得停止 ROS daemon 或
  其他 ROS 工作负载。
- `ground_station_core/upstream/` 是独立 WebSocket/JAR 上位机协议边界；通讯故障不得破坏已有
  ROS、仿真或实机会话。

### 机载端

- `src/onboard_control` 是唯一飞行控制权威：租约仲裁、起降编排、航点推进、失联保护、100 Hz
  PD+DOB、姿态/推力输出和 MAVROS 网关都位于机载 C++ 服务。
- 手动运动、悬停和航点最终共用同一个 `DobController`，唯一生产输出为
  `/mavros/setpoint_raw/attitude`；检测到多个 setpoint 发布者必须故障关闭。
- 起飞由 ArduPilot GUIDED/arm/takeoff 完成安全离地，稳定后切入机载控制；LAND 交给
  ArduPilot，并以实际解除武装作为可靠终态，不把 SetMode ACK 当成已经落地。
- 连续控制参数集中在 `src/onboard_control/config/control.yaml`。GUI 不复制控制增益或安全阈值。

### 独立摄像头服务

- `video_service/` 与 ROS/飞行生命周期解耦。地面站只通过 detached Qt 面板打开它；关闭面板
  不会停止正在运行的推流或录像。机载视频节点使用根目录 `start_onboard_video.sh` 和独立
  systemd unit，严禁加入 `start_onboard_control.sh` 的共同故障域。面板启停直接调用
  `/video_service/set_video_state`，不得依赖 onboard_control 或飞行租约在线。
- 生产链只打开一次 V4L2 摄像头，使用 FFmpeg 同时发布 MediaMTX RTSP/TCP 和保存录像；截图从
  本机 RTSP 获取，不会第二次占用摄像头。
- 机载端通过独立 `VideoControl`、`VideoCapture`、`VideoCaptureResult`、`VideoStatus` 和
  `SetVideoState` 通讯；飞行自动事件只从 onboard_control 发布，面板手动操作直接连接
  video_service，onboard_control 不提供纯视频手动代理。视频服务缺失、卡死或失败不得改变
  飞行状态、任务终态或安全链。
- 飞行自动开关以 MAVROS `ExtendedState` 的起飞/空中/落地边沿为准，解除武装是关闭备份；航点
  抓拍只在机载到达判定成立后、推进航点索引前异步发布。
- MediaMTX 已从当前源码树移除，地面 amd64 与飞机 ARM64 都必须把各自架构的 v1.20.0 安装到
  系统 `/usr/local/bin/mediamtx`；默认配置和代码不再回退到仓库内二进制。
- 当前地面站已安装并实测 amd64 MediaMTX v1.20.0；`setup_ground_station.sh` 会检查 FFmpeg、
  ffprobe、v4l2-ctl、固定 MediaMTX 路径及二进制能否在本机执行。删除仓库二进制后若旧面板后台
  仍存活，必须先对 `camera_service.py` 执行 `shutdown`，否则它仍会使用进程内存中的旧路径。
- 2026-08-20 经用户明确授权重写 `main`：历史 127 MiB MP4、53 MiB MediaMTX 和 25 MiB
  rtsp-simple-server 三个 blob 已从活动对象库彻底消失；`agent/task/assets` 图片/视频只保留本地，
  不再跟踪。`.git` 从约 290 MiB 降至约 41 MiB。
- 改写前的完整 `.git`（包括4个无法判定无用的不可达提交及独立对象）备份在
  `/home/nvidia/backups/ros2-ardupilot-git-pre-history-rewrite-20260820.tar.gz`，SHA-256 为
  `77ff2b0b6a82dfbf922c3f3b41effbffae20cdbbeea62aa1f832dd96c9625215`。活动仓库已清理这些对象，
  但仍可从该项目外备份恢复。远端历史改写后旧提交 ID 失效，其他机器应重新 clone。

## 当前接口与控制语义

- 当前线协议 `INTERFACE_VERSION = "3.2"`；`guided_interfaces` 与 `onboard_control` 包版本均为
  `3.2.0`。地面站、共享接口和机载运行产物必须同步部署。
- `ControlStatus` 是 GUI 的权威状态源，包含飞控/武装/位姿/速度/姿态/电池、控制模式、参考值、
  租约、航点进度、航点入点失败计数、无人机异常、推力语义、setpoint 冲突和控制周期诊断。
- 所有高层输入携带 `source_id`、单调序号、时间戳和 TTL。同一时刻只允许一个租约持有者；地面站
  以 5 Hz 心跳续租。首次成功租约以地面站发送时间和机载接收单调时钟建立相对基准，有序心跳持续
  刷新；TTL 比较相对流逝时间，不比较两机绝对日期，也不依赖外网 NTP。重复、乱序、过期或非持权
  命令仍必须拒绝。
- 租约丢失时机载端立即抓取当前位置悬停；默认 10 秒未恢复则请求 LAND。外部切走 GUIDED、
  位姿/飞控状态超时、非有限输出、setpoint 冲突或飞行中推力语义失效都属于安全降落条件。
- ArduPilot 必须启用 `GUID_OPTIONS` bit 3，使 `SET_ATTITUDE_TARGET.thrust` 表示真实归一化推力；
  未确认时拒绝原始姿态/推力控制。
- `hover_throttle: 0.22` 只是启动回退值；运行时由飞控 `MOT_THST_HOVER` 覆盖。不得把历史 SITL
  或某架实机的数值硬编码回控制器，更不得未经授权写飞控参数。
- `message_rates_configured=true` 只表示 `MessageInterval` 请求得到 ACK，不代表已经实测三路消息
  达到 100 Hz。状态日志必须区分 requested、accepted、applied 与 observed。
- 自动频率链必须包含 MAVLink `EXTENDED_SYS_STATE(245)` 2 Hz；当前 ArduPilot 真机默认不发送
  该消息，不主动请求就没有 `/mavros/extended_state`，也无法检测遥控器等非地面站起飞边沿。

## ROS 会话、仿真与实机隔离

- GUI 打开后默认保持 `ROS IDLE`，不会创建 DDS participant。启动仿真、正式实机连接或 Wi-Fi
  被动检测时才按需启动 ROS。
- 本地仿真固定使用 domain 231 + localhost-only 发现，并清除显式静态发现变量；实机固定使用
  domain 0 + subnet 发现。切换或断开会销毁旧 context，不保留跨会话 participant。
- SITL 使用自身 Home 建立 EKF；仿真禁止写入 GUI 缓存的实机 GPS 原点，否则会造成数百万米级
  ENU 偏移。实机原点只在正式连接流程中使用，并等待 FCU `gp_origin` 匹配回读。
- Wi-Fi 检测只订阅 `ControlStatus` 和远端日志，不申请租约、不发心跳/维护/飞行命令，也不管理
  远端进程。
- Humble/Jazzy 混合 Fast DDS 图曾稳定传输当前自定义状态与服务，但仍会出现
  `sequence size exceeds remaining buffer`。ROS 官方不保证跨发行版通信；统一发行版/RMW 或
  完成独立长期验收前，不得把当前结果视为完整实飞通信基线。

## 当前 GUI 与航点功能

- 主窗口默认 1600×920，最小 1180×700；采用 frameless 外框、自绘阴影和统一
  `ShadowMessageBox`。退出、真机危险操作、终止仿真、断开和清空航点继续使用默认取消确认。
  只有隔离仿真中的起飞、降落和发送航点按现有产品要求免二次确认。
- 手动操纵为双摇杆美国手布局，默认坐标系是“本地 ENU”，可切换“机体坐标”；左右摇杆灵敏度
  独立。鼠标和键盘必须共用 `OperationsPanel.trigger_motion()`，所有控件服从统一安全门控。
- 航点使用绝对本地 ENU `X/Y/Z/Yaw`。CSV 表头为 `index,x,y,z,yaw`，一次最多 256 点；导入先
  完整校验、默认取消确认，确认后才原子替换 GUI 列表。
- RViz 航点预览只显示名义直线路径、编号和权威实时机体位姿，不申请租约、不发送飞行命令或
  MAVROS setpoint。仿真预览复用 domain 231，实机预览使用 domain 0，断开时清理。
- `EventLog` 在事件产生处保存 DEBUG/INFO/WARN/ERROR、来源、时间和序号；Qt 只筛选已有等级，
  不按文本猜测。SITL/MAVROS 启动刷屏可降为 DEBUG，但显式 WARN/ERROR 不得屏蔽。
- 上位机通讯面板是无父级的普通 `Qt.Window`，不得恢复为主窗口的 transient `QDialog`：后者会
  被窗口管理器强制压在主窗口上方且缺少有效最小化提示。面板可独立最小化，再点主界面入口会
  恢复；主窗口退出时必须显式销毁该独立面板。
- 地面站与独立摄像头入口在原生 Wayland 下、且用户未显式指定时，于创建 `QApplication` 前选择
  Qt `adwaita` 窗口装饰；X11 和显式 `QT_WAYLAND_DECORATION` 配置不得被覆盖。
- 上位机通讯面板和摄像头配置面板的投影不能依赖 GNOME Wayland 合成器：两者使用与主窗口一致
  的 frameless 透明顶层窗口、14 px 留边和 Qt 自绘阴影，并保留独立任务栏、标题栏拖动、四边缩放、
  最小化、最大化/还原和关闭。摄像头的原生 `QVideoWidget` 不挂图形特效，阴影只画在独立背景框。
- LAND 是已武装状态下的安全动作：只要机载可靠命令链存在并持有控制权，就不得被 GUI busy、
  当前模式、位置/推力诊断或已有 LAND pending 锁死，并允许幂等重发。离线、关闭中、无控制权
  或多个机载状态端点冲突仍必须安全禁用。

## 当前航点参考生成与异常恢复

- `ExecuteWaypoints` 同时携带飞行策略、参考生成器和跟踪控制器。飞行策略当前只真正实现直线；
  “自动避障”“遇障悬停”仍是明确占位，选择后会告警并按直线执行。
- 参考生成器包含位置阶跃、二阶滤波、梯形速度和限 jerk S 曲线；跟踪侧包含位置 PD+DOB 与轨迹
  PD+DOB。GUI 默认组合为梯形速度 + 轨迹 PD+DOB；协议未知值、机载待机状态仍安全回退到位置
  阶跃 + 位置 PD+DOB，限 jerk S 曲线 + 轨迹 PD+DOB 仍是另一组已验证平滑组合。
- 选择组合只允许在解除武装待机时进行；机载端会锁定同一武装周期的方法，防止绕过 GUI 热切换。
- 平滑任务启动时若实际速度超过默认 `0.20 m/s`，机载端先抓取当前位置并悬停制动，首次立即记
  `1/10`，之后默认每秒重判；恢复后继续原任务，连续 10 次失败才锁存无人机异常。
- 航点进入位置候选区但速度不满足时使用独立入点计数器，默认每秒重判；达到 10 次后锁存异常。
  启动与入点计数互不污染，取消、新任务、解除武装和可靠异常清除会按各自语义重置。
- 机载可靠航点终态携带的 `waypoint_index/waypoint_count` 必须保留到 GUI；LAND 抢先切换状态时，
  仅当前票据的最终成功结果可补齐 `N/N`，迟到旧票据不得回填已经重置的进度。
- 上述 3.1 航点异常与恢复链已通过单元测试和多轮完整 SITL；截至当前没有实机飞行验证。

## 当前上位机 WebSocket 语义

- 命令映射唯一维护在 `ground_station_core/upstream/mapping.py`：01 起飞、02 原子替换 GUI 航点、
  03 起飞→巡检航点→末点 LAND、05 飞至 `(0,0,起飞高度)` 后 LAND、06 正常 LAND、07 原地
  紧急 LAND。所有动作复用现有 GUI 门控和高层 ROS 服务；实机危险动作仍需要本地人工确认。
- 低电量和无人机异常自动返航只在 WebSocket 实际在线的仿真会话中启用；WebSocket 断线时仿真
  与实机都不触发这两类自动飞行动作。WebSocket 在线实机当前仅在地面站日志/横幅提示，明确
  不下发航点或 LAND，实机实现保留为 TODO。该门控只决定是否新触发组合，不强行撤销断线前
  已经由机载端接收的飞行命令。
- 09 由真实 `VideoCaptureResult` 驱动，`pointNo` 仍来自 `taskPoints.index`，`pointPic` 使用实际
  JPG 路径；08 在巡检航点与降落可靠终态后等待图片结果和录像封装，再发送实际
  `videoPath/JPGPath`。真实路径为空时分别使用 `/home/share/test.mp4` 与
  `/home/share/jpg/test.jpg` 占位，但不反向判定飞行失败；返航不发 08/09。
- 状态 01 表示“已解除武装、位于机库阈值内且可再次起飞”的边沿。默认阈值为
  `|X|<1.0 m`、`|Y|<1.0 m`、`|Z|<0.5 m`，巡检落地后默认再等待 60 秒；这些值可由
  `UPSTREAM_HANGAR_*` 和 `UPSTREAM_INSPECTION_STANDBY_DELAY_SECONDS` 覆盖。
- 03 巡检即使已经解除武装，在上述待机时间和机库判定完成前仍属于活动组合；GUI 起飞必须保持
  禁用，与上位机“组合操作仍在执行”的拒绝语义一致，不能仅凭解除武装提前开放。
- 当前机库判定只有本地 ENU 阈值，没有独立机库硬件证据；`cameraAngle` 继续忽略。`photoNo`
  已按原值随航点传给视频服务，只用于图片命名，不得与 `pointNo` 混用，也不得校正其重复、负数
  或顺序。

## 当前摄像头兼容边界

- 支持原生 H.264 或 MJPEG，录像可选 MP4/MKV/AVI，RTSP 固定 TCP。服务、面板和飞行链完全
  解耦；地面站退出不会自动停止摄像头后台。
- 甲方 Windows 单机调参与局域网拉流验收教程位于
  `docs/Windows-USB摄像头调参与RTSP推流教程.md`：使用 OBS 调参与 H.264 编码、MediaMTX 提供
  RTSP/TCP、VLC 跨设备验证；示例 `rtsp://192.168.112.101:8556/camera` 中 IP 必须替换为推流
  电脑的真实局域网 IPv4，不能把示例 IP 当成任意可选地址。
- 摄像头预览在首个有效 sink 帧前以及断流、无效帧、停止和超时时必须使用普通 Qt 控件逐像素
  绘制不透明纯黑，并隐藏原生 `QVideoWidget` surface；`LoadedMedia/BufferedMedia` 本身不等于
  已有画面，状态文字只能显示在黑色预览区之外。暂停状态保留最后一帧。
- 多设备探测以本次实际 `selected_device` 为权威，不能把旧持久化设备路径与另一设备模式混用。
- RFC 2435 的 RTP/JPEG 宽高字段上限为 2040 像素；超限 MJPEG 模式会在面板隐藏并由后台拒绝，
  H.264 高分辨率不受该限制。
- HP Quanta 5MP 的 MJPEG 含 DRI/restart markers：录像保留原始 stream-copy，只有 RTSP 分支
  自动规范化为兼容 MJPEG；无 DRI 的 Wasintek MJPEG 和所有 H.264 继续零转码。DRI 兼容分支
  有明显 CPU/RSS 成本，不能把它推广到所有设备。
- Wasintek 设备已验证 MJPEG 1280×720@120；HP 已验证兼容分支 MJPEG 1920×1080@30。具体设备
  能力仍以目标机 `probe` 为准，不应把某台机器的 `/dev/videoN` 当成稳定标识。
- 机载默认配置位于 `video_service/config/camera.conf` 与 `lens.conf`，媒体目录为
  `/home/share`、`/home/share/jpg`。默认模式为 H.264 1920×1080@60，默认手动曝光为 25、增益
  为 200。FFmpeg 按编码、分辨率和帧率打开设备且 RTSP 可读后，服务等待 1 秒，再分步写入并
  读回全部镜头参数；设置或读回失败只令视频失败。飞机上手工运行 `start_onboard_video.sh`
  与 systemd 都优先使用 `/etc/ros2-ardupilot/camera.conf` 和 `lens.conf`，仅未部署系统配置时
  才回退仓库默认文件。
- 同型号 Wasintek 已在开发机和当前 Jetson 真机验证 H.264 1080p30/60、H.264 720p120 与 MJPEG
  720p30 的 RTSP、录像、JPG、镜头参数、封装和资源释放。Jetson 使用 NVIDIA FFmpeg 8.0.1；
  MediaMTX v1.20.0 ARM64 安装在 `/usr/local/bin/mediamtx`，SHA-256 为
  `2da379972ba86627632aa7e3f779c680ba04a5ee26ef2a20dc61cefcc24f73b8`。
- 同一曝光下 H.264/MJPEG 的光学运动模糊基本相同；MJPEG 独立帧通常更利于单帧抓拍但带宽高，
  H.264 更适合持续 720p120 推流录像。未经同一运动标靶 A/B，不作清晰度定量排名。

## 当前机载部署事实

- 机载 sparse checkout 清单包含两个 ROS 包、`video_service/`、根目录视频启停脚本、
  `start_drone/`、`start_onboard_control.sh`、`stop_onboard_control.sh` 和 `build_onboard_control.sh`；不得
  复制开发机的 `build/`、`install/` 到飞机。
- 文档当前的 `'/video_service/'` Git sparse 规则会拉整个目录；开发树约 90 MB，主要是 x86
  MediaMTX 与历史 demo。当前 Jetson 实际通过选择性 rsync 部署，目录约 440 KB，虽含 Qt 面板
  源码但不含上述大文件；机载 unit 不导入 PySide6、不创建窗口。正式 Git 部署前应决定目录级
  common/onboard/ground 拆分，不能误称当前飞机已经按整目录 sparse 拉取。
- `stop_onboard_video.sh` 默认停止独立 unit 并彻底清理残留视频节点、配置 RTSP 端口和真机摄像头
  占用者；`--restart` 清理后只重启 `video-service.service`。它不得调用飞控停止入口或操作飞控
  systemd unit。
- 新飞机具有两个独立一键入口：`src/onboard_control/deploy/install_onboard_service.sh` 构建、验证并
  部署四组件飞控 unit；`video_service/deploy/install_onboard_video_service.sh` 在系统媒体依赖已安装
  后部署视频配置、目录与独立 unit。二者重复执行均保留 `/etc/ros2-ardupilot/` 现场参数，飞控
  安装器遇到 active 服务会拒绝自动更新，绝不在未知飞行状态下重启。
- `start_onboard_control.sh --check` 只做发现和配置检查，不启动组件；正式运行会统一启动 MAVROS、Odin、
  extnav 和 onboard，并在已有实例时拒绝重复启动。`stop_onboard_control.sh` 同时停止 systemd
  服务和其他终端手工启动的项目组件，并验证无残留。
- 正式入口命名已统一：机载飞控为根目录 `start_onboard_control.sh` / `stop_onboard_control.sh`，完整
  地面站安装为 `setup_ground_station.sh`。旧的 `start_drone_all.sh`、`stop_onboard_service.sh`、
  `setup_project.sh` 已从当前树和真机工作区移除；真机 unit 与 sparse 配置均指向新名称。
- `setup_ground_station.sh` 是地面站唯一项目安装入口；它会构建地面仿真所需的共享接口和
  onboard_control 源码，但不会安装/启用飞机的两个 systemd unit。两个 deploy/install 脚本仅供
  机载计算机使用。
- 机载环境文件为 `/etc/ros2-ardupilot/onboard.env`。历史已确认的飞机串口是
  `/dev/ttyTHS1:460800`，但新部署优先使用人工确认的 `/dev/serial/by-id`；多个串口或 overlay
  候选时必须安全失败，不允许猜测。
- systemd 服务只等待 `network-online.target`，不得依赖 `systemd-time-wait-sync.service`、
  `time-sync.target` 或固定 `sleep`；自带路由器无外网时必须启动。离线开机后若再接入互联网，应在
  人工解锁前等待 Linux/MAVROS 时间状态稳定并重新核对 READY，但外网时间不是控制租约前置条件。
- 当前 Jetson 的 source/install/runtime 已原生构建并运行接口 3.2。2026-08-24 维护窗口已把
  61 个机载范围文件与本地 `465ce8a` 逐文件同步并通过 SHA-256 比对，两个安装器已把飞控与视频
  systemd unit 更新为当前模板；两项服务最终均为 active/enabled、零重启，MAVROS 为 connected、
  `armed=false`、STABILIZE。机载视频现场配置仍位于 `/etc/ros2-ardupilot/camera.conf`、
  `lens.conf`，安装器重装时未覆盖。
- 当前飞机 Git HEAD 仍为历史 `6a40713`，任务 22.5 通过逐文件同步部署，因此工作树有明确的
  3.2 修改和 `video_service/` 新目录；2026-08-24 的选择性同步也没有改写该历史 HEAD。另有 Odin
  自动生成的 `image/cam_in_ex.txt` 与 `src/odin_ros_driver/`。不得用 reset/clean 或盲目 pull
  覆盖，后续仍应选择性同步或重新建立可安全快进的 sparse checkout。
- 部署前备份为 `/home/nvidia/backups/task22_5-predeploy-20260819-2317.tar.gz`，SHA-256
  `a070336413b6308db55a2155526be21c87f11fb249ccaca031ca95847697b27c`。真机台架媒体已从生产
  目录移至 `/home/nvidia/task22_5-bench-artifacts-20260819/`。
- 2026-08-24 同步与 unit 重装前备份为
  `/home/nvidia/backups/onboard-sync-pre-20260824-224307.tar.gz`，SHA-256 为
  `e714dc92bf49153658e3bfb329e4cedcc19dca6591b3325ea33f85547528abd8`。
- 2026-08-25 离线时间修复已选择性同步并在 Jetson 原生重建；新 unit 无任何外网校时等待，生产
  source/install/runtime 对齐。部署前备份为
  `/home/nvidia/backups/offline-clock-predeploy-20260825-0009.tar.gz`，SHA-256 为
  `7f78963ad63dbf80adb73ec7b9fb0f359ba3236c6bdaab302a90800c29c74368`。

## 当前验证基线（2026-08-25）

- 离线时间修复已通过本地 180 项 Python、19 项 ROS/C++、隔离 smoke，以及 Jetson ARM64 的
  `+30 天 → -30 天` 地面时间跳变探针；新鲜命令接受、回退 30 秒命令仍按 TTL 拒绝。真实生产服务
  最终连续两次 `armed=false`、无租约/控制器/冲突/failsafe，约 100.03 Hz、零 deadline miss；
  全程未向真实 MAVROS 发送飞行命令。真实断 WAN 冷启动仍需下次现场开机补记运维验收。
- 上位机 08 媒体占位路径已提升为模块常量，并覆盖视频服务不可用和录像未封装场景；专项测试
  14 项通过。开发机与 `scq@192.168.112.101` 的 `main` 均按远端主线快进同步。

- 2026-08-24 任务 24 的窗口装饰、摄像头纯黑预览、LAND 门控和航点终态投影修复构成当前
  `main` 基线；工作树中的用户自有改动仍须保留。
- 正确加载 ROS 2 Jazzy 和项目 `install/` overlay 后，项目正式 Python 范围 `tests/`：177 passed。
- 当前 colcon 结果：19 tests、0 errors、0 failures、0 skipped。
- 任务 24 已通过真实 JAR + Qt + ROS + MAVROS + ArduPilot SITL 全流程：最终解除武装、航点进度
  `2/2`，已武装 LAND 误禁用和组合待机起飞误启用记录均为空；Ubuntu 24.04 GNOME Wayland
  目标机确认纯黑预览中心和四角均为不透明 `#000000`；两个子面板自绘阴影在真实 Wayland 渲染
  中均得到 alpha 33 的半透明阴影像素，不再依赖 `adwaita` 或合成器外部阴影。
- 当前 Jetson 已完成接口 3.2 ARM64 Release 构建、19 项测试和无 MAVROS 隔离 smoke；真实 FCU
  最终为 connected、`armed=false`、STABILIZE、无租约/控制器、约 100.03 Hz。真实
  `EXTENDED_SYS_STATE` 连续为 ON_GROUND，自动视频期望为关闭。
- 2026-08-24 同步重启后再次通过 19 项测试和隔离 smoke；真实控制状态为待机、无租约、无冲突、
  约 100.04 Hz、该次零 deadline miss。独立视频实际完成开启、RTSP/TCP H.264
  1920×1080@60 探测和关闭，生成 6.17 秒 MP4；最终视频 stopped，FFmpeg、MediaMTX 与 8554
  端口均释放。全程没有解锁或起飞。
- 真机 H.264 1080p30/MP4、MJPEG 720p30/MKV、三类 JPG、原始 `photoNo`、跨机 FFmpeg/Qt
  拉流、播放暂停、真机地址读取、无租约启停、stale 判定、视频 unit 干净停止、媒体故障隔离和
  恢复均通过；当前摄像头/8554 端口未占用，生产媒体目录保留本次 1080p60 验收的 MP4/JPG。
- 新默认 H.264 1920×1080@60 已在当前飞机复验：RTSP 与 32.17 秒 MP4 均为
  1920×1080@60，人工 JPG 为 1920×1080，镜头读回曝光 25/增益 200；最终视频 stopped、
  媒体进程/端口/设备均释放，飞控 unit 未被触及。
- 上位机任务 22 与平滑航点启动余速修复均完成真实 JAR + Qt + ROS + MAVROS + ArduPilot SITL
  多轮端到端验证，最终解除武装且无残留进程。
- 本次真机补测始终没有解锁或起飞；实际起飞/落地自动启停和实际飞抵航点自动抓拍仍只完成
  隔离 ROS 边沿与自动化验证，必须由用户未来人工飞行时补验。
- 2026-08-21 已向新飞机 `/home/nvidia/AprilTag/` 部署独立 AprilTag 平面相机外参标定工具：
  只订阅 `/odin1/odometry_highfreq`，运行时仅打开空闲相机，不发布 ROS 消息、不调用服务、不改
  参数或 systemd。真机 Python/ROS 环境中的合成自测通过；部署检查时 Odin 话题未发布，因此尚无
  真实标定结果。配置内相机内参、畸变、标签尺寸/朝向均为待现场确认值，不能直接作为生产外参。

## 已知风险与维护重点

- 当前飞机和地面开发机均为 Jazzy，已不再经过旧 Humble/Jazzy 混合 DDS 边界；当前主要部署
  风险是飞机 Git HEAD 与已验证的 3.2 工作树未形成可拉取提交，更新时必须保护现有部署文件。
- Linux 非实时调度下曾出现 deadline miss 和明显 jitter；平均 100 Hz 不等于硬实时。当前 Odin
  进程的 `LimitRTPRIO=0`、无有效 capability，IMU 线程申请 SCHED_FIFO/SCHED_RR 得到 EPERM 后
  回退 SCHED_OTHER。当前链路能 READY，但高负载下 Odin 时间抖动风险仍未量化；实机前仍需长时间
  统计，再评审 `LimitRTPRIO`/`CAP_SYS_NICE`、CPU 隔离和优先级，不能未经测试直接给整个飞控 unit
  提权。
- 2026-08-24 重启前 MAVROS 2.14.0 曾多次报告 `Time jump detected`。该告警表示 10 Hz MAVLink
  TIMESYNC 的时钟偏移样本在滤波收敛后连续偏离当前估计，并触发时间同步滤波器重置，不等于 Linux
  NTP 当前失效或飞控重启。命令租约已不依赖该绝对时间；若离线开机后联网发生大幅校时或该告警在
  飞行前持续复现，仍应记录
  `/mavros/timesync_status`、串口负载和飞控端时间源，避免带着不稳定时间戳进入外部定位验收。
- 当前 Jetson 的 `load-iwlwifi.service` 与 `nvpmodel.service` 为既有 failed unit；Wi-Fi 和本次
  视频/飞控测试仍可用，但失败原因尚未纳入任务 22.5。Odin launch 还会在无显示环境启动 RViz
  并报 Qt platform 错误，四组件服务仍可达到 READY；后续应作为独立运维任务处理。
- 避障、机库硬件确认、`cameraAngle` 云台控制和更多硬件异常类型仍未实现；不要把占位接口写成
  已完成功能，也不要未经明确允许实现 `TODO.md`。
- 当前维护热点是少数持续膨胀的核心文件：
  1. `GroundStationWindow` 同时承担 Qt 编排和上位机组合任务状态机；
  2. `GroundStationRosController` 同时承担 ROS 生命周期、租约、命令分派和状态聚合；
  3. `OnboardControlNode` 同时承担 MAVROS 网关、任务、安全、控制和状态发布；
  4. `OperationsPanel` 与 `tests/test_qt_gui.py` 体量较大。
  后续应做保持行为不变的定向拆分，不做全仓推倒重写；优先提取纯业务状态机，最后再谨慎调整
  100 Hz 机载控制路径。
- `DEPLOY_UBUNTU_2204.md` 仍有个别接口 3.0 的历史文字；当前接口必须以
  `ground_station_core/config.py` 和两个 `package.xml` 的 3.2/3.2.0 为准，后续文档任务再统一。

## 关键历史节点

- **2026-08-06：机载权威边界建立。** 地面站收敛为薄客户端，PD+DOB、租约、失联保护和唯一
  setpoint 发布迁入机载 C++；完整 SITL 基线形成。详见
  `agent/report/history/report-2026-08-06.md`。
- **2026-08-10：仿真/实机域隔离与生命周期收敛。** 仿真固定 domain 231/localhost，实机
  domain 0/subnet；DDS context 切换、退出和异常残留清理形成当前边界。详见
  `agent/report/report-2026-08-10-ground-simulation-domain-isolation-cleanup-fix.md`。
- **2026-08-11～13：机载部署与真机未武装链路。** 建立 portable build、sparse 部署、systemd
  校时门、彻底停止入口和 source/install 版本门；接口 3.0 在真机完成未武装通讯与连接验证。
  详见 `agent/report/report-2026-08-13-hardware-communication-interface-fix.md`。
- **2026-08-12：连续航点参考与可视化。** 接口 3.0 引入四种参考生成器、两种 PD+DOB 跟踪模式、
  CSV 导入和 RViz 航点预览；平滑组合只在 SITL 调参与验证。详见
  `agent/report/report-2026-08-12-waypoint-smooth-reference-methods.md`。
- **2026-08-17～18：独立 USB 摄像头服务。** 建立单次采集、RTSP/录像/截图、双设备切换和 HP
  MJPEG DRI 兼容路径。详见
  `agent/report/report-2026-08-17-camera-video-service-ground-station-integration.md` 与
  `agent/report/report-2026-08-18-hp-mjpeg-dri-rtsp-green-fix.md`。
- **2026-08-18～19：上位机时序与接口 3.1。** 实现 03 巡检组合、05/低电量/异常返航、可靠
  01/08/09 状态语义、航点入点异常和启动余速重试；只完成自动化与 SITL 验证。详见
  `agent/report/report-2026-08-18-task22-timeseq.md` 与
  `agent/report/report-2026-08-19-waypoint-start-speed-retry-fix.md`。
- **2026-08-19：独立机载视频与接口 3.2。** 完成视频 ROS 边界、任意起降边沿开关、航点/人工
  抓拍、上位机真实媒体路径、三模式面板、配置与目录迁移；随后在新 Jetson 上完成 ARM64 部署、
  未武装真实摄像头/跨机/故障隔离台架，并补上 `EXTENDED_SYS_STATE(245)` 自动请求。实际飞行
  边沿与航点抓拍仍未执行。详见 `agent/report/report-2026-08-19-task22-5-onboard-video.md` 与
  `agent/report/report-2026-08-19-task22-5-real-aircraft-bench.md`。

## 版本库与记录规范

- 每次功能或代码修改后，在 `agent/report/` 新建当日报告，不覆盖旧报告；失败指标和未覆盖边界
  必须如实记录。
- `agent/codex/`、`agent/grok/` 是本地过程目录，正式配置和长期证据不得只放在其中。
- 视频录屏、飞行日志、SITL/MAVProxy 产物、colcon/Python 缓存等按 `.gitignore` 管理；大视频不
  直接提交 Git，需分发时使用外部存储或经明确评估后使用 Git LFS。
- `TODO.md` 只记录计划；未经用户明确允许不得实现其中内容。
