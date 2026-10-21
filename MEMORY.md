<!-- 只维护当前基线、部署差异和未解决风险；实验过程与数据查阅 agent/report。 -->
# 项目重要记忆

维护日期：2026-10-21。开发机、refresh与new已原生部署Task6 odom/map导航、三策略地图任务和安装外参转换，飞行接口3.5；hp-desktop已同步3.5并原生重建地面站、控制/接口、独立避障、修正服务及项目Odin。项目Odin驱动和机载MAVROS九插件配置已在三机落地，refresh保留MAVROS2.16.0修复overlay。Task5及自主避障输出jerk的跟踪/净空风险仍未关闭，实机飞行尚未验收。
源码与实际运行结果优先于本文件；更新时替换旧结论，实验过程写报告，不追加开发流水账。

## 工作边界

- **严禁代理自行解锁或起飞实机，必须由用户手动操作。** 实机排查保持 `armed=false`；
  进程启动、服务 ACK、话题可见、台架通过均不代表可实飞。
- 执行规则以 `AGENTS.md` 为准；未经明确允许不得实现 `TODO.md`。
- 正式“连接实机服务”会申请租约、续发心跳、配置消息频率并写入人工确认的 GPS/EKF 原点。
  齿轮右侧 Wi-Fi 检测才是被动订阅入口，不申请控制权或管理远端进程。
- 同步必须保留逐机配置和用户标定，不用 `reset/clean` 清除现场改动；飞机原生构建，
  不复制开发机 `build/`、`install/`。核对源码、安装产物和运行接口，不能只看 Git HEAD。

## 当前架构与入口

- 独立 map 系位姿诊断脚本源码在 `dev/odom2map/odom_pose_in_map.py`；new 另有根目录副本
  `/home/nvidia/ros2-ardupilot-mavros-control/odom_pose_in_map.py`（与原始版本一致）。
  只订阅 `/tf` 和 `/odin1/odometry_highfreq`，输出 map 下 Odin IMU 位姿，不是 FCU 中心或最终 EKF 位姿；无输入时不输出数值。
  该工具未加入自动启动；refresh 本次未核对或部署，需要时从开发机 `dev/` 单独复制即可。
- 开发机及两架现役飞机均为 Ubuntu 24.04 / ROS 2 Jazzy；机载为 Jetson Orin NX / ARM64。
  Python 使用项目 `.venv`。旧 Ubuntu 22.04/Humble 台架数据不再代表现役环境。
- 开发机、refresh、new及hp-desktop飞行协议 **3.5**，`guided_interfaces/onboard_control/guided_sim` 包版本 **3.5.0**；不能混用新旧接口及安装产物。
  独立视频协议 **3.2**，AprilTag 修正协议 **2.0**。地面与机载共享接口必须同步重建。
- PySide6 地面站是薄客户端；`src/onboard_control` 是唯一飞行控制权威，负责租约、安全状态机、
  航点推进及 100 Hz PD+DOB。唯一生产输出为 `/mavros/setpoint_raw/attitude`；GUI、规划器、
  RViz 不得成为第二个 setpoint 发布者。控制参数集中在 `src/onboard_control/config/control.yaml`。
- GUI 初始 `ROS IDLE`；仿真使用 domain **231 / LOCALHOST**，实机使用 **0 / SUBNET**。
  切换会销毁旧 ROS context；SITL 使用自身 Home，禁止写入 GUI 缓存的实机 GPS 原点。
  退出只清理本项目启动的本地进程组，不停止 ROS daemon 或其他工作负载。
- 三个机载 unit 独立：`ros2-ardupilot-onboard.service`（MAVROS/Odin/extnav/onboard）、
  `video-service.service`、`odin-correction.service`。视频和修正不能绑定飞控共同启停/故障域；
  地面正式连接不启动或停止远端服务。避障另行启停，目前没有规划自启 unit。
- Odin 权威维护目录是 `drivers/odin/`，独立构建入口 `drivers/odin/build.sh`；完整机载构建先增量构建它。
  单独及集成启动均用 `-c odom|slam|reloc|original|/absolute/custom.yaml`，默认 odom，强制选择项目 overlay。
  四份 YAML 在 `drivers/odin/src/odin_ros_driver/config/`；`control_command.yaml` 仅链接 odom。
  reloc 的地图绝对路径由用户手动维护，`drivers/odin/maps/` 地图及运行/构建产物不入 Git。
  外部 `catkin_ws` 原驱动保留，干净终端只 source 它仍使用原驱动；省略 `-c` 读取`/etc/ros2-ardupilot/navigation.env`的`NAVIGATION_MODE`，缺省odom。
- odom/map保持“地图任务、本地控制”：原生TF为`odom ← map`，`p_local=R_OM p_map+t_OM+b`，`b`为既有extnav原点偏移；任务与采点以FCU中心为准。
  `ControlStatus`区分实际local值和task值；map预览用`fcu_local`避免MAVROS历史`map`命名冲突，点云不另做安装角旋转。
  三策略接受地图航点，普通小重定位用连续偏移/重新投递/重新查线；失效或大跳变撤销任务、本地制动悬停，默认15秒后LAND，不自动恢复旧任务。
  实际模式来自`/odin1/navigation_status`的设备ACK/session及原生TF；未定位禁止起飞。地面切换须未解锁、落地且无任务，重启原运行的飞控/避障，保留航点数值。
  安装外参入口`config/odin_extrinsics.yaml`，逐机以`ODIN_EXTRINSICS_FILE`覆盖；仓库零角默认保留。**TODO：map与Odin-Tag组合未适配，非零安装角Tag apply继续拒绝。**
  设计与维护入口见[导航说明](docs/odom-map.md)，验证及限制见[Task6报告](agent/report/report-2026-10-19-task6-odom-map.md)。
- 机载性能配置在开发机/refresh/new已落实：`config/mavros/aircraft.launch.xml`及9插件白名单由独立/整栈机载入口使用，
  复用已选MAVROS安装的apm参数，不再依赖手改系统插件文件；`gps_status`不在白名单。直连FCU遥控器手飞不依赖ROS的rc_io/manual_control；未改RC/模式参数。
  开发机SITL保持原配置。项目Odin的odom/reloc关闭raw DTOF、诊断CSV和传感器展示TF，slam保留展示TF，original不变；地图TF独立保留。
  开发机/refresh修改前备份在驱动config/backup/20261019-flight-trim，恢复不得跨机覆盖地图路径；new旧外部驱动保留。hp-desktop已同步源码并原生构建项目驱动；外部catkin_ws保持。
  源码级主机精简只在agent隔离副本评估，未合入正式驱动；HIGHRES_IMU请求仍由原控制器保持100Hz。
- 避障停止脚本已在开发机、refresh 更新：清理本机全部 domain/工作区的避障启动器、规划器/桥及已捕获的后代，
  保留孤儿身份，逐级 INT/TERM/KILL，复核无活动残留才返回成功；不停止飞控栈。脚本更新无需重建。
  new与hp-desktop均已同步此停止脚本；隔离验证与范围见[修复报告](agent/report/report-2026-10-19-stop-avoidance-fix-refresh.md)。

| 用途 | 当前入口 / 说明 |
| --- | --- |
| 地面站安装、启动 | `scripts/ground/setup_ground_station.sh`、`scripts/ground/start_ground_all.sh` |
| 机载构建 | `src/onboard_control/deploy/build_onboard_control.sh`；`--verify` 追加测试及隔离 smoke，不重启服务 |
| 飞控启停 | `scripts/onboard/start_onboard_control.sh`、`stop_onboard_control.sh`；启动 `--check` 不启动组件 |
| 视频启停 | `scripts/onboard/start_onboard_video.sh`、`stop_onboard_video.sh`；后者 `--restart` 只重启视频 |
| 修正启停 | `scripts/onboard/start_onboard_correction.sh`、`stop_onboard_correction.sh` |
| 避障构建、启停 | `src/onboard_control/deploy/build_avoidance.sh`、`scripts/onboard/start_avoidance.sh`、`stop_avoidance.sh` |
| Odin/extnav 单独启动 | `scripts/onboard/components/start_odin.sh`、`start_extnav.sh`；读取 `onboard.env`，不启动其他组件 |

地面站安装、机载构建（含verify）和避障构建共用`scripts/lib/build_dependencies.bash`，编译前仅对缺失的Eigen/JSON开发包执行sudo/apt；全量依赖安装器复用同一清单，`deps-check`保持只读。开发机与hp已同步验证；refresh/new下次同步须带上该共享文件及部署脚本，无需重启运行服务。

部署细节查 [机载部署指导](src/onboard_control/deploy/ONBOARD_DEPLOYMENT.md)、
[视频说明](video_service/README.md)、[修正说明](correction_service/README.md)、
[避障桥说明](src/avoidance_bridge/README.md)。旧根目录脚本和 `start_drone/` 入口已迁移。

## 逐机部署差异与待同步项

默认连接 `ssh drone-refresh`；不可达时停止并询问用户，不自行切换飞机。
最近核对别名为 refresh → `.186`、new → `.169`；历史名称/IP 曾变化，连接时核对目标。
两机工作区均为 `/home/nvidia/ros2-ardupilot-mavros-control`；地面笔记本为 `ssh hp-desktop`（`.101`）。

| 目标 | 最近确认的基线 | 下次操作必须注意 |
| --- | --- | --- |
| refresh | Task6 / 3.5；项目Odin、C++ extnav、控制/接口及桥均原生构建；MAVROS2.16.0修复overlay保持 | 航点启动保留DOB已原生Release部署，并在3架次实飞bag确认运行及扰动连续；按模式jerk10/8已安装回读，39项控制测试及隔离smoke通过，飞控栈保持停止；其余控制/外参配置保留，残余上拱和梯形相位转折仍在。自动安装俯仰启用3秒窗口，今日完整bag已锁定约-25.214°；原-22°回退/杆臂/roll/yaw保留。最近配置navigation.env=map（reloc.yaml）；四次未解锁模式切换及约100Hz串口基线通过；直线3架次已复核，自主避障实飞仍未验收。现场地图已更新为项目`drivers/odin/maps/20261020_103327/map_20261020_103624.bin`；外参`/etc/ros2-ardupilot/odin_extrinsics.yaml`。现场Git HEAD仍旧，以源码哈希/3.5实际接口为准，勿reset/clean |
| new | Task6 / 3.5；项目Odin、C++ extnav、两仓库原生Release重建；按模式jerk10/8与航点保留DOB已部署 | 2026-10-20 domain230未解锁全链READY，自动安装pitch锁定-19.8908°；3秒学习，roll/yaw=0、杆臂(.06,-.03,.05)，逐机YAML保留0°回退。保持odom，reloc地图尚未配置；仅自主任务输出jerk10/8，其余模式默认关闭，74项其他控制值和相机/Tag配置保留。旧避障方向限额迁至原生规划器，速度1/.2、加速度.35/.15。收尾飞控/避障停止，视频/修正active；现场Git HEAD仍旧，以源码哈希和安装接口为准 |
| hp-desktop | 2026-10-21 主工程2f340b3、独立避障8b4630c及开发机当前源码差异已同步；3.5接口、六个主工程包、独立避障与项目Odin原生Release重建 | 地面环境、隔离控制smoke和39项规划参数回读通过；备份`/home/scq/backups/code-sync-20261021/`，TODO及修正测试现场改动保留。无外部vrpn_mavros，C++ extnav仅同步维护源码，未新增机载服务；旧回退参数测试预期仍有偏差，见同步简报 |
| 开发机 | Task6 / 3.5；odom/map三策略、地面站/WebSocket、原生双树与连续参考，已重建 | Python175、C++32及导航DDS9通过；三策略map SITL完成。旧Task6末端异常的4Hz遥测冲突已查明，恢复100Hz后odom/map本轮均无失败；严格净空及实飞未验收 |

- Odin迁移：开发机、refresh及new已完成独立/完整机载构建；两机隔离domain230全链READY且armed=false，测试后飞控栈全部停止。
  refresh的`ODIN_CV_BRIDGE_SETUP`指向项目`drivers/odin/cv_bridge_ws/install/local_setup.bash`，OpenCV仅4.8；
  旧`ODIN_OVERLAY_SETUP`字段已不参与项目驱动选择。new已独立原生构建兼容cv_bridge及项目驱动，OpenCV固定/usr/lib/cmake/opencv4的4.8，不选该机/usr/local的4.14；hp-desktop已用系统cv_bridge原生构建项目Odin，勿复制refresh安装产物或逐机环境。
  后续Task6已完成真实重定位静态核对，绝对定位精度和实飞仍未验收；最初迁移见[迁移报告](agent/report/report-2026-10-19-project-odin-driver-migration.md)。

- **new 与 refresh 相机配置不能互相覆盖。** refresh 使用 UQ212 / Tag 边长 0.099 m；
  new 使用 Wasintek 1920×1080@30 / Tag0 0.170 m，修正活动目录为
  `/etc/ros2-ardupilot/correction-config`，由 `correction.env` 的 `CORRECTION_CONFIG_DIR` 选择。
  仓库根 `correction_service/config/` 默认属于 refresh；命名相机档案不会自动切换运行配置。
- refresh 系统与活动 MAVROS overlay 均为 **2.16.0**，活动路径为
  `/home/nvidia/mavros_param_fix_2.16.0_ws/install/local_setup.bash`，由 `onboard.env` 选择；
  保留项目参数重试修复和原插件裁剪。旧 `mavros_param_fix_ws` 已退出生产，不能再加载。
  new 于2026-10-20再次实查系统为 **2.14.0**；不复制refresh的overlay或环境文件。
  项目MAVROS profile须继承所选node.launch的namespace默认值：2.14为mavros、2.16为空。开发机/new已修正；refresh已随本次控制器同步带上该共享XML修正，2.16静态启动检查通过。
- **MAVROS参数加载回归已在refresh修复**：上游 `63f7392` 禁止全局节点重命名时也屏蔽了YAML，
  `thrust_scaling=NaN`会丢弃姿态目标；系统10月10日升级2.16.0后仍被旧overlay覆盖。
  当前使用2.16.0内置官方修复，保留命名/所有权修复；实读推力缩放1.0，隔离UDP姿态输出与重试测试通过。
  安装器按2.15.1/2.16.0 ABI选择源码与独立缓存，2.15.1额外回移官方参数修复。
  下次升级必须核对活动库与实际参数，起飞成功不等于姿态输出有效。详见[修复报告](agent/report/report-2026-10-13-refresh-native-mavros-fix.md)。
- 飞控环境为 `/etc/ros2-ardupilot/onboard.env`，视频优先使用同目录 `camera.conf/lens.conf`。
  串口和 overlay 按目标机核对，不照搬历史 `/dev/ttyTHS*`。systemd 不依赖外网校时；
  `network-online.target` 未必已有 Wi-Fi IPv4，视频入口保留 DDS 创建前的本地路由等待。
- refresh 的 video service 已换 Wasintek，相机当前为 `/dev/video2`；修正使用的 UQ212 为 `/dev/video0`。
  内参标定源码 `video_service/calibration/wainstek_cam_calib.py` 与现场 `~/camera_int_calib/wainstek_cam_calib.py`
  已按用户要求回退到最初备份，仅 `CAMERA_ID = 0` 改为 `2`；其余逐字节不变，原始1920×1080@30及保存行为保留。
  用户明确不保留其他适配；原版在项目OpenCV4.6下的检测参数构造崩溃也保留，不能把旧版本解码/GUI通过结论套用到当前脚本。
  尚未完成新镜头实测标定；new/hp-desktop未同步，不覆盖逐机镜头或内参。
- 独立 USB Jetson（`192.168.55.1`、序列号 `1424324322770`）不是现役飞机，基础环境已构建，
  无外设、后续补丁未逐项同步。新机事项查 [检查清单](src/onboard_control/deploy/JETSON_NEW_MACHINE_CHECKLIST.md)；
  厂商 OpenCV 4.8 必须使用配套 cv_bridge overlay，避免与系统 4.6 在 Odin 同进程混用。
- **此前回退的历史记录（已被本次3.4恢复部署替代）**：2026-10-12同步前186完整备份：`/home/nvidia/ros2-ardupilot-maintenance/sync-from-169-20261012-1536/`。最初规划器源码、修改、build/install和额外标定目录均保存在其中。此前回退任务与“回退并同步飞机代码版本”曾并发构建规划器；用户指定由后者接管，最后以其干净构建、测试及哈希核验为准。原主工程控制代码已同步并构建，当时交接仅补齐报告/记忆。
- 此前回退的186隔离smoke（历史）：interface=3.3、armed=false、setpoint_messages=0；控制单测通过。修正包5例中4通过、1失败：仓库Tag0=0.0465 m，而测试固定要求0.170 m；逐机活动配置未改，不能为过测覆盖标定。三个机载服务未启动，真实设备闭环未验收。
- 交接最后再次核对逐机标定和 `/etc` 哈希全部匹配；现场随后已有独立Odin进程，静态启动检查因此拒绝重复启动。没有停止或接管该现场进程，不应把服务unit停止等同所有驱动均未运行。

- **两机同名定位流仍须隔离**：同时运行domain0/SUBNET的Odin时会混合定位与点云。2026-10-13收尾refresh无Odin，new独立Odin普通连接后原始/down云各1发布者；此前缺失down发布者已由SDK集成解决。正式实机避障闭环仍未验收。
- 两机均已迁移C++ extnav和原生避障；new本次同步备份在`/home/nvidia/ros2-ardupilot-maintenance/new-sync-20261020/`，桥安装器另备份在`/home/nvidia/backups/extnav-cpp-20261020-160647/`。
  new部署与静态验收见[同步报告](agent/report/report-2026-10-20-new-aircraft-code-sync.md)。
  refresh本次旧源码、逐机配置和可恢复安装备份在`/home/nvidia/ros2-ardupilot-maintenance/refresh-native-mavros-20261013/`，
  两仓库现场改动另保存在命名Git stash，Tag CSV和无关现场文件已恢复；勿将stash里的旧避障代码重新覆盖生产。

## 不能误改的业务语义

- 机载 `ControlStatus` 是权威；同一时刻只有一个租约持有者，5 Hz 心跳续租。
  命令 TTL 使用双方相对单调时间基准，不比较绝对日期、不依赖 NTP；重复/乱序/过期命令拒绝。
  失联先悬停，默认 10 秒后 LAND；LAND 成功以实际解除武装为准，不能用 SetMode ACK 替代。
- 原始姿态控制必须确认 `GUID_OPTIONS` bit 3 的归一化推力语义；`hover_throttle=0.22`
  仅为回退值，运行时用 `MOT_THST_HOVER`。消息频率 ACK 不等于实测达到 100 Hz；
  必须请求 `EXTENDED_SYS_STATE(245)` 才能可靠检测其他来源的起降边沿。
- 航点是当前任务系（odom/map）的绝对ENU，上传时锁定；起飞高度、手动速度及返航仍为FCU local。CSV 为 `index,x,y,z,yaw`，文件 yaw 用角度、GUI 内部用弧度。
  默认“梯形速度 + 轨迹 PD+DOB”；策略、参考生成器、跟踪器在同一武装周期锁定。
  LAND 在持权且可靠连接时允许幂等重发，不能被普通 GUI busy 或诊断异常锁死。
- FCU 热重启只允许未解锁、落地、待机且无任务；必须观察启动时钟回退，恢复原点、
  参数和新鲜遥测后才成功。遥测恢复/重启 ACK 不等于控制链恢复，不自动恢复任务。
- 上位机映射唯一在 `ground_station_core/upstream/mapping.py`；自动低电量/异常返航目前
  只在 WebSocket 在线的仿真会话启用，实机仅提示。机库判定仅为 ENU 阈值；
  `cameraAngle` 尚未实现。`pointNo` 用任务点索引，`photoNo` 原样用于文件命名；
  08/09 使用视频服务实际结果，占位路径不代表已生成媒体。协议详见 [对接说明](docs/上位机-WebSocket%20对接.md)。
- 视频使用单次 V4L2 采集，FFmpeg 同时推流和录像，截图从 RTSP 获取；RTSP 固定 TCP。
  MediaMTX 使用系统 `/usr/local/bin/mediamtx`，当前部署 v1.20.0，仓库不带二进制。
  直播尚未启用去畸变；重复序列号摄像头使用 by-path 区分，不能依赖 `/dev/videoN`。
  RTP/JPEG 的 MJPEG 宽高上限为 2040；HP 的 DRI 兼容只转码 RTSP 分支，录像保持原码流。
  MJPEG兼容分支存在显著CPU转码成本，Python控制层重写无助于消除此项；目标飞机是否触发仍待实查，不能按仓库默认H.264推断现场。

## AprilTag / Odin 当前基线与风险

- extnav权威源码为`correction_service/extnav_patch/`的单线程C++/固定尺寸数学；公共入口仍`ros2 run extnav_bridge extnav_to_vision_pose`。
  安装器以Release迁移外部`/home/nvidia/vrpn_mavros`包并备份旧source/build/install；开发机隔离验证、refresh/new生产安装已完成，hp-desktop已同步维护源码，但无外部vrpn_mavros，未新增该机载安装。原Python仅保留作比较参考。
  raw全频、vision/辅助timer仍40/100Hz，CAS/session、冻结样本和计数保持；corrected保留原门控，两路辅助输出无读者时跳过构造/publish，有读者恢复，control计数仍推进。控制器消费MAVROS EKF。
  ARM的7,880组数值对照最大误差约7.1e-15，三套构建/安装入口公共契约通过；全服务地面生产CPU约68.78%→10.75%单核，持续订阅切换源时序正常。有限窗口不保证全部时序极值改善，未实飞验收。
  回退与边界见[C++重写报告](agent/report/report-2026-10-19-extnav-cpp-rewrite.md)，成本原因见[成本分析](agent/report/report-2026-10-19-extnav-cost-analysis.md)。
- C++ extnav已新增启动时自动安装俯仰：`auto_installation:true`后，未解锁静止默认3秒，比较Odin与FCU IMU的机体系重力方向，仅学习pitch，roll/yaw/杆臂保持配置；不要求机身摆平或两世界yaw相同。收敛后移除校准订阅并锁到节点重启，不写YAML；缺输入/移动/active Tag期间保留手工回退。节点开关默认false，项目外参YAML默认true，旧逐机YAML需补参数。
  开发机Release双目标及隔离安装入口验证通过；refresh/new均已原生重建、正式入口ARM隔离7例通过，并在逐机YAML启用自动补偿；refresh完整bag约-25.214°，new未解锁静态锁定-19.8908°，pitch输出约0.54°、同期FCU IMU约0.58°。hp已同步维护源码，未新增外部extnav部署。refresh回退备份在`/home/nvidia/backups/ros2-ardupilot-maintenance/auto-pitch-20261020/`。实际角及锁定状态见现有ExtnavCorrectionStatus；非零安装角Tag限制保持。估计是相对重力俯仰，不能独立消除两IMU水平标定偏差；原3.3°残差样本对应约-25.3°安装估计，该历史例不替代当前两机各自启动学习。用法及待部署项见[自动俯仰报告](agent/report/report-2026-10-20-auto-installation-pitch.md)。
- **Odin 近身拖点尚未解决，不能把接收端删点当成内部算法前过滤。** refresh 实际运行
  驱动 0.14.0 / SoC 0.14.2 / SLAM 0.13.1；`cloud_slam` 由设备输出，公开驱动/SDK 未查到
  算法输入最小距离或 FOV 裁剪参数。`cloud_raw_confidence_threshold` 只过滤主机发布的 raw 云。
  驱动已有 `sendimagemask` / `image_mask_abs_path` 向设备上传 PNG 的接口，当前关闭；
  是否作用于雷达算法、黑白定义及生效/清除条件未确认，不得按已验证的点云屏蔽使用。
  厂商 FAQ 承认 0.4 m 内拖点，建议近距感知用 raw 云并避让结构遮挡；raw 云仍需实际对比，
  不保证消除测距干扰。此次未改两机配置，详见 [驱动检查报告](agent/report/history/report-2026-10-10-odin-near-field-driver-audit.md)。
- 链路：raw Odin IMU → 公共 SE(2) corrected Odin IMU → 杆臂转换后的 `/extnav/pose_fcu`
  与 `/mavros/vision_pose/pose` → `/mavros/local_position/pose`（最终 FCU EKF）。
  corrected 不是 FCU 中心，也不是 EKF final；有效分支已移除多余固定 `+T_xy`，z 仍保留局部约定。
- extnav 唯一维护 active correction，以 Odin session + revision CAS 更新。
  停止修正服务或 clear 窗口不清 active；Odin 断流、时间戳回退或 frame 变化使其失效，不能重放旧样本。
  ACK 未决锁存 `application_unknown`，只允许同一候选幂等对账/重试。
- 首次候选保持 `planar_xy_yaw(H_WI * inverse(H_OI))`，**不能改为单点 `P-RQ` 重锚**。
  从第二个不同 Tag 起，滑窗才从原始 P/Q 点对按绝对逆方差重算 SE(2)，不累计增量或平均单 Tag yaw。
  同帧多 Tag 联合估计、自动航点触发未实现；相机/400 Hz raw 订阅只在任务期间启用，idle 释放。
- **修正服务性能待优化**：当前仍为四线程Python executor，实体共用默认互斥回调组，采样/apply另有任务线程。
  本地真实DDS同400Hz输入对照已证实单线程可显著降低CPU；正式修改与事务回归尚未实施。后续原生化应重点评估400Hz采样/同步前端，
  OpenCV检测已原生、滑窗/PnP不是主要热点；不要直接将30Hz相机发布改10Hz，现有时间戳门会降低实际检测频率。
  视频与遇障悬停的收益边界同见[机载性能评估](agent/report/report-2026-10-20-onboard-performance-assessment.md)，桌面结果不等于Jetson验收。
- Task32 已修复 OpenCV 与 AprilRobotics 官方图案的 Tag 基变换，并撤销错误相机 `Rz(180°)` 补偿。
  配置 Tag +X 指官方图案上方、+Y 左、+Z 上；方向改动必须用官方图案和独立物理真值验证，
  yaw 正确或自生成合成闭环不足以证明位置方向正确。
- UQ212 原生采集 MJPEG 1920×1080@120，解码/发布默认限 30 Hz；实际内参采用 30 张实测结果，
  状态仍 `calibrated_pending_independent_validation`。外参是机械近似
  `t=(0.04657,0.02897,-0.06726)m`，尚未独立标定验收。现场 Tag0 `(0,0)`、Tag1 `(0,1.8)`
  同向、边长 0.099 m；部署保留用户 CSV，不复用旧内外参下的窗口。
- **refresh安装角已补偿，精密标定仍未完成。** 使用逐机`pitch_cam=-0.3839724354`（约-22°）与既有杆臂，extnav以完整姿态及角速度转换FCU中心。
  当前静态raw pitch约-24.70°，补偿后extnav约-2.70°，FCU EKF约+0.60°，残差约3.30°；不能把任务给出的22°估计宣称为精密标定。
  EKF外部姿态不直接融合roll/pitch，GUI近零不证明extnav准确；new已部署自动安装俯仰，当前机械安装与旧零角基线不同。旧20°分析仅作历史参考。
- **绝对定位/航向精度仍未验收。** 短时稳定、检测率提升、用户两点 apply 成功和 EKF 跟随
  不构成真值证明；10° tilt / 45°跳变门只是用户授权的地面实验门限。
  尚需精测 Tag 世界位姿、明确物理参考点、重标外参及跨位置/跨 session 独立验证。
  当前 PoseStamped 不传 MAVLink estimator reset counter，应用修正仍有 EKF 瞬态风险。

## 避障当前基线与验收边界

- 原生 `search.vertical_distance_weight` 公共默认10、裸core默认0，惩罚累计上下路程；refresh于2026-10-20实查现场为80、aXY/aZ=1/1、Z `[0.2,1.8]`、100s/100m重规划，同步必须保留（Task5的30及Z[.6,1.4]仅为旧现场）。普通原语和终端shot均计入；高度差启发项仍关闭。tie因子重复加1已修正；当前公共调优YAML及refresh/new均lambda2（有效倍率2.0002），裸core默认1，不能混淆。加权启发式/位置hash仍不保证最优。真SLAM权重扫描lambda2是质量/耗时折中，但尚未闭环验收。

- 开发机正式桥与独立手动入口共用原生 dual_tree、目标/时间/位移/旧路径碰撞触发，
  实际状态重规划直接搜索首段加速度，避免0.2s重规划不断重播0.8s匀速首段；仅旧time_index入口保留init行为。
  service_only、latest_observation及桥专属规划profile已删除。
  当前公共调优YAML：safe_distance=.35m、vXY/vZ/aXY/aZ=2/2/2/2、horizon100m、resolution/voxel_size=.1m、max_tau=.6s、
  init_max_tau=.8s、collision_step=.05m、search_budget=.5s、input_timeout=1s、盲区0、tree_period20，
  下界[-20,-20,.2]、上界[20,20,1.8]；.2s/1m重规划。裸core和逐机现场配置不同，修改或同步勿互相覆盖，实际发布不保证严格5Hz。
- 直线模式control.yaml的straight_max_velocity_xy/z=1/.2、straight_max_acceleration_xy/z=.35/.15；
  原方法参数可显式覆盖，默认继承公共直线组；遇障悬停正常飞行及正常到点复用同组参数（梯形减速XY/Z=.40/.18）。自主避障独立使用原生search组：
  裸core方向默认水平速度范数1m/s、垂直绝对0.2m/s、水平/垂直加速度2m/s²；实际以所选YAML及原生解析结果为准，当前公共调优值为2/2/2/2，
  search.max_vel_xy/max_vel_z/max_acc_xy/max_acc_z在普通原语与shot的原时间内生效。
  旧scalar CLI填未显式给出的方向值，方向CLI优先。PlanResult/AvoidanceResult携带最终解析限额；遇障悬停仅紧急制动继承原生加速度，HORIZON制动预留仍使用原生限额。
  遇障悬停从首次等待时的实际p/v生成恒减速度参考，零速后保持预计停点；重复阻挡不移动起点或重置等待计时。
  正常参考复用直线组与误差制动策略分离已在开发机、refresh原生部署（代码541b8b4）；new已同步并原生重建；hp-desktop也已同步并原生重建onboard_control。
  超过0.8m位置误差的制动判断仅用于自主避障；遇障悬停与普通直线一样继续跟踪正常参考，不因参考领先自动停车。
  开发机SITL无障碍遇障悬停/普通直线实际XY峰值1.06359/1.06372m/s；正常参考一致，仍非实际硬限速。
  运动中遇障制动参考2m/s²，约.604s降至实际XY<.15m/s；阻挡无拍照，移障可恢复。
  设计与验证见[遇障悬停直线参数适配](agent/report/report-2026-10-13-hover-straight-shaping.md)。
  执行层重复包线和统一时间缩放已删除；用户确认PD+DOB输出限幅保持4/4用于纠偏。
  高加速度/低垂直速度时按可行区间采样，常规125候选保持；窄Z分支补精确停速候选，最多150，max_tau=.6保持。
  实际起点超速时允许有界减速首段，保留真实初速并检查位置/碰撞/加速度；后续仍满足正常速度包线。
  不得把名义速度限额理解为首段已在包线内或实际飞行硬限速。
  普通原语已有净空证书；shot/几何服务仍离散，不能声称全曲线连续净空。
- 首init短段留在起点cell时保留临时候选，不覆盖root空间hash；避免闭合root误裁剪短coast，
  该兼容分支保留20个初始时长；实际状态流已不走强制coast，超速恢复也保留同cell临时候选。
  常规同cell异速仍沿用position-only hash，不能保证所有滚动情形。
- **Task5已在refresh、new与hp-desktop同步两仓库并原生重建。** 同步时同时重建独立库、guided_interfaces、
  avoidance_bridge和onboard消费端，使用build_avoidance.sh；旧Task38服务/包线与新安装不能混用。
- `/odin1/cloud_slam_down`现由Odin host SDK在`cloud_slam`输出端直接生成，仅含x/y/z FLOAT32、point_step=12；不再携带RGB，新版不创建或发布原始cloud_slam，原9行发布/初始化/声明代码保留为注释，取消注释后重新构建即可恢复；内部临时XYZRGB转换保留。项目选定YAML顶层`cloud_downsample`默认enabled=true、voxel_size=0.1、robot_radius=0.4、center_mode=auto、debug_enabled=false；SDK实际读取启动传入的config_file。new已改用项目Odin配置；外部catkin_ws原驱动仍保留。不再同时启动`/home/nvidia/down_samp_exe.py`，避免双发布者；原Python保留备用，独立避障库未改。
- SDK已移除auto的1m距离启发式：auto/odom只用有效Odin里程计；缺少/非有限球心时只降采样，不猜原点，origin才显式使用(0,0,0)。有限正voxel、有限radius、合法模式在启动时严格检查；非法值仅关闭新增功能并报错。XYZ的NaN/Inf全部跳过；降采样不读取或保存RGB。球面删除、字节键排序及空结果不发布保持。GCC过滤函数局部禁FMA保留；负零符号位微小差异保留。上游剔除仍可能删除真实近障，二次盲区保持关闭；这不是设备内部SLAM的输入过滤。
- 调试需要将所选YAML的`debug_enabled`设true并重启Odin，再`ros2 topic echo /odin1/cloud_slam_down/debug std_msgs/msg/String`；默认没有调试发布器，无订阅时不计时或格式化。项目`drivers/odin`已集成原修改版SDK；外部`/home/nvidia/scq/projects/odin_ros_driver-modified`及私有远端`LostPatrol/odin_ros_driver-modified`仅保留既有副本，不再作为本项目启动源码。
- new曾在正常关闭后重连出现USB可枚举但设备无下行数据，原驱动二进制也复现；用户冷启动恢复；本次补充修复后用户要求普通重连，实际Odin连接成功且有原始/down云，不能据此认定旧重连问题根因已消除。不要把发布器存在当成点云有效；后续若复现先核对实际数据与Odin连接状态，不能靠改过滤器或自动重置设备掩盖。仅down发布证据与new待生效状态见[关闭原始云报告](agent/report/report-2026-10-13-task2-down-only.md)；仅XYZ输出证据见[RGB移除报告](agent/report/report-2026-10-13-task2-xyz-only.md)；校验与auto修复见[校验与auto修复报告](agent/report/report-2026-10-13-task2-cloud-validation-auto-fix.md)，首次迁移见[SDK集成报告](agent/report/report-2026-10-13-task2-down-samp-exec-inte.md)。

- 三入口均CLI→公共planner.yaml→原生默认；缺失/坏YAML告警回退，启动时复制，修改须重建/重启。
  正式点云直接来自/odin1/cloud_slam_down，桥默认将/mavros/local_position/odom中的实际FCU中心
  转换至规划frame并转发/avoidance/odom。手动给同一节点/avoidance/goal，地面站经onboard投递持久目标；
  独立odin.launch默认仍用Odin参考点，其linear twist已是世界系，入口默认odometry_twist_in_body=false；
  MAVROS odom仍为body twist，正式桥保持true。比较输出须统一参考点、状态及目标坐标。
- 桥只做坐标、目标与结果适配，不建占用/射线/自由证据地图，不逐格拒绝unknown、不做地图快照竞态重试。
  双树按帧轮换；未观测区域可通过（用户接受），不存在动态预测或表面包络保证。
  输入/schema/时效及碰撞检查由原生规划器负责。preview仅限量转发native filtered_cloud。
- SetGoal支持target_id、幂等与active/cancel，单个业务航点设置一次；失败投递可同ID重试。
  原生PlanResult有session/target_id/goal_revision/sequence，桥ACK确认原生revision后转发，旧目标/坐标结果拒绝。
  plan_sequence=0是能力心跳或本地投递失败，不得当作目标已接受；原生失败结果仍是非零有序流。
  原生idle/近目标每秒发布空target身份心跳，不搜索，用于单独重启后的session恢复。
- onboard仍唯一执行权威。自主模式不周期PLAN/VALIDATE，精确多项式按原段尺度求p/v/a，
  仅在延迟对应前缀内按实际FCU位置匹配，同位置回返按速度区分；0.08m/0.08m/s基础接入容差
  叠加原生加速度与时延余量，成功新段可运动中更新。自主PLAN首条可接入结果即恢复，不再要求停稳及重复结果窗口。
  自主避障航点准备/启动两处保留已学DOB扰动及previous_input，切增益按q=d−Lv重表达；其他模式保留原reset。
  REACH_HORIZON只作中间参考，无续段时按既有制动参数提前等待，不触发航点完成/拍照。
  完整曲线接近终点可执行完并保持，不依赖持续新结果；原生输入失败和能力失联仍撤销。
  输入故障恢复后原生允许一次实际状态规划，避免近目标0.3m门永久阻断恢复。
- 遇障悬停仍查实际FCU到下一业务航点的完整直线，使用同一原生地图；直线策略不依赖规划器。
  CHECK_LINE原停稳/连续通畅窗口、15秒LAND及实际到点、低速停留、拍照/推进保持；完成/取消/换点撤销旧goal，地图保留。
  2026-10-14删除遇障悬停误差制动，开发机/refresh DDS均8/8，SITL运动中遇障恢复通过；部署见[同步报告](agent/report/report-2026-10-14-hover-no-error-stop-sync.md)。
  桥单独重启后仍可从完整CANCEL身份撤销原目标。
- 实机仍仅支持未应用Tag的extnav基线：FCU状态/目标减权威杆臂T，返回多项式常数项加T；
  世界点云不按安装仰角另旋转；本次bag地面拟合倾角仅0.122°。Task6已接入无Tag时的可配置安装姿态；Tag有效分支、前移约2cm及真实雷达中心精密外参仍未适配。
  extnav身份2Hz发布使用独立2s接收超时；FCU时效不变，主动失效或身份变化仍立即处理。
- 净空仍按所选safe_distance比较0.1m体素质心（公共YAML .35m、裸core .25m），无voxel/采样补偿；不保证每个输入点或真实表面包络。
  Task5最终lambda1的两次自主绕障均26/26规划成功、约7秒完成航点/拍照/LAND，上升约5.6cm；
  实际表面净空.29570/.29367m仍未过既有>.35m标准，不能称整体验收通过。自主阻挡恢复通过（漂移.04410m、恢复3.01385s）。
  自主启动保留DOB的单变量对照把上升29.8cm降到2.25cm；不等于最终配置严格同高。
  原生最终对原bag的107条成功计划固定输入子集全部成功（不含原69条失败），开发机耗时中位34.34ms/max142.52ms，比旧版约4.38ms明显增加；
  refresh同输入107/107成功，中位43.994ms/max176.898ms，未叠加全传感器长期负载。
  加权搜索仍可能次优，真实表面/机体净空、长期负载和实飞闭环未验收。
  refresh现场权重30及Z[.6,1.4]已运行回读；静态测试结束飞控栈/规划器均停止。21项配置哈希保持。
  两机备份分别在refresh的`/home/nvidia/ros2-ardupilot-maintenance/task5-avoidance-20261013/`和hp的`/home/scq/backups/task5-avoidance-20261013/`。
  参考限额不保证实际硬上限；旧遇障悬停参考加速2m/s²曾实测XY最大1.4054m/s，现正常参考改为直线组。控制输出4/4保持用于纠偏；恒减速是参考限额，不能当作实测加速度硬限幅。
  用户确认当天没有直线对照；bag缺FCU实际输入/执行参考/控制输出，不能完整还原历史闭环或同状态复核原69条失败。
  一键RViz使用Odin输入，其参考点与原FCU适配不同，不是上述107条同输入对照，也不能据此展示DOB修复后的实际响应。
  开发机Task5一键回放使用用户指定Z[.2,1.5]，当时实机Z[.6,1.4]是历史现场；refresh当前实查Z[.2,1.8]。回放仍有高度越界/净空/搜索失败，见[高度对照报告](agent/report/report-2026-10-13-task5-rviz-height-02-15.md)。
  原因与未解问题查[分析报告](agent/report/report-2026-10-13-task5-avoidance-analysis.md)，
  实施/逐机结果查[执行报告](agent/report/report-2026-10-13-task5-avoidance-fix.md)。

- **按模式输出限jerk已在开发机、refresh和new原生部署**：`max_jerk_xy/z=10.0/8.0`，`jerk_limit_modes`的hover/motion/straight/avoidance/obstacle_hover/failsafe_hold六开关仅avoidance默认true。航点任务按策略选择，含启动速度等待及规划/制动等待；任务结束末点保持归hover。复用原PD+DOB输出限幅，不新增控制器；开关不清空DOB或上一输入，旁路仍记录倾角限幅后的命令。LAND/解除武装/实际停控清理历史，硬约束优先，关闭瞬间不保证jerk连续。开发机39项C++及24项真实node DDS测试通过；两机均39项控制测试/安装参数回读/隔离smoke/静态启动检查通过，逐机74项其他控制参数及标定保留。refresh原8/8、new原8/3已改10/8；两机飞控栈保持停止，new视频/修正继续active；hp-desktop已同步并原生安装本次模式开关及默认值。SITL显示起飞接管制动改善，但直线/遇障停车姿态指令更陡，停车时间未明显缩短；自主标准及激进场景仍未通过净空，不能当作避障修复或实飞验收。详见[实现、对照及部署](agent/report/report-2026-10-20-mode-selective-output-jerk.md)。

- **航点接续保留DOB已在开发机/refresh/new修复并原生Release安装**：准备航点和速度门控通过两处均保留扰动及既有输入，按当前速度重算内部观测状态；LAND/解除武装/停控清零保持。真实node隔离正负对照分别验证两处清零已消失；refresh三架次实录已确认实际加载；当时原机jerk8/3、开发机Z=8，仅供该批实录解释，当前已切换按模式10/8。new已同步；hp-desktop已同步并原生安装。实录启动dZ约.26—.36m/s²未清零；水平最大正高度误差由旧13.6cm减至2.3—6.0cm但弧形仍在，梯形边界快速转折约17—23°/s仍在，制动没有统一增大。第1次较强制动由更大P/D领先反馈解释，且同时转向90°；不能当同条件单因素对照。复核见[实录报告](agent/report/report-2026-10-20-refresh-post-dob-fix-flight-bags-analysis.md)，实现见[修复与两机核查](agent/report/report-2026-10-20-waypoint-dob-preservation-and-aircraft-audit.md)。

- **拱形历史证据需区分飞机**：8/14、9/1正常视频可能为new；今日两机4.6.3提交new=92b0cd78正式、refresh=3fc7011a beta，全库仅版本标记不同。EK3/VISO、推力指数等相同，但内环/IMU标定、当时Python/C++外参桥、安装角及odom/map不同；当前hover .223878/.225926相近，不能直接由参数大小判定故障或互抄调参。旧/新外环同输入输出一致，4.6.3同模型均可上拱，保留DOB能改善该合成工况，尚非实机根因排他证明；FCU vz与高度差分本有不同定义。修复前后4份bag均已离线在bags/且SHA256匹配；new关机前1246项参数与117个原文件在`agent/codex/new-offline-snapshot-20261020-1539/`，refresh新实录及配置证据在`agent/codex/straight-dob-fixed-20261020/`。历史复现见[对照报告](agent/report/report-2026-10-20-straight-history-comparison-and-sitl-reproduction.md)，实录分解见[完整bag诊断](agent/report/report-2026-10-20-straight-full-bag-height-twitch-braking-analysis.md)。

- **new往返实录已离线复核**：`bags/new-20261020/rosbag2_2026_10_20-16_24_47/`完整且SHA256匹配；实际odom、jerk8/3、安装pitch−19.9525°。0.5m起飞峰.6336m；约.4m/vz+.383m/s就接管，首步制动请求−.758被Z jerk限制至−.030m/s²。航点DOB确实保留；出航/返航最大正高度误差1.45/4.77cm，返航另含约6.78cm欠高恢复；爬升最大欠高8.46cm、完成后保持最低.9421m。梯形中部匀速仅.248/.321秒，俯仰边界约22°/s；高度峰谷当步Z jerk不活跃，残余竖直闭环原因未排他定位。仅分析，未改控制/配置；详见[new飞行分析](agent/report/report-2026-10-20-new-flight-performance-analysis.md)。

- **起飞交接仍需专用稳定阶段**：当前target−.1m即接管并宣告成功，hoverζz=.6；jerk关闭后1.5m SITL在z1.403/vz+.537接管仍超调7.26cm，首拍已请求−1.278m/s²制动，理想同状态PD也超调约7.30cm。仅增加低速接管仍超调5.70cm；克隆中只给初始稳定段ζz=1.5降至.146cm，属单轮SITL候选，不是已实施修复/实飞保证。正式方案需FCU上升→PD稳定阶段防重入、近目标低速后保留DOB恢复原增益再报告完成；不得简单让kTakeoff一直保留而每拍重置DOB。证据及限制见[真SLAM与起飞分析](agent/report/report-2026-10-20-real-slam-curves-clearance-takeoff-analysis.md)；生产参数仍ζz=.6。

- Task6三策略map SITL均实际到点/拍照/LAND，未放宽到点条件。原自主“10次入点失败”的主要上游问题已定位为临时UDP14550链路经MAVProxy默认每15秒请求ALL=4Hz，覆盖已接受的100Hz位置消息配置；正式入口仍直连TCP5762，不能据此归因于实机串口控制。
  仅关闭MAVProxy自动请求，保留0.2秒/1米重规划及jerk8/3，odom/map位置实测约100Hz，两轮均零失败、终点保持约0.3秒后完成；map含运动中0.12m重定位。生产算法及实机参数未改。
  原34.16秒/map与11.95秒/odom期间参考反复离开终点，并非连续固定末参考收敛；旧终点子集极值不代表整窗。10仅计数上限/异常标记，不触发控制变化，见[入点诊断](agent/report/report-2026-10-19-waypoint-arrival-convergence-analysis.md)。
  开发机统一SITL入口已固定传入`--mavproxy-args=--streamrate=-1`，由机载统一配置频率；重启地面站加载，无需改控制参数或机载重建。正式TCP/原问题UDP两轮自主回归均约100Hz、零入点失败，new已同步地面启动源码；hp地面端也已同步该源码。
  refresh当前直连ttyTHS1:460800、无GCS转发；45秒静态位置/速度约99.1Hz、extnav辅助约100Hz、vision约40Hz，未见同类持续降频。偶发源间隔60ms保留，不能声称严格恒定100Hz；串口丢包/溢出/解析错误均0。只读观测未改或重启extnav，独立绕障跟踪/净空风险仍保留，见[修复与机载检查](agent/report/report-2026-10-19-telemetry-rate-fix-refresh-audit.md)。

- **原生曲线与净空仍有已证缺口，尚未正式修复**：此前lambda5真SLAM公共配置回放活动窗127成功/19起点净空失败；13/127成功曲线侵入保留的cloud_slam_down输入点.35m净空，最小.319616m，同位置最近质心.354638m；PCL双树逐帧复建与198/198成功地图点数匹配。普通原语连续证书仅保证质心距离，shot/服务还有离散漏检。单条曲线p/v连续但最大a阶跃5.099m/s²，输出jerk10/8不能让参考自动可跟踪；既有10/8激进SITL参考距柱.3344m、实际中心.0641m且未扣机体尺寸。实际换轨只校验新曲线对实际p/v，未保证对旧执行参考p/v/a连续，曾记录10ms内参考速度跳.216m/s。跟踪保护后仍可能等待而未立即要求原生重规划；不能提高增益/放宽门限冒充修复。修复应直接处理原生体素占据几何、全曲线连续净空、a/jerk可实现性及换轨合同，见[真SLAM与起飞分析](agent/report/report-2026-10-20-real-slam-curves-clearance-takeoff-analysis.md)。上述缺口仍未修复，lambda2部署不等于避障验收。
  固定同一146组真SLAM输入、lambda1–5各3轮均127成功/19起点失败，lambda5逐系数复现误差0。开发机lambda1搜索中位327ms/P95约500ms，68/381成功耗时≥490ms；相对5平均路长−5.85%、代价−14.56%，仍有最大Δa3.526。lambda2中位7.84ms/P95 30.57ms，平均路长−3.81%、代价−9.55%；**开发机、refresh、new已仅将lambda5→2，原生安装及39项实际参数回读通过，其余逐机配置/标定保留；hp已于2026-10-21同步，原生构建及39项参数回读通过**。两机飞控/避障仍停止，new视频/修正active。详见[权重扫描](agent/report/report-2026-10-20-heuristic-weight-sweep-c2-design.md)及[lambda2部署与局部过渡评估](agent/report/report-2026-10-20-lambda2-aircraft-sync-local-quintic-assessment.md)。
  平滑方案优先评估原生局部五次窗口：lambda2的127条曲线532处替换，.2/.4/约.54s窗口最大改形仅.48/1.92/3.50mm，但89/87/87条产生原2m/s²水平加速度越限，峰2.32379；仅扩大窗口不能消除此满限额bang-bang边界的固定时间不可行性，需允许局部延时/适当调整边界，再考虑B-spline扩大优化范围。五次需扩现有4系数协议及全链消费者；本轮未实施平滑。用户允许未来平滑后移除控制输出jerk，不把现10/8作为局部方案硬门限；目前六模式开关仍保持。接续p/v/a和连续净空仍需修，不能用仅C2宣称已可飞。
  已有[三次B样条离线效果预览](agent/report/report-2026-10-21-cubic-bspline-preview.md)：同127条lambda2真SLAM曲线，保持原首末p/v/a及时间、位置贴近加jerk平方正则，两档实际三次输出均C2；偏保形最大同期偏移6.332mm、强平滑10.796cm，仍有87/39条超原v/a限额。只有边界硬约束，未加障碍/动力硬约束，不能直接部署；原末a未强置0，跨计划接续未修。三地图仅距离见证，seq785强平滑后输入点距离.358112m；生产和两机配置未变。另有[大转弯补充图](agent/report/report-2026-10-21-bspline-large-turns-preview.md)：真实最大角度主要为末端低速回钩，另用原生库生成米级合成L/U绕行，不能混作真实SLAM；B样条仍保留折返，合成前两例原参考到实体盒仅.327m，净空缺口未修。
  总加速度模长推力与ArduPilot Guided归一化推力入口的angle_boost存在重复倾斜补偿，SITL对照支持其造成部分高度偏高；真实飞控参数及视频约30cm高度差尚未完整验证。ROS原生入口会将旧scalar限额抬高至显式方向限额最大值，不应将scalar=1、方向=2理解为实际上限1。该项重复补偿尚未正式修复。复现条件、限制和证据见[偏轨SITL报告](agent/report/report-2026-10-19-post-obstacle-tracking-sitl.md)。

## 关键历史与资料索引

- 2026-08-20 曾经用户授权改写 Git 历史以移除大视频/二进制，旧提交 ID 已失效；完整改写前备份
  `/home/nvidia/backups/ros2-ardupilot-git-pre-history-rewrite-20260820.tar.gz`。大媒体不提交 Git。
- new 同步前完整现场归档在 `/home/nvidia/ros2-ardupilot-maintenance/new-sync-20261008-1920/`；
  refresh 路径迁移及标定归档在 `/home/nvidia/ros2-ardupilot-maintenance/refresh-sparse-20260918-101142/`。
  其中现场标定不是可删除缓存；其他备份和哈希按对应报告查询。
- 历史任务/报告集中在 `agent/task/history/`、`agent/report/history/`，近期报告在 `agent/report/`。
  历史资料先查 [报告索引](agent/report/INDEX.md)，再按当前任务需要阅读相关报告，勿每次全量加载。
  报告保留当时结论，已撤销的首次重锚、180°外参补偿及旧协议测试数据不能当作当前基线。
  曾有包测试将 Tag0 固定断言为 0.170 m、与默认 0.099 m 冲突；遇到该失败须核对当前配置，
  不把旧通过数量当作本次测试结果。
