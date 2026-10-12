<!-- 只维护当前基线、部署差异和未解决风险；实验过程与数据查阅 agent/report。 -->
# 项目重要记忆

维护日期：2026-10-13。Task38 / 3.4基线保留；两机Odin host SDK已原生集成近点剔除与降采样，并修复auto回退原点及非有限输入；down输出仅XYZ、12字节/点，替代独立Python进程。独立规划器仍保留169搜索/碰撞核心及现场参数。
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

- 开发机及两架现役飞机均为 Ubuntu 24.04 / ROS 2 Jazzy；机载为 Jetson Orin NX / ARM64。
  Python 使用项目 `.venv`。旧 Ubuntu 22.04/Humble 台架数据不再代表现役环境。
- 开发机及两机飞行协议 **3.4**，`guided_interfaces/onboard_control/guided_sim` 包版本 **3.4.0**；Task38共享接口已同步重建，不能混用旧3.3客户端/安装产物。
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

| 用途 | 当前入口 / 说明 |
| --- | --- |
| 地面站安装、启动 | `scripts/ground/setup_ground_station.sh`、`scripts/ground/start_ground_all.sh` |
| 机载构建 | `src/onboard_control/deploy/build_onboard_control.sh`；`--verify` 追加测试及隔离 smoke，不重启服务 |
| 飞控启停 | `scripts/onboard/start_onboard_control.sh`、`stop_onboard_control.sh`；启动 `--check` 不启动组件 |
| 视频启停 | `scripts/onboard/start_onboard_video.sh`、`stop_onboard_video.sh`；后者 `--restart` 只重启视频 |
| 修正启停 | `scripts/onboard/start_onboard_correction.sh`、`stop_onboard_correction.sh` |
| 避障构建、启停 | `src/onboard_control/deploy/build_avoidance.sh`、`scripts/onboard/start_avoidance.sh`、`stop_avoidance.sh` |
| Odin/extnav 单独启动 | `scripts/onboard/components/start_odin.sh`、`start_extnav.sh`；读取 `onboard.env`，不启动其他组件 |

部署细节查 [机载部署指导](src/onboard_control/deploy/ONBOARD_DEPLOYMENT.md)、
[视频说明](video_service/README.md)、[修正说明](correction_service/README.md)、
[避障桥说明](src/avoidance_bridge/README.md)。旧根目录脚本和 `start_drone/` 入口已迁移。

## 逐机部署差异与待同步项

默认连接 `ssh drone-refresh`；不可达时停止并询问用户，不自行切换飞机。
最近核对别名为 refresh → `.186`、new → `.169`；历史名称/IP 曾变化，连接时核对目标。
两机工作区均为 `/home/nvidia/ros2-ardupilot-mavros-control`；地面笔记本为 `ssh hp-desktop`（`.101`）。

| 目标 | 最近确认的基线 | 下次操作必须注意 |
| --- | --- | --- |
| refresh | Task38 / 3.4、Odin仅down新版已安装并验证；Task3 Python extnav按需corrected发布已同步原生构建，ARM/实际订阅进退通过 | 所有测试Odin/extnav/MAVROS已停止，三个unit inactive；下次Odin自动提供cloud_slam_down，不运行独立Python降采样。逐机标定、旧碰撞风险及20°安装角未改 |
| new | Task38 / 3.4、Odin XYZ输出保留；Task3 extnav与refresh同源，已原生构建，ARM/实际订阅进退通过 | 本次实查机载链路进程正在运行（飞行unit仍failed）；Odin仅down新版已安装，但旧进程尚未重启，下次重启后生效。相机/Tag/安装配置保留，CPU目标与尾部时序仍未验收 |
| hp-desktop | 2026-09-18 重建修正接口 2.0，已解决旧 install 缺少 `ApplySavedCorrection` | **尚未同步 Task38、GUI 仿真避障入口及最近点接入，需同步源码及重建接口** |
| 开发机 | 主工程3.4及最近点接入，恢复与169旧算法的兼容服务链并重建；执行层按需统一放慢轨迹时间 | 恢复版已重新验证真实SITL绕行、两种策略恢复和超时LAND；保留旧采样风险，实飞与长时负载仍未验收 |

- **new 与 refresh 相机配置不能互相覆盖。** refresh 使用 UQ212 / Tag 边长 0.099 m；
  new 使用 Wasintek 1920×1080@30 / Tag0 0.170 m，修正活动目录为
  `/etc/ros2-ardupilot/correction-config`，由 `correction.env` 的 `CORRECTION_CONFIG_DIR` 选择。
  仓库根 `correction_service/config/` 默认属于 refresh；命名相机档案不会自动切换运行配置。
- refresh MAVROS **2.15.1** 使用 `/home/nvidia/mavros_param_fix_ws` 的固定版本参数重试补丁，
  通过 `onboard.env` 的 `MAVROS_OVERLAY_SETUP` 加载，系统安装未覆盖。
  new 系统为 **2.14.0**，不能启用该补丁；系统升级后需重新检查补丁与 ABI。
- 飞控环境为 `/etc/ros2-ardupilot/onboard.env`，视频优先使用同目录 `camera.conf/lens.conf`。
  串口和 overlay 按目标机核对，不照搬历史 `/dev/ttyTHS*`。systemd 不依赖外网校时；
  `network-online.target` 未必已有 Wi-Fi IPv4，视频入口保留 DDS 创建前的本地路由等待。
- 独立 USB Jetson（`192.168.55.1`、序列号 `1424324322770`）不是现役飞机，基础环境已构建，
  无外设、后续补丁未逐项同步。新机事项查 [检查清单](src/onboard_control/deploy/JETSON_NEW_MACHINE_CHECKLIST.md)；
  厂商 OpenCV 4.8 必须使用配套 cv_bridge overlay，避免与系统 4.6 在 Odin 同进程混用。
- **此前回退的历史记录（已被本次3.4恢复部署替代）**：2026-10-12同步前186完整备份：`/home/nvidia/ros2-ardupilot-maintenance/sync-from-169-20261012-1536/`。最初规划器源码、修改、build/install和额外标定目录均保存在其中。此前回退任务与“回退并同步飞机代码版本”曾并发构建规划器；用户指定由后者接管，最后以其干净构建、测试及哈希核验为准。原主工程控制代码已同步并构建，当时交接仅补齐报告/记忆。
- 此前回退的186隔离smoke（历史）：interface=3.3、armed=false、setpoint_messages=0；控制单测通过。修正包5例中4通过、1失败：仓库Tag0=0.0465 m，而测试固定要求0.170 m；逐机活动配置未改，不能为过测覆盖标定。三个机载服务未启动，真实设备闭环未验收。
- 交接最后再次核对逐机标定和 `/etc` 哈希全部匹配；现场随后已有独立Odin进程，静态启动检查因此拒绝重复启动。没有停止或接管该现场进程，不应把服务unit停止等同所有驱动均未运行。

- **两机同名定位流仍须隔离**：同时运行domain0/SUBNET的Odin时会混合定位与点云。2026-10-13收尾refresh无Odin，new独立Odin普通连接后原始/down云各1发布者；此前缺失down发布者已由SDK集成解决。正式实机避障闭环仍未验收。
- 两机主工程main已同步Task3功能提交2c3960e，独立库仍621a2dc；此前核心源码核对保持，extnav另核对canonical/生产/实际导入源一致。原相机/Tag现场改动留在工作区，旧同步payload保存在逐机Git stash和维护备份。后续文档提交不改变功能基线。

## 不能误改的业务语义

- 机载 `ControlStatus` 是权威；同一时刻只有一个租约持有者，5 Hz 心跳续租。
  命令 TTL 使用双方相对单调时间基准，不比较绝对日期、不依赖 NTP；重复/乱序/过期命令拒绝。
  失联先悬停，默认 10 秒后 LAND；LAND 成功以实际解除武装为准，不能用 SetMode ACK 替代。
- 原始姿态控制必须确认 `GUID_OPTIONS` bit 3 的归一化推力语义；`hover_throttle=0.22`
  仅为回退值，运行时用 `MOT_THST_HOVER`。消息频率 ACK 不等于实测达到 100 Hz；
  必须请求 `EXTENDED_SYS_STATE(245)` 才能可靠检测其他来源的起降边沿。
- 航点是绝对本地 ENU；CSV 为 `index,x,y,z,yaw`，文件 yaw 用角度、GUI 内部用弧度。
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

## AprilTag / Odin 当前基线与风险

- extnav Python热路径已移除无读者corrected构造/发布，DDS匹配每0.1秒查询；raw保持全频处理，vision/control仍40/100Hz。raw_messages统计处理，corrected_messages仅统计publish，不能再要求相等；latest_msg移除，最终数值快照及CAS/session语义保持。两机已原生部署；同负载整节点CPU30%目标未达、时序尾部全面不退化未验收，C++隔离候选尚未实施，需用户决定。详见[Task3报告](agent/report/report-2026-10-13-extnav-performance-opt.md)。
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
- Task32 已修复 OpenCV 与 AprilRobotics 官方图案的 Tag 基变换，并撤销错误相机 `Rz(180°)` 补偿。
  配置 Tag +X 指官方图案上方、+Y 左、+Z 上；方向改动必须用官方图案和独立物理真值验证，
  yaw 正确或自生成合成闭环不足以证明位置方向正确。
- UQ212 原生采集 MJPEG 1920×1080@120，解码/发布默认限 30 Hz；实际内参采用 30 张实测结果，
  状态仍 `calibrated_pending_independent_validation`。外参是机械近似
  `t=(0.04657,0.02897,-0.06726)m`，尚未独立标定验收。现场 Tag0 `(0,0)`、Tag1 `(0,1.8)`
  同向、边长 0.099 m；部署保留用户 CSV，不复用旧内外参下的窗口。
- **refresh Odin 已改为仰角 20°，生产转换仍是零安装角，尚未适配。** raw/extnav pitch
  实测约 −19.42°，最终 FCU EKF 约 +0.62°；EKF3 的外部姿态输入不直接融合 roll/pitch，
  GUI 近零不能证明 extnav 已补偿。需重新核对 IMU→FCU 姿态、杆臂位置/速度及应用跳变预测；
  仅改 `pitch_cam` 或放宽 tilt 门不能替代完整转换。new 的安装变化未检查。
  证据见 [安装角分析](agent/report/history/report-2026-10-09-odin-20deg-coordinate-assessment.md)及
  [EKF 倾角分析](agent/report/history/report-2026-10-09-extnav-ekf-pitch-difference.md)。
- **绝对定位/航向精度仍未验收。** 短时稳定、检测率提升、用户两点 apply 成功和 EKF 跟随
  不构成真值证明；10° tilt / 45°跳变门只是用户授权的地面实验门限。
  尚需精测 Tag 世界位姿、明确物理参考点、重标外参及跨位置/跨 session 独立验证。
  当前 PoseStamped 不传 MAVLink estimator reset counter，应用修正仍有 EKF 瞬态风险。

## 避障当前基线与验收边界

- **当前恢复版以169实际现场代码为准，未重新合入搜索/碰撞算法改写。** `path_searching`整个目录与169开始时快照一致；原语检查在当前参数下最多42次。YAML：safe_distance=0.25、horizon=100、resolution=0.1、voxel_size=0.1、max_vel/max_acc=2、max_tau=0.6、collision_step=0.05、search_budget=0.3、input_timeout=0.5。源YAML盲区0.3m，169实际Odin入口仍默认0m覆盖；运行下界[-50,-50,-2]，上界[50,50,10]。
- 新增PlanMotion/service-only仅为接口适配，PLAN调用旧search/getSegments；CHECK_LINE/剩余曲线离散采样调用旧isSafe，不恢复连续距离算法。正式桥快照每次clearMap后setKdtree；独立Odin入口保留50帧双树累计及旧goal行为。核心19例18通过、旧CollisionBetweenPrimitiveEndpoints仍失败（0.09999036m<0.1m），不得改阈值或宣称全量通过。
- `/odin1/cloud_slam_down`现由两机Odin host SDK在`cloud_slam`输出端直接生成，仅含x/y/z FLOAT32、point_step=12；不再携带RGB，新版不创建或发布原始cloud_slam；内部临时XYZRGB转换保留；配置为`/home/nvidia/catkin_ws/src/odin_ros_driver/config/control_command.yaml`顶层`cloud_downsample`：enabled=true、voxel_size=0.1、robot_radius=0.4、center_mode=auto、debug_enabled=false。驱动沿用已有源码目录配置读取方式；不再同时启动`/home/nvidia/down_samp_exe.py`，避免双发布者。原Python保留备用，独立避障库未改。
- SDK已移除auto的1m距离启发式：auto/odom只用有效Odin里程计；缺少/非有限球心时只降采样，不猜原点，origin才显式使用(0,0,0)。有限正voxel、有限radius、合法模式在启动时严格检查；非法值仅关闭新增功能并报错。XYZ的NaN/Inf全部跳过；降采样不读取或保存RGB。球面删除、字节键排序及空结果不发布保持。GCC过滤函数局部禁FMA保留；负零符号位微小差异保留。上游剔除仍可能删除真实近障，二次盲区保持关闭；这不是设备内部SLAM的输入过滤。
- 调试需要将`debug_enabled`设true并重启Odin，再`ros2 topic echo /odin1/cloud_slam_down/debug std_msgs/msg/String`；默认没有调试发布器，无订阅时不计时或格式化。维护目录已由用户移至开发机`/home/nvidia/scq/projects/odin_ros_driver-modified`；私有远端`LostPatrol/odin_ros_driver-modified`已推送main `16735e8`。仓库采用独立凭据，仓库级http.proxy为空以绕过本机不稳定代理。
- new曾在正常关闭后重连出现USB可枚举但设备无下行数据，原驱动二进制也复现；用户冷启动恢复；本次补充修复后用户要求普通重连，实际Odin连接成功且有原始/down云，不能据此认定旧重连问题根因已消除。不要把发布器存在当成点云有效；后续若复现先核对实际数据与Odin连接状态，不能靠改过滤器或自动重置设备掩盖。仅down发布证据与new待生效状态见[关闭原始云报告](agent/report/report-2026-10-13-task2-down-only.md)；仅XYZ输出证据见[RGB移除报告](agent/report/report-2026-10-13-task2-xyz-only.md)；校验与auto修复见[校验与auto修复报告](agent/report/report-2026-10-13-task2-cloud-validation-auto-fix.md)，首次迁移见[SDK集成报告](agent/report/report-2026-10-13-task2-down-samp-exec-inte.md)。

- 独立算法工作区 `/home/nvidia/scq/projects/dyn_small_obs_avoidance-ros2`，远端
  `LostPatrol/dyn_small_obs_avoidance-ros2`（GPLv3）；仅含 `path_searching/path_planning`。
  `avoidance_bridge` 负责观测地图和规划请求，onboard 仍唯一执行控制。
  直线策略不依赖规划器；两种避障策略不可用时拒绝接单，不能回退直线。
- 遇障悬停检查实际 FCU 到下一个业务航点的完整直线；自主避障用多项式 p/v/a 复用原 PD+DOB；旧规划器超包线的曲线只做统一时间缩放，保持空间路径。
  阻塞保留任务、停止推进/拍照，持续有效 0.4 秒恢复，连续等待 15 秒锁存 LAND。
  响应关联控制会话、任务/航点、请求、桥/规划会话和坐标 revision，0.8 秒过期；不执行旧预览。
- 新鲜完整规划按**接收时最新 FCU 位置到连续曲线的三维最近点**接入，裁掉旧前缀并解析平移
  首段，参考/剩余校验/预览共用新零秒。等距取较晚点，位置/速度接入容差仍为 0.08 m / 0.08 m/s。
  不改变原有停稳恢复、包线、PD+DOB 或空间曲线；最近点接入不能保证所有飞行误差均无反向运动。
  正式任务完成后停止规划/验证请求并悬停，地图与服务继续运行；独立 `odin.launch.py` 默认仍持续
  对保存的 goal 规划。此前该 launch 的 YAML 优先修复已在2026-10-12回退，当前默认0.0覆盖YAML；
  正式桥的累计地图仍关闭二次盲区滤波，两入口不能混淆。
- 实机当前只支持**未应用 Tag 的 extnav 基线**。世界障碍加权威杆臂 T 进入 map，
  规划请求 p/goal 减 T、返回多项式常数项加 T；不能按 Odin 仰角另旋转世界点云。
  raw/FCU 接收时间配对不代表采样同步；实际雷达测量中心外参及近身观测覆盖尚未验收。
- 地图 0.1 m 体素、固定首帧原点半径 20 m；**未观测是 unknown，不是 free**。
  无 TTL 消障，只用新射线穿越退役占用。自由证据依赖静态假设，覆盖检查仅离散中心线，
  不保证整个机体扫掠体积、动态障碍预测或持续全视野覆盖。输入必须是当前注册扫描，不能冒用历史累计云。
- 净空按169当前**体素质心距离0.25m，严格小于才拒绝，无体素/采样补偿**；
  保留旧核心浮点KD树比较与离散检查，存在采样间隙，不保证连续曲线/每个原始点/真实表面包络。
  水平/垂直执行参考上限1.0/0.2m/s、加速度0.35/0.15m/s²，通过统一时间缩放兼容旧搜索参数；实际PD+DOB可能超调。
- 恢复版软件回归及真实ArduPilot SITL绕行、两种恢复、超时LAND已重验；两机仅隔离无FCU smoke和合成DDS验证，旧台架结果不代替本次真实设备闭环。
  GUI 默认仿真入口也通过两种避障策略；扫描演示仅限隔离仿真，缺少依赖会提示原因。
  **refresh 台架实际地面起点查询仍无可行轨迹/unknown，不能宣称实机航线可执行。**
  实飞、长时 P99、热稳态、视频并行负载和实际速度严格上限仍未验收；Linux 平均 100 Hz
  不等于硬实时，Odin 实时调度权限不足及启动窗口抖动需量化。
  恢复版证据见 [恢复报告](agent/report/history/report-2026-10-12-task38-restore-169-planner.md)。历史证据见 [Task38 集成报告](agent/report/history/report-2026-10-09-task38-dyn-integration.md)及
  [GUI 仿真报告](agent/report/history/report-2026-10-10-task38-gui-sitl-avoidance.md)，最近点接入与短时台架结果见
  [路径延迟适配报告](agent/report/history/report-2026-10-10-new-1-path-delay.md)。

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
