<!-- 只维护当前基线、部署差异和未解决风险；实验过程与数据查阅 agent/report。 -->
# 项目重要记忆

维护日期：2026-10-10。源码事实已核对；已连接 refresh 完成最近点接入构建与未解锁台架检查，new 连接超时。
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
- 飞行协议 **3.4**，`guided_interfaces/onboard_control/guided_sim` 包版本 **3.4.0**；
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
| refresh | 主工程选择性同步至 3.4 及最近点接入，Git HEAD 仍 `b692076`；规划库基于 `0d86e7e` / 0.2.0，仅追加 Odin launch 的 YAML 优先修复，均已原生构建 | 保留现场 Tag/相机配置及 planner.yaml 的 blind_radius=0.8；20° Odin 安装角尚未适配；不能只按 HEAD 判断版本 |
| new | 2026-10-08 已同步 main 的路径迁移和修正采集代码，飞行协议仍 3.3；独立规划库仍 Task37 `4119937`；本次 SSH 超时未修改 | **补齐 Task38、最近点接入、Odin launch 参数加载和执行位并原生重建**；逐机保留配置，重新验证真实 FCU/摄像头启动 |
| hp-desktop | 2026-09-18 重建修正接口 2.0，已解决旧 install 缺少 `ApplySavedCorrection` | **尚未同步 Task38、GUI 仿真避障入口及最近点接入，需同步源码及重建接口** |
| 开发机 | 3.4；GUI 仿真自动启动真实规划器、identity 桥和合成扫描；最近点接入已通过 DDS 回归及 SITL | GUI 入口改动仍未同步其他三机；本次控制层与独立 launch 修复仅同步 refresh，实机仍使用真实传感器输入 |

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

- **Odin 近身拖点尚未解决，不能把接收端删点当成内部算法前过滤。** refresh 实际运行
  驱动 0.14.0 / SoC 0.14.2 / SLAM 0.13.1；`cloud_slam` 由设备输出，公开驱动/SDK 未查到
  算法输入最小距离或 FOV 裁剪参数。`cloud_raw_confidence_threshold` 只过滤主机发布的 raw 云。
  驱动已有 `sendimagemask` / `image_mask_abs_path` 向设备上传 PNG 的接口，当前关闭；
  是否作用于雷达算法、黑白定义及生效/清除条件未确认，不得按已验证的点云屏蔽使用。
  厂商 FAQ 承认 0.4 m 内拖点，建议近距感知用 raw 云并避让结构遮挡；raw 云仍需实际对比，
  不保证消除测距干扰。此次未改两机配置，详见 [驱动检查报告](agent/report/report-2026-10-10-odin-near-field-driver-audit.md)。
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
  证据见 [安装角分析](agent/report/report-2026-10-09-odin-20deg-coordinate-assessment.md)及
  [EKF 倾角分析](agent/report/report-2026-10-09-extnav-ekf-pitch-difference.md)。
- **绝对定位/航向精度仍未验收。** 短时稳定、检测率提升、用户两点 apply 成功和 EKF 跟随
  不构成真值证明；10° tilt / 45°跳变门只是用户授权的地面实验门限。
  尚需精测 Tag 世界位姿、明确物理参考点、重标外参及跨位置/跨 session 独立验证。
  当前 PoseStamped 不传 MAVLink estimator reset counter，应用修正仍有 EKF 瞬态风险。

## 避障当前基线与验收边界

- 独立算法工作区 `/home/nvidia/scq/projects/dyn_small_obs_avoidance-ros2`，远端
  `LostPatrol/dyn_small_obs_avoidance-ros2`（GPLv3）；仅含 `path_searching/path_planning`。
  `avoidance_bridge` 负责观测地图和规划请求，onboard 仍唯一执行控制。
  直线策略不依赖规划器；两种避障策略不可用时拒绝接单，不能回退直线。
- 遇障悬停检查实际 FCU 到下一个业务航点的完整直线；自主避障用多项式 p/v/a 复用原 PD+DOB。
  阻塞保留任务、停止推进/拍照，持续有效 0.4 秒恢复，连续等待 15 秒锁存 LAND。
  响应关联控制会话、任务/航点、请求、桥/规划会话和坐标 revision，0.8 秒过期；不执行旧预览。
- 新鲜完整规划按**接收时最新 FCU 位置到连续曲线的三维最近点**接入，裁掉旧前缀并解析平移
  首段，参考/剩余校验/预览共用新零秒。等距取较晚点，位置/速度接入容差仍为 0.08 m / 0.08 m/s。
  不改变原有停稳恢复、包线、PD+DOB 或空间曲线；最近点接入不能保证所有飞行误差均无反向运动。
  正式任务完成后停止规划/验证请求并悬停，地图与服务继续运行；独立 `odin.launch.py` 默认仍持续
  对保存的 goal 规划。该 launch 的 blind_radius 已改为 YAML 优先、缺省用节点默认，显式参数覆盖；
  正式桥的累计地图仍关闭二次盲区滤波，两入口不能混淆。
- 实机当前只支持**未应用 Tag 的 extnav 基线**。世界障碍加权威杆臂 T 进入 map，
  规划请求 p/goal 减 T、返回多项式常数项加 T；不能按 Odin 仰角另旋转世界点云。
  raw/FCU 接收时间配对不代表采样同步；实际雷达测量中心外参及近身观测覆盖尚未验收。
- 地图 0.1 m 体素、固定首帧原点半径 20 m；**未观测是 unknown，不是 free**。
  无 TTL 消障，只用新射线穿越退役占用。自由证据依赖静态假设，覆盖检查仅离散中心线，
  不保证整个机体扫掠体积、动态障碍预测或持续全视野覆盖。输入必须是当前注册扫描，不能冒用历史累计云。
- 净空按用户指定**体素质心距离 0.45 m，严格小于才拒绝，无体素/采样补偿**；
  连续多项式用解析实根检查，不等于每个原始点或真实障碍表面的包络保证。
  水平/垂直参考速度上限 1.0/0.2 m/s、加速度 0.35/0.15 m/s²；实际 PD+DOB 可能超调。
- 软件回归、真实 ArduPilot SITL 的绕行/恢复/超时 LAND/取消场景及 refresh 未解锁台架已验证。
  GUI 默认仿真入口也通过两种避障策略；扫描演示仅限隔离仿真，缺少依赖会提示原因。
  **refresh 台架实际地面起点查询仍无可行轨迹/unknown，不能宣称实机航线可执行。**
  实飞、长时 P99、热稳态、视频并行负载和实际速度严格上限仍未验收；Linux 平均 100 Hz
  不等于硬实时，Odin 实时调度权限不足及启动窗口抖动需量化。
  证据见 [Task38 集成报告](agent/report/report-2026-10-09-task38-dyn-integration.md)及
  [GUI 仿真报告](agent/report/report-2026-10-10-task38-gui-sitl-avoidance.md)，最近点接入与短时台架结果见
  [路径延迟适配报告](agent/report/report-2026-10-10-new-1-path-delay.md)。

## 关键历史与资料索引

- 2026-08-20 曾经用户授权改写 Git 历史以移除大视频/二进制，旧提交 ID 已失效；完整改写前备份
  `/home/nvidia/backups/ros2-ardupilot-git-pre-history-rewrite-20260820.tar.gz`。大媒体不提交 Git。
- new 同步前完整现场归档在 `/home/nvidia/ros2-ardupilot-maintenance/new-sync-20261008-1920/`；
  refresh 路径迁移及标定归档在 `/home/nvidia/ros2-ardupilot-maintenance/refresh-sparse-20260918-101142/`。
  其中现场标定不是可删除缓存；其他备份和哈希按对应报告查询。
- 历史任务/报告集中在 `agent/task/history/`、`agent/report/history/`，近期报告在 `agent/report/`。
  报告保留当时结论，已撤销的首次重锚、180°外参补偿及旧协议测试数据不能当作当前基线。
  曾有包测试将 Tag0 固定断言为 0.170 m、与默认 0.099 m 冲突；遇到该失败须核对当前配置，
  不把旧通过数量当作本次测试结果。
