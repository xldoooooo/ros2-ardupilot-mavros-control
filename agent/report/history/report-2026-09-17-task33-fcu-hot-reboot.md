<!-- Task33 execution report: implementation, real unarmed verification and remaining scope. -->
# Task33：飞控热重启与热重联

日期：2026-09-17。任务来源：`agent/task/33-reboot-FCU.md`。

## 结果

实现 GUI 与机载薄脚本共用的飞控热重启流程，并在用户指定 **drone-new** 上完成真实未解锁验证。
当前 SSH 配置实际解析为 `192.168.112.169`；refresh 别名解析为 `.186`，本次未连接或部署 refresh。
任务期间没有调用实机解锁或起飞，没有恢复旧飞行任务。

## 实现

- 主 GUI 右上角在“退出地面站”左侧增加同风格“重启机载飞控”；仅实机已连接、未解锁、
  新鲜落地、待机且无任务/待执行命令时启用。确认框默认取消，确认返回后再次检查状态。
- 飞行协议升级 **3.3 / 包3.3.0**：增加重启命令以及 `on_ground`、`reboot_in_progress`。
  独立视频协议保持3.2。地面端与飞行包必须同时升级。
- 机载逻辑独立放入 `src/onboard_control/src/fcu_reboot.cpp`，复用原租约/命令封装。
  通过 MAVROS CommandLong 发普通246/param1=1，不使用强制重启、bootloader、解锁或起飞命令。
- 以飞控 TIMESYNC 启动时钟回退确认真实重启，不能以 ACK 或进程在线代替。
  对短到 MAVROS 没有报告断线的重启也清理旧会话证据；旧频率请求回调不能推进新会话。
- 重启前主动只读请求 GPS_GLOBAL_ORIGIN，最多等待3秒；恢复期间查询回读并重新发布已知原点。
  重建参数、频率配置；新鲜状态、落地、位置/速度、原点回读、新参数事件及无 setpoint 冲突
  持续满足2秒后才报“控制链路成功恢复”。总时限90秒；丢 ACK 不自动重发重启。
- `reboot_fcu.sh` 仅检查机载身份/本机进程、加载环境并调用 `tools/reboot_fcu.py`。
  Python 客户端使用项目环境，申请/续租/释放短租约，等待同一个机载命令终态。
  地面机直接执行已确认拒绝；其他客户端持权时沿用既有租约拒绝机制。
- 更新最小 sparse checkout 路径与部署文档，包含新脚本、客户端及 `.venv` 准备步骤。

MAVLink 命令语义依据：[MAVLink Common](https://mavlink.io/en/messages/common.html#MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN)。
另核对本机 ArduPilot `GCS_Common.cpp::send_gps_global_origin()`：无 EKF 原点时直接返回，不发原点消息。

## 测试与实机验证

- 最终完整 Python 回归：**293 passed**。部署路径补充后的定向回归：**20 passed**。
- 开发机 colcon 汇总：**24 tests，0 errors / failures / skipped**。
- new 原生 Release 构建通过；机上 colcon 汇总：**23 tests，0 errors / failures / skipped**。
- 真节点隔离 ROS 测试覆盖：断线、已解锁、非落地、落地过期拒绝；ACK不能提前成功；
  事务期间重复请求及起飞请求拒绝；原点/参数缺失时不成功；原点回读后完成；外部重启自动恢复。
  所有模拟起飞请求只在隔离测试域发送，且均被重启门控拒绝，未发送到飞机。
- Qt 测试覆盖默认取消、不满足条件禁用、仿真禁用、确认期间状态变化、防重复提交。
- 真实 GUI 验证使用 Qt offscreen 渲染和真实地面 ROS 客户端，执行正式实机连接、确认框及按钮。
  用户明确授权使用首次连接默认原点 `(30.2489634, 120.2052342, 488.0m)`；原点回读约
  `487.996m`，在既有确认容差内。实际按钮触发后分别收到两阶段成功日志。
- new 的机载脚本实际重启完成，两阶段成功结果正常退出。
- 从 MAVROS 单独发起一次已落地/未解锁/待机/无租约的飞控重启，onboard 未收到重启任务，
  仍自动检测并恢复；观测到成功终态耗时 **10.1秒**。
- 最终240秒被动观察记录2396条控制状态、246条FCU状态、447条落地状态及2359条启动时钟，
  **没有任何 armed=true，也没有收到 `/mavros/setpoint_raw/attitude` 输出**。
  GUI从发送到完整恢复约10.0秒；机载脚本约11.8秒；外部入口独立计时10.1秒。
  最终为3.3、STABILIZE、未解锁、已落地、定位/推力/频率配置就绪、无租约、无活动控制器。
- 外部热重启前后 onboard/MAVROS/Odin 的 PID 与命令行逐字相同；运行中可执行文件与最新安装文件
  SHA-256 同为 `8a8586170fa58631a2c2b161aa840cb6ce1b563899d60f08136c4c1013f07a76`，
  共享状态消息和包版本源码/install 比较一致。
- 部署加载新二进制时在未解锁落地检查后重启过机载总服务（其监督域包含 MAVROS/Odin/extnav/onboard）。
  随后的热重启验收只重启 FCU，不重启这些进程。独立视频/修正服务没有被停止或重启。

## 真实失败与修复

1. 首轮新机未配置/未回读 EKF 原点。虽然实际重启和定位/参数/消息链已恢复，仍在90秒后
   如实返回恢复超时，未伪报成功。240秒记录全部 `armed=false`，未收到姿态/推力 setpoint。
   增加重启前后主动原点查询；用户随后确认默认原点，经正式GUI连接写入后，三条路径均成功。
2. 首次全量测试1项使用硬编码3.2导致失败，已更新协议夹具；另一轮发现隔离节点发现超时，
   单项日志显示节点刚启动就到测试期限，已将仅测试的首次发现等待从8秒改为20秒。
   后续最终完整293项通过；未放宽生产安全阈值或恢复门控。

## 部署、证据与限制

- 机载工作区：`/home/nvidia/ros2-ardupilot-mavros-control`。
- 部署前备份：机上 `agent/codex/task33/pre-deploy.tgz`（原飞行源码与安装产物）；开发机
  `agent/codex/task33/new-before.tgz`（对比用原源码）。
- new 原 Git HEAD 为 `6a40713`，存在大量既有现场改动。仅同步本任务文件，未清理现场内容、
  未强制切换/重写机上 Git 历史。开发机已有 README/TODO/旧部署文档迁移等用户改动未纳入任务提交。
- 本地证据目录 `agent/codex/task33/`：完整测试日志、GUI前/确认/后截图、GUI日志、
  `new-script-reboot.log`（首轮失败）、`new-script-final.log`、`external-new.log`、
  `bench-first.jsonl`、`bench-final.jsonl`、机载事件及进程对比。
- 仅验收未解锁地面链路；没有实飞。频率配置 ACK 不等于遥测实测100Hz，位置可用也不是精度验收。
- USB消失后重新枚举、自动恢复旧任务不在任务范围。完全缺失原点或定位时会明确恢复失败；
  不为通过验收自动写入未经用户确认的默认坐标。
- refresh 尚未同步3.3；下次使用该机前需同步接口、机载包、新脚本并重建，不能直接混用3.3地面站。
