<!-- 航点行为、自动 Tag 校准状态机及两端升级入口。 -->
# 航点 Tag 校准工作流

## 使用与范围

航点编辑表的巡检、校准标志互相独立：

- 普通点：移动、到点；不抓拍、不上报巡检到达消息。
- 巡检点：到点即抓拍并上报；不等待同点的校准完成。
- 校准点：到点后暂停推进索引，开启已有下视校准服务并自动应用。
- 两种标志可同时启用；新增点及旧 CSV 导入点均默认普通点。

校准点只支持 **odom 绝对坐标＋直线/遇障悬停**；map、自主避障、`local_coordinates` 暂不支持。无校准点的原有策略不受此限制。旧上位机 `taskPoints` 仍显式视为巡检点，无需改变 V2.2 协议。

CSV 表头如下，`yaw` 单位为度，程序/ROS 中为弧度：

```csv
index,x,y,z,yaw,is_inspection,is_calibration,calibration_tag_id,continue_window
1,0,0,1.2,0,true,true,0,false
2,1,0,1.2,0,false,false,0,false
3,2,0,1.2,0,false,true,1,true
```

布尔字段接受 `true/false` 或 `1/0`；Tag ID 为非负 int32。旧五列表头仍可导入。非法字段在导入/提交时报错，未知 Tag、重复窗口 Tag、服务不可用和不兼容组合由提交时的机载权威检查拒绝，GUI 显示具体原因。

## 窗口和控制衔接

本任务首个校准点总是清空残留滑窗并执行 FIRST，忽略它的沿窗选项；**清窗不清除 extnav 已生效修正**。后续点沿窗时执行 NEXT，不沿窗则清窗开启新 FIRST 段。窗口长度沿用校准服务配置；同一 Tag 尚在窗口内时不能再次沿窗加入，退出窗口或新开段后可重用同一真实 Tag 参数。

沿窗依赖本任务记录的服务实例、Odin session、窗口 revision 与有效历史；重启、过期或手动修改历史不会静默改成 FIRST。到点之前及校准结束后仍允许手动面板操作，但修改历史后不能继续依赖原窗口。

状态机非阻塞运行：申请独占 → 必要清窗 → 启动 FIRST/NEXT → 核对本次 job 的应用状态及 extnav revision/最终样本 → 等待实际 MAVROS EKF 与修正后辅助 pose 的 XY/yaw 一致、低速且持续稳定 → 推进下一点。ACK 或固定延时不能单独推进任务。

应用前将原悬停目标（保留已有位置/朝向误差）保存到未经Tag修正的原始Odin坐标。应用/融合期间按最新raw与FCU位姿把固定物理锚点换算到当前FCU坐标：仅坐标重对齐不会指挥机体移动，实际漂移/转向仍产生回拉误差。水平速度从原始里程计的FCU机体系twist旋转到当前FCU世界轴；DOB与加速度历史随坐标轴旋转，仅在速度来源切换时重表达观测状态，不清零扰动。Z目标、速度来源及控制历史不变。

原始输入参数为`calibration_odometry_topic`，默认`/odin1/odometry_highfreq`，要求`header.frame_id=odom`、`child_frame_id=fcu`，与项目Odin驱动一致；使用同一消息的pose/twist。订阅在节点启动时建立、仅保留最新一帧，避免空中创建ROS图；普通飞行不依赖该输入。校准前等待原始输入，应用/融合时输入无效或超时复用LAND。原extnav速度及融合门控保持；余下航点仍为修正世界里的原始绝对目标，不再次变换。末点也等待校准与融合后才报告成功。

为定位融合等待，既有独立诊断文件每秒至多写一行`CALIBRATION_FUSION`：位置/yaw差、三组速度（FCU三维、extnav水平、两者水平差）、数据年龄（FCU pose/velocity、extnav pose/velocity、raw）、应用后时间戳、稳定持续时间和实际参考。诊断线程try-lock只拷贝定长快照，计算/格式化/落盘均在锁外，不参与飞行判定。

既有采样/求解/应用分阶段超时保持不变；机载 `calibration_timeout_seconds` 默认 **120 秒**，包含申请、应用及飞控融合。异常结束任务并调用既有原地 LAND；用户悬停、取消、降落或失联接管则异步停止校准，迟到结果不能恢复旧任务。校准独占期间面板允许看状态，不允许并发启动、清窗、应用或保存 Tag 配置；释放资源后恢复按钮。

## 构建与维护

飞行接口为 **3.6**，视频仍为 3.2，校准/extnav仍为 2.0。部署须更新两端源码，先停止旧进程，再原生重建消息、控制/修正服务及消费 `ControlStatus` 的 `avoidance_bridge`，不能混用旧可执行程序与新消息动态库：

```bash
source /opt/ros/jazzy/setup.bash
# 先加载本机配置的独立规划器 overlay（以下为相邻目录默认路径）。
source ../dyn_small_obs_avoidance-ros2/install/setup.bash
colcon build --base-paths src correction_service \
  --packages-select correction_interfaces guided_interfaces onboard_control guided_sim correction_service avoidance_bridge \
  --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

- `src/guided_interfaces/msg/Waypoint.msg`、`ground_station_core/waypoint_io.py`：标志与 CSV。
- `ground_station_core/qt_ui/waypoint_panel.py`、`ros_controller.py`：编辑、提交和状态展示。
- `src/onboard_control/src/waypoint_calibration.cpp`：到点异步状态机、窗口约束和融合衔接。
- `correction_service/correction_service/node.py`：复用校准算法及跨操作独占；面板逻辑在 `correction_panel.py`、`ros_client.py`。
- `ground_station_core/upstream/status_projector.py`：只统计巡检点，并处理抓拍/校准并发。

每次校准只处理请求指定的 Tag ID，按该 ID 配置的 `size_m` 求解位姿；同帧其他已配置或未知 ID 不参与 PnP 或样本准入。旧 `reject_multiple_tags` 字段仅为配置兼容保留，不再生效。目标缺失、PnP失败、重投影/倾斜质量不合格及时间同步失败仍按原逻辑拒绝。

自动校准不替代原有算法质量检查。SITL 注入候选可以检验真实桥、EKF 和控制衔接，但不等于相机检测或实机验收；实机解锁、起飞仍只能由用户手动完成。

## 遥测保护诊断

新增证据写入独立的 `onboard-control-diagnostics-<PID>-<启动时间ns>.log`，默认在
`~/.ros/log/`（优先遵循 `ROS_LOG_DIR`，其次 `ROS_HOME/log`）。启动时的 `onboard.log`
或服务 journal 会打印绝对路径；诊断正文不进入 `/rosout` 或地面站日志，以免共用ROS全局日志锁。
出问题时同时保留该文件与原 `onboard.log`。按 `event_ns` 关联实际事件，通常延迟约1秒输出；
锁忙或磁盘慢时可更晚。文件无法创建仅在启动时告警，不阻止飞行。它们不改变控制/校准逻辑和保护阈值。

- `[TELEMETRY_FAULT]`：故障前的有效性、位置/速度/状态接收年龄、原阈值、实际值、航点、
  校准阶段、命令序号及FCU会话计数。年龄`-1`表示从未收到；不要只看累计最大抖动。
- `[RAW_ODOMETRY]`：关联同一`event_ns`的原始Odin快照，包含接收年龄/超时值、frame/child、
  XYZ、body速度、四元数/平方范数、job、extnav revision和是否跟随融合；在取消任务前冻结。
  `TELEMETRY_FAULT.event=calibration_raw_*`区分`missing`、`stale`、`frame`、`xy_nonfinite`、
  `velocity_nonfinite`、`quaternion_invalid`；`calibration_fcu_pose_invalid`表示FCU位姿/yaw不可用。
  初始等待未满原超时、主动取消不额外报故障。文本最多保留63字节，`text_lengths`给出原长度（frame/child/job）。
- `[TELEMETRY_RX]`：故障前的最后一条样本，以及首条后续回调（`first_after`）。pose/velocity只保留
  日志输出前已处理的首条；`stream=raw_odin`可跨日志批次等待首条，始终使用原故障`event_ns`，
  不表示该帧数值有效或定位已恢复。无后续行不能单凭此判定源头停流。记录header、DDS发送/接收、
  回调处理墙钟时间、回调间隔和锁等待。DDS时间戳为原始中间件证据，0可能表示不支持；
  不跨主机直接相减，也不把不同实现的接收时间点当成相同语义。
- `[CONTROL_TIMING]`：仅记录≥100ms的控制周期间隔或已标记代码段耗时。
  `control_lock_wait_ms`表示状态锁等待，`execution_lock_wait_ms`表示命令/输出串行锁等待；
  `previous_work_ms`大指向上个控制周期内部耗时；
  二者都小时再结合DDS/回调时间检查调度或输入中断。
  `slow_operation`区分`status_graph`、`status_locked`、`control_dispatch`、
  `control_reference`（含校准推进）、`control_publish`和`status_publish`；`slow_age_at_event_ms`用于识别历史慢调用。
  `status_ms`是最近一次完成的状态回调六段耗时：图查询、等锁、维护、导航状态构造、其余字段、发布。
  慢事件同样附带raw快照；回调间隔大但等锁小时，需继续区分默认回调组调度、DDS与源头，不能直接归咎Odin。

- `[CONTROL_OUTPUT]`：附在慢事件/故障后，`event_ns`关联快照，`publish_begin_ns`、命令序号、
  Linux线程`tid`标识最近一次完成的输出。`wall_ms`与`cpu_ms`按compute/message/publish/relock排列，
  分别为PD+DOB计算、取时及消息构造、DDS发布、重新获得状态锁；CPU值`-1`表示采样失败。
  墙钟时间很大而线程CPU很小表明等待或未获调度；CPU也很大表明线程持续执行。
  `voluntary_switches/involuntary_switches`和`minor_faults/major_faults`是整段输出的线程计数增量，
  辅助区分主动等待、被抢占和缺页，不能仅凭计数认定某把DDS锁。旧输出须结合`publish_begin_ns`区分。
- `[PUBLISH_INFLIGHT]`：≥100ms仍未完成的姿态/状态发布，每秒最多各一条；记录线程、命令、
  已耗时及当时位置/速度/raw输入年龄。范围包含发布及其后的状态锁重获，不直接等同DDS阻塞；
  完成后的分段时间可进一步区分。该行在调用返回前即可落盘，无需等待遥测保护触发。

姿态和状态的消息发布在共享状态锁外；控制计算与状态转换仍受同步保护。命令/输出串行锁保持
已开始的姿态发布先于LAND/取消处理，关键输入使用独立回调组继续更新；不添加输出队列或修改保护阈值。
发布本身仍可能变慢，日志用于定位该剩余问题；本次未将离散结果、视频和避障消息的全部发布移出锁。

高频路径仅采样固定数量的线程CPU/调度计数、更新时间和定长内存快照，不格式化、不写盘、不增加高频话题。
独立1Hz回调用try-lock提取后释放控制锁，再写独占文件（不经过ROS/stdio标准输出锁）；每批最多一个故障快照和一个合并后的
最慢耗时快照（各附最近输出分段）、一条raw后续接收证据及最多两条在途发布记录，无无限队列。持续抖动的`count`是合并的事件数，不等于唯一超期周期数。
此诊断只需原生重建`onboard_control`并在地面重启加载新版，无消息接口或配置变更。
