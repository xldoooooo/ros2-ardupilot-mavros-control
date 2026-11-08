<!-- 航点行为、自动 Tag 校准状态机及两端升级入口。 -->
# 航点 Tag 校准工作流

## 使用与范围

航点编辑表的巡检、校准标志互相独立：

- 普通点：移动、到点；不抓拍、不上报巡检到达消息。
- 巡检点：到点即抓拍并上报；不等待同点的校准完成。
- 校准点：到点后暂停推进索引，开启已有下视校准服务并自动应用。
- 两种标志可同时启用；新增点及旧 CSV 导入点均默认普通点。

校准点只支持 **odom 绝对坐标＋直线/遇障悬停**；map、自主避障、`local_coordinates` 暂不支持。无校准点的原有策略不受此限制。旧上位机 `taskPoints` 仍显式视为巡检点，无需改变 V2.2 协议。

上位机03可使用[本地航线适配](../config/ground/route_profiles/README.md)补充校准点；手动
上传入口不应用该规则。地面站SITL启动专用参数 `simulation_calibration:=true`，仅将校准
动作替换为保持当前目标等待5秒；入点判定、拍照时序、索引推进和取消仍使用同一执行器。
实机参数默认为false，继续执行下述真实校准状态机。

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

沿窗依赖本任务记录的服务实例、Odin session、窗口 revision 与有效历史；重启或手动修改历史不会静默改成 FIRST。窗口时间间隔/跨度超限仅记日志警告，不阻止沿窗或触发降落。到点之前及校准结束后仍允许手动面板操作，但修改历史后不能继续依赖原窗口。

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
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DAMENT_CMAKE_SYMLINK_INSTALL=OFF
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

诊断文件为 `onboard-control-diagnostics-<PID>-<启动时间ns>.log`，优先使用 `ROS_LOG_DIR`，
其次 `ROS_HOME/log`，默认 `~/.ros/log/`；启动日志会打印路径。排查时与原 `onboard.log`、
MAVROS遥测诊断一起保留，用 `event_ns` 关联。诊断异步落盘可能延迟，不参与控制或改变保护阈值。

| 记录 | 主要含义 |
| --- | --- |
| `TELEMETRY_FAULT` / `RAW_ODOMETRY` | 取消任务前冻结的FCU/raw有效性、数据年龄、原阈值、frame及校准状态；年龄-1表示从未收到。 |
| `TELEMETRY_RX` | 故障前最后样本和首个后续回调；无后续行不能单凭此认定源头断流。 |
| `CONTROL_TIMING` | ≥100ms周期/关键阶段耗时，区分状态锁、输出锁、状态构造及发布；结合DDS/回调时间分析。 |
| `CONTROL_OUTPUT` | 最近已完成输出的计算、消息构造、发布及重获锁的墙钟/线程CPU耗时。 |
| `PUBLISH_INFLIGHT` | ≥100ms未完成发布，包含其后的锁重获；不直接等同DDS阻塞。 |
| `CALIBRATION_FUSION` | FCU与修正后pose的差、三组速度、输入年龄及稳定窗口，定位融合等待。 |

DDS时间戳为原始中间件证据，0可能不支持，不能跨主机直接相减。
线程CPU值-1表示采样失败；墙钟远大于CPU提示等待或未获调度，不能仅凭调度计数认定具体锁。
`gap_ms`含上一调用耗时，`count`为合并事件数，不等于唯一超期周期数。

高频路径仅更新定长快照，独立1Hz回调try-lock拷贝后在控制锁外写文件。
姿态/状态发布位于共享状态锁外，部分离散结果、视频和避障消息仍在锁内。
日志实现和发布分段位于 `src/onboard_control/src/`，修改后只需原生重建控制节点并在地面重启，
不能把诊断采样或历史测试结果当成问题已消失的证据。
