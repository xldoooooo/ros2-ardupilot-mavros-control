# 任务 09：地面站—真机通信测试报告

日期：2026-08-08

地面站：Ubuntu 24.04 / ROS 2 Jazzy / x86_64 / `192.168.112.176`

真机伴随计算机：Ubuntu 22.04 / ROS 2 Humble / aarch64 / `xld@192.168.112.186`

## 1. 结论

任务 09 的只读端到端测试已执行完毕，全程没有解锁或起飞实机。

结论分为两层：

1. **本任务实际覆盖的数据链路通过**：自定义 `/onboard_control/status`、原始 `/mavros/state`、新鲜 `/rosout`、GUI 状态与日志展示均能从真机 Humble 传到地面站 Jazzy；两个 60 秒状态观察窗均为 601/601 条，估算送达率 100%，字段完整，无非有限值或语义错误。
2. **不能宣称整个 Humble/Jazzy ROS 图完全兼容或可用于实飞**：跨发行版发现期间，真机的新机载节点和 MAVROS 反复输出 `sequence size exceeds remaining buffer`；Jazzy 图查询还把 Humble 发布者显示为未知节点、类型哈希 `INVALID`。这是明确且未解决的 DDS/类型发现兼容告警。

因此，本轮结果是“限定路径台架验证通过”，不是“跨发行版 ROS 2 获官方支持”，更不是“允许实飞”。

## 2. 安全边界与实际执行内容

全过程保持：

- 真机始终 `armed=false`；
- 飞控始终非 GUIDED，本轮观察为 `STABILIZE`；
- 没有调用 `/mavros/cmd/arming`、`/mavros/cmd/takeoff` 或 `/mavros/set_mode`；
- 没有发送 `FlightCommand`、`MotionIntent`、航点或控制心跳；
- 没有申请控制租约；
- 没有写 GPS 原点；
- 没有调用 `/mavros/set_message_interval`；
- 没有启动 Odin 或 extnav；
- 姿态 setpoint 观察窗内收到 0 条消息；
- 没有安装、启用或启动 systemd 服务。

旧 `/home/xld/odin.sh` 经审计会在启动后主动调用三次 `/mavros/set_message_interval`。这不符合本任务“只连接、不发送维护指令”的要求，因此没有执行旧脚本。飞控链路阶段只单独启动发行版自带的 MAVROS `apm.launch`。

MAVROS 启动本身会进行协议所必需的心跳、版本和参数读取；日志中的 MAVLink command 520 是版本查询，机载节点也只读查询了 `GUID_OPTIONS` 与 `MOT_THST_HOVER`。本任务没有发出模式、解锁、起飞或参数写入请求。

## 3. 为安全完成 GUI 测试而做的修正

修改前审计发现：

- ROS 客户端一旦发现兼容机载端便自动申请控制租约；
- “连接实机服务”工作流会申请租约、配置消息频率并写 GPS 原点；
- 这与任务 09 的“只是连接，不发送任何指令”冲突。

本轮已修正为：

- `GroundStationRosController` 启动后默认严格只读；
- 只有本地 SITL 工作流会显式 `enable_control()`；
- 实机连接不申请/续租，不发布心跳；
- 实机连接不配置消息频率、不写原点、不等待控制权；
- 实机连接在 2 秒观察窗内持续检查 `armed=false` 且本客户端无控制权，否则立即失败；
- 实机会话强制禁用起飞、降落、悬停、运动和航点发送，即使异常快照显示本 source 已持有租约也不开放；
- 按钮改为“连接实机（只读）”，确认文案明确列出零租约、零心跳、零飞行/维护指令；
- 只在实机只读会话动态订阅 `/rosout`，把远端来源、原始文本和 ROS 等级写入 GUI 事件总线；
- `ControlStatus.status_message` 仍作为跨发行版自定义协议内的权威状态日志逐字进入 GUI；
- 断开只读会话时若从未持有租约，不发送多余 release 请求。

预留 GPS 原点仍只保存在本地界面，当前实机只读工作流不会使用或写入。

## 4. 真机测试基线

测试前：

```text
MAVROS/Odin/extnav/onboard_control 进程：无
ROS_DOMAIN_ID=42 图节点：无
/dev/ttyTHS1 占用：无
onboard-control.service：inactive
mavros.service：inactive
```

旧回退仓库：

```text
/home/xld/ros2-ardupilot-mavros-control
HEAD = dad90678e02169b80b44f903753e157c4bfda7c5
工作树 = clean
```

新最小部署仓库：

```text
/home/onboard/ros2-ardupilot-mavros-control
HEAD = c8abad9ebdd26ea4c13a732547663a6edf2b6988
工作树 = clean
```

## 5. 阶段一：无 MAVROS 的纯 DDS 状态链

真机只启动新机载二进制，domain 42、`ROS_LOCALHOST_ONLY=0`；没有启动 MAVROS，因此飞控未连接。

Jazzy 能发现：

- `/onboard_control/status` 类型为 `guided_interfaces/msg/ControlStatus`；
- 发布者数为 1；
- 高层服务名称和 `AcquireControl` 服务类型可见；
- 状态消息能够完整反序列化。

60 秒只订阅探针结果：

```text
status_count                 601
expected_count_from_10hz     601
estimated_delivery_ratio     1.000000
publisher gap max            0.100265 s
receive gap max              0.104428 s
receive period p95           0.100849 s
interface_version            2.0（601/601）
fcu_connected                false（601/601）
armed                        false（601/601）
lease_active                 false（601/601）
active_command_sequence      0（601/601）
semantic_errors              0
```

首个严格探针因启动日志的 `/rosout` 10 秒 lifespan 已过而报告“未收到 rosout”，故整体按规则记为 FAIL，没有把状态话题成功美化成日志成功。随后先建立订阅再重启无 MAVROS 的机载节点，地面站准确收到：

```text
机载控制服务 2.0 启动：控制 100.0 Hz，接口 /onboard_control，MAVROS /mavros
```

## 6. 阶段二：真实 MAVROS 与飞控只读链路

只执行：

```text
ros2 launch mavros apm.launch fcu_url:=/dev/ttyTHS1:460800
```

没有运行旧 `odin.sh`，没有 Odin/extnav，也没有调用任何服务。

真机本地原始 `/mavros/state`：

```text
connected: true
armed: false
guided: false
manual_input: true
mode: STABILIZE
system_status: 3
```

地面站 Jazzy 收到的完整机载聚合状态包括：

```text
interface_version: 2.0
fcu_connected: true
armed: false
autopilot_mode: STABILIZE
local_position_valid: false
control_mode: 待机
controller_active: false
lease_active: false
active_command_sequence: 0
message_rates_configured: false
setpoint_conflict: false
```

未启动 Odin/extnav，因此 `local_position_valid=false` 是预期结果，本轮没有用虚假数据宣称位姿链通过。

机载参数只读检查最初因 MAVROS 参数列表尚未完成而保持禁用，随后准确变为：

```text
已确认 GUID_OPTIONS bit 3，并同步 MOT_THST_HOVER=0.20922
```

连接飞控后的第二个 60 秒探针：

```text
status_count                 601
expected_count_from_10hz     601
estimated_delivery_ratio     1.000000
publisher gap max            0.101488 s
receive gap max              0.110389 s
receive period p95           0.101017 s
interface_version            2.0（601/601）
fcu_connected                true（601/601）
armed                        false（601/601）
lease_active                 false（601/601）
active_command_sequence      0（601/601）
semantic_errors              0
remote rosout                8 条
```

8 条远端日志包含机载状态、MAVROS 参数警告/完成信息和最终推力语义确认，来源、等级和文本均成功解码。

额外 30 秒硬件守护窗：

```text
ControlStatus                300 条，全部 connected=true / armed=false
/mavros/state                29 条，全部 connected=true / armed=false
guided                       false（29/29）
mode                         STABILIZE（29/29）
attitude setpoint            0 条
lease_active                 false（300/300）
active_command_sequence      0（300/300）
```

## 7. GUI“连接实机（只读）”真实按钮测试

测试使用实际 PySide6 控件、按钮信号、生产 ROS 客户端和生产环境工作流；只把本地进程扫描替换为不变更进程的测试监督器，避免通信探针误清理人为启动的测试节点。

无 MAVROS 阶段和真实飞控连接阶段均通过：

```text
environment_active           true
connection_mode              hardware
onboard_available            true
interface_version            2.0
armed                        false
control_authority             false
controller_control_enabled   false
command_results              0
takeoff/land/hover/motion     全部禁用
waypoint_send                禁用
```

日志验证分两条路径：

1. 机载节点在 GUI 已订阅时启动，GUI 逐字显示远端启动日志和 `MOT_THST_HOVER=0.20922` 确认日志；
2. 远端节点已稳定运行且近期没有新 `/rosout` 时，GUI 仍从 `ControlStatus.status_message` 逐字显示同一权威状态文本。

一次早期 GUI 探针只把“短窗内收到新 `/rosout`”作为日志成功条件，节点稳定运行且近期无新日志时按规则记为 FAIL。该探针随后修正为分别验证“新鲜 `/rosout`”与始终存在的协议状态消息；产品代码没有伪造日志或放宽安全门控。

## 8. 跨发行版兼容性问题

实际版本：

```text
Humble rmw_fastrtps_cpp       6.2.10
Humble rcl_interfaces         1.2.3
Jazzy rmw_fastrtps_cpp        8.4.3
Jazzy rcl_interfaces          2.0.3
```

观察到：

- `ros2 topic info` 能看到 Humble 发布者，但节点名/命名空间显示为 unknown；
- Jazzy 对 Humble 端点报告 type hash `INVALID`；
- Humble 新机载节点和 MAVROS 在 Jazzy 端点加入/离开时反复输出 `sequence size exceeds remaining buffer`；
- 关闭 `/rosout` 订阅后，最小状态订阅仍能触发该告警，说明问题不限于日志话题；
- 已测状态话题仍能持续完整解码，节点没有崩溃，飞控状态没有改变。

这表明“具体稳定字段的 CDR 数据路径可工作”与“整个跨发行版 ROS 图兼容”不是同一结论。当前告警没有被忽略，也没有被包装成成功。实机部署前应统一 ROS 发行版/DDS 版本，或采用受支持、明确限定接口的桥接层后重新做长时间测试。

## 9. 旧环境保护与清理

测试后：

```text
MAVROS/Odin/extnav/onboard_control 进程：无
/dev/ttyTHS1 占用：无
ROS_DOMAIN_ID=42 图节点：无
onboard-control.service：inactive
mavros.service：inactive
```

旧仓库与新仓库提交、工作树均与测试前相同。

三个旧关键脚本测试前后 SHA-256 均不变：

```text
533cc0b8578d4dfde46dc26d5fd69dd16390e95b3fbd579a5ba316fde87b80b0  /home/xld/odin.sh
ddbfb5000a471981038ca78bb8520c9e7fc34405d51b13a301c93a1705a366da  /home/xld/ros2-ardupilot-mavros-control/odin1.sh
8bb06fe3f30e46ee50799c9d175ec227b0f0627bdc633888ef2c3b7280530d48  /home/xld/ros2-ardupilot-mavros-control/shfiles/start_mavros_real.sh
```

只新增了 MAVROS 正常运行日志目录 `/home/xld/.ros/log/2026-08-08-22-58-32-618001-xld-9130`；没有改动旧仓库、脚本、系统服务或启动配置。

## 10. 本地构建与回归

最终结果：

```text
colcon build（三包）                 成功
Python/Qt                           34 passed
ROS/C++                             5 tests, 0 errors/failures/skipped
ground_station --check-environment  成功
compileall                          成功
修改范围 flake8                     成功
git diff --check                    成功
```

新增回归直接覆盖：

- 发现兼容机载端时默认零租约调用；
- 只有显式启用后才允许申请租约；
- 实机工作流不得调用控制、消息频率或原点入口；
- `armed=true` 时实机观察立即失败；
- 实机会话全部飞行按钮强制禁用；
- 远端日志原文、来源和等级映射；
- GUI 确认文案明确只读边界。

本地 SITL 仅初始化回归同样通过，没有解锁/起飞：

```text
onboard_available            true
fcu_connected                true
armed                        false
control_authority            true
message_rates_configured     true
local_position_valid         true
cleanup                      4 个受管进程全部停止，零残留
```

这确认“默认只读”没有破坏本地仿真工作流的显式租约路径。

## 11. 未执行项目

按安全要求明确未执行：

- 旧 `odin.sh` 完整流程；
- Odin/extnav 与真实外部定位链；
- GPS 原点写入；
- 消息频率配置服务；
- 模式切换；
- 解锁、起飞、降落或任何实机姿态/推力控制；
- systemd 安装/启用；
- 跨发行版问题的系统包升级或中间件替换。

因此，本报告不宣称真实本地位姿链、外部定位或飞行控制已经验证。
