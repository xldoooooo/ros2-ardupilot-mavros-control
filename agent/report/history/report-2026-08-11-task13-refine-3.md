# Task 13 refine-3 执行与调查报告

日期：2026-08-11

范围：地面站手动操纵 UI、本地仿真启动链、MAVROS 告警、实机重启日志、日志/握手/行为真实性审查

安全边界：所有 SITL 与实机验证均保持 `armed=false`；未发送解锁、起飞、降落、运动、悬停、航点或原点写入命令。

## 一、结论摘要

任务 1、2、5 已实现并完成回归；任务 3、4 经源码和实机复现确认属于无实际故障的 MAVROS 告警，按要求不改业务代码；任务 6～9 仅完成调查，没有越界实施后续路线。

| 项目 | 结论 | 本次动作 |
| --- | --- | --- |
| 1. 手动按钮提示 | W/A/S/D、I/J/K/L、SPACE 均有动作语义 tooltip 和无障碍描述 | 已修改、测试 |
| 2. 详细状态 | “工程信息”改为“详细状态”，原信息保留左栏，当前按键增量位于右栏并随灵敏度实时变化 | 已修改、测试、视觉核对 |
| 2. 滚转/俯仰、频率、电池 | 底层 MAVROS 可取，但当前机载聚合接口没有完整提供；不建议地面站绕过机载接口直连 MAVROS | 仅调查 |
| 3. command 520 | 有效的能力请求 ACK 被 MAVROS 广播命令记账逻辑打印成“Unexpected”；随后能力信息正常到达 | 无需修改 |
| 4. STAT_RUNTIME index | 参数先以无索引更新到达、后在完整参数表中带真实索引到达；完整参数表随后成功收齐 | 无需修改 |
| 5. 仿真启动慢 | 主要是人为串行等待、重复 SITL 构建和 ArduPilot EKF 稳定时间，不是 CPU 算力不足 | 已优化，仅作用于仿真 |
| 6. 重启后出现终端日志 | 是地面站主动订阅全局 `/rosout` 的预期效果，不是 SSH 终端镜像；重启会带来启动日志及持久化历史补发 | 仅调查 |
| 7. 日志审查 | 安全状态覆盖尚可，但 `/rosout` 范围过宽、同一状态有 INFO/DEBUG 双路径，部分关键“恢复”转换缺失 | 仅调查 |
| 8. 握手审查 | 已有接口版本、租约、心跳、状态新鲜度、命令序号和最终结果，明显强于仅靠 topic；仍缺 boot/session 身份和结果超时闭环 | 仅调查 |
| 9. 行为真实性 | 起飞/航点成功有遥测确认；手动/悬停文案描述的是“接受/接管”而非物理完成；LAND、频率和原点仍只是下游接受或发布成功 | 仅调查 |

## 二、实现内容

### 2.1 手动操纵提示与详细状态

- 八个运动按钮明确使用“向飞行器发送指令，增加……期望速度/角速度”的措辞，避免把“机载端接受期望量”写成“飞行器已经完成运动”。J 键提示包含任务要求的“增加水平向左的期望速度”。
- SPACE 明确提示为向机载服务发送“制动并悬停”指令、清零目标速度并保持当前位置。
- 正常可用时展示动作语义；安全门控禁用时优先展示具体禁用原因；恢复可用后自动还原动作 tooltip。
- tooltip 同步写入 `accessibleDescription`，便于辅助技术获取同一语义。
- “工程信息”改名为“详细状态”。现有位姿、目标、控制周期和安全门控留在左栏；右栏新增：

  | 键位 | 默认中灵敏度 | 低灵敏度 | 高灵敏度 |
  | --- | ---: | ---: | ---: |
  | W/S 升降 | ±0.20 m/s | ±0.10 m/s | ±0.40 m/s |
  | A/D 偏航 | ±11.5 °/s | ±5.7 °/s | ±22.9 °/s |
  | I/K 前后 | ±0.20 m/s | ±0.10 m/s | ±0.40 m/s |
  | J/L 横移 | ±0.20 m/s | ±0.10 m/s | ±0.40 m/s |

这些数值是“每按一下发送的期望量增量”，不是实际运动测量值；机载端仍会累加参考量并按既有安全上限裁剪。偏航内部单位仍是 rad/s，仅在 UI 中换算成 °/s 展示。

视觉核对覆盖 1400×980 和项目最小窗口 1180×700；两栏无文字裁切。证据截图：`agent/codex/task13-detailed-state.png`。

### 2.2 本地仿真启动优化

仅调整 `EnvironmentInitializer._simulation_workflow()`，实机 `_hardware_workflow()` 的启动次序和参数均未改变。

1. 四个互相独立的 ROS 包存在性检查改为最多四线程并行。
2. SITL 启动后立即预热 RViz；`robot_state_publisher`、`pose_to_tf` 和 RViz 可以先等待稍后出现的 MAVROS pose，无需阻塞在完整飞控就绪之后。
3. SITL TCP 5762 可用后，MAVROS 和机载控制进程背靠背启动，让 ROS 发现和进程初始化并行。
4. 已存在 `build/sitl/bin/arducopter` 时向 `sim_vehicle.py` 添加 `--no-rebuild`，避免每次启动重复执行 waf 检查/构建；首次安装或二进制缺失时仍保留自动构建。
5. 将飞控参数表首次检查等待改为参数化配置：生产/实机默认保持 40.0 秒，只有仿真启动命令覆盖为 2.0 秒。此优化不会让实机过早把参数同步暂态判成故障。
6. RViz 在末尾只做短稳定性确认，不再重新启动或额外串行等待 2 秒。

没有删除现有可视化组件：URDF/静态 TF、pose→TF 桥和 RViz 均被当前配置实际使用。性能问题不是“启动了完全无用的服务”。

## 三、详细状态候选信息调查

建议继续维持“飞控/MAVROS → 机载聚合 `ControlStatus` → 地面站”的边界。地面站若直接订阅 `/mavros/*`，会绕过当前接口版本、仿真/实机切换和单一权威快照设计，并增加跨 Humble/Jazzy DDS 端点数量。

| 信息 | 当前可用来源 | 当前接口缺口 | 推荐后续实现 |
| --- | --- | --- | --- |
| 滚转/俯仰角 | `/mavros/local_position/pose` 四元数或 `/mavros/imu/data`；机载已经订阅前者 | 机载只从四元数提取 yaw，`ControlStatus` 没有 roll/pitch | 机载统一解算 roll/pitch，加入聚合状态；地面站只显示聚合值与新鲜度 |
| 地面站↔机载状态频率 | 地面站已有 `ControlStatus` 本地到达时间统计 | 正式详细状态未展示 rate/max-gap/age | 展示实测 Hz、最大间隔和最后更新时间 |
| 机载↔MAVROS/飞控频率 | 机载 pose、velocity、state、intent 回调时间；MAVROS diagnostics/MAVLink link 统计 | `message_rates_configured` 只是配置 ACK，不是实际到达频率 | 在机载统计每路 observed/expected/age；链路层数据单独标注，不合并成一个“通信频率” |
| 电池 | MAVROS sys_status 插件发布 `/mavros/battery` (`sensor_msgs/BatteryState`)，来自 `SYS_STATUS`/`BATTERY_STATUS` | 机载未订阅，`ControlStatus` 无电池字段；值还依赖飞控电池监视器配置 | 机载聚合电压、电流、剩余比例、数据有效性和年龄；未知值显示“不可用”而不是 0 |

适合后续放入右栏的优先信息：状态/位姿新鲜度、实测状态频率和最大断流、实际 XYZ 速度、当前飞控模式、租约剩余时间、当前命令序号、消息频率配置状态、悬停油门校准值、命令最终结果 RTT、控制输出是否触顶、故障进入与恢复时间。`ControlStatus` 已有租约剩余时间和活动命令序号，但 Python `VehicleSnapshot` 尚未保留这两个字段，是低成本候选。

## 四、两个 MAVROS 告警

### 4.1 `CMD: Unexpected command 520, result 0`

520 是 `MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES`。协议规定接收端先回 `COMMAND_ACK`，随后发送 `AUTOPILOT_VERSION`。MAVROS 2.14 的 command 插件只为等待同步应答的命令登记事务；广播命令不登记，但收到其合法 ACK 时仍走统一查找路径，于是打印 `Unexpected command`。`result 0` 是 accepted，不是失败。

本地 SITL 和 ArduCopter 4.6.3 真机均复现：该 WARN 后紧接 capabilities/version 信息，连接、参数同步和状态流正常。因此不修改项目代码，也不通过过滤器掩盖潜在的其他 command WARN。

参考：

- [MAVLink `MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES`](https://mavlink.io/en/messages/common.html#MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES)
- [MAVROS 2.14 command 插件源码](https://github.com/mavlink/mavros/blob/2.14.0/mavros/src/plugins/command.cpp)
- [MAVROS 2.14 sys_status 能力请求源码](https://github.com/mavlink/mavros/blob/2.14.0/mavros/src/plugins/sys_status.cpp)

### 4.2 `STAT_RUNTIME (65535/1246) ... different index: 1201/1246`

MAVROS 参数插件会缓存参数值和索引。`STAT_RUNTIME` 可先作为无索引的运行时更新到达（索引 65535），随后在完整参数拉取中以真实索引 1201/1246 再次到达；值可正常更新，但插件会把索引变化打印为 WARN。

真机冷启动和重启各复现一次：一次约 0.9 秒后完整参数表成功，一次经历对单项参数的 1 次重试后约 2 秒成功；随后 `GUID_OPTIONS`、`MOT_THST_HOVER=0.20922` 均验证通过，机载进入 READY。该告警本身无实际影响。只有出现“参数列表始终未完成”、`thrust_mode_verified` 长期为 false 或关键参数读取失败时，才应按真实故障处理。

参考：[MAVROS 2.14 param 插件源码](https://github.com/mavlink/mavros/blob/2.14.0/mavros/src/plugins/param.cpp)。

## 五、仿真启动性能结果

在同一台开发机、同一生产初始化入口、全程未武装条件下记录单调时钟。机器为 i7-9700KF（8 核）、31 GiB 内存，测试时约 21 GiB 可用、load 约 0.9，没有 CPU 或内存饱和证据。

| 里程碑 | 修改前 | 最终版本 |
| --- | ---: | ---: |
| SITL 启动 | T+2.996 s | T+1.096 s |
| RViz 启动 | T+50.472 s | T+1.097 s |
| MAVROS/机载开始启动 | T+7.356 / T+8.738 s | 两者均约 T+1.794 s |
| 进入第 4 阶段（频率/EKF） | T+50.465 s | T+16.937 s |
| 全部门控就绪 | T+52.866 s | T+43.487 s |
| 清理完成 | T+53.610 s | T+44.213 s |

最终安全就绪时间减少 9.379 秒，即 17.74%；RViz 提前 49.375 秒出现。最终快照的 onboard、FCU、控制权、位姿、推力参数和消息频率门均为 true，`armed=false`；四个受管进程均被清理，无 TCP 5762 或项目进程残留。

剩余约 26 秒的关键路径位于第 4 阶段：ArduPilot 在仿真时间推进中设置 EKF origin，随后 EKF3 才切换到 GPS。最终就绪点与“EKF3 ... using GPS”相邻。继续强行缩短将意味着跳过现有 local pose/EKF 安全门，不属于可接受的性能优化。

## 六、机载重启后地面站日志现象

地面站只在实机会话或 Wi-Fi 被动检测中订阅 `/rosout`，QoS 为 reliable + transient-local、深度 1000，并把消息标为 `remote-rosout:<logger>`。它没有读取 SSH PTY，也不会镜像 shell 的普通 stdout；只有 ROS logger 写入 `/rosout` 的内容会进入地面站。

实测步骤与任务描述一致：先连接、停止机载完整栈、再启动，地面站观察到 onboard/FCU 离线后恢复在线，收到 190 条 `remote-rosout:*` 启动事件，多数是 MAVROS 插件 INFO；最终状态恢复 connected/READY、`armed=false`，地面站仍为 `control_enabled=false`。这证明现象是当前设计的预期行为。

不过它存在可用性问题：

- `/rosout` 是 ROS domain 全局日志，不等于“仅该无人机的机载服务日志”；当前只排除 `ground_station_client`，没有主机、节点或会话白名单。
- transient-local 允许新订阅者收到发布者保留的历史日志，容易让用户误以为旧行刚刚发生。
- MAVROS 启动会枚举大量插件，INFO 量大，掩盖真正的飞行安全事件。

后续应保留远端日志能力，但增加 logger allowlist/会话时间边界、历史补发标记、同类聚合与去重；不要把整条 `/rosout` 直接等同于“机载终端”。

## 七、日志完整性与等级审查

### 已有的合理覆盖

- 地面站对 onboard 在线/离线、接口版本、FCU 连接、控制租约、重复 status publisher、武装变化、控制模式、setpoint 多发布者、failsafe 原因和控制 deadline miss 有转换日志。
- `ControlStatus.status_message` 只在文本变化时更新，避免 10 Hz 周期重复；地面站把这条聚合状态变化记为 DEBUG。
- 真机 `/rosout` 保留 ROS 原始严重度，WARN/ERROR 不会被降级。

### 主要问题

1. 机载 `set_status_message()` 对所有变化统一发 `RCLCPP_INFO`，而相同文本又随 `ControlStatus` 在地面站生成 DEBUG；启用远端 `/rosout` 时形成语义重复。手动参考更新、航点进度等高频用户态文本不都应是 INFO。
2. `local_position_valid`、`thrust_mode_verified`、`message_rates_configured`、飞控模式变化没有完整的进入/失效/恢复成对日志；用户难以判断门控何时恢复。
3. setpoint conflict 有“出现”ERROR，但缺明确“解除”INFO；failsafe 有原因变化，但正常恢复的闭环提示不够统一。
4. 状态新鲜度、pose/state age、状态到达率下降尚未形成阈值化告警；只看到最终离线时丢失诊断上下文。
5. 全局 `/rosout` 带来大量 MAVROS 插件启动 INFO，面向一般用户的默认日志噪声偏高。

建议等级：生命周期和用户明确操作为 INFO；安全门失效、重试和退化为 WARN；冲突、失联保护和命令确定失败为 ERROR；重复接受的运动参考、内部进度和正常参数枚举为 DEBUG。恢复事件应与故障事件成对，并携带 source/event id 便于地面站去重。

## 八、通信成功标准与握手机制审查

当前并非“topic 能收到就算成功”，已有多层应用握手：

1. `ControlStatus` 以 10 Hz 聚合发布；地面站以 2.5 秒本地到达新鲜度判在线，并检测接口版本 2.0。
2. 连续图查询发现多个 `/onboard_control/status` 发布者时锁定 endpoint conflict，停止控制传输。
3. 地面站以唯一 `source_id`、单调 sequence 和 3 秒租约申请控制；可靠 heartbeat 续租，断连主动 release，机载按 TTL 自动回收。
4. 正式实机流程依次等待 onboard、接口、租约、FCU、推力参数、消息频率和 local pose 安全门。
5. 高层服务先返回“接收/拒绝”；异步任务再通过 reliable `CommandResult` 携带同一 source/sequence、RUNNING 和 final 结果。
6. Wi-Fi 被动检测不申请租约，要求 3 秒窗口至少 5 Hz 且最大断流不超过 0.5 秒。

主要薄弱点：

- 没有 `boot_id/session_id`、状态序号或能力 bitmap；节点重启后只能靠时间新鲜度和本地 grant hint 修正。
- 地面站用本地到达时钟判断 freshness，未验证消息 header 时间戳/时钟偏差。
- 服务已接受后若机载进程在 final result 前退出，GUI pending 命令没有统一最终结果 watchdog，可能长期等待。
- 消息频率和 GPS 原点成功主要证明下游服务接受/发布，没有飞控“已应用状态”的独立读回。
- Humble/Jazzy 跨发行版只验证部分端点可传输，不能当作全图协议兼容握手。

后续优先加入：随机 boot id、递增 status sequence、capability flags、每张 command ticket 的最终期限、header 时间合理性检查，以及对频率/原点等配置的 applied-state 证据和 RTT。

## 九、日志与真实行为一致性

| 操作/日志 | 当前“成功”的真实标准 | 评价与边界 |
| --- | --- | --- |
| 起飞 | MAVROS 起飞服务先接受；最终还要求 `armed=true`、pose 有效、Z ≥ 请求高度−0.1 m，否则超时失败并请求 LAND | 项目内最强，不能仅因 ACK 宣称起飞；仍依赖同一套飞控定位遥测，不是独立外部真值 |
| 航点完成 | 三维位置进入容差并持续 `waypoint_hold_seconds` 后才前进，全部航点完成后 final success | 与估计位姿一致；定位漂移/欺骗仍会造成“估计已到、物理未到” |
| 手动运动 | 机载接收、裁剪并更新内部期望参考 | 文案写“接受运动参考/增加期望速度”才真实，不能写“飞机已左移” |
| 制动并悬停 | 控制器接管、目标速度清零并捕获位置参考 | 表示悬停控制已接管，不证明已经达到稳态或零速度；当前 tooltip 已按此表述 |
| LAND | MAVROS `SetMode` 返回 `mode_sent=true` | 当前结果写“降落指令已发送 — LAND 模式”，没有虚称“已着陆”；仍缺 touchdown、disarm 最终确认 |
| 消息频率配置 | 三个 MessageInterval 服务逐项返回 success | 证明请求被接受，不证明实际 100 Hz；应增加回调实测速率读回 |
| GPS 原点设置 | 机载向 ROS topic 重复发布请求 | 目前缺订阅者计数、MAVLink ACK 或飞控原点读回；成功措辞的证据最弱 |
| 日志传输 | `/rosout` 被地面站接收并按严重度入库 | 证明某 ROS logger 发过文本，不证明文本描述的物理事件真实；真实性必须来自状态机/遥测门 |

因此项目可以避免最典型的“起飞服务 ACK 就显示已经起飞”，但不能声称所有日志都与物理世界形成密码学或独立传感器级一致。后续路线应把日志分成 `accepted`、`applied`、`observed` 三类证据，UI 不把前两类翻译成物理完成。

## 十、验证记录

### 本地静态、单元与构建

- 第一次在未 source 本地 ROS overlay 的 shell 中运行全量 pytest：62 passed、2 failed；失败均为 `libguided_interfaces__rosidl_generator_py.so` 不在动态库路径，属于测试环境未加载，不是断言或代码失败。按项目正常入口加载 Jazzy 与 `install/setup.bash` 后复测通过。
- 最终全量 Python：65 passed（19.86 秒）。
- `colcon build --packages-select guided_interfaces onboard_control guided_sim`：3 包成功。
- `colcon test` + `colcon test-result --verbose`：5 tests、0 errors、0 failures。
- `compileall`、修改文件 flake8（致命规则及 E501/88 字符）和 `git diff --check`：通过。
- 新测试覆盖 tooltip、无障碍描述、详细状态左右栏、灵敏度实时数值、禁用/恢复 tooltip、四包并行预检、RViz/MAVROS/onboard 启动顺序、仿真专用 2 秒覆盖、实机流程保持不变，以及 SITL 二进制存在/缺失两条 `--no-rebuild` 分支。

### 未解锁 SITL

- 修改前与最终版均走真实生产初始化入口。
- 最终所有安全门 true、`armed=false`，未触发任何飞行动作。
- 清理后 4 个受管进程、TCP 5762 和相关项目进程均无残留。

### 实机与 Humble 兼容性

- 两轮启动真实 ArduCopter 4.6.3 + MAVROS + onboard，用于复现两条 WARN、参数完成和 READY；两轮最终均 `armed=false`。
- 被动地面站重启观察只接收状态和 `/rosout`，`control_enabled=false`，未申请控制租约或调用飞行服务。
- 为避免覆盖真机已有脏工作树，把本次 `guided_interfaces` 和 `onboard_control` 复制到 `/tmp/task13-humble-build.QDadmz` 独立构建；Humble/aarch64 两包构建成功，5 tests 零失败。
- 隔离 smoke 节点使用 `/task13_no_mavros` 和 `/task13_smoke` 前缀，不连接真实 MAVROS，读取 `fcu_parameter_check_initial_delay_seconds = 40.0`，证明实机默认未被仿真优化改变。
- smoke 的 `ros2 run` 包装进程退出后发现一个子节点残留，随即按 PID/完整命令行核对并终止；该节点始终使用隔离 MAVROS 前缀。最终删除临时目录，`drone-control.service=inactive`，无 mavros/onboard/arducopter 进程，串口无人占用。

真机脚本还发现一个既有环境易用性问题：若调用前没有 source `/opt/ros/humble/setup.bash`，脚本会在其自身加载 ROS 之前先因找不到 `ros2` 退出。本任务不包含脚本修改，列入后续路线。

## 十一、后续修改优先级

1. 先为 `ControlStatus` 增加 roll/pitch、电池有效性/年龄、各输入流 observed rate/age，再由地面站展示；不要让 GUI 直连更多 MAVROS topic。
2. 为命令加入 boot/session id、最终结果 deadline，并把成功证据显式分为 accepted/applied/observed。
3. 给消息频率、GPS 原点和 LAND 加实际状态读回；优先修复目前证据较弱的“成功”。
4. 收紧 `/rosout` logger 范围，标记历史补发并去重；同时把机载状态消息按重要性拆分 DEBUG/INFO/WARN/ERROR。
5. 补齐安全门“失效—恢复”成对日志和状态/pose 新鲜度退化告警。
6. 修复真机一键脚本的 ROS 环境自举顺序；继续把跨 Humble/Jazzy DDS 视为兼容风险，不能以部分 topic 可通替代正式支持结论。

## 十二、改动文件

- `ground_station_core/qt_ui/operations_panel.py`
- `ground_station_core/environment.py`
- `src/onboard_control/include/onboard_control/onboard_control_node.hpp`
- `src/onboard_control/src/onboard_control_node.cpp`
- `src/onboard_control/config/control.yaml`
- `src/guided_sim/launch/visualize.launch.py`
- `tests/test_qt_gui.py`
- `tests/test_environment_communication.py`
- `agent/codex/task13-detailed-state.png`
- 本报告与 `MEMORY.md`
