# GPS 原点重复连接与失败彻底断开修复简报

日期：2026-08-11

提交：`91ccc758175abd1691945b5655051ced5cf6be5d`

目标机：`xld@192.168.112.186`（Ubuntu 22.04 / ROS 2 Humble / aarch64）

## 一、结论

本次两个严重问题均已定位、修复并在真机未武装状态下验证：

1. 飞机不重新上电时，重复连接不再卡在 GPS 原点回读超时。机载端会把当前 FCU 会话中已经观察到且与请求匹配的 `gp_origin` 视为幂等成功，不再错误地强求飞控重复广播未变化状态。
2. 仿真/实机完整环境初始化失败、取消或前检查失败时，地面站不仅释放租约和清理本地进程，还会销毁当前 ROS 2 context。GUI 回到初始 `ROS IDLE` 后不再继续接收真机状态、遥测或远端日志。

实现提交 `91ccc75` 已推送 `main` 并同步真机，受测机载二进制由该提交构建。机载 Humble 原生构建和测试通过，最终 systemd 服务为 `enabled + active/running`。

## 二、安全边界

- 全程没有请求或执行解锁、起飞、模式切换、运动、悬停、航点或降落。
- 所有真实连接维护仅包含控制租约、心跳、消息频率 ACK 和 GPS 原点幂等确认。
- 每次真实连接前后均确认 `armed=false`、`STABILIZE`；最终无租约、控制模式待机、控制器未激活。
- 更新前向手动监督脚本 `bash ./start_drone_all.sh` 发送 `SIGINT`，由既有脚本清理四组件；没有逐进程强杀。构建/测试完成后由已启用的 systemd 服务恢复运行。

## 三、根因证据

### 3.1 GPS 原点超时

真实 ROS 图的只读检查确认：

- `/mavros/global_position/gp_origin` 发布端 QoS 为 `RELIABLE + TRANSIENT_LOCAL`；
- 缓存回读为 `30.2489634, 120.2052342, 487.995768...`；
- GUI 默认请求为 `30.2489634, 120.2052342, 488.0`，经纬度完全一致，高度误差约 4.2 毫米，明显在既有 0.5 米容差内；
- 连续 28 秒订阅没有收到新消息，证明它不是周期状态流。

`af01e8b` 引入“必须观察飞控回读”后，`on_global_origin()` 只在 `origin_confirmation_active_` 已经置位时确认成功。机载节点启动时虽已从 transient-local 缓存收到正确原点，但当时尚无请求，代码只保存数值、不保存“已经观察”事实。之后重复发布同一个原点时，ArduPilot 不保证再次广播未变化的 `GPS_GLOBAL_ORIGIN`，于是固定在 8 秒后失败。

这不是消息丢失，也不是容差错误；是把已应用状态错误实现为“必须在请求之后新到达的事件”。

### 3.2 失败后仍显示飞控连接与实时遥测

`EnvironmentInitializer._start_workflow()` 的异常路径原先只调用 `_terminate_local_processes()`。该函数会：

- 关闭远端日志开关；
- 释放控制租约；
- 清理本项目本地仿真进程；
- 清空一次本地快照。

但它不会调用 `GroundStationRosController.stop()`。domain 0 的 DDS participant 和 `/onboard_control/status` 订阅仍然运行，下一条约 10 Hz 状态立刻把快照重新填满，因此出现最终错误之后又记录“机载服务已发现/飞控链路已连接”，GUI 高度、航向等也继续更新。

## 四、修改内容

### 4.1 机载 GPS 原点改为幂等的已应用状态确认

文件：

- `src/onboard_control/include/onboard_control/onboard_control_node.hpp`
- `src/onboard_control/src/onboard_control_node.cpp`

行为：

- 新增 `global_origin_observed_`，区分默认零值与真实 FCU 回读；
- 收到 `gp_origin` 时保存其“已观察”事实；节点刚启动、transient 消息早于第一条 FCU State 到达时允许暂存，若随后确认断线则立即作废；
- 已知 FCU 断线时清除该事实，禁止用上一条 FCU 会话的缓存确认新请求；
- 若当前 FCU 已连接、已有真实回读且请求值匹配，直接发布可靠 final success：`当前值已匹配，无需重复写入`，不再发送无意义的第二轮写请求；
- 若缓存不存在或不匹配，仍沿用原有发布后等待匹配新回读、8 秒超时失败的严格逻辑，没有把“发布成功”降格冒充“应用成功”。

### 4.2 所有完整环境失败统一销毁 ROS context

文件：`ground_station_core/environment.py`

行为：

- 新增 `_terminate_environment_session()`，统一执行租约/进程清理和 `self._ros.stop()`；
- 完整仿真或实机工作流的前检查失败、用户取消、运行异常和主动断开全部走同一会话边界；
- 纯 Wi-Fi 只读通讯检测仍保持原职责，不申请/释放租约、不管理进程，也不会错误复用完整环境清理；
- 最终错误文字明确为“本地 ROS 连接与仿真进程已清理”，并在工作流报告最终失败前同步完成 context 停止流程。

### 4.3 回归测试

文件：

- `tests/test_onboard_truthfulness.py`
- `tests/test_environment_communication.py`

新增覆盖：

- 首次设置仍必须等待匹配回读；错误回读不能提前成功；
- 同一 FCU 会话第二次请求相同原点立即 final success，且原点发布请求计数不增加；
- 完整实机连接在已建立状态/日志/控制链后失败时，DDS context、domain、控制开关和远端日志开关全部归零；
- 独立通讯检测失败/取消的零命令边界继续由原测试保护。

## 五、验证结果

### 5.1 本地自动验证

| 项目 | 结果 |
| --- | --- |
| Jazzy Release 三包构建 | 通过 |
| Python 全量 | `75 passed` |
| ROS/C++ | `5 tests, 0 errors, 0 failures` |
| 修改范围 flake8 致命项 | 通过 |
| `compileall` | 通过 |
| 修改范围 `git diff --check` | 通过 |
| 干净环境 `ground_station.py --check-environment` | 通过 |

### 5.2 真实旧机载端失败闭环

在部署机载修复前，使用修改后的生产地面站工作流连接旧节点，真实复现 GPS 原点 8 秒回读超时。失败收尾结果：

- `ready=False`；
- `domain=None`；
- `control=False`；
- `VehicleSnapshot` 的 onboard/FCU/位姿/电池/租约等均恢复默认值；
- 最终错误之后观察 3 秒，晚到的 onboard/flight-controller/remote-rosout 链路事件为 0；
- 飞机最终仍为 `armed=false`、`STABILIZE`、无租约。

这直接验证了用户所述“错误后又显示飞控已连接并继续同步遥测”的真实修复，而不只是替身单测。

### 5.3 真机部署与重复连接

- 真机通过 SHA-256 校验的 Git bundle 严格快进到 `91ccc75`；GitHub 上同一提交已经推送至 `main`；
- Humble/aarch64 Release 构建成功；
- 真机 `onboard_workspace.sh test` 为 `5 tests, 0 errors, 0 failures`；
- 飞机未重新开机，仅重启机载四组件后，生产地面站连续执行两轮完整连接/断开；
- 第 1 轮 `set_gp_origin ticket=2`、第 2 轮 `ticket=4` 均立即返回：
  `GPS 原点已由飞控回读确认，当前值已匹配，无需重复写入`；
- 两轮连接均报告 `armed=false`、`STABILIZE`、控制租约有效、本地位置有效；
- 两轮断开均为 `CleanupReport(... errors=())`，2 秒观察窗内晚到链路事件均为 0；
- 最终状态：`armed=false`、`STABILIZE`、`control_mode=IDLE`、`controller_active=false`、无租约、`setpoint_conflict=false`；姿态 setpoint 为 1 个发布者/1 个订阅者；
- systemd 服务最终 `enabled`、`active/running`、`Result=success`。

## 六、已知独立现象

两轮地面站 Jazzy DDS context 建立/销毁期间，真机 Humble/Fast DDS 的 MAVROS/Odin 进程仍记录了既有的 `sequence size exceeds remaining buffer`。该跨发行版反序列化告警在任务 12.5 已记录，本次修改没有触碰 DDS 类型、QoS 或发现配置；告警没有造成状态中断、租约失败、原点失败或进程退出。

本任务没有把这一独立问题伪称为已解决，也没有用屏蔽日志的方式掩盖它。
