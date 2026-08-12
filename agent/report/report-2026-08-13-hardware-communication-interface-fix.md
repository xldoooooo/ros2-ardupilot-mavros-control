# 真机通讯检测与完整连接接口错配修复简报

## 1. 结论

- 日期：2026-08-13。
- 已复现并修复“启动真机机载服务后，地面站无法检测通讯链路、无法连接真机”。
- 主因不是 Wi-Fi、ROS domain 或飞控掉线，而是**源码与安装产物版本错配**：真机源码已经更新到
  3.0.0，但 systemd 加载的 `install/` 仍为 2.2.0；地面站按接口 3.0 解码旧
  `ControlStatus` 时无法得到有效样本。
- 真机现已原生构建并运行接口 3.0；地面站被动通讯检测和完整连接/断开均真实通过。
- 全程没有发送解锁、起飞、降落、运动、航点或姿态/推力命令；完整连接按产品既有设计申请/释放
  控制租约、确认消息频率并幂等确认 GPS 原点。

## 2. 故障证据与根因

### 2.1 可复现现象

修复前使用与 GUI 相同的 `EnvironmentInitializer.test_hardware_communication()`：

```text
1/2 正在被动等待机载 ControlStatus
通讯检测失败: 等待机载控制服务超时
snapshot.interface_version = ""
snapshot.onboard_available = false
```

地面端不是收到旧版本后主动拒绝，而是完全没有可解码的 3.0 状态。真机停止前日志同时反复出现
Fast DDS `sequence size exceeds remaining buffer`。

### 2.2 版本错配

真机同一工作区的四个 manifest 直接证明：

| 位置 | guided_interfaces | onboard_control |
|---|---:|---:|
| `src/` | 3.0.0 | 3.0.0 |
| `install/` | 2.2.0 | 2.2.0 |

旧进程启动日志也明确报告“机载控制服务 2.2”，地面站常量为 3.0。ROS 自定义消息结构已经新增轨迹
字段，2.2/3.0 不能作为同一线协议混用。前一任务只同步了 Git sparse 源码并刻意没有重建/重启，
但启动入口没有阻止用户随后运行旧安装产物，这是本次需要修复的部署缺口。

### 2.3 现场附加状态

- 开始本轮排查时 systemd 已于 `03:06:49` 被停止，状态为 inactive；无服务本身也会让任何检测
  超时，但它不能解释服务运行期间的 2.2/3.0 反序列化错误。
- 旧启动器使用隐式 QoS 且复用 ROS CLI daemon 的短时 `topic echo` 做 READY 检查，现场多次收不到
  best-effort 状态，导致进程仍运行但 120 秒后误报 full readiness 未达到。
- 旧节点在 SIGINT 关闭 context 时，`status_tick()` 仍可能调用 `count_publishers()`，现场产生
  `could not count publishers: rcl node's context is invalid` 并 abort；这不是通讯失败主因，但会
  污染正常服务重启结果。

## 3. 代码修复

### 3.1 禁止旧安装产物静默启动

- `start_drone/runtime_common.bash` 新增 package manifest 版本读取与源码/安装一致性检查。
- `start_drone_all.sh` 在启动 MAVROS、Odin、extnav、onboard 之前检查
  `guided_interfaces` 和 `onboard_control`。
- 真机实测旧安装树现在以退出码 1 安全停止，明确输出：

```text
stale guided_interfaces install: source=3.0.0, installed=2.2.0;
run onboard_workspace.sh verify before starting
```

检查失败时没有创建任何飞行栈进程。

### 3.2 稳定只读 READY 探针

- 显式使用 `guided_interfaces/msg/ControlStatus`，避免依赖 daemon 的类型推断。
- 使用 `--no-daemon --spin-time 1`，避开陈旧 CLI daemon 图缓存。
- QoS 固定为 best-effort、volatile，与机载状态发布者一致。

### 3.3 正常关闭竞态

- `status_tick()` 在 ROS context 已关闭时直接返回。
- 图查询与 SIGINT 发生竞态并抛出 `std::runtime_error` 时，仅在 context 确认失效后安全退出；运行期
  的真实图查询错误仍继续抛出，不会被粗暴吞掉。

## 4. 真机部署

- 修复提交：`0748dda`。
- 因真机 GitHub HTTPS 超时，使用只包含 `4741066..0748dda` 的 Git bundle 同步，bundle
  SHA-256 为 `6dd2465014f4cef65a677b579fa61df59fd146ae2cf9587c26eca63bac27dba9`。
- 部署前备份：
  `/home/onboard/ros2-ardupilot-mavros-control/.deployment-backups/`
  `pre-interface3-deploy-20260813-031710.tar.gz`。
- 备份 SHA-256：
  `7da02278f10b142a981b45715b30a8a72f79555e5af18ebbd6ab95cdcb972efa`。
- 真机 `onboard_workspace.sh verify`：Humble/aarch64 Release 两包构建成功，13 tests、0 errors、
  0 failures；domain 231/localhost 隔离 smoke 报告接口 3.0、FCU 未连接、未武装、零姿态消息。
- 构建后 `start_drone_all.sh --check` 成功且没有启动组件。

## 5. 真实链路验证

### 5.1 机载同机只读状态

systemd 启动后新版探针打印：

```text
READY: FCU connected, unarmed, rates/parameters/local position verified
```

权威状态为接口 3.0、FCU 已连接、`armed=false`、STABILIZE、本地位姿有效、消息频率 ACK 已完成、
`GUID_OPTIONS` bit 3 与 `MOT_THST_HOVER=0.20922` 已确认。

### 5.2 地面站被动通讯检测

使用 GUI 同一生产工作流实测：

```text
ControlStatus 31 条 / 10.29 Hz
最大接收间隔 110.9 ms
飞控已连接、未武装、租约持有者无
```

检测没有申请租约、发送心跳/维护/飞行命令或管理远端进程。

### 5.3 完整连接与断开

- 连续两轮使用生产 `initialize_hardware()` 成功。
- 两轮均确认接口 3.0、FCU 在线、未武装、控制租约属于本轮客户端、频率/推力/位姿门全部通过。
- 正常清理报告：`managed_stopped=0`、无 stale 进程、无残留、无错误；地面 DDS context 回到
  IDLE，真机租约清空。
- 最终只读状态：`armed=false`、STABILIZE、控制模式 IDLE、controller inactive、无租约、
  setpoint 冲突为 false。
- 对 `/mavros/setpoint_raw/attitude` 观察 3 秒，退出码 124，确认零姿态 setpoint 消息。

## 6. 本地回归

- `colcon build --packages-select guided_interfaces onboard_control guided_sim`：3 包成功。
- ROS/C++：13 tests，0 errors、0 failures、0 skipped。
- Python：104 passed。
- Bash 语法与 `git diff --check` 通过；本机未安装 `shellcheck`，因此该项未执行，不能报告为通过。
- 新增回归会构造 source 3.0/install 2.2，确认启动门拒绝；改为 3.0/3.0 后确认通过。

## 7. 未消除的边界

- 本轮真实连接期间，Humble/Jazzy Fast DDS 对其他跨发行版端点仍记录既有
  `sequence size exceeds remaining buffer`；当前计数不能解释为接口 3.0 仍失败，因为生产
  `ControlStatus`、租约服务和维护服务已连续实测成功。
- ROS 官方不保证 Humble/Jazzy 混合图兼容。本修复解决的是明确的 2.2/3.0 部署错配和启动探针
  缺口，不等同于消除了跨发行版 DDS 风险。实飞前仍应统一 ROS/DDS 版本或完成独立长期验收。
- 最新 1.00 m/s 平滑航点参数仍未做真机飞行验证；本轮只验证未武装通讯与高层连接，严禁把结果
  扩大为航点飞行验证。
