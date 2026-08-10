# 真机同步与四组件一键启动实测简报（2026-08-10）

## 任务结论

当前机载修改已同步至 `/home/onboard/ros2-ardupilot-mavros-control`，并在真机的
Ubuntu 22.04 / ROS 2 Humble 环境完成 Release 构建、单元测试、隔离烟雾测试和两轮真实
通信链启动验证。新增根目录脚本 `start_all.sh`，可用一行命令替代原先依次执行的四个
`start_*.sh`。

整个过程始终确认飞控为 `armed=false`、`guided=false`、`STABILIZE`。没有请求模式切换、
解锁、起飞、控制租约、GPS 原点或飞行指令，也没有发布姿态/推力 setpoint。

## 一键启动与退出

真机桌面终端执行：

```bash
cd /home/onboard/ros2-ardupilot-mavros-control && bash start_all.sh
```

脚本统一启动以下四部分：

1. MAVROS，默认 `/dev/ttyTHS1:460800`；
2. Odin 驱动；
3. Odin 到 MAVROS 的 extnav 桥；
4. `onboard_control_node`。

四部分继承同一个 ROS domain，当前默认 domain 0。脚本完成静态前置检查、拒绝已有实例、
按独立进程组启动并汇总日志；只有在 FCU 已连接且未解锁、消息频率配置完成、推力参数确认、
本地位置有效后才打印 `READY`。同一终端按 `Ctrl+C` 会停止本次启动的全部进程，日志保留在
`/tmp/ros2_ardupilot_onboard/<时间戳>/`。

原四个脚本均未删除或改写，仍可人工回退。部署前 SHA-256 为：

- `start_1.sh`: `a4d435b042b18c7bf27fd01d1e9743179716f9e3ae4f55e8788a9f91459826c8`
- `start_2.sh`: `350a50257b03c3f1fc7a65113fdda00662bec694885372c27b48e86aaf5234be`
- `start_odin.sh`: `aae5fa7b3abc2c5db0b64f2e86ccea7add8afddf8ea7ad9b6f29090725a244ed`
- `start_extnav.sh`: `5fb5f9ebabb377cd417c8a7232572ce75c45a10ddfd5c69b6bf1fdb8b5ffb337`

完整备份位于：

```text
/home/onboard/ros2-ardupilot-mavros-control/.deployment-backups/
pre-integrated-start-20260810-205108.tar.gz
```

备份 SHA-256 为
`ea0ec5bdde38560abf2f47ae594a04a082d16d45879b7029bfde1585d8ab027b`。

## 同步内容

- 机载控制节点：连接/重连后自动异步配置 MAVLink 消息 32、31、105 为 100 Hz；等待 MAVROS
  参数同步后确认 `GUID_OPTIONS` 与 `MOT_THST_HOVER`，并减少启动暂态误报。
- `start_all.sh`：四组件生命周期、就绪检查、重复实例保护、合并日志和整组退出。
- 部署说明：加入一键启动、停止、日志、配置覆盖和已知限制。
- extnav 集成命令显式应用 `T=(0.06,-0.03,0.05)`；原 `start_extnav.sh` 中注释与续行组合可能
  使这组三轴偏移没有传入，集成运行日志已确认新值生效。

最终机载 `start_all.sh` 与开发机文件 SHA-256 均为
`4b8a3307a71fecb84904df5374b90d1d54309b18181d5a5f1cdece4b5c624452`。

## 验证结果

### 构建与自动化测试

- 真机执行 `onboard_workspace.sh verify`：Humble 依赖检查通过，`guided_interfaces` 与
  `onboard_control` Release 构建成功。
- 真机 ROS/C++：5 tests，0 errors，0 failures，0 skipped。
- 真机隔离 domain 231 / localhost-only 烟雾测试：接口 `2.0`、FCU 未连接、未解锁、
  2 秒窗口内姿态 setpoint 为 0。
- 开发机 Python 全量回归：51 passed（13.76 秒），包含新增一键脚本结构、安全边界和 shell
  语法检查。

### 真机通信链

一键脚本连续执行两轮，均到达：

```text
[startup] READY: FCU connected, unarmed, rates/parameters/local position verified
```

第一轮 12 秒只读采样结果：

| 话题/状态 | 实测结果 |
|---|---:|
| `/mavros/local_position/pose` | 93.72 Hz，最大间隔 0.019 s |
| `/mavros/local_position/velocity_local` | 93.72 Hz |
| `/mavros/state` | 0.92 Hz |
| `/onboard_control/status` | 10.00 Hz，最大间隔 0.102 s |
| `/odin1/odometry` | 326.99 Hz |
| extnav pose / velocity | 100.06 / 99.97 Hz |
| `/mavros/vision_pose/pose` | 39.99 Hz |
| 姿态 setpoint | 0 条 |

聚合状态确认接口 `2.0`、本地位置有效、三路消息频率已配置、推力模式已确认、
`MOT_THST_HOVER=0.20922`、控制器禁用、租约为空、setpoint 冲突为假。控制循环报告约
100.02 Hz，最大抖动 3.711 ms，deadline miss 为 0。

开发机生产版 `GroundStationRosController` 只读接入 domain 0 后，3 秒收到 30 条状态，最大
间隔约 0.104 秒；FCU 连接、本地位置、消息频率和推力检查均为真，控制权、控制器和租约均为
假。该探针从未进入控制流程，因此停止时也没有发送释放租约请求。

在第一轮仍运行时再次执行 `start_all.sh`，脚本以返回码 1 拒绝重复实例并列出现有进程，原
四组件未受影响。两轮结束均通过 `Ctrl+C` 整组退出；最终检查无 MAVROS、Odin、extnav、
`onboard_control` 残留，`/dev/ttyTHS1` 无占用，相关 ROS 话题消失。

## 发现并处理的问题

1. 首轮最初在加载 Humble 环境时因 `set -u` 与
   `/opt/ros/humble/setup.bash` 的未定义变量不兼容而提前退出，此时任何组件都尚未启动。
   已用临时关闭 nounset 的 `source_setup` 包装修复，重新同步后两轮实机启动均通过。
2. Odin 的既有 launch 会同时启动 RViz。纯 SSH 无 `DISPLAY` 时 RViz 子进程会因 xcb 后端
   退出；Odin 驱动、点云/里程计、extnav 和其余通信链继续正常工作并通过 `READY`。需要
   RViz 时应从真机桌面终端执行一键命令。
3. Jazzy 地面客户端加入 Humble ROS 图后，机载各终端仍会成批打印
   `sequence size exceeds remaining buffer`。告警出现期间上述状态和位置数据仍按实测频率
   到达，socket 队列也没有阻塞证据，因此本轮结果再次不支持“地面站消息量过大堵塞通信”
   的解释。它仍属于 Humble/Jazzy Fast DDS 发现/类型兼容问题；自动设置 MAVLink 频率只解决
   飞控不主动输出 local-position 的 0 Hz 问题，不会修复该 DDS 告警。

## 最终状态与限制

- 部署、构建和通信测试已完成，最终由本任务启动的四组件全部停止；真机未解锁。
- 未执行实飞测试，不能用本报告替代用户的人工安全检查和飞行验证。
- 真机 sparse 仓库因本次部署包含本地修改和备份；没有 reset、commit、pull、推送或清理
  用户原有文件。
- Humble/Jazzy 跨发行版 DDS 告警仍未根治。长期方案仍是统一 ROS 发行版与 RMW/Fast DDS
  版本，或部署受支持的桥接后重新验证。
