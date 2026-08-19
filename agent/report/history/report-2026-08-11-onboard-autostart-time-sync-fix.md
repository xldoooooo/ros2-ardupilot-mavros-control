# 真机机载自启动首次校时修复简报

日期：2026-08-11

## 结果

已确认问题来自开机启动时序，而不是串口路径、权限、波特率或 systemd/交互终端环境差异。
机载四组件原先在系统首次 NTP 校时之前启动；约 5 分钟的墙钟跳变使已创建的 MAVROS/ROS
定时链异常，最终表现为 `ControlStatus` 仍以约 10 Hz 发布，但 `fcu_connected=false`。

采用最小修复：继续使用 systemd 开机自启动，只让服务等待
`systemd-time-wait-sync.service` 完成并越过 `time-sync.target` 后再启动。不使用固定秒数
`sleep`，不修改 MAVROS、机载控制协议或飞控逻辑。

## 根因证据

同一次冷启动的 monotonic journal 顺序如下：

| 开机后时间 | 事件 |
| ---: | --- |
| 12.328 s | 原机载 systemd 服务启动 |
| 24.081 s | MAVROS 收到 ArduPilot HEARTBEAT，串口链路曾真实建立 |
| 39.641 s | `systemd-timesyncd` 首次 NTP 同步 |
| 约 43 s | MAVROS 报告 `Time jump detected`；ROS 时间由约 21:06 跳到 21:11 |

因此 `/dev/ttyTHS1:460800` 能正常打开，飞控也曾回复。冷启动实例在校时后退化；同机系统
时间稳定后，手工运行相同 `start_drone_all.sh` 约 6 秒即重新收到 HEARTBEAT，排除了固定环境
配置差异。

## 修改

- 真机 `/etc/systemd/system/ros2-ardupilot-onboard.service`：
  - `Wants=network-online.target systemd-time-wait-sync.service`
  - `After=network-online.target time-sync.target`
- 保留原服务的用户、工作区、ROS domain、FCU 串口、重启策略和四组件启动命令。
- 旧单元已备份为
  `/etc/systemd/system/ros2-ardupilot-onboard.service.pre-timesync-20260811`。
- 同步更新仓库内 systemd 示例、部署文档及配置回归测试。

## 验证

- 本地 `systemd-analyze verify`：通过。
- 部署专项测试：10 passed。
- 完整 Python 测试：正确加载 Jazzy 与项目 install overlay 后 75 passed。
  第一次未加载 `install/setup.bash` 的运行有 3 项因缺少
  `libguided_interfaces__rosidl_generator_py.so` 失败，属于测试终端环境错误，未作为通过结果。
- 真机执行 `systemctl start` 后，启动顺序的 monotonic 时间为：
  `systemd-time-wait-sync` 1,759,805,938 us，`time-sync.target` 1,759,806,906 us，
  机载服务 ExecStart 1,759,809,836 us，顺序符合配置。
- 启动器打印 `READY`，MAVROS 收到 ArduPilot HEARTBEAT。
- 地面机通过 domain 0/subnet 只读观测：
  `interface_version=2.1`、`fcu_connected=true`、`armed=false`、
  `local_position_valid=true`、`message_rates_configured=true`、
  `thrust_mode_verified=true`、`lease_owner=''`；状态流约 9.999 Hz。

## 安全边界与剩余验证

本次没有发送租约、模式、原点、解锁、起飞或飞行控制命令，飞机始终报告
`armed=false`。为避免未经确认重启整台真机，本次没有执行再次冷重启；下一次正常开机时仍应
保留一次 journal 观察作为最终冷启动复验。依赖单元已经由一次实际 systemd 启动事务拉起并
验证，当前机载服务保持 `enabled`、`active/running`，可供地面站连接。
