# 真机机载服务同步与未武装测试简报

日期：2026-08-11

## 一、完成结论

真机 `xld@192.168.112.186` 已从旧的 `c8abad9` / 接口 2.0 快进并原生编译到 `7c1f8cc` / 接口 2.1。真机 `HEAD` 与 `origin/main` 均为 `7c1f8cc`，跟踪文件无差异。

为“消息频率真实确认”增加的两路 IMU 订阅已确认不存在：源码与真实 ROS 图中均无 `/mavros/imu/data` 和 `/mavros/imu/data_raw` 机载订阅。

## 二、同步与回滚保护

- 同步前确认 systemd 服务 inactive、四组件零进程、`/dev/ttyTHS1` 无占用。
- 旧正式工作树、构建和安装前缀已备份到：
  `/home/onboard/ros2-ardupilot-mavros-control/.deployment-backups/pre-sync-7c1f8cc-20260811-194224/`。
- 快照 `workspace-snapshot.tar.gz` SHA-256：
  `594b12d8b638f3663ecf67552a72ccb619b3fdc91d83fef61ecab24d5522a4cc`，最终 `sha256sum -c` 通过。
- 真机 GitHub 直连失败后，按用户指示临时执行 `clashon`，以 HTTP/1.1 完成 fast-forward；同步后已执行 `clashoff`，代理进程与临时传输包已清理。

## 三、Humble/aarch64 原生验证

- 依赖检查：Humble 下 16 个 ROS 依赖全部就绪。
- Release 构建：`guided_interfaces` 与 `onboard_control` 通过。
- ROS/C++ 测试：`5 tests, 0 errors, 0 failures, 0 skipped`。
- 隔离 smoke：`interface=2.1`、`fcu_connected=false`、`armed=false`、`setpoint_messages=0`。
- 便携启动器检测到 `/dev/ttyTHS1` 与 `/dev/ttyTHS2` 两个串口后正确拒绝猜测。根据旧已验证启动器的明确配置，systemd drop-in 固定 `MAVROS_FCU_DEVICE=/dev/ttyTHS1`。

## 四、真实四组件链路测试

仅启动 MAVROS、Odin、extnav 和 `onboard_control_node`，未创建地面站控制会话，未发送模式、解锁、起飞、原点、运动或航点命令。

- 聚合状态样本：490 条，平均 `9.9994 Hz`，最大间隔 `326.561 ms`。
- 接口 `2.1`，FCU 连接成功，全程 `armed=false`，飞控模式 `STABILIZE`。
- 控制租约 false、控制器 active false、姿态 setpoint 实收 `0` 条。
- `message_rates_configured=true` 只代表四个 MessageInterval ACK 已成功；本次没有恢复或执行 100 Hz 实测确认。
- 本地位姿有效，`GUID_OPTIONS` bit 3 与 `MOT_THST_HOVER=0.20922` 已确认。
- 电池有效，实测电压 `23.435 V`；俯仰约 `-0.125°`，偏航约 `0.336°`。
- 真实 ROS 图检查中两路 IMU 机载订阅数为 0。

## 五、服务交付状态与限制

- systemd 仍为 `enabled`，测试后为 `inactive/dead`、`Result=success`。
- drop-in 同时设置 `SuccessExitStatus=130`，避免把监督脚本收到 SIGINT 后的正常停止误记为服务失败。
- 最终四组件进程数为 0，`/dev/ttyTHS1` 无占用。
- Odin 外部驱动在停止时仍输出 `calib.yaml` 缺失与一次 abort 日志，extnav 输出 `ExternalShutdownException`；它们发生在 SIGINT 关停阶段，未影响 READY 或留下进程/串口占用，但不应将关停日志宣称为完全无告警。

全程未解锁、未起飞。
