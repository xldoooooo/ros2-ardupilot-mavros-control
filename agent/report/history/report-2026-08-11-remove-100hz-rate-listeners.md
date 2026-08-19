# 移除 100 Hz 实测监听简报

日期：2026-08-11

## 一、完成结论

本次只撤销“消息频率真实确认”引入的高频监听和验证状态机：

- 删除机载节点对 `/mavros/imu/data` 与 `/mavros/imu/data_raw` 的两路订阅。
- 删除位置、姿态、IMU 三路样本计数、时间窗、新鲜度与频率门限检查。
- 删除为 45 秒实测窗增加的启动等待时间和 GPS 原点先后顺序调整，恢复原有 25 秒配置等待流程。
- `message_rates_configured` 恢复为四个 `MessageInterval` 服务 ACK 全部成功后置位，不再声称已实测 100 Hz。

机载生产节点本次净减少 158 行头文件/C++ 实现代码。

## 二、明确保留的内容

- “详细状态”中的俯仰、偏航、电池、当前飞控模式和地面站↔机载状态频率显示全部保留。
- 俯仰/偏航继续从控制本就需要的本地 pose 中解算；电池继续使用低频 `BatteryState`。
- 地面站↔机载频率仍是地面站对 10 Hz `ControlStatus` 实际到达时间的最近 5 秒统计，不需要 IMU 数据。
- 原有三条 100 Hz MAVLink 消息配置请求保留；为电池显示增加的 `SYS_STATUS` 1 Hz 请求保留。本次不对实机实际频率做监听或结论。
- LAND 解除武装确认、GPS 原点回读确认和已完成的日志等级调整均未撤销。
- 没有修改用户已有的 Task 14、RViz、TODO 或参考资料改动。

## 三、验证结果

- Jazzy `colcon build --packages-select guided_interfaces onboard_control`：通过。
- 机载隔离真值测试：`1 passed`；不再发布或检查 100 Hz IMU 流。
- Python 全量回归：`74 passed`。
- ROS/C++ 包级测试：`5 tests, 0 errors, 0 failures, 0 skipped`。
- `compileall`、致命级 flake8、`bash -n` 与 `git diff --check`：通过。
- 隔离 domain 运行时图检查：订阅列表无 `/imu/data` 和 `/imu/data_raw`；接口 2.1，FCU 未连接、未武装。

## 四、实机边界

按用户要求，本次不连接实机验证 100 Hz，不向实机同步或启动本次代码。没有连接飞控、申请租约、设置消息频率或 GPS 原点，全程没有解锁或起飞。
