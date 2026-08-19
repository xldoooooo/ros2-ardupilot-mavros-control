# 详细状态与真实结果确认改造简报

日期：2026-08-11

## 一、完成结论

本次只实施用户明确指定的三类修改：

1. “详细状态”增加俯仰/偏航、地面站↔机载状态频率、电池和当前飞控模式。
2. 加强原分析报告“九、日志与真实行为一致性”中证据薄弱的 LAND、消息频率和 GPS 原点终态。
3. 对“七、日志完整性与等级审查”只调整不合理的严重度；原有日志信息没有因该项审查而增加或删除。

`ControlStatus` 线级结构新增字段，接口与两个 ROS 包版本同步升级为 `2.1` / `2.1.0`。地面站 2.1 与旧机载接口不兼容，正式使用时必须同步部署地面站和机载端。

## 二、详细状态

- 机载端从已有 `/mavros/local_position/pose` 四元数解算 pitch/yaw，并随原子 `ControlStatus` 聚合发布。
- 电池从 MAVROS `/mavros/battery` 聚合，带 `battery_valid`；只有 FCU 在线、电池存在、消息不超过 5 秒且电压为有限正数时才展示，电流或百分比无效时只隐藏对应分项。
- 为保证电池源稳定到达，机载消息频率配置新增 `SYS_STATUS` 1 Hz；SITL 原始 MAVROS 电池实测为 12.60 V、0 A、100%。
- 飞控模式继续使用机载聚合的 `autopilot_mode`，GUI 不直连 MAVROS。
- 地面站按 `ControlStatus` 本地单调时钟到达时间计算最近 5 秒实际接收频率与当前消息年龄，避免把配置值当成链路实测值。
- 1500×900 无显示平台实图检查中，新增四行均完整显示，无截断或相互覆盖。

## 三、真实行为终态

### LAND

- `SetMode(mode_sent=true)` 现在只发布 `RUNNING`，文案保持“LAND 模式已发送”。
- 只有随后观察到 FCU `armed: true → false` 才发布 final `SUCCEEDED`，明确为“已解除武装，已确认降落完成”。
- 未武装时拒绝无意义 LAND；LAND 发送后 120 秒仍未解除武装则 final `FAILED`。该确认是飞控解除武装证据，不等价于独立外部 touchdown 传感器。

### 消息频率

- 四条 MessageInterval ACK 只表示请求被飞控接受，不再直接设置 `message_rates_configured=true`。
- 对位置、姿态和 IMU 三路 ROS 输入分别使用单调时钟实测；每路连续观察至少 1.5 秒、频率至少 50 Hz 且最新样本年龄不超过 0.5 秒后，才报告成功并记录三路实测速率。
- 给 EKF 本地位置初始化保留 45 秒验证窗；生产初始化等待票据同步扩展至 50 秒。真机流程先完成 GPS 原点回读，再等待频率实测，避免“本地位姿依赖原点、频率又先等待本地位姿”的环形门控。

### GPS 原点

- 发布前确认 MAVROS `set_gp_origin` 已有订阅者。
- 服务接收后先发布 `RUNNING`，再等待 MAVROS `/global_position/gp_origin`（FCU `GPS_GLOBAL_ORIGIN` 回传）。
- 只有经纬高在设定容差内匹配才 final `SUCCEEDED`；错误回读不成功，8 秒无匹配回读或中途断开 FCU 则 final `FAILED`。

MAVROS 2.14 的 global-position 插件把 `gp_origin` 设为 transient-local 回读话题，sys-status 插件把 `SYS_STATUS` / `BATTERY_STATUS` 转为 `sensor_msgs/BatteryState`：

- https://github.com/mavlink/mavros/blob/2.14.0/mavros/src/plugins/global_position.cpp
- https://github.com/mavlink/mavros/blob/2.14.0/mavros/src/plugins/sys_status.cpp

## 四、日志等级调整

保留原有状态文字和去重机制，只按影响修改等级：

- DEBUG：重复运动参考、航点中间进度、正常等待 MAVROS 参数同步。
- INFO：生命周期、控制权、正常操作接收/完成和实测门控恢复。
- WARN：参数同步超时/配置非法/读取失败、消息频率未达标、GPS 原点无回读、失联悬停及失败后的保位。
- ERROR：确定命令失败、LAND 无解除武装确认、外部切换飞控模式、failsafe LAND。

LAND、频率、原点新增的 accepted/applied/observed 状态与结果属于第三节的真实行为改造，不是为了补日志数量而新增日志。

## 五、验证记录

### 自动测试

- `colcon build --packages-select guided_interfaces onboard_control`：通过。
- Python 全量：`74 passed`。
- `colcon test` / `colcon test-result --verbose`：`5 tests, 0 errors, 0 failures, 0 skipped`。
- 新隔离状态机测试验证：频率 ACK 不提前成功；错误原点回读不成功；LAND ACK 不在解除武装前成功；新增姿态和电池字段可由机载状态读出。
- `compileall`、Python 致命 flake8 规则、`bash -n`、`git diff --check`：通过。
- 本地隔离 smoke：接口 2.1、FCU 未连接、未武装、2 秒内零姿态 setpoint。

### 未解锁 SITL

最终走生产初始化入口并通过全部门控：

- `interface=2.1`
- `armed=false`
- 飞控模式 `STABILIZE`
- 本地位置有效
- 俯仰约 `-0.09°`，偏航约 `94.72°`
- 地面站实际状态接收频率约 `9.9998 Hz`
- 电池 `12.60 V / 100%`
- `message_rates_configured=true`

首次 8 秒、随后 20 秒频率验证窗都在 EKF 尚未产出本地位置时如实失败；最终改为每路连续观察和 45 秒上限后通过，没有降低 50 Hz 成功阈值。每轮失败和最终成功后均完成本地进程清理。

### 真机 Humble/aarch64

- 真机仅用于 `/tmp/task13-detail-humble.*` 隔离源码副本的 Humble 原生 Release 构建与测试。
- `guided_interfaces`、`onboard_control` 两包构建成功，5 项 ROS/C++ 测试零失败。
- 临时目录已删除，`drone-control.service=inactive`，未发现 MAVROS、onboard、ArduCopter 或 Odin 运行进程。
- 未覆盖 `/home/onboard` 正式部署目录，未同步本次 2.1 代码，未启动真实 MAVROS/Odin/extnav，未连接或占用飞控串口。
- 全程未解锁、未起飞，也未向真机发送降落、悬停、运动、航点、消息频率或 GPS 原点命令。

## 六、改动范围

- 地面站：`config.py`、`environment.py`、`models.py`、`ros_controller.py`、`qt_ui/operations_panel.py`
- 接口：`ControlStatus.msg` 与 `guided_interfaces/package.xml`
- 机载：CMake/package/config、节点头文件与实现、部署 smoke/文档
- 测试：bootstrap、ROS 状态映射、Qt 详情和新增机载真实终态隔离测试
- 部署总览：只同步接口版本说明

用户已有的 `TODO.md`、`agent/task/13-refine-3.md`、`src/guided_sim/rviz/quadcopter.rviz`、`agent/ref/`、Task 14 文档和 `integration/` 改动均未修改或纳入本次提交。
