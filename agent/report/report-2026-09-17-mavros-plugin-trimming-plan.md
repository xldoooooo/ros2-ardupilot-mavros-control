# MAVROS 插件裁剪评估（仅方案，未实施）

本报告记录 new 飞机当前插件配置、候选裁剪范围及验收要求；不修改任何插件配置。

## 现场基线

new 的 MAVROS 2.14.0 由 apm.launch 启动，实际读取
`/opt/ros/jazzy/share/mavros/launch/apm_pluginlists.yaml` 与同目录 `apm_config.yaml`。
之前对 waypoint、camera、distance_sensor、geofence、obstacle_distance 等插件的禁用仍在生效。
实际初始化约 32 个插件，大部分位于同一个 mavros_node 进程。

发现 denylist 中 `vision_speed_estimate` 与实际插件名 `vision_speed` 不匹配，后者仍初始化。
开发机系统配置与 new 不同；更关键的是 ground_station_core/environment.py 的 SITL 入口
只加载 apm_config.yaml，没有加载插件列表，故目前不能用实机裁剪情况推断 SITL。

## 建议分批范围

| 批次 | 插件 | 前提与影响 |
| --- | --- | --- |
| 第一批 | setpoint_accel, setpoint_attitude, setpoint_position, setpoint_velocity, setpoint_trajectory, trajectory | 当前机载输出使用 setpoint_raw；确认没有外部控制工具使用这些输入 |
| 第二批 | fake_gps, gps_input, gps_rtk, landing_target, odometry, vision_speed, sim_state, guided_target, open_drone_id | 当前主链路不使用；逐项检查外部订阅/发布者及诊断需求 |
| 保守保留 | command, sys_status, sys_time, param, local_position, global_position, imu, setpoint_raw, vision_pose, home_position | 控制、参数、状态、同步、原点与外部定位通路；imu 存在共享姿态数据用途 |
| 暂不裁剪 | esc_status, esc_telemetry, gps_status, log_transfer, nav_controller_output, rc_io, manual_control | 可能用于诊断、日志、遥控状态观察，收益尚未测量 |

MAVROS odometry 插件不是 Odin 驱动；当前外部定位经 vision_pose 送入飞控。
删除空闲插件不保证减少串口流量，也不会减少 ArduPilot 固件参数数量。

## 后续实施方法（本次不执行）

将项目专用 YAML 纳入版本管理，由 SITL 与实机共用或明确分层，避免修改 /opt 被包升级覆盖。
现有 apm.launch 内部固定传入系统插件文件，不能假设传一个未声明参数就能覆盖；需要使用
node.launch 的 pluginlists_yaml/config_yaml 接口，或增加项目 launch 包装并核对原参数。
先实施第一批，记录实际插件清单，再评估第二批。维持原 READY、参数校验与控制门。

验收：SITL 完整初始化、参数与位姿就绪、时间同步、原点通路、raw attitude 输出接口存在，
再在未解锁实机检查同样接口和启动单调时间。实机不由代理执行解锁或起飞。
尚未实施、没有裁剪后性能数据，不能预先声称节省具体秒数。

## 参数优先读取独立评估

优先读取只发送 GUID_OPTIONS 与 MOT_THST_HOVER 的 PARAM_REQUEST_READ，不写参数。
复用现有 MAVROS 路由、正常参数事件校验并保留整表回退，风险评估为中等工程风险，
不是高风险飞行动作。主要风险是路由/目标/编码错误、重连旧状态和无界重试。
先使用独立验证程序，只允许两个名称、确认未武装、有限重试并在状态过期时停止发送；
不将验证程序安装进正式服务。部署启动脚本前仍备份 new 的原文件以便回退。
生产化前还需覆盖断线、缺失/错误参数及回退测试；本轮实测结果另见任务完成报告。
