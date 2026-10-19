<!-- Repository-owned aircraft MAVROS plugin selection and launch usage. -->
# MAVROS 机载配置

`aircraft_plugins.yaml` 是当前机载飞行链路的 9 插件白名单；
`aircraft.launch.xml` 使用它，并复用当前已选 MAVROS 安装中的 `apm_config.yaml`，
不修改 `/opt/ros`，不复制系统参数，也不需要重编 MAVROS 或项目。
独立 `scripts/onboard/components/start_mavros.sh` 和整栈
`scripts/onboard/start_onboard_control.sh` 均自动使用此配置，原有 MAVROS overlay 选择保持有效。
launch 的 `namespace` 默认为空，保留机载 MAVROS 内部的 `/mavros/...` 接口命名；
不要再额外传入 `namespace:=mavros`，避免与内部 UAS 命名重复。

| 插件 | 保留原因 |
| --- | --- |
| `command` | FCU 指令、消息频率配置 |
| `global_position` | GPS/EKF 原点及全局位置 |
| `imu` | 姿态遥测与 UAS 内部姿态/坐标变换依赖 |
| `local_position` | FCU 最终本地位置、速度 |
| `param` | 飞控参数读取及校验 |
| `setpoint_raw` | 当前生产姿态/推力控制输出 |
| `sys_status` | FCU 连接、模式、电池及扩展系统状态 |
| `sys_time` | 时间同步 |
| `vision_pose` | 外部定位送入 FCU |

未启用的原冗余插件为 `manual_control`、`rc_io`、`setpoint_accel`、
`setpoint_attitude`、`setpoint_position`、`setpoint_velocity`、`setpoint_trajectory`、
`gps_status`、`guided_target`、`odometry`、`sim_state`、`home_position`。
GPS 状态插件的正确标识是 `gps_status`；旧拼写 `gpsstatus` 不能匹配它。
物理遥控器直接连接 FCU 的手动控制由 FCU 处理，不依赖 MAVROS 的 `rc_io` 或
`manual_control`；经 MAVROS 的 RC override、RC 话题及 MANUAL_CONTROL 接口不在本机载配置内。
此配置只选择 ROS 插件，不修改 FCU 的遥控器、通道或飞行模式参数。

开发机 SITL/地面站仍使用原来的系统 MAVROS 配置；未改其启动路径。
需要完整调试插件时，可使用原命令 `ros2 launch mavros apm.launch ...`，
或者启动本文件并用 `pluginlists_yaml:=/absolute/debug_plugins.yaml` 显式覆盖白名单。
仅修改 YAML 后重启相应机载进程即可生效。

在已加载 ROS/MAVROS overlay 的终端，可独立启动此配置：

```bash
ros2 launch "$(pwd)/config/mavros/aircraft.launch.xml" fcu_url:=/dev/serial/by-id/FCU:460800
```

串口示例需替换为实际设备；正常机载启动建议使用上述脚本读取逐机 `onboard.env`。
飞行验收仍由用户手动解锁/起飞完成。
