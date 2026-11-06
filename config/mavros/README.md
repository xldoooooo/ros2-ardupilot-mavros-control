<!-- Repository-owned aircraft MAVROS plugin selection and launch usage. -->
# MAVROS 机载配置

`aircraft_plugins.yaml` 是当前机载飞行链路的 9 插件白名单；
`aircraft.launch.xml` 使用它，并读取所选 `config/aircraft/<编号>/mavros/apm_config.yaml`，
 `start_mavros.sh` 和`start_onboard_control.sh` 均自动使用此配置，原有 MAVROS overlay 选择保持有效。
launch 沿用所装 MAVROS `node.launch` 的默认命名空间：2.14为 `mavros`，2.16为根；
不要统一强制为空或重复添加命名空间，运行时接口应仍在 `/mavros/...`。

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

白名单外的插件不加载，包括其他 setpoint、MAVLink任务上传、RC override、测距/光流、
日志下载与相机/云台接口；这些功能当前分别由项目控制器、物理接收机或独立服务处理。
需要维护/诊断接口时，先确认实际消费者，再显式扩展白名单，而不是根据插件数量推断算力收益。

GPS 状态插件的正确标识是 `gps_status`；旧拼写 `gpsstatus` 不能匹配它。
物理遥控器直接连接 FCU 的手动控制由 FCU 处理，不依赖 MAVROS 的 `rc_io` 或
`manual_control`；经 MAVROS 的 RC override、RC 话题及 MANUAL_CONTROL 接口不在本机载配置内。
此配置只选择 ROS 插件，不修改 FCU 的遥控器、通道或飞行模式参数，
也不关闭 FCU 自身的传感器融合、Home/返航、围栏等功能。
禁用可省去插件实例、接口及相应回调，但各插件的实际开销不同，不能将表格行数当作算力收益。

开发机 SITL/地面站固定读取 `config/aircraft/simulation/mavros/apm_config.yaml`；
需要完整调试插件时，可使用原命令 `ros2 launch mavros apm.launch ...`，
或者启动本文件并用 `pluginlists_yaml:=/absolute/debug_plugins.yaml` 显式覆盖白名单。
仅修改 YAML 后重启相应机载进程即可生效。

在已加载 ROS/MAVROS overlay 的终端，可独立启动此配置：

```bash
bash scripts/onboard/components/start_mavros.sh
```

入口自动读取逐机 `onboard.env`，串口必须按实际接线确认。
