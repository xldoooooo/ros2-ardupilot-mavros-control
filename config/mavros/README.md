<!-- Repository-owned aircraft MAVROS plugin selection and launch usage. -->
# MAVROS 机载配置

`aircraft_plugins.yaml` 是当前机载飞行链路的 9 插件白名单；
`aircraft.launch.xml` 使用它，并读取所选 `config/aircraft/<编号>/mavros/apm_config.yaml`，
 `start_mavros.sh` 和`start_onboard_control.sh` 均自动使用此配置，原有 MAVROS overlay 选择保持有效。
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

以下按 refresh 当前 MAVROS 2.16 的插件注册清单列出全部 55 个未启用插件，
包括此前手动禁用的插件。白名单之外的插件均不加载；禁用理由针对当前项目的实机飞行链路。

| 禁用插件 | 原本用途 | 本项目禁用理由 |
| --- | --- | --- |
| `actuator_control` | 直接发送执行器控制量 | 项目发送姿态/推力目标，由 FCU 完成电机混控，不直接控制执行器。 |
| `adsb` | 桥接 ADS-B 空中交通信息 | 当前飞行链路不收发这类交通信息。 |
| `altitude` | 发布独立的高度遥测 | 当前使用 `local_position` 的位置高度，不消费此插件的高度话题。 |
| `cam_imu_sync` | 发布 FCU 触发相机的精确时间戳 | 当前相机链路不使用 FCU 硬件触发时间戳。 |
| `camera` | MAVLink 相机协议接口 | 视频及 Tag 图像由独立相机链路处理，不经过 MAVLink 相机协议。 |
| `cellular_status` | 向 MAVLink 组件转发蜂窝网络状态 | 项目没有通过此接口上报蜂窝网络状态。 |
| `companion_process_status` | 向 FCU 上报伴随计算机进程状态 | 当前控制与健康检查链路不使用此状态接口；FCU 心跳仍由 `sys_status` 处理。 |
| `debug_value` | 桥接 MAVLink 调试数值 | 属于调试接口，实际飞行功能没有消费者。 |
| `distance_sensor` | 在 ROS 与 FCU 间桥接测距传感器数据 | 当前不通过此接口注入或读取测距数据；机载感知使用 Odin 点云。 |
| `esc_status` | 发布电调状态 | 当前飞行控制和界面没有消费者，电池状态由 `sys_status` 提供。 |
| `esc_telemetry` | 发布 ArduPilot 电调遥测 | 当前不消费单个电调的遥测数据。 |
| `fake_gps` | 将本地定位转换为模拟 GPS 输入 | 外部定位通过 `vision_pose` 注入，不伪装成 GPS。 |
| `ftp` | MAVLink 飞控文件传输 | 飞行链路不通过 MAVLink 上传或下载 FCU 文件；与项目的网络文件传输无关。 |
| `geofence` | 上传、下载 FCU 围栏 | 当前项目不经 MAVROS 管理 FCU 围栏。 |
| `gimbal_control` | MAVLink 云台协议 v2 | 当前相机链路没有使用此云台控制接口。 |
| `gps_input` | 发送外部 `GPS_INPUT` 数据 | 外部定位使用 `vision_pose`，没有外接 GPS 数据注入需求。 |
| `gps_rtk` | 转发 RTCM 差分数据及 RTK 基线 | 当前项目没有通过此接口传输 RTK 差分或读取基线数据。 |
| `gps_status` | 发布 GPS 原始数据及详细状态 | 当前没有这些话题的消费者；GPS/EKF 原点和全局位置由 `global_position` 保留。 |
| `guided_target` | 桥接 FCU 的引导目标 | 当前目标由项目导航链路管理，不使用此插件的目标接口。 |
| `hil` | 硬件在环仿真数据接口 | 实机飞行不使用 HIL；开发机仿真继续使用自己的配置。 |
| `home_position` | 读取、设置及发布 Home 点 | 当前项目不使用 Home 话题或设置接口；定位需要的 EKF 原点由 `global_position` 提供。 |
| `landing_target` | 向 FCU 提供精准降落目标 | 当前 Tag 用于定位修正，不通过 `LANDING_TARGET` 协议执行精准降落。 |
| `log_transfer` | 下载 FCU 飞行日志 | 日志下载不属于当前机载实时飞行链路，可在维护时另行启用。 |
| `mag_calibration_status` | 发布磁罗盘校准进度和结果 | 属于维护、校准功能，实际飞行没有消费者。 |
| `manual_control` | 通过 MAVLink 发送或接收摇杆控制 | 物理遥控器直接连接 FCU，不通过此软件摇杆接口。 |
| `mocap_pose_estimate` | 向 FCU 注入动捕位姿 | 当前定位源不是此动捕接口，统一使用 `vision_pose`。 |
| `mount_control` | 旧版 MAVLink 云台、挂载控制 | 项目不使用此挂载控制接口。 |
| `nav_controller_output` | 发布 FCU 导航控制器状态 | 当前导航与控制链路不消费该遥测话题。 |
| `obstacle_distance` | 将障碍物距离数组发送给 FCU | 避障在机载规划器内完成，不向 FCU 注入此类避障数据。 |
| `obstacle_distance_3d` | 将三维障碍物位置发送给 FCU | 当前规划器直接使用 Odin 点云，不经此接口向 FCU 传输障碍物。 |
| `odometry` | 双向桥接 MAVLink `ODOMETRY` | 外部定位输入走 `vision_pose`，FCU 本地里程计输出走 `local_position/odom`，均不依赖此插件。 |
| `onboard_computer_status` | 向 FCU 上报机载计算机资源状态 | 当前项目没有此上报接口的发布者或依赖。 |
| `open_drone_id` | 向 FCU 发送 Open Drone ID 数据 | 当前项目没有通过 MAVROS 提供此协议的数据。 |
| `optical_flow` | 将光流数据发布到 ROS | 当前定位和控制不消费 MAVROS 光流话题。 |
| `play_tune` | 请求 FCU 播放提示音 | 当前飞行功能不调用该服务。 |
| `px4flow` | 将 PX4Flow 传感器数据发布到 ROS | 当前项目没有使用此传感器的 ROS 接口。 |
| `rallypoint` | 上传、下载 FCU 集结点 | 当前项目不经 MAVROS 管理集结点。 |
| `rangefinder` | 发布 ArduPilot 测距遥测 | 当前没有对应 ROS 消费者，不影响 FCU 自身使用测距传感器。 |
| `rc_io` | 发布 RC 通道及接收 RC override | 项目不读取 RC 话题，也不发送 RC override；手动操纵由接收机与 FCU 直接完成。 |
| `setpoint_accel` | 发送加速度或力目标 | 当前控制器输出姿态/推力，经 `setpoint_raw` 发送，不使用该入口。 |
| `setpoint_attitude` | 通过独立话题发送姿态和推力 | 当前使用 `setpoint_raw` 的姿态接口，不使用这套额外的姿态话题。 |
| `setpoint_position` | 发送位置目标 | 当前位置控制在项目控制器内完成，不向此接口发送位置目标；同时省去其位姿缓存订阅。 |
| `setpoint_trajectory` | 定时采样轨迹并发送目标 | 当前轨迹由项目控制器执行，不使用此接口；禁用后也省去其空闲定时器。 |
| `setpoint_velocity` | 发送速度目标 | 当前速度控制在项目控制器内完成，不使用此接口。 |
| `sim_state` | 发布仿真真实状态 | 实机没有该仿真数据用途；开发机 SITL 配置独立。 |
| `tdr_radio` | 发布 3DR 数传电台状态 | 当前项目不消费该电台状态接口。 |
| `terrain` | 在 FCU 与地形服务间桥接地形高程 | 当前使用本地坐标与机载地图，没有此 SRTM 地形服务链路。 |
| `trajectory` | 接收 FCU 规划轨迹并回传修正轨迹 | 当前规划、避障和轨迹执行在项目内部完成，不走此 MAVLink 轨迹协议。 |
| `tunnel` | 传输自定义 MAVLink 隧道载荷 | 当前没有使用该隧道协议的功能。 |
| `vfr_hud` | 发布空速、航向、高度、油门等 HUD 遥测 | 当前没有该话题的消费者；所需位置、速度及系统状态由保留插件提供。 |
| `vibration` | 发布振动水平及加速度计削顶统计 | 用于诊断，当前飞行控制链路没有消费者。 |
| `vision_speed` | 单独注入视觉速度估计 | 当前外部定位通过 `vision_pose` 注入，没有独立视觉速度输入。 |
| `waypoint` | 上传、下载及执行 FCU 航点任务 | 当前航点、规划和执行由项目导航链路管理，不通过 MAVROS 上传 FCU 任务。 |
| `wheel_odometry` | 根据轮速或轮距计算里程计 | 飞机没有使用轮式编码器定位。 |
| `wind_estimation` | 发布 FCU 风速、风向估计 | 当前控制器和界面不消费此风场估计话题。 |

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

串口示例需替换为实际设备；正常机载启动建议使用上述脚本读取逐机 `onboard.env`。
