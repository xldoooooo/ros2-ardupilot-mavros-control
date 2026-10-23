<!-- Quick index of repository-owned runtime parameters. -->
| 文件 | 用途 |
| --- | --- |
| `aircraft.local.env` | 本机选择固定编号：`uav001` 为1号机、`uav002` 为2号机、`simulation` 为仿真，不提交 Git。 |
| `host.local.env` | 本机依赖路径、构建环境、地面实机 ROS domain 和默认 GPS 原点；参照 `host.local.env.example`，不提交 Git。 |
| `aircraft/<编号>/aircraft.yaml` | 飞机显示名称，改名不改变固定编号。 |
| `aircraft/<编号>/onboard.env` | 飞控串口、波特率、ROS 通信及依赖路径。 |
| `aircraft/<编号>/navigation.env` | 保存 odom/map 导航模式。 |
| `aircraft/<编号>/control.yaml` | 机载控制器参数。 |
| `aircraft/<编号>/odin_extrinsics.yaml` | Odin 安装姿态、杆臂和外部定位桥参数。 |
| `aircraft/<编号>/odin/{odom,slam,reloc,original}.yaml` | Odin 各工作模式、点云处理及地图路径。 |
| `aircraft/<编号>/odin/calibration/calib.yaml` | 对应 Odin 设备的原厂标定。 |
| `aircraft/<编号>/mavros/apm_config.yaml` | 对应 MAVROS 版本的 ArduPilot 插件参数。 |
| `mavros/aircraft_plugins.yaml` | 机载 MAVROS 插件名单。 |
| `mavros/aircraft.launch.xml` | 机载 MAVROS 启动配置。 |
| `aircraft/<编号>/avoidance/planner.yaml` | 避障规划器、坐标桥及输入话题参数。 |
| `aircraft/<编号>/video/camera.conf` | Wasintek 视频采集、推流和存储参数。 |
| `aircraft/<编号>/video/lens.conf` | Wasintek 镜头控制参数。 |
| `video/Wasintek/intrinsics.yaml` | 当前 Wasintek 镜头内参。 |
| `video/Wasintek/intrinsics.yaml.old` | 旧 Wasintek 镜头内参，仅作存档。 |
| `aircraft/<编号>/correction/general_settings.yaml` | AprilTag 修正算法参数。 |
| `aircraft/<编号>/correction/camera.conf`、`lens.conf` | UQ212 下视相机采集及镜头控制参数。 |
| `aircraft/<编号>/correction/intrinsics.yaml`、`extrinsics.yaml` | UQ212 相机标定。 |
| `aircraft/<编号>/correction/tag_pose.csv` | Tag 尺寸和地图位置。 |
| `aircraft/<编号>/correction.env` | 独立修正服务的 ROS 通信配置。 |
| `ground/video.json` | 本机地面视频工具保存的偏好，不提交 Git。 |
