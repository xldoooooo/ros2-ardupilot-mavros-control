<!-- Quick index of repository-owned runtime parameters. -->
| 文件 | 用途 |
| --- | --- |
| `aircraft.local.env` | 本机选择固定编号：`uav001` 为1号机、`uav002` 为2号机、`simulation` 为仿真，不提交 Git。 |
| `host.local.env` | 本机依赖路径、构建环境、地面实机 ROS domain 和默认 GPS 原点；参照 `host.local.env.example`，不提交 Git。 |
| `aircraft/<编号>/aircraft.yaml` | 飞机显示名称，改名不改变固定编号。 |
| `aircraft/<编号>/onboard.env` | 飞控串口、波特率、ROS 通信及依赖路径。 |
| `aircraft/<编号>/navigation.env` | 保存 odom/map 导航模式。 |
| `aircraft/<编号>/control.yaml` | 机载控制器参数。 |
| `aircraft/<编号>/odin_extrinsics.yaml` | Host 固定 pitch/FLU 杆臂，及 extnav 局部原点/发布参数。 |
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
| `aircraft/<编号>/correction/intrinsics.yaml`、`extrinsics.yaml` | UQ212 内参及相机光学系→FCU FLU 的完整 T_fcu_camera。 |
| `aircraft/<编号>/correction/tag_pose.csv` | Tag 尺寸和地图位置。 |
| `aircraft/<编号>/correction.env` | 独立修正服务的 ROS 通信配置。 |
| `ground/video.json` | 本机地面视频工具保存的偏好，不提交 Git。 |
| `ground/media.json` | 地面存储、直播编码、公网 RTSP 转推和远程 FTP；参照 `media.example.conf`，凭据只放此类本机配置。 |
| `ground/simulation_media.json` | 仿真样例目录和直播帧率；参照同名 example，原文件与独立轮播不受直播编码影响。 |

上位机 WebSocket 地址和编号保存在 `~/.config/ros2-ardupilot-ground-station/upstream.json`，
也可用 `UPSTREAM_CONFIG_PATH` 指定文件。现场地址和凭据不提交 Git。
配置中的 `_local_configuration` 仅保留原值供人工恢复，运行时不读取；面板保存 WebSocket 地址时保留此内容。

控制超时在所选 `aircraft/<编号>/control.yaml` 的 `ros__parameters` 下配置，例如
`pose_timeout_seconds` 同时用于 FCU 位姿/速度和自动校准 raw/extnav 数据检查。
省略 ROS 参数时采用代码缺省值；编辑后重启对应服务生效，不能把 `0` 当作通用禁用开关。
