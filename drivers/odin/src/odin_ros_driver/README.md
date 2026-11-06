<!-- Project Odin package overview, runtime interfaces and source navigation. -->
# Odin ROS 驱动包

本包源自 Manifold Tech 的 Odin ROS Driver，在本项目中以 ROS 2 方式独立构建。
保留厂商 [Apache 2.0 许可证](LICENSE)及[版本记录](CHANGELOG.md)；项目部署、配置选择和
安装外参以[项目 Odin 指南](../../README.md)为准。

## 构建与启动

在项目根目录执行：

```bash
./drivers/odin/build.sh
./scripts/onboard/components/start_odin.sh --check
./scripts/onboard/components/start_odin.sh -c odom
```

项目使用所选飞机 `config/aircraft/<编号>/odin/` 下的配置，不再读取本包旧
`config/control_command.yaml`。`odom`、`slam`、`reloc` 分别对应 `custom_map_mode` 的
0、1、2；建图与重定位步骤见[重定位指南](RELOCALIZATION_GUIDE.md)。
Jetson OpenCV/cv_bridge 必须匹配 ABI，详见[OpenCV 配置](../../../../src/onboard_control/deploy/JETSON_OPENCV_OVERLAY.md)。

直接使用 ROS 2 launch 时，先加载 ROS 和 `drivers/odin/install/local_setup.bash`：

```bash
ros2 launch odin_ros_driver odin1_ros2.launch.py config_file:=/absolute/custom.yaml rviz:=true
```

## 核心输出

| 话题 | 用途 |
| --- | --- |
| `/odin1/odometry`、`/odin1/odometry_highfreq` | `odom←fcu` 位姿，FCU 机体系 twist；安装外参已补偿。 |
| `/odin1/odometry_imu` | 原始 IMU 参考点诊断流；仅有订阅者时发布，禁止用作飞控或 Tag 标定输入。 |
| `/odin1/cloud_slam_down` | 项目 XYZ 降采样世界点云，正式避障输入。 |
| `/odin1/navigation_status` | 实际工作模式、连接 session、地图路径及重定位证据。 |
| `/odin1/image`、`/odin1/image/compressed` | 配置开启时的设备 RGB。 |
| `/odin1/imu`、`/odin1/cloud_raw` | 配置开启时的原始传感器流；飞行配置默认关闭。 |
| `/tf` | 传感器展示 TF 由配置控制；原生 `odom←map` 独立发布。 |

原始 DTOF 点含 `x/y/z/intensity/confidence/offset_time`，SLAM/渲染云含 `x/y/z/rgb`；
项目 down 云只含 XYZ。字段布局及 QoS 以 `include/host_sdk_sample.h` 的发布实现为准。
原始 `cloud_slam` 已在项目 SDK 中关闭，不能通过选择 `original` 配置恢复。

`use_host_ros_time`：0 使用设备时间，1 使用接收时主机 ROS 时间，2 使用设备到主机时间对齐。
同一记录/显示链须采用一致时间约定；TF 历史缓存警告不能通过改写业务坐标解决。
`recorddata`、`savedata` 等诊断输出按需开启，数据不提交 Git。

## 设备相机 AE/AWB

驱动提供 `/odin1/get_ae`、`set_ae`、`get_awb`、`set_awb`，类型位于本包 `srv/`。
模式 0 为自动，1 为手动；手动 AE 的曝光单位为秒。查询完整字段及设备返回码：

```bash
ros2 interface show odin_ros_driver/srv/SetAe
ros2 service call /odin1/get_ae odin_ros_driver/srv/GetAe '{}'
ros2 service call /odin1/set_ae odin_ros_driver/srv/SetAe '{mode: 0}'
```

这些接口控制 Odin 内置相机，与独立视频/下视 UVC 相机无关。请求成功需同时检查响应与读回值，
取值范围以接口定义和目标设备固件为准。

## 排查与维护

- USB 访问失败：确认同一设备未被其他驱动占用、用户在 `plugdev` 组、项目
  `system/udev/99-odin-usb.rules` 已安装；组变更后重新登录，规则变更后重新插拔设备。
- `transfer no mem`、设备断流：检查 USB 速率、线缆、供电、内核日志及 SDK/固件兼容性；
  减少无人使用的原始云、图像和录制输出。不要把增大 USB 缓冲当作所有断流的通用修复。
- 运行时 OpenCV/符号错误：检查进程实际加载库和 cv_bridge overlay，不混用开发机安装产物。
- 高频 rosbag 丢帧：先核对发布 QoS、录制 QoS、磁盘和接收频率；可参考
  `script/rosbag2_qos.yaml`，不能只凭录制命令正常退出认定完整。

| 修改目标 | 入口 |
| --- | --- |
| 连接、配置、地图上传/下载、运行状态 | `src/host_sdk_sample.cpp`、`src/yaml_parser.cpp` |
| 点云、位姿、时间戳和 TF 发布 | `include/host_sdk_sample.h` |
| FCU 安装外参及协方差传播 | `include/fcu_odometry.h` |
| 可选图像节点与启动参数 | `launch_ROS2/odin1_ros2.launch.py` |
| SDK API/数据类型 | `include/lidar_api.h`、`include/lidar_api_type.h` |

厂商参考：[Odin1 文档](https://manifoldtechltd.github.io/wiki/Odin1/Cover.html)。
