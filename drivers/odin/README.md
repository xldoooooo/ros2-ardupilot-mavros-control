<!-- Standalone project driver layout, configuration selection, and build instructions. -->
# 项目 Odin 驱动

本目录独立维护完整 `odin_ros_driver`，使用项目修改后的 Host SDK，保留 XYZ down 点云输出。
不依赖地面站、飞控或避障包，不修改项目外的 `catkin_ws`。

## 飞控中心输出

`/odin1/odometry` 和 `/odin1/odometry_highfreq` 的 pose 为 `odom←fcu`，
参考点为飞控中心、机体轴为前/左/上；标准 Odometry 的线速度和角速度均在 `fcu` 表达。
Host 发布层只右乘固定 `I←F`，不学习初始姿态、不移动或旋转 odom/map 世界系。
物理 IMU/雷达/设备相机仍保留自身坐标，TF 链为 `odom←fcu←imu←lidar`；
SLAM 点云、设备内部算法、WIWC 外参载荷及 `odom←map` 不作安装转换。

唯一 Odin 安装配置为 `config/aircraft/<编号>/odin_extrinsics.yaml` 的 `/**/ros__parameters`：
`pitch_cam` 定义 `R_FI=Ry(pitch_cam)`（rad，上仰25°为 `-0.4363323129985824`），
`odin_x/y/z` 是飞控到 IMU 的向量（水平 FLU、m）。只补固定 pitch，roll/yaw 为零，自动学习禁用。
驱动直接读取该文件，包括直接启动二进制的场景；修改后重启驱动和 extnav。
refresh 本次值为 `(.100,-.029,.060)m`，因此右乘的平移是 `-R_IF*odin_xyz`，不能直接取反后忽略安装角。

旧 SDK 的线速度在 odom、角速度在 IMU；发布时包含旋转杆臂速度并转为 FCU body twist。
6×6 协方差按一阶雅可比传播全部交叉块，采用 odom 平移/世界小角 pose 误差与旧混合 twist 约定，
在当前姿态条件下计算；未知哨兵保留。厂商未公开协方差扰动定义，不能据此承诺统计标定准确性。
二进制传感器诊断日志仍为设备原始 IMU 位姿；业务里程计和路径为 FCU。
设备相机重投影单独订阅 `/odin1/odometry_imu`（标准低频原始 IMU 诊断流，仅有读者时发布），
它沿用 SDK 的混合速度约定，只供传感器几何/诊断，禁止用作飞控或 Tag 标定输入。

## 构建

在项目根目录执行 `./drivers/odin/build.sh`。源码、构建和安装分别位于本目录的
`src/`、`build/` 和 `install/`。项目完整机载构建也会先增量构建此驱动。
Jetson 的厂商 OpenCV 必须配套相同 ABI 的 cv_bridge，可用 `ODIN_CV_BRIDGE_SETUP`
指定其 setup 文件，详见[构建说明](../../src/onboard_control/deploy/JETSON_OPENCV_OVERLAY.md)。

## 启动与配置

在项目根目录运行：

```bash
./scripts/onboard/components/start_odin.sh             # 默认 odom
./scripts/onboard/components/start_odin.sh -c slam
./scripts/onboard/components/start_odin.sh -c reloc
./scripts/onboard/components/start_odin.sh -c original
./scripts/onboard/components/start_odin.sh -c /absolute/custom.yaml
```

完整机载入口 `./scripts/onboard/start_onboard_control.sh` 使用相同的 `-c` 选项，默认也为 odom。
`--check` 只检查配置路径及安装入口，不连接设备。切换配置前先停止现有 Odin 进程。

| 配置文件（项目根 `config/aircraft/<编号>/odin/`） | 用途 |
| --- | --- |
| `original.yaml` | 厂商原始配置，原样保留其开关；当前原版默认是 SLAM 模式。 |
| `odom.yaml` | 项目里程计模式，`custom_map_mode: 0`。 |
| `slam.yaml` | 项目建图模式，`custom_map_mode: 1`。 |
| `reloc.yaml` | 项目重定位模式，`custom_map_mode: 2`；地图绝对路径由使用者手动维护。 |

项目启动入口直接读取所选飞机的模式 YAML；修改后重启 Odin，无需重新构建。
项目 `odom`/`reloc` 配置关闭无人使用的原始 DTOF、诊断 CSV 和传感器展示 TF；
高频定位、下采样点云与独立的 `odom→map` 重定位 TF 保持。`slam` 关闭原始 DTOF/CSV，
保留传感器 TF 方便建图查看；开发机需要完整传感器调试时显式选择 `original` 或自定义配置。
裁剪前配置分别保存在各设备的 `src/odin_ros_driver/config/backup/20261019-flight-trim/`，
恢复时保留本机地图路径和标定，勿跨机覆盖。
`original.yaml` 只代表原始配置，不会恢复被项目 SDK 关闭的原始 cloud_slam 发布；其缺少
`cloud_downsample` 配置时也不会输出项目 down 云。正式避障使用项目配置。

地图文件放在 `maps/`，可存放 `.map`、`.bin` 等实际设备支持的格式；重定位须选厂商支持的地图，
不能仅通过改后缀转换格式。驱动默认建图输出也在此目录，保存命令沿用驱动的 `set_param.sh save_map 1`。
重定位配置保留 `/CHANGE_ME/…` 占位路径，须先手动填写；启动器不会自动猜测或替换地图。
地图、录制数据、运行标定和构建输出均不进入 Git。

## 原生 ROS 2 启动

```bash
source /opt/ros/jazzy/setup.bash
# Jetson 按需先 source 匹配的 cv_bridge。
source drivers/odin/install/local_setup.bash
ros2 launch odin_ros_driver odin.launch.py
ros2 launch odin_ros_driver odin.launch.py config_file:=/absolute/custom.yaml
ros2 launch odin_ros_driver odin1_ros2.launch.py rviz:=true
```

两个 launch 名称等价，默认 odom、不打开 RViz，辅助图像节点按所选 YAML 开关启动。
仅 source 原有 `catkin_ws` 的干净终端仍使用原驱动；项目启动脚本只在自己的子进程环境中选择项目驱动。

## 来源与修改位置

完整基线来自 refresh 现用驱动，保留厂商 [Apache 2.0 许可证](src/odin_ros_driver/LICENSE)及上游说明。
`original.yaml` 来自[厂商提交 f51051f](https://github.com/manifoldsdk/odin_ros_driver/tree/f51051f2d861f7643d4d33d2ade2952efe1a4672)，
其余项目配置沿用已有修改版参数。所用厂商 AMD64/ARM64 静态 SDK 库是构建必需输入，不是本地构建产物。

- 修改配置读取及输出目录：`src/odin_ros_driver/src/host_sdk_sample.cpp`。
- 修改点云、里程计发布：`src/odin_ros_driver/include/host_sdk_sample.h`。
- 修改启动及辅助节点选择：`src/odin_ros_driver/launch_ROS2/odin1_ros2.launch.py`。
- 逐机地图路径和标定不得通过同步另一架飞机的配置覆盖。
