<!-- Standalone project driver layout, configuration selection, and build instructions. -->
# 项目 Odin 驱动

本目录独立维护完整 `odin_ros_driver`，使用项目修改后的 Host SDK，保留 XYZ down 点云输出。
不依赖地面站、飞控或避障包，不修改项目外的 `catkin_ws`。

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

| 配置文件（`src/odin_ros_driver/config/`） | 用途 |
| --- | --- |
| `original.yaml` | 厂商原始配置，原样保留其开关；当前原版默认是 SLAM 模式。 |
| `odom.yaml` | 项目里程计模式，`custom_map_mode: 0`。 |
| `slam.yaml` | 项目建图模式，`custom_map_mode: 1`。 |
| `reloc.yaml` | 项目重定位模式，`custom_map_mode: 2`；地图绝对路径由使用者手动维护。 |

`control_command.yaml` 仅是指向 `odom.yaml` 的兼容链接，不另维护第五份配置。
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
