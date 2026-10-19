<!-- Keep vendor OpenCV and cv_bridge ABI consistent for project Odin. -->
# Jetson Odin OpenCV 4.8 与 cv_bridge

Jetson 厂商 OpenCV 4.8 与 Ubuntu 的 OpenCV 4.6 不能在 Odin 进程混用。系统
`ros-jazzy-cv-bridge 4.1.0` 链接 4.6；厂商 OpenCV 主机必须准备同版本源码构建的 cv_bridge。
不要替换或降级 Jetson 系统 OpenCV，也不要在旧 `catkin_ws` 中重新构建。

已有兼容 overlay 可直接只读引用。新建时，在项目根目录执行：

```bash
mkdir -p drivers/odin/cv_bridge_ws/src
git clone --depth 1 --branch 4.1.0 \
  https://github.com/ros-perception/vision_opencv.git \
  drivers/odin/cv_bridge_ws/src/vision_opencv
source /opt/ros/jazzy/setup.bash
(cd drivers/odin/cv_bridge_ws && \
  CMAKE_BUILD_PARALLEL_LEVEL=2 colcon build --packages-select cv_bridge \
    --cmake-args -DCMAKE_BUILD_TYPE=Release -DAMENT_CMAKE_SYMLINK_INSTALL=OFF)
```

标签 `4.1.0` 的已验证提交为 `f5b738d9694f0cee5904440d03912fc249943f8a`。
CMake 应选择厂商 OpenCV 4.8；非默认安装位置可显式传入 `-DOpenCV_DIR=<vendor-config-dir>`。
在 `/etc/ros2-ardupilot/onboard.env` 配置真实路径：

```bash
ODIN_CV_BRIDGE_SETUP=/home/nvidia/ros2-ardupilot-mavros-control/drivers/odin/cv_bridge_ws/install/local_setup.bash
```

首次尚无该文件时，可在当前终端 `export ODIN_CV_BRIDGE_SETUP=...`。
`./drivers/odin/build.sh` 与 Odin 启动入口加载该 cv_bridge 后，使用项目驱动安装目录。
它们不构建或修改外部 overlay。完整机载构建也会调用项目 Odin 构建器。

构建后在项目根目录只读验证：

```bash
grep -E '^(OpenCV_DIR|cv_bridge_DIR):' drivers/odin/build/odin_ros_driver/CMakeCache.txt
# Use the same cv_bridge overlay selected for Odin startup.
source drivers/odin/cv_bridge_ws/install/local_setup.bash
ldd drivers/odin/install/odin_ros_driver/lib/odin_ros_driver/host_sdk_sample \
  | grep -E 'cv_bridge|opencv|not found'
```

`libcv_bridge.so` 应来自已指定的 overlay；OpenCV 仅包含 `.408`，且无 `not found`。
系统 OpenCV 4.6 主机可使用系统 cv_bridge，不必配置此变量。链接检查不启动 Odin，
也不能代替设备数据与定位验收。
