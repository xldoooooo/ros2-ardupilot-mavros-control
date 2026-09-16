<!-- 本文记录 Jetson 上为 Odin 隔离 OpenCV ABI 的可复现 cv_bridge overlay 构建步骤。 -->

# Jetson Odin OpenCV 4.8 与 cv_bridge 隔离构建

适用环境为 Jetson Linux 39.2.1、ROS 2 Jazzy，以及系统厂商 OpenCV 4.8。系统安装的
`ros-jazzy-cv-bridge 4.1.0` 链接 Ubuntu OpenCV 4.6；Odin 同时链接厂商 OpenCV 4.8，导致一个
进程加载 `.406` 和 `.408` 两套库。

不要替换或降级 Jetson 的系统 OpenCV。先在 Odin 的独立 overlay 中编译同版本 `cv_bridge`：

```bash
cd /home/nvidia/catkin_ws/src
git clone --depth 1 --branch 4.1.0 \
  https://github.com/ros-perception/vision_opencv.git

cd /home/nvidia/catkin_ws
source /opt/ros/jazzy/setup.bash
export CMAKE_BUILD_PARALLEL_LEVEL=2
colcon build --symlink-install --packages-select cv_bridge
```

已验证官方标签 `4.1.0` 对应提交：

```text
f5b738d9694f0cee5904440d03912fc249943f8a
```

重建 Odin 前必须 source 这层 overlay。Odin 的 CMake ROS 版本检测正则没有包含 `jazzy`，清空
CMake 缓存后会错误进入 ROS 1 分支，因此必须显式传入厂家构建脚本使用的开关：

```bash
cd /home/nvidia/catkin_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export CMAKE_BUILD_PARALLEL_LEVEL=2
colcon build --symlink-install \
  --packages-select odin_ros_driver \
  --cmake-args -DBUILD_SYSTEM=ROS2
```

若由旧的非 symlink 构建切换到 `--symlink-install` 时出现
`existing path cannot be removed: Is a directory`，只清理 Odin 包自己的构建目录后重试：

```bash
rm -rf /home/nvidia/catkin_ws/build/odin_ros_driver
```

验证命令：

```bash
source /opt/ros/jazzy/setup.bash
source /home/nvidia/catkin_ws/install/setup.bash

ros2 pkg prefix cv_bridge
grep -E '^(OpenCV_DIR|cv_bridge_DIR):' \
  /home/nvidia/catkin_ws/build/odin_ros_driver/CMakeCache.txt
ldd /home/nvidia/catkin_ws/install/odin_ros_driver/lib/odin_ros_driver/host_sdk_sample \
  | grep -E 'cv_bridge|opencv|not found'
```

验收标准：`libcv_bridge.so` 来自 `/home/nvidia/catkin_ws/install/cv_bridge`，OpenCV 库只包含
`.408`，且没有 `not found`。验证过程中不要运行 `host_sdk_sample`；没有连接外设时，这只证明
链接关系正确，不代表 Odin 硬件已经就绪。
