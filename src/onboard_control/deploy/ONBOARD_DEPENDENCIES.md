<!-- Fresh-machine dependencies and incremental native build prerequisites. -->
# 全新 Ubuntu 24.04 机载机依赖安装

`setup_onboard_dependencies.sh` 用于全新 Ubuntu 24.04 amd64 或 arm64 主机。它配置带
`signed-by` 限定的 ROS 官方 apt 源，安装 ROS 2 Jazzy、MAVROS、编译工具、修正服务与视频工具，
下载 MAVROS 必需的 GeographicLib 数据集，并按 CPU 架构安装固定的 MediaMTX v1.20.0。
Odin 的公开编译依赖默认一并安装，驱动源码位于项目 `drivers/odin/src/odin_ros_driver`。

脚本不安装桌面环境，不启用或启动 systemd 服务，不访问飞控串口，也不发送模式、解锁或起飞命令。
extnav 仍为独立工作区，运行集成安装器前须按该飞机的部署配置安装。Jetson 厂商 OpenCV
需要匹配的 `cv_bridge`，参见 [OpenCV overlay 说明](JETSON_OPENCV_OVERLAY.md)。

在仓库根目录以普通用户执行：

```bash
./src/onboard_control/deploy/setup_onboard_dependencies.sh
```

## 项目 Odin 构建

默认依赖包括 OpenCV、PCL、yaml-cpp、OpenSSL、libusb 开发包和 Odin 所需 ROS 包。
飞行接口 3.5 的 Odin 运行状态解析另需 `nlohmann-json3-dev`，依赖脚本一并安装。
地面站安装、机载构建（含 `--verify`）和避障构建也会在编译前补装缺少的
`libeigen3-dev` / `nlohmann-json3-dev`；清单统一维护在 `scripts/lib/build_dependencies.bash`。
仅缺包时调用 sudo/apt，已有包不升级，不重写 ROS 软件源或启动服务。离线机器须预先安装这两个包。
`onboard_workspace.sh deps-check` 仍只读，报告缺失依赖，不自动安装。
旧命令中的 `--with-odin-deps` 仍可使用，已无需显式追加。
完整机载构建每次都会增量构建项目 Odin，首次部署不再依赖外部 `catkin_ws` 安装产物：

```bash
./src/onboard_control/deploy/build_onboard_control.sh --verify
# 只重建 Odin，不启动设备：
./drivers/odin/build.sh
```

Odin 的 `build/`、`install/`、`log/` 均位于 `drivers/odin/`，与主工作区和旧 `catkin_ws`
相互独立。启动入口固定使用项目安装产物；`ODIN_OVERLAY_SETUP` 不再用于选择 Odin。
Jetson 使用厂商 OpenCV 4.8 时，在 `onboard.env` 设置匹配的 `ODIN_CV_BRIDGE_SETUP`，
构建和启动加载同一 overlay，避免进程混用系统 OpenCV 4.6。已有兼容 overlay 可只读引用；
不在外部驱动工作区重新构建。

## 可选 Jetson 相机源码依赖

本节仅供独立联合标定工具的旧 GStreamer 相机驱动使用。当前 `correction_service` 自带 UVC
采集，使用基础安装已有的 OpenCV/V4L2，不需要 `--with-camera-deps` 或下列 NVIDIA 插件。

Wasintek GStreamer 相机源码使用 `gstreamer-1.0`、`gstreamer-app-1.0` 和 `gstreamer-video-1.0`
开发接口，并在运行时使用标准解析、V4L2 和 appsink 元件。安装其公开依赖可执行：

```bash
./src/onboard_control/deploy/setup_onboard_dependencies.sh --with-camera-deps
```

该选项安装 GStreamer 开发包、检查工具及 base/good/bad 插件，不获取相机源码，也不安装 Jetson
驱动。硬件流水线还依赖 `nvjpegdec`、`nvvidconv` 和 NVMM；它们必须来自与当前 Jetson、JetPack/
L4T 版本匹配的 NVIDIA 供应商组件，不能用普通 Ubuntu apt 包或其他架构文件替换。构建或启动相机
节点前必须在目标机验证两项均可发现，并检查输出中的插件来源：

```bash
gst-inspect-1.0 nvjpegdec
gst-inspect-1.0 nvvidconv
```

任一命令失败时应先修复对应 Jetson 多媒体组件，不能把安装普通 GStreamer 插件当作硬件解码链
已经就绪。

脚本会使用 `sudo` 完成系统安装。重复执行是安全的，MediaMTX 下载包会用仓库已记录的官方发行包
SHA-256 校验值验证。网络受限时可暂时跳过大型外部数据或 MediaMTX，待网络恢复后再次补装：

```bash
./src/onboard_control/deploy/setup_onboard_dependencies.sh --skip-geographiclib
./src/onboard_control/deploy/setup_onboard_dependencies.sh --skip-mediamtx
```

国内网络可在单次命令中指定兼容 ROS 官方签名的镜像。脚本仍会验证 ROS 官方密钥指纹，并通过
`signed-by` 把该密钥只绑定到 ROS 源：

```bash
ROS_APT_MIRROR=https://mirrors.ustc.edu.cn/ros2/ubuntu \
  ./src/onboard_control/deploy/setup_onboard_dependencies.sh
```

若 `/etc/apt/sources.list` 或其他 `.list` 已有 ROS 2 源，脚本会先备份原文件为
`.before-onboard-deps`，再只注释重复的 ROS 行，避免重复定义和 `Signed-By` 冲突。其他 apt 源不变。

需要运行仓库的额外 Python 检验时，由目标普通用户在仓库根目录创建项目环境；系统 ROS Python
包必须通过 `--system-site-packages` 对该环境可见：

```bash
python3 -m venv --system-site-packages .venv
```

依赖脚本只安装 `python3-venv`，不会代替目标用户创建或覆盖 `.venv`。

跳过项不是完整生产环境。MAVROS 缺少 GeographicLib geoid 数据时会退出；视频服务要求
`/usr/local/bin/mediamtx` 恰为 v1.20.0。

全新主机的 extnav 或飞控硬件尚未接齐时，可先完成原生构建、单测、隔离 smoke、环境文件和
systemd unit 安装：

```bash
./src/onboard_control/deploy/install_onboard_service.sh --install-only
```

`--install-only` 会执行完整的软件验证和 `daemon-reload`，但跳过硬件 `--check`，不会启用或启动
飞控 unit。extnav 和硬件路径全部安装、连接并人工核对后，再在确认的安全维护窗口执行正常模式：

```bash
./src/onboard_control/deploy/install_onboard_service.sh
```

正常模式会再次构建和验证，完成硬件检查后才 `enable --now`。两种模式都不会停止已有服务；若飞控
unit 已在运行，安装器会保持原有安全拒绝。

主项目工作区的所有构建入口统一使用普通安装，并显式设置
`AMENT_CMAKE_SYMLINK_INSTALL=OFF`。不要在同一个 `build/` 和 `install/` 上交替使用
`--symlink-install`：`ament_cmake_python` 生成目录与符号链接的布局不同，切换模式会出现
`Is a directory` 等冲突。若此前一次失败尝试已把 CMake 缓存设为 ON，重新运行上述安装器即可由
显式 OFF 覆盖；无需删除源码，也无需重装系统。extnav 是另一个独立工作区，其安装器仍可保留自己的
`--symlink-install`，不会与主项目工作区冲突。

视频和独立修正服务也支持先安装、不启动：

```bash
./video_service/deploy/install_onboard_video_service.sh --install-only
./correction_service/deploy/install_correction_service.sh --install-only
```

这两个入口仍执行各自原有的工具、依赖、源码 overlay、接口版本与配置检查，并完成构建、现场配置
保留、unit 校验和 `daemon-reload`；它们只跳过末尾的 `enable --now`。修正服务直接构建并验证本包 `correction_service/uvc_camera_node`，
不需要联合标定目录或相机 overlay。新飞机默认相机路径、镜头
标定和 by-path 设备名不能沿用旧机结论；逐项核对并完成台架验证后，才可去掉 `--install-only`
分别重跑正常安装器以启用和启动服务。
