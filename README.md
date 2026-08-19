# 地面站

## 新机部署完整地面站

1. 安装 ROS、MAVROS、项目源码，并按
   [地面站视频依赖](video_service/README.md#地面站视频依赖ubuntu-2404--amd64)
   安装 FFmpeg、v4l-utils 与 amd64 MediaMTX；
2. 在完整项目检出根目录运行：

   ```bash
   ./setup_ground_station.sh
   ```

这是地面站唯一的项目安装入口：它创建 Python 环境，安装 GUI 依赖，构建地面仿真与共享 ROS
接口并检查运行环境。地面站不执行
`src/onboard_control/deploy/install_onboard_service.sh` 或
`video_service/deploy/install_onboard_video_service.sh`；这两个脚本只用于飞机上安装开机自启的
systemd 服务。地面 Qt 面板需要本机摄像头时，会按需启动普通用户态视频进程，也不会安装机载
`video-service.service`。


```bash
ROS_DOMAIN_ID=0
ROS_LOCALHOST_ONLY=0
```

# 机载服务

## 新机载计算机部署索引

当前已验证目标是 Jetson Orin NX、Ubuntu 24.04、ROS 2 Jazzy、aarch64。更换飞机时不要复制
开发机的 `build/`、`install/` 或 x86 MediaMTX，应在新机按以下顺序原生部署：

1. 安装 ROS 2 Jazzy、MAVROS 和机载硬件驱动，原生构建 Odin 与 extnav overlay；
2. 按[机载服务最小部署指导](src/onboard_control/deploy/ONBOARD_DEPLOYMENT.md)建立 sparse checkout、
   配置串口/overlay 并执行安全 smoke；
3. 人工核对 `/dev/serial/by-id` 或板载串口，不得猜测飞控设备；
4. 执行 `./src/onboard_control/deploy/install_onboard_service.sh`，一键构建并部署只负责 MAVROS、
   Odin、extnav、onboard_control 四组件的飞控 systemd unit；
5. 按[独立视频服务新飞机部署步骤](video_service/README.md#新飞机快速部署ubuntu-2404--jazzy--arm64)
   手工安装 FFmpeg 与系统 ARM64 MediaMTX，再执行
   `./video_service/deploy/install_onboard_video_service.sh` 部署独立 `video-service.service`；
6. 保持 `armed=false`，依次完成构建测试、隔离 smoke、MAVROS 只读状态、视频 RTSP/录像/抓拍和
   摄像头故障隔离验收，不能用进程启动成功代替硬件验证。

飞控与视频必须是两个独立 unit。视频服务不能加入 `start_onboard_control.sh`，也不能通过
`Requires=`、`PartOf=` 或 `BindsTo=` 绑定飞控服务。


## 重新构建机载控制

地面开发机和飞机都在仓库根目录执行同一个命令：

```bash
./build_onboard_control.sh
```

如需同时检查依赖、运行单元测试和隔离 smoke：

```bash
./build_onboard_control.sh --verify
```

脚本按目标机系统选择其原生 ROS 发行版。它不会启动、停止或重启机载服务，也不会发送飞行
命令；构建完成后如需让运行中的飞机加载新产物，应另行选择安全窗口重启对应服务。

## 开机自启与自动重启

飞控 unit 配置：

```text
/etc/systemd/system/ros2-ardupilot-onboard.service
```
新机必须按实际用户、工作区、ROS 与两个硬件 overlay 修改绝对路径，人工核对后再启用：

```bash
sudo systemd-analyze verify /etc/systemd/system/ros2-ardupilot-onboard.service
sudo systemctl daemon-reload
sudo systemctl enable ros2-ardupilot-onboard.service
```


机载计算机开机后，会等待网络就绪和系统首次校时完成，然后运行：

```text
/home/<机载用户>/ros2-ardupilot-mavros-control/start_onboard_control.sh
```

若机载服务在系统首次 NTP 校时之前启动，墙钟跳变可能使已创建的 MAVROS/ROS 定时链异常，
导致地面站无法连接。

该服务会自动重启：`start_onboard_control.sh` 启动的四个组件中，任一主要进程异常退出后，脚本会清理
其余组件并以失败状态退出，systemd 在 10 秒后重启整组服务。

通过 `systemctl stop` 主动停止服务不会触发自动重启。

飞控四组件的一键 systemd 安装入口为：

```bash
./src/onboard_control/deploy/install_onboard_service.sh
```

它会构建和验证机载包、首次生成现场环境文件、安装四组件 unit 并开启自启动；服务已 active 时
会拒绝自动更新，避免在未知飞行状态下重启。完整边界见
[机载部署指导](src/onboard_control/deploy/ONBOARD_DEPLOYMENT.md)。

视频 unit 为 `/etc/systemd/system/video-service.service`，其可替换占位符模板和完整安装命令位于
[video_service/README.md](video_service/README.md#新飞机快速部署ubuntu-2404--jazzy--arm64)。

视频服务安装器位于自己的部署目录，启停入口仍位于项目根目录：

```bash
./video_service/deploy/install_onboard_video_service.sh
./start_onboard_video.sh
./stop_onboard_video.sh
./stop_onboard_video.sh --restart
```

新飞机先按视频 README 手工安装 FFmpeg、v4l-utils 与系统 MediaMTX，再执行第一条部署配置、
媒体目录和独立 systemd unit；已有现场配置不会被覆盖。其余三条用于手动启动、彻底停止和清理
后重启。

最后一条会先彻底清理独立视频 unit、残留节点、RTSP 端口和真实摄像头占用者，再只重启
`video-service.service`；不会停止或重启飞控四组件。


## 配置

`start_onboard_control.sh` 会自动读取以下配置文件：

```text
/etc/ros2-ardupilot/onboard.env
```
```bash
ROS_DOMAIN_ID=0
ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
MAVROS_FCU_DEVICE=/dev/serial/by-id/<已核对的飞控设备>
MAVROS_FCU_BAUD=460800
ODIN_OVERLAY_SETUP=/home/<机载用户>/<odin工作区>/install/setup.bash
EXTNAV_OVERLAY_SETUP=/home/<机载用户>/<extnav工作区>/install/setup.bash
```

板载 `/dev/ttyTHS1:460800` 只能在逐机核对接线后使用；存在稳定 `/dev/serial/by-id` 时优先使用
后者。当前 Jetson 使用 Jazzy 的 `ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET`；旧 Humble 目标才使用
`ROS_LOCALHOST_ONLY=0`。

使能端口：

永久提权：
```bash
sudo usermod -aG dialout $USER
```
重新登录后用 `id` 确认 `dialout` 已生效；不要用 `chmod 777` 把串口永久开放给所有用户。

## 一键启动

```bash
cd /home/<机载用户>/ros2-ardupilot-mavros-control
./start_onboard_control.sh
```

## 结束机载服务

```bash
cd /home/<机载用户>/ros2-ardupilot-mavros-control
./stop_onboard_control.sh
```

该入口先停止 systemd 服务，再清理从其他终端手工启动的 MAVROS、Odin、extnav、
onboard_control 和相关 RViz；只有服务 inactive 且目标进程全部退出时才返回成功。

或者在启动机载服务的脚本的终端按一次 Ctrl+C, 出现
```
[startup] stopping all four components...
```
后等待，直到出现成功提示。连续Ctrl+C会导致残留


# 远程桌面

Ubuntu2404 remmina远程桌面

1. 终端输入`seahorse`，左上角加号添加“密码密钥环”，名称为“Login”，用户名 密码 均留空，直接点continue；
2. 右键Login密钥环，设为默认
3. 设置-系统-远程桌面：打开桌面共享，关闭远程登录；
4. 确认 设置-系统-用户 开启了自动登录
