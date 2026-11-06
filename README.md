<!-- ROS 2 地面站、机载控制及独立媒体/定位服务的使用入口。 -->
# ROS 2 ArduPilot 地面站与机载控制

用于内部无人机巡检与 SITL 验证的 ROS 2 项目，包含 PySide6 地面站、机载 PD+DOB 控制器、Odin 定位、航点避障、AprilTag 校准与集中媒体服务。当前部署环境为 Ubuntu 24.04 / ROS 2 Jazzy；机载计算机使用 Jetson Orin NX。

支持 odom/map 航点、直线/遇障悬停/自主避障策略、上位机 WebSocket 接入，以及地面 RTSP 直播、录像和 FTP 下载。可选 FPGA 避障模块独立于默认 CPU 路径。

**实机解锁与起飞只能由用户手动操作。构建、静态检查与 SITL 结果不能替代实机验收。**

## 地面站

安装 ROS 2、MAVROS、编译工具及[地面视频依赖](video_service/deploy/GROUND_VIDEO_DEPLOYMENT.md)后，在仓库根目录执行：

```bash
./scripts/ground/setup_ground_station.sh
bash scripts/ground/start_ground_all.sh
```

安装脚本建立项目 `.venv`、构建 ROS 包并执行检查。顶栏提供桌面登录自启和自动连接飞机选项；相关配置、服务与日志见[启动与日志](docs/startup-and-logs.md)。

本地仿真由地面站手动初始化，使用独立 ROS domain 231；实机会话默认使用 domain 0。上位机通过 WebSocket 发送高层任务，机载控制器负责实际执行。接入与样例媒体测试见[WebSocket 对接](docs/websocket/上位机-WebSocket%20对接.md)和[仿真巡检](docs/simulation-workflow.md)。

## 机载部署

按[新机检查清单](src/onboard_control/deploy/JETSON_NEW_MACHINE_CHECKLIST.md)和[机载部署指导](src/onboard_control/deploy/ONBOARD_DEPLOYMENT.md)安装依赖、选择逐机配置并在目标机原生构建；不得复制开发机的 `build/`、`install/` 或其他架构二进制。

```bash
./src/onboard_control/deploy/build_onboard_control.sh --verify
./scripts/onboard/start_onboard_control.sh --check
```

飞控、避障、视频与 Tag 修正分别使用独立 systemd 服务。部署入口和使用方法分别见：

- [机载控制与飞控服务](src/onboard_control/deploy/ONBOARD_DEPLOYMENT.md)
- [CPU 避障桥](src/avoidance_bridge/README.md)与[可选 FPGA 模块](fpga_avoidance/README.md)
- [视频与集中媒体服务](video_service/README.md)
- [AprilTag 修正服务](correction_service/README.md)与[航点自动校准](docs/tag-waypoint-workflow.md)

## 配置与开发

本机 `config/aircraft.local.env` 选择 `uav001`、`uav002` 或 `simulation`；参数在 `config/aircraft/<编号>/`，本机依赖路径在 `config/host.local.env`。修改运行参数后重启对应服务，通常无需重建；接口或源码变更需原生重建。同步时保留目标机的标定、地图路径和本机环境。

- [配置索引](config/README.md)：控制、相机、MAVROS、Odin 与地面媒体配置。
- [Odin 驱动](drivers/odin/README.md)：项目维护源码与独立构建入口；点云配置位于逐机 Odin YAML。
- [odom/map 导航](docs/odom-map.md)：坐标链、模式切换与导航维护入口。
- [测试说明](tests/README.md)：项目环境、测试选择与隔离验证。

地面媒体服务独立于 GUI 运行，默认提供 HTTP 8090、只读 FTP 2121 和 RTSP 8554；实际地址由本机媒体配置指定。下载路径相对于媒体 `root/public`，不再使用飞机旧 `/home/share`。
