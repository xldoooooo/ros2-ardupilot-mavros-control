<!-- Ground station deployment and ROS distribution compatibility. -->
# 地面站部署（Ubuntu 22.04 / 24.04）

地面支持 Ubuntu 22.04/Humble 和24.04/Jazzy；当前正式飞机使用24.04/Jazzy。
实机通信优先保持两端 ROS 发行版一致，跨发行版 DDS 需重新验证。
机载安装、串口/overlay 和台架核验见[机载部署](../src/onboard_control/deploy/ONBOARD_DEPLOYMENT.md)。

## 安装

先安装目标发行版 ROS、编译工具、FFmpeg/ffprobe、v4l2-ctl、匹配架构的 MediaMTX，
视频依赖见[地面视频部署](../video_service/deploy/GROUND_VIDEO_DEPLOYMENT.md)。
然后在普通用户目录检出：

```bash
git clone https://github.com/xldoooooo/ros2-ardupilot-mavros-control.git
cd ros2-ardupilot-mavros-control
bash scripts/ground/setup_ground_station.sh
```

安装器创建项目 `.venv`，安装 Python 依赖，原生构建/测试飞行、仿真及修正相关包，检查环境，
安装应用菜单入口。它会用 sudo/apt 补装缺少的 Eigen、JSON 开发包及 `libxcb-cursor0`，
不会启动 MAVROS、访问真实串口或发送飞行命令；其他缺包会报告名称，补齐后重新运行。

## 本机配置

`config/host.local.env` 不入 Git，用于 `GROUND_STATION_ROS_DISTRO`、
`GROUND_STATION_PYTHON`、overlay 路径、`GROUND_STATION_HARDWARE_DOMAIN_ID` 等本机设置；
参照 `config/host.local.env.example`。不依赖旧用户配置目录或 shell 初始化文件。
地面启动不读取飞机 `onboard.env`。实机 domain 默认0，应与飞机一致；231保留本地仿真。

仿真需要 ArduPilot `sim_vehicle.py` 和 MAVProxy。启动器从 PATH、项目虚拟环境和常见
源码/虚拟环境位置发现；非标准位置可设置 `GROUND_STATION_MAVPROXY` 等对应覆盖，
找不到时在启动仿真进程前报错。

## 启动与验证

```bash
bash scripts/ground/start_ground_all.sh --check-environment
bash scripts/ground/start_ground_all.sh
```

先手动初始化本地仿真验证 UI、航点和媒体；上位机仿真见[工作流](simulation-workflow.md)。
实机台架保持拆桨、未解锁，独立 Wi-Fi 图标只做被动通信检查；“连接实机服务”会申请租约并
写入确认的 GPS 原点。实机解锁和起飞只能由用户手动执行。

开机自启、独立服务与日志见[启动与日志](startup-and-logs.md)。
