<!-- Native onboard deployment, independent services and operational maintenance. -->
# 机载部署与维护

当前目标为 Jetson Orin NX、Ubuntu 24.04、ROS 2 Jazzy、aarch64。全新系统先按
[新机清单](JETSON_NEW_MACHINE_CHECKLIST.md)及[依赖说明](ONBOARD_DEPENDENCIES.md)准备。
实机解锁、起飞只能由用户手动执行；编译和隔离测试不代表硬件或飞行验收。

## 检出、配置与构建

在部署用户自己的目录检出源码，目标机必须原生构建，不能复制开发机 `build/`、`install/`
或跨架构二进制。已有现场仓库先保留本地修改，不使用 reset/clean 或整仓覆盖。

```bash
git clone --filter=blob:none --no-checkout \
  https://github.com/xldoooooo/ros2-ardupilot-mavros-control.git
cd ros2-ardupilot-mavros-control
git sparse-checkout init --no-cone
git sparse-checkout set '/.gitignore' '/project_configuration.py' \
  '/src/guided_interfaces/' '/src/correction_interfaces/' '/src/onboard_control/' \
  '/src/avoidance_bridge/' '/drivers/' '/config/' '/system/' \
  '/correction_service/' '/video_service/' '/scripts/onboard/' '/scripts/lib/' '/scripts/diagnostics/'
git checkout main
```

`config/aircraft.local.env` 选择 `AIRCRAFT_ID`；现场参数集中在
`config/aircraft/<AIRCRAFT_ID>/`，本机依赖路径可写入不提交的 `config/host.local.env`。
确认 `onboard.env` 的 FCU 串口、波特率、ROS domain、`EXTNAV_OVERLAY_SETUP`，
Jetson 另设匹配 OpenCV ABI 的 `ODIN_CV_BRIDGE_SETUP`，见
[OpenCV overlay](JETSON_OPENCV_OVERLAY.md)。逐机地图、设备标定和机械外参不得跨机覆盖。

```bash
./src/onboard_control/deploy/onboard_workspace.sh show-config
./src/onboard_control/deploy/onboard_workspace.sh deps-check
./src/onboard_control/deploy/build_onboard_control.sh --verify
```

`deps-check` 只读；正常构建会补装缺少的 Eigen/JSON 开发包，再增量构建项目 Odin、
飞行/修正接口、控制节点、修正服务和避障桥。完整依赖仍由前置安装器准备。
构建使用所选独立 CPU 规划器 overlay（`AVOIDANCE_WORKSPACE`，默认相邻
`dyn_small_obs_avoidance-ros2`），该工作区须先按[避障指南](../../avoidance_bridge/README.md)准备。
不加 `--verify` 时只构建；底层 `onboard_workspace.sh` 可单独执行 `build/test/smoke/verify`。

隔离 smoke 使用 localhost、非零独立 ROS domain 和专用 MAVROS 前缀，不连接硬件。
当前成功摘要必须为 `interface=3.6, fcu_connected=false, armed=false, setpoint_messages=0`。
构建不重启任何已运行服务；需在维护窗口重启对应进程使新产物生效。

## 四组件飞控服务

```bash
./scripts/onboard/start_onboard_control.sh --check
./src/onboard_control/deploy/install_onboard_service.sh --install-only
```

`--check` 只发现配置、串口和安装入口，不连接设备。安装器 `--install-only` 仍执行构建、
测试和隔离 smoke，安装 unit 与 Odin USB 规则，但不启用或启动服务，适合硬件未接齐的新机。
硬件及 extnav 核验完成后，在未解锁维护窗口运行不带选项的安装器，将执行 `enable --now`。
安装器拒绝更新 active 飞控 unit，须先确认飞机状态再停止服务。

前台使用：

```bash
./scripts/onboard/start_onboard_control.sh
./scripts/onboard/stop_onboard_control.sh
```

四组件为 MAVROS、项目 Odin、extnav 和 onboard_control；systemd unit 为
`ros2-ardupilot-onboard.service`。重复实例会拒绝启动，任一组件退出会清理本组。
Ctrl+C 或停止脚本结束组件；日志在 `/tmp/ros2_ardupilot_onboard/<时间戳>/`。
`READY` 要求同一条状态中的 FCU 连接、未解锁、遥测配置、推力语义及本地位姿均有效。
等待超时会保留受监督组件，原因见 `readiness.log`，不能把进程存在当成已就绪。

完整入口检查实际 MAVROS TIMESYNC 参数，恢复被禁用的 MAVLINK 同步频率并等待有效
`/mavros/timesync_status`。onboard 自动维护必要消息频率、读取 `GUID_OPTIONS` 和
`MOT_THST_HOVER`，不自动解锁/起飞。`priority_parameter_reads` 可关闭额外按名只读请求；
自定义路由/目标或签名链需同步配置，见 `control.yaml`。

分组件入口在 `scripts/onboard/components/`：`start_link.sh`、`start_mavros.sh`、
`start_odin.sh`、`start_extnav.sh`；Odin 固定使用项目 `drivers/odin/install`，
旧 `ODIN_OVERLAY_SETUP` 不再选择驱动。

## 独立服务与导航

| 功能 | 部署入口 | unit / 说明 |
| --- | --- | --- |
| 视频 | `video_service/deploy/install_onboard_video_service.sh` | `video-service.service`；[媒体与相机配置](../../../video_service/README.md)。 |
| AprilTag 修正 | `correction_service/deploy/install_correction_service.sh` | `odin-correction.service`；启动 idle、不自动开相机；[接口与配置](../../../correction_service/README.md)。 |
| extnav | `correction_service/deploy/install_extnav_correction.sh` | 独立工作区原生构建，安装前停止飞控链；[实现](../../../correction_service/extnav_patch/README.md)。 |
| CPU 避障 | `build_avoidance.sh` 后运行 `install_avoidance_service.sh` | `ros2-ardupilot-avoidance.service`；[配置与验证](../../avoidance_bridge/README.md)。 |

视频、修正和避障分别启停，不能对飞控 unit 加 `Requires/PartOf/BindsTo` 形成共同故障域。
视频/修正也支持各自安装器的 `--install-only`。MediaMTX 由依赖安装器按架构安装，仓库不携带其二进制。
机载缓存及地面集中媒体目录以当前媒体配置为准，不使用历史 `/home/share` 路径。

Odin 显式选模式：`start_odin.sh -c odom|slam|reloc|original`，也可传绝对 YAML 路径。
不传 `-c` 时机载脚本读取 `navigation.env` 的 `NAVIGATION_MODE=odom|map`，map 对应 reloc。
地图必须先在 `odin/reloc.yaml` 配置存在、可读、非空的绝对路径。

运行中模式切换使用地面站导航开关或 `scripts/onboard/switch_navigation_mode.sh map|odom`。
切换需新鲜状态证明 FCU 已连接、未解锁、已落地、无飞行/航点/Tag 任务；停止并恢复原运行的
飞控及避障，视频和修正不参与重启。机载用户需相关命令的无交互 sudo 权限。
失败不自动回滚，检查 `/tmp/ros2_ardupilot_onboard/navigation-switch/` 和 journal。
模式事实来自 `/odin1/navigation_status`，不是配置文件；详见
[odom/map](../../../docs/odom-map.md)及[Tag 航点工作流](../../../docs/tag-waypoint-workflow.md)。
map 与 active Tag 修正组合尚未支持。

## ROS 网络与台架核验

正式 Jazzy 链默认 `ROS_DOMAIN_ID=0`、`ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET`。
飞机、地面站及诊断终端必须在进程启动前使用一致 domain；231 保留本地仿真。
地面站 domain 由 `host.local.env` 的 `GROUND_STATION_HARDWARE_DOMAIN_ID` 指定。
Humble 使用 `ROS_LOCALHOST_ONLY=0`，跨发行版 DDS 不保证兼容，不能沿用 Jazzy/Jazzy 结论。

各服务最多等 30 秒局域网 IPv4，超时告警后离线继续；不等待外网或 NTP。
网络稍后恢复时，在未解锁状态重启受影响服务恢复发现。离线后重新联网并发生校时，应在人工
解锁前等待系统时钟与 MAVROS TIMESYNC 稳定并复核定位。

台架保持拆桨、未解锁：先读 `/onboard_control/status` 和 `/mavros/state`，再核对外部定位、
参数、位置/速度实测速率。GUI 独立 Wi-Fi 图标只做被动通信检查；“连接实机服务”会申请租约、
维护遥测并写入确认的 GPS 原点，不等于只读检查。请求 ACK 不能替代实际频率或定位精度。

## 更新与 MAVROS 维护

干净的 sparse checkout 可执行：

```bash
./src/onboard_control/deploy/onboard_workspace.sh update
./src/onboard_control/deploy/build_onboard_control.sh --verify
```

更新使用 `git pull --ff-only`，不覆盖本地修改；完整 checkout 直接使用 Git 同步。
接口变更必须同时更新地面站、接口与消费端并原生重建，当前飞行接口为 3.6、独立视频为3.2、
correction 为2.0。旧进程须重启。仅拉取源码不会更新已安装 unit；入口路径改变时重跑相应安装器。

Jazzy MAVROS 2.15.1/2.16.0 的参数重试及插件参数加载修复由以下入口安装独立 overlay：

```bash
./src/onboard_control/deploy/install_mavros_param_fix.sh
```

安装器使用固定版本及 SHA-256 校验，不覆盖 `/opt/ros/jazzy`，不自动重启服务。
在所选 `onboard.env` 设置 `MAVROS_OVERLAY_SETUP`，重启后核对实际加载的
`libmavros_plugins.so` 路径和 `/mavros/setpoint_raw` 的 `thrust_scaling=1.0`。
移除此配置并重启可回退系统版；MAVROS 升级后重新评估 ABI，不复用旧版本 overlay。
项目保留 node.launch 的版本默认命名空间，避免 2.14/2.16 差异导致话题路径错误。

补丁含遥测诊断，日志为 `$ROS_LOG_DIR/mavros-telemetry-*.log`（默认 `~/.ros/log/`），
与 `onboard-control-diagnostics-*.log` 配对分析。`MAVROS_EVENT` 是慢调用或入口间隔，
`MAVROS_INFLIGHT` 是未返回调用；`gap_ms` 含上一调用耗时，不能直接当作串口断流。
源码入口为 `mavros_diagnostics/` 和 `patches/mavros-telemetry-diagnostics.patch`。

GitHub 直连受限可从可联网开发机建立临时反向 SOCKS 隧道，例如
`ssh -R 127.0.0.1:19080 drone-refresh`，在飞机当前命令设置
`HTTPS_PROXY=socks5h://127.0.0.1:19080`；不写入全局代理。

## 飞控热重启

GUI“重启机载飞控”和机载 `scripts/onboard/reboot_fcu.sh` 共用 FlightCommand，仅允许新鲜
未解锁、已落地、空闲无任务状态。脚本使用项目 `.venv` 和短租约，地面站已持权时需先断开。
重启只作用于飞控，不重启计算机，不恢复旧任务。

成功分为飞控启动时钟回退的重启证据，以及定位/原点/参数/遥测恢复并稳定的链路证据；ACK
本身不代表成功，也不触发自动重发。最多等待90秒，只恢复此前从飞控回读的 GPS 原点。
USB 消失后重新枚举不在此恢复范围。维护入口为 `src/onboard_control/src/fcu_reboot.cpp`
和 `src/onboard_control/scripts/reboot_fcu_client.py`（均相对项目根目录）。
