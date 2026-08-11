# Ubuntu 22.04 / ROS 2 Humble 部署与无桨调试

本文档用于在新电脑上部署完整项目。启动器从自身位置推导仓库根目录，并自动发现 ROS、
项目 `.venv`、ArduPilot、Odin/extnav overlay 和唯一飞控串口；正常布局不需要编辑脚本。

## 安全边界

- 首次部署和台架调试必须拆除螺旋桨，并持续确认飞控 `armed=false`。
- 脚本不允许自动解锁或起飞实机；这两项只能由操作者人工执行。
- 串口或 ROS overlay 存在多个候选时，启动器会拒绝猜测并安全退出。

## 1. 完整检出与自动部署

```bash
git clone https://github.com/xldoooooo/ros2-ardupilot-mavros-control.git
cd ros2-ardupilot-mavros-control
bash setup_project.sh
```

`setup_project.sh` 自动完成：

- 依据 Ubuntu 22.04/24.04 选择 Humble/Jazzy；
- 创建项目本地 `.venv` 并安装 Qt 依赖；
- 原生构建三个 ROS 包；
- 执行 ROS/C++ 测试和地面站环境检查。

脚本不会调用 `apt`、`sudo`、MAVROS、真实串口或任何飞行命令。若报告系统依赖缺失，
先人工审查缺包名称，安装后重新执行同一命令。

## 2. 地面站与 SITL

```bash
bash start_ground_all.sh --check-environment
bash start_ground_all.sh
```

启动器优先使用仓库内 `.venv`，并在 22.04 上使用 Humble 的
`ROS_LOCALHOST_ONLY`，在 24.04 上使用 Jazzy 的
`ROS_AUTOMATIC_DISCOVERY_RANGE`。本地仿真仍固定为 domain 231，实机固定为 domain 0。

ArduPilot 会依次从当前 `PATH`、仓库同级目录、当前用户目录、`/opt` 与 `/usr/local/src`
的常见源码布局中寻找 `sim_vehicle.py`。

## 3. 机载四组件发现检查

先执行只发现、不启动：

```bash
bash start_drone_all.sh --check
```

成功输出会列出 ROS 发行版、当前仓库、FCU 串口、Odin 和 extnav 的实际安装前缀。
随后保持拆桨和未解锁，再由操作者运行：

```bash
bash start_drone_all.sh
```

启动器只在发现唯一串口时继续，优先采用 `/dev/serial/by-id` 稳定名称；否则检查
`ttyTHS`、`ttyACM` 和 `ttyUSB`。Odin/extnav 通过 ament package index 反查其 overlay，
不依赖用户名或工作区目录名称。

## 4. 无法唯一判断时

多块串口设备或多个旧 overlay 本身具有安全歧义，程序不会静默选择。确认实际硬件后，
只对当前命令提供覆盖值，不需要修改仓库文件：

```bash
MAVROS_FCU_DEVICE=/dev/serial/by-id/<已确认设备> bash start_drone_all.sh --check
ODIN_OVERLAY_SETUP=<已确认的setup.bash> \
EXTNAV_OVERLAY_SETUP=<已确认的setup.bash> \
  bash start_drone_all.sh --check
```

这些覆盖只用于消除真实歧义。不得根据名称猜测设备，更不得把自动发现扩展为自动解锁或起飞。

## 5. 实机无桨验收

一键启动报告 `READY` 前，聚合状态必须满足：接口 `2.0`、FCU 已连接且未解锁、消息频率
配置完成、推力语义确认、本地位置有效。随后只使用 GUI 的 Wi-Fi 图标进行被动通信检查；
正式“连接实机服务”会申请租约并写入人工确认的 GPS/EKF 原点，不属于只读检查。

任何 `armed=true`、位置跳变、`thrust_mode_verified=false`、
`setpoint_conflict=true` 或重复进程都必须立即停止调试。
