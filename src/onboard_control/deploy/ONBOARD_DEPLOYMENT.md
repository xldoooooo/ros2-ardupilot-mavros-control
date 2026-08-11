# 机载服务最小部署与验证

本文档用于把 `guided_interfaces` 与 `onboard_control` 两个 ROS 2 包部署到伴随计算机，当前实机目标为 Ubuntu 22.04、ROS 2 Humble、aarch64。

## 安全边界

- 严禁在本流程中解锁或起飞实机。
- 初次部署只允许编译、单元测试和隔离烟雾测试；不启动真实 MAVROS、Odin、外部定位桥或地面站控制客户端。
- 不修改 `/home/xld` 下的旧仓库、旧脚本和旧工作空间。旧启动流程继续作为回退方案。
- 不安装或启用 systemd 自启动服务。完成独立台架通信验证前，不得把新机载节点加入开机启动。
- `smoke` 使用非零独立 ROS domain、localhost-only 发现和专用 MAVROS 前缀，不会发现真实 MAVROS；它只检查待机状态，并断言没有姿态 setpoint 消息。

## 1. 首次最小拉取

在部署用户自己的目录执行；启动器会从自身位置解析工作区，不要求固定用户名或绝对路径：

```bash
git clone \
  --filter=blob:none \
  --no-checkout \
  https://github.com/xldoooooo/ros2-ardupilot-mavros-control.git \
  ros2-ardupilot-mavros-control

cd ros2-ardupilot-mavros-control
git sparse-checkout init --no-cone
git sparse-checkout set \
  '/src/guided_interfaces/' \
  '/src/onboard_control/' \
  '/start_drone/' \
  '/start_drone_all.sh'
git checkout main
```

检出后，除 Git 元数据外只应出现：

```text
src/guided_interfaces/
src/onboard_control/
start_drone/
start_drone_all.sh
```

不要复制开发机的 `build/` 或 `install/`。目标机必须针对 Humble/aarch64 原生编译。

### GitHub 直连超时时

当前无人机能解析 `github.com`，但实测 HTTPS 443 直连可能超时。不要写入永久系统代理，也不要复制包含地面站历史的完整 Git bundle。可从能够访问 GitHub 的地面机建立仅绑定无人机 localhost、只在当前 SSH 会话存活的反向动态 SOCKS 隧道：

```bash
ssh -R 127.0.0.1:19080 xld@192.168.112.186
```

然后在这个 SSH 会话内临时设置：

```bash
export HTTPS_PROXY=socks5h://127.0.0.1:19080
export HTTP_PROXY=socks5h://127.0.0.1:19080
```

再执行本节的 `git clone`，或执行后文的 `onboard_workspace.sh update`。退出 SSH 后隧道自动消失；不要把代理写入 `/home/xld/.gitconfig`、shell 启动文件或 systemd 环境。

## 2. 无系统修改的依赖检查

```bash
./src/onboard_control/deploy/onboard_workspace.sh show-config
./src/onboard_control/deploy/onboard_workspace.sh deps-check
```

`deps-check` 只读取工具链、Eigen 头文件和 ROS 包索引，不依赖已初始化的 rosdep 数据库，也不会调用 `apt` 或 `sudo`。如果报告缺包，应先人工审查清单；不要直接批量安装或升级已有 ROS 环境。

## 3. 编译、单元测试与安全烟雾测试

```bash
./src/onboard_control/deploy/onboard_workspace.sh build
./src/onboard_control/deploy/onboard_workspace.sh test
./src/onboard_control/deploy/onboard_workspace.sh smoke
```

也可一次执行：

```bash
./src/onboard_control/deploy/onboard_workspace.sh verify
```

成功烟雾测试必须同时报告：

```text
interface=2.2
fcu_connected=false
armed=false
setpoint_messages=0
```

这只证明 Humble 上的二进制能够启动并发布安全待机状态，不证明跨发行版 DDS、Odin/MAVROS、外部定位、飞控参数或真实控制链已经可用。

## 4. 后续更新

```bash
./src/onboard_control/deploy/onboard_workspace.sh update
./src/onboard_control/deploy/onboard_workspace.sh verify
```

`update` 只允许 sparse checkout，并要求 Git 工作树干净；它会同时维护两个机载 ROS 包、
`start_drone/` 分步入口和 `start_drone_all.sh` 一键入口。更新使用 `git pull --ff-only`，
不会 reset、覆盖本地修改或触碰 `/home/xld`，也不会把仅供地面使用的
`start_ground_all.sh` 检出到无人机。

## 5. Humble/Jazzy 网络变量

`ROS_DOMAIN_ID` 只负责把 DDS 参与者划入彼此隔离的逻辑网络，不是地址或链路质量参数。
未设置时默认值为 `0`。使用 `42` 可以隔离同一局域网中的其他 ROS 系统，但必须在进程
启动前同时应用给机载控制、MAVROS、Odin、extnav、地面站和诊断 CLI；只改其中一个终端，
或在节点启动后才修改环境，会让 `ros2 topic list` 看起来为空。

机载 Humble 使用：

```bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
```

地面站 Jazzy 使用：

```bash
export ROS_DOMAIN_ID=42
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
```

`ROS_AUTOMATIC_DISCOVERY_RANGE` 和 `ROS_STATIC_PEERS` 不是 Humble 的通用配置接口。Humble 默认通过同网段多播发现；若多播不可靠，需要为双方实际使用的 Fast DDS 版本分别验证 XML discovery 配置。

上面的 `42` 只是两端一致配置的示例。当前四终端手工启动且没有加载统一环境时，应全部保持
默认 domain 0；不要只在某一条启动命令里硬编码 42。domain 值不会修复 Humble/Jazzy 的
跨发行版序列化兼容问题。

ROS 官方不保证 Humble 与 Jazzy 跨发行版通信。即使状态话题和服务在台架测试中可见，也只能视为当前版本组合的实测结果，不能据此宣称适合实飞。

任务 09 在 Humble `rmw_fastrtps_cpp 6.2.10` 与 Jazzy `8.4.3` 间实测时，
稳定收到自定义状态话题，但两端在发现部分跨发行版端点时反复输出
`sequence size exceeds remaining buffer`。这属于仍未解决的兼容性告警；在统一
ROS 发行版/DDS 版本或加入受支持桥接并复验之前，禁止把当前结果解释为完整 ROS 图兼容。

## 6. 下一步通信测试顺序

所有测试均保持螺旋桨拆除、飞控不解锁，并由低风险到高风险逐级进行：

1. **DDS 协议测试**：不启动 MAVROS，只启动新机载节点；地面站先只读取 `/onboard_control/status`，确认接口 `2.2`、`armed=false`。GUI 中齿轮右侧的独立 Wi-Fi 图标只订阅状态与远端 `/rosout`，测量状态频率和最大接收间隔；它不会申请租约、配置消息频率、写 GPS 原点或建立控制会话。
2. **MAVROS 只读测试**：只单独启动 MAVROS 并读取 `/mavros/state`，确认 `armed=false`。Odin/extnav 应留到单独评审后的外部定位测试，不调用任何模式、解锁、起飞或原点服务。
3. **同机链路测试**：启动新机载节点；它会自动、异步配置控制链必需的 `LOCAL_POSITION_NED`、`ATTITUDE_QUATERNION`、`HIGHRES_IMU` 为 100 Hz，并在飞控重连后重试。只观察聚合状态是否正确反映飞控连接、位姿、`GUID_OPTIONS` 与 setpoint 发布者冲突；使用独立 Wi-Fi 图标，不点击“连接实机服务”，不发送 `FlightCommand`/`MotionIntent`/航点。自动消息频率维护不包含模式、解锁、起飞或参数写入。
4. **完整连接与维护测试**：原“连接实机服务”按钮保留正式功能，会在明确风险确认后申请控制租约、发送心跳、确认消息频率并写 GPS 原点，连接成功后按权威状态开放控制按钮。只有前三步均通过并单独完成安全评审后才可测试；该按钮本身不会发送解锁或起飞请求，但不属于零命令通讯检测。
5. **飞行控制测试**：本任务明确禁止，不能进行解锁、起飞或实机姿态/推力控制。

## 7. 实机四组件一键启动

机载工作区根目录的 `start_drone_all.sh` 自动发现并统一启动四个组件：

- MAVROS（优先唯一的 `/dev/serial/by-id`，波特率默认 460800）；
- Odin 驱动；
- Odin 到 MAVROS 的外部定位桥；
- `onboard_control_node`。

在真机桌面终端中执行一行命令：

```bash
bash start_drone_all.sh --check
bash start_drone_all.sh
```

脚本默认让四个组件都使用 ROS domain 0，并等待以下只读安全条件成立后打印
`READY`：飞控已连接且未解锁、三路必需高频 MAVLink 消息与电池状态频率已配置、推力参数已确认、
本地位姿有效。它不会请求模式切换、解锁、起飞、控制租约或发送飞行指令。

脚本启动时会读取 `/etc/ros2-ardupilot/onboard.env`，也可用
`ONBOARD_ENV_FILE` 指定另一文件。已人工确认串口或 overlay 后，应在该机专用
环境文件中配置，使 systemd 与交互终端直接执行脚本时共享同一选择。

在运行脚本的同一终端按 `Ctrl+C` 会停止本次启动的全部进程。每次运行的合并日志位于
`/tmp/ros2_ardupilot_onboard/<时间戳>/`；若检测到四个组件中的任一既有实例，脚本会拒绝
重复启动，须先确认旧实例的来源和状态，不要直接强杀。

仅当自动发现报告多个真实候选时，人工确认后才可写入机载环境文件；
临时调试也可对当前命令覆盖配置，例如：

```bash
MAVROS_FCU_DEVICE=/dev/serial/by-id/<已确认设备> \
  bash start_drone_all.sh --check
```

Odin 的现有 launch 文件同时启动 RViz。在无图形环境的纯 SSH 会话中，RViz 会因没有
`DISPLAY` 而退出，但 Odin 驱动、外部定位桥和其余飞行数据链仍可运行；需要 RViz 时应从
真机桌面终端启动。该现象与 Humble/Jazzy 间的
`sequence size exceeds remaining buffer` 告警无关。

分步入口现集中在 `start_drone/`，包含 `start_link.sh`、`start_mavros.sh`、
`start_odin.sh` 和 `start_extnav.sh`；集成脚本不修改它们。部署前备份与本次实机测试结论
记录在对应日期的任务报告中。

## 8. systemd 示例

`onboard-control.service.example` 仅供未来部署参考，任务 08 不安装、不启用。使用前必须替换 `ONBOARD_USER`，确认 `/etc/ros2-ardupilot/onboard.env`，并完成独立的台架安全评审。服务只启动 `onboard_control`，不会替代 Odin、MAVROS 或 extnav 的生命周期管理。

服务通过 `systemd-time-wait-sync.service` 排在首次系统校时之后。机载 ROS/MAVROS
进程不得在该时点之前启动：开机保存时间与 NTP 时间之间的跳变会破坏已经创建的 ROS
墙钟定时器。不要用固定秒数的 `sleep` 替代这个就绪条件。
