# 机载服务最小部署与验证

本文档用于把飞行包、独立 `video_service` 和独立 AprilTag-Odin 修正服务部署到伴随计算机。

全新 Ubuntu 24.04 Jetson 请先按 [新机操作清单](JETSON_NEW_MACHINE_CHECKLIST.md)
配置系统，并用 [前置依赖安装器](ONBOARD_DEPENDENCIES.md) 补齐 ROS、MAVROS、视频及可选驱动依赖。
硬件尚未接齐时，`install_onboard_service.sh --install-only` 仍完成构建、测试和隔离 smoke，
但只安装配置与 unit，不启用或启动飞控服务；硬件路径和外部驱动需随后独立验收。
当前实机目标为 Jetson Orin NX、Ubuntu 24.04、ROS 2 Jazzy、aarch64；旧
Ubuntu 22.04/Humble 飞机只作为兼容历史，不代表当前运行基线。

## 安全边界

- 严禁在本流程中解锁或起飞实机。
- 初次部署只允许编译、单元测试和隔离烟雾测试；完成单独安全核验前，不启动真实 MAVROS、
  Odin、外部定位桥或地面站控制客户端。
- 保留目标机既有仓库、脚本、工作空间和配置；不得用 reset/clean 或整仓覆盖破坏现场修改。
- 完成独立台架通信验证前不得启用 systemd 自启动。当前 `192.168.112.169` 已完成未武装台架
  验证，飞控 unit 与独立视频 unit 均可保持 enabled，但两者不得建立共同故障域。
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
  '/src/correction_interfaces/' \
  '/src/onboard_control/' \
  '/correction_service/' \
  '/start_onboard_correction.sh' \
  '/stop_onboard_correction.sh' \
  '/video_service/' \
  '/start_onboard_video.sh' \
  '/stop_onboard_video.sh' \
  '/start_drone/' \
  '/start_onboard_control.sh' \
  '/stop_onboard_control.sh' \
  '/reboot_fcu.sh'
git checkout main
```

检出后，除 Git 元数据外只应出现：

```text
src/guided_interfaces/
src/correction_interfaces/
src/onboard_control/
correction_service/
start_onboard_correction.sh
stop_onboard_correction.sh
video_service/
start_onboard_video.sh
stop_onboard_video.sh
start_drone/
start_onboard_control.sh
stop_onboard_control.sh
src/onboard_control/deploy/build_onboard_control.sh
reboot_fcu.sh
src/onboard_control/scripts/reboot_fcu_client.py
```

不要复制开发机的 `build/` 或 `install/`。目标机必须针对自身 ROS 发行版和 aarch64 原生编译。

### GitHub 直连超时时

当前无人机能解析 `github.com`，但实测 HTTPS 443 直连可能超时。不要写入永久系统代理，也不要复制包含地面站历史的完整 Git bundle。可从能够访问 GitHub 的地面机建立仅绑定无人机 localhost、只在当前 SSH 会话存活的反向动态 SOCKS 隧道：

```bash
ssh -R 127.0.0.1:19080 nvidia@192.168.112.169
```

然后在这个 SSH 会话内临时设置：

```bash
export HTTPS_PROXY=socks5h://127.0.0.1:19080
export HTTP_PROXY=socks5h://127.0.0.1:19080
```

再执行本节的 `git clone`，或执行后文的 `onboard_workspace.sh update`。退出 SSH 后隧道自动消失；
不要把代理写入目标用户的 Git 配置、shell 启动文件或 systemd 环境。

## 2. 无系统修改的依赖检查

```bash
./src/onboard_control/deploy/onboard_workspace.sh show-config
./src/onboard_control/deploy/onboard_workspace.sh deps-check
```

`deps-check` 只读取工具链、Eigen 头文件和 ROS 包索引，不依赖已初始化的 rosdep 数据库，也不会调用 `apt` 或 `sudo`。如果报告缺包，应先人工审查清单；不要直接批量安装或升级已有 ROS 环境。

## 3. 编译、单元测试与安全烟雾测试

地面开发机和飞机都可以直接从仓库根目录执行同一个快捷入口：

```bash
./src/onboard_control/deploy/build_onboard_control.sh
```

默认以 Release 模式重建飞行包和独立修正接口/节点。修正节点构建不启动相机或飞控。若需同时执行依赖检查、
单元测试和隔离 smoke：

```bash
./src/onboard_control/deploy/build_onboard_control.sh --verify
```

脚本按主机自动选择 Jazzy/Humble，不启动、停止或重启机载服务，也不发送飞行命令。若构建时服务
正在运行，当前进程不会被替换；需要让新产物生效时，应另行选择安全窗口重启服务。

底层分阶段入口仍可直接使用：

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
interface=3.2
fcu_connected=false
armed=false
setpoint_messages=0
```

这只证明目标 ROS 发行版上的二进制能够启动并发布安全待机状态，不证明 Odin/MAVROS、
外部定位、飞控参数或真实控制链已经可用。

## 4. 后续更新

```bash
./src/onboard_control/deploy/onboard_workspace.sh update
./src/onboard_control/deploy/onboard_workspace.sh verify
```

`update` 只允许 sparse checkout，并要求 Git 工作树干净；它会同时维护飞行 ROS 包、
`correction_interfaces`、独立 `correction_service/` 与 `video_service/`、
根目录修正服务启停入口、
根目录视频启停入口、`start_drone/` 分步入口、`start_onboard_control.sh` 一键入口、
`stop_onboard_control.sh` 飞控彻底停止入口和部署目录 `src/onboard_control/deploy/build_onboard_control.sh`。更新使用
`git pull --ff-only`，
不会 reset 或覆盖本地修改，也不会把仅供地面使用的
`start_ground_all.sh` 检出到无人机。

`update` 只更新源码，不会替代目标机原生构建。源码与 `install/` 中
`guided_interfaces` 或 `onboard_control` 的包版本不一致时，`start_onboard_control.sh` 会在启动
任何飞行栈进程前安全失败，并提示先执行 `onboard_workspace.sh verify`；禁止绕过该检查继续
运行旧飞行协议二进制。`correction_interfaces` 不加入该飞行启动硬门：正常构建仍会安装它，
但缺失时 extnav 必须退化为 identity，而不能切断原 Odin→MAVROS 链。

## 5. ROS 网络变量

`ROS_DOMAIN_ID` 只负责把 DDS 参与者划入彼此隔离的逻辑网络，不是地址或链路质量参数。
未设置时默认值为 `0`。使用 `42` 可以隔离同一局域网中的其他 ROS 系统，但必须在进程
启动前同时应用给机载控制、MAVROS、Odin、extnav、地面站和诊断 CLI；只改其中一个终端，
或在节点启动后才修改环境，会让 `ros2 topic list` 看起来为空。

当前飞机与地面站均为 Jazzy，正式实机链使用 domain 0 + 同网段发现：

```bash
export ROS_DOMAIN_ID=0
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
```

`ROS_DOMAIN_ID` 必须在节点启动前对同一链路全部进程保持一致。隔离 smoke 固定使用非零 domain
与 localhost-only；不要把 smoke 的 domain 带入真实 MAVROS。domain 值不会改善地址、链路质量
或 DDS 序列化兼容性。

旧 Ubuntu 22.04/Humble 飞机使用 `ROS_LOCALHOST_ONLY=0` 进行同网段发现；Humble 不应直接照搬
Jazzy 的 `ROS_AUTOMATIC_DISCOVERY_RANGE`/`ROS_STATIC_PEERS`。历史 Humble/Jazzy 混合链曾出现
`sequence size exceeds remaining buffer`，ROS 官方也不保证跨发行版通信；若恢复旧机，必须按
双方实际 RMW 版本重新验证，不能沿用当前 Jazzy/Jazzy 结论。

## 6. 下一步通信测试顺序

所有测试均保持螺旋桨拆除、飞控不解锁，并由低风险到高风险逐级进行：

1. **DDS 协议测试**：不启动 MAVROS，只启动新机载节点；地面站先只读取 `/onboard_control/status`，确认接口 `3.2`、`armed=false`。GUI 中齿轮右侧的独立 Wi-Fi 图标只订阅状态与远端 `/rosout`，测量状态频率和最大接收间隔；它不会申请租约、配置消息频率、写 GPS 原点或建立控制会话。
2. **MAVROS 只读测试**：只单独启动 MAVROS 并读取 `/mavros/state`，确认 `armed=false`。Odin/extnav 应留到单独评审后的外部定位测试，不调用任何模式、解锁、起飞或原点服务。
3. **同机链路测试**：启动新机载节点；它会自动、异步配置控制链必需的
   `LOCAL_POSITION_NED(32)`、`ATTITUDE_QUATERNION(31)`、`HIGHRES_IMU(105)` 为 100 Hz，
   `BATTERY_STATUS(147)` 为 2 Hz，并额外请求 `EXTENDED_SYS_STATE(245)` 为 2 Hz，以取得真实
   起飞/落地状态；飞控重连后会重试。只观察聚合状态是否正确反映飞控连接、位姿、
   `GUID_OPTIONS` 与 setpoint 发布者冲突；使用独立 Wi-Fi 图标，不点击“连接实机服务”，不发送
   `FlightCommand`/`MotionIntent`/航点。自动消息频率维护不包含模式、解锁、起飞或参数写入。
4. **完整连接与维护测试**：原“连接实机服务”按钮保留正式功能，会在明确风险确认后申请控制租约、发送心跳、确认消息频率并写 GPS 原点，连接成功后按权威状态开放控制按钮。只有前三步均通过并单独完成安全评审后才可测试；该按钮本身不会发送解锁或起飞请求，但不属于零命令通讯检测。
5. **飞行控制测试**：本任务明确禁止，不能进行解锁、起飞或实机姿态/推力控制。

## 7. 实机四组件一键启动

### systemd 一键部署

确认 ROS 2 Jazzy、MAVROS、Odin 和 extnav 已安装，源码完成 sparse checkout 后，以普通机载用户
执行：

```bash
./src/onboard_control/deploy/install_onboard_service.sh
```

安装器会运行原生构建、单测和隔离 smoke，首次生成 `/etc/ros2-ardupilot/onboard.env`，根据当前
用户名与仓库路径安装 `/etc/systemd/system/ros2-ardupilot-onboard.service`，完成只读 `--check`
后执行 `enable --now`。已有 `onboard.env` 不会被覆盖。若飞控服务已经 active，安装器会拒绝更新，
必须由人工确认飞机未解锁并进入维护窗口后先停止服务；安装器绝不自行停止运行中的飞控链。

首次自动发现存在多个串口、多个 Odin/extnav overlay 或缺少硬件包时，`--check` 会明确失败。此时
应人工核对并编辑 `/etc/ros2-ardupilot/onboard.env`，不能为了“一键”而猜测飞控设备。

机载工作区根目录的 `start_onboard_control.sh` 自动发现并统一启动四个组件：

- MAVROS（优先唯一的 `/dev/serial/by-id`，波特率默认 460800）；
- Odin 驱动；
- Odin 到 MAVROS 的外部定位桥；
- `onboard_control_node`。

在真机桌面终端中执行一行命令：

```bash
bash start_onboard_control.sh --check
bash start_onboard_control.sh
```

脚本默认让四个组件都使用 ROS domain 0，并等待以下只读安全条件成立后打印
`READY`：飞控已连接且未解锁、必需的 MAVLink 控制遥测、电池与扩展飞行状态频率已配置、推力参数已确认、
本地位姿有效。它不会请求模式切换、解锁、起飞、控制租约或发送飞行指令。
就绪探针绕过可能陈旧的 ROS CLI daemon，并以显式 best-effort/volatile QoS 订阅接口版本明确的
`guided_interfaces/msg/ControlStatus`；这只改善同机只读探针的确定性，不解决跨 ROS 发行版
DDS 兼容性。

四组件并行拉起，不再插入固定的串行等待。飞控连接后 onboard 异步请求一次非强制参数同步，
跳过 MAVROS 默认的自动拉表延迟；约 2 秒后开始只读核验必要参数，未通过时每秒重试，
通过后维持每 5 秒复核。拉表响应不能代替 `GUID_OPTIONS` 与 `MOT_THST_HOVER` 的实际值校验。
机载端同时以 volatile QoS 接收 MAVROS `/param/event`：本连接的两个必要参数一旦齐全，立即
复用同一套类型/数值检查，不再等待整表服务退出；整表仍后台同步，缓存服务仍作为回退和定期复核。
断线清除已收参数，新事件会使较早发起的缓存读取结果失效，避免旧结果覆盖新参数。
机载端默认同时经现有 MAVROS 路由按名称读取这两个参数，不另开串口。仅在已连接、状态新鲜、
未武装且控制器未启用时发送（仅持有地面站租约不阻止只读同步）；每次连接前 10 秒内最多四轮，轮间至少 1 秒，收到对应值即停止请求。
断线重置重试状态；无回复不影响整表同步和缓存回退，READY 的实际参数类型/数值校验不变。
`priority_parameter_reads:=false` 可关闭优先读取；`parameter_request_topic` 默认
`/uas1/mavlink_sink`，`parameter_target_system` / `parameter_target_component` 默认 `1` / `1`，
自定义 MAVROS 路由或目标时必须同步配置。只读请求源为 `255/190`，应避免其他发送者复用该标识。
此路径发送无签名请求；要求签名且拒绝无签名参数读取的链路应关闭它，继续使用 MAVROS 原生同步。
编译新增显式 `libmavconn` 依赖（MAVROS 已依赖安装），用其标准 MAVLink 编码/转换接口；
升级源码后需重建 onboard_control，无需重建 MAVROS。
READY 使用单个持续订阅和 GNU `timeout` 的 120 秒相对定时器，不使用受校时影响的 Bash
`SECONDS`；同一条新状态必须满足全部条件。匹配状态或探针错误保存在该次日志目录的
`readiness.log`。取消启动时也会清理此订阅进程；超时后四组件继续受监督运行。

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
  bash start_onboard_control.sh --check
```

Odin 的现有 launch 文件同时启动 RViz。在无图形环境的纯 SSH 会话中，RViz 会因没有
`DISPLAY` 而退出，但 Odin 驱动、外部定位桥和其余飞行数据链仍可运行；需要 RViz 时应从
真机桌面终端启动。该无显示告警不影响其余三条数据链是否达到 `READY`，但仍应单独记录和处理。

分步入口现集中在 `start_drone/`，包含 `start_link.sh`、`start_mavros.sh`、
`start_odin.sh` 和 `start_extnav.sh`。两个硬件脚本会自动读取
`/etc/ros2-ardupilot/onboard.env`，因此可在两个终端中分别直接一键启动 Odin 和 extnav；
两者都不启动 MAVROS、onboard_control、修正或视频服务。
部署前备份与本次实机测试结论
记录在对应日期的任务报告中。

## 8. systemd 示例

`onboard-control.service.example` 是四组件生产模板，由
`install_onboard_service.sh` 自动替换用户、HOME 与工作区。当前 Jetson 已部署的
`ros2-ardupilot-onboard.service` 负责同一套四组件集成启动。

任务 22.5 的视频节点必须另外使用
`video_service/deploy/video-service.service.example`。不要把它加入
`start_onboard_control.sh` 的受监督子进程，也不要在 systemd 中对飞控服务声明
依赖。

任务 27 的修正节点同样使用独立的
`correction_service/deploy/correction-service.service.example`。在确认未解锁并人工停止飞控
主服务后，依次执行：

```bash
./correction_service/deploy/install_extnav_correction.sh
./correction_service/deploy/install_correction_service.sh
./start_onboard_correction.sh
./stop_onboard_correction.sh
```

extnav 安装器覆盖生产源前会创建带 SHA-256 的定点备份，只构建、不重启飞控；修正服务启动后
保持 idle、相机关闭。后两个根目录脚本分别用于前台启动和彻底停止该独立 unit；
停止修正节点不会清除 extnav 已应用的 active correction。完整接口、Tag 坐标约定和 clear 方法见
`correction_service/README.md`。`odin-correction.service` 不得对飞控 unit 设置
`Requires=`、`PartOf=` 或 `BindsTo=`。
视频 unit 也不得设置 `Requires=`/`PartOf=`：摄像头、FFmpeg、MediaMTX 或磁盘失败只能重启视频 unit，
不能清理飞行栈。飞机 aarch64 还必须把 MediaMTX v1.20.0 ARM64 安装到系统
`/usr/local/bin/mediamtx`；Git 仓库不再携带任何架构的 MediaMTX 可执行文件。
镜头配置应复制为 `/etc/ros2-ardupilot/lens.conf`，媒体目录 `/home/share/jpg` 需由
视频服务用户可写。模板中的 `ONBOARD_USER`、`ONBOARD_HOME_PATH`、
`ONBOARD_WORKSPACE_PATH` 必须全部替换。默认采集模式现为 H.264 1920×1080@60；完整依赖安装、
配置复制、ARM64 摘要校验、systemd 命令、QoS 和隔离验收步骤见 `video_service/README.md`。

服务只排在 `network-online.target` 之后，不依赖 `systemd-time-wait-sync.service` 或
`time-sync.target`。控制租约以地面站首个请求和后续有序心跳建立相对时间基准，机载安全超时使用
单调时钟，因此自带路由器没有外网时也必须能启动和作业；不得重新加入外网 NTP 完成门槛，也不要
用固定秒数 `sleep` 伪装就绪。若离线开机后再接入互联网，应在人工解锁前等待系统校时与 MAVROS
TIMESYNC 重新稳定，并重新核对 FCU、本地位置和推力语义。

## 飞控热重启（Task33，飞行接口 3.3）

地面站右上角“重启机载飞控”只在实机会话、机载在线、未解锁、新鲜落地状态、
待机且无任务时启用；确认框默认取消。机载端再次检查同样条件，最终通过 MAVROS
发送 `MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN`（246，param1=1，其余参数为0）。
不会解锁、起飞、恢复飞行任务或重启机载计算机。

无地面站时，在飞机工作区执行 `./reboot_fcu.sh`。脚本要求本机存在机载环境文件且正在运行
该工作区的 onboard_control，然后用项目 Python 客户端申请短租约、调用同一个 `FlightCommand`
并等待终态。如果地面站持有租约，脚本会拒绝，请先断开地面站。地面开发机直接执行会被拒绝。
机载 Python 环境须预先创建（已有环境不必重建）：

```bash
python3 -m venv --system-site-packages .venv
./reboot_fcu.sh
```

变更了 `ControlStatus` 的落地和重启状态字段，必须同步构建 `guided_interfaces` 和
`onboard_control`（版本 3.3.0），并更新地面站；独立视频接口仍为 3.2。新加入的 C++ 实现
位于 `src/onboard_control/src/fcu_reboot.cpp`，Python 脚本客户端位于 `src/onboard_control/scripts/reboot_fcu_client.py`。
构建不会使正在运行的旧进程自动升级；部署后在确认未解锁、落地且无任务时重启机载服务。

成功证据分两阶段输出到主 GUI 日志和脚本终端：

- **飞控重启成功**：ArduPilot `timesync_status.remote_timestamp_ns` 回退超过1秒且回到启动后
  30秒内。ACK 不是成功证据；ACK 丢失也不自动重发重启。
- **控制链路成功恢复**：未解锁、新鲜落地/状态、重启后的连续位置和速度、原点回读、
  新鲜 GUID_OPTIONS/MOT_THST_HOVER 参数事件、消息频率配置以及无输出发布冲突，持续满足2秒。
  消息配置 ACK 不表示实测速率达标，定位有数据不等于已完成实飞精度验收。

整个事务最多等待90秒。发送重启前先只读请求 GPS_GLOBAL_ORIGIN（最多等待3秒），
重启后也主动请求回读，不依赖飞控自发广播。只恢复此前由飞控回读的原点，不凭空填入默认坐标；无原点、无定位、
参数或遥测缺失时会明确报告恢复失败。机载节点持续运行期间，独立 FCU 重启也由相同启动时钟
证据触发恢复。该机制针对 ArduPilot 的启动相对时钟，不处理 USB 消失后重新枚举，不恢复旧任务。
