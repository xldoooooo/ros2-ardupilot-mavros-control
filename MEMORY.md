# 项目重要记忆

## 环境、入口与构建

- 开发机为 Ubuntu 24.04、ROS 2 Jazzy；已安装 MAVROS 与 ArduPilot SITL。
- 地面站入口仍为仓库根目录 `ground_station.py`；任务 04 已用 PySide6/Qt 6 完整替换 Tkinter。模块化界面位于 `ground_station_core/qt_ui/`，结构化日志位于 `ground_station_core/event_log.py`；没有浏览器/Web GUI。入口会在未手动 source 时自动加载 `/opt/ros/<distro>/setup.bash` 和本仓库 `install/setup.bash` 后原位重启，因此构建完成并安装 `requirements-gui.txt` 后可直接运行 `python ground_station.py`。
- 工作空间需构建 `guided_interfaces`、`onboard_control`、`guided_sim`：`source /opt/ros/jazzy/setup.bash && colcon build --packages-select guided_interfaces onboard_control guided_sim`。
- 仓库历史中的 `test_takeoff5.py <高度>` 走与 GUI 相同的高层机载协议，不含独立控制算法；任务 04 开始前该文件已在用户工作树中删除，本任务未恢复或改动该用户状态。
- `python ground_station.py --check-environment` 可在不导入或创建 Qt 窗口的情况下验证自动 overlay、`guided_interfaces` 和地面站 ROS 客户端线程。
- 外部仿真日志写入 `/tmp/ros2_ardupilot_ground_station/`。

## PySide6/Qt GUI（任务 04/05/07）

- 界面采用浅色低饱和工程主题；任务 05 删除大标题，把环境、机载链路、飞行器、控制模式、控制健康和最近动态压成单行状态带。主内容为连接/飞行动作、手动控制/遥测、航点左中右三栏同时显示，底部为可拖动实时日志。
- `ground_station_core/qt_ui/state.py` 是按钮状态的统一策略：只有显式完成环境连接且 ROS、飞控、租约、位姿、推力语义和发布者诊断通过时才开放相应飞控；LAND 作为安全动作保留较少门控。仿真或实机会话已建立时禁用“启动本地仿真/连接实机服务”（须先关闭/断开）；无会话时禁用全部航点编辑与发送组件。关闭入口拆为“关闭本地仿真”与“断开实机连接”，按 `connection_mode`/`pending_mode` 互斥启用（仿真会话只能关仿真，实机会话只能断实机）。原点齿轮在会话建立或环境工作流进行中禁用，仅空闲时可改本地缓存。
- 实机连接、真机起降/航点、仿真终止、实机断开、航点清空和任何退出均有默认取消的确认框；只有仿真会话的起飞、降落、发送航点按任务 10 要求免二次确认。退出与清理在后台执行，Qt 主线程不阻塞。飞控/EKF 原点齿轮只保存本地缓存：完整“连接实机服务”会申请控制租约、配置消息频率并写入该原点；齿轮右侧独立 Wi-Fi 图标只检测状态/日志通讯，不申请租约、不发命令、不管理进程，也不使用原点。**本地仿真禁止**写入 GUI 缓存原点——SITL 使用自身 Home（默认 CMAC 一带）建立 EKF，本地位姿应在米级；若把与 SITL Home 不一致的经纬高（如杭州默认缓存）写入，local ENU 会偏移数百万米，GUI 实际位姿与 RViz TF 会发散。
- 按钮角色样式：`primary`/`success`/`danger` 均有 hover 变深（`accent_hover`/`success_hover`/`danger_hover`），与中性按钮悬停反馈一致。
- `EventLog` 在事件产生处保存 DEBUG/INFO/WARN/ERROR、来源、时间和序号。SITL/MAVROS/机载/RViz stdout 会实时 tee 到同一日志和原磁盘文件；Qt 只筛选已有等级，不按文本猜测。四个等级现为独立复选框，可任意组合显示。
- 任务 07（日志部分保留）：`ProcessSupervisor._explicit_output_level` 将 SITL/MAVROS 等 chatty 源的启动刷屏（如 `Embedding file ...`、ROS `[INFO]` 插件加载）降为 DEBUG；生产仿真进程名是 `mavros_sim`，分类规则和测试必须直接覆盖该名称。显式 WARN/ERROR 不变。日志“自动滚动”关闭后必须保留视口位置，禁止 `setTextCursor(End)` 强制跳底。
- 原点 3 个、起降参数 3 个与航点 4 个数值框共 10 个控件统一禁止鼠标滚轮改值。任务 10 追加修复后，起降与航点共 7 个紧凑输入恢复窄型上下步进箭头，并显示 `m`/`m/s`/`°` 后缀；起飞参数跟随起飞按钮、降落速度跟随降落按钮、航点参数跟随航点编辑状态同步禁用和变灰。航点清空会停止 GUI 进度跟踪，避免机载端旧任务快照恢复已清空进度。
- 菜单栏仅“设置”“帮助”：设置为“显示实时日志”“恢复默认布局”；日志自动滚动/清空在日志面板工具栏操作，不在菜单重复。右上“在此处打开终端”通过 `QProcess.startDetached()` 在当前目录启动系统终端，其右侧为红色“退出地面站”入口。
- 完整顶层窗口使用 frameless + 透明留边 + `outerWindowFrame`，具有连续四边轮廓和 Qt 自绘阴影；最小化、最大化/还原、关闭、菜单空白拖动及四边/四角缩放。
- 确认、警告、帮助和关于统一使用 `ShadowMessageBox`：保留模态和默认取消语义，并添加自绘标题栏、关闭按钮、四边轮廓、圆角和阴影；不要重新使用静态 `QMessageBox.information/about/warning` 绕过统一外框。
- Qt 依赖为 `PySide6>=6.7,<7`；本机验证为 6.11.1。生产 Python 代码已无 Tkinter 或旧 `ground_station_core.gui` 引用。
- 1600×920 为默认尺寸，1180×700 为最小尺寸；三栏各自使用滚动区，工作区宽度和日志高度均可用 splitter 调整，设置菜单可恢复默认比例。

## 重构后的部署边界

- `src/guided_interfaces/` 是上位机与机载计算机唯一共享的 ROS 2 高层协议。`ExecuteWaypoints.flight_strategy` 改变了请求结构，接口与 `guided_interfaces`/`onboard_control` 包版本已同步升级为 `2.0`/`2.0.0`；旧 `1.0` 机载端必须在命令传输前被拒绝。该字段预留直线/自动避障/遇障悬停；当前机载仅实现直线飞行，其余值会告警并按直线执行。
- `src/onboard_control/` 是无 GUI 的机载 C++ 服务：控制权仲裁、起降编排、航点推进、失联保护、100 Hz PD+DOB、姿态/推力输出和 MAVROS 网关全部在此。
- `ground_station_core/ros_controller.py` 是薄客户端，只发布心跳/运动意图并调用高层服务；地面站不创建任何 MAVROS setpoint 发布器，也不保存安全关键的持续控制状态。
- `ground_station_core/environment.py` 的仿真路径会在本机启动 SITL、MAVROS、同款机载 C++ 节点和 RViz；实机路径只连接局域网中的远端机载服务，不启动或终止远端 MAVROS、Odin、extnav 或控制节点。
- `src/guided_sim/` 只保留 URDF、RViz 和 TF 可视化桥；旧的未启用 C++ 控制器与 Python PD+DOB/飞行模式副本均已删除，不能恢复成双实现。

## 控制与 ArduPilot 约束

- 上/下/左/右/前/后、偏航、悬停和航点最终全部进入同一个 C++ `DobController`，输出 `/mavros/setpoint_raw/attitude`；该话题只允许机载节点一个发布者。
- 运动按键是带来源、序号和 TTL 的速度增量意图。机载端累加/限速，并积分为有界虚拟位置参考；模式切换时明确抓取当前位置并重置 DOB。
- ArduPilot 必须设置 `GUID_OPTIONS` bit 3（值至少包含 `8`），使 `SET_ATTITUDE_TARGET.thrust` 表示真实归一化推力；未确认时机载端拒绝进入原始姿态/推力控制。
- 机载端同时读取飞控 `MOT_THST_HOVER` 并同步控制器悬停推力。SITL 实测约为 `0.3744`；旧硬编码 `0.2` 会在真实推力模式中持续掉高，禁止恢复。
- 控制器使用实测 `dt`，并对 DOB、加速度、倾角、推力、虚拟参考距离及非有限值做限幅/保护。100 Hz timer 位于独立回调组，多线程执行器避免状态图查询阻塞控制调度。
- 起飞由 ArduPilot GUIDED/arm/takeoff 完成安全离地，接近目标高度后切入同一 PD+DOB 并保持请求高度；LAND 交给 ArduPilot。

## 高层协议与安全状态机

- 同一时刻只有一个 `source_id` 可持有控制租约；地面站以 5 Hz 心跳续租。所有飞行输入共享单调序号并校验时间戳、TTL、重复及乱序。
- 租约丢失时，机载端立即抓取当前位置独立悬停；默认等待 10 秒仍未恢复则自动切换 LAND。地面站恢复租约后需显式发送悬停或新任务退出失联状态。
- 飞控状态或本地位姿/速度超时、非有限控制输出、姿态 setpoint 多发布者、飞行中推力语义校验失效会触发机载 LAND；外部切走 GUIDED 时停止发送 setpoint，避免争夺飞控模式。
- `ControlStatus` 聚合飞控状态、实际/目标位姿速度、模式、航点进度、租约、推力校验、发布者冲突和控制频率/抖动诊断；GUI 只展示该权威状态。
- 地面站 ROS 客户端默认只观察；本地 SITL 和完整实机连接工作流才会显式开启租约，避免 ROS 节点启动即自动取得控制权。独立 Wi-Fi 通讯检测始终保持默认观察态，只创建状态/日志订阅，不申请/续租、不发心跳或命令。
- ROS 2 DDS 当前按“可信同一局域网”设计。两端设置相同 `ROS_DOMAIN_ID` 和 `ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET`；多播不可靠时配置 `ROS_STATIC_PEERS`。本阶段没有加入跨网段路由、VPN、SROS2 身份或加密。

## 生命周期与部署文件

- 仿真依次启动 `sim_vehicle.py -v ArduCopter --param GUID_OPTIONS=8`、TCP 5762 MAVROS、`onboard_control_node` 和 RViz；MAVProxy stdin 必须保持打开。
- 机载启动入口为 `ros2 launch onboard_control control.launch.py`；`src/onboard_control/deploy/` 提供 systemd 与局域网环境示例，安装时必须替换 `WORKSPACE`。
- 上位机局域网变量示例为 `ground_station.env.example`，机载变量示例为 `src/onboard_control/deploy/onboard.env.example`。
- 清理只释放控制租约并结束本地仿真受管进程；历史残留匹配要求本项目独有节点或本地 SITL 端点参数，不能停止 ROS daemon 或任意其他 ROS 工作负载。

## 已验证基线（任务 03，2026-08-06）

- 修改前备份：`/home/nvidia/scq/projects/backups/ros2-ardupilot-sitl-hardware-pre-task03-20260806-175032.tar.gz`；SHA-256 `1ae274af2a8ec5e8a41e92527f2863583b86c39a686a0b79cb285648bedf7add`，gzip 校验通过。
- `colcon build --packages-select guided_interfaces onboard_control guided_sim` 成功；`colcon test-result --verbose` 为 5 tests、0 errors/failures；增加未 source 直接启动回归后 Python 为 7 passed。
- 在线协议注入通过：唯一租约、第二客户端拒绝、重复/乱序拒绝、过期命令拒绝、主动释放。
- 完整 SITL 通过起飞、前后左右上下、左右偏航、悬停、航点、失联悬停/恢复、LAND/解锁；六向与偏航方向检查均无失败。
- 5 秒悬停漂移 `0.039 m`，单航点三维误差 `0.089 m`，最终平均控制频率 `100.03 Hz`，setpoint 冲突为 false，清理 4 个受管进程且无残留。
- 独立失联试验通过：释放租约后立即机载悬停，10 秒宽限期后自主 LAND 并解锁。
- 非实时 Ubuntu 上完整飞行期间记录过 1 次 deadline miss、最大调度间隔抖动 `203.404 ms`；平均频率与飞行未受影响，但实机部署应继续做 CPU/调度隔离和长时间统计，不能宣称硬实时。
- 无真机可用，因此远端伴随计算机、物理飞控、Odin/extnav 和真实 Wi-Fi 端到端尚未验证；详细使用、调试、证据和限制见 `agent/report/report-2026-08-06.md` 的任务 03 章节。

## 已验证基线（任务 04，2026-08-06）

- PySide6/Qt 自动回归为 14 passed；覆盖结构化日志、子进程实时 tee、状态门控、危险确认、输入焦点、航点和 900×650/1600×1000 布局。
- `colcon build` 三包成功，`colcon test-result --verbose` 为 5 tests、0 errors/failures；环境诊断、compileall、flake8 F 类和 `git diff --check` 均通过。
- 真实 Qt 按钮信号驱动完整 SITL：初始化、起飞、前后左右上下、左右偏航、悬停、单航点、LAND、清理和异步退出全部通过；本轮统一日志 642 条，WARN 精确筛选 12 条。
- 航点结束三维误差约 `0.041 m`，控制频率约 `100.18 Hz`，setpoint conflict 为 false；记录 1 次 deadline miss、最大抖动约 `5.09 ms`，不能宣称硬实时。
- 清理后无 SITL、MAVROS、onboard_control、guided_sim 或 RViz 残留。详细设计、测试、使用和限制见 `agent/report/report-2026-08-06-task04-qt.md`。

## 已验证基线（任务 05，2026-08-06）

- Python 全量回归 18 passed；覆盖七个数值框禁用滚轮、日志多等级组合、三栏最小/放大布局、旧机载进度隔离、菜单/终端参数和窗口阴影。
- `colcon build` 三包成功，`colcon test-result --verbose` 为 5 tests、0 errors/failures；compileall、flake8 致命错误/行长检查和 `git diff --check` 均通过。
- 真实 Qt→SITL 通过环境初始化、起飞、单航点、清空旧 `1/1` 进度、LAND/解锁与清理；清空后 GUI 保持 `尚未执行`，即使机载快照仍为 `1/1`。
- 真实回归停止 4 个受管进程且无残留；控制频率 100.00 Hz，记录 1 次 deadline miss 和最大抖动 323.31 ms，不能宣称硬实时。offscreen 环境下 RViz 因无显示后端退出，不影响控制链路。
- 任务 05 详细改动、截图、构建缓存处理和限制见 `agent/report/report-2026-08-06-task05-beautify.md`。

## 窗口外框追加验证（2026-08-07）

- 主窗口阴影已从中央内容卡片迁移到完整 `outerWindowFrame`；最大化自动取消阴影留边，还原后恢复。Qt 全量回归为 20 passed。
- 所有当前消息提示入口统一为 `ShadowMessageBox`，测试覆盖标题、关闭按钮、默认 Cancel、430×180 最小可读面积和独立阴影表面。
- `colcon test-result --verbose` 仍为 5 tests、0 errors/failures；环境诊断、compileall、flake8 致命错误/行长检查和 `git diff --check` 通过。
- 视觉证据为 `/tmp/task05-main-outer-shadow-v2.png` 和 `/tmp/task05-dialog-shadow-v2.png`；详细说明见 `agent/report/report-2026-08-07-window-shadow.md`。

## 已验证基线（任务 07 日志部分 + 终端撤销，2026-08-07）

- 任务 07 的集成终端已按用户反馈撤销；外部终端入口保留为右上“终端”按钮。
- 保留：SITL/MAVROS 启动刷屏降为 DEBUG；日志面板内“自动滚动”可关闭且不强制跳底。
- Python 回归应覆盖外部终端入口、自动滚动关闭、Embedding 降 DEBUG；详见当日撤销报告。

## 审计问题修正基线（2026-08-08）

- 航点请求结构对应的协议/包版本已统一升级为 `2.0`/`2.0.0`；回归确认旧 `1.0` 状态在访问 ROS 传输实体前即被拒绝，并校验 Python、C++ 与两个包版本保持同步。
- `mavros_sim` 已纳入 chatty 源；对真实历史日志重放的 410 条 ROS `[INFO]` 全部归类为 DEBUG，显式 WARN 仍为 WARN。
- 原点对话框、环境卡、摘要和仿真按钮 tooltip 已统一说明 SITL 使用自身 Home；完整实机连接继续写入缓存原点，独立 Wi-Fi 通讯检测不读取或写入原点。
- Python 全量为 27 passed；三包构建成功；ROS/C++ 为 5 tests、0 errors/failures；环境诊断、修改范围 flake8、compileall 和 `git diff --check` 通过。

## 任务 08 机载最小部署基线（2026-08-08）

- 真机为 Ubuntu 22.04/Humble/aarch64；新工作区固定为 `/home/onboard/ros2-ardupilot-mavros-control`，父目录保持 `root:root`，仓库目录为 `xld:xld`。使用 partial clone + non-cone sparse checkout，只检出 `src/guided_interfaces/` 与 `src/onboard_control/`；禁止把地面站、`guided_sim` 或开发机 build/install 复制到机载端。
- `src/onboard_control/deploy/onboard_workspace.sh` 提供 `update/deps-check/build/test/smoke/verify`。默认依赖检查不调用 apt/sudo/rosdep 初始化；Humble 烟雾测试固定使用非零 domain 231、`ROS_LOCALHOST_ONLY=1`、独立 `/_task08_smoke_mavros` 前缀，不发送命令，并验证接口 2.0、FCU 未连接、未武装以及 2 秒窗口内零姿态 setpoint。
- Humble 编译暴露 `Clock::now()` const API 差异；已改用 Humble/Jazzy 都支持的 `Node::now()`，不改变控制逻辑。最终真机 Release 构建成功，5 tests、0 failures；aarch64 动态库在 source Humble + overlay 后全部解析，隔离烟雾通过且无进程残留。
- `/home/xld` 旧仓库保持 `dad9067` 且工作树干净，`odin.sh`、旧 `odin1.sh`、`start_mavros_real.sh` 的部署前后 SHA-256 一致；未安装/启用 systemd，旧启动流程继续作为回退。
- 真机对 GitHub HTTPS 直连会超时；部署文档提供只在当前 SSH 会话存活的 localhost 反向动态 SOCKS 隧道。不得把代理永久写入 `/home/xld`、系统环境或服务配置。
- 本任务没有启动真实 MAVROS/Odin/extnav，没有连接飞控、申请租约、写原点、设置消息频率、解锁或起飞。Humble/Jazzy 跨发行版 DDS 不受 ROS 官方保证，仍需按只读 DDS→MAVROS 只读→同机状态链的顺序做独立台架验证，不能据本轮结果宣称可实飞。

## 任务 09 真机通信基线（2026-08-08）

- 真机 domain 42 端到端实测完成，全程 `armed=false`、非 GUIDED、零租约、命令序号 0；没有模式、解锁、起飞、原点、消息频率或飞行输入调用。旧 `odin.sh` 会主动设置三种消息频率，本任务未执行；只单独启动 MAVROS 串口连接，未启动 Odin/extnav。
- 无 MAVROS 与连接飞控两个 60 秒窗口均收到 601/601 条 `/onboard_control/status`，估算送达率 100%；最大接收间隔分别约 104.43 ms 与 110.39 ms，字段无语义错误。硬件守护窗另收 300 条聚合状态、29 条原始 `/mavros/state`，全部 `armed=false`/`guided=false`/`STABILIZE`，姿态 setpoint 为 0 条。
- 任务执行时临时只读改造的 GUI 链路探针通过：不打开控制、命令结果 0、全部飞行按钮禁用；远端新鲜 `/rosout` 与 `ControlStatus.status_message` 都能逐字进入 GUI 日志。随后按用户澄清把该能力抽离为齿轮右侧独立 Wi-Fi 图标，原“连接实机服务”恢复完整功能；旧报告中“连接实机按钮永久只读”的产品语义已被本次调整取代。`/rosout` 历史样本有 10 秒 lifespan，不能把稳定节点近期无新日志误判为链路失败。
- 新 Wi-Fi 按钮使用生产 `GroundStationRosController`/`EnvironmentInitializer` 在隔离本机 domain 231 做了真实 Qt→ROS 集成验证：3 秒收 30 条状态（9.98 Hz）、最大接收间隔 100.2 ms、远端日志 1 条；环境保持 `none`、控制未开启、租约/命令结果均为 0，飞行按钮全禁用，测试后无进程残留。完整实机连接的恢复仅由替身回归验证其调用 `enable_control`、`set_rates`、`set_gp_origin`，未对真机执行该有状态路径。
- 调整后 Python 全量 37 passed；三包构建成功；ROS/C++ 为 5 tests、0 errors/failures；环境诊断、离屏视觉核对、compileall、修改范围 flake8 与 `git diff --check` 均通过。新报告为 `agent/report/report-2026-08-08-task09-wifi-communication-adjustment.md`。
- 2026-08-09 已在真机伴随计算机上点击调整后的真实 Qt Wi-Fi 按钮复验：两次检测均收 30 条状态（9.98 Hz），最大接收间隔分别 102.2/101.0 ms；第二次逐字接收真机 Humble 端安全发布的 8 条测试 rosout。全程 `armed=false`、`environment=false/mode=none`、`control_enabled=false`、无控制权/租约/命令结果，飞行按钮全禁用。为隔离按钮职责只启动新 `onboard_control_node`，未启动 MAVROS/Odin/extnav，故结果正确报告 FCU 未连接；未点击正式“连接实机”。结束后进程零残留、systemd inactive、串口无人占用、两套真机仓库仍干净。详见 `agent/report/report-2026-08-09-task09-real-wifi-button-validation.md`。
- Humble `rmw_fastrtps_cpp 6.2.10` 与 Jazzy `8.4.3` 发现部分跨发行版端点时反复报告 `sequence size exceeds remaining buffer`；Jazzy 图中 Humble 发布者节点名未知且 type hash 为 `INVALID`。已测数据路径稳定不等于整个 ROS 图兼容，统一发行版/DDS 或受支持桥接并复验前禁止据此实飞。
- 测试后真机 MAVROS/onboard/Odin/extnav 零残留，串口无占用，systemd 仍 inactive；旧仓库 `dad9067`、新仓库 `c8abad9`、工作树与三个旧脚本哈希均保持不变。详细证据见 `agent/report/report-2026-08-08-task09-communication.md`。

## 任务 10 UI 精修基线（2026-08-09）

- 卡片长说明统一收进标题右侧圆形问号；日志默认隐藏 DEBUG，并删除“等级”标签。右上角依次为“在此处打开终端”、红色“退出地面站”和窗口控件；任何退出均须默认取消的二次确认。
- Wi-Fi 纯订阅检测运行时按钮改为红色终止方块。专用取消只设置当前 `communication` 工作流的取消事件，不复用会释放租约或终止进程的通用清理路径；取消完成按 WARN 展示。
- 仿真会话的起飞、降落、发送航点不再二次确认；真机对应操作及其他危险确认保持。起飞 `2.5 m/s`、降落 `0.5 m/s` 是依据本机 ArduPilot `WP_SPD_UP_DEFAULT`/`LAND_SPD_MS_DEFAULT` 的 **UI 预设**，按用户要求没有加入 ROS 协议、机载 C++ 或飞控参数写入，不能宣称已影响实际速度。
- 航点编辑采用 XYZYaw + 白色“+”单行、28 px 输入/表头/正文行高、上/下/红色减号/清空操作。策略选择不再依赖会被桌面样式裁切的原生 combo popup，而使用固定移到控件下方的三 action `QMenu`，一次完整显示三项且没有滚动视口；发送按钮为蓝色，航点进度 chunk 为绿色。
- 同一航点任务的后续 RUNNING 结果不得重新显示“等待机载任务进度”；只有发送新任务时才重置进度，从而兼顾防闪烁和新旧任务隔离。
- Python 全量为 44 passed；三包构建成功；ROS/C++ 为 5 tests、0 errors/failures；环境自检、compileall、致命级 flake8、修改范围 `git diff --check` 通过。初版说明见 `agent/report/report-2026-08-09-task10-refine-2.md`；箭头/单位、完整策略菜单、“+”与输入门控追加修复见 `agent/report/report-2026-08-09-task10-refine-2-followup.md`，视觉证据为 `agent/codex/task10-refine-2-followup-minimum.png` 和 `task10-refine-2-strategy-menu.png`。

## 任务 11 手动操作模式 UI 基线（2026-08-09）

- 主工作区由三栏改为两栏：左侧组合操作区、右侧航点区。左侧第一排为“环境与连接”和“飞行动作”左右并排，第二排“手动操纵”横跨整行；1180×700 最小窗口通过既有纵向滚动访问堆叠内容。
- 手动操纵按美国手排列成两个十字键组：左组 W/S 升降、A/D 左右偏航，右组 I/K 前后、J/L 左右平移；Space 悬停独立居中。卡片内旧灰色快捷键说明已删除，实际位姿、目标速度/位姿、控制周期和安全门控位于左下。
- 本次仅调整 Qt 组件层次与布局，八个运动按钮的 `vx/vy/vz/yaw_rate` 增量映射不变；未修改 ROS、C++ 控制、协议或飞控逻辑。
- 加载 Jazzy 与本仓库 overlay 后 Python 全量为 45 passed；任务 11 定向回归为 2 passed。compileall、致命级 flake8 和修改范围 `git diff --check` 通过。视觉证据为 `agent/codex/task11-mode-main.png` 与 `agent/codex/task11-mode-minimum.png`，详细说明见 `agent/report/report-2026-08-09-task11-mode.md`。

## 任务 12 手动操纵美化基线（2026-08-09）

- 左右十字按键现位于独立圆角方形摇杆底盘，中央圆盘实时显示按压偏移；鼠标按住或键盘触发时按钮蓝色高亮，结束后回中。左/右底盘分别贴近卡片两侧，中间保留间隔。
- 两个摇杆各有低 `0.5×`、中 `1.0×`、高 `2.0×` 独立灵敏度，默认 `1.0×` 保持 `VELOCITY_SCALE=0.2` 的原基准。左倍率作用于升降/偏航，右倍率作用于前后/左右；鼠标与键盘必须共用 `OperationsPanel.trigger_motion()`，不得恢复两套硬编码命令。
- 手动 XY 默认使用“机体坐标”：GUI 依据最新 `VehicleSnapshot.yaw` 把每次右摇杆机体增量旋转为本地 ENU 后沿既有 `MotionIntent` 发送；可切换“本地 ENU”直通。协议与机载 C++ 未变化，机载仍累计 ENU 增量。
- 手动区常驻显示坐标系、控制权和最近手动指令年龄；“悬停”已改为“制动并悬停  SPACE”。主遥测为实际高度/航向和目标水平/升降/偏航大数字摘要，完整 XYZ、目标位姿、jitter、deadline miss、安全门控默认折叠在“工程信息”。
- “环境与连接”和“飞行动作”卡片内部显式顶对齐，标题左上角一致。任务未包含且未实现手动输入锁、ARM/DISARM、RTL、自动回中、输入超时或游戏手柄。
- Qt 全量为 24 passed；加载 Jazzy 与 overlay 后 Python 全量为 46 passed；compileall、致命级 flake8、修改范围 `git diff --check` 与新增差异行长度检查通过。视觉证据为 `agent/codex/task12-beautify-2-main.png`、`task12-beautify-2-active.png`、`task12-beautify-2-expanded.png` 和 `task12-beautify-2-minimum.png`，详细说明见 `agent/report/report-2026-08-09-task12-beautify-2.md`。
- 任务 12 追加修正：坐标系实际切换会写入 `INFO/operator` 日志并说明机体旋转或 ENU 固定轴语义；坐标系与左右灵敏度三个选择器统一复用航点策略的 `DownwardComboBox`，完整菜单固定向下显示且无裁切。全量仍为 46 passed，视觉证据为 `agent/codex/task12-coordinate-menu.png`，详见 `agent/report/report-2026-08-09-task12-coordinate-dropdown-fix.md`。
- 任务 12 第二次追加调整：坐标系与左右灵敏度选择器现和手动运动按钮共用 `UiAvailability.motion` 门控，禁用时显示统一 `flight_reason`，恢复后还原原始 tooltip；双摇杆布局使用响应式外侧留白、可伸缩底盘和固定 24 px 中缝。1600×920 时两侧留白各 101 px，1180×700 时约 35 px，不再贴边或形成过大中央空洞。全量仍为 46 passed，详见 `agent/report/report-2026-08-09-task12-manual-control-gating-responsive-spacing.md`。

## GUI 窗口性能优化基线（2026-08-09）

- 保留 frameless、透明留边、完整阴影、全部 QSS、控件尺寸与布局；没有改用原生标题栏、取消阴影、冻结界面或降低渲染分辨率。`GroundStationWindow._refresh()` 仍以 10 Hz 读取 ROS 权威快照，但只有门控显示签名变化时才重新应用按钮/tooltip/icon 状态；遥测和状态文本仅在格式化结果变化时写入。
- 仅 `centralRoot` 使用 `WA_OpaquePaintEvent`，其区域始终被 `windowSurface` 完整覆盖。面板、viewport、圆角卡片和阴影外框禁止使用该提示；试验中这些区域会因跳过背景清除而变黑，相关方案已撤销并加入回归约束。
- 本机 X11、5120×2880、DPR 2.0、1228×924 逻辑窗口的程序化 resize 从约 83.80 ms/次降至 63.96 ms/次；稳定预热后的 10 次刷新为 0 次 Paint、0 次 ToolTipChange。2456×1848 修改前后截图除日志时间戳外逐像素一致。
- Qt 定向回归 26 passed、Python 全量 48 passed；三包构建成功，ROS/C++ 为 5 tests、0 errors/failures；环境自检、compileall、致命级 flake8 和修改范围 `git diff --check` 通过。详见 `agent/report/report-2026-08-09-gui-window-performance-optimization.md`。

## 航点执行区压缩追加基线（2026-08-09）

- “发送并执行航点”上方的灰色状态说明已从布局和绘制中移除；原 `status_label` 仍作为不可见状态接收器，保持航点新增、清空、运行结果的更新链与内部接口不变。
- 1228×924、1600×920 和 1180×700 三种尺寸下，执行卡高度均由 148 px 降为 118 px，航点表格高度均增加 30 px；进度条、策略选择、发送门控和结果处理均未改变。
- Python 全量回归 48 passed；三包普通模式构建成功，ROS/C++ 为 5 tests、0 errors/failures；环境自检、compileall 和致命级 flake8（88 字符项目标准）通过。详见 `agent/report/report-2026-08-09-waypoint-execution-panel-compression.md`。

## 版本库卫生

- `.gitignore` 排除 Python/colcon 产物、rosbag、ArduPilot/MAVProxy 日志、EEPROM、飞行记录和通用临时文件。
- `.deep-copilot/` 运行数据不应重新提交；本机 `AGENTS.md` 保持本地。
- 仓库不维护 `README.md`；正式执行与部署说明以当日报告、配置注释和 launch/deploy 示例为准。
