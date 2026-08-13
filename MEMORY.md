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
- 地面站 GUI 默认保持 `ROS IDLE`，单纯打开窗口不会创建 DDS participant；启动仿真、完整实机连接或独立 Wi-Fi 检测时才在后台按需启动 ROS。ROS 启动后仍默认只观察，只有本地 SITL 和完整实机连接显式开启租约；Wi-Fi 检测只创建状态/日志订阅，不申请/续租、不发心跳或命令。
- ROS 2 DDS 当前按“可信同一局域网”设计。未设置 `ROS_DOMAIN_ID` 时为 domain 0；42 只是一种可选隔离值，必须在 MAVROS、Odin、extnav、onboard、地面站和诊断 CLI 启动前一致应用，不能只改一个终端或用它修复跨发行版兼容。Jazzy 使用 `ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET`；Humble 的发现配置需按其实际 RMW 验证。本阶段没有加入跨网段路由、VPN、SROS2 身份或加密。

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
- `src/onboard_control/deploy/onboard_workspace.sh` 提供 `update/deps-check/build/test/smoke/verify`。默认依赖检查不调用 apt/sudo/rosdep 初始化；Humble 烟雾测试固定使用非零 domain 231、`ROS_LOCALHOST_ONLY=1`、独立 `/_task08_smoke_mavros` 前缀，不发送命令，并验证当前接口版本、FCU 未连接、未武装以及 2 秒窗口内零姿态 setpoint。
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

## 2026-08-10 当前实机链路与目录清理基线

- 当前手工启动的实机 MAVROS、Odin、extnav、onboard 与开发机地面站均未设置 `ROS_DOMAIN_ID`，实际同在 domain 0；仅在新 CLI 设置 42 会进入空的隔离图。历史 domain 42 测试不代表当前运行配置。
- 实机当前 `GUID_OPTIONS=8`、`MOT_THST_HOVER=0.20921991765499115`。连接后约 35.6 秒才完成 MAVROS 参数表同步，旧“无法同时读取”日志是启动暂态；同步后 `thrust_mode_verified=true`。机载端现延后 40 秒首次检查、60 秒后才报同步超时，地面站等待门同步扩为 60 秒，相同状态日志去重。
- 当前拆分启动遗漏旧 `odin.sh` 的 message 32/31/105 频率请求；本次运行的 8 秒、20 秒和 45 秒窗口内 `/mavros/local_position/pose` 均为 0 Hz，且 `message_rates_configured=false`。该字段只表示当前 onboard 实例是否完成自己的设置序列，不是飞控 interval 的读回，当前 0 Hz 结论来自直接采样。历史仿真与完整实机连接工作流在飞行就绪前本来就调用 `request_set_rates()`，旧 `odin.sh` 也设置三路；新的自动逻辑是把既有前置条件从地面站/脚本迁到机载节点，不是首次加入。用户曾观察到打印 buffer error 后仍收到 pose，可能是当时 message 32 已由这些路径或 ArduPilot `SRx_*` stream 配置。自动逻辑消除启动顺序不确定性，不修复跨发行版 DDS。
- 追加复核中，Jazzy 只读 participant 存在时 8 次新建 echo 有 6 次打印 buffer error、2 次不打印，说明错误本身具有发现时序随机性；Humble 长订阅在一次 buffer error 后仍稳定收到 `/mavros/state` 43 条/45 秒，证明它不等于 executor 或整个通信链路阻塞。当前无 pose 在 Jazzy participant 加入前已存在；要判断 DDS 是否让某些 local-pose 新订阅者偶发匹配失败，必须在确认 message 32 持续发布的正样本窗口做同机长订阅与重复新订阅对照。
- 地面站开启期间 10 秒 UDP 仅 68,444 字节（地→机 12,086，机→地 56,358），socket 队列为 0、5 秒 UDP error 增量为 0；“消息太多造成阻塞”不符合证据。`sequence size exceeds remaining buffer` 与 Jazzy/Humble Fast DDS discovery/type 兼容问题一致，domain 与增大 buffer 均非根治；生产方案仍须统一发行版/RMW 或受支持桥接。
- 已把无引用的 `shfiles/`、旧 SITL `logs/`、tlog/raw、terrain、EEPROM、parm、两个旧 CSV、FishROS 临时脚本和 Python 缓存移入系统回收站，约 169 MiB；删除 5 个 `guided_sim`/`run` 空目录。保留标准 colcon `build/install/log`、明确资源目录和用户既有工作树改动。
- 本地验证为 Python 50 passed、ROS/C++ 5 tests 零失败、隔离消息频率重连通过；实机全程只读、`armed=false`、STABILIZE、零租约/零姿态 setpoint，未部署或重启新二进制。完整记录见 `agent/report/report-2026-08-10-domain-parameter-dds-cleanup.md`。

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
- 所有 `*.mp4` 视频录屏作为本地证据文件统一忽略，禁止直接纳入 Git 历史；需分发大视频时应使用外部存储或经明确评估后使用 Git LFS。
- `.deep-copilot/` 运行数据不应重新提交；本机 `AGENTS.md` 保持本地。
- 仓库不维护 `README.md`；正式执行与部署说明以当日报告、配置注释和 launch/deploy 示例为准。

## 2026-08-10 真机一键启动部署基线

- 真机 `/home/onboard/ros2-ardupilot-mavros-control/start_drone_all.sh` 已集成 MAVROS、Odin、extnav、
  `onboard_control_node`；使用
  `cd /home/onboard/ros2-ardupilot-mavros-control && bash start_drone_all.sh` 一行启动，默认 domain 0，
  `Ctrl+C` 整组退出，日志位于 `/tmp/ros2_ardupilot_onboard/<时间戳>/`，已有任一实例时拒绝
  重复启动。
- 真机 Humble Release 构建、5 个 ROS/C++ 测试和隔离烟雾通过；一键脚本两轮均到达
  `READY`。实测 local pose 93.72 Hz、onboard status 10.00 Hz、vision pose 39.99 Hz，参数
  `GUID_OPTIONS=8`、`MOT_THST_HOVER=0.20922`，全程 `armed=false`、STABILIZE、零租约、零姿态
  setpoint。最终四组件全部停止且串口无占用。
- 原四个分步脚本现集中在 `start_drone/`，文件名为 `start_link.sh`、`start_mavros.sh`、
  `start_odin.sh`、`start_extnav.sh`。部署前备份为
  `.deployment-backups/pre-integrated-start-20260810-205108.tar.gz`，SHA-256
  `ea0ec5bdde38560abf2f47ae594a04a082d16d45879b7029bfde1585d8ab027b`；最终
  `start_drone_all.sh` SHA-256 为
  `4b8a3307a71fecb84904df5374b90d1d54309b18181d5a5f1cdece4b5c624452`。
- 纯 SSH 无 `DISPLAY` 时 Odin launch 的 RViz 子进程退出，但 Odin 数据链仍通过就绪检查；
  真机桌面终端可正常启动 RViz。Jazzy 客户端接入 Humble 图仍触发 Fast DDS
  `sequence size exceeds remaining buffer`，数据在告警期间继续到达；该跨发行版问题与
  MAVLink 频率配置是两个独立问题。详细记录见
  `agent/report/report-2026-08-10-real-deployment-integrated-start.md`。
- 本地新增 `start_ground_all.sh`，一行启动完整 Qt 地面站；它只加载 Jazzy、本地 overlay
  和项目 Python，默认 domain 0/子网发现，并允许透传 `--check-environment`。隔离 domain 231
  环境自检通过。该文件明确只属于地面仓库，不同步到无人机。
- 真机调整后的 `start_drone_all.sh` 与 `start_drone/` 四个分步脚本已按原哈希同步回本地；
  本地旧 `start_all.sh`、`start_drone.sh`、`start_ground.sh` 已移入回收站。真机复核不存在
  `start_ground_all.sh`。同步完成回归为 Python 52 passed，详见
  `agent/report/report-2026-08-10-start-layout-ground-launcher-complete.md`。

## 2026-08-10 地面站仿真/实机隔离与退出修复基线

- 地面站 GUI 默认保持 `ROS IDLE`，单纯打开窗口不创建 DDS participant。完整本地仿真固定
  使用 domain 231 + `LOCALHOST`，并显式传给 SITL、仿真 MAVROS、仿真 onboard 和 RViz；
  真机仍使用 domain 0 + `SUBNET`，机载端无需设置或修改 domain。
- 同一 `GroundStationRosController` 使用独立 `rclpy.Context`/executor 动态切换 domain；切换
  前释放租约、停止旧 context、拒绝旧队列命令。仿真时隐藏 `ROS_STATIC_PEERS` 和
  `ROS_DISCOVERY_SERVER`，切回真机时恢复 GUI 启动值，因此仿真不会沿静态发现配置接触真机。
- 多个 `/onboard_control/status` 发布者会触发 `endpoint_conflict`，租约、心跳和全部命令故障
  关闭；机载进程重启后，过期的本地 grant hint 会清除并重新申请，避免无法恢复控制租约。
- `ProcessSupervisor` 在组长退出前保存 PGID/后代 PID，并对保存目标完成 INT/TERM/KILL；日志
  线程仍阻塞时不再跨线程关闭 `TextIOWrapper`。并发清理被合并且有 15 秒等待上限，退出流程
  不再先 join 后二次清理，清理期间所有环境入口禁用。
- 全量 Python 60 passed；真实本地 SITL 在 domain 231/LOCALHOST 达到 onboard、FCU、租约和
  local pose 就绪，始终 `armed=false`，结束后 4 个受管进程全部退出且 TCP 5762/项目残留为空。
  本任务未连接/修改真机，未改机载运行逻辑。详见
  `agent/report/report-2026-08-10-ground-simulation-domain-isolation-cleanup-fix.md`。

## 2026-08-10 任务 12.5 仿真/实机切换追加修复

- “断开真机”与“终止仿真”现在都是完整会话边界：释放租约、清理本机受管进程后立即停止
  `GroundStationRosController` 并销毁当前 DDS context，IDLE 不保留旧 participant。此前真机
  domain 0 context 会留到下一次点击仿真才销毁，导致跨 Humble/Jazzy 的端点撤销告警被误认为
  仿真流量影响真机。
- 同一 GUI 的 `source_id` 保持稳定时，租约 acquire/heartbeat/release 序号也在控制器生命周期内
  跨 context 单调递增；不得恢复为每个 `_spin()` 从 0 开始，否则真机→仿真→真机后会被机载端
  拒绝为“重复或乱序”。
- 仿真继续固定为 domain 231 + `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`，并清除显式
  `ROS_STATIC_PEERS`/`ROS_DISCOVERY_SERVER`。Wi-Fi 出站方向抓包为 0 包；真机栈运行时 8 秒
  仿真-only context 窗口也无远端新增输出。
- 固定同一 source 的未武装真机实测：首次租约序号 2/释放 3，经仿真 context 后以 6 重连、7
  释放成功；最终 `armed=false`、STABILIZE、零租约，3 秒姿态 setpoint 为 0。Python 62 passed，
  三包构建成功，ROS/C++ 5 tests 零失败。直接 Humble/Jazzy 真机连接/销毁仍可能触发 Fast DDS
  反序列化告警，这是 ROS 官方不保证的跨发行版边界，不是仿真 domain 231 出站；详见
  `agent/report/report-2026-08-10-task12.5-simulation-hardware-switch-bugfix.md`。

## 2026-08-11 机载启动文件更新范围

- 机载 sparse checkout 与 `onboard_workspace.sh update` 必须同时维护
  `src/guided_interfaces/`、`src/onboard_control/`、根目录 `start_drone/` 和
  `start_drone_all.sh`；`start_ground_all.sh` 仅属于地面端，禁止同步到无人机。

## 2026-08-11 任务 13 健壮性精修基线

- W/A/S/D、I/J/K/L 与 SPACE 具有“发送期望量/接管悬停”语义 tooltip 和无障碍描述；门控
  禁用时优先显示安全原因，恢复后还原动作说明。“工程信息”已更名“详细状态”，左栏保留
  既有诊断，右栏随左右灵敏度实时显示 W/S、A/D、I/K、J/L 单次增量。默认分别为
  `±0.20 m/s`、`±11.5 °/s`、`±0.20 m/s`、`±0.20 m/s`。

- 仿真启动并行检查四个 ROS 包，SITL 启动后立即预热 RViz，TCP 可用后并行初始化 MAVROS
  与 onboard；已有 ArduCopter SITL 二进制时使用 `--no-rebuild`。生产/实机飞控参数首次检查
  仍默认 40 秒，仅仿真命令覆盖为 2 秒。真实未武装 SITL 就绪从 52.866 秒降至 43.487 秒
  （快 9.379 秒/17.74%），RViz 提前 49.375 秒；剩余关键路径是 EKF 模拟时间稳定，禁止跳过
  local pose/EKF 安全门。
- MAVROS command 520 是能力广播请求的有效 accepted ACK 被 command 插件记账逻辑误报为
  unexpected；`STAT_RUNTIME (65535/...) different index` 是无索引运行时更新随后遇到完整参数表
  真实索引。SITL/真机均复现且能力/参数表随后完成，无实际影响，不应修改或粗暴屏蔽。
- 地面站重启机载栈后显示启动日志来自实机会话对全局 `/rosout` 的 reliable +
  transient-local 订阅，不是 SSH 终端镜像；实测恢复时收 190 条远端 ROS 启动事件并重新上线。
  后续应对白名单、历史补发标记、去重和等级做治理。
- 当前握手已有接口版本、状态新鲜度、单发布者检测、source/sequence 租约与心跳、服务接受及
  reliable final `CommandResult`；后续优先补 boot/session id、状态序号、每票 final deadline、
  header 时间校验与 applied-state 读回。起飞/航点有遥测确认，LAND、消息频率和 GPS 原点仍
  只证明模式请求接受/服务 ACK/topic 发布，日志必须区分 accepted/applied/observed。
- Roll/pitch 可由机载已订阅的 pose 四元数解算；电池来自 MAVROS `/mavros/battery`；通信频率
  必须分 GCS↔onboard、onboard 输入流与 MAVLink 链路三个层次。后续都应先加入机载聚合
  `ControlStatus`，不让 GUI 绕过接口直订阅更多 MAVROS topic。
- 最终 Python 65 passed，本地三包构建与 ROS/C++ 5 tests 通过；Humble/aarch64 临时两包构建、
  5 tests 和隔离默认参数 smoke 通过。实机两轮和被动重启观察始终 `armed=false`，最终服务
  inactive、飞行进程零残留、串口无人占用。详见
  `agent/report/report-2026-08-11-task13-refine-3.md`。

## 2026-08-11 Ubuntu 22.04/Humble 可移植部署入口

- `setup_project.sh` 现在从当前检出自动完成发行版选择、项目 `.venv`、PySide6、三包 Release
  构建、ROS/C++ 测试与环境检查；Ubuntu 22.04 优先 Humble，正常部署不再填写开发机路径。
- `start_drone/runtime_common.bash` 统一发现 ROS、项目 Python、Odin/extnav ament overlay 和唯一
  FCU 串口；`start_drone_all.sh --check` 只输出发现结果而不启动组件。多个串口或多个 overlay
  时必须安全失败，禁止为了“全自动”猜测真实飞控。
- 地面站及四个机载分步脚本已移除 `/home/nvidia`、`/home/xld`、`/home/onboard`、固定
  `/opt/ros/{jazzy,humble}` 和 `/dev/ttyTHS1` 运行依赖。ArduPilot SITL 会搜索 PATH 与常见
  源码布局；Humble DDS 切换使用 `ROS_LOCALHOST_ONLY`，Jazzy 继续使用自动发现范围变量。
- 自动入口在当前 24.04/Jazzy 开发机完整执行成功；Python 73 passed，三包 Release 构建成功，
  ROS/C++ 5 tests 零失败，干净环境地面站检查通过。没有连接或操作实机；22.04/Humble 仍需在
  师兄目标机执行 `setup_project.sh` 和无桨 `start_drone_all.sh --check` 复验。详细见
  `agent/report/report-2026-08-11-ubuntu2204-humble-portable-deployment.md`。

## 2026-08-11 详细状态与真实终态确认

- `ControlStatus` 2.1 新增 pitch 与电池有效性/电压/电流/百分比；机载从已有 pose 与 MAVROS
  BatteryState 聚合，地面站只消费权威状态。GUI“详细状态”增加俯仰/偏航、最近 5 秒实际
  ControlStatus 到达频率及年龄、电池和飞控模式。
- 机载把 SYS_STATUS 设为 1 Hz 以稳定提供电池 fallback。未解锁 SITL 最终实测状态约 10 Hz、
  电池 12.60 V/100%、姿态和本地位姿有效、飞控 STABILIZE；全程未发送飞行动作。
- LAND 的 SetMode ACK 只产生 RUNNING，必须观察到 FCU 解除武装才 final success，120 秒无解除
  武装则失败。频率配置必须对位置/姿态/IMU 各连续实测至少 1.5 秒、≥50 Hz 且样本新鲜；
  45 秒窗用于等待 EKF 本地位置。GPS 原点必须等待匹配的 FCU gp_origin 回读，8 秒无回读失败。
- 原有日志文字不因等级审查增删：运动参考/航点进度/正常参数等待降为 DEBUG，退化与重试为
  WARN，确定失败、外部模式中断和 failsafe LAND 为 ERROR；行为确认新增的状态/result 单独归于
  accepted/applied/observed 改造。
- 最终 Python 74 passed，本地 ROS/C++ 5 tests 与接口 2.1 隔离 smoke 通过；真机 `/tmp` Humble/
  aarch64 两包 Release 构建与 5 tests 通过，临时目录已清理、服务 inactive、无飞行进程。
  正式机载部署目录未更新；接口 2.1 上线时必须和地面站同步部署。详见
  `agent/report/report-2026-08-11-detailed-status-and-truthful-results.md`。

## 2026-08-11 撤销 100 Hz 实测监听

- 用户明确撤销消息频率真实确认：机载节点不再订阅 `/mavros/imu/data`
  或 `/mavros/imu/data_raw`，不再对位置/姿态/IMU 做样本计数、实测频率或 45 秒
  等待。`message_rates_configured` 现仅表示四个 `MessageInterval` ACK 成功，
  不代表已观察到 100 Hz。
- 原有三条 100 Hz 配置请求和电池 `SYS_STATUS` 1 Hz 保留。俯仰/偏航、
  电池、飞控模式、地面站↔机载 `ControlStatus` 实际到达频率显示保留；
  LAND 解除武装和 GPS 原点回读确认、日志等级调整也保留。
- 本地运行时图确认两路 IMU 订阅为零；Python 74 passed，ROS/C++ 5 tests
  零失败。本次未连接或同步实机，未验证实机 100 Hz。详见
  `agent/report/report-2026-08-11-remove-100hz-rate-listeners.md`。

## 2026-08-11 真机开机自启动配置

- 真机 `xld@192.168.112.186` 已安装并启用系统服务
  `/etc/systemd/system/ros2-ardupilot-onboard.service`，开机进入 `multi-user.target` 后以 `xld`
  用户运行 `/home/onboard/ros2-ardupilot-mavros-control/start_drone_all.sh`。服务固定加载 Humble、
  项目 overlay、`/home/xld/ws` Odin overlay 和 `/home/xld/vrpn_mavros` extnav overlay，使用
  domain 0/subnet；失败后间隔 10 秒重试，停止时先向监督脚本发送 SIGINT。
- 配置阶段只执行 `systemctl enable`，没有 `start` 或重启真机；交付时服务为
  `enabled`、`inactive/dead`，四组件零残留且 `/dev/ttyTHS1` 无占用。首次开机实际运行仍需由
  用户观察 `systemctl status` 与 journal 验证。
- 真机脚本哈希仍为旧已验证版本
  `4b8a3307a71fecb84904df5374b90d1d54309b18181d5a5f1cdece4b5c624452`，并不支持后来本地版本
  才加入的真正 `--check`。一次按新文档执行的 `--check` 因此短暂启动四组件，发现后立即通过
  脚本 SIGINT 清理；只读状态确认全程 `armed=false`、STABILIZE，最终无进程或串口占用。
  不得再把该远端旧脚本的 `--check` 当作无启动检查。详见
  `agent/report/report-2026-08-11-onboard-boot-autostart.md`。

## 2026-08-11 真机机载 2.1 同步与未武装测试

- 真机正式 sparse 工作树已快进到 `7c1f8cc`，Humble/aarch64 Release 两包构建、
  5 项 ROS/C++ 测试和隔离 2.1 smoke 通过。旧工作树/安装前缀备份位于
  `.deployment-backups/pre-sync-7c1f8cc-20260811-194224/`，快照 SHA-256 为
  `594b12d8b638f3663ecf67552a72ccb619b3fdc91d83fef61ecab24d5522a4cc`。
- 真实四组件链路实收 490 条状态，平均 9.9994 Hz；全程 `armed=false`、
  STABILIZE、零租约、零姿态 setpoint。电池 23.435 V，位姿/推力参数有效，
  两路 IMU 机载订阅为 0；`message_rates_configured` 只代表 ACK，未验证 100 Hz。
- systemd drop-in 固定旧部署已确认的 `/dev/ttyTHS1`，并将 SIGINT/130 识别为
  正常停止。最终服务 `enabled` 但 inactive，Result=success，四组件零进程、
  串口无占用。Odin/extnav 关停时仍有外部驱动异常日志，但无残留。详见
  `agent/report/report-2026-08-11-real-onboard-sync-and-test.md`。

## 2026-08-11 真机交互启动配置修复

- `start_drone_all.sh` 会读取 `/etc/ros2-ardupilot/onboard.env`（可由
  `ONBOARD_ENV_FILE` 覆盖），让交互终端与 systemd 共用已人工确认的 FCU 串口和
  Odin/extnav overlay；没有配置时仍拒绝猜测多个串口或 overlay。
- 真机环境文件固定已验证的 `/dev/ttyTHS1`、`/home/xld/ws/install/setup.bash` 和
  `/home/xld/vrpn_mavros/install/setup.bash`。直接执行 `./start_drone_all.sh --check`
  已通过且没有启动组件；服务仍 inactive/Result=success，飞行进程为零，串口空闲。
- 修复提交 `72cc836` 已同步真机；本地目标测试 10 passed、完整 Python 测试
  74 passed。详见 `agent/report/report-2026-08-11-manual-onboard-launch-env-fix.md`。

## 2026-08-11 GPS 原点重复连接与失败彻底断开修复

- `gp_origin` 是 reliable + transient-local 的已应用状态，不是周期事件。机载端必须记录当前
  FCU 会话是否已真实观察原点：匹配请求可幂等 final success 且无需重复发布；不存在或不匹配
  时仍须等待新回读并保留 8 秒失败门。已知 FCU 断线会作废缓存，禁止跨 FCU 会话误确认。
- 完整仿真/实机工作流的前检查失败、取消、异常和主动断开都是完整会话边界：释放租约、清理
  本地进程后必须停止 `GroundStationRosController` 并销毁 DDS context。失败后的 GUI 应为
  `ROS IDLE`、快照全清，不能因仍订阅 10 Hz 状态而重新显示飞控连接或继续刷新遥测。纯 Wi-Fi
  只读检测仍不得复用会发送 release/管理进程的清理路径。
- 真实旧机载端原点超时回归确认修改后的地面站 `ready=false`、domain=None、快照全清，最终错误
  后 3 秒无晚到链路事件。修复部署后同一 GUI 身份连续两轮连接/断开均成功，原点 ticket 2/4
  都返回“当前值已匹配”，每轮断开后 2 秒无晚到重连。
- 提交 `91ccc75` 已推送 `main` 并同步真机；本地 Python 75 passed、ROS/C++ 5 tests 零失败，
  真机 Humble/aarch64 Release 构建与 5 tests 零失败。最终 systemd 服务 enabled + active，飞机
  `armed=false`、STABILIZE、IDLE、无租约、setpoint 单发布者。
- Humble/Jazzy context 切换期间既有 `sequence size exceeds remaining buffer` 仍会出现，但未造成
  状态、租约、原点或进程失败；该独立跨发行版 DDS 问题没有在本修复中屏蔽或宣称解决。详见
  `agent/report/report-2026-08-11-gps-origin-reconnect-and-failure-teardown.md`。

## 2026-08-11 机载开机自启动首次校时门

- 真机冷启动故障不是串口或 systemd 环境差异：服务在开机 12.328 秒启动，MAVROS 在
  24.081 秒已收到 FCU HEARTBEAT，但 `systemd-timesyncd` 到 39.641 秒才首次同步，约 5 分钟
  墙钟跳变随后破坏已创建的 ROS/MAVROS 定时链；系统时间稳定后手工运行同一脚本可立即连接。
- 机载服务现在 `Wants=network-online.target systemd-time-wait-sync.service` 且
  `After=network-online.target time-sync.target`，保留开机自启动但不再早于首次校时；禁止退回
  固定秒数 `sleep`。仓库 systemd 示例和部署测试同步维护该顺序。
- 新单元通过解析与真实 systemd 启动顺序验证，地面端只读收到约 9.999 Hz 状态，最终
  `fcu_connected=true`、`armed=false`、本地位姿/频率/推力参数就绪、租约为空。全量 Python
  75 passed。没有执行整机冷重启、租约、原点、模式、解锁、起飞或控制命令；下一次正常开机
  仍需复验 cold-boot journal。详见
  `agent/report/report-2026-08-11-onboard-autostart-time-sync-fix.md`。

## 2026-08-12 Task 16 beautify-3 与接口 2.2

- 手动操纵默认坐标系为本地 ENU；摘要固定为实际高度/速度/航向与三类指令速度；详细左栏固定为
  实际位姿、目标位姿、实际速度、目标速度、控制周期、安全门控，右栏只保留单次增量与
  “俯仰/滚转”。顶部状态固定为控制权、飞控模式、电池、通讯频率、最近指令，1180×700 时自动
  换成坐标行和状态行。
- 通讯绿色范围使用命名常量 `10±1 Hz`；实机电池阈值为 23.5/22.5 V 且只显示电压，仿真阈值为
  50%/25% 且只显示百分比。为避免把 yaw 假装成 roll，`ControlStatus` 2.2 新增真实 roll，机载从
  已有位姿四元数统一解算并聚合，地面站 `VehicleSnapshot` 同步保留。
- 本地最终 Python 76 passed、ROS/C++ 5 tests 零失败、三包构建和环境检查通过。真机已同步并运行
  `b84ca3e`；重启前置 Humble 监视器实收 164 条状态/16.501319 秒（9.8780 Hz），最后 FCU、位姿、
  电池和频率配置有效，全部 `armed=false`、零租约、42 秒零姿态 setpoint，roll 为有限值。
- **当前真机不应视为飞行 READY**：最终 `thrust_mode_verified=false`，一键脚本 120 秒后报告 full
  readiness 未达到。安全门因此禁止起飞/运动；必须另行查明 MAVROS 参数服务发现/同步并恢复该门，
  禁止绕过。跨 Humble/Jazzy Fast DDS 晚加入订阅仍有发现异常，不能把部分话题可达当成实飞通信
  基线。详见 `agent/report/report-2026-08-12-task16-beautify-3.md`。

## 2026-08-12 通讯频率状态块宽度防抖

- 通讯频率块按配置正常范围计算整数位数，以同位数宽字符样本完成 style polish 后的 `sizeHint`
  作为最小宽度；9.xx/10.xx 正常波动不再改变自身宽度或推挤“最近指令”。没有设置固定/最大
  宽度，更长内容仍可扩展，1180×700 两行响应布局保持不变。
- 几何回归覆盖 10.00→9.99→10.00→9.98 的宽度/X 坐标稳定性及 100.00 Hz 扩展；最终 Python
  76 passed，静态检查与环境检查通过。本次未连接真机。详见
  `agent/report/report-2026-08-12-communication-rate-chip-width-stability.md`。

## 2026-08-12 当前改动整理与过程目录清理

- 根目录 `README.md` 与 `integration/` 已按用户要求纳入版本管理；任务 14～16 的任务输入、有效
  素材和报告，以及 AP 重启调查 DOCX/PDF 一并保留。
- `agent/codex/`、`agent/grok/` 是本地代理过程目录，内容已清空并由 `.gitignore` 整目录忽略；
  不应再把需要长期维护的正式配置只放在这两个目录中。
- 清理了 `agent/ref/` 中与任务 15 素材完全相同的视频副本，以及三张未引用的 8×1 像素空白
  PNG。完整清单与验证结果见
  `agent/report/report-2026-08-12-current-changes-review-and-push.md`。

## 2026-08-12 任务 17 CSV 航点导入

- 航点操作行最右侧新增普通白色“从文件导入”按钮，选择器只允许单文件；航点表支持本地文件
  URL 拖放，多文件会明确拒绝。导入前完整解析，随后使用默认取消的统一阴影确认框；只有确认后
  才原子替换 GUI 列表并重置旧进度，取消或失败均保留原列表。
- CSV 表头为 `index,x,y,z,yaw`，序号必须从 1 连续递增；位置按绝对本地 ENU 米读取，Yaw 从度
  转内部弧度。单次上限 256，与机载执行器上限一致；UTF-8 BOM、空尾行可接受，缺列、非有限值、
  越界、乱序和第 257 条均整体拒绝。
- `ground_station_core/waypoint_io.py` 以格式描述/加载器分派预留未来 Excel 扩展点，但当前只注册
  CSV。示例为 `examples/waypoints-example.csv`；文件选择器默认打开该目录。
- 统一 `ShadowMessageBox` 会在 Qt 完成正文换行后重新取最终 `sizeHint`，长导入警告不再裁掉
  末段。最终 Python 88 passed；三包构建成功，ROS/C++ 5 tests 零失败；环境、compileall、致命
  flake8 和 88 字符检查通过。本任务未连接实机、未启动飞行仿真、未发送命令、解锁或起飞。详见
  `agent/report/report-2026-08-12-task17-csv.md`。

## 2026-08-12 任务 15 RViz 航点预览

- 航点操作行在“从文件导入”左侧新增“预览”；首期只显示编号航点、逐点直连的名义 Path 和
  `ControlStatus` 权威实时机体位姿。预览激活后的航点编辑会替换 retained 快照，清空会显式删除
  旧 Marker/Path；它不申请租约、不占飞行命令 ticket，也不创建 MAVROS setpoint publisher。
- 仿真固定复用 domain 231 + `LOCALHOST` 的受管 `rviz`；实机固定在地面端以 domain 0 + `SUBNET`
  启动一个 `rviz_hardware_preview` 并复用。模式、控制器实际 domain 和发现范围必须一致，切换/断开
  时本地预览随会话清理，不建立仿真—实机桥。
- 预览话题为 `/ground_station/waypoint_markers`、`/ground_station/waypoint_path` 和
  `/ground_station/vehicle_pose`；模型 description 与 TF 使用 `ground_station_preview` 命名空间，
  位姿桥不再直订远端 MAVROS。RViz 只保留 Interact 工具。
- 最终 Python 95 passed；三包构建成功，ROS/C++ 5 tests 零失败。临时 domain 232 实际 RViz 验证
  3 个航点、2 段直线和实时机体模型成功；没有连接实机、执行飞行、解锁或起飞。详见
  `agent/report/report-2026-08-12-task15-rviz-waypoint-preview.md`。

## 2026-08-12 任务 15 RViz 启动位姿与空闲抖动修复

- 任务 15 初版错误地让仿真模型依赖只有点击“预览”后才启用的 `/ground_station/vehicle_pose`，
  导致启动 RViz 时 `ground_station_preview/a1`～`a4` 无法连接 `map`。现由 launch 参数选择位姿源：
  仿真在 domain 231 直用本地域 `/mavros/local_position/pose`，实机 domain 0 仍用地面聚合位姿，
  不增加远端 MAVROS 订阅或跨域 bridge。
- 可视化进程组使用 nice 5；RViz 为 15 FPS、RobotModel 0.1 秒更新，调试 TF 图层默认关闭，位姿
  订阅 depth 1。现场采样 RViz 约从 11% 降至 4.6% 单核 CPU，但 ToDesk/Xorg/MAVROS 高负载下
  非实时控制周期 miss 仍可能增加，禁止把它报告成零抖动。
- 空闲且未武装时 deadline miss 只保留在权威遥测/工程面板，不再逐次写 WARN；控制器激活或飞机
  已武装时告警保持。真实未武装 SITL 在未点击预览时已持续获得机体 TF，30 秒 controller WARN
  为 0；Python 96 passed，三包构建成功，ROS/C++ 5 tests 零失败。未连接、解锁或起飞实机。详见
  `agent/report/report-2026-08-12-task15-rviz-startup-and-idle-jitter-fix.md`。

## 2026-08-12 地面站异常退出无残留

- `start_ground_all.sh` 不再 `exec` GUI，而是保留父级生命周期监督壳；GUI 退出后始终通过隐藏的
  `ground_station.py --cleanup-local-processes` 复核项目专属本地进程。正常清理时幂等无操作，
  GUI 遭 `SIGKILL` 时仍可清除 SITL/MAVROS/onboard/RViz 残留。
- GUI 捕获 `SIGHUP/SIGINT/SIGQUIT/SIGTERM`，由 Qt 主循环转入统一安全退出；窗口关闭保留原二次
  确认，外部信号不弹无人可操作的确认框。Qt 事件循环异常返回也会执行一次锁保护的同步兜底。
- 真实启动入口的终端进程组 HUP、仅 GUI HUP、GUI SIGKILL 三条进程级回归均为零残留；AP、
  MAVROS、RViz 生产 argv 形态替身均被识别并终止。最终 Python 100 passed，环境检查、compileall、
  致命 flake8、shellcheck 与本任务 diff 检查通过。本任务未连接实机、解锁或起飞。详见
  `agent/report/report-2026-08-12-ground-station-exit-cleanup.md`。

## 2026-08-12 航点平稳参考生成与轨迹 PD+DOB

- `ExecuteWaypoints`/`ControlStatus` 接口升级为 3.0：避障空壳、参考生成和跟踪控制是三个独立字段；
  GUI 默认继续使用 `STEP_POSITION + POSITION_PD_DOB` 基线。执行卡去掉标题，进度/发送按 2:1
  同行，下方为三列下拉框。组合只在解除武装待机时可改，机载端还锁定同一武装周期并拒绝绕过 GUI
  换方法；解除武装后清锁。
- 新增可插拔直线航段生成器：位置阶跃、二阶滤波、普通梯形速度、七阶段限 jerk S 曲线；统一输出
  位置/速度/加速度/航向参考。`TRAJECTORY_PD_DOB` 复用唯一 `DobController`，使用独立低带宽增益和
  加速度前馈；基线分支不启用前馈且原增益/到达语义不变。所有方法参数按独立前缀集中在
  `src/onboard_control/config/control.yaml`。
- 最终推荐默认 XY 参考速度 0.30 m/s、加速 0.18 m/s²、减速 0.20 m/s²、jerk 0.15 m/s³，配
  `trajectory_dob_L_xy/z=0.5/0.3`。4 m 往返 SITL 中，S 曲线实际峰值约 0.327 m/s、匀速
  0.303±0.019 m/s、最大倾角约 1.73°、姿态变化率 RMS 约 0.64°/s；梯形对照姿态变化率约
  1.29°/s，原基线为约 3.71 m/s、25.15°、52.88°/s。90° 转向复测结果相近。
- 主推荐机载进程树约占单核 3.9%～4.0%，100 Hz 航点控制零新增 deadline miss；连续完成多轮
  起飞/往返/降落并清理全部本地进程。Python 正式测试 103 passed，ROS/C++ 汇总 13 tests 零失败，
  三包构建和静态检查通过。根目录无范围 pytest 只因用户未跟踪的
  `integration/websocket_test_demo` 缺少 `ws_demo` 在收集阶段失败，没有修改该无关目录。本任务仅
  使用隔离 SITL，未连接、解锁或起飞实机。详见
  `agent/report/report-2026-08-12-waypoint-smooth-reference-methods.md`。

## 2026-08-13 control.yaml 中文注释补全

- `src/onboard_control/config/control.yaml` 的分组说明已全部改为中文，全部 67 个参数均增加同行中文
  注释，说明用途、单位或判定语义。修改前后通过 YAML 解析比较确认参数键值完全一致，不改变控制
  效果和安全阈值。
- 隔离 domain 232 + `LOCALHOST` 下，机载节点成功读取该文件并以接口 3.0、100 Hz 启动；未启动
  SITL，未连接、解锁或起飞实机。详见
  `agent/report/report-2026-08-13-control-yaml-chinese-comments.md`。

## 2026-08-13 三轮 5 米往返爬升示例航点

- `examples/waypoints-5m-altitude-ladder-3-rounds.csv` 包含 13 个绝对本地 ENU 航点：从
  `(0, 0, 0.8)` 出发，沿 X 正方向进行三轮 5 m 去程/回程；每个水平航段末端原地上升
  0.5 m，并在 0°/180°之间交替转向，最终到达 `(0, 0, 3.8)`、航向 0°。
- 文件已通过项目 CSV 加载器和既有航点导入测试；未启动仿真或操作实机。详见
  `agent/report/report-2026-08-13-three-round-waypoints.md`。

## 2026-08-13 机载接口 3.0 源码同步与本地验证

- 真机 sparse 源码工作树已从 `31b19fc` 快进到 `4741066`；本地/真机
  `src/onboard_control` 与 `src/guided_interfaces` tree 对象一致。同步前备份为真机
  `.deployment-backups/pre-source-sync-20260813-025708.tar.gz`，SHA-256
  `092f15ea2497218ce5af47d21ad0338980e0dea1d3b415c6027c1bba10f64fab`。
- 本次只同步源码，没有在真机构建、替换 `install/` 或重启服务；同一 systemd 主进程保持运行，
  当前实际节点仍为接口 2.2。接口 3.0 上线必须另行完成 Humble/aarch64 构建、测试、隔离 smoke
  和安全窗口重启，不能把源码同步等同于部署生效。
- 本地三包 Release 构建成功，ROS/C++ 13 tests 零失败，加载本地 overlay 后 Python 103 passed；
  domain 232/LOCALHOST 隔离 smoke 为接口 3.0、FCU 未连接、未武装、零姿态 setpoint。
- `4741066` 把平滑航点水平参考速度调到 1.00 m/s、速度保护调到 1.20 m/s，并调整加速度、jerk、
  到达速度和保持时间；这些值通过静态/单元测试但没有真机飞行验证。当前真机启动日志仍提示
  full readiness 未达到，禁止宣称飞行 READY。详见
  `agent/report/report-2026-08-13-onboard-source-sync-local-build-test.md`。

## 2026-08-13 真机通讯/连接接口错配修复

- 地面站无法检测/连接的主因是部署只更新了真机源码到接口 3.0，真机 `install/` 中实际运行的
  `guided_interfaces`/`onboard_control` 仍为 2.2.0；Jazzy 地面站按 3.0 结构无法反序列化旧
  `ControlStatus`，因此表现为状态完全收不到，而不是正常的“版本不兼容”提示。排查时服务还已于
  03:06:49 被停止，这会独立造成无发布者。
- `start_drone_all.sh` 现在启动任何组件前比较源码/安装包版本；3.0/2.2 错配会明确拒绝。只读
  READY 探针固定 `--no-daemon`、显式消息类型与 best-effort/volatile QoS；机载状态图查询也在
  context 关闭竞态中安全退出，不再因 `count_publishers` 导致 SIGINT 阶段 abort。
- 真机已在提交 `0748dda` 原生完成 Humble/aarch64 Release 构建、13 tests 和隔离 smoke；当前
  systemd 运行接口 3.0 并打印 READY。被动检测实测 31 条/3 秒、10.29 Hz、最大间隔 110.9 ms；
  完整连接连续两轮成功获得并释放租约，最终 `armed=false`、STABILIZE、待机、无租约、3 秒零姿态
  setpoint。
- Humble/Jazzy Fast DDS 在其他跨发行版端点上仍产生既有 `sequence size exceeds remaining
  buffer`，本次没有伪称解决 ROS 官方不保证的跨发行版兼容性；它不再阻断当前接口 3.0 状态与
  高层服务实测，但仍是实飞前必须消除或单独验收的风险。详见
  `agent/report/report-2026-08-13-hardware-communication-interface-fix.md`。

## 2026-08-13 192.168.112.101 仿真启动依赖修复

- 目标机点击启动仿真失败的第一根因是 `sim_vehicle.py` 按命令名启动 `mavproxy.py`，而已有
  MAVProxy 位于未加入 PATH 的 `/home/scq/venv-ardupilot/bin`；第二根因是 MAVProxy 1.8.74
  wheel 未声明实际导入的 `future`，导致项目 venv 内运行即退出。
- `requirements-gui.txt` 现显式安装 MAVProxy 与 future；地面站会从显式配置、PATH、项目 venv
  和常见 ArduPilot venv 定位可执行入口，只向 SITL 子进程注入其 bin，并在启动任何进程前真实
  执行 `mavproxy.py --version`。新电脑执行 `setup_project.sh` 不应再漏掉该链路依赖。
- 目标机完整未武装仿真已达到 SITL/MAVROS/onboard/RViz 就绪；最终状态为 `armed=false`、
  STABILIZE，本地位姿、租约和推力语义门有效。清理停止 4 个受管进程，5760/5762 无监听且无
  项目仿真残留。目标机与本机正式 Python 测试均为 105 passed；根目录无范围 pytest 仍受既有
  `integration/websocket_test_demo` 缺少 `ws_demo` 的收集问题影响。详见
  `agent/report/report-2026-08-13-remote-simulation-start-fix.md`。

## 2026-08-13 跨地面/飞机机载构建快捷入口

- 仓库根目录新增可执行 `./build_onboard_control`：无参数时自动选择当前主机 ROS 发行版，以
  Release 模式重建 `guided_interfaces` 与 `onboard_control`；`--verify` 额外执行依赖检查、
  13 项 ROS/C++ 测试和 domain 231/localhost 隔离 smoke。
- 入口复用 `onboard_workspace.sh`，不启动、停止或重启机载服务，不连接真实 MAVROS，不发送
  飞行命令。构建期间已有服务继续运行；需要加载新产物时仍须另选安全窗口重启。
- 机载 non-cone sparse checkout 清单已加入根目录 `/build_onboard_control`，首次检出文档、更新
  流程和自动回归同步维护。地面 Jazzy 默认构建与 `--verify` 均通过，正式 Python 为 106 passed。
- 提交 `0d5fb42` 已通过 bundle 快进到飞机，机载 sparse 清单已加入入口；飞机 Humble/aarch64
  实际执行 `./build_onboard_control`，两包 2.15 秒构建通过。构建前后
  `ros2-ardupilot-onboard.service` 保持 active，主 PID 2455 和启动时间未变化；没有重启服务或发送
  飞行命令。飞机保留既有未跟踪 `.deployment-backups/`。详见
  `agent/report/report-2026-08-13-portable-onboard-build-entry.md`。

## 2026-08-13 真机彻底停止入口

- 真机实际停止入口为 `/home/xld/stop_onboard_service.sh`，不在机载 sparse Git 工作树内。旧版本
  只有 `systemctl stop`，只能清理 systemd cgroup，无法结束从桌面终端手动启动的 Odin、MAVROS
  或 extnav；README 中项目目录相对路径并不准确。
- 真机脚本已原位改为：先停止 `ros2-ardupilot-onboard.service`，再精确匹配并收集 MAVROS、Odin、
  extnav、onboard_control、Odin RViz 及启动器的全部后代，依次发送 SIGINT、SIGTERM、必要时
  SIGKILL，最终同时确认服务 inactive 和目标进程为零。修改前备份为
  `/home/xld/stop_onboard_service.sh.pre-codex-20260813-163351`。
- 真实验收清除了 GNOME Terminal scope 中残留的 Odin PID 29919/29921；其未在 5 秒内响应 SIGINT，
  但在 SIGTERM 阶段退出。随后 `/dev/ttyTHS1` 无占用，`start_drone_all.sh --check` 返回 0 且没有
  启动组件；停止脚本重复执行也返回 0。没有解锁、起飞或正式重启服务。详见
  `agent/report/report-2026-08-13-onboard-stop-script-complete.md`。
- 随后停止入口正式迁入仓库根目录 `stop_onboard_service.sh` 并保留可执行位；机载 sparse checkout、
  部署文档、README 和自动测试已同步维护。飞机后续应从
  `/home/onboard/ros2-ardupilot-mavros-control/stop_onboard_service.sh` 执行，不再依赖 `/home/xld`
  下的工作副本。
- 提交 `b6f9544` 已推送远端 main 并快进同步飞机，飞机 Git 索引模式为 `100755`、文件
  SHA-256 为 `4e2f0ec30f3f9b2c39ee2c77d20ab4b05e4f8fb9ee8c087b2572b39d7c7ac248`。
  `/home/xld` 工作副本已移为 `stop_onboard_service.sh.pre-project-move-20260813-210247`，原 51 字节
  版本备份继续保留。同步期间已有 systemd 服务保持同一主 PID 4130、启动时间 20:40:18、零重启；
  未停止或重启服务。由于服务正在运行，`start_drone_all.sh --check` 按预期报告现有组件，不能把
  该结果误记为可在运行中通过的检查。
