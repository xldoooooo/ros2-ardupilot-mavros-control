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
- 实机连接、起降、航点、断开、清空和飞行中退出均有默认取消的确认框；退出与清理在后台执行，Qt 主线程不阻塞。飞控/EKF 原点：齿轮仅改本地缓存；**仅实机连接**工作流会 `set_gp_origin`。**本地仿真禁止**写入 GUI 缓存原点——SITL 使用自身 Home（默认 CMAC 一带）建立 EKF，本地位姿应在米级；若把与 SITL Home 不一致的经纬高（如杭州默认缓存）写入，local ENU 会偏移数百万米，GUI 实际位姿与 RViz TF 会发散。
- 按钮角色样式：`primary`/`success`/`danger` 均有 hover 变深（`accent_hover`/`success_hover`/`danger_hover`），与中性按钮悬停反馈一致。
- `EventLog` 在事件产生处保存 DEBUG/INFO/WARN/ERROR、来源、时间和序号。SITL/MAVROS/机载/RViz stdout 会实时 tee 到同一日志和原磁盘文件；Qt 只筛选已有等级，不按文本猜测。四个等级现为独立复选框，可任意组合显示。
- 任务 07（日志部分保留）：`ProcessSupervisor._explicit_output_level` 将 SITL/MAVROS 等 chatty 源的启动刷屏（如 `Embedding file ...`、ROS `[INFO]` 插件加载）降为 DEBUG；显式 WARN/ERROR 不变。日志“自动滚动”关闭后必须保留视口位置，禁止 `setTextCursor(End)` 强制跳底。
- GPS 与航点共七个数值框统一禁止鼠标滚轮改值，并使用自带 SVG 的明确上下箭头；航点清空会停止 GUI 进度跟踪，避免机载端旧任务快照恢复已清空进度。
- 菜单栏仅“设置”“帮助”：设置为“显示实时日志”“恢复默认布局”；日志自动滚动/清空在日志面板工具栏操作，不在菜单重复。右上“终端”通过 `QProcess.startDetached()` 在当前目录启动系统终端。
- 完整顶层窗口使用 frameless + 透明留边 + `outerWindowFrame`，具有连续四边轮廓和 Qt 自绘阴影；最小化、最大化/还原、关闭、菜单空白拖动及四边/四角缩放。
- 确认、警告、帮助和关于统一使用 `ShadowMessageBox`：保留模态和默认取消语义，并添加自绘标题栏、关闭按钮、四边轮廓、圆角和阴影；不要重新使用静态 `QMessageBox.information/about/warning` 绕过统一外框。
- Qt 依赖为 `PySide6>=6.7,<7`；本机验证为 6.11.1。生产 Python 代码已无 Tkinter 或旧 `ground_station_core.gui` 引用。
- 1600×920 为默认尺寸，1180×700 为最小尺寸；三栏各自使用滚动区，工作区宽度和日志高度均可用 splitter 调整，设置菜单可恢复默认比例。

## 重构后的部署边界

- `src/guided_interfaces/` 是上位机与机载计算机唯一共享的 ROS 2 高层协议，接口版本为 `1.0`。`ExecuteWaypoints.flight_strategy` 预留直线/自动避障/遇障悬停；当前机载仅实现直线飞行，其余值会告警并按直线执行。
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

## 版本库卫生

- `.gitignore` 排除 Python/colcon 产物、rosbag、ArduPilot/MAVProxy 日志、EEPROM、飞行记录和通用临时文件。
- `.deep-copilot/` 运行数据不应重新提交；本机 `AGENTS.md` 保持本地。
- 仓库不维护 `README.md`；正式执行与部署说明以当日报告、配置注释和 launch/deploy 示例为准。
