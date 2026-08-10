# Task 04：PySide6/Qt 地面站 GUI 重构执行报告

## 1. 任务结论

本次任务已在不修改飞行控制算法、机载状态机和 ROS 2 高层协议的边界内完成：

- 根入口仍为 `ground_station.py`，界面已从 Tkinter 完整替换为 PySide6/Qt 6；
- 生产 Python 源码中已无 `tkinter`、旧 `GroundStationApp` 或
  `ground_station_core.gui` 引用，原 795 行 Tk 文件已删除；
- GUI 拆分为主题、通用组件、状态策略、环境/飞行动作、航点、日志和主窗口模块；
- 所有飞控按钮都由统一状态策略门控，未初始化环境时保持禁用；
- 实机连接、GPS 原点、起飞、降落、航点、断开、飞行中退出等操作都有默认取消的二次确认；
- 新增 DEBUG/INFO/WARN/ERROR 结构化日志总线，等级在事件产生或外部日志接收时确定；
- SITL/MAVROS/机载节点/RViz 输出会同时写入原磁盘日志和 Qt 实时日志框；
- 日志框支持精确等级筛选、来源/消息搜索、累计计数、清空显示和只纵向自动滚动；
- 真实 Qt→ROS→SITL 闭环通过起飞、八个方向、悬停、单航点、降落、清理和异步安全退出；
- Python/Qt 回归为 14 passed，ROS/colcon 为 5 tests、0 errors/failures。

任务文件中的 “Pyslide” 结合历史技术选型报告按 PySide6 理解。历史报告已经比较
PyQt6 与 PySide6，并明确推荐 Qt 官方绑定 PySide6，以避免默认引入 PyQt 的 GPL/商业
许可选择。本机最终验证版本为 PySide6 6.11.1；仓库声明兼容范围为 `>=6.7,<7`。

## 2. 范围与边界

本轮只改上位机 GUI、GUI 所需日志桥和进程输出桥：

- 未修改 `src/onboard_control/` 中的 PD、DOB、模式仲裁、航点到达判定、失联保护、
  推力映射或 100 Hz 控制循环；
- 未修改 `guided_interfaces` 消息/服务定义和接口版本；
- 未增加第二个 MAVROS setpoint 发布者；
- 未实现 `TODO.md` 中的速度参考加速度/jerk、无扰切换或键盘自动重复处理；
- 未把地面站日志显示逻辑伪装成机载安全逻辑；
- 真机路径仍只连接/释放远端控制租约，不远程启动或终止机载进程。

开始任务前工作树已存在两项非本轮改动：未跟踪的 `agent/task/04-qt.md`，以及
`test_takeoff5.py` 的删除。本轮读取任务文件但没有创建、恢复或擅自改写该脚本；两项
状态均保留给用户处理。

## 3. 实现架构

### 3.1 单文件入口

`ground_station.py` 继续负责：

1. 自动加载 ROS 2 与本工作空间 overlay；
2. 保留 `--check-environment` 无窗口诊断；
3. 检查 PySide6，缺失时输出明确安装命令；
4. 创建 `QApplication`、应用统一主题并显示 `GroundStationWindow`。

运行依赖记录在 `requirements-gui.txt`：

```text
PySide6>=6.7,<7
```

### 3.2 Qt 模块

| 文件 | 职责 |
|---|---|
| `ground_station_core/qt_ui/main_window.py` | 主窗口、线程信号桥、环境/飞行动作编排、状态刷新、安全退出 |
| `ground_station_core/qt_ui/operations_panel.py` | 环境、GPS、起降、手动运动意图和遥测组件 |
| `ground_station_core/qt_ui/waypoint_panel.py` | 数值航点编辑、表格、排序、清空、上传和进度 |
| `ground_station_core/qt_ui/log_panel.py` | 实时日志、等级/文本筛选、计数和自动滚动 |
| `ground_station_core/qt_ui/state.py` | 从权威 `VehicleSnapshot` 推导所有按钮可用性 |
| `ground_station_core/qt_ui/widgets.py` | 卡片、顶部状态卡、活动提示条 |
| `ground_station_core/qt_ui/theme.py` | 浅色低饱和工程主题和统一 QSS |
| `ground_station_core/event_log.py` | 与 Qt 无关、线程安全、有界的结构化日志总线 |

界面代码不再集中在一个文件中。主窗口只组合业务面板并桥接后端；航点模型、日志显示、
状态门控和样式均有独立模块与测试。

### 3.3 线程模型

- ROS 客户端继续在原常驻 Python 线程运行；
- 环境初始化继续在 `EnvironmentInitializer` 工作线程运行；
- 外部进程各有一个轻量输出读取线程，将 stdout 同时写入磁盘和结构化日志；
- 清理和退出使用后台线程，不再在 Qt 主线程同步等待多秒；
- 所有 Qt 控件修改都通过 `_ThreadBridge` 信号回到主线程；
- 关闭窗口时先锁定全部操作，后台释放租约、清理本地仿真、停止 ROS，再真正销毁窗口。

## 4. 视觉与响应式布局

### 4.1 视觉原则

- 浅灰工作区、白色卡片、深蓝主操作、低饱和绿色成功和红色危险操作；
- 仅用左侧细色条表达顶部状态，不使用大面积多色装饰；
- 数字遥测使用等宽字体，文字状态使用中英文工程术语；
- 禁用按钮统一为灰色，语义角色颜色不会覆盖 disabled 样式；
- ARMED、FAILSAFE、setpoint conflict 和危险按钮才使用红色强调。

### 4.2 信息层级

从上到下依次为：

1. 产品标题和当前环境；
2. 机载链路、飞行器、控制模式、控制健康四个总览卡；
3. 最近工作流/命令活动提示；
4. 左侧连接/飞行动作与手动控制页签，右侧航点任务；
5. 可拖动高度的统一实时日志；
6. 接口版本和本客户端 `source_id` 状态栏。

左侧使用“连接与飞行动作”“手动控制与遥测”两个明确页签，避免默认窗口中把手动按钮
藏在深滚动区。主工作区、日志区以及左右栏都使用 `QSplitter`；环境、航点栏分别由
`QScrollArea` 承载。900×650 时内容通过各自滚动区访问，不重叠、不挤压日志；
1600×1000 时正常扩展留白和表格区域。

### 4.3 反复视觉调试中修复的问题

1. 首版默认高度下手动控制位于左侧长滚动区底部：改成显式页签；
2. 首版 900×650 航点卡被垂直压缩，产生视觉叠压：航点整栏加入滚动容器并限制表格高度；
3. 语义按钮 QSS 覆盖通用 disabled 样式：增加角色按钮专用禁用规则；
4. 日志自动滚动把横向视图拉到长消息末尾：改为仅滚动纵向并保留横向位置；
5. 三行 GPS 输入占用过多高度：改为同一行三列、有界数值输入；
6. 原自由文本坐标可能直到点击才发现非法值：改用有范围、精度和单位的 `QDoubleSpinBox`。

## 5. 功能对应关系

| 原 Tk 功能 | 新 Qt 实现 |
|---|---|
| 初始化仿真环境 | “启动本地仿真”，复用完整 `EnvironmentInitializer` 流程 |
| 初始化实机环境 | “连接实机服务”，确认后读取 GPS 原点并连接远端服务 |
| GPS 原点输入/发送 | 三个有界数值框 + 写入确认 |
| 起飞/降落 | 语义化按钮 + 高风险确认 + 可靠结果显示 |
| W/S、I/K、J/L、A/D | 八个手动按钮和窗口快捷键，仍只发送增量意图 |
| Space 悬停 | 悬停按钮和快捷键 |
| 实际/目标状态 | 手动页签中的位姿、速度、目标、频率、抖动、安全门控 |
| 航点添加/删除/排序/清空 | 数值输入 + 只读表格 + 四个编辑动作 |
| 航点发送/进度 | 确认摘要 + 机载 1-based 进度 + 可靠结果 |
| 断开/关闭仿真 | 后台释放租约、清理本地进程、显示可审计报告 |
| 关闭窗口 | 飞行中二次确认 + 异步安全清理 |

快捷键在 GPS/航点数值框、搜索框、日志框、下拉框或按钮获得焦点时不会穿透成飞行命令。
本轮没有实现 `TODO.md` 中尚未授权的自动重复策略变更。

## 6. 状态门控与高风险确认

### 6.1 统一可用性策略

`derive_availability()` 每 100 ms 根据一次不可变机载快照计算全部按钮状态。一般飞行控制
必须同时满足：

- 操作者已经显式完成仿真初始化或实机连接；
- ROS 客户端和机载状态可用；
- 飞控已连接；
- 本客户端持有唯一控制租约；
- 本地位置有效；
- `GUID_OPTIONS` 真实推力语义已验证；
- 未检测到 setpoint 多发布者冲突；
- 环境工作流未忙、窗口未退出。

进一步限制：

- 起飞只在未武装且机载模式为 IDLE 时可用；
- 手动、悬停和航点只在已武装且处于 KEYBOARD/HOVER/WAYPOINT 时可用；
- GPS 原点只允许在未武装时写入；
- 航点列表可离线编辑，但没有完整飞行门控时不能上传；
- 航点运行期间锁定编辑和重复发送；
- LAND 作为安全动作只要求可靠命令链路和已武装，不会被本地位置、推力诊断或
  setpoint conflict 额外禁用；
- 初始化进行中仍保留“断开”作为取消入口，但清理线程运行时禁止重复点击。

### 6.2 二次确认

以下动作默认按钮均为“取消”：

- 连接实机服务；
- 从当前环境切换到本地仿真；
- 写入 GPS 原点；
- 起飞；
- 降落；
- 发送航点任务；
- 清空本地航点；
- 断开/关闭本地仿真；
- 飞行器武装或控制器活动时退出窗口。

飞行中断开/退出提示明确说明：该动作释放地面站租约，不等价于立即 LAND；机载端仍按
既有失联悬停与宽限期降落策略运行。

## 7. 结构化实时日志

### 7.1 等级产生位置

日志等级不是由 Qt 文本框分析消息含义后猜测：

- GUI 操作者动作在主窗口发出时明确调用 DEBUG/INFO/WARN；
- 环境工作流在产生进度、成功、取消和异常时明确给出等级；
- ROS 客户端在产生命令结果、租约变化、武装变化、FAILSAFE、发布者冲突和 deadline
  miss 时明确给出等级；
- 子进程输出接收时只映射输出自带的 `[DEBUG]`、`[INFO]`、`[WARN]/[WARNING]`、
  `[ERROR]/[FATAL]` 标记；无等级标记的普通 stdout 按 INFO 接收，不进行关键词猜测。

Qt `LogPanel` 只按 `LogEvent.level` 精确筛选。

### 7.2 输出与容量

- 原磁盘日志仍写入 `/tmp/ros2_ardupilot_ground_station/*.log`；
- GUI 内存历史上限为 8000 条，避免长时间仿真无限占用内存；
- 磁盘日志仍保留完整外部输出；
- 日志面板可筛选全部、DEBUG、INFO、WARN、ERROR，并可搜索来源或正文；
- “清空显示”不篡改源日志，只隐藏当前历史，后续事件继续实时显示。

## 8. 自动化测试

新增：

- `tests/test_event_log.py`：日志序号/来源/等级、子进程实时 tee、显式 WARN 映射和磁盘日志；
- `tests/test_qt_gui.py`：状态门控、LAND 安全例外、日志精确筛选、危险确认、键盘焦点保护、
  航点上传、900×650/1600×1000 响应布局。

最终命令与结果：

```text
source /opt/ros/jazzy/setup.bash
source install/setup.bash
QT_QPA_PLATFORM=offscreen python3 -m pytest -q
14 passed in 4.50s

colcon build --packages-select guided_interfaces onboard_control guided_sim
Summary: 3 packages finished

colcon test --packages-select guided_interfaces onboard_control guided_sim
colcon test-result --verbose
Summary: 5 tests, 0 errors, 0 failures, 0 skipped

python3 ground_station.py --check-environment
workspace environment OK: guided_interfaces + rclpy available

python3 -m compileall -q ground_station.py ground_station_core tests
通过

python3 -m flake8 --select F ground_station.py ground_station_core tests
通过

git diff --check
通过
```

生产 Python/配置范围执行 `rg` 后不存在 `tkinter`、`GroundStationApp` 或旧 GUI 模块引用。
旧任务报告中对当时 Tk 基线的历史文字保留，不属于运行时代码。

## 9. 实际显示与交互验证

### 9.1 渲染检查

- 在本机 `DISPLAY=:0` 实际创建并显示 Qt 窗口；
- 以 1320×900 检查连接页和手动页；
- 以 offscreen 900×650 和 1600×1000 精确设置窗口尺寸并截图；
- 检查顶部状态、GPS、起降、手动控制、航点表、执行进度、日志工具栏和状态栏；
- 检查离线灰态、在线可用态、ARMED 红色状态和 100 Hz 控制健康状态；
- 检查滚动区、分割器、表格和日志长行，无重叠或错位。

本次实际截图临时证据：

```text
/tmp/task04-qt-sitl-ready.png
/tmp/task04-qt-sitl-airborne.png
/tmp/task04-qt-sitl-landed.png
```

### 9.2 真实 Qt/SITL 闭环

验收脚本不是直接调用机载算法，而是创建真实 `GroundStationWindow`，触发 Qt 按钮信号，
经过现有 `GroundStationRosController` 与环境编排完成闭环。结果：

1. ROS 客户端 ready；
2. 仿真按钮启用并通过 Qt 信号启动 SITL/MAVROS/机载 C++/RViz；
3. 环境完成时飞控连接、租约、位姿和推力门控全部通过；
4. 起飞按钮启用，确认后成功起飞并进入机载 HOVER；
5. 依次点击前、后、左、右、上、下、左偏航、右偏航，八项均收到命令结果；
6. 点击悬停，机载 PD+DOB 接管；
7. 通过 Qt 数值框添加相对小位移航点，发送并完成；
8. 点击 LAND，飞行器落地并解除武装；
9. WARN 精确筛选得到 12 条可见事件；
10. 点击清理，控制权主动释放，受管进程全部结束；
11. 关闭窗口，异步安全退出完成；
12. 统一日志总线本轮累计接收 642 条事件。

代表性状态：

```text
仿真就绪：control_authority=true, local_position_valid=true,
           thrust_mode_verified=true, hover_throttle=0.374409,
           control_rate_hz=100.13, setpoint_conflict=false

航点完成：position=(0.289, 0.265, 0.612),
           target=(0.320, 0.250, 0.590),
           三维误差约 0.041 m,
           control_rate_hz=100.18, setpoint_conflict=false
```

`onboard_control.log` 的本轮段落明确记录：

- 起飞完成，机载 PD+DOB 进入悬停；
- 八次运动意图按成对增量回到零；
- PD+DOB 悬停接管；
- 1 个航点启动并完成；
- LAND 指令发送并解除武装；
- 控制权主动释放。

清理后的进程扫描没有 SITL、MAVROS、onboard_control、guided_sim 或 RViz 残留。

## 10. 历史 GUI 问题处理

结合 `MEMORY.md`、任务 03 报告和 GUI 选型报告，本轮处理了以下界面侧遗留：

- 按历史推荐采用 PySide6，而不是继续扩展 Tk 或默认引入 PyQt 许可约束；
- 保留任务 03 的自动 overlay 启动，未重新引入直接运行时缺少 `guided_interfaces` 的问题；
- 不再让工作线程跨线程操作 GUI；Qt 信号桥替代 Tk 轮询队列；
- 原 Tk 中飞控按钮从窗口创建开始始终可点击的问题改为统一状态门控；
- 原 Tk 没有集中日志、等级筛选和实时外部进程输出的问题已完成；
- 原 Tk 关闭路径会同步阻塞主线程的问题改为异步清理；
- 原 Tk 仅排除 `Entry` 焦点的键盘保护扩展到所有数值框、搜索框、日志框、下拉框和按钮；
- 航点、GPS 输入改用带边界的数值组件，减少非法文本和单位歧义；
- 命令结果仍来自机载权威协议，不用 GUI 自行假定按钮已经生效。

## 11. 使用说明

首次安装 Qt 依赖：

```bash
python3 -m pip install -r requirements-gui.txt
```

工作空间至少构建一次后直接启动：

```bash
python3 ground_station.py
```

推荐操作顺序：

1. 观察 ROS/机载/飞控/控制健康四个状态卡；
2. 启动本地仿真，或在安全条件下确认连接实机；
3. 等待活动提示报告环境完成，确认位置和推力均为 OK；
4. 起飞后切换“手动控制与遥测”页签，或编辑并确认航点；
5. 通过底部日志按等级和文本定位问题；
6. 降落并解除武装后断开/清理；
7. 退出窗口时等待异步清理完成，不强杀 Python 进程。

## 12. 已知限制与诚实说明

1. 本轮没有真实无人机，未验证实际串口、物理飞控、Odin/extnav、真实 Wi-Fi 和真机安全检查；
2. 900×650 能正常使用但需要滚动左右工作区；这是保留合理组件尺寸后的有意行为；
3. 外部进程没有携带标准等级标记的行按 INFO 处理，不通过消息关键词主观升级；
4. GUI 内存日志有 8000 条上限，完整长时日志应查看磁盘文件；
5. SITL 本轮记录 1 次 deadline miss、最大抖动约 5.09 ms，平均频率约 100 Hz，飞行、
   航点和清理未失败；这不是硬实时证明，且本轮按任务边界没有调整控制算法；
6. PySide6 需按 `requirements-gui.txt` 安装；本机 Ubuntu 仓库未提供对应 PySide6 系统包，
   本轮在已有 Python 环境中通过 pip 安装 6.11.1 验证；
7. 真机首次使用仍必须遵循任务 03 报告的拆桨/固定、独立急停、单轴低高度和断链验证顺序。

## 13. 验收对照

| task04 要求 | 状态 | 证据 |
|---|---|---|
| PySide/Qt 替换 Tkinter并完全移除 Tk | 完成 | 单 Qt 入口、旧 GUI 删除、生产代码无 Tk 引用 |
| 浅色、简明、硬朗工程风格 | 完成 | 统一低饱和主题、实际截图检查 |
| 原功能对应 | 完成 | 环境/GPS/起降/手动/航点/清理全部映射并实测 |
| 互斥和无效操作禁用 | 完成 | `derive_availability` 与状态矩阵测试 |
| 高危操作二次确认 | 完成 | 实机、原点、起降、航点、断开、飞行退出 |
| 实时统一日志与四级筛选 | 完成 | 642 条真实事件、WARN 精确筛选、进程 tee |
| 必要时扩展机载通信 | 无需 | 现有 `ControlStatus` 已覆盖所需信息，协议未改 |
| 不同分辨率与拖动 | 完成 | splitter/scroll，900×650 与 1600×1000 测试 |
| 模块化且保留单文件入口 | 完成 | `qt_ui/` 七个职责模块 + `ground_station.py` |
| 修复历史 GUI 问题 | 完成 | 状态门控、日志、线程退出、焦点、数值输入、布局 |
| 组件合理、无重叠错位 | 完成 | 多轮截图修正与尺寸回归 |
| 反复调试和功能验证 | 完成 | 14 Python/Qt + 5 ROS + 真实 Qt/SITL 全流程 |

在当前“无真机”的可验证范围内，task04 已完成。真机和真实网络相关指标继续保持待物理
联调状态，不作虚假完成声明。
