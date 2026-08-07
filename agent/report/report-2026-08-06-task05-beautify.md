# Task 05：Qt 界面美化与修复执行报告

## 1. 任务结论

`agent/task/05-beautify.md` 的 8 项要求均已实现，并在现有 Qt 高层客户端边界内完成验证：

1. 七个 `QDoubleSpinBox` 的步进区改为非白色按钮背景，并使用明确的上下 SVG 箭头；
2. GPS 三项与航点四项数值框均忽略滚轮增减，滚轮事件可继续交给外层滚动页；
3. 删除占高的大标题/副标题，把环境、四项状态和动态消息压缩为单行状态带；
4. 日志改为 DEBUG、INFO、WARN、ERROR 四个独立复选框，可显示任意等级组合；
5. 连接与飞行动作、手动控制与遥测、航点任务改为左中右三栏同时显示；
6. 清空已完成航点时同步重置进度，并阻止机载旧快照把进度重新显示出来；
7. 新增“文件 / 设置 / 帮助”菜单和右上角“终端”按钮；
8. 主内容表面新增四边边框、圆角和阴影，同时保留系统原生窗口装饰。

未修改机载 C++ 控制算法、ROS 2 接口、飞行模式、安全门控或 `TODO.md` 内容。

## 2. 主要实现

### 2.1 数值输入

- `widgets.py` 新增 `NoWheelDoubleSpinBox`，统一覆盖 `wheelEvent()` 并忽略事件；
- `operations_panel.py` 和 `waypoint_panel.py` 的七个数值输入全部使用该控件；
- `theme.py` 为上下按钮定义独立背景、边界、悬停/按下状态和 12×8 矢量箭头；
- 新增 `qt_ui/assets/chevron-up.svg` 与 `chevron-down.svg`，避免系统桌面主题差异导致箭头缺失。

### 2.2 顶部与三栏布局

- 默认窗口由 1320×900 调整为 1600×920，最小尺寸为 1180×700；
- 删除原产品大标题，菜单栏承担应用级入口；
- 环境、机载链路、飞行器、控制模式、控制健康和最近动态合并为一条 38–42 px 状态带；
- `OperationsPanel` 不再使用页签：自身作为左栏，`manual_panel` 作为中栏，航点作为右栏；
- 三栏继续分别使用 `QScrollArea`，最小窗口下可独立纵向滚动；
- 水平与垂直 `QSplitter` 保留，并可通过“设置 → 恢复默认布局”恢复比例。

### 2.3 日志筛选

`LogPanel` 用四个默认勾选、互不排斥的复选框替换单选下拉框。筛选仍只读取
`LogEvent.level`，不会根据消息正文猜测等级；来源/正文搜索、累计计数、自动滚动和清空显示
行为保持不变。菜单可切换自动滚动、显示日志、恢复布局和清空日志显示。

### 2.4 航点进度修复

根因是清空本地列表后，100 ms 周期刷新仍会读取机载端残留的最后一次
`waypoint_count/index`，从而立即覆盖刚清空的进度条。

修复后 `WaypointPanel` 显式维护进度跟踪状态：

- 发送任务进入 running 状态时才启用进度跟踪；
- 最终结果到达后保留完成进度，供操作者查看；
- 清空列表调用 `reset_progress()`，设置为 `0/1`、`尚未执行` 并关闭跟踪；
- 后续旧机载快照即使仍报告 `1/1`，也不能重新激活进度；
- 该行为只清理 GUI 展示，不伪装成取消仍在机载端运行的任务。

### 2.5 菜单、终端与窗口表面

- “文件”：当前目录打开终端、退出；
- “设置”：日志自动滚动、显示实时日志、恢复默认布局、清空日志显示；
- “帮助”：键盘控制说明、关于地面站；
- 菜单栏右上角提供常驻“终端”按钮，并注册 `Ctrl+Alt+T`；
- 终端启动不经过 shell，依次寻找 `$TERMINAL`、`x-terminal-emulator`、
  `gnome-terminal`、`konsole`、`xfce4-terminal`、`mate-terminal`、`xterm`，通过
  `QProcess.startDetached()` 传入当前工作目录；
- `windowSurface` 使用一像素四边边框、7 px 圆角和 `QGraphicsDropShadowEffect`。

## 3. 自动化与构建验证

最终结果：

```text
source /opt/ros/jazzy/setup.bash
source install/setup.bash
python3 -m pytest -q
18 passed in 9.80s

flake8 ground_station_core/qt_ui tests/test_qt_gui.py \
  --select=E9,F63,F7,F82,E501 --max-line-length=88
通过

python3 -m compileall -q ground_station_core/qt_ui tests/test_qt_gui.py
通过

colcon build
Summary: 3 packages finished

colcon test --event-handlers console_direct+
colcon test-result --verbose
Summary: 5 tests, 0 errors, 0 failures, 0 skipped

git diff --check
通过
```

Qt 测试新增或加强以下覆盖：

- 七个数值框滚轮后数值不变，事件保持未接受；
- DEBUG 与 ERROR 可组合显示，同时隐藏 INFO 与 WARN；
- 1180×700 和 1800×1000 下三栏与日志同时存在且可用；
- 已完成进度在清空后不受旧 `1/1` 快照影响；
- 大标题不存在，状态条高度受控，菜单、终端按钮和阴影存在；
- 终端程序与当前工作目录参数经过 mock 验证，未调用 shell。

## 4. 实际渲染与真实 SITL 验证

### 4.1 渲染检查

分别生成 1600×920、1180×700 离线窗口截图，并检查三栏、步进箭头、状态带、日志复选框、
菜单、边缘和阴影。真实飞行阶段另生成任务完成与清空后的截图：

```text
/tmp/task05-beautify-1600x920.png
/tmp/task05-beautify-1180x700.png
/tmp/task05-real-waypoint-complete.png
/tmp/task05-real-waypoint-cleared.png
```

在 1180×700 下，每栏通过独立滚动区访问被折叠的纵向内容，没有恢复页签或隐藏任一主栏。

### 4.2 Qt → ROS 2 → SITL

真实回归创建 `GroundStationWindow`，从 GUI 工作流启动本地 SITL/MAVROS/机载 C++ 服务：

```text
ROS ready                         0.5 s
SITL operational                51.6 s
takeoff                          4.3 s
single waypoint completion       1.0 s
landing and disarm               3.1 s
```

关键缺陷复现与修复证据：

```text
清空前 GUI 进度：value=1, maximum=1, format='机载进度 1/1'
清空后 GUI 进度：value=0, maximum=1, format='尚未执行'
清空后机载旧快照：waypoint_index/count=1/1
```

随后 LAND 并解除武装；清理成功停止 4 个受管进程，`remaining=()`、`errors=()`；额外进程检查
未发现 `arducopter`、`mavros_node`、`onboard_control_node` 或 `MicroXRCEAgent` 残留。

## 5. 过程异常与限制

- 第一次全量 pytest 未 source 工作空间，`guided_interfaces` 类型支持动态库无法加载；按项目要求
  source `/opt/ros/jazzy/setup.bash` 与 `install/setup.bash` 后 18 项全部通过。这不是测试结果美化，
  未 source 的失败原因已定位并保留在本报告。
- 首次误用 `colcon build --symlink-install`，与已有普通构建缓存冲突；普通构建也因刚写入的缓存
  继续失败。仅将 `build/guided_interfaces` 移至可恢复备份
  `/tmp/task05-guided-build.gR39Dh/guided_interfaces` 后重新普通构建，三包成功；未删除源码或安装目录。
- 真实飞行使用 `QT_QPA_PLATFORM=offscreen`，RViz 因无显示后端退出 `-6`，但 SITL、MAVROS、
  机载服务和 Qt 飞行链路均完成；RViz 不是本次航点修复的控制依赖。
- 本轮负载下机载平均控制频率显示 100.00 Hz，但记录 1 次 deadline miss，截图中的最大抖动为
  323.31 ms；该机器不是硬实时环境，不能据此宣称硬实时性能。
- 未连接真实飞行器；实机网络、伴随计算机、物理飞控和终端窗口管理器的可见呈现仍需现场验收。

## 6. 文件清单

本任务修改：

- `ground_station_core/qt_ui/main_window.py`
- `ground_station_core/qt_ui/operations_panel.py`
- `ground_station_core/qt_ui/waypoint_panel.py`
- `ground_station_core/qt_ui/log_panel.py`
- `ground_station_core/qt_ui/widgets.py`
- `ground_station_core/qt_ui/theme.py`
- `tests/test_qt_gui.py`
- `MEMORY.md`

本任务新增：

- `ground_station_core/qt_ui/assets/chevron-up.svg`
- `ground_station_core/qt_ui/assets/chevron-down.svg`
- `agent/report/report-2026-08-06-task05-beautify.md`

开始本任务时，task04 的 Qt 重构文件尚未提交，`test_takeoff5.py` 仍是用户已有删除状态；本任务
在 task04 当前工作树上继续美化，没有恢复该文件，也没有创建提交。
