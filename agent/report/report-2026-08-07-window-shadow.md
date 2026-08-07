# Qt 主窗口与子窗口外框阴影修复报告

## 1. 结论

已按用户追加要求，把原先只包围中央内容卡片的阴影改为覆盖整个地面站窗口，并统一处理
确认、警告、帮助、关于等消息子窗口：

- 完整主窗口具有四边轮廓、圆角和可见投影，范围包含菜单栏、主内容和状态栏；
- 主窗口使用透明阴影留边，不再依赖桌面主题是否提供系统阴影；
- 补齐自绘最小化、最大化/还原、关闭按钮；
- 菜单栏空白处可拖动窗口，双击可最大化/还原；
- 普通状态支持四条边和四个角的系统级缩放；
- 最大化时自动移除透明留边、圆角和阴影，避免屏幕边缘出现缝隙；
- 所有代码内 `QMessageBox` 入口统一使用带标题栏、关闭按钮、四边轮廓、圆角和阴影的
  `ShadowMessageBox`；
- 危险操作仍保留原按钮语义、默认取消和模态行为，没有降低安全确认标准。

## 2. 根因与实现

### 2.1 原问题

任务 05 首版把 `QGraphicsDropShadowEffect` 添加在 `windowSurface` 上，但 Qt 的菜单栏和
状态栏属于 `QMainWindow` 独立区域，不在该表面内部。因此截图虽能看到中央卡片边缘，整个
顶层窗口仍没有连续外轮廓和阴影。

### 2.2 完整主窗口外框

`GroundStationWindow` 现在使用：

- `FramelessWindowHint`：移除不受 QSS 控制的系统装饰；
- `WA_TranslucentBackground`：让顶层窗口四周 14 px 留边透明；
- `outerWindowFrame`：覆盖菜单、中央区和状态栏的统一背景表面；
- 1 px 深灰外轮廓、8 px 圆角；
- 30 px 模糊半径、3 px 纵向偏移的 `QGraphicsDropShadowEffect`；
- 内容区比外框再内缩 1 px，保证四条边不会被菜单或中央背景覆盖。

由于使用无原生标题栏，补充了完整窗口行为：

- 菜单栏右侧依次为终端、最小化、最大化/还原、关闭；
- 菜单空白区域调用 `startSystemMove()`；
- 透明外沿按坐标映射为 `Left/Right/Top/BottomEdge` 组合并调用
  `startSystemResize()`；
- 四边和四角显示对应的水平、垂直或对角缩放光标；
- 最大化时内容边距为 1 px、阴影关闭、圆角归零；还原后恢复 14 px 阴影留边。

关闭按钮仍进入原 `closeEvent()` 安全流程：飞行中需要确认，之后异步释放租约、清理本地
仿真并停止 ROS，不是直接终止进程。

### 2.3 消息子窗口

`widgets.py` 新增 `ShadowMessageBox`，保留 `QMessageBox` 的图标、标准按钮、默认按钮和
返回值，只替换外观层：

- 无原生装饰且背景透明；
- 14 px 阴影留边；
- 1 px 四边轮廓、8 px 圆角和 28 px 投影；
- 自绘标题栏、标题文字和关闭按钮；
- 标题栏可调用 `startSystemMove()` 拖动；
- 最小可读尺寸为 430×180，长文本按 `sizeHint()` 增加高度；
- 延迟锁定最终尺寸，避免 `QMessageBox` 在显示后再次收缩而裁切正文或按钮。

原静态 `QMessageBox.information/about/warning` 和危险确认框均改由 `_message_box()` /
`_show_notice()` 创建。代码搜索确认没有绕过统一外框的其他 `QMessageBox` 构造入口。

## 3. 验证结果

### 3.1 自动化

```text
source /opt/ros/jazzy/setup.bash
source install/setup.bash
python3 -m pytest -q
20 passed in 14.07s

colcon test --event-handlers console_direct+
colcon test-result --verbose
Summary: 5 tests, 0 errors, 0 failures, 0 skipped

python3 ground_station.py --check-environment
[GS] workspace environment OK: guided_interfaces + rclpy available

python3 -m compileall -q ground_station.py ground_station_core tests
通过

flake8 ground_station_core/qt_ui tests/test_qt_gui.py \
  --select=E9,F63,F7,F82,E501 --max-line-length=88
通过

git diff --check
通过
```

新增回归覆盖：

- 主窗口确实使用 frameless 与透明背景；
- 外框几何精确位于四周 14 px 阴影留边内部；
- 外框投影存在，旧中央卡片不再重复投影；
- 最小化、最大化、关闭按钮可见；
- 左上与右下坐标正确映射为对应对角缩放边；
- 最大化关闭阴影并移除留边，还原后恢复；
- 消息框具有 frameless、透明背景、标题、关闭按钮、完整阴影表面；
- 消息框保留默认 Cancel，并达到不裁切的最小尺寸。

### 3.2 视觉证据

使用与生产相同 QSS 在 Qt offscreen 平台渲染并人工检查：

```text
/tmp/task05-main-outer-shadow-v2.png
/tmp/task05-dialog-shadow-v2.png
```

主窗口截图可见投影连续包围菜单栏、左右边、状态栏和底边；确认起飞子窗口可见独立外轮廓、
阴影、自绘标题栏、危险图标、完整正文和“确认执行 / 取消”按钮。

## 4. 范围与限制

- 本轮只修改 Qt 窗口装饰、消息框外观和相应测试，未修改 ROS 协议、状态门控、机载控制、
  航点逻辑或 `TODO.md`；因此没有重复执行一次完整飞行，改用全量 Python 与 ROS 测试验证。
- 阴影由 Qt 自身渲染，不依赖 GNOME/KDE 窗口管理器；不同桌面合成器的透明边缘抗锯齿可能
  略有差异，但轮廓和投影几何不依赖系统主题。
- 系统文件选择器等未来可能新增的原生对话框不在当前代码中；当前所有提示/确认类子窗口已统一。
- task04/task05 尚未提交的工作树和用户已有 `test_takeoff5.py` 删除状态均原样保留，本轮未提交。

## 5. 文件变更

- `ground_station_core/qt_ui/main_window.py`
- `ground_station_core/qt_ui/widgets.py`
- `ground_station_core/qt_ui/theme.py`
- `tests/test_qt_gui.py`
- `MEMORY.md`
- `agent/report/report-2026-08-07-window-shadow.md`
