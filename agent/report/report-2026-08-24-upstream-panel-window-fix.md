# 上位机通讯面板窗口行为修复简报

日期：2026-08-24

## 任务目标

修复地面站“上位机通讯面板”始终压在主窗口上方，以及系统标题栏右上角最小化按钮无法正常
工作的两个窗口行为问题。

本任务只修改地面端 Qt 窗口与自动化测试，没有连接实机、申请飞行控制权、解锁或起飞。

## 根因

通讯面板原本是带主窗口父级的 `QDialog`。Qt 会为这种窗口建立指向主窗口的 transient parent，
Linux 窗口管理器通常因此强制对话框保持在主窗口上方。同时，`QDialog` 的默认窗口标志不包含
`WindowMinimizeButtonHint`；即使桌面主题绘制了最小化控件，也不能保证该操作有效。

## 修改内容

- 将 `UpstreamCommunicationPanel` 从带父级的 `QDialog` 改为无父级的普通顶层
  `QWidget(..., Qt.Window)`，消除 transient 层级关系并启用正常的原生最小化窗口提示。
- 保持面板非模态、可关闭复用且关闭不影响 WebSocket 连接的既有业务语义。
- 面板最小化后再次点击主窗口“上位机通讯面板”按钮时，使用 `showNormal()` 恢复窗口。
- 主窗口最终退出时显式关闭并延迟销毁独立面板，避免无父级顶层窗口继续维持 Qt 事件循环。
- 新增 GUI 回归，检查无父级、`Qt.Window` 类型、最小化提示、无置顶标志、无 transient parent、
  最小化成功和入口按钮恢复成功。

## 变更文件

- `ground_station_core/qt_ui/upstream_panel.py`
- `ground_station_core/qt_ui/main_window.py`
- `tests/test_qt_gui.py`
- `MEMORY.md`

## 验证结果

1. `git diff --check`：通过。
2. 面板定向回归：`2 passed, 43 deselected`。
3. `tests/test_qt_gui.py`：`45 passed`。
4. 首次直接运行完整测试时未加载项目 ROS overlay，结果为 `160 passed, 7 failed`；7 项失败均为
   `ModuleNotFoundError: guided_interfaces` 或由其导致的 ROS 客户端未就绪，属于测试环境缺失，
   不是本次功能回归。
5. 按项目基线加载 `/opt/ros/jazzy/setup.bash` 与 `install/setup.bash` 后重新运行完整
   `tests/`：`167 passed in 39.37s`。

## 未覆盖边界

自动化测试使用 Qt offscreen 平台，已验证窗口类型、窗口标志、transient 关系和最小化状态切换。
不同 Linux 桌面窗口管理器的标题栏绘制外观仍建议在目标上位机桌面做一次人工点击确认；当前无
已知未实现功能或测试失败。
