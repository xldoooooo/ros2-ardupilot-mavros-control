# Task 07 部分撤销：集成终端回退

## 1. 结论

按用户反馈“终端效果不理想”，**撤销全部与集成终端相关的修改**，并**保留任务 07 中的日志改进**。

| 项 | 状态 |
| --- | --- |
| 底部 PTY 集成终端（`ConsolePanel` / `TerminalSession`） | 已删除 |
| 移除右上“终端”按钮 / 文件菜单外部终端 | 已恢复为任务 05 行为 |
| SITL/MAVROS 启动刷屏 INFO→DEBUG | **保留** |
| 日志“自动滚动”可关闭且不强制跳底 | **保留** |

## 2. 撤销内容

- 删除 `ground_station_core/qt_ui/console_panel.py`
- 删除 `ground_station_core/qt_ui/terminal_session.py`
- `main_window.py` 恢复 `LogPanel` 直挂底部 splitter；恢复 `_open_terminal()`、菜单“在当前目录打开终端”、右上“终端”按钮
- `theme.py` 移除 console 页签与 terminal 暗色样式
- 测试恢复外部终端入口用例，移除集成终端页签用例；保留 `test_log_auto_scroll_can_be_disabled`

## 3. 保留内容

- `process_manager.ProcessSupervisor._explicit_output_level`：Embedding / chatty 源启动噪音 → DEBUG
- `log_panel.LogPanel` 自动滚动修复：关闭时不 `setTextCursor(End)` 强制跳底
- `tests/test_event_log.py::test_sitl_mavros_startup_noise_is_demoted_to_debug`
- `tests/test_qt_gui.py::test_log_auto_scroll_can_be_disabled`

## 4. 说明

- 先前完整实现报告 `report-2026-08-07-task07-terminal.md` 仍保留作历史记录；当前代码以本撤销说明为准。
- 集成终端相关 TODO 重新打开，待后续更成熟方案再实现。
