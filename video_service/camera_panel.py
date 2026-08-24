#!/usr/bin/env python3
"""可由地面站启动、也可单独运行的摄像头配置面板入口。"""

from __future__ import annotations

import os
import sys
from pathlib import Path


# 独立脚本的 sys.path 默认只有 video_service/；补入项目根以复用地面端窗口外框。
_PROJECT_ROOT = Path(__file__).resolve().parent.parent
if str(_PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(_PROJECT_ROOT))


def _configure_wayland_window_decorations() -> None:
    """在原生 Wayland 会话中选用带阴影留边的 Qt Adwaita 装饰。"""
    if (
        os.environ.get("XDG_SESSION_TYPE", "").casefold() == "wayland"
        and os.environ.get("WAYLAND_DISPLAY")
    ):
        # 不覆盖命令行或桌面环境已经明确选择的装饰插件。
        os.environ.setdefault("QT_WAYLAND_DECORATION", "adwaita")


def main() -> int:
    """创建独立Qt事件循环；退出不停止摄像头后台。"""
    # 装饰插件必须在 QApplication 构造前选择；模块导入也放在此后，
    # 避免未来 Qt 初始化副作用提前固化默认 bradient 装饰。
    _configure_wayland_window_decorations()
    from camera_app.panel import (
        DESKTOP_APPLICATION_NAME,
        PANEL_STYLE_SHEET,
        CameraPanelWindow,
    )
    from PySide6.QtWidgets import QApplication

    # GNOME Dock 会读取这两个 Qt 字段；使用 ASCII 名称避免桌面编码误判。
    QApplication.setApplicationName(DESKTOP_APPLICATION_NAME)
    QApplication.setApplicationDisplayName(DESKTOP_APPLICATION_NAME)
    application = QApplication(sys.argv)
    application.setStyle("Fusion")
    application.setStyleSheet(PANEL_STYLE_SHEET)
    window = CameraPanelWindow()
    window.show()
    return application.exec()


if __name__ == "__main__":
    raise SystemExit(main())
