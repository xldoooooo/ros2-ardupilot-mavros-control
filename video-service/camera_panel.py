#!/usr/bin/env python3
"""可由地面站启动、也可单独运行的摄像头配置面板入口。"""

from __future__ import annotations

import sys

from PySide6.QtWidgets import QApplication

from camera_app.panel import (
    DESKTOP_APPLICATION_NAME,
    CameraPanelWindow,
    PANEL_STYLE_SHEET,
)


def main() -> int:
    """创建独立Qt事件循环；退出不停止摄像头后台。"""
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
