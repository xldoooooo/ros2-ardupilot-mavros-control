#!/usr/bin/env python3
"""可独立运行的地面媒体文件管理入口；退出不影响媒体后台。"""

from __future__ import annotations

import sys
import argparse
from pathlib import Path

# 独立脚本补入项目根，与摄像头面板采用相同导入方式。
_PROJECT_ROOT = Path(__file__).resolve().parent.parent
if str(_PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(_PROJECT_ROOT))


def main() -> int:
    """启动独立文件窗口，复用项目 Python 与 Qt 装饰配置。"""
    parser = argparse.ArgumentParser(description="地面媒体文件管理")
    parser.add_argument("--aircraft-id", default="")
    parser.add_argument("--session-id", default="")
    arguments, qt_arguments = parser.parse_known_args()
    from video_service.camera_panel import _configure_wayland_window_decorations

    _configure_wayland_window_decorations()
    from PySide6.QtWidgets import QApplication
    from video_service.file_manager_app import FileManagerWindow, PANEL_STYLE_SHEET

    QApplication.setApplicationName("ROS 2 ArduPilot Media Files")
    QApplication.setApplicationDisplayName("ROS 2 ArduPilot Media Files")
    application = QApplication([sys.argv[0], *qt_arguments])
    application.setStyle("Fusion")
    application.setStyleSheet(PANEL_STYLE_SHEET)
    window = FileManagerWindow(
        aircraft_id=arguments.aircraft_id, session_id=arguments.session_id
    )
    window.show()
    return application.exec()


if __name__ == "__main__":
    raise SystemExit(main())
