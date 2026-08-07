#!/usr/bin/env python3
"""ROS2-ArduPilot PySide6/Qt 地面站单文件入口。"""

from __future__ import annotations

import sys
from pathlib import Path

from ground_station_core.bootstrap import (
    WorkspaceBootstrapError,
    ensure_workspace_environment,
)


def main() -> None:
    """自动加载工作空间后创建 Qt 应用和地面站主窗口。"""
    try:
        ensure_workspace_environment(Path(__file__))
    except WorkspaceBootstrapError as error:
        print(f"[GS] {error}", file=sys.stderr, flush=True)
        raise SystemExit(2) from error

    # 该只读诊断入口供安装检查和未 source shell 回归使用，不创建窗口。
    if "--check-environment" in sys.argv[1:]:
        import guided_interfaces.msg  # noqa: F401
        import rclpy

        from ground_station_core.ros_controller import GroundStationRosController

        controller = GroundStationRosController(source_id="environment-check")
        controller.start(timeout=5.0)
        if not controller.ready:
            error = controller.error or "ROS 2 客户端未就绪"
            controller.stop()
            print(f"[GS] workspace environment check failed: {error}", file=sys.stderr)
            raise SystemExit(3)
        controller.stop()
        rclpy_version = (
            rclpy.__version__ if hasattr(rclpy, "__version__") else "available"
        )
        print(
            "[GS] workspace environment OK: "
            f"guided_interfaces + rclpy {rclpy_version}"
        )
        return

    try:
        from PySide6.QtWidgets import QApplication
    except ModuleNotFoundError as error:
        print(
            "[GS] 缺少 PySide6。请执行："
            "python3 -m pip install -r requirements-gui.txt",
            file=sys.stderr,
            flush=True,
        )
        raise SystemExit(4) from error

    from ground_station_core.qt_ui import GroundStationWindow
    from ground_station_core.qt_ui.theme import apply_theme

    application = QApplication(sys.argv)
    application.setApplicationName("ArduPilot ROS 2 Ground Station")
    application.setOrganizationName("ros2-ardupilot-mavros-control")
    apply_theme(application)
    window = GroundStationWindow()
    window.show()
    raise SystemExit(application.exec())


if __name__ == "__main__":
    main()
