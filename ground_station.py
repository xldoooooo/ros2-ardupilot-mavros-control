#!/usr/bin/env python3
"""ROS2-ArduPilot PySide6/Qt 地面站单文件入口。"""

from __future__ import annotations

import os
import signal
import sys
from collections.abc import Callable
from pathlib import Path

from ground_station_core.bootstrap import (
    WorkspaceBootstrapError,
    ensure_workspace_environment,
)

# 覆盖终端断开、键盘中断、POSIX quit 与常规进程终止；SIGKILL 无法被捕获。
_TERMINATION_SIGNALS = (
    signal.SIGHUP,
    signal.SIGINT,
    signal.SIGQUIT,
    signal.SIGTERM,
)


def _configure_wayland_window_decorations() -> None:
    """在原生 Wayland 会话中选用带阴影留边的 Qt Adwaita 装饰。"""
    if (
        os.environ.get("XDG_SESSION_TYPE", "").casefold() == "wayland"
        and os.environ.get("WAYLAND_DISPLAY")
    ):
        # 尊重操作者显式指定的 Qt Wayland 装饰插件；只修复默认 bradient
        # 在 GNOME Wayland 下没有边框阴影的兼容差异。
        os.environ.setdefault("QT_WAYLAND_DECORATION", "adwaita")


def _install_termination_signal_handlers(
    callback: Callable[[str], None],
) -> Callable[[], None]:
    """把终端/进程管理器的可捕获终止信号转交 GUI，并返回恢复函数。"""
    previous_handlers: dict[signal.Signals, object] = {}

    def handle(signum: int, _frame: object) -> None:
        signal_name = signal.Signals(signum).name
        callback(signal_name)

    for signum in _TERMINATION_SIGNALS:
        previous_handlers[signum] = signal.getsignal(signum)
        signal.signal(signum, handle)

    def restore() -> None:
        """恢复进程进入 Qt 事件循环前的信号处理器。"""
        for signum, previous in previous_handlers.items():
            signal.signal(signum, previous)

    return restore


def main() -> None:
    """自动加载工作空间后创建 Qt 应用和地面站主窗口。"""
    try:
        ensure_workspace_environment(Path(__file__))
    except WorkspaceBootstrapError as error:
        print(f"[GS] {error}", file=sys.stderr, flush=True)
        raise SystemExit(2) from error

    # 启动脚本的父级兜底入口只扫描并终止本项目本机进程，不创建 Qt/ROS 客户端。
    if "--cleanup-local-processes" in sys.argv[1:]:
        from ground_station_core.process_manager import ProcessSupervisor

        report = ProcessSupervisor().terminate_all()
        if not report.success:
            print(
                "[GS] local process cleanup failed: "
                f"remaining={report.remaining}, errors={report.errors}",
                file=sys.stderr,
                flush=True,
            )
            raise SystemExit(5)
        return

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

    # Qt 在 QApplication 构造时选择 Wayland 客户端装饰，必须提前设置。
    _configure_wayland_window_decorations()
    try:
        from PySide6.QtCore import QTimer
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

    # Python 信号处理器只写入一个待处理槽；Qt 定时器再从主事件循环发起退出，
    # 避免终端 SIGHUP 直接终止进程并绕过窗口的环境清理线程。
    pending_signal: list[str | None] = [None]

    def remember_signal(signal_name: str) -> None:
        pending_signal[0] = signal_name

    restore_signal_handlers = _install_termination_signal_handlers(remember_signal)
    signal_timer = QTimer()
    signal_timer.setInterval(50)

    def dispatch_signal() -> None:
        signal_name = pending_signal[0]
        if signal_name is None:
            return
        pending_signal[0] = None
        window.request_external_shutdown(signal_name)

    signal_timer.timeout.connect(dispatch_signal)
    signal_timer.start()
    try:
        exit_code = application.exec()
    finally:
        # 覆盖 QApplication.quit、事件循环异常返回等未经过 closeEvent 的路径；
        # 正常安全退出已经缓存清理结果，因此这里不会重复操作后端。
        signal_timer.stop()
        window.finalize_process_exit()
        restore_signal_handlers()
    raise SystemExit(exit_code)


if __name__ == "__main__":
    main()
