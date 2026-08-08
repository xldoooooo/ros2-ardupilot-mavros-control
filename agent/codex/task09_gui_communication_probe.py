#!/usr/bin/env python3
"""点击真实 Qt Wi-Fi 通讯检测按钮并审计零指令、状态与远端日志。"""

from __future__ import annotations

import json
import os
import sys
import time
from pathlib import Path

# 无显示终端也可运行真实 Qt 控件与信号；有桌面时可由调用方覆盖该变量。
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

# 直接从 agent/codex 执行时也加载仓库根目录中的地面站包。
PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from PySide6.QtWidgets import QApplication  # noqa: E402

from ground_station_core.environment import EnvironmentInitializer  # noqa: E402
from ground_station_core.event_log import EventLog  # noqa: E402
from ground_station_core.qt_ui.main_window import GroundStationWindow  # noqa: E402
from ground_station_core.qt_ui.theme import apply_theme  # noqa: E402
from ground_station_core.ros_controller import GroundStationRosController  # noqa: E402


class _NoProcessSupervisor:
    """Wi-Fi 诊断若触碰包命令或进程生命周期，立即让探针失败。"""

    @staticmethod
    def run_checked(*_args, **_kwargs) -> object:
        raise AssertionError("Wi-Fi 通讯检测不得执行进程命令")

    @staticmethod
    def terminate_all() -> object:
        raise AssertionError("Wi-Fi 通讯检测不得启动或停止任何进程")


def main() -> int:
    """运行实际 Wi-Fi 按钮信号路径并输出机器可读的安全审计摘要。"""
    application = QApplication.instance() or QApplication([])
    apply_theme(application)
    events = EventLog()
    controller = GroundStationRosController(
        source_id="task09-gui-communication-probe",
        event_log=events,
    )
    environment = EnvironmentInitializer(
        controller,
        supervisor=_NoProcessSupervisor(),
        event_log=events,
    )
    window = GroundStationWindow(
        event_log=events,
        ros_controller=controller,
        environment=environment,
        auto_start=False,
    )
    window.show()
    controller.start()
    window._refresh()
    window.operations.communication_test_button.click()

    deadline = time.monotonic() + 25.0
    while environment.busy and time.monotonic() < deadline:
        application.processEvents()
        time.sleep(0.02)
    application.processEvents()
    window._refresh()

    snapshot = controller.snapshot()
    remote_logs = [
        event
        for event in events.snapshot()
        if event.source.startswith("remote-rosout:")
    ]
    status_message_logs = [
        event
        for event in events.snapshot()
        if event.source == "onboard" and event.message == snapshot.status_message
    ]
    diagnostic_results = [
        event
        for event in events.snapshot()
        if "实机通讯链路检测" in event.message
    ]
    window.log_panel.poll()
    flight_buttons = {
        "takeoff": window.operations.takeoff_button.isEnabled(),
        "land": window.operations.land_button.isEnabled(),
        "hover": window.operations.hover_button.isEnabled(),
        "motion_any": any(
            button.isEnabled() for button in window.operations.motion_buttons.values()
        ),
        "waypoint_send": window.waypoints.send_button.isEnabled(),
    }
    failures: list[str] = []
    if environment.busy:
        failures.append("GUI Wi-Fi 通讯检测超时")
    if window._environment_active or window._connection_mode != "none":
        failures.append("Wi-Fi 检测错误建立了环境/控制会话")
    if not snapshot.onboard_available or snapshot.interface_version != "2.0":
        failures.append("GUI 未收到兼容机载聚合状态")
    if snapshot.control_authority or controller.control_enabled:
        failures.append("Wi-Fi 检测意外开启或取得控制租约")
    if any(flight_buttons.values()):
        failures.append(f"无环境会话时存在已启用飞行按钮：{flight_buttons}")
    if controller.results_after(0):
        failures.append("Wi-Fi 检测产生了命令结果，说明存在命令传输")
    if not diagnostic_results or "检测通过" not in diagnostic_results[-1].message:
        failures.append("GUI 日志未记录通讯检测通过指标")
    if snapshot.status_message and not status_message_logs:
        failures.append("GUI EventLog 未原样记录机载 ControlStatus.status_message")
    if snapshot.status_message and snapshot.status_message not in window.log_panel.displayed_text:
        failures.append("机载状态消息未原样出现在 GUI 日志面板")
    if remote_logs and not all(
        event.message in window.log_panel.displayed_text for event in remote_logs
    ):
        failures.append("远端 rosout 未原样出现在 GUI 日志面板")

    summary = {
        "result": "PASS" if not failures else "FAIL",
        "failures": failures,
        "environment_active": window._environment_active,
        "connection_mode": window._connection_mode,
        "onboard_available": snapshot.onboard_available,
        "interface_version": snapshot.interface_version,
        "fcu_connected": snapshot.connected,
        "armed": snapshot.armed,
        "lease_active": snapshot.lease_active,
        "lease_owner": snapshot.lease_owner,
        "control_authority": snapshot.control_authority,
        "controller_control_enabled": controller.control_enabled,
        "flight_buttons_enabled": flight_buttons,
        "command_results": len(controller.results_after(0)),
        "diagnostic_result": (
            diagnostic_results[-1].message if diagnostic_results else ""
        ),
        "remote_rosout_count": len(remote_logs),
        "status_message": snapshot.status_message,
    }

    # 仅停止本地常驻订阅客户端；无控制态时 release_control 是零传输返回。
    controller.stop()
    window._allow_close = True
    window.close()
    application.processEvents()
    print(json.dumps(summary, ensure_ascii=False, indent=2, sort_keys=True))
    return 1 if failures else 0


if __name__ == "__main__":
    raise SystemExit(main())
