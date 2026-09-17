"""upstream 闭环的 Qt 交互回归；使用共享替身，不连接实机。"""

from __future__ import annotations

from tests.support.qt import (
    _application,
    _close_window,
    _operational_snapshot,
    _window,
)
import math
from dataclasses import replace
from PySide6.QtCore import Qt
from PySide6.QtTest import QTest
from ground_station_core.models import CommandResult, FlightMode
from ground_station_core.upstream.mapping import parse_command


def test_land_button_and_upstream_land_bypass_pending_and_workflow_locks() -> None:
    """LAND 请求未终结或 UI 工作流正忙时仍可安全重发。"""
    landing = replace(_operational_snapshot(armed=True), active_mode=FlightMode.LAND)
    window, ros = _window(landing)
    try:
        window._environment_active = True
        window._connection_mode = "simulation"
        window._workflow_busy = True
        window._pending_commands.add("land")
        window._refresh()

        assert window.operations.land_button.isEnabled()
        window._land()
        assert ros.calls == [("land", None)]
        assert window.operations.land_button.isEnabled()

        command = parse_command({"clientNo": "UAV01001", "commandNo": "06"}, "UAV01001")
        window._handle_upstream_command(command)
        assert ros.calls == [("land", None), ("land", None)]
    finally:
        _close_window(window)


def test_inspection_standby_locks_takeoff_until_sequence_really_completes() -> None:
    """03 降落后的延时/入库阶段不得与新起飞或巡检并发。"""
    window, ros = _window(_operational_snapshot(armed=False))
    execute = parse_command({"clientNo": "UAV01001", "commandNo": "03"}, "UAV01001")
    try:
        window._initialize_simulation()
        window.waypoints.replace_waypoints(((1.0, 0.0, 0.5, 0.0),), "组合锁测试")
        window._handle_upstream_command(execute)

        ros.results.append(CommandResult(1, 1, "takeoff", True, "起飞完成"))
        ros.current_snapshot = _operational_snapshot(armed=True)
        window._refresh()
        assert ros.calls[-1][0] == "waypoints"

        ros.results.append(CommandResult(2, 2, "waypoints", True, "巡检完成"))
        window._refresh()
        assert ros.calls[-1][0] == "land"
        assert window._upstream_sequence is not None
        assert window._upstream_sequence.ticket == 3

        # 组合 LAND 中人工重发会覆盖机载 ticket，GUI 必须跟随新 ticket，
        # 否则落地后会永久停在“组合动作正在执行”。
        window._land()
        assert window._upstream_sequence.ticket == 4
        assert ("landing", 4) in window._upstream.calls

        ros.results.append(CommandResult(3, 4, "land", True, "降落完成"))
        ros.current_snapshot = _operational_snapshot(armed=False)
        window._refresh()
        assert window._upstream_sequence is not None
        assert window._upstream_sequence.phase == "standby"
        assert not window.operations.takeoff_button.isEnabled()

        calls_before_retry = len(ros.calls)
        window._handle_upstream_command(execute)
        assert len(ros.calls) == calls_before_retry
        assert "组合动作正在执行" in window.activity_banner.message_label.text()

        window._upstream_sequence.standby_not_before = 0.0
        window._refresh()
        assert window._upstream_sequence is None
        window._refresh()
        assert window.operations.takeoff_button.isEnabled()
    finally:
        _close_window(window)


def test_upstream_commands_route_only_to_active_environment_and_sync_gui() -> None:
    """03/05 按可靠终态串行起飞、航点、降落和待机阶段。"""
    window, ros = _window(_operational_snapshot(armed=False))
    upstream = window._upstream
    task = parse_command(
        {
            "clientNo": "UAV01001",
            "commandNo": "02",
            "taskPoints": [
                {
                    "index": 8,
                    "x": 1.0,
                    "y": 2.0,
                    "z": 1.5,
                    "forwardAngle": 90,
                    "cameraAngle": 1495,
                    "photoNo": 1,
                }
            ],
        },
        "UAV01001",
    )
    try:
        # 上位机可独立连接，但没有仿真/实机会话时不能产生任何 ROS 动作。
        window._handle_upstream_command(task)
        assert not window.waypoints.waypoints
        assert not ros.calls
        assert any(
            "尚未启动仿真或连接实机" in event.message
            for event in window.event_log.snapshot()
        )

        window._initialize_simulation()
        window._handle_upstream_command(task)
        assert window.waypoints.table.rowCount() == 1
        assert window.waypoints.waypoints[0][:3] == (1.0, 2.0, 1.5)
        assert math.isclose(window.waypoints.waypoints[0][3], math.pi / 2.0)
        assert ("staged", None) in upstream.calls
        ignored_logs = [
            event
            for event in window.event_log.snapshot()
            if "photoNo 已随航点" in event.message
        ]
        assert len(ignored_logs) == 1

        execute = parse_command({"clientNo": "UAV01001", "commandNo": "03"}, "UAV01001")
        window._handle_upstream_command(execute)
        assert ros.calls[-1] == (
            "takeoff",
            window.operations.takeoff_altitude(),
        )
        assert not any(call[0] == "waypoints" for call in ros.calls)
        assert window._upstream_sequence is not None
        assert window._upstream_sequence.phase == "takeoff"

        return_home = parse_command(
            {"clientNo": "UAV01001", "commandNo": "05"}, "UAV01001"
        )
        calls_before_return = len(ros.calls)
        window._handle_upstream_command(return_home)
        assert len(ros.calls) == calls_before_return
        assert "尚未起飞" in window.activity_banner.message_label.text()

        # 只有起飞可靠终态到达后才下发巡检航点。
        ros.results.append(CommandResult(1, 1, "takeoff", True, "起飞完成"))
        ros.current_snapshot = replace(
            _operational_snapshot(armed=True), active_command_sequence=1
        )
        window._refresh()
        assert ros.calls[-1][0] == "waypoints"
        assert ros.calls[-1][1][4] == ("1",)
        assert window._waypoint_running
        assert ("mission", (2, "inspection", (8,))) in upstream.calls
        assert not window.waypoints.send_button.isEnabled()

        # 巡检航点终态后立即在末点进入 LAND。
        ros.results.append(CommandResult(2, 2, "waypoints", True, "巡检航点完成"))
        ros.current_snapshot = replace(
            ros.current_snapshot,
            active_mode=FlightMode.HOVER,
            active_command_sequence=2,
        )
        window._refresh()
        assert ros.calls[-1][0] == "land"
        assert ("landing", 3) in upstream.calls

        # 巡检降落后的 60 s 参数化等待不能被提前释放。
        ros.results.append(CommandResult(3, 3, "land", True, "降落完成"))
        ros.current_snapshot = _operational_snapshot(armed=False)
        window._refresh()
        assert window._upstream_sequence is not None
        assert window._upstream_sequence.phase == "standby"
        assert window._upstream_sequence.standby_not_before > 0.0
        window._upstream_sequence.standby_not_before = 0.0
        window._refresh()
        assert window._upstream_sequence is None
        assert ("standby_release", None) in upstream.calls

        # 05 保留当前三项组合，返回原点起飞高度后降落。
        ros.current_snapshot = _operational_snapshot(armed=True)
        window._refresh()
        window.operations.takeoff_altitude_input.setValue(0.7)
        window._handle_upstream_command(return_home)
        assert ros.calls[-1][0] == "waypoints"
        return_points = ros.calls[-1][1][0]
        assert return_points[0][:3] == (0.0, 0.0, 0.7)
        assert window.waypoints.waypoints == return_points
        assert ("mission", (4, "return", (1,))) in upstream.calls
        ros.results.append(CommandResult(4, 4, "waypoints", True, "返航点完成"))
        window._refresh()
        assert ros.calls[-1][0] == "land"
        assert ("landing", 5) in upstream.calls

        ros.results.append(CommandResult(5, 5, "land", True, "返航降落完成"))
        ros.current_snapshot = _operational_snapshot(armed=False)
        window._refresh()
        assert window._upstream_sequence is None

        ros.current_snapshot = _operational_snapshot(armed=True)
        window._refresh()
        emergency_land = parse_command(
            {"clientNo": "UAV01001", "commandNo": "07"}, "UAV01001"
        )
        window._handle_upstream_command(emergency_land)
        assert ros.calls[-1][0] == "land"
        assert ("landing", 6) in upstream.calls
        assert window._upstream_sequence is not None
        assert window._upstream_sequence.kind == "emergency"
        assert window.operations.land_button.isEnabled()
    finally:
        _close_window(window)


def test_upstream_command_06_reuses_normal_land_path() -> None:
    """新增 06 接口复用原降落门控与可靠 ticket，不建立第二套控制实现。"""
    window, ros = _window(_operational_snapshot(armed=True))
    upstream = window._upstream
    try:
        window._initialize_simulation()
        normal_land = parse_command(
            {"clientNo": "UAV01001", "commandNo": "06"}, "UAV01001"
        )
        window._handle_upstream_command(normal_land)
        assert ros.calls[-1][0] == "land"
        assert ("landing", 1) in upstream.calls
        assert window.operations.land_button.isEnabled()
    finally:
        _close_window(window)


def test_emergency_land_keeps_standby_blocked_until_hangar_threshold() -> None:
    """07 降落在机库外时不释放 01，进入 XYZ 阈值后才释放。"""
    window, ros = _window(replace(_operational_snapshot(armed=True), x=2.0))
    upstream = window._upstream
    try:
        window._initialize_simulation()
        emergency_land = parse_command(
            {"clientNo": "UAV01001", "commandNo": "07"}, "UAV01001"
        )
        window._handle_upstream_command(emergency_land)
        assert window._upstream_sequence is not None
        assert window._upstream_sequence.kind == "emergency"

        ros.results.append(CommandResult(1, 1, "land", True, "机库外降落完成"))
        ros.current_snapshot = replace(_operational_snapshot(armed=False), x=2.0)
        window._refresh()
        assert window._upstream_sequence is not None
        assert window._upstream_sequence.phase == "standby"
        releases = upstream.calls.count(("standby_release", None))

        ros.current_snapshot = replace(ros.current_snapshot, x=0.5)
        window._refresh()
        assert window._upstream_sequence is None
        assert upstream.calls.count(("standby_release", None)) == releases + 1
    finally:
        _close_window(window)


def test_low_power_and_abnormal_states_drive_return_and_recovery_sequences() -> None:
    """WebSocket 在线仿真才执行低电量/异常返航与恢复组合。"""
    low_window, low_ros = _window(_operational_snapshot(armed=True))
    low_upstream = low_window._upstream
    try:
        low_upstream.connected = True
        low_window._initialize_simulation()
        low_upstream.low_power_return_required = True
        low_window._refresh()

        waypoint_calls = [call for call in low_ros.calls if call[0] == "waypoints"]
        assert len(waypoint_calls) == 1
        assert waypoint_calls[0][1][0][0][:3] == (0.0, 0.0, 0.3)
        assert low_window._upstream_sequence is not None
        assert low_window._upstream_sequence.kind == "low_power"
        assert ("mission", (1, "return", (1,))) in low_upstream.calls
    finally:
        _close_window(low_window)

    abnormal_snapshot = replace(
        _operational_snapshot(armed=True),
        vehicle_abnormal=True,
        vehicle_abnormal_reason="航点连续入点失败 10 次",
    )
    window, ros = _window(abnormal_snapshot)
    upstream = window._upstream
    try:
        upstream.connected = True
        window._initialize_simulation()
        assert ros.calls[-1][0] == "waypoints"
        assert window.mode_badge.value_label.text() == "无人机状态异常"
        assert window._upstream_sequence is not None
        assert window._upstream_sequence.kind == "abnormal"

        ros.results.append(CommandResult(1, 1, "waypoints", True, "异常返航点完成"))
        window._refresh()
        assert ros.calls[-1][0] == "land"

        ros.results.append(CommandResult(2, 2, "land", True, "异常返航降落完成"))
        ros.current_snapshot = replace(
            _operational_snapshot(armed=False),
            vehicle_abnormal=True,
            vehicle_abnormal_reason=abnormal_snapshot.vehicle_abnormal_reason,
        )
        window._refresh()
        assert ros.calls[-1][0] == "clear_abnormal"
        assert window._upstream_sequence is not None
        assert window._upstream_sequence.phase == "clear_abnormal"

        ros.results.append(
            CommandResult(3, 3, "clear_abnormal", True, "异常状态已清除")
        )
        ros.current_snapshot = _operational_snapshot(armed=False)
        window._refresh()
        assert window._upstream_sequence is None
        assert ("standby_release", None) in upstream.calls
    finally:
        _close_window(window)


def test_disconnected_upstream_disables_low_power_and_abnormal_auto_return() -> None:
    """WebSocket 断线时仿真和实机都不得由两类状态下发飞行命令。"""
    low_window, low_ros = _window(_operational_snapshot(armed=True))
    low_upstream = low_window._upstream
    try:
        low_window._initialize_simulation()
        low_upstream.low_power_return_required = True
        low_window._refresh()

        assert low_upstream.snapshot().connected is False
        assert not any(call[0] in {"waypoints", "land"} for call in low_ros.calls)
        assert low_window._upstream_sequence is None
    finally:
        _close_window(low_window)

    abnormal_snapshot = replace(
        _operational_snapshot(armed=True),
        vehicle_abnormal=True,
        vehicle_abnormal_reason="航点连续入点失败 10 次",
    )
    abnormal_window, abnormal_ros = _window(abnormal_snapshot)
    try:
        abnormal_window._environment_active = True
        abnormal_window._connection_mode = "hardware"
        abnormal_window._refresh()

        assert abnormal_window._upstream.snapshot().connected is False
        assert not any(call[0] in {"waypoints", "land"} for call in abnormal_ros.calls)
        assert abnormal_window._upstream_sequence is None
    finally:
        _close_window(abnormal_window)


def test_connected_hardware_only_reports_low_power_and_abnormal_states() -> None:
    """WebSocket 在线实机两类触发只告警一次，TODO 分支不操作飞机。"""
    snapshot = replace(
        _operational_snapshot(armed=True),
        vehicle_abnormal=True,
        vehicle_abnormal_reason="航点连续入点失败 10 次",
    )
    window, ros = _window(snapshot)
    upstream = window._upstream
    try:
        upstream.connected = True
        upstream.low_power_return_required = True
        window._environment_active = True
        window._connection_mode = "hardware"

        window._refresh()
        window._refresh()

        assert not any(call[0] in {"waypoints", "land"} for call in ros.calls)
        assert window._upstream_sequence is None
        messages = [event.message for event in window.event_log.snapshot()]
        assert sum("实机低电量" in message for message in messages) == 1
        assert sum("实机无人机状态异常" in message for message in messages) == 1
        assert "未执行自动返航或降落" in window.activity_banner.message_label.text()
    finally:
        _close_window(window)


def test_upstream_panel_exposes_configuration_mapping_raw_frames_and_json() -> None:
    """面板同步新语义，支持原始帧搜索并在关闭时保存输入。"""
    window, _ros = _window(_operational_snapshot(armed=False))
    upstream = window._upstream
    try:
        upstream.journal.append("RX", '{"type":"SYSTEM"}')
        QTest.mouseClick(window.upstream_panel_button, Qt.MouseButton.LeftButton)
        _application().processEvents()
        panel = window._upstream_panel
        assert panel is not None and panel.isVisible()
        assert [panel.tabs.tabText(index) for index in range(panel.tabs.count())] == [
            "连接配置",
            "指令映射",
            "原始报文",
            "JSON 格式",
        ]
        assert panel.url_input.text() == "ws://127.0.0.1:8581/ws"
        assert panel.client_no_input.text() == "UAV01001"
        assert panel.mapping_table.rowCount() == 6
        panel._poll()
        assert '{"type":"SYSTEM"}' in panel.raw_log.toPlainText()
        panel.raw_search_input.setText("SYSTEM")
        QTest.mouseClick(panel.raw_search_button, Qt.MouseButton.LeftButton)
        assert panel.raw_log.textCursor().selectedText() == "SYSTEM"
        json_guide = panel.json_text.toPlainText()
        assert "0C: WebSocket 在线时上报低电量" in json_guide
        assert "在线实机当前只提示地面站（TODO）" in json_guide
        assert "断线不触发动作" in json_guide
        assert "03: 起飞至设定高度" in json_guide
        assert "08: 仅巡检航点全部完成时发送" in json_guide
        assert "无人机异常: WebSocket 在线仿真执行返航降落" in json_guide
        for command_no in ("01", "02", "03", "05", "06", "07"):
            assert f"接收命令 {command_no}（BROADCAST）" in json_guide
        for status_no in (
            "01",
            "02",
            "03",
            "05",
            "07",
            "08",
            "09",
            "0A",
            "0B",
            "0C",
        ):
            assert f"状态 {status_no}" in json_guide
        panel.url_input.setText("ws://10.0.0.8:8581/ws")
        panel.client_no_input.setText("UAV02002")
        panel.close()
        assert not any(call[0] == "disconnect" for call in upstream.calls)
        assert ("save", ("ws://10.0.0.8:8581/ws", "UAV02002")) in upstream.calls
    finally:
        _close_window(window)


def test_upstream_panel_is_independent_and_minimizable() -> None:
    """通讯面板保持独立层级，并自行绘制可缩放、可最小化的窗口阴影。"""
    window, _ros = _window(_operational_snapshot(armed=False))
    try:
        QTest.mouseClick(window.upstream_panel_button, Qt.MouseButton.LeftButton)
        _application().processEvents()
        panel = window._upstream_panel
        assert panel is not None and panel.isVisible()
        assert panel.parentWidget() is None
        assert panel.windowType() == Qt.WindowType.Window
        assert panel.windowFlags() & Qt.WindowType.WindowMinimizeButtonHint
        assert panel.windowFlags() & Qt.WindowType.FramelessWindowHint
        assert not panel.windowFlags() & Qt.WindowType.WindowStaysOnTopHint
        assert panel.testAttribute(Qt.WidgetAttribute.WA_TranslucentBackground)
        assert panel.windowHandle() is not None
        assert panel.windowHandle().transientParent() is None
        assert panel.outer_window_frame.graphicsEffect() is panel._window_shadow
        assert panel._window_shadow.blurRadius() == 30.0
        assert panel._window_shadow.offset().y() == 3.0
        assert panel.contentsMargins().left() == panel._SHADOW_MARGIN + 1
        assert panel.outer_window_frame.geometry() == panel.rect().adjusted(
            panel._SHADOW_MARGIN,
            panel._SHADOW_MARGIN,
            -panel._SHADOW_MARGIN,
            -panel._SHADOW_MARGIN,
        )
        shadow_pixel = (
            panel.grab()
            .toImage()
            .pixelColor(panel._SHADOW_MARGIN - 4, panel.height() // 2)
        )
        assert 0 < shadow_pixel.alpha() < 255

        panel.showMaximized()
        _application().processEvents()
        assert panel.isMaximized()
        assert panel.contentsMargins().left() == 1
        assert not panel._window_shadow.isEnabled()
        assert panel.maximize_button.toolTip() == "还原"
        panel.showNormal()
        _application().processEvents()
        assert panel.contentsMargins().left() == panel._SHADOW_MARGIN + 1
        assert panel._window_shadow.isEnabled()

        panel.showMinimized()
        _application().processEvents()
        assert panel.isMinimized()

        QTest.mouseClick(window.upstream_panel_button, Qt.MouseButton.LeftButton)
        _application().processEvents()
        assert panel.isVisible()
        assert not panel.isMinimized()
    finally:
        _close_window(window)
