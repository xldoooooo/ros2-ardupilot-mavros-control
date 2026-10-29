"""可选后端的缺失回退、权威显示与 SetBool 异步传输；不连接实机。"""

from dataclasses import replace
import json
import time
from types import SimpleNamespace

import pytest

from ground_station_core.models import AvoidanceBackendSnapshot, CommandResult, FlightMode
from ground_station_core.ros_controller import GroundStationRosController
from tests.support.qt import _close_window, _operational_snapshot, _window


def _backend(**changes) -> AvoidanceBackendSnapshot:
    """构造具备可选模块但未探板的 CPU 权威状态。"""
    return replace(AvoidanceBackendSnapshot(
        manager_available=True, available=True, ready=True, error="", source="cpu",
    ), **changes)


def test_optional_status_missing_defaults_cpu_but_loss_preserves_last_fpga_selection() -> None:
    """从未选择时默认 CPU；已选 FPGA 后失联不能假称自动切回 CPU。"""
    controller = GroundStationRosController()
    assert controller.avoidance_backend_snapshot().backend == "cpu"
    assert not controller.avoidance_backend_snapshot().manager_available
    controller._backend_service_available = True
    controller._ingest_avoidance_backend_status(SimpleNamespace(data=json.dumps({
        "backend": "fpga", "available": True, "ready": True,
        "error": "", "source": "cpu_fallback",
    })))
    assert controller.avoidance_backend_snapshot().backend == "fpga"
    assert controller.avoidance_backend_snapshot().source == "cpu_fallback"
    controller._ingest_avoidance_backend_status(SimpleNamespace(data=json.dumps({
        "backend": "cpu", "available": True, "ready": False,
        "source": "cpu", "source_confirmed": False,
    })))
    assert controller.avoidance_backend_snapshot().backend == "fpga"
    assert controller.avoidance_backend_snapshot().source == "unknown"
    controller._backend_status_received_at = time.monotonic() - 6.0
    stale = controller.avoidance_backend_snapshot()
    assert stale.backend == "fpga" and not stale.available and not stale.ready
    assert stale.source == "unknown"
    assert "实际后端和求解来源待确认" in stale.error
    controller._ingest_avoidance_backend_status(SimpleNamespace(data="{}"))
    assert "状态无效" in controller.avoidance_backend_snapshot().error
    assert controller.avoidance_backend_snapshot().backend == "fpga"
    controller._reset_avoidance_backend_runtime()
    assert controller.avoidance_backend_snapshot() == AvoidanceBackendSnapshot()


def test_missing_backend_manager_preserves_cpu_takeoff_and_three_strategies() -> None:
    """旧客户端完全不提供新接口时，CPU 就绪能力及原策略门控保持。"""
    window, ros = _window(replace(_operational_snapshot(armed=False), avoidance_ready=True))
    try:
        window._environment_active = True
        window._connection_mode = "simulation"
        for strategy in range(3):
            window.waypoints.strategy_combo.setCurrentIndex(strategy)
            window._refresh()
            assert window.operations.takeoff_button.isEnabled()
        assert window.waypoints.backend_combo.currentData() == "cpu"
        assert not window.waypoints.backend_combo.model().item(1).isEnabled()
        assert not window.waypoints.backend_combo.isEnabled()
        assert "CPU 默认" in window.waypoints.backend_status_label.text()
        assert not ros.calls
    finally:
        _close_window(window)


def test_backend_gui_waits_for_actual_status_and_displays_fallback_and_mock() -> None:
    """异步请求/拒绝均不乐观显示 FPGA，实际回退与 MOCK 来源明确可见。"""
    window, ros = _window(replace(_operational_snapshot(armed=False), on_ground=True))
    ros.backend = _backend()
    ros.avoidance_backend_snapshot = lambda: ros.backend
    ros.request_avoidance_backend = lambda target: ros._record("avoidance_backend", target)
    try:
        window._environment_active = True
        window._connection_mode = "simulation"
        window._refresh()
        assert window.waypoints.backend_combo.isEnabled()
        window.waypoints.backend_combo.activated.emit(1)
        assert ros.calls[-1] == ("avoidance_backend", "fpga")
        assert window.waypoints.backend_combo.currentData() == "cpu"
        assert not window.operations.takeoff_button.isEnabled()
        ros.results.append(CommandResult(1, 1, "avoidance_backend", False, "板卡不可达"))
        window._refresh()
        assert window._backend_switch_target is None
        assert window.waypoints.backend_combo.currentData() == "cpu"
        assert window.operations.takeoff_button.isEnabled()
        window.waypoints.backend_combo.activated.emit(1)
        ros.backend = _backend(backend="fpga", source="cpu_fallback")
        window._refresh()
        assert window._backend_switch_target is None
        assert window.waypoints.backend_combo.currentData() == "fpga"
        assert "cpu_fallback" in window.waypoints.backend_status_label.text()
        assert "CPU 回退" in window.waypoints.backend_status_label.text()
        ros.backend = replace(ros.backend, source="mock")
        window._refresh()
        assert "mock" in window.waypoints.backend_status_label.text()
        assert "MOCK 模拟" in window.waypoints.backend_status_label.text()
        ros.backend = replace(ros.backend, manager_available=False, available=False,
                              ready=False, source="unknown", age_seconds=6.0)
        window._refresh()
        assert window.waypoints.backend_combo.currentData() == "fpga"
        assert "FPGA 最后选择 · 状态未知" in window.waypoints.backend_status_label.text()
        assert not window.waypoints.backend_combo.isEnabled()
    finally:
        _close_window(window)


@pytest.mark.parametrize("changes", [
    {"armed": True}, {"on_ground": False}, {"controller_active": True},
    {"active_mode": FlightMode.WAYPOINT}, {"waypoint_count": 1},
    {"reboot_in_progress": True},
])
def test_backend_transport_rechecks_ground_only_gate(changes) -> None:
    """请求在队列里等待后必须重新核对 ControlStatus，禁止空中切源。"""
    controller = GroundStationRosController(source_id="qt-test")
    controller._state._snapshot = replace(
        _operational_snapshot(armed=False), **{"on_ground": True, **changes},
    )
    controller._state._last_status_time = time.monotonic()
    controller.enable_control()
    controller.request_avoidance_backend("fpga")
    controller._process_one_command({"node": object(), "clients": {}}, {})
    assert not controller.results_after(0)[-1].success
    assert "落地、未解锁且无任务" in controller.results_after(0)[-1].message


def test_backend_setbool_response_is_final_and_does_not_require_flight_result() -> None:
    """标准 SetBool 的 success 响应真实终结后台请求，无额外飞行协议字段。"""
    controller = GroundStationRosController(source_id="qt-test")
    controller._state._snapshot = replace(_operational_snapshot(armed=False), on_ground=True)
    controller._state._last_status_time = time.monotonic()
    controller.enable_control()
    controller._backend_snapshot = _backend()
    controller._backend_status_received_at = time.monotonic()
    controller._backend_service_available = True
    requests = []
    future = SimpleNamespace(done=lambda: True, result=lambda: SimpleNamespace(
        success=True, message="FPGA selected",
    ))
    client = SimpleNamespace(
        service_is_ready=lambda: True,
        call_async=lambda request: requests.append(request) or future,
    )
    pending = {}
    for target, expected in (("fpga", True), ("cpu", False)):
        ticket = controller.request_avoidance_backend(target)
        controller._process_one_command({
            "node": object(), "clients": {"avoidance_backend": client},
            "SetBool": SimpleNamespace(Request=SimpleNamespace),
        }, pending)
        assert requests[-1].data is expected
        assert ticket in pending
        controller._poll_pending_services(pending)
        result = controller.results_after(0)[-1]
        assert result.command == "avoidance_backend" and result.success and result.final
        assert not pending
