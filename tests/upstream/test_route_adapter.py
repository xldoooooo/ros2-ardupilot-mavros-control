"""验证双校准适配、业务映射及配置失败/手动编辑后的 03 隔离。"""

from dataclasses import replace

import pytest

from ground_station_core.qt_ui import main_window
from ground_station_core.upstream.mapping import parse_command
from ground_station_core.upstream.route_adapter import adapt_route
from ground_station_core.waypoint_io import Waypoint
from tests.support.qt import _close_window, _operational_snapshot, _window


def _task():
    """非连续业务点号用于区分列表首点和协议 index。"""
    return parse_command({
        "clientNo": "UAV01001", "commandNo": "02", "type": 1, "routId": "pending-id",
        "taskPoints": [
            {"index": 41, "x": 0, "y": 0, "z": 1, "forwardAngle": 90,
             "cameraAngle": 0, "photoNo": "photo-A"},
            {"index": 99, "x": 6, "y": 0, "z": 1, "forwardAngle": 0,
             "cameraAngle": 0, "photoNo": "photo-B"},
        ],
    }, "UAV01001")


def _execute():
    """03 本身不携带业务航点。"""
    return parse_command({"clientNo": "UAV01001", "commandNo": "03", "type": 1}, "UAV01001")


def test_default_route_preserves_business_identity_and_original_points():
    """首点附加校准，插入固定纯校准点，原业务坐标和照片不变。"""
    task = _task()
    route = adapt_route(task.waypoints, task.point_indexes, task.photo_nos, "unknown")
    assert route.waypoints[0] == task.waypoints[0]._replace(is_calibration=True)
    assert route.waypoints[1] == Waypoint(5., 0., 1., 0., False, True, 1, True)
    assert route.waypoints[2] == task.waypoints[1]
    assert route.point_indexes == (41, 0, 99)
    assert route.photo_nos == ("photo-A", "", "photo-B")
    assert not task.waypoints[0].is_calibration
    assert adapt_route(task.waypoints, task.point_indexes, task.photo_nos) == route


def test_dedicated_profile_inserts_plain_and_inspection_points_in_file_order(tmp_path):
    """多个航线共用配置，前后插入只引用原始业务点号。"""
    (tmp_path / "default.yaml").write_text("name: default\n", encoding="utf-8")
    (tmp_path / "workshop.yaml").write_text("""
name: workshop
route_ids: [A, B]
point_overrides:
  - point_index: 41
    is_inspection: false
insert_points:
  - after_point: first
    position: [1, 0, 1]
  - after_point: first
    position: [2, 0, 1]
    is_inspection: true
    point_index: 50
    photo_no: new-photo
  - before_point: 99
    position: [3, 0, 1]
""", encoding="utf-8")
    task = _task()
    route = adapt_route(task.waypoints, task.point_indexes, task.photo_nos, "B", directory=tmp_path)
    assert route.name == "workshop"
    assert [point.x for point in route.waypoints] == [0, 1, 2, 3, 6]
    assert [point.is_inspection for point in route.waypoints] == [False, False, True, False, True]
    assert route.point_indexes == (41, 0, 50, 0, 99)
    assert route.photo_nos == ("photo-A", "", "new-photo", "", "photo-B")


def test_new_adaptation_failure_prevents_old_route_execution(monkeypatch):
    """新 02 适配失败后保留旧表供查看，但 03 不得执行旧任务。"""
    window, ros = _window(_operational_snapshot(armed=False))
    try:
        window._initialize_simulation()
        window._handle_upstream_command(_task())
        assert window._upstream_route is not None

        def fail(*args, **kwargs):
            """模拟现场 YAML 引用了不存在的原始点号。"""
            raise ValueError("航线适配失败：不存在的原始点号 123")

        monkeypatch.setattr(main_window, "adapt_route", fail)
        window._handle_upstream_command(_task())
        before = list(ros.calls)
        window._handle_upstream_command(_execute())
        assert ros.calls == before
        assert window._upstream_route is None
        assert "不存在的原始点号" in window.activity_banner.message_label.text()
    finally:
        _close_window(window)


def test_upstream_03_uses_frozen_route_and_edit_requires_new_02():
    """03 显示并执行同一完整副本；编辑后禁止用重新编号回报业务结果。"""
    window, ros = _window(_operational_snapshot(armed=False))
    try:
        window._initialize_simulation()
        window._handle_upstream_command(_task())
        assert len(window.waypoints.waypoints) == 2
        window._handle_upstream_command(_execute())
        sequence = window._upstream_sequence
        assert sequence is not None
        assert len(sequence.waypoints) == 3
        assert sequence.point_indexes == (41, 0, 99)
        assert sequence.photo_nos == ("photo-A", "", "photo-B")
        assert window.waypoints.waypoints == sequence.waypoints
        window._abort_upstream_sequence("测试编辑")
        window._pending_commands.clear()
        ros.current_snapshot = replace(ros.current_snapshot, armed=False)
        window._refresh()
        window.waypoints._edit_waypoint_attribute(0, "calibration_tag_id", 2)
        before = list(ros.calls)
        window._handle_upstream_command(_execute())
        assert ros.calls == before
        assert "映射已失效" in window.activity_banner.message_label.text()
        window._handle_upstream_command(_task())
        assert not window._upstream_route_error
    finally:
        _close_window(window)
