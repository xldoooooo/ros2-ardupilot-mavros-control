"""独立航点面板的类型属性与 CSV 往返回归，不启动 ROS 或连接设备。"""

from __future__ import annotations

import math
from dataclasses import replace
from pathlib import Path

import pytest
from PySide6.QtCore import QEvent, QPoint, Qt
from PySide6.QtTest import QTest
from PySide6.QtWidgets import QApplication, QLineEdit

from ground_station_core.models import FlightMode, VehicleSnapshot
from ground_station_core.qt_ui.state import derive_availability
from ground_station_core.qt_ui.theme import apply_theme
from ground_station_core.qt_ui.waypoint_panel import WaypointPanel
from ground_station_core.waypoint_io import (
    Waypoint,
    load_waypoint_file,
    save_waypoint_file,
)


@pytest.fixture
def panel():
    """复用 Qt 应用但独立创建面板，确保每个用例结束后释放控件。"""
    application = QApplication.instance() or QApplication([])
    apply_theme(application)
    widget = WaypointPanel()
    widget.resize(610, 680)
    widget.show()
    application.processEvents()
    yield widget
    widget.close()
    widget.deleteLater()
    application.sendPostedEvents(None, QEvent.Type.DeferredDelete)


def _toggle_checkbox(panel: WaypointPanel, row: int, column: int) -> None:
    """真实点击表格复选框指示器，验证 GUI 到具名字段的更新路径。"""
    rectangle = panel.table.visualItemRect(panel.table.item(row, column))
    QTest.mouseClick(
        panel.table.viewport(), Qt.MouseButton.LeftButton,
        pos=QPoint(rectangle.left() + 10, rectangle.center().y()),
    )
    QApplication.processEvents()


def test_new_points_are_ordinary_and_flags_can_be_combined(panel: WaypointPanel) -> None:
    """巡检、校准、沿窗分别可选；勾选旧行不改变下一新点的默认类型。"""
    panel.add_button.click()
    assert panel.table.columnCount() == 9
    assert panel.waypoints == (Waypoint(0.0, 0.0, 1.0, 0.0),)
    ordinary_color = panel.table.item(0, 0).foreground().color()
    _toggle_checkbox(panel, 0, 5)
    assert panel.waypoints[0].is_inspection
    assert not panel.waypoints[0].is_calibration
    inspection_color = panel.table.item(0, 0).foreground().color()
    assert inspection_color != ordinary_color
    _toggle_checkbox(panel, 0, 6)
    _toggle_checkbox(panel, 0, 8)
    assert panel.waypoints[0][4:] == (True, True, 0, True)
    assert panel.table.item(0, 0).foreground().color() != inspection_color
    assert "巡检 + 校准" in panel.table.item(0, 0).toolTip()
    _toggle_checkbox(panel, 0, 5)
    assert panel.waypoints[0][4:] == (False, True, 0, True)
    panel.add_button.click()
    assert panel.waypoints[1][4:] == (False, False, 0, False)


def test_coordinate_and_tag_edits_preserve_record_type_and_other_attributes(
    panel: WaypointPanel,
) -> None:
    """坐标及大整数 ID 实时编辑不丢标志，非法整数文本不覆盖有效值。"""
    point = Waypoint(1.123456, 2.0, 3.0, 0.123456, True, True, 2**40, True)
    panel.replace_waypoints((point,), "测试")
    panel._edit_waypoint(0, 1, 9.87)
    panel._edit_waypoint(0, 4, 45.6)
    assert isinstance(panel.waypoints[0], Waypoint)
    assert panel.waypoints[0].x == 9.87
    assert math.isclose(panel.waypoints[0].yaw, math.radians(45.6))
    assert panel.waypoints[0][4:] == point[4:]
    item = panel.table.item(0, 7)
    panel.table.setCurrentItem(item)
    panel.table.editItem(item)
    QApplication.processEvents()
    editor = next(
        control for control in panel.table.viewport().findChildren(QLineEdit)
        if control.isVisible()
    )
    assert editor.text() == str(point.calibration_tag_id)
    editor.selectAll()
    QTest.keyClicks(editor, "-")
    assert editor.text() == str(point.calibration_tag_id)
    QTest.keyClick(editor, Qt.Key.Key_Backspace)
    assert not editor.hasAcceptableInput()
    assert panel.waypoints[0].calibration_tag_id == point.calibration_tag_id
    QTest.keyClicks(editor, "7")
    assert panel.waypoints[0].calibration_tag_id == 7
    QTest.keyClick(editor, Qt.Key.Key_Return)
    QApplication.processEvents()
    assert panel.waypoints[0][4:] == (True, True, 7, True)
    panel._refresh_table(0)
    assert panel.table.item(0, 7).text() == "7"
    assert panel.table.item(0, 8).checkState() == Qt.CheckState.Checked


def test_refresh_move_import_export_and_send_keep_all_attributes(
    panel: WaypointPanel, tmp_path: Path,
) -> None:
    """刷新、排序、增删、导入导出及发送副本均完整保留追加字段。"""
    points = (
        Waypoint(1.0, 2.0, 3.0, 0.0, True, True, 42, False),
        Waypoint(4.0, 5.0, 6.0, 0.0, False, True, 7, True),
    )
    panel.replace_waypoints(points, "测试")
    changes = []
    panel.waypoints_changed.connect(changes.append)
    panel._refresh_table(1)
    assert not changes
    panel.up_button.click()
    assert panel.waypoints == points[::-1]
    panel._append_waypoint((7.0, 8.0, 9.0, 0.0))
    assert panel.waypoints[-1] == Waypoint(7.0, 8.0, 9.0, 0.0)
    panel.remove_button.click()
    assert panel.waypoints == points[::-1]
    destination = save_waypoint_file(tmp_path / "panel.csv", panel.waypoints)
    panel.clear_waypoints()
    panel.replace_waypoints(load_waypoint_file(destination), "CSV")
    assert panel.waypoints == points[::-1]
    sent = []
    panel.send_requested.connect(lambda *arguments: sent.append(arguments))
    panel.send_button.click()
    assert sent[0][0] == points[::-1]
    assert all(isinstance(point, Waypoint) for point in sent[0][0])
    assert len(changes) == 5


def test_availability_locks_new_attribute_controls_with_existing_editor(
    panel: WaypointPanel,
) -> None:
    """沿用表格编辑锁，任务执行时复选框和 Tag 编辑不能改写航点。"""
    panel.replace_waypoints((Waypoint(1.0, 2.0, 3.0, 0.0),), "测试")
    state = derive_availability(
        VehicleSnapshot(), ros_ready=False, busy=False, closing=False,
        environment_active=False, waypoint_count=1, waypoint_running=False,
    )
    panel.apply_availability(state)
    assert not panel.table.isEnabled()
    panel._edit_waypoint_attribute(0, "is_calibration", True)
    panel._edit_waypoint_attribute(0, "calibration_tag_id", 42)
    assert panel.waypoints[0][4:] == (False, False, 0, False)
    panel.apply_availability(replace(state, waypoint_edit=True))
    _toggle_checkbox(panel, 0, 6)
    assert panel.waypoints[0].is_calibration


def test_calibration_status_and_failure_remain_visible_without_false_completion(panel: WaypointPanel) -> None:
    """直线校准与失败原因在真实布局可见；校准/降落/取消不冒充末点成功。"""
    panel.replace_waypoints((Waypoint(0.0, 0.0, 1.0, 0.0, is_calibration=True),), "测试")
    panel.set_result("任务执行中", running=True)
    snapshot = VehicleSnapshot(
        active_mode=FlightMode.WAYPOINT, waypoint_index=1, waypoint_count=1,
        calibration_active=True, calibration_job_id="job-42", calibration_detail="等待融合结果",
    )
    panel.update_progress(snapshot)
    QApplication.processEvents()
    assert panel.status_label.isVisible()
    assert panel.status_label.parentWidget().content_layout.indexOf(panel.status_label) >= 0
    assert "校准中" in panel.status_label.text() and "job-42" in panel.status_label.text()
    assert "等待融合结果" in panel.progress.toolTip()
    assert panel.progress.value() == 0
    failure = replace(snapshot, active_mode=FlightMode.LAND, calibration_active=False,
                      calibration_detail="下视相机打开失败")
    panel.set_result("航点失败：下视相机打开失败，原地降落", running=False)
    panel.update_progress(failure)
    assert panel.status_label.isVisible() and "下视相机打开失败" in panel.status_label.text()
    assert panel.progress.value() == 0
    panel.update_progress(replace(failure, active_mode=FlightMode.HOVER, calibration_detail=""))
    assert "下视相机打开失败" in panel.status_label.text()
    assert panel.progress.value() == 0
    panel.complete_progress(1, 1)
    panel.update_progress(replace(snapshot, calibration_active=False))
    assert panel.progress.value() == 1
