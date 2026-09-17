"""waypoints 闭环的 Qt 交互回归；使用共享替身，不连接实机。"""

from __future__ import annotations

from tests.support.qt import (
    _FakeEnvironment,
    _application,
    _close_window,
    _operational_snapshot,
    _window,
)
import math
from dataclasses import replace
from pathlib import Path
from unittest.mock import patch
from PySide6.QtCore import QMimeData, QPoint, QPointF, Qt, QUrl
from PySide6.QtGui import QDragEnterEvent, QDropEvent
from ground_station_core.config import PROJECT_ROOT
from ground_station_core.event_log import LogLevel
from ground_station_core.models import CommandResult, FlightMode


def test_add_current_waypoint_uses_displayed_pose_and_rejects_invalid_pose() -> None:
    """实测点击追加原始实际位姿，保留旧点与输入值，失效时禁止打点。"""
    pose = (1.23456, -2.34567, -0.01234, -1.23456)
    snapshot = replace(
        _operational_snapshot(armed=False),
        x=pose[0],
        y=pose[1],
        z=pose[2],
        yaw=pose[3],
        target_x=99.0,
    )
    window, ros = _window(snapshot)
    try:
        panel = window.waypoints
        assert not panel.add_current_button.isEnabled()
        window._environment_active = True
        window._connection_mode = "hardware"
        window._refresh()
        panel.add_button.click()
        original = panel.waypoints
        calls = list(ros.calls)
        # 后端变化尚未渲染时，应记录用户眼前那一帧。
        ros.current_snapshot = replace(snapshot, x=8.76543)
        panel.add_current_button.click()
        assert panel.waypoints == original + (pose,)
        assert panel.x_input.value() == 0.0
        assert panel.table.currentRow() == 1
        assert panel.add_current_button.width() == panel.add_current_button.height()
        assert panel.add_current_button.x() > panel.add_button.x()
        assert not panel.add_current_button.icon().isNull()
        assert ros.calls == calls
        window._refresh()
        panel.add_current_button.click()
        assert panel.waypoints[-1] == (8.76543, *pose[1:])
        count = len(panel.waypoints)
        for invalid in (
            replace(snapshot, local_position_valid=False),
            replace(snapshot, onboard_available=False),
            replace(snapshot, connected=False),
            replace(snapshot, yaw=float("nan")),
        ):
            ros.current_snapshot = invalid
            window._refresh()
            assert not panel.add_current_button.isEnabled()
            panel.add_current_button.click()
            assert len(panel.waypoints) == count
    finally:
        _close_window(window)


def test_environment_session_gates_start_buttons_and_waypoint_widgets() -> None:
    """无会话时航点组件全禁用；会话建立后禁用双启动入口、原点齿轮并互斥启用关闭按钮。"""
    window, _ros = _window(_operational_snapshot(armed=False))
    try:
        assert window.operations.simulation_button.isEnabled()
        assert window.operations.hardware_button.isEnabled()
        assert window.operations.communication_test_button.isEnabled()
        assert window.operations.origin_settings_button.isEnabled()
        assert not window.operations.stop_simulation_button.isEnabled()
        assert not window.operations.disconnect_hardware_button.isEnabled()
        assert not window.waypoints.x_input.isEnabled()
        assert not window.waypoints.add_button.isEnabled()
        assert not window.waypoints.import_button.isEnabled()
        assert not window.waypoints.export_button.isEnabled()
        assert not window.waypoints.preview_button.isEnabled()
        assert not window.waypoints.send_button.isEnabled()
        assert not window.waypoints.table.isEnabled()

        window._initialize_simulation()
        window._refresh()
        assert window._environment_active
        assert window._connection_mode == "simulation"
        assert not window.operations.simulation_button.isEnabled()
        assert not window.operations.hardware_button.isEnabled()
        assert not window.operations.communication_test_button.isEnabled()
        assert not window.operations.origin_settings_button.isEnabled()
        assert window.operations.stop_simulation_button.isEnabled()
        assert not window.operations.disconnect_hardware_button.isEnabled()
        assert window.waypoints.x_input.isEnabled()
        assert window.waypoints.add_button.isEnabled()
        assert window.waypoints.import_button.isEnabled()
        assert not window.waypoints.export_button.isEnabled()
        assert not window.waypoints.preview_button.isEnabled()
        assert not window.waypoints.send_button.isEnabled()
    finally:
        _close_window(window)


def test_waypoint_confirmation_and_responsive_two_column_splitters() -> None:
    """飞行前锁定三项选择，仿真有效组合免确认、实机保留确认。"""
    from ground_station_core.models import (
        WaypointFlightStrategy,
        WaypointReferenceGenerator,
        WaypointTrackingController,
    )

    window, ros = _window(_operational_snapshot(armed=False))
    try:
        window._environment_active = True
        window._connection_mode = "simulation"
        window._refresh()
        assert window.waypoints.add_button.isEnabled()
        window.waypoints.add_button.click()
        window.waypoints.x_input.setValue(2.0)
        window.waypoints.add_button.click()
        window._refresh()
        assert not window.waypoints.send_button.isEnabled()
        assert window.waypoints.strategy_combo.isEnabled()
        assert window.waypoints.reference_combo.isEnabled()
        assert window.waypoints.tracking_combo.isEnabled()
        assert window.waypoints.strategy_combo.count() == 3
        assert window.waypoints.reference_combo.count() == 4
        assert window.waypoints.tracking_combo.count() == 2
        assert window.waypoints.selected_strategy() is WaypointFlightStrategy.STRAIGHT
        assert (
            window.waypoints.selected_reference_generator()
            is WaypointReferenceGenerator.TRAPEZOIDAL_PROFILE
        )
        assert (
            window.waypoints.selected_tracking_controller()
            is WaypointTrackingController.TRAJECTORY_PD_DOB
        )
        window.waypoints.reference_combo.setCurrentIndex(
            WaypointReferenceGenerator.JERK_LIMITED_S_CURVE.value
        )
        window.waypoints.tracking_combo.setCurrentIndex(
            WaypointTrackingController.TRAJECTORY_PD_DOB.value
        )

        ros.current_snapshot = _operational_snapshot(armed=True)
        window._refresh()
        assert window.waypoints.send_button.isEnabled()
        assert not window.waypoints.strategy_combo.isEnabled()
        assert not window.waypoints.reference_combo.isEnabled()
        assert not window.waypoints.tracking_combo.isEnabled()

        window._confirm_action = lambda *_args, **_kwargs: (_ for _ in ()).throw(
            AssertionError("仿真航点发送不得弹出二次确认")
        )
        window._send_waypoints(window.waypoints.waypoints)
        assert len(ros.calls) == 1
        assert ros.calls[-1][0] == "waypoints"

        window._waypoint_running = False
        window._pending_commands.discard("waypoints")
        ros.current_snapshot = _operational_snapshot(armed=False)
        window._connection_mode = "hardware"
        window._refresh()
        window.waypoints.strategy_combo.setCurrentIndex(1)  # 自动避障（预留）
        ros.current_snapshot = _operational_snapshot(armed=True)
        window._refresh()
        window._confirm_action = lambda *_args, **_kwargs: False
        window._send_waypoints(
            window.waypoints.waypoints, window.waypoints.selected_strategy()
        )
        assert len(ros.calls) == 1
        window._confirm_action = lambda *_args, **_kwargs: True
        window._send_waypoints(
            window.waypoints.waypoints, window.waypoints.selected_strategy()
        )
        assert len(ros.calls) == 2
        assert ros.calls[-1][0] == "waypoints"
        (
            waypoints_arg,
            strategy_arg,
            generator_arg,
            controller_arg,
            photo_nos_arg,
        ) = ros.calls[-1][1]
        assert len(waypoints_arg) == 2
        # 界面可选避障，但当前实现路径仍传递所选策略枚举；执行侧按直线处理。
        assert strategy_arg is WaypointFlightStrategy.AVOID
        assert generator_arg is WaypointReferenceGenerator.JERK_LIMITED_S_CURVE
        assert controller_arg is WaypointTrackingController.TRAJECTORY_PD_DOB
        assert photo_nos_arg == ()

        for width, height in ((1180, 700), (1800, 1000)):
            window.resize(width, height)
            _application().processEvents()
            assert window.size().width() == width
            assert window.size().height() == height
            assert window.operations.isVisible()
            assert window.operations.manual_panel.isVisible()
            assert window.waypoints.isVisible()
            assert window.log_panel.isVisible()
            assert window.workspace_splitter.count() == 2
            assert window.operations.width() >= 650
            assert window.operations.manual_panel.width() >= 640
            assert window.waypoints.width() > 380
            assert window.log_panel.height() >= window.log_panel.minimumHeight()
    finally:
        _close_window(window)


def test_unverified_generator_controller_pair_requires_explicit_confirmation() -> None:
    """自由组合保持可用，但不推荐搭配在仿真中也必须默认取消告警。"""
    from ground_station_core.models import (
        WaypointReferenceGenerator,
        WaypointTrackingController,
    )

    window, ros = _window(_operational_snapshot(armed=False))
    try:
        window._environment_active = True
        window._connection_mode = "simulation"
        window._refresh()
        window.waypoints.add_button.click()
        window.waypoints.reference_combo.setCurrentIndex(
            WaypointReferenceGenerator.JERK_LIMITED_S_CURVE.value
        )
        window.waypoints.tracking_combo.setCurrentIndex(
            WaypointTrackingController.POSITION_PD_DOB.value
        )
        ros.current_snapshot = _operational_snapshot(armed=True)
        window._refresh()

        confirmations: list[tuple[str, str]] = []
        approve = [False]

        def confirm(title: str, message: str, **_kwargs) -> bool:
            confirmations.append((title, message))
            return approve[0]

        window._confirm_action = confirm
        window._send_waypoints(window.waypoints.waypoints)
        assert not ros.calls
        assert confirmations[-1][0] == "实验组合警告"
        assert "不是已验证搭配" in confirmations[-1][1]

        approve[0] = True
        window._send_waypoints(window.waypoints.waypoints)
        assert ros.calls[-1][0] == "waypoints"
    finally:
        _close_window(window)


def test_waypoint_preview_button_publishes_snapshot_and_tracks_later_edits() -> None:
    """预览点击只发布可视化，随后编辑/清空会替换 RViz 中的旧快照。"""
    environment = _FakeEnvironment()
    window, ros = _window(_operational_snapshot(armed=False), environment=environment)
    try:
        window._environment_active = True
        window._connection_mode = "simulation"
        window._refresh()
        panel = window.waypoints
        panel.x_input.setValue(1.0)
        panel.y_input.setValue(-2.0)
        panel.z_input.setValue(3.0)
        panel.yaw_input.setValue(90.0)
        panel.add_button.click()
        window._refresh()

        assert panel.preview_button.isEnabled()
        panel.preview_button.click()
        assert environment.preview_modes == ["simulation"]
        assert window._waypoint_preview_active
        preview_calls = [call for call in ros.calls if call[0] == "waypoint_preview"]
        assert preview_calls[-1][1] == panel.waypoints
        assert not any(call[0] == "waypoints" for call in ros.calls)
        assert "复用仿真" in window.activity_banner.message_label.text()

        # 预览激活后不用重新开窗口；列表变化只替换 retained Marker/Path 快照。
        panel.x_input.setValue(4.0)
        panel.add_button.click()
        preview_calls = [call for call in ros.calls if call[0] == "waypoint_preview"]
        assert len(preview_calls) == 2
        assert len(preview_calls[-1][1]) == 2
        assert environment.preview_modes == ["simulation"]

        panel.clear_waypoints()
        assert ros.calls[-1] == ("waypoint_preview", ())
        assert not panel.preview_button.isEnabled()
    finally:
        _close_window(window)


def test_waypoint_editor_compacts_rows_icons_and_downward_strategy_popup() -> None:
    """航点输入紧凑，执行卡无标题且三种方法选择完整向下显示。"""
    from ground_station_core.qt_ui.theme import COLORS, STYLE_SHEET

    window, _ros = _window(_operational_snapshot(armed=False))
    try:
        window._environment_active = True
        window._connection_mode = "simulation"
        window._refresh()
        panel = window.waypoints
        assert panel.add_button.text() == "+"
        assert panel.add_button.property("role") == "neutral"
        input_heights = {
            control.height()
            for control in (
                panel.x_input,
                panel.y_input,
                panel.z_input,
                panel.yaw_input,
            )
        }
        assert input_heights == {panel._ROW_HEIGHT}
        assert all(
            control.buttonSymbols() == control.ButtonSymbols.UpDownArrows
            for control in (
                panel.x_input,
                panel.y_input,
                panel.z_input,
                panel.yaw_input,
            )
        )
        assert [
            panel.x_input.suffix(),
            panel.y_input.suffix(),
            panel.z_input.suffix(),
            panel.yaw_input.suffix(),
        ] == ["m", "m", "m", "°"]
        assert "单位" in panel.x_input.toolTip()
        assert panel.table.horizontalHeader().height() == panel._ROW_HEIGHT
        assert panel.table.verticalHeader().defaultSectionSize() == panel._ROW_HEIGHT

        panel.add_button.click()
        panel.x_input.setValue(2.0)
        panel.add_button.click()
        assert all(
            panel.table.rowHeight(row) == panel._ROW_HEIGHT
            for row in range(panel.table.rowCount())
        )
        assert panel.up_button.text() == panel.down_button.text() == ""
        assert panel.remove_button.text() == ""
        assert not panel.up_button.icon().isNull()
        assert not panel.down_button.icon().isNull()
        assert not panel.remove_button.icon().isNull()
        assert "删除" in panel.remove_button.accessibleName()
        assert panel.up_button.x() < panel.down_button.x() < panel.remove_button.x()
        assert panel.clear_button.text() == "清空"
        assert panel.preview_button.text() == "预览"
        assert panel.import_button.text() == "从文件导入"
        assert panel.export_button.text() == "导出到文件"
        assert panel.preview_button.property("role") == "neutral"
        assert panel.import_button.property("role") == "neutral"
        assert panel.export_button.property("role") == "neutral"
        assert panel.clear_button.x() < panel.preview_button.x()
        assert panel.preview_button.x() < panel.import_button.x()
        assert panel.import_button.x() < panel.export_button.x()
        assert "RViz" in panel.preview_button.toolTip()
        assert "CSV" in panel.import_button.toolTip()
        assert "CSV" in panel.export_button.toolTip()
        assert "QPushButton#previewWaypointButton" in STYLE_SHEET
        assert "QPushButton#importWaypointButton" in STYLE_SHEET
        assert "QPushButton#exportWaypointButton" in STYLE_SHEET
        assert panel.send_button.property("role") == "primary"
        assert not panel.parentWidget().isWindow()
        execution_card = panel.progress.parentWidget()
        assert execution_card.title_label.text() == ""
        assert not execution_card.title_label.isVisible()
        assert (
            abs(
                panel.progress.geometry().center().y()
                - panel.send_button.geometry().center().y()
            )
            <= 1
        )
        assert panel.progress.width() > panel.send_button.width() * 1.8
        assert panel.reference_combo.count() == 4
        assert panel.tracking_combo.count() == 2
        # 结果状态仍按原链路更新，但不再作为灰色说明行参与布局。
        assert not panel.status_label.isVisible()
        assert panel.status_label.parentWidget() is not None
        assert (
            panel.status_label.parentWidget().content_layout.indexOf(panel.status_label)
            == -1
        )
        panel.set_result("航点状态回归", running=False)
        assert panel.status_label.text() == "航点状态回归"
        assert "QProgressBar#waypointProgress::chunk" in STYLE_SHEET
        assert COLORS["success"] in STYLE_SHEET

        combo = panel.strategy_combo
        combo.showPopup()
        _application().processEvents()
        popup = combo.popup_menu
        popup_top = popup.frameGeometry().top()
        combo_bottom = combo.mapToGlobal(QPoint(0, combo.height())).y()
        assert popup_top >= combo_bottom
        actions = popup.actions()
        assert len(actions) == combo.count() == 3
        assert [action.text() for action in actions] == [
            combo.itemText(index) for index in range(combo.count())
        ]
        assert all(popup.actionGeometry(action).height() > 0 for action in actions)
        assert popup.actionGeometry(actions[-1]).bottom() < popup.height()
        assert popup.width() >= max(
            popup.fontMetrics().horizontalAdvance(action.text()) + 24
            for action in actions
        )
        actions[1].trigger()
        _application().processEvents()
        assert combo.currentIndex() == 1
        assert not popup.isVisible()

        panel._progress_tracking = True
        window._waypoint_running = True
        window._refresh()
        assert not panel.add_button.isEnabled()
        assert not panel.import_button.isEnabled()
        assert panel.export_button.isEnabled()
        assert panel.preview_button.isEnabled()
        assert not panel.strategy_combo.isEnabled()
        assert all(
            not control.isEnabled()
            for control in (
                panel.x_input,
                panel.y_input,
                panel.z_input,
                panel.yaw_input,
            )
        )
    finally:
        _close_window(window)


def test_waypoint_file_button_and_table_drop_replace_only_after_confirmation(
    tmp_path: Path,
) -> None:
    """按钮和真实表格拖放共用解析/确认链，取消不变、确认后按行替换。"""
    window, _ros = _window(_operational_snapshot(armed=True))
    try:
        window._environment_active = True
        window._connection_mode = "simulation"
        window._refresh()
        panel = window.waypoints
        panel.x_input.setValue(9.0)
        panel.add_button.click()
        original = panel.waypoints

        selected = tmp_path / "selected.csv"
        selected.write_text(
            "index,x,y,z,yaw\n1,1.25,-2.5,3.0,90\n2,4.0,5.0,6.0,-180\n",
            encoding="utf-8",
        )
        confirmations: list[tuple[str, str]] = []
        approve = [False]

        def confirm(title: str, message: str, **_kwargs) -> bool:
            confirmations.append((title, message))
            return approve[0]

        window._confirm_action = confirm
        with patch(
            "ground_station_core.qt_ui.main_window.QFileDialog.getOpenFileName",
            return_value=(str(selected), "CSV 文件 (*.csv)"),
        ) as chooser:
            panel.import_button.click()
        chooser.assert_called_once()
        chooser_arguments = chooser.call_args.args
        assert Path(chooser_arguments[2]) == PROJECT_ROOT / "examples"
        assert "*.csv" in chooser_arguments[3]
        assert panel.waypoints == original
        assert confirmations[-1][0] == "从文件导入航点"
        assert "清空并替换当前列表中的 1 个航点" in confirmations[-1][1]
        assert "绝对本地 ENU" in confirmations[-1][1]

        approve[0] = True
        with patch(
            "ground_station_core.qt_ui.main_window.QFileDialog.getOpenFileName",
            return_value=(str(selected), "CSV 文件 (*.csv)"),
        ):
            panel.import_button.click()
        assert len(panel.waypoints) == panel.table.rowCount() == 2
        assert panel.waypoints[0][:3] == (1.25, -2.5, 3.0)
        assert math.isclose(panel.waypoints[0][3], math.pi / 2.0)
        assert math.isclose(panel.waypoints[1][3], -math.pi)
        assert panel.table.item(0, 0).text() == "1"
        assert panel.table.item(0, 1).text() == "+1.25"
        assert panel.progress.format() == "尚未执行"

        dropped = tmp_path / "dropped.csv"
        dropped.write_text(
            "index,x,y,z,yaw\n1,-3.0,7.5,2.0,45\n",
            encoding="utf-8",
        )
        mime_data = QMimeData()
        mime_data.setUrls([QUrl.fromLocalFile(str(dropped))])
        drag_event = QDragEnterEvent(
            QPoint(10, 10),
            Qt.DropAction.CopyAction,
            mime_data,
            Qt.MouseButton.LeftButton,
            Qt.KeyboardModifier.NoModifier,
        )
        panel.table.dragEnterEvent(drag_event)
        assert drag_event.isAccepted()
        drop_event = QDropEvent(
            QPointF(10.0, 10.0),
            Qt.DropAction.CopyAction,
            mime_data,
            Qt.MouseButton.LeftButton,
            Qt.KeyboardModifier.NoModifier,
        )
        panel.table.dropEvent(drop_event)
        assert drop_event.isAccepted()
        assert len(confirmations) == 3
        assert panel.waypoints[0][:3] == (-3.0, 7.5, 2.0)
        assert math.isclose(panel.waypoints[0][3], math.pi / 4.0)
        assert any(
            event.source == "waypoint-import" and event.level is LogLevel.INFO
            for event in window.event_log.snapshot()
        )
    finally:
        _close_window(window)


def test_waypoint_export_button_saves_only_current_gui_list_as_csv(
    tmp_path: Path,
) -> None:
    """非空列表启用导出，保存器默认项目 export 目录且不访问机载任务。"""
    window, ros = _window(_operational_snapshot(armed=True))
    try:
        window._environment_active = True
        window._connection_mode = "simulation"
        window._refresh()
        panel = window.waypoints
        assert not panel.export_button.isEnabled()

        panel.x_input.setValue(1.25)
        panel.y_input.setValue(-2.5)
        panel.z_input.setValue(3.0)
        panel.yaw_input.setValue(90.0)
        panel.add_button.click()
        assert panel.export_button.isEnabled()

        destination_without_suffix = tmp_path / "chosen-waypoints"
        with patch(
            "ground_station_core.qt_ui.main_window.QFileDialog.getSaveFileName",
            return_value=(str(destination_without_suffix), "CSV 文件 (*.csv)"),
        ) as chooser:
            panel.export_button.click()

        chooser.assert_called_once()
        chooser_arguments = chooser.call_args.args
        default_path = Path(chooser_arguments[2])
        assert default_path.parent == PROJECT_ROOT / "export"
        assert default_path.name.startswith("waypoints-export-")
        assert default_path.suffix == ".csv"
        assert chooser_arguments[3] == "CSV 文件 (*.csv)"
        assert (tmp_path / "chosen-waypoints.csv").read_text(
            encoding="utf-8"
        ).splitlines() == [
            "index,x,y,z,yaw",
            "1,1.25,-2.5,3,90",
        ]
        assert not any(call[0] == "waypoints" for call in ros.calls)
        assert any(
            event.source == "waypoint-export" and event.level is LogLevel.INFO
            for event in window.event_log.snapshot()
        )
    finally:
        _close_window(window)


def test_invalid_or_multiple_dropped_files_preserve_existing_waypoints(
    tmp_path: Path,
) -> None:
    """拖入错误格式或多个文件会明确告警，且不会部分清空当前列表。"""
    window, _ros = _window(_operational_snapshot(armed=True))
    try:
        window._environment_active = True
        window._connection_mode = "simulation"
        window._refresh()
        window.waypoints.add_button.click()
        original = window.waypoints.waypoints
        invalid = tmp_path / "invalid.csv"
        invalid.write_text("index,x,y,z,yaw\n1,not-a-number,0,1,0\n", encoding="utf-8")
        second = tmp_path / "second.csv"
        second.write_text("index,x,y,z,yaw\n1,0,0,1,0\n", encoding="utf-8")
        notices: list[tuple[str, str]] = []
        window._show_notice = lambda title, message, _icon: notices.append(
            (title, message)
        )
        window._confirm_action = lambda *_args, **_kwargs: (_ for _ in ()).throw(
            AssertionError("无效或多文件导入不得进入替换确认")
        )

        window.waypoints.files_dropped.emit((str(invalid),))
        assert window.waypoints.waypoints == original
        assert notices[-1][0] == "航点导入失败"
        assert "不是有效数字" in notices[-1][1]

        window.waypoints.files_dropped.emit((str(invalid), str(second)))
        assert window.waypoints.waypoints == original
        assert "一次只能导入一个" in notices[-1][1]
    finally:
        _close_window(window)


def test_clearing_completed_waypoints_resets_stale_onboard_progress() -> None:
    """清空本地列表后，机载旧快照不能把已结束任务进度重新显示出来。"""
    window, ros = _window(_operational_snapshot(armed=True))
    try:
        window.waypoints.add_button.click()
        window.waypoints.set_result("任务执行中", running=True)
        stale_snapshot = replace(
            ros.current_snapshot, waypoint_count=1, waypoint_index=1
        )
        window.waypoints.update_progress(stale_snapshot)
        assert window.waypoints.progress.value() == 1
        assert "1/1" in window.waypoints.progress.format()

        window.waypoints.clear_waypoints()
        window.waypoints.update_progress(stale_snapshot)
        assert window.waypoints.progress.value() == 0
        assert window.waypoints.progress.maximum() == 1
        assert window.waypoints.progress.format() == "尚未执行"
    finally:
        _close_window(window)


def test_running_waypoint_result_does_not_reset_existing_progress() -> None:
    """后续 RUNNING 结果不得把已完成航点进度闪回“等待机载任务进度”。"""
    window, ros = _window(_operational_snapshot(armed=True))
    try:
        window._environment_active = True
        window._connection_mode = "simulation"
        window._refresh()
        window.waypoints.add_button.click()
        window.waypoints.x_input.setValue(2.0)
        window.waypoints.add_button.click()
        window._refresh()
        window.waypoints.set_result("任务执行中", running=True)
        progressed = replace(ros.current_snapshot, waypoint_count=2, waypoint_index=1)
        window.waypoints.update_progress(progressed)
        before = (
            window.waypoints.progress.value(),
            window.waypoints.progress.maximum(),
            window.waypoints.progress.format(),
        )

        window.waypoints.set_result("前往航点 2/2", running=True)
        assert (
            window.waypoints.progress.value(),
            window.waypoints.progress.maximum(),
            window.waypoints.progress.format(),
        ) == before
        assert "等待" not in window.waypoints.progress.format()

        # 只有真正发送下一项新任务时，才允许显式回到新的等待状态。
        window.waypoints.set_result("上一任务完成", running=False)
        window._send_waypoints(window.waypoints.waypoints)
        assert window.waypoints.progress.value() == 0
        assert window.waypoints.progress.format() == "等待机载任务进度…"
    finally:
        _close_window(window)


def test_waypoint_progress_counts_reached_points_not_current_target() -> None:
    """飞向首点时为 0 格、切到次点时为 1 格，可靠完成后才满格。"""
    window, ros = _window(_operational_snapshot(armed=True))
    try:
        window.waypoints.set_result("任务执行中", running=True)
        to_first = replace(
            ros.current_snapshot,
            active_mode=FlightMode.WAYPOINT,
            waypoint_count=2,
            waypoint_index=1,
        )
        window.waypoints.update_progress(to_first)
        assert window.waypoints.progress.value() == 0
        assert window.waypoints.progress.format() == "已完成 0/2"

        to_second = replace(to_first, waypoint_index=2)
        window.waypoints.update_progress(to_second)
        assert window.waypoints.progress.value() == 1
        assert window.waypoints.progress.format() == "已完成 1/2"

        complete = replace(to_second, active_mode=FlightMode.HOVER)
        window.waypoints.update_progress(complete)
        assert window.waypoints.progress.value() == 2
        assert window.waypoints.progress.format() == "已完成 2/2"
    finally:
        _close_window(window)


def test_waypoint_terminal_result_fills_progress_when_land_hides_hover_status() -> None:
    """巡检终态紧接 LAND 时，必须用可靠结果补齐最后一格。"""
    window, ros = _window(_operational_snapshot(armed=True))
    try:
        window.waypoints.replace_waypoints(
            (
                (1.0, 0.0, 1.0, 0.0),
                (2.0, 0.0, 1.0, 0.0),
            ),
            "竞态测试",
        )
        window.waypoints.set_result("任务执行中", running=True)
        window._waypoint_running = True
        window._active_waypoint_ticket = 7
        window._pending_commands.add("waypoints")

        # GUI 最后看到的 WAYPOINT 帧仍指向末点，因此是 1/2。
        to_last = replace(
            ros.current_snapshot,
            active_mode=FlightMode.WAYPOINT,
            waypoint_index=2,
            waypoint_count=2,
        )
        window.waypoints.update_progress(to_last)
        assert window.waypoints.progress.value() == 1

        # 完成后立即降落会清空 ControlStatus 的航点字段；中间
        # HOVER 2/2 帧可能未被 GUI 采到，但可靠终态仍带 2/2。
        ros.current_snapshot = replace(
            to_last,
            active_mode=FlightMode.LAND,
            waypoint_index=0,
            waypoint_count=0,
        )
        ros.results.append(
            CommandResult(
                1,
                7,
                "waypoints",
                True,
                "航点任务完成",
                True,
                waypoint_index=2,
                waypoint_count=2,
            )
        )
        window._refresh()

        assert window.waypoints.progress.value() == 2
        assert window.waypoints.progress.maximum() == 2
        assert window.waypoints.progress.format() == "已完成 2/2"

        # 已消费终态的重复/迟到结果不再属于当前 ticket，不能在操作者
        # 清空显示后把旧任务重新补满。
        window.waypoints.reset_progress()
        ros.results.append(
            CommandResult(
                2,
                7,
                "waypoints",
                True,
                "重复的航点任务完成",
                True,
                waypoint_index=2,
                waypoint_count=2,
            )
        )
        window._refresh()
        assert window.waypoints.progress.value() == 0
        assert window.waypoints.progress.format() == "尚未执行"
    finally:
        _close_window(window)


def test_overridden_waypoint_task_ignores_old_cancellation_result() -> None:
    """05 覆盖旧任务后，旧 ticket 的取消终态不得解锁新任务 GUI。"""
    window, ros = _window(_operational_snapshot(armed=True))
    try:
        window._environment_active = True
        window._connection_mode = "simulation"
        window.waypoints.replace_waypoints(((1.0, 0.0, 1.0, 0.0),), "测试")
        window._refresh()
        first_ticket = window._send_waypoints(window.waypoints.waypoints)
        second_ticket = window._send_waypoints(
            ((0.0, 0.0, 1.0, 0.0),), allow_running_override=True
        )
        assert (first_ticket, second_ticket) == (1, 2)
        ros.results.extend(
            (
                CommandResult(1, 1, "waypoints", False, "旧任务已覆盖", True),
                CommandResult(2, 2, "waypoints", True, "新任务执行中", False),
            )
        )
        window._consume_results()
        assert window._waypoint_running
        assert window._active_waypoint_ticket == 2
        assert "waypoints" in window._pending_commands
        assert window.waypoints.status_label.text() == "新任务执行中"
    finally:
        _close_window(window)
