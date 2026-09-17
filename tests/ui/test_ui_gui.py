"""ui 闭环的 Qt 交互回归；使用共享替身，不连接实机。"""

from __future__ import annotations

from tests.support.qt import (
    _application,
    _close_window,
    _operational_snapshot,
    _window,
)
from dataclasses import replace
from unittest.mock import patch
from PySide6.QtCore import QEvent, QObject, Qt
from PySide6.QtTest import QTest
from PySide6.QtWidgets import QApplication, QLabel, QMessageBox, QPushButton
from ground_station_core.event_log import EventLog, LogLevel
from ground_station_core.qt_ui.log_panel import LogPanel


def test_log_panel_combines_independent_source_generated_levels() -> None:
    """DEBUG 默认关闭；各等级仍可独立组合且不猜测消息严重度。"""
    application = _application()
    events = EventLog()
    panel = LogPanel(events)
    panel.show()
    events.debug("source", "debug line")
    events.info("source", "warning word but source says info")
    events.warn("source", "warn line")
    events.error("source", "error line")
    events.info("source", "long " + "payload " * 100)
    panel.poll()
    assert panel.viewer.horizontalScrollBar().value() == 0
    assert not panel.level_checks[LogLevel.DEBUG].isChecked()
    assert "debug line" not in panel.displayed_text
    panel.level_checks[LogLevel.DEBUG].setChecked(True)
    panel.level_checks[LogLevel.INFO].setChecked(False)
    panel.level_checks[LogLevel.WARN].setChecked(False)
    application.processEvents()

    assert panel.selected_levels() == frozenset((LogLevel.DEBUG, LogLevel.ERROR))
    assert "debug line" in panel.displayed_text
    assert "error line" in panel.displayed_text
    assert "warn line" not in panel.displayed_text
    assert "warning word but source says info" not in panel.displayed_text
    assert panel.search_input.height() == panel.clear_button.height()
    assert not any(label.text() == "等级" for label in panel.findChildren(QLabel))
    panel.close()


def test_stable_refresh_skips_reapplying_unchanged_widget_state() -> None:
    """稳定快照仍被读取，但不得重复写门控属性或触发 tooltip 事件。"""

    class TooltipCounter(QObject):
        """统计应用级 tooltip 变化，验证周期刷新不会制造事件风暴。"""

        def __init__(self) -> None:
            super().__init__()
            self.changes = 0

        def eventFilter(self, watched: object, event: QEvent) -> bool:  # noqa: N802
            if event.type() == QEvent.Type.ToolTipChange:
                self.changes += 1
            return super().eventFilter(watched, event)

    window, ros = _window(_operational_snapshot(armed=False))
    application = _application()
    counter = TooltipCounter()
    application.installEventFilter(counter)
    try:
        with (
            patch.object(
                window.operations,
                "apply_availability",
                wraps=window.operations.apply_availability,
            ) as apply_operations,
            patch.object(
                window.waypoints,
                "apply_availability",
                wraps=window.waypoints.apply_availability,
            ) as apply_waypoints,
        ):
            window._refresh()
            application.processEvents()
            assert apply_operations.call_count == 0
            assert apply_waypoints.call_count == 0
            assert counter.changes == 0

            # 快照数值变化仍须立即进入原有遥测显示路径。
            ros.current_snapshot = replace(ros.current_snapshot, z=1.234)
            window._refresh()
            application.processEvents()
            assert window.operations.altitude_summary_value.text() == "+1.23"
            assert apply_operations.call_count == 0

            # 门控输入变化时，两块面板仍各完整应用一次原逻辑。
            window._environment_active = True
            window._connection_mode = "simulation"
            window._refresh()
            assert apply_operations.call_count == 1
            assert apply_waypoints.call_count == 1
    finally:
        application.removeEventFilter(counter)
        _close_window(window)


def test_compact_status_menu_shadow_and_external_entries_are_present() -> None:
    """顶部保留上位机、摄像头、修正、红色退出入口和四周阴影。"""
    window, _ros = _window(_operational_snapshot(armed=False))
    try:
        assert window.findChild(type(window.connection_label), "windowTitle") is None
        assert [action.text() for action in window.menuBar().actions()] == [
            "设置",
            "帮助",
        ]
        assert [action.text() for action in window.settings_menu.actions()] == [
            "显示实时日志",
            "恢复默认布局",
        ]
        assert window.upstream_panel_button.isVisible()
        assert window.upstream_panel_button.text() == "上位机通讯面板"
        assert window.camera_panel_button.isVisible()
        assert window.camera_panel_button.text() == "摄像头配置面板"
        assert window.correction_panel_button.isVisible()
        assert window.correction_panel_button.text() == "Tag-Odin 修正面板"
        assert not hasattr(window, "terminal_button")
        assert window.exit_button.isVisible()
        assert window.exit_button.text() == "退出地面站"
        assert window.exit_button.property("role") == "danger"
        assert window.camera_panel_button.x() > window.upstream_panel_button.x()
        assert window.correction_panel_button.x() > window.camera_panel_button.x()
        assert window.exit_button.x() > window.correction_panel_button.x()
        assert window.windowFlags() & Qt.WindowType.FramelessWindowHint
        assert window.testAttribute(Qt.WidgetAttribute.WA_TranslucentBackground)
        assert window.outer_window_frame.graphicsEffect() is not None
        assert window.outer_window_frame.geometry() == window.rect().adjusted(
            14, 14, -14, -14
        )
        assert window.window_surface.graphicsEffect() is None
        assert window.minimize_button.isVisible()
        assert window.maximize_button.isVisible()
        assert window.close_button.isVisible()
        assert window._resize_edges_at(0, 0) == (Qt.Edge.LeftEdge | Qt.Edge.TopEdge)
        assert window._resize_edges_at(window.width(), window.height()) == (
            Qt.Edge.RightEdge | Qt.Edge.BottomEdge
        )
        assert window.activity_banner.maximumHeight() <= 42
        for badge in (
            window.link_badge,
            window.aircraft_badge,
            window.mode_badge,
            window.control_badge,
        ):
            assert badge.maximumHeight() <= 42
    finally:
        _close_window(window)


def test_all_message_boxes_use_frameless_shadow_surface() -> None:
    """确认、警告、帮助和关于共用带标题、边框与阴影的子窗口实现。"""
    window, _ros = _window(_operational_snapshot(armed=False))
    dialog = window._message_box(
        "确认起飞",
        "已读取 5 个绝对本地 ENU 航点（单次上限 256 个）。\n"
        "Yaw 按角度读取，确认后将清空并替换当前 GUI 航点列表。\n\n"
        "该操作不会取消已经发送到机载端的任务。",
        QMessageBox.Icon.Warning,
        QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.Cancel,
        QMessageBox.StandardButton.Cancel,
    )
    try:
        dialog.show()
        QTest.qWait(50)
        QApplication.instance().processEvents()
        assert dialog.windowFlags() & Qt.WindowType.FramelessWindowHint
        assert dialog.testAttribute(Qt.WidgetAttribute.WA_TranslucentBackground)
        assert dialog.surface.graphicsEffect() is not None
        assert dialog.surface.geometry() == dialog.rect().adjusted(14, 14, -14, -14)
        title = dialog.findChild(type(window.connection_label), "dialogTitle")
        close_button = dialog.findChild(QPushButton, "dialogCloseButton")
        assert title is not None
        assert close_button is not None
        assert dialog.defaultButton() is dialog.button(
            QMessageBox.StandardButton.Cancel
        )
        assert dialog.width() >= 430
        assert dialog.height() >= 180
        message_label = dialog.findChild(QLabel, "qt_msgbox_label")
        assert message_label is not None
        assert message_label.height() >= message_label.heightForWidth(
            message_label.width()
        )
    finally:
        dialog.close()
        _close_window(window)


def test_maximized_window_removes_shadow_margin_and_restores_it() -> None:
    """最大化时不保留透明缝隙，还原后恢复完整外缘阴影。"""
    window, _ros = _window(_operational_snapshot(armed=False))
    try:
        window._toggle_maximized()
        QTest.qWait(40)
        assert window.isMaximized()
        assert window.contentsMargins().left() == 1
        assert not window._window_shadow.isEnabled()
        assert window.outer_window_frame.property("windowMaximized") is True
        assert window.maximize_button.text() == "❐"

        window._toggle_maximized()
        QTest.qWait(40)
        assert not window.isMaximized()
        assert window.contentsMargins().left() == 15
        assert window._window_shadow.isEnabled()
        assert window.outer_window_frame.property("windowMaximized") is False
        assert window.maximize_button.text() == "□"
    finally:
        _close_window(window)


def test_log_auto_scroll_can_be_disabled() -> None:
    """关闭自动滚动后，追加日志不得把视口强制拉到底部。"""
    application = _application()
    events = EventLog()
    panel = LogPanel(events)
    panel.resize(640, 240)
    panel.show()
    application.processEvents()

    for index in range(80):
        events.info("source", f"seed line {index:03d} " + ("x" * 40))
    panel.poll()
    application.processEvents()

    vertical = panel.viewer.verticalScrollBar()
    assert vertical.maximum() > 0
    vertical.setValue(0)
    panel.auto_scroll.setChecked(False)
    application.processEvents()
    assert vertical.value() == 0

    events.info("source", "new line while auto-scroll disabled " + ("y" * 40))
    panel.poll()
    application.processEvents()
    # 允许 1px 量级布局误差，但绝不能跳到末尾。
    assert vertical.value() <= 2
    assert vertical.value() < vertical.maximum()
    panel.close()
