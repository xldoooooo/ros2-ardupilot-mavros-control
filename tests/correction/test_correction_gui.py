"""correction 闭环的 Qt 交互回归；使用共享替身，不连接实机。"""

from __future__ import annotations

from tests.support.qt import (
    _FakeEnvironment,
    _close_window,
    _operational_snapshot,
    _window,
)
import sys
from unittest.mock import patch
from ground_station_core.config import PROJECT_ROOT


def test_correction_panel_launcher_is_detached_from_ground_station_cleanup() -> None:
    """修正面板紧邻摄像头入口，独立启动且不进入地面站清理链。"""
    environment = _FakeEnvironment()
    window, _ros = _window(_operational_snapshot(armed=False), environment=environment)
    try:
        with patch(
            "ground_station_core.qt_ui.main_window.QProcess.startDetached",
            return_value=(True, 9876),
        ) as start_detached:
            window._open_correction_panel()

        panel_script = PROJECT_ROOT / "correction_service" / "correction_panel.py"
        start_detached.assert_called_once_with(
            sys.executable, [str(panel_script)], str(panel_script.parent)
        )
        assert environment.cleanup_calls == 0
        assert "已打开独立 Tag-Odin 修正面板" in (
            window.activity_banner.message_label.text()
        )
    finally:
        _close_window(window)
