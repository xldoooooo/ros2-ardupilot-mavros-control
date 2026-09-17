"""Qt 桌面平台入口在 X11、Wayland 与显式覆盖下的兼容回归。"""

from __future__ import annotations

import os
from unittest.mock import patch

from ground_station import (
    _configure_wayland_window_decorations as configure_ground_station,
)
from video_service.camera_panel import (
    _configure_wayland_window_decorations as configure_camera_panel,
)


def test_wayland_entrypoints_select_adwaita_decoration_before_qapplication() -> None:
    """两个 GUI 入口在原生 Wayland 下都避开无阴影的默认 bradient。"""
    for configure in (configure_ground_station, configure_camera_panel):
        with patch.dict(
            os.environ,
            {"XDG_SESSION_TYPE": "wayland", "WAYLAND_DISPLAY": "wayland-0"},
            clear=True,
        ):
            configure()
            assert os.environ["QT_WAYLAND_DECORATION"] == "adwaita"


def test_qt_decoration_selection_respects_x11_and_explicit_override() -> None:
    """X11 不新增 Wayland 变量，操作者显式插件选择也不会被覆盖。"""
    for configure in (configure_ground_station, configure_camera_panel):
        with patch.dict(
            os.environ,
            {"XDG_SESSION_TYPE": "x11", "DISPLAY": ":0"},
            clear=True,
        ):
            configure()
            assert "QT_WAYLAND_DECORATION" not in os.environ

        with patch.dict(
            os.environ,
            {
                "XDG_SESSION_TYPE": "wayland",
                "WAYLAND_DISPLAY": "wayland-0",
                "QT_WAYLAND_DECORATION": "custom-decoration",
            },
            clear=True,
        ):
            configure()
            assert os.environ["QT_WAYLAND_DECORATION"] == "custom-decoration"
