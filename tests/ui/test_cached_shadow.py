"""缓存阴影复用与隐藏动画时序的隔离 Qt 回归。"""

from unittest.mock import patch

from PySide6.QtCore import QAbstractAnimation
from PySide6.QtGui import QColor
from PySide6.QtWidgets import QFrame

from ground_station_core.event_log import LogLevel
from ground_station_core.qt_ui.cached_shadow import CachedShadowEffect, _shadow_texture
from ground_station_core.qt_ui.widgets import ActivityBanner
from tests.support.qt import _application


def test_resize_and_glow_opacity_reuse_shadow_texture() -> None:
    """连续缩放和脉冲透明度变化不得重新执行模糊生成。"""
    app = _application()
    frame = QFrame()
    frame.setStyleSheet("background: white; border-radius: 8px;")
    frame.resize(500, 300)
    effect = CachedShadowEffect(frame)
    frame.setGraphicsEffect(effect)
    frame.show()
    app.processEvents()
    try:
        # 将颜色转为动画使用的完整 alpha 缓存后，之后只变透明度。
        effect.setColor(QColor(32, 173, 103, 80))
        app.processEvents()
        # Wayland 首次 expose 异步完成；主动绘制一次后再测缓存复用。
        frame.grab()
        initial = _shadow_texture.cache_info().misses
        for size, alpha in ((550, 120), (600, 217), (470, 40)):
            frame.resize(size, size // 2)
            effect.setColor(QColor(32, 173, 103, alpha))
            app.processEvents()
            frame.grab()
            assert effect.color().alpha() == alpha
        assert _shadow_texture.cache_info().misses == initial
    finally:
        frame.close()


def test_hidden_banner_pauses_frames_and_preserves_elapsed_time() -> None:
    """隐藏期间不耗费动画帧，已过期成功提示不能在显示时重放。"""
    app = _application()
    banner = ActivityBanner()
    banner.resize(600, 42)
    banner.show()
    app.processEvents()
    try:
        banner.set_message("完成", LogLevel.INFO)
        banner._animation.setCurrentTime(400)
        with patch("ground_station_core.qt_ui.widgets.monotonic", return_value=100.0):
            banner.hide()
        assert banner._animation.state() == QAbstractAnimation.State.Paused
        assert banner._animation.currentTime() == 400
        with patch("ground_station_core.qt_ui.widgets.monotonic", return_value=102.0):
            banner.show()
        assert banner._animation.state() == QAbstractAnimation.State.Stopped
        assert banner._glow.color().alpha() == 0

        banner.hide()
        with patch("ground_station_core.qt_ui.widgets.monotonic", return_value=200.0):
            banner.set_message("连接中", LogLevel.INFO, state="busy")
        assert banner._animation.state() == QAbstractAnimation.State.Paused
        with patch("ground_station_core.qt_ui.widgets.monotonic", return_value=202.0):
            banner.show()
        assert banner._animation.state() == QAbstractAnimation.State.Running
        assert banner._animation.currentTime() == 2000
        assert banner._animation.currentLoopTime() == 600
    finally:
        banner.close()
