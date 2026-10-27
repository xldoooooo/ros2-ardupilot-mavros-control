"""验证地面站启动偏好与自动连接；全部使用替身，不连接飞机。"""

from dataclasses import replace
from pathlib import Path

from ground_station_core import startup
from ground_station_core.event_log import EventLog
from ground_station_core.qt_ui.main_window import GroundStationWindow
from tests.support.qt import (
    _FakeEnvironment,
    _FakeRosController,
    _FakeUpstreamService,
    _application,
    _close_window,
    _operational_snapshot,
)


def _startup_window(path: Path, environment: _FakeEnvironment) -> tuple:
    """为每个场景创建独立偏好文件与纯本地 Qt 窗口。"""
    _application()
    events = EventLog()
    ros = _FakeRosController(events, replace(_operational_snapshot(armed=False), on_ground=True))
    window = GroundStationWindow(
        event_log=events,
        ros_controller=ros,
        environment=environment,
        upstream_service=_FakeUpstreamService(),
        startup_preferences_path=path,
    )
    return window, ros


def test_saved_auto_connect_starts_full_workflow_with_saved_origin(tmp_path) -> None:
    """首次打开也使用原正式连接流程，且偏好可跨窗口恢复。"""
    path = tmp_path / "startup.json"
    origin = (31.0, 121.0, 28.0)
    startup.save_preferences(True, origin, path)
    environment = _FakeEnvironment()
    window, ros = _startup_window(path, environment)
    try:
        _application().processEvents()
        assert window.auto_connect_checkbox.isChecked()
        assert window._connection_mode == "hardware"
        assert environment.last_origin == origin
        assert not window.windowIcon().isNull()
        assert not ros.calls  # 自动连接只经环境入口，不请求解锁、起飞或航点。
        window._confirm_action = lambda *_args, **_kwargs: True
        window._disconnect_hardware()
        assert not window.auto_connect_checkbox.isChecked()
        assert startup.load_preferences(path)["auto_connect_aircraft"] is False
        if window._cleanup_thread is not None:
            window._cleanup_thread.join(timeout=1)
        _application().processEvents()
    finally:
        _close_window(window)


def test_failed_first_connection_retries_and_simulation_stays_selected(tmp_path) -> None:
    """飞机晚开机时失败后继续重试，仿真运行期间不切换到实机。"""
    environment = _FakeEnvironment()
    window, ros = _startup_window(tmp_path / "startup.json", environment)
    attempts = []

    def connect(origin, _status, done) -> bool:
        """首轮模拟离线，次轮模拟飞机上线。"""
        attempts.append(origin)
        done(len(attempts) > 1, "离线" if len(attempts) == 1 else "已连接")
        return True

    environment.initialize_hardware = connect
    try:
        window.auto_connect_checkbox.setChecked(True)
        window._auto_connect_hardware()
        assert len(attempts) == 1 and not window._environment_active
        window._auto_connect_hardware()
        assert len(attempts) == 1
        window._auto_connect_next_attempt = 0
        window._auto_connect_hardware()
        assert len(attempts) == 2 and window._environment_active
        window._connection_mode = "simulation"
        window._auto_connect_next_attempt = 0
        window._auto_connect_hardware()
        assert len(attempts) == 2 and not ros.calls
    finally:
        _close_window(window)


def test_returning_link_maintains_origin_only_after_disarm(tmp_path) -> None:
    """已有会话掉线后等待飞机返回，已解锁时不会重新执行连接维护。"""
    environment = _FakeEnvironment()
    window, ros = _startup_window(tmp_path / "startup.json", environment)
    maintenance_attempts = []

    def maintain(origin, _status, done, *, restart_preview=False) -> bool:
        """第一轮维护失败，下一轮成功；两次均不触碰原环境。"""
        maintenance_attempts.append(origin)
        assert restart_preview
        done(len(maintenance_attempts) > 1, "维护完成" if len(maintenance_attempts) > 1 else "稍后重试")
        return True

    environment.maintain_hardware_connection = maintain
    try:
        window.auto_connect_checkbox.setChecked(True)
        window._auto_connect_hardware()
        window._auto_connect_next_attempt = 0
        ros.current_snapshot = replace(ros.current_snapshot, onboard_available=False)
        window._auto_connect_hardware()
        assert window._auto_reconnect_required
        environment.last_origin = None
        window._waypoint_preview_active = True
        ros.current_snapshot = replace(ros.current_snapshot, onboard_available=True, armed=True)
        window._auto_connect_hardware()
        assert not maintenance_attempts
        ros.current_snapshot = replace(ros.current_snapshot, armed=False)
        window._auto_connect_hardware()
        assert maintenance_attempts == [window.operations.origin()]
        assert window._environment_active and window._connection_mode == "hardware"
        assert window._waypoint_preview_active and window._auto_reconnect_required
        window._auto_connect_next_attempt = 0
        window._auto_connect_hardware()
        assert len(maintenance_attempts) == 2
        assert not window._auto_reconnect_required and not ros.calls
        assert not environment.cleanup_calls and environment.last_origin is None
    finally:
        _close_window(window)


def test_desktop_icon_and_login_autostart_use_project_launcher(tmp_path, monkeypatch) -> None:
    """桌面与登录入口共用原启动脚本，开关不修改其他应用自启项。"""
    monkeypatch.setenv("XDG_CONFIG_HOME", str(tmp_path / "config"))
    monkeypatch.setenv("XDG_DATA_HOME", str(tmp_path / "data"))
    launcher = startup.install_desktop_launcher()
    startup.set_login_autostart(True)
    contents = startup.autostart_path().read_text(encoding="utf-8")
    assert launcher.read_text(encoding="utf-8") == contents
    assert "scripts/ground/start_ground_all.sh" in contents
    assert f"Icon={startup.ICON_PATH}" in contents
    startup.set_login_autostart(False)
    assert not startup.autostart_path().exists() and launcher.is_file()
