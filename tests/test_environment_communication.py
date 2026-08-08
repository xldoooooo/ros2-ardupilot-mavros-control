"""完整实机连接与独立零命令通讯检测的职责边界回归。"""

from __future__ import annotations

import threading
import time
from types import SimpleNamespace

import pytest

from ground_station_core.config import INTERFACE_VERSION
from ground_station_core.environment import EnvironmentInitializer
from ground_station_core.event_log import EventLog
from ground_station_core.models import VehicleSnapshot
from ground_station_core.process_manager import CleanupReport


class _TrackingSupervisor:
    """记录本地检查/清理；可配置为只要诊断触碰进程管理就失败。"""

    def __init__(self, *, forbid_process_calls: bool = False) -> None:
        self.forbid_process_calls = forbid_process_calls
        self.run_calls = 0
        self.terminate_calls = 0

    def run_checked(self, *_args, **_kwargs) -> object:
        """模拟完整连接所需的本地 ROS 包检查。"""
        self.run_calls += 1
        if self.forbid_process_calls:
            raise AssertionError("通讯检测不得执行本地/远端进程命令")
        return SimpleNamespace(returncode=0, stdout="/tmp/guided_interfaces")

    def terminate_all(self) -> CleanupReport:
        """模拟完整连接前的本地仿真清理。"""
        self.terminate_calls += 1
        if self.forbid_process_calls:
            raise AssertionError("通讯检测不得启动或停止任何进程")
        return CleanupReport()


class _DiagnosticRos:
    """生成状态接收统计，并让任何租约、维护或飞行入口立即失败。"""

    def __init__(self, snapshot: VehicleSnapshot, events: EventLog) -> None:
        self.ready = True
        self.error = None
        self.event_log = events
        self.current_snapshot = snapshot
        self.control_enabled = False
        self.remote_logs_enabled = False
        self._status_count = 0

    def snapshot(self) -> VehicleSnapshot:
        """返回测试指定的权威机载快照。"""
        return self.current_snapshot

    def status_observation(self) -> tuple[int, tuple[float, ...]]:
        """模拟窗口内连续到达的十条状态及其本地接收时间。"""
        now = time.monotonic()
        self._status_count += 10
        times = tuple(now - 0.009 + index * 0.001 for index in range(10))
        return self._status_count, times

    def enable_remote_logs(self) -> None:
        """仅记录本地订阅开关，并注入一条真实来源格式的远端日志。"""
        self.remote_logs_enabled = True
        self.event_log.info("remote-rosout:onboard_control_node", "armed=false")

    def disable_remote_logs(self) -> None:
        """记录诊断结束后本地 rosout 订阅关闭。"""
        self.remote_logs_enabled = False

    @staticmethod
    def enable_control() -> None:
        raise AssertionError("通讯检测不得开启控制租约")

    @staticmethod
    def release_control(*_args, **_kwargs) -> bool:
        raise AssertionError("通讯检测不得发送租约释放请求")

    @staticmethod
    def request_set_rates() -> int:
        raise AssertionError("通讯检测不得配置消息频率")

    @staticmethod
    def request_set_gp_origin(*_origin: float) -> int:
        raise AssertionError("通讯检测不得写入 GPS 原点")

    @staticmethod
    def request_takeoff(*_args) -> int:
        raise AssertionError("通讯检测不得发送起飞指令")

    @staticmethod
    def mark_environment_stopped() -> None:
        raise AssertionError("通讯检测不得改变环境状态")


class _FullConnectionRos:
    """记录完整实机连接应恢复的租约、维护与原点调用。"""

    def __init__(self, events: EventLog) -> None:
        self.ready = True
        self.error = None
        self.event_log = events
        self.control_enabled = False
        self.remote_logs_enabled = False
        self.lease_error = ""
        self.calls: list[tuple[str, object]] = []
        self._snapshot = VehicleSnapshot(
            onboard_available=True,
            interface_version=INTERFACE_VERSION,
            connected=True,
            local_position_valid=True,
            control_authority=True,
            thrust_mode_verified=True,
        )

    def snapshot(self) -> VehicleSnapshot:
        """返回已经通过完整连接全部等待门的快照。"""
        return self._snapshot

    def disable_remote_logs(self) -> None:
        self.remote_logs_enabled = False
        self.calls.append(("disable_remote_logs", None))

    def enable_remote_logs(self) -> None:
        self.remote_logs_enabled = True
        self.calls.append(("enable_remote_logs", None))

    def enable_control(self) -> None:
        self.control_enabled = True
        self.calls.append(("enable_control", None))

    def release_control(self, timeout: float = 1.0) -> bool:
        self.calls.append(("release_control", timeout))
        return True

    def mark_environment_stopped(self) -> None:
        self.calls.append(("mark_environment_stopped", None))

    def request_set_rates(self) -> int:
        self.calls.append(("set_rates", None))
        return 101

    def request_set_gp_origin(self, *origin: float) -> int:
        self.calls.append(("set_gp_origin", origin))
        return 102

    @staticmethod
    def wait_for_result(_ticket: int, timeout: float) -> object:
        """让维护请求立即以终态成功返回。"""
        assert timeout == 0.2
        return SimpleNamespace(success=True, message="ok")


def _connected_snapshot(*, armed: bool = False) -> VehicleSnapshot:
    """构造兼容、在线的机载状态。"""
    return VehicleSnapshot(
        onboard_available=True,
        interface_version=INTERFACE_VERSION,
        connected=True,
        armed=armed,
        autopilot_mode="STABILIZE",
    )


def test_communication_workflow_only_observes_status_and_remote_logs(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """Wi-Fi 检测成功时不得触碰租约、维护、飞行或进程生命周期入口。"""
    monkeypatch.setattr(
        "ground_station_core.environment._COMMUNICATION_OBSERVATION_SECONDS", 0.02
    )
    events = EventLog()
    ros = _DiagnosticRos(_connected_snapshot(), events)
    supervisor = _TrackingSupervisor(forbid_process_calls=True)
    initializer = EnvironmentInitializer(
        ros, supervisor=supervisor, event_log=events
    )
    statuses: list[str] = []

    result = initializer._communication_workflow(
        lambda _level, message: statuses.append(message)
    )

    assert not ros.remote_logs_enabled
    assert supervisor.run_calls == 0
    assert supervisor.terminate_calls == 0
    assert "通讯链路检测通过" in result
    assert "未申请控制权" in result
    assert "远端日志 1 条" in result
    assert any("全程零控制命令" in message for message in statuses)


def test_communication_failure_does_not_cleanup_or_send_release() -> None:
    """Wi-Fi 检测异常收尾同样只能关闭本地日志订阅，不能执行通用清理。"""
    events = EventLog()
    ros = _DiagnosticRos(_connected_snapshot(), events)
    supervisor = _TrackingSupervisor(forbid_process_calls=True)
    initializer = EnvironmentInitializer(
        ros, supervisor=supervisor, event_log=events
    )
    initializer._communication_workflow = lambda _status: (_ for _ in ()).throw(
        RuntimeError("synthetic link failure")
    )
    finished = threading.Event()
    result: list[tuple[bool, str]] = []

    assert initializer.test_hardware_communication(
        lambda *_args: None,
        lambda success, message: (result.append((success, message)), finished.set()),
    )
    assert finished.wait(2.0)

    assert result and not result[0][0]
    assert "未申请控制权" in result[0][1]
    assert supervisor.run_calls == 0
    assert supervisor.terminate_calls == 0


def test_communication_cancel_only_stops_observation_thread(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """用户取消 Wi-Fi 检测时只能置取消事件，不能进入通用进程/租约清理。"""
    monkeypatch.setattr(
        "ground_station_core.environment._COMMUNICATION_OBSERVATION_SECONDS", 5.0
    )
    events = EventLog()
    ros = _DiagnosticRos(_connected_snapshot(), events)
    supervisor = _TrackingSupervisor(forbid_process_calls=True)
    initializer = EnvironmentInitializer(
        ros, supervisor=supervisor, event_log=events
    )
    finished = threading.Event()
    result: list[tuple[bool, str]] = []

    assert initializer.test_hardware_communication(
        lambda *_args: None,
        lambda success, message: (result.append((success, message)), finished.set()),
    )
    deadline = time.monotonic() + 1.0
    while not ros.remote_logs_enabled and time.monotonic() < deadline:
        time.sleep(0.01)
    assert ros.remote_logs_enabled
    assert initializer.cancel_hardware_communication_test()
    assert finished.wait(2.0)

    assert result and not result[0][0]
    assert "已取消" in result[0][1]
    assert "未申请控制权" in result[0][1]
    assert not ros.remote_logs_enabled
    assert supervisor.run_calls == 0
    assert supervisor.terminate_calls == 0


def test_communication_rejects_existing_control_session_without_side_effects() -> None:
    """已有本客户端控制会话时应前置拒绝，不能假称执行零命令检测。"""
    events = EventLog()
    ros = _DiagnosticRos(_connected_snapshot(), events)
    ros.control_enabled = True
    supervisor = _TrackingSupervisor(forbid_process_calls=True)
    initializer = EnvironmentInitializer(
        ros, supervisor=supervisor, event_log=events
    )

    with pytest.raises(RuntimeError, match="已有控制会话"):
        initializer._communication_workflow(lambda *_args: None)

    assert not ros.remote_logs_enabled
    assert supervisor.run_calls == 0
    assert supervisor.terminate_calls == 0


def test_full_hardware_connection_restores_control_rates_and_origin() -> None:
    """原连接实机按钮仍须申请控制、配置频率、写原点并保留远端日志。"""
    events = EventLog()
    ros = _FullConnectionRos(events)
    supervisor = _TrackingSupervisor()
    initializer = EnvironmentInitializer(
        ros, supervisor=supervisor, event_log=events
    )
    origin = (31.0, 121.0, 10.0)

    result = initializer._hardware_workflow(origin, lambda *_args: None)

    assert "连接完成" in result
    assert ros.control_enabled
    assert ros.remote_logs_enabled
    assert ("enable_control", None) in ros.calls
    assert ("set_rates", None) in ros.calls
    assert ("set_gp_origin", origin) in ros.calls
    assert supervisor.run_calls == 1
    assert supervisor.terminate_calls == 1
