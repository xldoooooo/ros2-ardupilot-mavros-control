"""完整实机连接与独立零命令通讯检测的职责边界回归。"""

from __future__ import annotations

import os
from pathlib import Path
import threading
import time
from types import SimpleNamespace

import pytest

from ground_station_core.config import (
    HARDWARE_DISCOVERY_RANGE,
    HARDWARE_DOMAIN_ID,
    INTERFACE_VERSION,
    SIMULATION_DISCOVERY_RANGE,
    SIMULATION_DOMAIN_ID,
)
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


class _BlockingSupervisor(_TrackingSupervisor):
    """让首个清理停在进程阶段，以验证第二个调用只等待同一结果。"""

    def __init__(self) -> None:
        super().__init__()
        self.entered = threading.Event()
        self.release = threading.Event()

    def terminate_all(self) -> CleanupReport:
        self.terminate_calls += 1
        self.entered.set()
        assert self.release.wait(2.0)
        return CleanupReport(managed_stopped=4)


class _SimulationSupervisor(_TrackingSupervisor):
    """验证仿真依赖并行检查及四个受管进程的启动顺序。"""

    def __init__(self) -> None:
        super().__init__()
        self.package_barrier = threading.Barrier(4)
        self.packages: list[str] = []
        self.mavproxy_probes = 0
        self.start_calls: list[tuple[str, tuple[str, ...]]] = []
        self.start_kwargs: dict[str, dict[str, object]] = {}
        self.sequence: list[str] = []
        self.log_directory = Path("/tmp/task13-simulation-test")

    def run_checked(self, command, **_kwargs) -> object:
        """四个检查必须同时到达屏障，串行实现会在此测试失败。"""
        argv = tuple(str(part) for part in command)
        if Path(argv[0]).name == "mavproxy.py":
            self.mavproxy_probes += 1
            return SimpleNamespace(returncode=0, stdout="MAVProxy Version: test")
        package = argv[-1]
        self.packages.append(package)
        self.package_barrier.wait(timeout=2.0)
        return SimpleNamespace(returncode=0, stdout=f"/tmp/{package}")

    def start(self, name, command, **kwargs) -> object:
        """记录启动命令并返回始终存活的轻量进程记录。"""
        argv = tuple(str(part) for part in command)
        self.start_calls.append((str(name), argv))
        self.start_kwargs[str(name)] = dict(kwargs)
        self.sequence.append(f"start:{name}")
        return SimpleNamespace(
            name=str(name),
            running=True,
            log_path=Path(f"/tmp/{name}.log"),
        )


class _PreviewSupervisor(_TrackingSupervisor):
    """记录 RViz 复用/启动及其显式 ROS 传输环境。"""

    def __init__(self) -> None:
        super().__init__()
        self.records: dict[str, object] = {}
        self.start_calls: list[tuple[str, tuple[str, ...], dict[str, object]]] = []

    def get(self, name: str) -> object | None:
        """返回指定模拟进程记录。"""
        return self.records.get(name)

    def start(self, name, command, **kwargs) -> object:
        """记录 launch 命令和环境，并返回稳定存活的进程。"""
        record = SimpleNamespace(
            name=str(name),
            running=True,
            log_path=Path(f"/tmp/{name}.log"),
        )
        self.records[str(name)] = record
        self.start_calls.append(
            (str(name), tuple(str(part) for part in command), dict(kwargs))
        )
        return record


class _DiagnosticRos:
    """生成状态接收统计，并让任何租约、维护或飞行入口立即失败。"""

    def __init__(self, snapshot: VehicleSnapshot, events: EventLog) -> None:
        self.ready = True
        self.domain_id: int | None = HARDWARE_DOMAIN_ID
        self.error = None
        self.start_calls = 0
        self.stop_calls = 0
        self.start_transports: list[tuple[int, str]] = []
        self.event_log = events
        self.current_snapshot = snapshot
        self.control_enabled = False
        self.remote_logs_enabled = False
        self._status_count = 0

    def start(self, *, domain_id: int, discovery_range: str) -> None:
        """模拟首次连接操作按需创建 ROS 客户端。"""
        self.start_calls += 1
        self.domain_id = domain_id
        self.start_transports.append((domain_id, discovery_range))
        self.ready = True

    def stop(self) -> None:
        """模拟完整断开会立即销毁当前 DDS context。"""
        self.stop_calls += 1
        self.ready = False
        self.domain_id = None

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
        self.domain_id = HARDWARE_DOMAIN_ID
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

    def stop(self) -> None:
        """记录断开按钮销毁 ROS context，而非留到后续仿真切换。"""
        self.ready = False
        self.domain_id = None
        self.control_enabled = False
        self.remote_logs_enabled = False
        self.calls.append(("stop", None))

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


class _SimulationRos(_FullConnectionRos):
    """提供仿真初始化所需的就绪状态与维护结果，不生成真实 ROS 流量。"""

    def __init__(self, events: EventLog) -> None:
        super().__init__(events)
        self.domain_id = SIMULATION_DOMAIN_ID

    def request_set_rates(self) -> int:
        self.calls.append(("set_rates", None))
        return 201


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


def test_communication_workflow_starts_ros_lazily(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """打开 GUI 时可保持 DDS 空闲，首次只读检测才启动 ROS。"""
    monkeypatch.setattr(
        "ground_station_core.environment._COMMUNICATION_OBSERVATION_SECONDS", 0.02
    )
    events = EventLog()
    ros = _DiagnosticRos(_connected_snapshot(), events)
    ros.ready = False
    initializer = EnvironmentInitializer(
        ros,
        supervisor=_TrackingSupervisor(forbid_process_calls=True),
        event_log=events,
    )

    result = initializer._communication_workflow(lambda *_args: None)

    assert ros.start_calls == 1
    assert ros.start_transports == [
        (HARDWARE_DOMAIN_ID, HARDWARE_DISCOVERY_RANGE)
    ]
    assert ros.ready
    assert "通讯链路检测通过" in result


def test_same_gui_switches_from_simulation_domain_to_hardware_domain() -> None:
    """结束仿真后应在同一控制器对象内切回 domain 0，无需重启 GUI。"""
    events = EventLog()
    ros = _DiagnosticRos(VehicleSnapshot(), events)
    initializer = EnvironmentInitializer(
        ros,
        supervisor=_TrackingSupervisor(),
        event_log=events,
    )

    initializer._ensure_ros_ready(
        SIMULATION_DOMAIN_ID, SIMULATION_DISCOVERY_RANGE
    )
    initializer._ensure_ros_ready(
        HARDWARE_DOMAIN_ID, HARDWARE_DISCOVERY_RANGE
    )

    assert ros.start_transports == [
        (SIMULATION_DOMAIN_ID, SIMULATION_DISCOVERY_RANGE),
        (HARDWARE_DOMAIN_ID, HARDWARE_DISCOVERY_RANGE),
    ]
    assert ros.domain_id == HARDWARE_DOMAIN_ID


@pytest.mark.parametrize("prebuilt_sitl", [True, False])
def test_simulation_parallelizes_safe_startup_and_uses_sim_only_fast_param_check(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path, prebuilt_sitl: bool
) -> None:
    """安全并行且仅仿真快查参数；有二进制才跳过 ArduPilot 构建。"""
    sim_vehicle = tmp_path / "sim_vehicle.py"
    sim_vehicle.touch()
    if prebuilt_sitl:
        sitl_binary = tmp_path / "build" / "sitl" / "bin" / "arducopter"
        sitl_binary.parent.mkdir(parents=True)
        sitl_binary.touch()
    apm_config = tmp_path / "apm.yaml"
    apm_config.touch()
    monkeypatch.setattr(
        "ground_station_core.environment.find_sim_vehicle", lambda: sim_vehicle
    )
    mavproxy = tmp_path / "mavproxy-bin" / "mavproxy.py"
    mavproxy.parent.mkdir()
    mavproxy.touch()
    monkeypatch.setattr(
        "ground_station_core.environment.find_mavproxy", lambda: mavproxy
    )
    monkeypatch.setattr(
        "ground_station_core.environment.ardupilot_root", lambda _path: tmp_path
    )
    monkeypatch.setattr(
        "ground_station_core.environment.mavros_apm_config", lambda: apm_config
    )

    events = EventLog()
    ros = _SimulationRos(events)
    supervisor = _SimulationSupervisor()
    initializer = EnvironmentInitializer(
        ros, supervisor=supervisor, event_log=events
    )
    stable_calls: list[tuple[str, float]] = []

    def wait_tcp(_host, _port, _timeout, _process) -> bool:
        """SITL 端口等待开始前 RViz 必须已经启动。"""
        supervisor.sequence.append("wait:sitl-tcp")
        assert supervisor.sequence[:3] == [
            "start:sitl",
            "start:rviz",
            "wait:sitl-tcp",
        ]
        return True

    initializer._wait_tcp = wait_tcp
    initializer._wait_process_stable = lambda process, duration: stable_calls.append(
        (process.name, duration)
    )
    initializer._wait_onboard = lambda *_args: None
    initializer._wait_connected = lambda *_args: None
    initializer._wait_thrust_mode = lambda *_args: None
    initializer._wait_control_authority = lambda *_args: None
    initializer._wait_local_position = lambda *_args: None
    initializer._wait_ticket = lambda *_args: SimpleNamespace(
        success=True, message="ok"
    )

    result = initializer._simulation_workflow(lambda *_args: None)

    assert "仿真闭环初始化完成" in result
    assert sorted(supervisor.packages) == [
        "guided_interfaces",
        "guided_sim",
        "mavros",
        "onboard_control",
    ]
    assert supervisor.mavproxy_probes == 1
    assert [name for name, _command in supervisor.start_calls] == [
        "sitl",
        "rviz",
        "mavros_sim",
        "onboard_control",
    ]
    onboard_command = dict(supervisor.start_calls)["onboard_control"]
    sitl_command = dict(supervisor.start_calls)["sitl"]
    rviz_command = dict(supervisor.start_calls)["rviz"]
    assert ("--no-rebuild" in sitl_command) is prebuilt_sitl
    sitl_environment = supervisor.start_kwargs["sitl"]["extra_environment"]
    assert sitl_environment["PATH"].split(os.pathsep)[0] == str(mavproxy.parent)
    assert "fcu_parameter_check_initial_delay_seconds:=2.0" in onboard_command
    assert rviz_command[:4] == ("nice", "-n", "5", "ros2")
    assert "pose_topic:=/mavros/local_position/pose" in rviz_command
    assert stable_calls == [("mavros_sim", 1.0), ("rviz", 0.2)]
    assert ("set_rates", None) in ros.calls


def test_waypoint_preview_reuses_simulation_rviz_without_second_window() -> None:
    """仿真已有 RViz 时点击预览不得再次调用任何进程启动。"""
    events = EventLog()
    ros = SimpleNamespace(
        ready=True,
        domain_id=SIMULATION_DOMAIN_ID,
        discovery_range=SIMULATION_DISCOVERY_RANGE,
        event_log=events,
    )
    supervisor = _PreviewSupervisor()
    supervisor.records["rviz"] = SimpleNamespace(running=True)
    initializer = EnvironmentInitializer(
        ros, supervisor=supervisor, event_log=events
    )

    message = initializer.ensure_waypoint_preview("simulation")

    assert "复用仿真" in message
    assert supervisor.start_calls == []
    assert supervisor.run_calls == 0


def test_hardware_waypoint_preview_starts_one_isolated_local_rviz(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """实机预览只启动一个本地窗口，并显式继承 domain 0/SUBNET。"""
    monkeypatch.setenv("ROS_DISTRO", "jazzy")
    events = EventLog()
    ros = SimpleNamespace(
        ready=True,
        domain_id=HARDWARE_DOMAIN_ID,
        discovery_range=HARDWARE_DISCOVERY_RANGE,
        event_log=events,
    )
    supervisor = _PreviewSupervisor()
    initializer = EnvironmentInitializer(
        ros, supervisor=supervisor, event_log=events
    )

    first = initializer.ensure_waypoint_preview("hardware")
    second = initializer.ensure_waypoint_preview("hardware")

    assert "独立实机 RViz" in first
    assert "复用地面端实机 RViz" in second
    assert len(supervisor.start_calls) == 1
    name, command, kwargs = supervisor.start_calls[0]
    assert name == "rviz_hardware_preview"
    assert command == (
        "nice",
        "-n",
        "5",
        "ros2",
        "launch",
        "guided_sim",
        "visualize.launch.py",
        "pose_topic:=/ground_station/vehicle_pose",
    )
    environment = kwargs["extra_environment"]
    assert environment["ROS_DOMAIN_ID"] == str(HARDWARE_DOMAIN_ID)
    assert environment["ROS_AUTOMATIC_DISCOVERY_RANGE"] == "SUBNET"
    assert supervisor.run_calls == 2


def test_waypoint_preview_rejects_transport_mismatch_before_process_access() -> None:
    """会话模式与当前 DDS domain 不一致时不得尝试打开 RViz。"""
    events = EventLog()
    ros = SimpleNamespace(
        ready=True,
        domain_id=SIMULATION_DOMAIN_ID,
        discovery_range=SIMULATION_DISCOVERY_RANGE,
        event_log=events,
    )
    supervisor = _PreviewSupervisor()
    initializer = EnvironmentInitializer(
        ros, supervisor=supervisor, event_log=events
    )

    with pytest.raises(RuntimeError, match="拒绝跨域"):
        initializer.ensure_waypoint_preview("hardware")

    assert supervisor.start_calls == []
    assert supervisor.run_calls == 0


def test_session_cleanup_stops_dds_context_at_disconnect_boundary() -> None:
    """断开真机/终止仿真后不得在 IDLE 留下旧 domain participant。"""
    events = EventLog()
    ros = _FullConnectionRos(events)
    initializer = EnvironmentInitializer(
        ros,
        supervisor=_TrackingSupervisor(),
        event_log=events,
    )

    report = initializer.cleanup()

    assert report.success
    assert not ros.ready
    assert ros.domain_id is None
    assert ros.calls[-1] == ("stop", None)


def test_failed_hardware_connection_stops_dds_and_clears_live_telemetry() -> None:
    """完整连接失败也必须销毁观察端，不能在 GUI IDLE 后继续刷新真机状态。"""
    events = EventLog()
    ros = _FullConnectionRos(events)
    supervisor = _TrackingSupervisor()
    initializer = EnvironmentInitializer(
        ros,
        supervisor=supervisor,
        event_log=events,
    )
    finished = threading.Event()
    results: list[tuple[bool, str]] = []

    def fail_after_connecting(_origin, _status) -> str:
        """模拟原点确认失败前已经建立状态、日志和租约链路。"""
        ros.enable_remote_logs()
        ros.enable_control()
        raise RuntimeError("GPS 原点确认超时")

    initializer._hardware_workflow = fail_after_connecting
    assert initializer.initialize_hardware(
        (30.0, 120.0, 10.0),
        lambda *_args: None,
        lambda success, message: (
            results.append((success, message)),
            finished.set(),
        ),
    )
    assert finished.wait(2.0)

    assert results and not results[0][0]
    assert "GPS 原点确认超时" in results[0][1]
    assert "本地 ROS 连接与仿真进程已清理" in results[0][1]
    assert not ros.ready
    assert ros.domain_id is None
    assert not ros.control_enabled
    assert not ros.remote_logs_enabled
    assert supervisor.terminate_calls == 1
    assert ros.calls[-1] == ("stop", None)


def test_concurrent_cleanup_requests_share_one_bounded_process_cleanup() -> None:
    """终止按钮与退出同时发生时不得重复清理或互锁。"""
    events = EventLog()
    ros = _FullConnectionRos(events)
    supervisor = _BlockingSupervisor()
    initializer = EnvironmentInitializer(
        ros,
        supervisor=supervisor,
        event_log=events,
    )
    reports: list[CleanupReport] = []

    first = threading.Thread(
        target=lambda: reports.append(initializer._terminate_local_processes())
    )
    second = threading.Thread(
        target=lambda: reports.append(initializer._terminate_local_processes())
    )
    first.start()
    assert supervisor.entered.wait(1.0)
    second.start()
    time.sleep(0.05)
    supervisor.release.set()
    first.join(timeout=2.0)
    second.join(timeout=2.0)

    assert not first.is_alive()
    assert not second.is_alive()
    assert supervisor.terminate_calls == 1
    assert reports == [
        CleanupReport(managed_stopped=4),
        CleanupReport(managed_stopped=4),
    ]


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
