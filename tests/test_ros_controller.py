"""地面站高层协议客户端的排队、状态映射与无服务失败测试。"""

import math
import os
import time
from types import SimpleNamespace

from ground_station_core.config import INTERFACE_VERSION
from ground_station_core.event_log import EventLog, LogLevel
from ground_station_core.models import (
    FlightMode,
    VehicleSnapshot,
    WaypointReferenceGenerator,
    WaypointTrackingController,
)
from ground_station_core.ros_controller import (
    GroundStationRosController,
    PREVIEW_FRAME_ID,
    VEHICLE_POSE_TOPIC,
    WAYPOINT_MARKERS_TOPIC,
    WAYPOINT_PATH_TOPIC,
    _VehicleStateStore,
)


def test_controller_rejects_takeoff_without_onboard_and_stops() -> None:
    """机载服务不存在时必须明确失败，不能退回地面站本地控制。"""
    controller = GroundStationRosController(source_id="pytest-no-onboard")
    controller.start()
    try:
        assert controller.ready
        ticket = controller.request_takeoff(0.3)
        result = controller.wait_for_result(ticket, timeout=3.0)
        assert result is not None
        assert not result.success
        assert "机载控制服务不可用" in result.message
    finally:
        controller.stop()

    assert not controller.ready
    assert controller.active_mode is FlightMode.IDLE


def test_all_flight_inputs_share_monotonic_sequence() -> None:
    """服务命令与方向意图必须共享同一序号，供机载端拒绝旧输入。"""
    controller = GroundStationRosController(source_id="pytest-order")
    takeoff = controller.request_takeoff(0.3)
    motion = controller.adjust_velocity(0.2, 0.0, 0.0, 0.0)
    waypoint = controller.request_waypoints(((1.0, 0.0, 1.0, 0.0),), strategy=1)

    queued = [controller._command_queue.get_nowait() for _ in range(3)]
    assert [item.ticket for item in queued] == [takeoff, motion, waypoint]
    assert [item.name for item in queued] == ["takeoff", "motion", "waypoints"]
    assert queued[2].argument == {
        "waypoints": ((1.0, 0.0, 1.0, 0.0),),
        "strategy": 1,
        "reference_generator": 0,
        "tracking_controller": 0,
    }
    assert takeoff < motion < waypoint


def test_waypoint_request_preserves_three_independent_gui_choices() -> None:
    """避障空壳、命令生成和跟踪控制必须作为独立字段原子排队。"""
    controller = GroundStationRosController(source_id="pytest-waypoint-methods")

    controller.request_waypoints(
        ((4.0, 0.0, 1.5, 0.0),),
        strategy=2,
        reference_generator=3,
        tracking_controller=1,
    )

    queued = controller._command_queue.get_nowait()
    assert queued.argument == {
        "waypoints": ((4.0, 0.0, 1.5, 0.0),),
        "strategy": 2,
        "reference_generator": 3,
        "tracking_controller": 1,
    }


def test_waypoint_transport_writes_all_method_fields_to_ros_request() -> None:
    """队列中的三项选择必须进入同一个 ExecuteWaypoints 服务请求。"""

    class FakeWaypoint:
        """提供 ExecuteWaypoints 所需的最小几何字段。"""

        def __init__(self) -> None:
            self.position = SimpleNamespace(x=0.0, y=0.0, z=0.0)
            self.yaw = 0.0

    class FakeRequest:
        """保存地面站写入的完整航点服务载荷。"""

        def __init__(self) -> None:
            self.stamp = None
            self.source_id = ""
            self.sequence = 0
            self.ttl_ms = 0
            self.flight_strategy = 0
            self.reference_generator = 0
            self.tracking_controller = 0
            self.waypoints: list[FakeWaypoint] = []

    class FakeClient:
        """记录异步服务请求，不伪造执行终态。"""

        def __init__(self) -> None:
            self.requests: list[FakeRequest] = []

        @staticmethod
        def service_is_ready() -> bool:
            return True

        def call_async(self, request: FakeRequest) -> object:
            self.requests.append(request)
            return SimpleNamespace(done=lambda: False)

    class FakeNow:
        """提供 builtin time 消息的最小替身。"""

        @staticmethod
        def to_msg() -> object:
            return object()

    controller = GroundStationRosController(source_id="gcs-waypoint-transport")
    controller._state._snapshot = VehicleSnapshot(
        onboard_available=True,
        interface_version=INTERFACE_VERSION,
        control_authority=True,
        lease_owner="gcs-waypoint-transport",
    )
    controller._state._last_status_time = time.monotonic()
    controller.enable_control()
    ticket = controller.request_waypoints(
        ((4.0, -1.0, 1.5, 0.25),),
        strategy=2,
        reference_generator=3,
        tracking_controller=1,
    )
    client = FakeClient()
    pending: dict[int, tuple] = {}
    controller._process_one_command(
        {
            "node": SimpleNamespace(
                get_clock=lambda: SimpleNamespace(now=lambda: FakeNow())
            ),
            "clients": {"waypoints": client},
            "ExecuteWaypoints": SimpleNamespace(Request=FakeRequest),
            "Waypoint": FakeWaypoint,
        },
        pending,
    )

    assert ticket in pending
    assert len(client.requests) == 1
    request = client.requests[0]
    assert request.flight_strategy == 2
    assert request.reference_generator == 3
    assert request.tracking_controller == 1
    waypoint = request.waypoints[0]
    assert (waypoint.position.x, waypoint.position.y, waypoint.position.z) == (
        4.0,
        -1.0,
        1.5,
    )
    assert waypoint.yaw == 0.25


def test_clear_abnormal_uses_dedicated_flight_command_code() -> None:
    """入库恢复请求必须使用 3.1 接口的独立命令码。"""

    class FakeRequest:
        """保存 FlightCommand 服务请求及全部命令常量。"""

        COMMAND_TAKEOFF = 1
        COMMAND_LAND = 2
        COMMAND_HOVER = 3
        COMMAND_CANCEL = 4
        COMMAND_CONFIGURE_RATES = 5
        COMMAND_CLEAR_ABNORMAL = 6

        def __init__(self) -> None:
            self.stamp = None
            self.source_id = ""
            self.sequence = 0
            self.ttl_ms = 0
            self.command = 0
            self.value = 0.0

    class FakeClient:
        """记录一次异步请求。"""

        def __init__(self) -> None:
            self.requests: list[FakeRequest] = []

        @staticmethod
        def service_is_ready() -> bool:
            return True

        def call_async(self, request: FakeRequest) -> object:
            self.requests.append(request)
            return SimpleNamespace(done=lambda: False)

    class FakeNow:
        @staticmethod
        def to_msg() -> object:
            return object()

    controller = GroundStationRosController(source_id="gcs-clear-abnormal")
    controller._state._snapshot = VehicleSnapshot(
        onboard_available=True,
        interface_version=INTERFACE_VERSION,
        control_authority=True,
        lease_owner="gcs-clear-abnormal",
    )
    controller._state._last_status_time = time.monotonic()
    controller.enable_control()
    ticket = controller.request_clear_abnormal()
    client = FakeClient()
    pending: dict[int, tuple] = {}
    controller._process_one_command(
        {
            "node": SimpleNamespace(
                get_clock=lambda: SimpleNamespace(now=lambda: FakeNow())
            ),
            "clients": {"flight": client},
            "FlightCommand": SimpleNamespace(Request=FakeRequest),
        },
        pending,
    )

    assert ticket in pending
    assert len(client.requests) == 1
    assert client.requests[0].command == FakeRequest.COMMAND_CLEAR_ABNORMAL


def test_waypoint_preview_builds_retained_markers_path_and_live_pose() -> None:
    """预览生成球/编号/直线路径，且不进入飞行命令或租约序号通道。"""
    from builtin_interfaces.msg import Time
    from rclpy.qos import DurabilityPolicy
    from visualization_msgs.msg import Marker

    class FakePublisher:
        """保存消息与 QoS，模拟 rclpy publisher。"""

        def __init__(self, topic: str, qos: object) -> None:
            self.topic = topic
            self.qos = qos
            self.messages: list[object] = []

        def publish(self, message: object) -> None:
            self.messages.append(message)

    class FakeNow:
        """提供标准 builtin_interfaces/Time。"""

        @staticmethod
        def to_msg() -> Time:
            return Time(sec=123, nanosec=456)

    class FakeNode:
        """按话题创建可审计的假 publisher。"""

        def __init__(self) -> None:
            self.publishers: dict[str, FakePublisher] = {}

        @staticmethod
        def get_clock() -> object:
            return SimpleNamespace(now=lambda: FakeNow())

        def create_publisher(
            self, _message_type: object, topic: str, qos: object
        ) -> FakePublisher:
            publisher = FakePublisher(topic, qos)
            self.publishers[topic] = publisher
            return publisher

    controller = GroundStationRosController(source_id="gcs-preview-message")
    controller._ready = True
    controller._active_domain_id = 231
    node = FakeNode()
    entities: dict[str, object] = {"node": node}
    waypoints = (
        (1.0, -2.0, 3.0, 0.0),
        (4.0, 5.0, 6.0, math.pi / 2.0),
    )

    assert controller.publish_waypoint_preview(waypoints)
    assert controller._next_ticket == 1
    assert controller._command_queue.empty()
    controller._process_one_waypoint_preview(entities)

    marker_publisher = node.publishers[WAYPOINT_MARKERS_TOPIC]
    path_publisher = node.publishers[WAYPOINT_PATH_TOPIC]
    pose_publisher = node.publishers[VEHICLE_POSE_TOPIC]
    assert marker_publisher.qos.durability is DurabilityPolicy.TRANSIENT_LOCAL
    assert path_publisher.qos.durability is DurabilityPolicy.TRANSIENT_LOCAL
    marker_array = marker_publisher.messages[-1]
    assert len(marker_array.markers) == 5
    assert marker_array.markers[0].action == Marker.DELETEALL
    assert [marker_array.markers[index].text for index in (2, 4)] == ["1", "2"]
    path = path_publisher.messages[-1]
    assert path.header.frame_id == PREVIEW_FRAME_ID
    assert len(path.poses) == 2
    assert path.poses[0].pose.position.x == 1.0
    assert math.isclose(path.poses[1].pose.orientation.z, math.sqrt(0.5))
    assert math.isclose(path.poses[1].pose.orientation.w, math.sqrt(0.5))

    status = SimpleNamespace(
        local_position_valid=True,
        position=SimpleNamespace(x=7.0, y=8.0, z=9.0),
        roll=0.0,
        pitch=0.0,
        yaw=math.pi / 2.0,
    )
    controller._publish_vehicle_preview_pose(entities, status)
    pose = pose_publisher.messages[-1]
    assert pose.header.frame_id == PREVIEW_FRAME_ID
    assert (pose.pose.position.x, pose.pose.position.y, pose.pose.position.z) == (
        7.0,
        8.0,
        9.0,
    )
    assert math.isclose(pose.pose.orientation.z, math.sqrt(0.5))
    assert math.isclose(pose.pose.orientation.w, math.sqrt(0.5))

    # 空快照显式删除旧 Marker 并覆盖为空 Path，不会留下上一次较长列表。
    assert controller.publish_waypoint_preview(())
    controller._process_one_waypoint_preview(entities)
    assert len(marker_publisher.messages[-1].markers) == 1
    assert marker_publisher.messages[-1].markers[0].action == Marker.DELETEALL
    assert path_publisher.messages[-1].poses == []


def test_idle_deadline_misses_do_not_spam_warn_but_active_control_still_warns() -> None:
    """空闲抖动只留在遥测；武装或控制激活后的新增超期仍必须告警。"""
    events = EventLog()
    controller = GroundStationRosController(
        source_id="gcs-deadline-log-gating", event_log=events
    )

    controller._log_status_transitions(
        VehicleSnapshot(deadline_miss_count=4),
        VehicleSnapshot(deadline_miss_count=5, max_jitter_ms=18.0),
    )
    assert not any(event.source == "controller" for event in events.snapshot())

    controller._log_status_transitions(
        VehicleSnapshot(deadline_miss_count=5, controller_active=True),
        VehicleSnapshot(
            deadline_miss_count=6,
            max_jitter_ms=19.0,
            controller_active=True,
        ),
    )
    controller_events = [
        event for event in events.snapshot() if event.source == "controller"
    ]
    assert len(controller_events) == 1
    assert controller_events[0].level is LogLevel.WARN
    assert "累计 6 次" in controller_events[0].message


def test_status_store_maps_remote_mode_and_lease_owner(monkeypatch) -> None:
    """GUI 展示必须来自机载聚合状态，而不是本地猜测。"""
    monotonic_now = [100.0]
    monkeypatch.setattr(time, "monotonic", lambda: monotonic_now[0])
    store = _VehicleStateStore("gcs-test")
    vector = SimpleNamespace(x=1.0, y=2.0, z=3.0)
    message = SimpleNamespace(
        interface_version=INTERFACE_VERSION,
        fcu_connected=True,
        armed=True,
        autopilot_mode="GUIDED",
        local_position_valid=True,
        position=vector,
        velocity=SimpleNamespace(x=0.1, y=0.2, z=0.3),
        roll=-0.1,
        pitch=-0.2,
        yaw=0.4,
        battery_valid=True,
        battery_voltage=15.8,
        battery_current=3.2,
        battery_percentage=0.74,
        control_mode=4,
        controller_active=True,
        target_position=vector,
        target_velocity=SimpleNamespace(x=0.5, y=0.0, z=0.0),
        target_acceleration=SimpleNamespace(x=0.2, y=0.0, z=-0.1),
        target_yaw=0.6,
        target_yaw_rate=0.1,
        active_reference_generator=3,
        active_tracking_controller=1,
        reference_phase=4,
        lease_owner="gcs-test",
        lease_active=True,
        active_command_sequence=44,
        waypoint_index=2,
        waypoint_count=3,
        waypoint_arrival_failure_count=9,
        vehicle_abnormal=True,
        vehicle_abnormal_reason="航点入点失败",
        message_rates_configured=True,
        thrust_mode_verified=True,
        hover_throttle=0.39,
        setpoint_conflict=False,
        failsafe_reason="",
        status_message="执行中",
        control_rate_hz=99.8,
        max_jitter_ms=1.2,
        deadline_miss_count=2,
    )
    store.update(message)
    monotonic_now[0] = 100.1
    store.update(message)
    monotonic_now[0] = 100.15
    snapshot = store.snapshot()
    status_count, receive_times = store.observation()

    assert snapshot.onboard_available
    assert snapshot.active_mode is FlightMode.WAYPOINT
    assert snapshot.control_authority
    assert snapshot.waypoint_index == 2
    assert snapshot.active_command_sequence == 44
    assert snapshot.waypoint_arrival_failure_count == 9
    assert snapshot.vehicle_abnormal
    assert snapshot.vehicle_abnormal_reason == "航点入点失败"
    assert snapshot.control_rate_hz == 99.8
    assert snapshot.hover_throttle == 0.39
    assert snapshot.roll == -0.1
    assert snapshot.pitch == -0.2
    assert snapshot.yaw == 0.4
    assert (snapshot.target_ax, snapshot.target_ay, snapshot.target_az) == (
        0.2,
        0.0,
        -0.1,
    )
    assert (
        snapshot.active_reference_generator
        is WaypointReferenceGenerator.JERK_LIMITED_S_CURVE
    )
    assert (
        snapshot.active_tracking_controller
        is WaypointTrackingController.TRAJECTORY_PD_DOB
    )
    assert snapshot.reference_phase == 4
    assert snapshot.battery_valid
    assert snapshot.battery_voltage == 15.8
    assert snapshot.battery_current == 3.2
    assert snapshot.battery_percentage == 0.74
    assert abs(snapshot.status_rate_hz - 10.0) < 1e-9
    assert abs(snapshot.status_age_seconds - 0.05) < 1e-9
    assert status_count == 2
    assert len(receive_times) == 2

    store.mark_disconnected()
    assert store.observation() == (0, ())


def test_previous_interface_version_is_rejected_before_command_transport() -> None:
    """2.2 机载端不得在 ControlStatus 3.1 升级后被误判为兼容。"""
    controller = GroundStationRosController(source_id="gcs-version-gate")
    controller._state._snapshot = VehicleSnapshot(
        onboard_available=True,
        interface_version="2.2",
        control_authority=True,
        lease_owner="gcs-version-gate",
    )
    controller._state._last_status_time = time.monotonic()
    ticket = controller.request_takeoff(0.3)

    # 版本检查发生在读取任何 ROS 实体前，空字典可证明请求没有进入传输层。
    controller._process_one_command({}, {})
    result = controller.wait_for_result(ticket, timeout=0.1)

    assert INTERFACE_VERSION == "3.1"
    assert result is not None
    assert not result.success
    assert "接口版本不兼容" in result.message


def test_compatible_onboard_is_passive_until_control_is_explicitly_enabled() -> None:
    """仅发现兼容机载端时不得自动申请租约或发布心跳。"""

    class FakeClient:
        """记录租约服务调用，不执行任何 ROS 传输。"""

        def __init__(self) -> None:
            self.requests: list[object] = []

        @staticmethod
        def service_is_ready() -> bool:
            return True

        def call_async(self, request: object) -> object:
            self.requests.append(request)
            return SimpleNamespace(done=lambda: False)

    class FakeNow:
        """提供最小 ROS 时钟消息接口。"""

        @staticmethod
        def to_msg() -> object:
            return object()

    client = FakeClient()
    controller = GroundStationRosController(source_id="gcs-passive-default")
    controller._state._snapshot = VehicleSnapshot(
        onboard_available=True,
        interface_version=INTERFACE_VERSION,
    )
    controller._state._last_status_time = time.monotonic()
    entities = {
        "node": SimpleNamespace(
            get_clock=lambda: SimpleNamespace(now=lambda: FakeNow())
        ),
        "clients": {"lease": client},
        "AcquireControl": SimpleNamespace(Request=SimpleNamespace),
    }
    state = {
        "sequence": 0,
        "future": None,
        "last_attempt": 0.0,
        "last_heartbeat": 0.0,
        "granted_hint": False,
        "grant_hint_deadline": 0.0,
        "release_future": None,
    }

    controller._update_lease(entities, state)
    assert not controller.control_enabled
    assert client.requests == []

    # 即便构造出兼容状态，默认观察态也会在读取传输实体前拒绝排队命令。
    ticket = controller.request_takeoff(0.3)
    controller._process_one_command({}, {})
    rejected = controller.wait_for_result(ticket, timeout=0.1)
    assert rejected is not None
    assert not rejected.success
    assert "尚未启用控制会话" in rejected.message

    controller.enable_control()
    controller._update_lease(entities, state)
    assert controller.control_enabled
    assert len(client.requests) == 1
    assert client.requests[0].release is False


def test_remote_rosout_is_verbatim_and_only_enabled_for_explicit_real_link() -> None:
    """远端日志须保留原文/来源/等级，且默认不进入 GUI 事件总线。"""
    events = EventLog()
    controller = GroundStationRosController(
        source_id="gcs-rosout", event_log=events
    )
    message = SimpleNamespace(
        name="/onboard_control_node",
        level=30,
        msg="机载只读日志：armed=false",
    )

    controller._ingest_remote_rosout(message)
    assert events.snapshot() == ()

    controller.enable_remote_logs()
    controller._ingest_remote_rosout(message)
    received = events.snapshot()[-1]
    assert received.level is LogLevel.WARN
    assert received.source == "remote-rosout:onboard_control_node"
    assert received.message == message.msg


def test_controller_rebuilds_context_when_switching_domains() -> None:
    """同一 GUI 进程须能销毁仿真 context，再创建 domain 0 实机 context。"""
    controller = GroundStationRosController(source_id="gcs-domain-switch")
    controller.start(domain_id=230, discovery_range="LOCALHOST")
    first_thread = controller._thread
    try:
        assert controller.ready
        assert controller.domain_id == 230
        assert controller.discovery_range == "LOCALHOST"

        controller.start(domain_id=229, discovery_range="LOCALHOST")
        assert controller.ready
        assert controller.domain_id == 229
        assert controller._thread is not first_thread
        assert first_thread is not None and not first_thread.is_alive()
    finally:
        controller.stop()

    assert controller.domain_id is None


def test_simulation_hides_and_hardware_restores_explicit_discovery_peers(
    monkeypatch,
) -> None:
    """仿真不得沿用指向真机的静态 peer/discovery server。"""
    monkeypatch.setenv("ROS_DISTRO", "jazzy")
    monkeypatch.setenv("ROS_STATIC_PEERS", "192.168.112.186")
    monkeypatch.setenv("ROS_DISCOVERY_SERVER", "192.168.112.186:11811")
    controller = GroundStationRosController(source_id="gcs-discovery-isolation")

    controller._apply_transport_environment(231, "LOCALHOST")
    assert "ROS_LOCALHOST_ONLY" not in os.environ
    assert os.environ["ROS_AUTOMATIC_DISCOVERY_RANGE"] == "LOCALHOST"
    assert "ROS_STATIC_PEERS" not in os.environ
    assert "ROS_DISCOVERY_SERVER" not in os.environ

    controller._apply_transport_environment(0, "SUBNET")
    assert "ROS_LOCALHOST_ONLY" not in os.environ
    assert os.environ["ROS_AUTOMATIC_DISCOVERY_RANGE"] == "SUBNET"
    assert os.environ["ROS_STATIC_PEERS"] == "192.168.112.186"
    assert os.environ["ROS_DISCOVERY_SERVER"] == "192.168.112.186:11811"


def test_humble_uses_localhost_only_instead_of_jazzy_discovery_range(
    monkeypatch,
) -> None:
    """Humble 仿真/实机切换必须使用其实际支持的发现环境变量。"""
    monkeypatch.setenv("ROS_DISTRO", "humble")
    monkeypatch.setenv("ROS_AUTOMATIC_DISCOVERY_RANGE", "SUBNET")
    controller = GroundStationRosController(source_id="gcs-humble-discovery")

    controller._apply_transport_environment(231, "LOCALHOST")
    assert os.environ["ROS_LOCALHOST_ONLY"] == "1"
    assert "ROS_AUTOMATIC_DISCOVERY_RANGE" not in os.environ

    controller._apply_transport_environment(0, "SUBNET")
    assert os.environ["ROS_LOCALHOST_ONLY"] == "0"
    assert "ROS_AUTOMATIC_DISCOVERY_RANGE" not in os.environ


def test_lease_sequence_remains_monotonic_across_rebuilt_contexts() -> None:
    """同一 source_id 重建仿真/实机 context 后不得把租约序号重置为 1。"""
    controller = GroundStationRosController(source_id="gcs-lease-context-order")

    first_context = [controller._next_lease_sequence() for _ in range(7)]
    rebuilt_context = [controller._next_lease_sequence() for _ in range(3)]

    assert first_context == list(range(1, 8))
    assert rebuilt_context == [8, 9, 10]


def test_endpoint_conflict_blocks_commands_and_lease_transmission() -> None:
    """重复机载状态发布者出现时不得申请租约或发送任何高层命令。"""
    controller = GroundStationRosController(source_id="gcs-endpoint-conflict")
    controller._state._snapshot = VehicleSnapshot(
        onboard_available=True,
        interface_version=INTERFACE_VERSION,
        endpoint_conflict=True,
        control_authority=True,
    )
    controller._state._endpoint_conflict = True
    controller._state._last_status_time = time.monotonic()
    controller.enable_control()
    controller._update_lease(
        {"node": object(), "clients": {"lease": object()}},
        {
            "sequence": 0,
            "future": None,
            "last_attempt": 0.0,
            "last_heartbeat": 0.0,
            "granted_hint": False,
            "grant_hint_deadline": 0.0,
            "release_future": None,
        },
    )
    assert not controller.control_enabled
    ticket = controller.request_takeoff(0.3)

    controller._process_one_command({}, {})
    result = controller.wait_for_result(ticket, timeout=0.1)

    assert result is not None
    assert not result.success
    assert "多个机载状态发布者" in result.message


def test_stale_grant_hint_reacquires_after_onboard_restart() -> None:
    """机载进程重启清空租约后，地面站不得永久只发送无效心跳。"""

    class FakeClient:
        def __init__(self) -> None:
            self.requests: list[object] = []

        @staticmethod
        def service_is_ready() -> bool:
            return True

        def call_async(self, request: object) -> object:
            self.requests.append(request)
            return SimpleNamespace(done=lambda: False)

    class FakeNow:
        @staticmethod
        def to_msg() -> object:
            return object()

    client = FakeClient()
    controller = GroundStationRosController(source_id="gcs-reacquire")
    controller._state._snapshot = VehicleSnapshot(
        onboard_available=True,
        interface_version=INTERFACE_VERSION,
        control_authority=False,
    )
    controller._state._last_status_time = time.monotonic()
    controller.enable_control()
    state = {
        "sequence": 4,
        "future": None,
        "last_attempt": 0.0,
        "last_heartbeat": 0.0,
        "granted_hint": True,
        "grant_hint_deadline": time.monotonic() - 0.1,
        "release_future": None,
    }
    entities = {
        "node": SimpleNamespace(
            get_clock=lambda: SimpleNamespace(now=lambda: FakeNow())
        ),
        "clients": {"lease": client},
        "AcquireControl": SimpleNamespace(Request=SimpleNamespace),
    }

    controller._update_lease(entities, state)

    assert not state["granted_hint"]
    assert len(client.requests) == 1
    assert client.requests[0].release is False
