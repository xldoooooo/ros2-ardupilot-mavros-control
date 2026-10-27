"""航点请求、终态进度与 RViz 预览的 ROS 客户端回归。"""

from __future__ import annotations

import math
import time
from dataclasses import replace
from types import SimpleNamespace

import pytest
from ground_station_core.config import INTERFACE_VERSION
from ground_station_core.models import VehicleSnapshot
from ground_station_core.ros_controller import (
    GroundStationRosController,
    PREVIEW_FRAME_ID,
    VEHICLE_POSE_TOPIC,
    WAYPOINT_MARKERS_TOPIC,
    WAYPOINT_PATH_TOPIC,
)


def test_waypoint_result_keeps_authoritative_terminal_progress() -> None:
    """本地结果队列必须保留机载终态的航点计数。"""
    controller = GroundStationRosController(source_id="pytest-waypoint-result")

    controller._ingest_command_result(
        SimpleNamespace(
            sequence=7,
            command="waypoints",
            message="航点任务完成",
            final=True,
            waypoint_index=3,
            waypoint_count=3,
        ),
        successful=True,
    )

    result = controller.results_after(0)[0]
    assert result.ticket == 7
    assert result.waypoint_index == 3
    assert result.waypoint_count == 3


def test_waypoint_request_preserves_three_independent_gui_choices() -> None:
    """锁定策略、命令生成和跟踪控制必须作为独立字段原子排队。"""
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
        "photo_nos": (),
        "coordinate_frame": "odom",
        "local_coordinates": False,
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
        photo_nos=("客户/A-7",),
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
    assert waypoint.has_photo_no
    assert waypoint.photo_no == "客户/A-7"


@pytest.mark.parametrize("navigation_frame", ["odom", "map"])
def test_waypoint_preview_builds_retained_markers_path_and_live_pose(navigation_frame) -> None:
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
    controller._state._snapshot = VehicleSnapshot(
        navigation_frame=navigation_frame,
        task_to_local=(10.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0),
    )
    expected_frame = "fcu_local" if navigation_frame == "map" else PREVIEW_FRAME_ID
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
    assert path.header.frame_id == expected_frame
    assert path.header.stamp == Time()
    assert all(marker.header.frame_id == expected_frame for marker in marker_array.markers)
    assert all(marker.header.stamp == Time() for marker in marker_array.markers)
    assert all(pose.header.frame_id == expected_frame for pose in path.poses)
    assert len(path.poses) == 2
    assert path.poses[0].pose.position.x == (11.0 if navigation_frame == "map" else 1.0)
    assert math.isclose(path.poses[1].pose.orientation.z, math.sqrt(0.5))
    assert math.isclose(path.poses[1].pose.orientation.w, math.sqrt(0.5))

    status = SimpleNamespace(
        header=SimpleNamespace(stamp=Time(sec=8141, nanosec=123)),
        local_position_valid=True,
        navigation_frame=navigation_frame,
        navigation_ready=True,
        position=SimpleNamespace(x=7.0, y=8.0, z=9.0),
        roll=0.0,
        pitch=0.0,
        yaw=math.pi / 2.0,
    )
    controller._publish_vehicle_preview_pose(entities, status)
    pose = pose_publisher.messages[-1]
    assert pose.header.frame_id == expected_frame
    assert pose.header.stamp == status.header.stamp
    assert (pose.pose.position.x, pose.pose.position.y, pose.pose.position.z) == (
        7.0,
        8.0,
        9.0,
    )
    assert math.isclose(pose.pose.orientation.z, math.sqrt(0.5))
    assert math.isclose(pose.pose.orientation.w, math.sqrt(0.5))

    # 地图 TF 用 Odin 设备秒；静态航点不能按地面 wall-clock 查询造成外推失败。
    if navigation_frame == "map":
        from geometry_msgs.msg import TransformStamped
        from rclpy.time import Time as RosTime
        from tf2_ros import Buffer, ExtrapolationException

        buffer = Buffer()
        transform = TransformStamped(child_frame_id="map")
        transform.header.frame_id = "odom"
        transform.transform.translation.x = 10.0
        transform.transform.rotation.w = 1.0
        for second in (8140, 8141):
            transform.header.stamp.sec = second
            buffer.set_transform(transform, "odin")
        transform = TransformStamped(child_frame_id="fcu_local")
        transform.header.frame_id = "odom"
        transform.transform.rotation.w = 1.0
        buffer.set_transform_static(transform, "onboard_control")
        with pytest.raises(ExtrapolationException):
            buffer.lookup_transform("map", expected_frame, RosTime(seconds=1793093372))
        display_transform = buffer.lookup_transform(
            "map", expected_frame, RosTime.from_msg(path.header.stamp)
        )
        assert display_transform.transform.translation.x + path.poses[0].pose.position.x == 1.0

        # 设备重启归零后旧 Buffer 拒收新地图；重开显示进程的全新缓存才能采用该变换。
        rebooted_map = TransformStamped(child_frame_id="map")
        rebooted_map.header.frame_id = "odom"
        rebooted_map.header.stamp.sec = 1
        rebooted_map.transform.translation.x = 20.0
        rebooted_map.transform.rotation.w = 1.0
        buffer.set_transform(rebooted_map, "odin")
        assert buffer.lookup_transform("map", expected_frame, RosTime()).transform.translation.x == -10.0
        fresh_buffer = Buffer()
        fresh_buffer.set_transform(rebooted_map, "odin")
        fresh_buffer.set_transform_static(transform, "onboard_control")
        assert fresh_buffer.lookup_transform("map", expected_frame, RosTime()).transform.translation.x == -20.0

    # TF 持续微调会更新坐标，但不把相同航点按 10 Hz 写入事件日志。
    event_count = len(controller._events.snapshot())
    assert controller.publish_waypoint_preview(waypoints)
    controller._process_one_waypoint_preview(entities)
    assert len(controller._events.snapshot()) == event_count

    if navigation_frame == "map":
        # 重启后尚未定位时，清屏必须在固定系完成；恢复定位仍按 FCU 坐标显示。
        controller._state._snapshot = replace(controller._state._snapshot, navigation_ready=False)
        assert controller.publish_waypoint_preview(waypoints)
        controller._process_one_waypoint_preview(entities)
        assert path_publisher.messages[-1].header.frame_id == PREVIEW_FRAME_ID
        assert not path_publisher.messages[-1].poses
        markers = marker_publisher.messages[-1].markers
        assert len(markers) == 1 and markers[0].action == Marker.DELETEALL
        assert markers[0].header.frame_id == PREVIEW_FRAME_ID
        controller._state._snapshot = replace(controller._state._snapshot, navigation_ready=True)
        assert controller.publish_waypoint_preview(waypoints)
        controller._process_one_waypoint_preview(entities)
        assert path_publisher.messages[-1].header.frame_id == expected_frame
        assert len(path_publisher.messages[-1].poses) == len(waypoints)

    # 空快照显式删除旧 Marker 并覆盖为空 Path，不会留下上一次较长列表。
    assert controller.publish_waypoint_preview(())
    controller._process_one_waypoint_preview(entities)
    assert len(marker_publisher.messages[-1].markers) == 1
    assert marker_publisher.messages[-1].markers[0].action == Marker.DELETEALL
    assert path_publisher.messages[-1].poses == []
