"""RViz 航点图层、只读位姿桥、时间域与运行依赖回归。"""

import yaml

from ground_station_core.config import PROJECT_ROOT
from ground_station_core.ros_controller import (
    VEHICLE_POSE_TOPIC,
    WAYPOINT_MARKERS_TOPIC,
    WAYPOINT_PATH_TOPIC,
)


def test_rviz_config_uses_matching_retained_marker_and_path_topics() -> None:
    """MarkerArray/Path 类型与话题必须匹配，且模型使用隔离 description/TF。"""
    config = (
        PROJECT_ROOT / "src" / "guided_sim" / "rviz" / "quadcopter.rviz"
    ).read_text(encoding="utf-8")

    assert "Class: rviz_default_plugins/MarkerArray" in config
    assert f"Value: {WAYPOINT_MARKERS_TOPIC}" in config
    assert "Class: rviz_default_plugins/Path" in config
    assert f"Value: {WAYPOINT_PATH_TOPIC}" in config
    assert config.count("Durability Policy: Transient Local") >= 3
    assert "Value: /ground_station_preview/robot_description" in config
    # RViz 自行插入斜杠；尾部再带 '/' 会错误查找 preview//base_link。
    assert "TF Prefix: ground_station_preview" in config
    assert "TF Prefix: ground_station_preview/" not in config
    assert "/mavros/local_position/pose" not in config
    assert "Update Interval: 0.10000000149011612" in config
    assert "Frame Rate: 15" in config
    tf_display = config.split("Class: rviz_default_plugins/TF", 1)[1].split(
        "Class: rviz_default_plugins/MarkerArray", 1
    )[0]
    assert "Enabled: false" in tf_display
    assert "Value: false" in tf_display


def test_pose_bridge_selects_local_sim_or_aggregated_hardware_pose() -> None:
    """仿真立即消费本地域 MAVROS；实机启动器改用地面聚合位姿。"""
    pose_bridge = (
        PROJECT_ROOT / "src" / "guided_sim" / "scripts" / "pose_to_tf.py"
    ).read_text(encoding="utf-8")
    launch = (
        PROJECT_ROOT / "src" / "guided_sim" / "launch" / "visualize.launch.py"
    ).read_text(encoding="utf-8")
    package = (
        PROJECT_ROOT / "src" / "guided_sim" / "package.xml"
    ).read_text(encoding="utf-8")
    environment = (PROJECT_ROOT / "ground_station_core" / "environment.py").read_text(
        encoding="utf-8"
    )

    assert "/mavros/local_position/pose" in pose_bridge
    assert "depth=1" in pose_bridge
    assert "DeclareLaunchArgument" in launch
    assert "default_value='/mavros/local_position/pose'" in launch
    assert VEHICLE_POSE_TOPIC in environment
    assert "pose_topic:={pose_topic}" in environment
    assert '"nice"' in environment
    assert "ground_station_preview/base_link" in launch
    assert "ground_station_preview_robot_state_publisher" in launch
    assert "ground_station_waypoint_preview_rviz" in launch
    assert "<exec_depend>nav_msgs</exec_depend>" in package
    assert "<exec_depend>visualization_msgs</exec_depend>" in package
    assert "<exec_depend>tf2_geometry_msgs</exec_depend>" in package


def test_pose_bridge_converts_fcu_pose_to_map_without_changing_source() -> None:
    """MAP 模型坐标与姿态转入显示系；原始 FCU 位姿和源时刻保持。"""
    import importlib.util
    import math
    from types import SimpleNamespace

    import pytest
    from geometry_msgs.msg import PoseStamped, TransformStamped
    from tf2_ros import Buffer

    spec = importlib.util.spec_from_file_location(
        "pose_to_tf", PROJECT_ROOT / "src/guided_sim/scripts/pose_to_tf.py"
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    output = []
    warnings = []
    bridge = SimpleNamespace(
        parent_frame="map", child_frame="ground_station_preview/base_link",
        tf_broadcaster=SimpleNamespace(sendTransform=output.append),
        tf_buffer=Buffer(),
        get_logger=lambda: SimpleNamespace(warning=lambda message, **kwargs: warnings.append(message)),
    )
    missing = PoseStamped()
    missing.header.frame_id = "fcu_local"
    module.PoseToTF.pose_callback(bridge, missing)
    assert not output
    assert warnings
    transform = TransformStamped(child_frame_id="fcu_local")
    transform.header.frame_id = "map"
    transform.transform.translation.x = -10.0
    transform.transform.rotation.z = math.sqrt(.5)
    transform.transform.rotation.w = math.sqrt(.5)
    bridge.tf_buffer.set_transform_static(transform, "test")
    for frame in ("map", "fcu_local"):
        pose = PoseStamped()
        pose.header.frame_id = frame
        pose.header.stamp.sec = 123
        pose.pose.position.x = 4.0
        pose.pose.orientation.w = 1.0
        module.PoseToTF.pose_callback(bridge, pose)
        transform = output[-1]
        assert transform.header.frame_id == "map"
        assert transform.header.stamp == pose.header.stamp
        assert pose.header.stamp.sec == 123
        assert transform.child_frame_id == bridge.child_frame
        assert pose.pose.position.x == 4.0
        if frame == "fcu_local":
            assert transform.transform.translation.x == pytest.approx(-10.0)
            assert transform.transform.translation.y == pytest.approx(4.0)
            assert transform.transform.rotation.z == pytest.approx(math.sqrt(.5))
        else:
            assert transform.transform.translation.x == 4.0
            assert transform.transform.rotation == pose.pose.orientation


def test_pose_bridge_mode_switches_keep_queryable_model_chain_through_real_dds() -> None:
    """设备秒地图与墙钟位姿经真实 DDS 连续切换两轮，模型整链始终可查询。"""
    import importlib.util
    import time

    import pytest
    import rclpy
    from geometry_msgs.msg import PoseStamped, TransformStamped
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.parameter import Parameter
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from rclpy.time import Time
    from tf2_msgs.msg import TFMessage
    from tf2_ros import Buffer, StaticTransformBroadcaster, TransformException, TransformListener

    spec = importlib.util.spec_from_file_location(
        "pose_to_tf_dds", PROJECT_ROOT / "src/guided_sim/scripts/pose_to_tf.py"
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    context = Context()
    rclpy.init(context=context, domain_id=226)
    bridge = module.PoseToTF(
        context=context,
        parameter_overrides=[Parameter("pose_topic", value=VEHICLE_POSE_TOPIC)],
    )
    probe = rclpy.create_node("map_preview_time_probe", context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(bridge)
    executor.add_node(probe)
    buffer = Buffer(node=probe)
    listener = TransformListener(buffer, probe)
    qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
    map_publisher = probe.create_publisher(TFMessage, "/tf", QoSProfile(depth=10))
    pose_publisher = probe.create_publisher(PoseStamped, VEHICLE_POSE_TOPIC, qos)
    static_broadcaster = StaticTransformBroadcaster(probe)

    map_transform = TransformStamped(child_frame_id="map")
    map_transform.header.frame_id = "odom"
    map_transform.header.stamp.sec = 8141
    map_transform.transform.translation.x = 10.0
    map_transform.transform.rotation.w = 1.0
    local_transform = TransformStamped(child_frame_id="fcu_local")
    local_transform.header.frame_id = "odom"
    local_transform.transform.translation.x = -.1
    local_transform.transform.rotation.w = 1.0
    joint_transform = TransformStamped(child_frame_id="ground_station_preview/a1")
    joint_transform.header.frame_id = "ground_station_preview/base_link"
    joint_transform.transform.translation.x = .2
    joint_transform.transform.rotation.w = 1.0
    static_broadcaster.sendTransform([local_transform, joint_transform])
    pose = PoseStamped()
    pose.pose.position.x = 4.0
    pose.pose.orientation.w = 1.0
    try:
        for index, frame in enumerate(("map", "fcu_local", "map", "fcu_local")):
            pose.header.frame_id = frame
            pose.header.stamp.sec = 1793093372 + index
            deadline = time.monotonic() + 5.0
            while time.monotonic() < deadline:
                map_publisher.publish(TFMessage(transforms=[map_transform]))
                pose_publisher.publish(pose)
                # TF 有两侧 listener 和模型回发；先处理这一批回调，避免测试生产者淹没 executor。
                for _ in range(8):
                    executor.spin_once(timeout_sec=.01)
                try:
                    model = buffer.lookup_transform("map", joint_transform.child_frame_id, Time())
                except TransformException:
                    continue
                if model.header.stamp != pose.header.stamp:
                    continue
                assert model.transform.translation.x == pytest.approx(-5.9 if frame == "fcu_local" else 4.2)
                assert pose.pose.position.x == 4.0
                break
            else:
                pytest.fail(f"第 {index + 1} 步切换到 {frame} 后未建立模型 TF 链：{buffer.all_frames_as_yaml()}")
    finally:
        listener.unregister()
        executor.remove_node(bridge)
        executor.remove_node(probe)
        bridge.destroy_node()
        probe.destroy_node()
        executor.shutdown()
        context.shutdown()


def test_avoidance_rviz_layers_share_map_and_limit_optional_cloud_subscription() -> None:
    """规划轨迹默认可见；点云默认关闭且订阅 QoS 不积压可靠重传。"""
    config = yaml.safe_load((
        PROJECT_ROOT / "src" / "guided_sim" / "rviz" / "quadcopter.rviz"
    ).read_text(encoding="utf-8"))["Visualization Manager"]
    assert config["Global Options"]["Fixed Frame"] == "map"
    displays = {display["Name"]: display for display in config["Displays"]}
    trajectory = displays["Avoidance Trajectory"]
    assert trajectory["Class"] == "rviz_default_plugins/Path"
    assert trajectory["Enabled"]
    assert trajectory["Topic"]["Value"] == "/ground_station/avoidance_path"
    cloud = displays["Obstacle Preview"]
    assert cloud["Class"] == "rviz_default_plugins/PointCloud2"
    assert not cloud["Enabled"]
    assert cloud["Topic"]["Value"] == "/onboard_control/obstacle_preview"
    assert cloud["Topic"]["Reliability Policy"] == "Best Effort"
    assert cloud["Topic"]["Depth"] == 1
