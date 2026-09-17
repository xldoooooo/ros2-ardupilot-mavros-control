"""RViz 航点图层、只读位姿桥和运行依赖的静态回归。"""

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
