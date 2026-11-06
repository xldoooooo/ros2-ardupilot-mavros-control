"""Verify that legacy automatic-pitch settings cannot repeat driver compensation."""

import math

import pytest
from tests.support.extnav import ProcessPair, native_executable


# Both candidates run as isolated ROS child processes.
pytestmark = pytest.mark.integration


def test_legacy_auto_installation_stays_disabled(monkeypatch):
    """Both implementations keep configured metadata and do not subscribe to FCU IMU."""
    executable = native_executable()
    from nav_msgs.msg import Odometry

    pair = ProcessPair(executable, (0., -math.pi / 6, 0.), monkeypatch,
                        extra_parameters={"auto_installation": True})
    try:
        pair.ready()
        raw = Odometry()
        raw.header.frame_id = "odom"
        raw.child_frame_id = "fcu"
        raw.header.stamp.sec = 100
        raw.pose.pose.orientation.w = 1.
        pair.publish(raw)
        pair.compare_state(installation_pitch_rad=-math.pi / 6)
        assert pair.call().accepted
        for endpoint in pair.endpoints:
            subscriptions = pair.node.get_subscriptions_info_by_topic("/mavros/imu/data")
            assert all(item.node_name != endpoint["name"] for item in subscriptions)
    finally:
        pair.close()
