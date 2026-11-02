"""Verify that standard FCU odometry receives only origin and world-frame conversion."""

import importlib.util
import math
from pathlib import Path

import numpy as np
import pytest
import rclpy
from nav_msgs.msg import Odometry
from rclpy.context import Context
from rclpy.parameter import Parameter


@pytest.mark.parametrize("mount_pitch", (0.0, -math.radians(30.0)))
def test_fcu_input_preserves_pose_and_rotates_body_twist(mount_pitch, monkeypatch):
    """Installation metadata must not repeat physical driver-side compensation."""
    path = Path(__file__).resolve().parents[2] / "correction_service/extnav_patch/extnav_to_vision_pose.py"
    spec = importlib.util.spec_from_file_location("extnav_installation_test", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    monkeypatch.setenv("ROS_AUTOMATIC_DISCOVERY_RANGE", "LOCALHOST")
    monkeypatch.delenv("ROS_LOCALHOST_ONLY", raising=False)
    context = Context()
    rclpy.init(args=[], context=context, domain_id=229)
    origin = np.array((0.10, -0.029, 0.06))
    node = module.OdometryBridge(context=context, parameter_overrides=[
        Parameter("pitch_cam", value=mount_pitch),
        *(Parameter(name, value=float(value)) for name, value in zip(
            ("odin_x", "odin_y", "odin_z"), origin)),
    ])
    try:
        raw = Odometry()
        raw.header.frame_id = "odom"
        raw.child_frame_id = "fcu"
        raw.pose.pose.position.x, raw.pose.pose.position.y, raw.pose.pose.position.z = (2., -1., .8)
        body_velocity = np.array((.3, -.2, .1))
        body_angular = np.array((.2, -.3, .7))
        raw.twist.twist.linear.x, raw.twist.twist.linear.y, raw.twist.twist.linear.z = body_velocity
        raw.twist.twist.angular.x, raw.twist.twist.angular.y, raw.twist.twist.angular.z = body_angular
        raw.twist.covariance = np.arange(36, dtype=float).tolist()
        for attitude in ((0., 0., 0.), (.2, -.1, 1.4)):
            quaternion = module.rpy_to_quaternion(*attitude)
            (raw.pose.pose.orientation.x, raw.pose.pose.orientation.y,
             raw.pose.pose.orientation.z, raw.pose.pose.orientation.w) = quaternion
            rotation = module.quaternion_to_rotation_matrix(quaternion)
            for correction in (None, (1.2, -2.3, .6)):
                components = module._planar_components(raw, *correction) if correction else None
                output = node._transform_sample(raw, components, correction_valid=correction is not None,
                                                correction_revision=1, correction_session="synthetic")
                expected_rotation = rotation
                expected_position = np.array((2., -1., .8)) + origin
                if correction:
                    c, sine = math.cos(correction[2]), math.sin(correction[2])
                    world_rotation = np.array(((c, -sine, 0.), (sine, c, 0.), (0., 0., 1.)))
                    expected_rotation = world_rotation @ rotation
                    expected_position[:2] = (world_rotation @ np.array((2., -1., .8)))[:2] + correction[:2]
                    corrected = module.apply_planar_correction(raw, *correction)
                    assert corrected.twist == raw.twist
                np.testing.assert_allclose(output.position, expected_position, atol=1e-12)
                np.testing.assert_allclose(output.velocity, expected_rotation @ body_velocity, atol=1e-12)
                np.testing.assert_allclose(output.angular, expected_rotation @ body_angular, atol=1e-12)
                np.testing.assert_allclose(module.quaternion_to_rotation_matrix(output.quaternion),
                                           expected_rotation, atol=1e-12)
    finally:
        node.destroy_node()
        context.shutdown()
