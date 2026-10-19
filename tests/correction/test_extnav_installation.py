"""Verify mounted Odin attitude and rotating lever against a known FCU trajectory."""

import importlib.util
import math
from pathlib import Path

import numpy as np
import pytest
import rclpy
from geometry_msgs.msg import Point, Quaternion, Vector3
from rclpy.context import Context
from rclpy.parameter import Parameter


@pytest.mark.parametrize("mount_pitch", (0.0, -math.radians(22.0)))
def test_installation_and_lever_recover_fcu_pose_and_velocity(mount_pitch, monkeypatch):
    """Recover a rolling/yawing FCU from the sensor pose without rotating its world."""
    path = Path(__file__).resolve().parents[2] / "correction_service/extnav_patch/extnav_to_vision_pose.py"
    spec = importlib.util.spec_from_file_location("extnav_installation_test", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    monkeypatch.setenv("ROS_AUTOMATIC_DISCOVERY_RANGE", "LOCALHOST")
    monkeypatch.delenv("ROS_LOCALHOST_ONLY", raising=False)
    context = Context()
    rclpy.init(args=[], context=context, domain_id=229)
    lever = np.array((0.06, -0.03, 0.05))
    node = module.OdometryBridge(context=context, parameter_overrides=[
        Parameter("pitch_cam", value=mount_pitch),
        *(Parameter(name, value=float(value)) for name, value in zip(
            ("odin_x", "odin_y", "odin_z"), lever)),
    ])
    try:
        for attitude in ((0.0, 0.0, 0.0), (0.2, -0.1, 1.4)):
            q_fcu = module.rpy_to_quaternion(*attitude)
            q_sensor = module.multiply_quaternions(q_fcu, node.q_FI)
            rotation = module.quaternion_to_rotation_matrix(q_fcu)
            position = np.array((2.0, -1.0, 0.8))
            velocity = np.array((0.3, -0.2, 0.1))
            omega_fcu = np.array((0.2, -0.3, 0.7))
            sensor_position = position + rotation @ lever
            sensor_velocity = velocity + rotation @ np.cross(omega_fcu, lever)
            sensor_omega = node.R_FI.T @ omega_fcu
            actual_position, actual_velocity, actual_quaternion = node._apply_transform(
                Point(**dict(zip(("x", "y", "z"), sensor_position))),
                Quaternion(**dict(zip(("x", "y", "z", "w"), q_sensor))),
                Vector3(**dict(zip(("x", "y", "z"), sensor_velocity))),
                Vector3(**dict(zip(("x", "y", "z"), sensor_omega))),
            )
            assert np.allclose(actual_position, position + lever, atol=1e-12)
            assert np.allclose(actual_velocity, velocity, atol=1e-12)
            assert np.allclose(module.quaternion_to_rotation_matrix(actual_quaternion),
                               rotation, atol=1e-12)
    finally:
        node.destroy_node()
        context.shutdown()
