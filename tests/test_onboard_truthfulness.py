"""隔离验证机载端只在观察到真实结果后报告维护与降落成功。"""

from __future__ import annotations

import math
import os
import subprocess
import time
from pathlib import Path

import pytest


PROJECT_ROOT = Path(__file__).resolve().parents[1]


def test_onboard_confirms_observed_rates_origin_and_land() -> None:
    """ACK 不是终态；频率、原点和降落都必须等待对应遥测证据。"""
    rclpy = pytest.importorskip("rclpy")
    from geographic_msgs.msg import GeoPointStamped
    from geometry_msgs.msg import PoseStamped, TwistStamped
    from guided_interfaces.msg import CommandResult, ControlStatus
    from guided_interfaces.srv import AcquireControl, FlightCommand, SetGpsOrigin
    from mavros_msgs.msg import State
    from mavros_msgs.srv import MessageInterval, SetMode
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import BatteryState, Imu

    executable = (
        PROJECT_ROOT
        / "install"
        / "onboard_control"
        / "lib"
        / "onboard_control"
        / "onboard_control_node"
    )
    if not executable.is_file():
        pytest.skip("onboard_control 尚未构建")

    domain_id = str(170 + os.getpid() % 20)
    process_environment = os.environ.copy()
    process_environment["ROS_DOMAIN_ID"] = domain_id
    context = Context()
    previous_domain = os.environ.get("ROS_DOMAIN_ID")
    os.environ["ROS_DOMAIN_ID"] = domain_id
    rclpy.init(context=context)
    node = rclpy.create_node("truthfulness_probe", context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)

    sensor_qos = QoSProfile(depth=10)
    sensor_qos.reliability = ReliabilityPolicy.BEST_EFFORT
    latched_qos = QoSProfile(depth=1)
    latched_qos.reliability = ReliabilityPolicy.RELIABLE
    latched_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

    states = node.create_publisher(State, "/truth_mavros/state", 10)
    poses = node.create_publisher(
        PoseStamped, "/truth_mavros/local_position/pose", sensor_qos
    )
    velocities = node.create_publisher(
        TwistStamped,
        "/truth_mavros/local_position/velocity_local",
        sensor_qos,
    )
    attitudes = node.create_publisher(Imu, "/truth_mavros/imu/data", sensor_qos)
    highres_imus = node.create_publisher(
        Imu, "/truth_mavros/imu/data_raw", sensor_qos
    )
    batteries = node.create_publisher(
        BatteryState, "/truth_mavros/battery", sensor_qos
    )
    origin_echoes = node.create_publisher(
        GeoPointStamped,
        "/truth_mavros/global_position/gp_origin",
        latched_qos,
    )

    interval_requests: list[tuple[int, float]] = []
    land_requests: list[str] = []
    origin_requests: list[GeoPointStamped] = []
    statuses: list[ControlStatus] = []
    results: list[CommandResult] = []

    def configure_interval(request, response):
        interval_requests.append((request.message_id, request.message_rate))
        response.success = True
        return response

    def set_mode(request, response):
        land_requests.append(request.custom_mode)
        response.mode_sent = True
        return response

    node.create_service(
        MessageInterval,
        "/truth_mavros/set_message_interval",
        configure_interval,
    )
    node.create_service(SetMode, "/truth_mavros/set_mode", set_mode)
    node.create_subscription(
        GeoPointStamped,
        "/truth_mavros/global_position/set_gp_origin",
        origin_requests.append,
        10,
    )
    node.create_subscription(
        ControlStatus,
        "/truth_onboard/status",
        statuses.append,
        sensor_qos,
    )
    node.create_subscription(
        CommandResult,
        "/truth_onboard/command_result",
        results.append,
        50,
    )

    process = subprocess.Popen(
        [
            str(executable),
            "--ros-args",
            "-p",
            "mavros_prefix:=/truth_mavros",
            "-p",
            "interface_prefix:=/truth_onboard",
            "-p",
            "status_frequency_hz:=20.0",
            "-p",
            "fcu_parameter_check_initial_delay_seconds:=60.0",
        ],
        cwd=PROJECT_ROOT,
        env=process_environment,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
    )

    def spin_until(predicate, timeout: float = 5.0) -> bool:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.01)
            if predicate():
                return True
        return False

    def publish_state(armed: bool) -> None:
        message = State()
        message.connected = True
        message.armed = armed
        message.mode = "GUIDED" if armed else "LAND"
        states.publish(message)

    def call(client, request, timeout: float = 3.0):
        future = client.call_async(request)
        assert spin_until(future.done, timeout), "服务调用未在期限内返回"
        return future.result()

    try:
        assert spin_until(lambda: bool(statuses)), "机载状态话题未启动"
        publish_state(True)
        assert spin_until(lambda: any(status.fcu_connected for status in statuses))
        assert spin_until(lambda: len(interval_requests) == 4)
        assert {message_id for message_id, _rate in interval_requests} == {
            1,
            31,
            32,
            105,
        }

        # 全部 MAVLink ACK 到达后仍没有实测高频流，状态不得提前报成功。
        end = time.monotonic() + 0.4
        while time.monotonic() < end:
            publish_state(True)
            executor.spin_once(timeout_sec=0.01)
        assert statuses[-1].message_rates_configured is False

        pitch = math.radians(-6.0)
        yaw = math.radians(35.0)
        pose = PoseStamped()
        pose.pose.orientation.x = -math.sin(pitch / 2.0) * math.sin(yaw / 2.0)
        pose.pose.orientation.y = math.sin(pitch / 2.0) * math.cos(yaw / 2.0)
        pose.pose.orientation.z = math.cos(pitch / 2.0) * math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(pitch / 2.0) * math.cos(yaw / 2.0)
        velocity = TwistStamped()
        imu = Imu()
        battery = BatteryState()
        battery.present = True
        battery.voltage = 15.7
        battery.current = -3.1
        battery.percentage = 0.73

        # 高频发布三路受检流；只有实际频率达标后状态位才允许变为 true。
        end = time.monotonic() + 2.0
        next_state = 0.0
        while time.monotonic() < end:
            poses.publish(pose)
            velocities.publish(velocity)
            attitudes.publish(imu)
            highres_imus.publish(imu)
            batteries.publish(battery)
            if time.monotonic() >= next_state:
                publish_state(True)
                next_state = time.monotonic() + 0.05
            executor.spin_once(timeout_sec=0.001)
            time.sleep(0.004)
        assert spin_until(
            lambda: statuses and statuses[-1].message_rates_configured,
            2.0,
        )
        measured = statuses[-1]
        assert measured.interface_version == "2.1"
        assert measured.autopilot_mode == "GUIDED"
        assert measured.battery_valid
        assert measured.battery_voltage == pytest.approx(15.7, abs=0.01)
        assert measured.battery_current == pytest.approx(-3.1, abs=0.01)
        assert measured.battery_percentage == pytest.approx(0.73, abs=0.01)
        assert measured.pitch == pytest.approx(pitch, abs=0.01)
        assert measured.yaw == pytest.approx(yaw, abs=0.01)

        lease_client = node.create_client(
            AcquireControl, "/truth_onboard/acquire_control"
        )
        origin_client = node.create_client(
            SetGpsOrigin, "/truth_onboard/set_gps_origin"
        )
        land_client = node.create_client(
            FlightCommand, "/truth_onboard/flight_command"
        )
        assert lease_client.wait_for_service(timeout_sec=3.0)
        assert origin_client.wait_for_service(timeout_sec=3.0)
        assert land_client.wait_for_service(timeout_sec=3.0)

        lease = AcquireControl.Request()
        lease.stamp = node.get_clock().now().to_msg()
        lease.source_id = "truth-gcs"
        lease.sequence = 1
        lease.lease_duration_ms = 5000
        assert call(lease_client, lease).granted

        origin = SetGpsOrigin.Request()
        origin.stamp = node.get_clock().now().to_msg()
        origin.source_id = "truth-gcs"
        origin.sequence = 1
        origin.ttl_ms = 3000
        origin.origin.latitude = 31.123456
        origin.origin.longitude = 121.654321
        origin.origin.altitude = 18.5
        assert call(origin_client, origin).accepted
        assert spin_until(lambda: bool(origin_requests))
        assert spin_until(
            lambda: any(
                result.command == "set_gp_origin"
                and result.status == CommandResult.STATUS_RUNNING
                for result in results
            )
        )
        assert not any(
            result.command == "set_gp_origin" and result.final for result in results
        )

        wrong_echo = GeoPointStamped()
        wrong_echo.position.latitude = origin.origin.latitude + 0.01
        wrong_echo.position.longitude = origin.origin.longitude
        wrong_echo.position.altitude = origin.origin.altitude
        origin_echoes.publish(wrong_echo)
        end = time.monotonic() + 0.2
        while time.monotonic() < end:
            executor.spin_once(timeout_sec=0.01)
        assert not any(
            result.command == "set_gp_origin" and result.final for result in results
        )

        right_echo = GeoPointStamped()
        right_echo.position = origin.origin
        origin_echoes.publish(right_echo)
        assert spin_until(
            lambda: any(
                result.command == "set_gp_origin"
                and result.final
                and result.status == CommandResult.STATUS_SUCCEEDED
                for result in results
            )
        )

        land = FlightCommand.Request()
        land.stamp = node.get_clock().now().to_msg()
        land.source_id = "truth-gcs"
        land.sequence = 2
        land.ttl_ms = 3000
        land.command = FlightCommand.Request.COMMAND_LAND
        assert call(land_client, land).accepted
        assert spin_until(lambda: land_requests == ["LAND"])
        assert spin_until(
            lambda: any(
                result.command == "land"
                and result.status == CommandResult.STATUS_RUNNING
                for result in results
            )
        )
        assert not any(result.command == "land" and result.final for result in results)

        publish_state(False)
        assert spin_until(
            lambda: any(
                result.command == "land"
                and result.final
                and result.status == CommandResult.STATUS_SUCCEEDED
                for result in results
            )
        )
    finally:
        process.terminate()
        try:
            process.wait(timeout=5.0)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=2.0)
        executor.remove_node(node)
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown(context=context)
        if previous_domain is None:
            os.environ.pop("ROS_DOMAIN_ID", None)
        else:
            os.environ["ROS_DOMAIN_ID"] = previous_domain
