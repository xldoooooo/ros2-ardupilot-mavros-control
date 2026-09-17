#!/usr/bin/env python3
"""Onboard reboot client: acquire a lease, request the shared command and print results.

Run through scripts/onboard/reboot_fcu.sh; never calls MAVROS, arms, takes off or restarts a process.
"""
import time
import uuid

import rclpy
from rclpy.qos import qos_profile_sensor_data
from guided_interfaces.msg import CommandResult, ControlHeartbeat, ControlStatus
from guided_interfaces.srv import AcquireControl, FlightCommand


def main() -> int:
    """Bound discovery and completion; release our lease even after a rejected command."""
    rclpy.init()
    node = rclpy.create_node("onboard_reboot_client")
    source = "onboard-reboot-" + uuid.uuid4().hex[:12]
    sequence = 0
    status = None
    final = None
    lease_owned = False
    prefix = "/onboard_control"

    def envelope(request):
        nonlocal sequence
        sequence += 1
        request.stamp = node.get_clock().now().to_msg()
        request.source_id = source
        request.sequence = sequence
        return request

    def on_status(message):
        nonlocal status
        status = message

    def on_result(message):
        nonlocal final
        if message.source_id == source and message.command == "reboot_fcu":
            print(message.message, flush=True)
            if message.final:
                final = message.status == CommandResult.STATUS_SUCCEEDED

    def wait_future(future, seconds=5.0):
        rclpy.spin_until_future_complete(node, future, timeout_sec=seconds)
        if not future.done():
            raise RuntimeError("机载服务响应超时；不自动重发重启")
        return future.result()

    def heartbeat():
        nonlocal sequence
        if not lease_owned:
            return
        sequence += 1
        message = ControlHeartbeat()
        message.header.stamp = node.get_clock().now().to_msg()
        message.source_id = source
        message.sequence = sequence
        message.lease_duration_ms = 3000
        publisher.publish(message)

    node.create_subscription(ControlStatus, prefix + "/status", on_status, qos_profile_sensor_data)
    node.create_subscription(CommandResult, prefix + "/command_result", on_result, 50)
    publisher = node.create_publisher(ControlHeartbeat, prefix + "/heartbeat", 10)
    lease = node.create_client(AcquireControl, prefix + "/acquire_control")
    flight = node.create_client(FlightCommand, prefix + "/flight_command")
    node.create_timer(0.2, heartbeat)
    try:
        deadline = time.monotonic() + 8.0
        while status is None and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if status is None or not lease.wait_for_service(timeout_sec=2) or not flight.wait_for_service(timeout_sec=2):
            raise RuntimeError("onboard_control 服务异常或不可用")
        if node.count_publishers(prefix + "/status") != 1:
            raise RuntimeError("机载服务端点不唯一")
        if status.interface_version != "3.3":
            raise RuntimeError("机载接口版本不匹配，要求 3.3")
        if not status.fcu_connected or status.armed or not status.on_ground or status.reboot_in_progress or status.control_mode != ControlStatus.MODE_IDLE:
            raise RuntimeError("飞控不满足在线、未解锁、已落地、待机条件")
        request = envelope(AcquireControl.Request())
        request.lease_duration_ms = 3000
        reply = wait_future(lease.call_async(request))
        if not reply.granted:
            raise RuntimeError(reply.message)
        lease_owned = True
        request = envelope(FlightCommand.Request())
        request.ttl_ms = 3000
        request.command = FlightCommand.Request.COMMAND_REBOOT_FCU
        reply = wait_future(flight.call_async(request))
        if not reply.accepted:
            raise RuntimeError(reply.message)
        print(reply.message, flush=True)
        deadline = time.monotonic() + 100.0
        while final is None and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if final is None:
            raise RuntimeError("未收到重启终态，请检查机载日志；不会重发重启")
        return 0 if final else 1
    except Exception as error:
        print(f"重启失败：{error}", flush=True)
        return 1
    finally:
        if lease_owned:
            request = envelope(AcquireControl.Request())
            request.release = True
            request.lease_duration_ms = 3000
            lease_owned = False
            try:
                wait_future(lease.call_async(request), 2.0)
            except Exception:
                pass  # Bounded lease expires even if the server disappeared.
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
