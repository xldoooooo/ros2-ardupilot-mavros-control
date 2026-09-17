"""Exercise the real onboard node against isolated MAVROS doubles; never use domain 0."""
import os
import subprocess
import time
from pathlib import Path

import pytest


def test_reboot_requires_ground_and_boot_evidence_then_recovers(tmp_path):
    """Reject unsafe/stale states; ACK alone is insufficient; external reboot also recovers."""
    rclpy = pytest.importorskip("rclpy")
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
    from mavros_msgs.msg import State, ExtendedState, TimesyncStatus, ParamEvent
    from mavros_msgs.srv import CommandLong, MessageInterval, ParamPull
    from geometry_msgs.msg import PoseStamped, TwistStamped
    from geographic_msgs.msg import GeoPointStamped
    from guided_interfaces.msg import ControlStatus, CommandResult, ControlHeartbeat
    from guided_interfaces.srv import AcquireControl, FlightCommand

    root = Path(__file__).resolve().parents[2]
    executable = root / "install/onboard_control/lib/onboard_control/onboard_control_node"
    if not executable.exists():
        pytest.skip("build onboard_control first")
    domain = 200 + os.getpid() % 20
    context = Context()
    rclpy.init(context=context, domain_id=domain)
    node = rclpy.create_node("reboot_test", context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    env = dict(os.environ, ROS_DOMAIN_ID=str(domain), ROS_AUTOMATIC_DISCOVERY_RANGE="LOCALHOST")
    env.pop("ROS_STATIC_PEERS", None)
    state = State(connected=True, armed=False, mode="STABILIZE")
    ground = ExtendedState(landed_state=1)
    boot = 100_000_000_000
    publish_ground = True
    publish_params = True
    echo_origin = True
    latest = []
    results = []
    commands = []
    intervals = []
    origin_writes = []
    lease_active = False
    sequence = 0
    pubs = {}
    for typ, topic in [(State, "state"), (ExtendedState, "extended_state"),
                       (TimesyncStatus, "timesync_status"), (PoseStamped, "local_position/pose"),
                       (TwistStamped, "local_position/velocity_local"), (ParamEvent, "param/event"),
                       (GeoPointStamped, "global_position/gp_origin")]:
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL) if topic.endswith("gp_origin") else 10
        pubs[topic] = node.create_publisher(typ, "/reboot_test_mavros/" + topic, qos)
    heartbeats = node.create_publisher(ControlHeartbeat, "/reboot_test_onboard/heartbeat", 10)
    node.create_subscription(ControlStatus, "/reboot_test_onboard/status", latest.append, qos_profile_sensor_data)
    node.create_subscription(CommandResult, "/reboot_test_onboard/command_result", results.append, 50)
    def command(request, response):
        if request.command == 246:
            commands.append(request)
        response.success = True
        return response
    def interval(request, response):
        intervals.append(request.message_id)
        response.success = True
        return response
    def pull(request, response):
        response.success = True
        return response
    def origin(request):
        origin_writes.append(request)
        if echo_origin:
            pubs["global_position/gp_origin"].publish(request)
    node.create_service(CommandLong, "/reboot_test_mavros/cmd/command", command)
    node.create_service(MessageInterval, "/reboot_test_mavros/set_message_interval", interval)
    node.create_service(ParamPull, "/reboot_test_mavros/param/pull", pull)
    node.create_subscription(GeoPointStamped, "/reboot_test_mavros/global_position/set_gp_origin", origin, 10)
    lease = node.create_client(AcquireControl, "/reboot_test_onboard/acquire_control")
    flight = node.create_client(FlightCommand, "/reboot_test_onboard/flight_command")
    def tick():
        nonlocal boot, sequence
        boot += 50_000_000
        pubs["state"].publish(state)
        if publish_ground:
            pubs["extended_state"].publish(ground)
        pubs["timesync_status"].publish(TimesyncStatus(remote_timestamp_ns=boot))
        pose = PoseStamped()
        pose.pose.orientation.w = 1.0
        pubs["local_position/pose"].publish(pose)
        pubs["local_position/velocity_local"].publish(TwistStamped())
        if publish_params:
            guid = ParamEvent(param_id="GUID_OPTIONS")
            guid.value.type = 2
            guid.value.integer_value = 8
            hover = ParamEvent(param_id="MOT_THST_HOVER")
            hover.value.type = 3
            hover.value.double_value = 0.22
            pubs["param/event"].publish(guid)
            pubs["param/event"].publish(hover)
        if lease_active:
            sequence += 1
            hb = ControlHeartbeat(source_id="reboot-test", sequence=sequence, lease_duration_ms=3000)
            hb.header.stamp = node.get_clock().now().to_msg()
            heartbeats.publish(hb)
    node.create_timer(0.05, tick)
    def spin(seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.01)
    def until(predicate, seconds=8):
        deadline = time.monotonic() + seconds
        while not predicate() and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.01)
        assert predicate(), [r.message for r in results]
    def request(typ):
        nonlocal sequence
        sequence += 1
        req = typ.Request(source_id="reboot-test", sequence=sequence)
        req.stamp = node.get_clock().now().to_msg()
        return req
    def reboot():
        req = request(FlightCommand)
        req.ttl_ms = 3000
        req.command = FlightCommand.Request.COMMAND_REBOOT_FCU
        future = flight.call_async(req)
        until(future.done)
        return future.result()
    with (tmp_path / "onboard.log").open("w") as output:
        process = subprocess.Popen([str(executable), "--ros-args", "-p", "mavros_prefix:=/reboot_test_mavros",
            "-p", "interface_prefix:=/reboot_test_onboard"], env=env, stdout=output, stderr=output)
        try:
            until(lambda: latest and lease.service_is_ready() and flight.service_is_ready(), 20)
            req = request(AcquireControl)
            req.lease_duration_ms = 3000
            future = lease.call_async(req)
            until(future.done)
            assert future.result().granted
            lease_active = True
            saved = GeoPointStamped()
            saved.position.latitude = 30.0
            saved.position.longitude = 120.0
            saved.position.altitude = 40.0
            pubs["global_position/gp_origin"].publish(saved)
            spin(0.3)
            for connected, armed, landed in [(False, False, 1), (True, True, 1), (True, False, 2)]:
                state.connected, state.armed, ground.landed_state = connected, armed, landed
                spin(0.3)
                assert not reboot().accepted
            state.connected, state.armed, ground.landed_state = True, False, 1
            spin(0.3)
            publish_ground = False
            spin(3.2)
            assert not reboot().accepted
            assert not commands
            publish_ground = True
            until(lambda: latest[-1].on_ground)
            assert reboot().accepted
            until(lambda: len(commands) == 1)
            assert commands[0].command == 246 and commands[0].param1 == 1
            assert commands[0].param2 == 0 and commands[0].param3 == 0
            spin(0.4)
            assert not any(r.final and r.command == "reboot_fcu" for r in results)
            assert not reboot().accepted  # No duplicate reboot during an active transaction.
            req = request(FlightCommand)
            req.ttl_ms = 3000
            req.command = FlightCommand.Request.COMMAND_TAKEOFF
            req.value = 1.0
            future = flight.call_async(req)
            until(future.done)
            assert not future.result().accepted
            echo_origin = False
            publish_params = False
            boot = 1_000_000_000
            until(lambda: origin_writes)
            spin(2.3)
            assert latest[-1].reboot_in_progress
            assert not any(r.final and r.command == "reboot_fcu" for r in results)
            echo_origin = True
            publish_params = True
            until(lambda: any(r.final and r.status == 1 and r.command == "reboot_fcu" for r in results), 10)
            until(lambda: not latest[-1].reboot_in_progress)
            assert not latest[-1].armed and latest[-1].on_ground
            assert origin_writes[-1].position == saved.position
            assert intervals.count(245) >= 2
            # External reboot has no requested command; the same passive recovery applies.
            spin(1.2)
            boot = 500_000_000
            until(lambda: latest[-1].reboot_in_progress)
            until(lambda: not latest[-1].reboot_in_progress, 10)
            assert len(commands) == 1
        finally:
            process.terminate()
            process.wait(timeout=10)
            executor.shutdown()
            node.destroy_node()
            rclpy.shutdown(context=context)
