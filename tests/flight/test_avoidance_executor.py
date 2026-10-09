"""Real onboard executor regression against localhost DDS doubles, without a flight simulator."""
import os
import subprocess
import time
from pathlib import Path

import pytest


@pytest.fixture
def avoidance_node(tmp_path, monkeypatch):
    """Own only this node/process; fictitious MAVROS names cannot address aircraft topics."""
    monkeypatch.setenv("ROS_AUTOMATIC_DISCOVERY_RANGE", "LOCALHOST")
    monkeypatch.delenv("ROS_STATIC_PEERS", raising=False)
    rclpy = pytest.importorskip("rclpy")
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import qos_profile_sensor_data
    from geometry_msgs.msg import PoseStamped, TwistStamped
    from mavros_msgs.msg import State, ExtendedState, ParamEvent
    from mavros_msgs.srv import MessageInterval, ParamPull
    from guided_interfaces.msg import (
        AvoidanceRequest, AvoidanceResult, ControlHeartbeat, ControlStatus,
        CommandResult, MotionIntent, Waypoint,
    )
    from guided_interfaces.srv import AcquireControl, ExecuteWaypoints, FlightCommand

    root = Path(__file__).resolve().parents[2]
    executable = root / "install/onboard_control/lib/onboard_control/onboard_control_node"
    if not executable.exists():
        pytest.skip("build onboard_control first")
    domain = 150 + os.getpid() % 10
    context = Context()
    rclpy.init(context=context, domain_id=domain)
    node = rclpy.create_node("avoidance_executor_double", context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    mavros, onboard = "/avoidance_executor_double_mavros", "/avoidance_executor_double_onboard"

    class Harness:
        """Publish measured state and service responses; never emulate controller transitions."""
        state = State(connected=True, armed=True, mode="GUIDED")
        sequence = 0
        leased = False
        policy = "silent"
        height = 1.

        def __init__(self):
            self.status, self.results, self.requests = [], [], []
            self.node, self.executor, self.mavros = node, executor, mavros
            self.pubs = {name: node.create_publisher(kind, mavros + "/" + name, 10)
                         for kind, name in [(State, "state"), (ExtendedState, "extended_state"),
                                            (PoseStamped, "local_position/pose"),
                                            (TwistStamped, "local_position/velocity_local"),
                                            (ParamEvent, "param/event")]}
            self.heartbeat = node.create_publisher(ControlHeartbeat, onboard + "/heartbeat", 10)
            self.reply = node.create_publisher(AvoidanceResult, onboard + "/avoidance_result", 10)
            self.motion = node.create_publisher(MotionIntent, onboard + "/motion_intent", 10)
            node.create_subscription(ControlStatus, onboard + "/status", self.status.append,
                                     qos_profile_sensor_data)
            node.create_subscription(CommandResult, onboard + "/command_result", self.results.append, 50)
            node.create_subscription(AvoidanceRequest, onboard + "/avoidance_request", self.plan, 10)
            self.lease = node.create_client(AcquireControl, onboard + "/acquire_control")
            self.flight = node.create_client(FlightCommand, onboard + "/flight_command")
            self.waypoints = node.create_client(ExecuteWaypoints, onboard + "/execute_waypoints")
            node.create_service(MessageInterval, mavros + "/set_message_interval", self.success)
            node.create_service(ParamPull, mavros + "/param/pull", self.success)
            node.create_timer(.025, self.tick)

        @staticmethod
        def success(request, response):
            response.success = True
            return response

        def tick(self):
            self.pubs["state"].publish(self.state)
            self.pubs["extended_state"].publish(ExtendedState(landed_state=2 if self.state.armed else 1))
            pose = PoseStamped()
            pose.pose.position.z, pose.pose.orientation.w = self.height, 1.
            self.pubs["local_position/pose"].publish(pose)
            self.pubs["local_position/velocity_local"].publish(TwistStamped())
            for name, value in [("GUID_OPTIONS", 8), ("MOT_THST_HOVER", .22)]:
                event = ParamEvent(param_id=name)
                event.value.type = 2 if isinstance(value, int) else 3
                if isinstance(value, int):
                    event.value.integer_value = value
                else:
                    event.value.double_value = value
                self.pubs["param/event"].publish(event)
            if self.leased:
                self.sequence += 1
                heartbeat = ControlHeartbeat(source_id="avoidance-test", sequence=self.sequence,
                                             lease_duration_ms=3000)
                heartbeat.header.stamp = node.get_clock().now().to_msg()
                self.heartbeat.publish(heartbeat)
            self.reply.publish(self.result())

        def result(self, request=None):
            result = AvoidanceResult(bridge_session="bridge-test", planner_session="planner-test",
                                     coordinate_revision=1, ready=True, status=AvoidanceResult.REACH_END)
            result.header.frame_id = "map"
            result.header.stamp = node.get_clock().now().to_msg()
            if request:
                for name in ("controller_session", "task_revision", "waypoint_index", "request_id", "mode"):
                    setattr(result, name, getattr(request, name))
            return result

        def plan(self, request):
            self.requests.append((time.monotonic(), request))
            if self.policy == "silent":
                return
            result = self.result(request)
            if self.policy == "blocked":
                result.status, result.detail = AvoidanceResult.NO_PATH, "synthetic obstacle"
            elif self.policy == "wrong_task":
                result.task_revision += 1
            elif self.policy == "wrong_request":
                result.request_id += 10000
            self.reply.publish(result)

        def spin(self, seconds):
            deadline = time.monotonic() + seconds
            while time.monotonic() < deadline:
                executor.spin_once(timeout_sec=.005)

        def until(self, predicate, seconds=5):
            deadline = time.monotonic() + seconds
            while not predicate() and time.monotonic() < deadline:
                executor.spin_once(timeout_sec=.005)
            assert predicate(), ([r.message for r in self.results[-8:]],
                                 self.status[-1] if self.status else "no status")

        def request(self, kind):
            self.sequence += 1
            request = kind.Request(source_id="avoidance-test", sequence=self.sequence)
            request.stamp = node.get_clock().now().to_msg()
            return request

        def call(self, client, request):
            future = client.call_async(request)
            self.until(future.done)
            return future.result()

        def execute(self, strategy=2, generator=2, tracking=1):
            request = self.request(ExecuteWaypoints)
            request.ttl_ms = 3000
            request.flight_strategy = strategy
            request.reference_generator, request.tracking_controller = generator, tracking
            waypoint = Waypoint()
            waypoint.position.x, waypoint.position.z = 4., 1.
            request.waypoints = [waypoint]
            return self.call(self.waypoints, request)

        def command(self, command):
            request = self.request(FlightCommand)
            request.command, request.ttl_ms = command, 3000
            return self.call(self.flight, request)

    harness = Harness()
    env = dict(os.environ, ROS_DOMAIN_ID=str(domain))
    args = [str(executable), "--ros-args"]
    for name, value in {"mavros_prefix": mavros, "interface_prefix": onboard,
                        "avoidance_wait_timeout_seconds": 1.2,
                        "avoidance_result_timeout_seconds": .3,
                        "avoidance_resume_seconds": .1}.items():
        args += ["-p", f"{name}:={value}"]
    with (tmp_path / "avoidance-onboard.log").open("w") as log:
        process = subprocess.Popen(args, env=env, stdout=log, stderr=log)
        try:
            harness.until(lambda: harness.status and harness.lease.service_is_ready()
                          and harness.waypoints.service_is_ready()
                          and harness.status[-1].thrust_mode_verified
                          and harness.status[-1].avoidance_ready, 20)
            request = harness.request(AcquireControl)
            request.lease_duration_ms = 3000
            assert harness.call(harness.lease, request).granted
            harness.leased = True
            harness.spin(.1)
            yield harness
        finally:
            process.terminate()
            process.wait(timeout=10)
            executor.shutdown(timeout_sec=1.)
            node.destroy_node()
            rclpy.shutdown(context=context)


def test_identity_recovery_and_late_success_after_cancel_or_hover(avoidance_node):
    """Incorrect identities cannot release waiting; cancellation revokes even valid late replies."""
    from guided_interfaces.msg import ControlStatus
    from guided_interfaces.srv import FlightCommand
    h = avoidance_node
    for bad_policy, command in [("wrong_task", FlightCommand.Request.COMMAND_CANCEL),
                                ("wrong_request", FlightCommand.Request.COMMAND_HOVER)]:
        h.policy = bad_policy
        start = len(h.requests)
        assert h.execute().accepted
        h.until(lambda: len(h.requests) > start)
        h.spin(.35)
        assert h.status[-1].avoidance_state == ControlStatus.AVOIDANCE_WAITING
        assert h.status[-1].waypoint_index == 1
        h.policy = "clear"
        h.until(lambda: h.status[-1].avoidance_state == ControlStatus.AVOIDANCE_EXECUTING)
        assert h.status[-1].active_reference_generator == 2
        assert h.status[-1].active_tracking_controller == 1
        old_request = h.requests[-1][1]
        h.policy = "silent"
        assert h.command(command).accepted
        h.until(lambda: h.status[-1].control_mode == ControlStatus.MODE_HOVER)
        for _ in range(3):
            h.reply.publish(h.result(old_request))
            h.spin(.12)
        assert h.status[-1].avoidance_state == ControlStatus.AVOIDANCE_DISABLED
        assert h.status[-1].waypoint_count == 0
        assert h.status[-1].control_mode == ControlStatus.MODE_HOVER


def test_timeout_land_retries_pending_expiry_and_disarm_latch(avoidance_node):
    """Repeated obstacles keep one deadline; service loss/negative ACK/hung calls must recover."""
    from rclpy.callback_groups import ReentrantCallbackGroup
    from rclpy.task import Future
    from mavros_msgs.srv import SetMode
    from guided_interfaces.msg import ControlStatus, CommandResult, MotionIntent
    from guided_interfaces.srv import FlightCommand
    h = avoidance_node
    h.policy = "blocked"
    assert h.execute().accepted
    h.until(lambda: h.requests)
    first_request = h.requests[0][0]
    h.until(lambda: h.status[-1].avoidance_state == ControlStatus.AVOIDANCE_LAND, 2.)
    assert 0.8 < time.monotonic() - first_request < 1.6
    assert len(h.requests) >= 3  # Repeated NO_PATH did not postpone the original deadline.
    assert any(r.final and r.status == CommandResult.STATUS_FAILED and r.command == "waypoints"
               for r in h.results)
    assert not h.command(FlightCommand.Request.COMMAND_HOVER).accepted
    assert not h.execute().accepted
    h.sequence += 1
    motion = MotionIntent(source_id="avoidance-test", sequence=h.sequence, ttl_ms=3000)
    motion.header.stamp = h.node.get_clock().now().to_msg()
    motion.velocity_delta.x = .1
    h.motion.publish(motion)
    h.until(lambda: any(r.sequence == motion.sequence and r.status == CommandResult.STATUS_REJECTED
                        for r in h.results))

    attempts, hung = [], []
    behavior = "negative"

    async def mode(request, response):
        """Keep one reply pending while allowing timers and later service requests to execute."""
        attempts.append((time.monotonic(), request.custom_mode))
        if behavior == "hang":
            pending = Future()
            hung.append(pending)
            await pending
        response.mode_sent = behavior != "negative"
        return response

    h.node.create_service(SetMode, h.mavros + "/set_mode", mode,
                          callback_group=ReentrantCallbackGroup())
    h.until(lambda: len(attempts) >= 2, 4.)
    assert all(name == "LAND" for _, name in attempts)
    assert attempts[1][0] - attempts[0][0] >= .85
    behavior = "hang"
    h.until(lambda: hung, 3.)
    hanging_attempt = attempts[-1][0]
    count = len(attempts)
    behavior = "ack"
    h.until(lambda: len(attempts) > count, 3.)
    assert attempts[-1][0] - hanging_attempt >= 1.9
    count = len(attempts)
    h.until(lambda: len(attempts) > count, 2.)  # ACK alone did not establish FCU LAND.
    assert h.status[-1].avoidance_state == ControlStatus.AVOIDANCE_LAND
    assert not h.command(FlightCommand.Request.COMMAND_HOVER).accepted
    h.state.mode = "LAND"
    h.spin(.3)
    count = len(attempts)
    h.spin(1.2)
    assert len(attempts) == count
    assert h.status[-1].avoidance_state == ControlStatus.AVOIDANCE_LAND
    h.state.armed = False
    h.until(lambda: not h.status[-1].armed and h.status[-1].avoidance_state == ControlStatus.AVOIDANCE_DISABLED)
    for pending in hung:
        pending.set_result(True)  # Late ACK must not resurrect LAND after disarm invalidated it.
    h.spin(.3)
    assert h.status[-1].control_mode == ControlStatus.MODE_IDLE
    assert len(attempts) == count


def test_takeoff_preserves_preflight_configuration_and_failure_unlocks(avoidance_node):
    """Takeoff cancellation must not erase the new lock; failed unarmed attempts leave no lock."""
    from mavros_msgs.srv import SetMode, CommandBool, CommandTOL
    from guided_interfaces.msg import ControlStatus, CommandResult
    from guided_interfaces.srv import FlightCommand
    h = avoidance_node
    h.state.armed, h.state.mode = False, "STABILIZE"
    h.height = 0.
    h.until(lambda: not h.status[-1].armed and h.status[-1].autopilot_mode == "STABILIZE")

    def takeoff():
        request = h.request(FlightCommand)
        request.command, request.ttl_ms = FlightCommand.Request.COMMAND_TAKEOFF, 3000
        request.value = 2.
        request.flight_strategy, request.reference_generator, request.tracking_controller = 2, 2, 1
        assert h.call(h.flight, request).accepted
        return request.sequence

    def failed_unlocked(sequence):
        h.until(lambda: any(r.sequence == sequence and r.final and
                            r.status == CommandResult.STATUS_FAILED for r in h.results))
        h.until(lambda: not h.status[-1].waypoint_configuration_locked)
        assert not h.status[-1].armed

    # No fictitious mode service exists yet: ensure_guided fails synchronously within takeoff.
    failed_unlocked(takeoff())
    mode_accept, arm_accept = False, False
    calls = []

    def mode(request, response):
        calls.append(("mode", request.custom_mode))
        response.mode_sent = mode_accept
        if mode_accept:
            h.state.mode = request.custom_mode
            h.pubs["state"].publish(h.state)
        return response

    def arm(request, response):
        calls.append(("arm", request.value))
        response.success = arm_accept
        if arm_accept:
            h.state.armed = request.value
            h.pubs["state"].publish(h.state)
        return response

    def climb(request, response):
        calls.append(("takeoff", request.altitude))
        response.success = True
        h.height = float(request.altitude)
        return response

    h.node.create_service(SetMode, h.mavros + "/set_mode", mode)
    h.node.create_service(CommandBool, h.mavros + "/cmd/arming", arm)
    h.node.create_service(CommandTOL, h.mavros + "/cmd/takeoff", climb)
    h.spin(.3)
    failed_unlocked(takeoff())  # Negative GUIDED ACK is asynchronous but must clear the lock too.
    assert calls[-1] == ("mode", "GUIDED")
    mode_accept = True
    failed_unlocked(takeoff())  # PreArm rejection also remains editable while unarmed.
    assert calls[-1] == ("arm", True)
    arm_accept = True
    successful = takeoff()
    h.until(lambda: any(r.sequence == successful and r.final and
                        r.status == CommandResult.STATUS_SUCCEEDED for r in h.results))
    h.until(lambda: h.status[-1].armed and h.status[-1].control_mode == ControlStatus.MODE_HOVER)
    status = h.status[-1]
    assert status.waypoint_configuration_locked
    assert (status.flight_strategy, status.locked_reference_generator,
            status.locked_tracking_controller) == (2, 2, 1)
    assert calls[-1] == ("takeoff", 2.)
    for changed in [(0, 2, 1), (2, 1, 1), (2, 2, 0)]:
        response = h.execute(*changed)
        assert not response.accepted
        assert "禁止修改" in response.message
    h.policy = "clear"
    assert h.execute(2, 2, 1).accepted
    h.until(lambda: h.status[-1].avoidance_state == ControlStatus.AVOIDANCE_EXECUTING)
    assert h.status[-1].waypoint_configuration_locked
