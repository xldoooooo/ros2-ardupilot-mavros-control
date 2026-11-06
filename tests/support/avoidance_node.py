"""Isolated production executor fixture and measured-output helpers for flight tests."""
import os
import subprocess
import time
from pathlib import Path

import pytest


@pytest.fixture
def avoidance_node(tmp_path, monkeypatch, request):
    """Own only this node/process; fictitious MAVROS names cannot address aircraft topics."""
    monkeypatch.setenv("ROS_AUTOMATIC_DISCOVERY_RANGE", "LOCALHOST")
    monkeypatch.delenv("ROS_STATIC_PEERS", raising=False)
    rclpy = pytest.importorskip("rclpy")
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import qos_profile_sensor_data
    from geometry_msgs.msg import PoseStamped, TwistStamped
    from mavros_msgs.msg import State, ExtendedState, ParamEvent, AttitudeTarget
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
        position_x = 0.
        velocity_x = 0.
        velocity_z = 0.
        motion_limits = (1., .2, 2., 2.)

        def __init__(self):
            """Own the publishers, subscriptions and service clients used by this test."""
            self.status, self.results, self.requests, self.paths = [], [], [], []
            self.attitudes = []
            self.delayed_replies = []
            self.active_plan = None
            self.plan_sequences = {}
            self.last_plan_result = 0.
            self.last_bridge_heartbeat = 0.
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
            node.create_subscription(AttitudeTarget, mavros + "/setpoint_raw/attitude",
                                     self.attitudes.append, qos_profile_sensor_data)
            node.create_subscription(CommandResult, onboard + "/command_result", self.results.append, 50)
            node.create_subscription(AvoidanceRequest, onboard + "/avoidance_request", self.plan, 10)
            from nav_msgs.msg import Path as RosPath
            node.create_subscription(RosPath, onboard + "/avoidance_path", self.paths.append, 10)
            self.lease = node.create_client(AcquireControl, onboard + "/acquire_control")
            self.flight = node.create_client(FlightCommand, onboard + "/flight_command")
            self.waypoints = node.create_client(ExecuteWaypoints, onboard + "/execute_waypoints")
            node.create_service(MessageInterval, mavros + "/set_message_interval", self.success)
            node.create_service(ParamPull, mavros + "/param/pull", self.success)
            node.create_timer(.025, self.tick)

        @staticmethod
        def success(request, response):
            """Acknowledge a fictitious MAVROS setup service without hardware access."""
            response.success = True
            return response

        def tick(self):
            """Publish independent measured state, parameter events and lease heartbeats."""
            self.pubs["state"].publish(self.state)
            self.pubs["extended_state"].publish(ExtendedState(landed_state=2 if self.state.armed else 1))
            pose = PoseStamped()
            pose.pose.position.x = self.position_x
            pose.pose.position.z, pose.pose.orientation.w = self.height, 1.
            self.pubs["local_position/pose"].publish(pose)
            twist = TwistStamped()
            twist.twist.linear.x = self.velocity_x
            twist.twist.linear.z = self.velocity_z
            self.pubs["local_position/velocity_local"].publish(twist)
            # Match the real bridge's 1 Hz heartbeat. The executor consumes depth=1:
            # a heartbeat immediately after a reply would overwrite that reply in DDS.
            now = time.monotonic()
            if now - self.last_bridge_heartbeat >= 1.:
                self.reply.publish(self.result())
                self.last_bridge_heartbeat = now
            for deadline, result in list(self.delayed_replies):
                if now >= deadline:
                    self.reply.publish(result)
                    self.delayed_replies.remove((deadline, result))
            if self.active_plan and now - self.last_plan_result >= .1:
                self.emit_result(self.active_plan)
                self.last_plan_result = now
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

        def result(self, request=None):
            """Build one planner observation with the matching request identity."""
            result = AvoidanceResult(max_velocity_xy=self.motion_limits[0],
                                     max_velocity_z=self.motion_limits[1],
                                     max_acceleration_xy=self.motion_limits[2],
                                     max_acceleration_z=self.motion_limits[3],
                                     bridge_session="bridge-test", planner_session="planner-test",
                                     coordinate_revision=1, ready=True, status=AvoidanceResult.REACH_END)
            result.header.frame_id = "map"
            result.header.stamp = node.get_clock().now().to_msg()
            if request:
                self.plan_sequences[request.request_id] = self.plan_sequences.get(request.request_id, 0) + 1
                result.plan_sequence = self.plan_sequences[request.request_id]
                for name in ("controller_session", "task_revision", "waypoint_index", "request_id", "mode"):
                    setattr(result, name, getattr(request, name))
            return result

        def plan(self, request):
            """Keep PLAN active; CANCEL revokes it without creating a fictitious trajectory."""
            self.requests.append((time.monotonic(), request))
            if request.mode == request.CANCEL:
                if self.active_plan and request.request_id == self.active_plan.request_id:
                    self.active_plan = None
                return
            if request.mode == request.PLAN:
                self.active_plan = request
            self.emit_result(request)

        def emit_result(self, request):
            """Emulate independent native results, preserving one request identity."""
            if self.policy == "silent":
                return
            result = self.result(request)
            if self.policy == "blocked":
                result.status, result.detail = AvoidanceResult.NO_PATH, "synthetic obstacle"
            elif self.policy == "wrong_task":
                result.task_revision += 1
            elif self.policy == "wrong_request":
                result.request_id += 10000
            elif self.policy == "delayed_curve" and request.mode == request.PLAN:
                from guided_interfaces.msg import PolynomialSegment
                # The original prefix is behind the current measured position at delivery.
                curve = PolynomialSegment(duration=6.1, x=[.234, .12, -.12 / 12.2, 0.],
                                          y=[0., 0., 0., 0.], z=[1., 0., 0., 0.])
                result.segments = [curve]
                self.position_x, self.velocity_x = .24, .10
                self.delayed_replies.append((time.monotonic() + .05, result))
                return
            elif request.mode == request.PLAN:
                from guided_interfaces.msg import PolynomialSegment
                speed = .05
                duration = 2. * (request.goal.x - self.position_x) / speed
                result.segments = [PolynomialSegment(
                    duration=duration,
                    x=[self.position_x, speed, -speed / (2. * duration), 0.],
                    y=[0., 0., 0., 0.],
                    z=[self.height, 0., 0., 0.])]
            self.reply.publish(result)

        def spin(self, seconds):
            """Process DDS callbacks for a bounded interval."""
            deadline = time.monotonic() + seconds
            while time.monotonic() < deadline:
                executor.spin_once(timeout_sec=.005)

        def until(self, predicate, seconds=5):
            """Wait for observed behavior and include native state on failure."""
            deadline = time.monotonic() + seconds
            while not predicate() and time.monotonic() < deadline:
                executor.spin_once(timeout_sec=.005)
            assert predicate(), ([r.message for r in self.results[-8:]],
                                 {name: getattr(self.status[-1], name) for name in
                                  ("avoidance_state", "avoidance_detail", "control_mode",
                                   "reference_phase", "target_position", "position")}
                                 if self.status else "no status")

        def request(self, kind):
            """Issue a unique command sequence in this fixture lease."""
            self.sequence += 1
            request = kind.Request(source_id="avoidance-test", sequence=self.sequence)
            request.stamp = node.get_clock().now().to_msg()
            return request

        def call(self, client, request):
            """Wait for one asynchronous response from the isolated service."""
            future = client.call_async(request)
            self.until(future.done)
            return future.result()

        def execute(self, strategy=2, generator=2, tracking=1, goal_x=4., goal_yaw=0.):
            """Submit a waypoint to the real executor through its isolated command service."""
            request = self.request(ExecuteWaypoints)
            request.ttl_ms = 3000
            request.flight_strategy = strategy
            request.reference_generator, request.tracking_controller = generator, tracking
            # These existing arrival/capture regressions explicitly exercise inspection points.
            waypoint = Waypoint(is_inspection=True)
            waypoint.position.x, waypoint.position.z = goal_x, 1.
            waypoint.yaw = goal_yaw
            request.waypoints = [waypoint]
            return self.call(self.waypoints, request)

        def command(self, command):
            """Send a bounded flight command to the fictitious aircraft namespace."""
            request = self.request(FlightCommand)
            request.command, request.ttl_ms = command, 3000
            return self.call(self.flight, request)

    harness = Harness()
    env = dict(os.environ, ROS_DOMAIN_ID=str(domain))
    args = [str(executable), "--ros-args"]
    parameters = {"mavros_prefix": mavros, "interface_prefix": onboard,
                        "avoidance_wait_timeout_seconds": 1.2,
                        "avoidance_result_timeout_seconds": .3,
                        "avoidance_resume_seconds": .1}
    parameters.update(getattr(request, "param", {}))
    for name, value in parameters.items():
        if isinstance(value, bool):
            value = str(value).lower()
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


def _issued_acceleration(message):
    """Invert the real attitude/thrust mapping for this fixture's known hover calibration."""
    q = message.orientation
    body_z = (2. * (q.x * q.z + q.w * q.y),
              2. * (q.y * q.z - q.w * q.x),
              1. - 2. * (q.x * q.x + q.y * q.y))
    ratio = (message.thrust / .22 if message.thrust <= .22 else
             1. + (message.thrust - .22) * 1.5 / .78)
    return tuple(9.8 * ratio * component - (9.8 if axis == 2 else 0.)
                 for axis, component in enumerate(body_z))
