"""Real onboard executor regressions, including native-stream path joining, on localhost DDS doubles."""
import time

import pytest

pytestmark = pytest.mark.integration

from tests.support.avoidance_node import avoidance_node, _issued_acceleration


@pytest.mark.parametrize("avoidance_node,strategy,generator", [
    pytest.param({"max_yaw_rate": .7, "waypoint_hold_seconds": .1}, strategy, generator,
                 id=f"strategy-{strategy}-generator-{generator}")
    for strategy, generator in [(0, 0), (0, 1), (0, 2), (0, 3), (1, 2), (2, 2)]
], indirect=["avoidance_node"])
def test_short_segment_yaw_rate_and_arrival_wait(avoidance_node, strategy, generator):
    """Check actual attitude messages and prevent completion/capture while a 180-degree reference turns."""
    import math
    from guided_interfaces.msg import CommandResult, VideoCapture
    from guided_interfaces.srv import FlightCommand
    h = avoidance_node
    captures = []
    subscription = h.node.create_subscription(VideoCapture, "/video_service/capture", captures.append, 10)
    h.policy = "clear"
    assert h.command(FlightCommand.Request.COMMAND_HOVER).accepted
    h.spin(.1)
    checkpoint = len(h.attitudes)
    assert h.execute(strategy=strategy, generator=generator, goal_x=.01, goal_yaw=math.pi).accepted
    h.until(lambda: h.status[-1].target_yaw > .1)
    if strategy == 1:
        # The real planner stops refreshing a near-goal curve; keep its last accepted result.
        h.policy, h.active_plan = "silent", None
    h.spin(.3)
    assert not captures
    assert not any(result.final and result.command == "waypoints" for result in h.results)
    h.until(lambda: any(result.final and result.command == "waypoints" and
            result.status == CommandResult.STATUS_SUCCEEDED for result in h.results), 8.)
    h.until(lambda: bool(captures))
    messages = h.attitudes[checkpoint:]
    assert len(messages) > 50
    for previous, current in zip(messages, messages[1:]):
        q0, q1 = previous.orientation, current.orientation
        yaw0 = math.atan2(2. * (q0.w * q0.z + q0.x * q0.y), 1. - 2. * (q0.y ** 2 + q0.z ** 2))
        yaw1 = math.atan2(2. * (q1.w * q1.z + q1.x * q1.y), 1. - 2. * (q1.y ** 2 + q1.z ** 2))
        interval = ((current.header.stamp.sec - previous.header.stamp.sec) +
                    (current.header.stamp.nanosec - previous.header.stamp.nanosec) * 1.e-9)
        # Publication timestamps can differ slightly from the control-loop measurement point.
        assert abs(math.remainder(yaw1 - yaw0, 2. * math.pi)) <= .7 * interval + .005
    assert abs(math.remainder(h.status[-1].target_yaw - math.pi, 2. * math.pi)) < 1.e-6
    h.node.destroy_subscription(subscription)


@pytest.mark.parametrize("avoidance_node", [{"waypoint_hold_seconds": .15}], indirect=True)
def test_waypoint_capture_keeps_arrival_identity_before_advancing(avoidance_node):
    """Real DDS capture events keep each arrived point's identity while task progress advances."""
    from guided_interfaces.msg import CommandResult, VideoCapture, Waypoint
    from guided_interfaces.srv import ExecuteWaypoints
    from rclpy.qos import ReliabilityPolicy

    h = avoidance_node
    captures = []
    subscription = h.node.create_subscription(
        VideoCapture, "/video_service/capture", captures.append, 10)
    try:
        h.until(lambda: bool(h.node.get_publishers_info_by_topic("/video_service/capture")))
        publishers = h.node.get_publishers_info_by_topic("/video_service/capture")
        assert all(info.qos_profile.reliability == ReliabilityPolicy.RELIABLE
                   for info in publishers)
        request = h.request(ExecuteWaypoints)
        request.ttl_ms = 3000
        request.waypoints = [
            Waypoint(is_inspection=True, has_photo_no=True, photo_no="arrival-one"),
            Waypoint(is_inspection=True),
        ]
        for waypoint in request.waypoints:
            waypoint.position.z = h.height
        assert h.call(h.waypoints, request).accepted
        h.until(lambda: any(result.final and result.command == "waypoints" and
                            result.status == CommandResult.STATUS_SUCCEEDED for result in h.results))
        h.until(lambda: len(captures) == 2)
        h.until(lambda: h.status[-1].waypoint_index == 2)
        assert [(capture.waypoint_index, capture.photo_no, capture.kind) for capture in captures] == [
            (1, "arrival-one", VideoCapture.KIND_UPSTREAM_WAYPOINT),
            (2, "", VideoCapture.KIND_GCS_WAYPOINT),
        ]
        assert all(capture.source_id == request.source_id and
                   capture.mission_sequence == request.sequence for capture in captures)
        assert captures[0].sequence < captures[1].sequence
    finally:
        h.node.destroy_subscription(subscription)


def _jerk_test_parameters(mode, override):
    """Use a visibly slow ramp and disable disturbance learning to isolate mode selection."""
    parameters = {"max_jerk_xy": .1, "max_jerk_z": .1,
                  "dob_L_xy": 0., "dob_L_z": 0.,
                  "trajectory_dob_L_xy": 0., "trajectory_dob_L_z": 0.,
                  "trajectory_wn_xy": 2.236, "trajectory_wn_z": 2.236,
                  "avoidance_wait_timeout_seconds": 8.}
    if override:
        parameters["jerk_limit_modes." + mode] = mode != "avoidance"
    return parameters


def test_output_jerk_parameter_defaults(avoidance_node):
    """Read declarations from the running node, including each independent startup switch."""
    from rcl_interfaces.srv import GetParameters
    h = avoidance_node
    client = h.node.create_client(GetParameters, "/onboard_control_node/get_parameters")
    h.until(client.service_is_ready)
    names = ["max_jerk_xy", "max_jerk_z"] + ["jerk_limit_modes." + mode for mode in
             ("hover", "motion", "straight", "avoidance", "obstacle_hover", "failsafe_hold")]
    values = h.call(client, GetParameters.Request(names=names)).values
    assert [value.double_value for value in values[:2]] == [10., 8.]
    assert [value.bool_value for value in values[2:]] == [False, False, False, True, False, False]


@pytest.mark.parametrize("avoidance_node,mode,override", [
    pytest.param(_jerk_test_parameters(mode, override), mode, override,
                 id=f"{mode}-{'override' if override else 'default'}")
    for mode in ("hover", "motion", "straight", "avoidance", "obstacle_hover", "failsafe_hold")
    for override in (False, True)
], indirect=["avoidance_node"])
def test_output_jerk_follows_runtime_mode(avoidance_node, mode, override):
    """A measured pose step must reach the real output immediately or through the selected ramp."""
    from guided_interfaces.msg import ControlStatus, MotionIntent
    from guided_interfaces.srv import FlightCommand
    h = avoidance_node
    assert h.command(FlightCommand.Request.COMMAND_HOVER).accepted
    if mode in ("straight", "avoidance", "obstacle_hover"):
        h.policy = "clear"
        strategy = {"straight": 0, "avoidance": 1, "obstacle_hover": 2}[mode]
        assert h.execute(strategy=strategy).accepted
        h.until(lambda: h.status[-1].control_mode == ControlStatus.MODE_WAYPOINT)
        if strategy:
            h.until(lambda: h.status[-1].avoidance_state == ControlStatus.AVOIDANCE_EXECUTING)
        if strategy == 1:
            # Freeze this reference; fresh plans would follow the injected measured pose step.
            h.policy, h.active_plan = "silent", None
    elif mode == "motion":
        h.sequence += 1
        motion = MotionIntent(source_id="avoidance-test", sequence=h.sequence, ttl_ms=3000)
        motion.header.stamp = h.node.get_clock().now().to_msg()
        motion.velocity_delta.x = .1
        h.motion.publish(motion)
        h.until(lambda: h.status[-1].control_mode == ControlStatus.MODE_MOTION)
    elif mode == "failsafe_hold":
        h.leased = False
        h.until(lambda: h.status[-1].control_mode == ControlStatus.MODE_FAILSAFE_HOLD, 4.)
    _assert_pose_step_output_jerk(h, (mode == "avoidance") != override)


def _assert_pose_step_output_jerk(h, enabled):
    """Exercise both axes without approaching acceleration, tilt or tracking-error limits."""
    h.spin(.15)
    assert h.attitudes
    before = _issued_acceleration(h.attitudes[-1])
    h.position_x += .2
    h.height += .2
    h.spin(.15)
    after = _issued_acceleration(h.attitudes[-1])
    changes = [abs(after[axis] - before[axis]) for axis in (0, 2)]
    if enabled:
        assert max(changes) < .08, changes
    else:
        assert min(changes) > .5, changes


@pytest.mark.parametrize("avoidance_node,phase", [
    pytest.param(_jerk_test_parameters("avoidance", False), phase, id=phase)
    for phase in ("preparation", "planning_wait", "completed")
], indirect=["avoidance_node"])
def test_autonomous_jerk_spans_preparation_wait_and_completion(avoidance_node, phase):
    """Task identity retains autonomous shaping in temporary holds and releases it on completion."""
    from guided_interfaces.msg import ControlStatus, CommandResult
    h = avoidance_node
    if phase == "preparation":
        h.velocity_x = .3
        h.spin(.15)
    elif phase == "completed":
        h.policy = "clear"
    assert h.execute(strategy=1, goal_x=.04 if phase == "completed" else 4.).accepted
    if phase == "preparation":
        h.until(lambda: h.status[-1].control_mode == ControlStatus.MODE_HOVER and
                h.status[-1].waypoint_count == 1)
    else:
        h.until(lambda: h.status[-1].avoidance_state == (
            ControlStatus.AVOIDANCE_EXECUTING if phase == "completed" else ControlStatus.AVOIDANCE_WAITING))
    if phase == "completed":
        h.policy, h.active_plan = "silent", None
        h.position_x = .04
        h.until(lambda: any(r.final and r.command == "waypoints" and
                r.status == CommandResult.STATUS_SUCCEEDED for r in h.results), 5.)
        h.until(lambda: h.status[-1].control_mode == ControlStatus.MODE_HOVER)
    _assert_pose_step_output_jerk(h, phase != "completed")


@pytest.mark.parametrize("strategy", [0, 2, 1])
def test_tracking_error_stop_is_only_for_autonomous_flight(avoidance_node, strategy):
    """Clear straight/obstacle-hover references advance past 0.8 m; native curves still stop."""
    from guided_interfaces.msg import ControlStatus
    h = avoidance_node
    h.policy = "delayed_curve" if strategy == 1 else "clear"
    assert h.execute(strategy=strategy, goal_x=.6 if strategy == 1 else 4.).accepted
    if strategy == 1:
        h.until(lambda: h.status[-1].avoidance_state == ControlStatus.AVOIDANCE_EXECUTING)
        h.policy = "silent"
        h.active_plan = None
        h.delayed_replies.clear()
        h.position_x = 1.5
        h.until(lambda: "轨迹跟踪误差" in h.status[-1].avoidance_detail)
        assert h.status[-1].avoidance_state in (
            ControlStatus.AVOIDANCE_BRAKING, ControlStatus.AVOIDANCE_WAITING)
    else:
        # Keep measured state stationary to exercise reference lag beyond the former threshold.
        h.until(lambda: h.status[-1].target_position.x > 1., 5.)
        h.spin(.3)
        status = h.status[-1]
        assert status.target_position.x - status.position.x > 1.
        assert status.target_velocity.x > 0.
        assert status.control_mode == ControlStatus.MODE_WAYPOINT
        assert status.avoidance_state == (
            ControlStatus.AVOIDANCE_EXECUTING if strategy == 2 else ControlStatus.AVOIDANCE_DISABLED)
        assert not any(r.final and r.command == "waypoints" for r in h.results)


def test_native_stream_joins_current_position_and_replaces_moving_curve(avoidance_node):
    """One PLAN streams new curves; trimming preserves native timing and rejects old sequences."""
    from guided_interfaces.msg import AvoidanceRequest, ControlStatus, PolynomialSegment
    h = avoidance_node
    h.policy = "delayed_curve"
    assert h.execute(strategy=1, goal_x=.6).accepted
    h.until(lambda: h.status[-1].avoidance_state == ControlStatus.AVOIDANCE_EXECUTING)
    paths = [p for p in h.paths if p.poses]
    assert paths and abs(paths[0].poses[0].pose.position.x - .24) < 1e-8
    assert len(paths[0].poses) == 62  # The physically delayed prefix is about 0.05 seconds long.
    assert all(p.poses[0].pose.position.x >= .24 - 1e-8 for p in paths)
    h.until(lambda: h.status[-1].target_position.x > .24 + .005)
    assert abs(h.status[-1].target_position.x - .24) < .08
    plans = [r for _, r in h.requests if r.mode == AvoidanceRequest.PLAN]
    assert len(plans) == 1
    assert not any(r.mode == AvoidanceRequest.VALIDATE_TRAJECTORY for _, r in h.requests)

    h.policy = "silent"
    h.delayed_replies.clear()
    request = plans[0]
    old = h.result(request)
    old.status = old.NO_PATH
    replacement = h.result(request)
    replacement.segments = [PolynomialSegment(
        duration=3.6, x=[.24, .1, 1. / 36., -1. / 129.6], y=[0., 0., 0., 0.], z=[1., 0., 0., 0.])]
    path_count = len(h.paths)
    h.reply.publish(replacement)
    h.until(lambda: any(p.poses and abs(p.poses[-1].pose.position.x - .6) < 1e-8
                       and len(p.poses) == 37 for p in h.paths[path_count:]), .25)
    h.reply.publish(old)
    h.spin(.08)
    assert h.status[-1].avoidance_state == ControlStatus.AVOIDANCE_EXECUTING
    assert len([r for _, r in h.requests if r.mode == AvoidanceRequest.PLAN]) == 1
    h.spin(.5)  # Native goal proximity stops replanning while the bridge heartbeat continues.
    assert h.status[-1].avoidance_state == ControlStatus.AVOIDANCE_EXECUTING
    assert h.status[-1].target_position.x > .29
    h.position_x, h.velocity_x = .6, 0.
    from guided_interfaces.msg import CommandResult
    h.until(lambda: any(r.final and r.command == "waypoints" and
                        r.status == CommandResult.STATUS_SUCCEEDED for r in h.results), 5.)
    h.until(lambda: h.status[-1].control_mode == ControlStatus.MODE_HOVER)
    assert h.status[-1].avoidance_state == ControlStatus.AVOIDANCE_DISABLED


def test_single_delayed_plan_resumes_without_another_success_or_full_stop(avoidance_node):
    """A near-goal one-shot curve joins moving state despite normal delay-induced velocity error."""
    from rclpy.duration import Duration
    from guided_interfaces.msg import AvoidanceRequest, ControlStatus, PolynomialSegment
    h = avoidance_node
    h.position_x, h.velocity_x = .024, .12
    h.spin(.15)
    assert h.execute(strategy=1, goal_x=.12 + .5 - 1.12 / 3.).accepted
    h.until(lambda: any(r.mode == AvoidanceRequest.PLAN for _, r in h.requests))
    request = next(r for _, r in h.requests if r.mode == AvoidanceRequest.PLAN)
    result = h.result(request)
    result.header.stamp = (h.node.get_clock().now() - Duration(seconds=.2)).to_msg()
    result.segments = [PolynomialSegment(
        duration=1., x=[0., .12, .5, -1.12 / 3.],
        y=[0., 0., 0., 0.], z=[1., 0., 0., 0.])]
    h.reply.publish(result)
    h.until(lambda: h.status[-1].avoidance_state == ControlStatus.AVOIDANCE_EXECUTING, .5)
    assert any(p.poses and abs(p.poses[0].pose.position.x - .024) < 1e-7 for p in h.paths)
    h.spin(1.3)  # The native planner may send only identity heartbeats after this success.
    assert h.status[-1].avoidance_state == ControlStatus.AVOIDANCE_EXECUTING
    assert len([r for _, r in h.requests if r.mode == AvoidanceRequest.PLAN]) == 1


def test_identity_recovery_and_late_success_after_cancel_or_hover(avoidance_node):
    """Incorrect identities cannot release waiting; cancellation revokes even valid late replies."""
    from guided_interfaces.msg import AvoidanceRequest, ControlStatus
    from guided_interfaces.srv import FlightCommand
    h = avoidance_node
    for bad_policy, command in [("wrong_task", FlightCommand.Request.COMMAND_CANCEL),
                                ("wrong_request", FlightCommand.Request.COMMAND_HOVER)]:
        h.policy = bad_policy
        start = len(h.requests)
        assert h.execute(strategy=1).accepted
        h.until(lambda: len(h.requests) > start)
        h.spin(.35)
        assert h.status[-1].avoidance_state == ControlStatus.AVOIDANCE_WAITING
        assert h.status[-1].waypoint_index == 1
        h.policy = "clear"
        h.until(lambda: h.status[-1].avoidance_state == ControlStatus.AVOIDANCE_EXECUTING)
        assert h.status[-1].active_reference_generator == ControlStatus.REFERENCE_PLANNER_TRAJECTORY
        assert h.status[-1].active_tracking_controller == 1
        old_request = next(r for _, r in reversed(h.requests) if r.mode != AvoidanceRequest.CANCEL)
        h.policy = "silent"
        assert h.command(command).accepted
        h.until(lambda: h.status[-1].control_mode == ControlStatus.MODE_HOVER)
        h.until(lambda: any(r.mode == AvoidanceRequest.CANCEL and
                            r.request_id == old_request.request_id for _, r in h.requests))
        assert h.active_plan is None
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
    from guided_interfaces.msg import AvoidanceRequest, ControlStatus, CommandResult, MotionIntent
    from guided_interfaces.srv import FlightCommand
    h = avoidance_node
    h.policy = "blocked"
    assert h.execute().accepted
    h.until(lambda: h.requests)
    first_request = h.requests[0][0]
    h.until(lambda: h.status[-1].avoidance_state == ControlStatus.AVOIDANCE_LAND, 2.)
    assert 0.8 < time.monotonic() - first_request < 1.6
    checks = [r for _, r in h.requests if r.mode == AvoidanceRequest.CHECK_LINE]
    assert len(checks) >= 3  # Repeated NO_PATH did not postpone the original deadline.
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
    from std_msgs.msg import Float64
    h = avoidance_node
    altitude_pub = h.node.create_publisher(Float64, h.mavros + "/global_position/rel_alt", 10)
    h.node.create_timer(.025, lambda: altitude_pub.publish(Float64(data=h.height)))
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
    h.motion_limits = (.6, .2, 1.7, 1.4)
    h.spin(1.1)  # Different native limits must not override clear obstacle-hover flight shaping.
    h.height = 1.
    h.spin(.1)
    assert h.execute(2, 2, 1).accepted
    h.until(lambda: h.status[-1].avoidance_state == ControlStatus.AVOIDANCE_EXECUTING)
    assert h.status[-1].waypoint_configuration_locked
    h.until(lambda: any(abs(s.target_acceleration.x - .35) < 1e-6 for s in h.status))
    # Publish plausible progress so the existing large tracking-error gate does not stop the test.
    deadline = time.monotonic() + 5.
    while time.monotonic() < deadline and h.status[-1].target_velocity.x < 1. - 1e-6:
        h.position_x = h.status[-1].target_position.x
        h.velocity_x = h.status[-1].target_velocity.x
        h.spin(.025)
    assert h.status[-1].target_velocity.x == pytest.approx(1., abs=1e-6)
    h.velocity_x = .7
    h.velocity_z = .3
    h.spin(.075)
    h.policy = "blocked"
    first = len(h.status)
    h.until(lambda: any(s.avoidance_state == ControlStatus.AVOIDANCE_BRAKING and
                       abs(s.target_acceleration.x + 1.7) < 1e-6 for s in h.status[first:]))
    braking = next(s for s in h.status[first:] if abs(s.target_acceleration.x + 1.7) < 1e-6)
    stop_x = braking.target_position.x + braking.target_velocity.x ** 2 / (2. * 1.7)
    assert braking.target_acceleration.z == pytest.approx(-1.4, abs=1e-6)
    stop_z = h.height + h.velocity_z ** 2 / (2. * 1.4)
    h.until(lambda: abs(h.status[-1].target_velocity.x) < 1e-6 and
            abs(h.status[-1].target_acceleration.x) < 1e-6)
    assert h.status[-1].target_position.x == pytest.approx(stop_x, abs=1e-6)
    assert h.status[-1].target_position.z == pytest.approx(stop_z, abs=1e-6)
    h.spin(.2)
    assert h.status[-1].target_position.x == pytest.approx(stop_x, abs=1e-6)
