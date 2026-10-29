#!/usr/bin/env python3
"""Exercise the optional backend's real DDS contract on localhost domain 226."""

from __future__ import annotations

import json
import math
import os
from pathlib import Path
import sys
import threading
import time

import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from path_planning.msg import PlanResult, PolynomialSegment
from path_planning.srv import PlanMotion, SetGoal
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header, String

# Import the editable optional package while using installed native ROS types.
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "workspace/src/fpga_uav_planner_ros2"))
from fpga_uav_planner_ros2.flight_backend_node import FlightBackend
from dyn_ising_adapter import xyz_to_cell3d


class Contract:
    """Drive synthetic FCU/world input and inspect native results over actual DDS."""

    def __init__(self):
        """Use only MOCK software solving and an isolated namespace."""
        rclpy.init(args=["--ros-args", "-r", "__ns:=/fpga_contract",
                         "-p", "planning_frame:=odom", "-p", "cloud_topic:=/fpga_contract/cloud"])
        self.backend = FlightBackend(dict(
            mock=True, input_timeout=0.5, planning_timeout=2.0,
            replan_interval=0.35, replan_distance=100.0,
            max_velocity_xy=1.0, max_velocity_z=1.0,
            max_acceleration_xy=1.5, max_acceleration_z=1.5,
            near_goal_direct_m=0.0, near_goal_tiny_m=0.0,
            strict_spine_latency_fuse_ms=0.0, adaptive_solver_profile=False,
            quality_guard=False, iterations=40, restarts=1,
            window_margin_m=1.0, uav_radius_m=0.15,
        ))
        self.node = rclpy.create_node("observer")
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.backend)
        self.executor.add_node(self.node)
        self.results, self.status = [], []
        self.node.create_subscription(PlanResult, "/fpga_contract/plan_result", self.results.append, 20)
        self.node.create_subscription(String, "/fpga_contract/status", lambda message: self.status.append(json.loads(message.data)), 10)
        self.odom_pub = self.node.create_publisher(Odometry, "/avoidance/odom", qos_profile_sensor_data)
        self.cloud_pub = self.node.create_publisher(PointCloud2, "/fpga_contract/cloud", qos_profile_sensor_data)
        self.goal_client = self.node.create_client(SetGoal, "/fpga_contract/set_goal")
        self.query_client = self.node.create_client(PlanMotion, "/fpga_contract/plan_motion")
        self.cloud_enabled = True
        self.cloud_frame = "odom"
        self.points = [(8.0, 8.0, 1.0)]
        self.position = np.array((0.0, 0.0, 1.0))
        self.body_velocity = np.array((0.2, 0.0, 0.0))
        self.last_input = 0.0
        self.entered, self.release = threading.Event(), threading.Event()
        self.gate_armed = False
        self.plan_calls = 0
        original = self.backend.planner.plan_executable

        def gated(*args, **kwargs):
            """Delay one real solve so replacement/cancel races are reproducible."""
            self.plan_calls += 1
            if self.gate_armed:
                self.gate_armed = False
                self.entered.set()
                if not self.release.wait(3.0):
                    raise RuntimeError("test did not release native solve")
            return original(*args, **kwargs)

        self.backend.planner.plan_executable = gated

    def spin(self, seconds=0.05):
        """Publish normal sensor traffic while advancing both real ROS nodes."""
        until = time.monotonic() + seconds
        while time.monotonic() < until:
            now = time.monotonic()
            if now - self.last_input >= 0.04:
                stamp = self.node.get_clock().now().to_msg()
                odom = Odometry()
                odom.header = Header(frame_id="odom", stamp=stamp)
                odom.child_frame_id = "base_link"
                odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z = self.position
                odom.pose.pose.orientation.z = math.sin(math.pi / 4.0)
                odom.pose.pose.orientation.w = math.cos(math.pi / 4.0)
                (odom.twist.twist.linear.x, odom.twist.twist.linear.y,
                 odom.twist.twist.linear.z) = self.body_velocity
                self.odom_pub.publish(odom)
                if self.cloud_enabled:
                    header = Header(frame_id=self.cloud_frame, stamp=stamp)
                    self.cloud_pub.publish(point_cloud2.create_cloud_xyz32(header, self.points))
                self.last_input = now
            self.executor.spin_once(timeout_sec=0.01)

    def until(self, predicate, description, seconds=5.0):
        """Wait for one expected observable transition with live sensor input."""
        deadline = time.monotonic() + seconds
        while not predicate() and time.monotonic() < deadline:
            self.spin(0.02)
        if not predicate():
            raise AssertionError(f"timeout: {description}; results={[(r.target_id, r.status, r.detail) for r in self.results[-5:]]}")

    def call(self, client, request):
        """Send a native service request through DDS and wait for its response."""
        self.until(client.service_is_ready, "service discovery")
        future = client.call_async(request)
        self.until(future.done, "service response")
        return future.result()

    def goal(self, identity, xyz=(2.0, 0.5, 1.0), active=True):
        """Set or cancel a persistent native target identity."""
        request = SetGoal.Request()
        request.header.frame_id = "odom"
        request.target_id, request.active = identity, active
        request.goal.x, request.goal.y, request.goal.z = xyz
        return self.call(self.goal_client, request)

    def query(self, goal=(2.0, 0.0, 1.0), *, mode=PlanMotion.Request.CHECK_LINE, segments=(), elapsed=0.0):
        """Check a line or exact cubic without asking the planner for a new route."""
        request = PlanMotion.Request()
        request.header.frame_id, request.mode = "odom", mode
        request.header.stamp = self.node.get_clock().now().to_msg()
        request.start.z = 1.0
        request.goal.x, request.goal.y, request.goal.z = goal
        request.segments = list(segments)
        request.trajectory_elapsed = elapsed
        response = self.call(self.query_client, request)
        assert response.planner_session == self.backend.session == response.result.planner_session
        return response.result

    def arm_gate(self):
        """Pause the next underlying native solve until the test replaces/cancels it."""
        self.entered.clear()
        self.release.clear()
        self.gate_armed = True

    def close(self):
        """Stop only the nodes and software worker created by this test."""
        self.release.set()
        self.backend.close()
        self.backend.worker.shutdown(wait=True, cancel_futures=True)
        self.executor.shutdown()
        self.backend.destroy_node()
        self.node.destroy_node()
        rclpy.shutdown()


def main():
    """Check core goals, query geometry, stale input and honest MOCK provenance."""
    if os.environ.get("ROS_DOMAIN_ID") != "226" or os.environ.get("ROS_AUTOMATIC_DISCOVERY_RANGE") != "LOCALHOST":
        raise RuntimeError("run with ROS_DOMAIN_ID=226 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST")
    test = Contract()
    checks = []
    try:
        test.until(lambda: test.backend.board_ready() and not test.backend.input_reason(), "MOCK/input readiness")
        assert np.allclose(test.backend.odom[1], (0.0, 0.2, 0.0), atol=1.0e-10)
        checks.append("body twist rotated to planning frame")
        start, goal = np.array((0.0, 0.0, 1.0)), np.array((2.0, 0.0, 1.0))
        frame, _, _, _ = test.backend.grid(start, goal, np.empty((0, 3)))
        outside = np.array(((1.0, frame.y_min - 0.01, 1.0),))
        frame, occupied, _, _ = test.backend.grid(start, goal, outside)
        assert xyz_to_cell3d(*outside[0], frame)[1] < 0
        assert occupied[xyz_to_cell3d(1.0, frame.y_min + 0.1, 1.0, frame)]
        original_lower = test.backend.lower.copy()
        try:
            test.backend.lower[0] = 0.05
            frame, occupied, _, _ = test.backend.grid(np.array((0.1, 0.0, 1.0)), goal, np.empty((0, 3)))
            assert occupied[xyz_to_cell3d(0.1, 0.0, 1.0, frame)]
        finally:
            test.backend.lower = original_lower
        checks.append("outside-window obstacles inflated inward and partial boundary cells rejected")
        assert test.query().status == PlanResult.REACH_END
        request = PlanMotion.Request()
        request.header.frame_id, request.mode = "odom", PlanMotion.Request.CHECK_LINE
        request.start.z, request.goal.x, request.goal.z = 1.0, 2.0, 1.0
        assert test.call(test.query_client, request).result.status == PlanResult.INVALID_INPUT
        request.header.stamp = test.node.get_clock().now().to_msg()
        request.header.stamp.sec -= 2
        assert test.call(test.query_client, request).result.status == PlanResult.STALE_INPUT
        request.header.stamp = test.node.get_clock().now().to_msg()
        request.header.stamp.sec += 1
        assert test.call(test.query_client, request).result.status == PlanResult.STALE_INPUT
        for field in ("start", "goal", "velocity"):
            request.header.stamp = test.node.get_clock().now().to_msg()
            geometry = getattr(request, field)
            old = geometry.x
            geometry.x = float("nan")
            assert test.call(test.query_client, request).result.status == PlanResult.INVALID_INPUT
            geometry.x = old
        original_grid = test.backend.grid

        def delayed_grid(*args, **kwargs):
            """Make a service calculation cross its input/request freshness deadline."""
            result = original_grid(*args, **kwargs)
            time.sleep(test.backend.timeout + 0.05)
            return result

        test.backend.grid = delayed_grid
        try:
            assert test.query().status == PlanResult.STALE_INPUT
        finally:
            test.backend.grid = original_grid
        test.spin(0.1)
        checks.append("query timestamp, finite state and return-time freshness checks")
        test.points = [(1.0, 0.0, 1.0)]
        test.spin(0.15)
        assert test.query().status == PlanResult.NO_PATH
        checks.append("CHECK_LINE clear and blocked")

        segment = PolynomialSegment(duration=1.0, x=[0.0, 2.0, 0.0, 0.0],
                                    y=[0.0, 4.0, -4.0, 0.0], z=[1.0, 0.0, 0.0, 0.0])
        test.points = [(1.0, 1.0, 1.0)]
        test.spin(0.15)
        assert test.query().status == PlanResult.REACH_END
        assert test.query(mode=PlanMotion.Request.VALIDATE_TRAJECTORY, segments=[segment]).status == PlanResult.NO_PATH
        assert test.query(mode=PlanMotion.Request.VALIDATE_TRAJECTORY, segments=[segment], elapsed=0.9).status == PlanResult.REACH_END
        checks.append("actual cubic collision and exact remaining-time translation")
        for invalid_elapsed in (-0.1, float("nan")):
            assert test.query(mode=PlanMotion.Request.VALIDATE_TRAJECTORY, segments=[segment], elapsed=invalid_elapsed).status == PlanResult.INVALID_INPUT
        invalid_segment = PolynomialSegment(duration=-1.0, x=list(segment.x), y=list(segment.y), z=list(segment.z))
        assert test.query(mode=PlanMotion.Request.VALIDATE_TRAJECTORY, segments=[invalid_segment]).status == PlanResult.INVALID_INPUT
        checks.append("invalid elapsed time and segment duration rejected")

        test.points = [(8.0, 8.0, 1.0)]
        test.spin(0.15)
        test.arm_gate()
        cancelled = test.goal("cancelled")
        assert cancelled.accepted
        test.until(test.entered.is_set, "cancel test native solve start")
        assert test.goal("cancelled", active=False).accepted
        test.release.set()
        test.spin(0.3)
        assert not any(r.target_id == "cancelled" and r.status in (PlanResult.REACH_END, PlanResult.REACH_HORIZON) for r in test.results)
        checks.append("cancelled in-flight result discarded")

        test.arm_gate()
        old = test.goal("replaced", (2.0, 0.5, 1.0))
        test.until(test.entered.is_set, "replacement native solve start")
        current = test.goal("current", (1.5, 0.5, 1.0))
        assert current.accepted and current.goal_revision > old.goal_revision
        test.release.set()
        test.until(lambda: any(r.target_id == "current" and r.status == PlanResult.REACH_END for r in test.results), "current target executable trajectory")
        assert not any(r.target_id == "replaced" and r.status in (PlanResult.REACH_END, PlanResult.REACH_HORIZON) for r in test.results)
        success = [r for r in test.results if r.target_id == "current" and r.status == PlanResult.REACH_END][-1]
        provenance = json.loads(success.detail)
        assert provenance["source"].startswith("mock:")
        assert success.goal_revision == current.goal_revision and len(success.segments) > 0
        assert success.planner_session == current.planner_session
        duplicate = test.goal("current", (1.5, 0.5, 1.0))
        assert duplicate.accepted and duplicate.goal_revision == current.goal_revision
        stale_cancel = test.goal("replaced", active=False)
        assert not stale_cancel.accepted and test.backend.target[0] == "current"
        first, last = success.segments[0], success.segments[-1]
        assert np.allclose((first.x[1], first.y[1], first.z[1]), (0.0, 0.2, 0.0), atol=1.0e-10)
        end_v = [axis[1] + 2 * axis[2] * last.duration + 3 * axis[3] * last.duration**2 for axis in (last.x, last.y, last.z)]
        assert np.linalg.norm(end_v) < 1.0e-9
        checks.append("replacement identities, idempotent retry and honest MOCK executable result")

        test.goal("current", active=False)
        test.arm_gate()
        test.backend.budget = 0.1
        timed_out = test.goal("timed-out", (2.0, 0.5, 1.0))
        test.until(test.entered.is_set, "timeout native solve start")
        test.until(lambda: any(r.target_id == "timed-out" and r.status == PlanResult.TIMEOUT for r in test.results), "native planning timeout")
        timeout_result = [r for r in test.results if r.target_id == "timed-out" and r.status == PlanResult.TIMEOUT][-1]
        assert not timeout_result.segments and timeout_result.goal_revision == timed_out.goal_revision
        assert test.goal("timed-out", active=False).accepted
        heartbeat_start = len(test.results)
        test.spin(1.05)
        assert any(r.sequence == 0 and not r.target_id and not r.segments for r in test.results[heartbeat_start:])
        test.backend.budget = 2.0
        current = test.goal("after-timeout", (1.5, 0.5, 1.0))
        test.release.set()
        test.until(lambda: any(r.target_id == "after-timeout" and r.status == PlanResult.REACH_END for r in test.results), "new generation after expired worker")
        assert not any(r.target_id == "timed-out" and r.status in (PlanResult.REACH_END, PlanResult.REACH_HORIZON) for r in test.results)
        checks.append("timeout empty result, cancelled identity heartbeat and late worker result discarded")

        # Broken frames and missing inputs must return empty failure outcomes.
        test.cloud_frame = "wrong-frame"
        before = len(test.results)
        test.until(lambda: any(r.target_id == "after-timeout" and r.status == PlanResult.STALE_INPUT for r in test.results[before:]), "invalid cloud-frame failure")
        assert all(not r.segments for r in test.results[before:] if r.status not in (PlanResult.REACH_END, PlanResult.REACH_HORIZON))
        assert test.query().status == PlanResult.STALE_INPUT
        test.cloud_frame = "odom"
        test.spin(0.15)
        test.cloud_enabled = False
        before = len(test.results)
        test.spin(0.75)
        assert any(r.status == PlanResult.STALE_INPUT for r in test.results[before:])
        assert test.query().status == PlanResult.STALE_INPUT
        checks.append("invalid frame and aged cloud clear trajectory outcomes")

        test.cloud_enabled = True
        test.spin(0.15)
        test.goal("after-timeout", active=False)
        horizon_target = test.goal("horizon", (12.0, 0.5, 1.0))
        test.until(lambda: any(r.target_id == "horizon" and r.status == PlanResult.REACH_HORIZON for r in test.results), "window-clipped horizon result")
        horizon = [r for r in test.results if r.target_id == "horizon" and r.status == PlanResult.REACH_HORIZON][-1]
        assert horizon.goal_revision == horizon_target.goal_revision
        assert not any(r.target_id == "horizon" and r.status == PlanResult.REACH_END for r in test.results)
        first = horizon.segments[0]
        t = min(0.2, first.duration / 2.0)
        c = np.array((first.x, first.y, first.z))
        position = c[:, 0] + c[:, 1] * t + c[:, 2] * t**2 + c[:, 3] * t**3
        velocity = c[:, 1] + 2.0 * c[:, 2] * t + 3.0 * c[:, 3] * t**2
        test.position = position
        test.body_velocity = np.array((velocity[1], -velocity[0], velocity[2]))

        def joins_updated_state(result):
            """Identify a later rolling plan that starts from the new measured state."""
            if result.target_id != "horizon" or result.status != PlanResult.REACH_HORIZON or not result.segments:
                return False
            segment = result.segments[0]
            return (np.allclose((segment.x[0], segment.y[0], segment.z[0]), position, atol=1.0e-8) and
                    np.allclose((segment.x[1], segment.y[1], segment.z[1]), velocity, atol=1.0e-8))

        test.until(lambda: any(joins_updated_state(r) for r in test.results), "rolling horizon replans from measured p/v")
        checks.append("local horizon semantics and rolling replan from updated measured position/velocity")
        test.goal("horizon", active=False)
        test.until(lambda: test.backend.future is None, "previous target worker drains")
        test.backend.board_time = time.monotonic()
        original_ping = test.backend.solver.ping
        ping_count = [0]

        def lose_board_after_solve(*args, **kwargs):
            """Inject loss after a real software solve without connecting physical hardware."""
            ping_count[0] += 1
            if ping_count[0] >= 2:
                raise RuntimeError("injected post-solve board loss")
            return original_ping(*args, **kwargs)

        test.backend.solver.ping = lose_board_after_solve
        try:
            test.goal("board-loss", (2.0, 0.5, 1.0))
            test.until(lambda: any(r.target_id == "board-loss" and r.status == PlanResult.NO_PATH for r in test.results), "injected post-solve board loss")
            failure = [r for r in test.results if r.target_id == "board-loss" and r.status == PlanResult.NO_PATH][-1]
            assert ping_count[0] >= 2 and "post-solve board loss" in failure.detail
            assert not failure.segments
            assert not any(r.target_id == "board-loss" and r.status in (PlanResult.REACH_END, PlanResult.REACH_HORIZON) for r in test.results)
        finally:
            test.backend.solver.ping = original_ping
            test.goal("board-loss", active=False)
        checks.append("injected post-solve board loss suppresses successful candidate")

        test.position = np.array((0.0, 0.0, 1.0))
        test.body_velocity = np.array((0.2, 0.0, 0.0))
        test.points = [(8.0, 8.0, 1.0)]
        test.spin(0.1)
        finish_goal = (0.18, 0.03, 1.0)
        finish = test.goal("terminal-hold", finish_goal)
        test.until(lambda: test.backend.valid_curve is not None, "near-goal complete curve accepted")
        held_curve = test.backend.valid_curve
        count = test.plan_calls
        test.spin(max(1.2, sum(held_curve[2]) + 0.1))
        assert test.backend.valid_curve is held_curve and test.plan_calls == count
        assert test.backend.target[:2] == ("terminal-hold", finish.goal_revision)
        assert not any(r.target_id == "terminal-hold" and r.status not in (PlanResult.REACH_END, PlanResult.REACH_HORIZON) for r in test.results)
        checks.append("near-goal finish retains original cubics through terminal time without refitting")

        before = len(test.results)
        test.points = [finish_goal]
        test.until(lambda: any(r.target_id == "terminal-hold" and r.status == PlanResult.NO_PATH for r in test.results[before:]), "blocked held terminal point revoked")
        assert test.backend.valid_curve is None
        test.points = [(8.0, 8.0, 1.0)]
        test.until(lambda: test.backend.valid_curve is not None, "actual-state curve recovery after dynamic obstacle")
        assert test.plan_calls > count
        count = test.plan_calls
        test.cloud_enabled = False
        before = len(test.results)
        test.until(lambda: any(r.target_id == "terminal-hold" and r.status == PlanResult.STALE_INPUT for r in test.results[before:]), "held curve revoked on expired input")
        assert test.backend.valid_curve is None
        test.cloud_enabled = True
        test.until(lambda: test.backend.valid_curve is not None, "actual-state recovery after input returns")
        assert test.plan_calls > count
        count = test.plan_calls
        original_ping = test.backend.solver.ping

        def lose_board_during_hold(*args, **kwargs):
            """Fail a periodic held-curve probe while retaining normal synthetic input."""
            raise RuntimeError("injected terminal-hold board loss")

        before = len(test.results)
        test.backend.solver.ping = lose_board_during_hold
        try:
            test.until(lambda: any(r.target_id == "terminal-hold" and r.status == PlanResult.NO_PATH for r in test.results[before:]), "periodic probe revokes held curve on board loss")
            assert test.backend.valid_curve is None
        finally:
            test.backend.solver.ping = original_ping
        test.until(lambda: test.backend.valid_curve is not None, "actual-state recovery after board probe returns")
        assert test.plan_calls > count
        assert all(not r.segments for r in test.results[before:] if r.status not in (PlanResult.REACH_END, PlanResult.REACH_HORIZON))
        test.goal("terminal-hold", active=False)
        assert test.backend.valid_curve is None
        checks.append("held finish revoked by dynamic blockage, stale input and board loss with actual-state recovery")
        test.spin(1.1)
        assert test.status and all(item["mock"] and not item["hardware_participation"] for item in test.status)
        assert any(r.sequence == 0 and not r.target_id and not r.segments for r in test.results)
        checks.append("idle capability heartbeat and zero claimed hardware participation")
        print(json.dumps({"status": "PASS", "domain": 226, "discovery": "LOCALHOST", "checks": checks,
                          "success_segments": len(success.segments), "success_provenance": provenance,
                          "planning_ms": success.planning_ms, "hardware_tested": False}, indent=2))
    finally:
        test.close()


if __name__ == "__main__":
    main()
