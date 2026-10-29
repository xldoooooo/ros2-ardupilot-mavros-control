"""Optional native-protocol FPGA backend; all FCU output remains in onboard_control."""
from __future__ import annotations

import argparse
from concurrent.futures import ThreadPoolExecutor
from dataclasses import replace
import json
import math
import os
from pathlib import Path
import struct
import sys
import time
import uuid

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, qos_profile_sensor_data
from nav_msgs.msg import Odometry
from path_planning.msg import PlanResult, PolynomialSegment
from path_planning.srv import SetGoal, PlanMotion
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import String
import yaml

# The delivered solver uses sibling imports; only this optional process exposes that directory.
SOLVER_PATH = Path(__file__).resolve().parent / 'fpga_solver'
sys.path.insert(0, str(SOLVER_PATH))
from dyn_ising_adapter import (make_grid_frame3d, pointcloud_to_grid3d,
                              inflate_obstacles_3d, xyz_to_cell3d, kino_kwargs_from_request)
from fpga_uav_solver import FpgaUavSolver
from protocol import UdpSession, CMD_PING
from uav_kino_planner import FpgaKinoPathPlanner, executable_segments_clear


class Transport:
    """Own one board connection and keep firmware caches within this process."""

    def __init__(self, config):
        """Defer the socket until the enabled backend's first explicit probe."""
        self.config = config
        self.session = None
        self._invalidate_fpga_cache()

    def _invalidate_fpga_cache(self):
        """Discard board-side problem identities after a transport failure."""
        self._fpga_problem_crc = self._fpga_j_crc = None
        self._fpga_uav_problem_crc = self._fpga_uav_j_crc = None

    def transact(self, command, payload=b'', timeout=3.0):
        """Bound board loss without blocking the ROS callbacks or CPU planner."""
        if self.session is None:
            self.session = UdpSession(fpga_ip=self.config.get('fpga_ip', '192.168.2.10'),
                                      port=int(self.config.get('udp_port', 4000)),
                                      bind_ip=self.config.get('bind_ip', '0.0.0.0'),
                                      broadcast_fallback=False, native_receive=True)
        try:
            response, elapsed = self.session.transact(command, payload,
                                                      timeout=min(timeout, .6 if command == CMD_PING else 3.0))
            return response, elapsed, 'UDP'
        except Exception:
            self.session.close()
            self.session = None
            self._invalidate_fpga_cache()
            raise

    def _decode_ping_payload(self, payload):
        """Decode the original solver's current runtime identity check."""
        if len(payload) < 24:
            raise ValueError('FPGA PING missing runtime capabilities')
        values = struct.unpack_from('<6I', payload)
        return dict(zip(('status', 'counts', 'build_id', 'version', 'hardware', 'transport_flags'), values))

    def append(self, message):
        """Keep solver diagnostics on the independent backend console."""
        print(message, flush=True)


class FlightBackend(Node):
    """Implement native goal, query and result contracts in an isolated namespace."""

    def __init__(self, config):
        """Subscribe to the original coordinate bridge's FCU-centre odometry."""
        super().__init__('flight_backend')
        self.config = config
        self.frame = self.declare_parameter('planning_frame', 'odom').value
        self.cloud_topic = self.declare_parameter('cloud_topic', '/odin1/cloud_slam_down').value
        self.body_twist = self.declare_parameter('odometry_twist_in_body', True).value
        self.lower = np.asarray(self.declare_parameter('search_lower', [-20., -20., .2]).value)
        self.upper = np.asarray(self.declare_parameter('search_upper', [20., 20., 1.8]).value)
        self.timeout = float(config.get('input_timeout', 1.0))
        self.budget = float(config.get('planning_timeout', 1.0))
        self.near_goal_distance = float(config.get('near_goal_replan_distance', .3))
        self.mock = bool(config.get('mock', False))
        self.limits = [float(config.get(key, 2.0)) for key in
                       ('max_velocity_xy', 'max_velocity_z', 'max_acceleration_xy', 'max_acceleration_z')]
        if not all(math.isfinite(v) and v > 0 for v in [self.timeout, self.budget, self.near_goal_distance, *self.limits]):
            raise ValueError('invalid backend timing or motion limits')
        if self.lower.shape != (3,) or self.upper.shape != (3,) or not np.all(self.lower < self.upper):
            raise ValueError('invalid backend search bounds')
        self.session = uuid.uuid4().hex
        self.target = None
        self.revision = self.sequence = 0
        self.odom = self.cloud = None
        self.odom_time = self.cloud_time = 0.0
        self.input_error = ''
        self.board_time = 0.0
        self.board_error = 'board has not been probed'
        self.source = 'unprobed'
        self.transactions = 0
        self.hardware_transactions = 0
        self.solve_ms = self.cpu_ms = 0.0
        self.last_attempt = 0.0
        self.last_position = None
        self.previous = []
        # Cache only a complete, published curve with its target generation and steady start.
        self.valid_curve = None
        self.worker = ThreadPoolExecutor(max_workers=1, thread_name_prefix='fpga-planner')
        self.future = None
        self.job = None
        self.job_expired = False
        self.transport = Transport(config)
        if self.mock:
            # Explicit offline mode stays visible and cannot be selected by the flight manager.
            from .fpga_kino_planner_node import FpgaKinoPlannerNode
            self.solver = FpgaKinoPlannerNode._make_mock_solver(self)
        else:
            self.solver = FpgaUavSolver(self.transport)
        self.planner = FpgaKinoPathPlanner(self.solver)
        self.result_pub = self.create_publisher(PlanResult, 'plan_result', 10)
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status_pub = self.create_publisher(String, 'status', qos)
        self.preview_pub = self.create_publisher(PointCloud2, 'filtered_cloud', qos_profile_sensor_data)
        self.create_subscription(Odometry, '/avoidance/odom', self.receive_odom, qos_profile_sensor_data)
        self.create_subscription(PointCloud2, self.cloud_topic, self.receive_cloud, qos_profile_sensor_data)
        self.create_service(SetGoal, 'set_goal', self.set_goal)
        self.create_service(PlanMotion, 'plan_motion', self.query)
        self.create_timer(.05, self.tick)
        self.create_timer(1.0, self.heartbeat)

    @staticmethod
    def xyz(point):
        """Convert ROS geometry without changing its world-frame meaning."""
        value = np.array([point.x, point.y, point.z], dtype=float)
        if not np.isfinite(value).all():
            raise ValueError('non-finite geometry')
        return value

    def receive_odom(self, message):
        """Rotate a valid child-frame twist into the common planning frame."""
        try:
            if message.header.frame_id != self.frame:
                raise ValueError('odometry frame mismatch')
            position = self.xyz(message.pose.pose.position)
            velocity = self.xyz(message.twist.twist.linear)
            if self.body_twist:
                q = message.pose.pose.orientation
                quaternion = np.array([q.x, q.y, q.z, q.w])
                norm = np.linalg.norm(quaternion)
                if not math.isfinite(norm) or norm < 1e-6:
                    raise ValueError('invalid odometry orientation')
                quaternion /= norm
                vector, scalar = quaternion[:3], quaternion[3]
                velocity += 2 * np.cross(vector, np.cross(vector, velocity) + scalar * velocity)
            self.odom = (position, velocity, message.header.stamp)
            self.odom_time = time.monotonic()
        except (ValueError, TypeError) as exc:
            self.odom = None
            self.input_error = str(exc)

    def receive_cloud(self, message):
        """Keep complete finite world-frame input without a second installation rotation."""
        try:
            if message.header.frame_id != self.frame:
                raise ValueError('cloud frame mismatch')
            points = point_cloud2.read_points_numpy(message, field_names=('x', 'y', 'z'), skip_nans=True)
            points = np.asarray(points, dtype=np.float64).reshape(-1, 3)
            points = points[np.isfinite(points).all(axis=1)]
            if not len(points):
                raise ValueError('cloud contains no finite map points')
            if len(points) > int(self.config.get('max_cloud_points', 500000)):
                raise ValueError('cloud exceeds backend point cap')
            self.cloud = (points, message.header.stamp)
            self.cloud_time = time.monotonic()
            self.preview_pub.publish(message)
        except (ValueError, TypeError, AssertionError) as exc:
            self.cloud = None
            self.input_error = str(exc)

    def input_reason(self):
        """Use steady reception age like the original receive-clock CPU profile."""
        now = time.monotonic()
        if self.odom is None or self.cloud is None:
            return self.input_error or 'waiting for FCU odometry and world cloud'
        if now - min(self.odom_time, self.cloud_time) > self.timeout:
            return 'stale FCU odometry or cloud'
        return ''

    def board_ready(self):
        """Read the enabled worker's latest probe without doing network I/O here."""
        return not self.board_error and time.monotonic() - self.board_time < 3.0

    def outcome(self, status, detail, target=None, stamp=None):
        """Create an empty atomic outcome with original target and process identities."""
        message = PlanResult()
        message.header.frame_id = self.frame
        message.header.stamp = stamp or self.get_clock().now().to_msg()
        message.planner_session = self.session
        message.target_id = target[0] if target else ''
        message.goal_revision = target[1] if target else self.revision
        self.sequence += 1
        message.sequence = self.sequence
        message.status, message.detail = status, detail
        (message.max_velocity_xy, message.max_velocity_z,
         message.max_acceleration_xy, message.max_acceleration_z) = self.limits
        if self.cloud:
            message.cloud_stamp = self.cloud[1]
            message.map_points = len(self.cloud[0])
        if self.odom:
            message.odometry_stamp = self.odom[2]
        return message

    def set_goal(self, request, response):
        """Invalidate in-flight computation on every accepted replacement or matching cancel."""
        response.planner_session = self.session
        try:
            if request.header.frame_id != self.frame or not request.target_id:
                raise ValueError('goal frame or target identity invalid')
            if request.active:
                goal = self.xyz(request.goal)
                if np.any(goal < self.lower) or np.any(goal > self.upper):
                    raise ValueError('goal outside planning bounds')
                if self.target is not None and self.target[0] == request.target_id and np.array_equal(self.target[2], goal):
                    response.accepted, response.goal_revision = True, self.revision
                    response.detail = 'same active target retained'
                    return response
                self.revision += 1
                self.target = (request.target_id, self.revision, goal)
                self.previous = []
                self.valid_curve = None
                self.last_attempt = 0.0
            elif self.target and self.target[0] != request.target_id:
                raise ValueError('active target identity mismatch')
            elif self.target:
                self.revision += 1
                self.target = None
                self.previous = []
                self.valid_curve = None
            response.accepted = True
            response.detail = 'goal accepted' if request.active else 'matching goal cancelled'
        except ValueError as exc:
            response.accepted, response.detail = False, str(exc)
        response.goal_revision = self.revision
        return response

    def grid(self, start, goal, points, clip=True):
        """Build one uncarved inflated voxel map for search, curves and line queries."""
        resolution = float(self.config.get('resolution_m', .2))
        resolution_z = float(self.config.get('resolution_z_m', resolution))
        frame, effective, clipped = make_grid_frame3d(
            tuple(start), tuple(goal), frame_id=self.frame,
            resolution_m=resolution, resolution_z_m=resolution_z,
            window_margin_m=float(self.config.get('window_margin_m', 2.0)),
            window_radius_m=float(self.config.get('window_radius_m', 8.0)) if clip else 0.,
            max_axis_cells=int(self.config.get('max_axis_cells', 120)),
            z_min_m=float(self.lower[2]), z_max_m=float(self.upper[2]),
            max_z_cells=int(self.config.get('max_z_cells', 40)))
        # Two half voxel diagonals cover input quantisation and candidate-cell extent.
        # The continuous collision checker then treats every occupied cell as a solid box.
        radius = (float(self.config.get('uav_radius_m', .35)) +
                  float(self.config.get('safety_margin_m', 0.0)) +
                  math.sqrt(2 * resolution ** 2 + resolution_z ** 2))
        # Include obstacle points outside the search window before inflation and crop afterwards.
        # Otherwise points just beyond a window edge could disappear from its clearance map.
        pad = int(math.ceil(radius / min(resolution, resolution_z))) + 1
        padded = replace(frame, x_min=frame.x_min-pad*resolution,
                         y_min=frame.y_min-pad*resolution, z_min=frame.z_min-pad*resolution_z,
                         cols=frame.cols+2*pad, rows=frame.rows+2*pad, layers=frame.layers+2*pad)
        raw = pointcloud_to_grid3d(points, padded, assume_finite=True)
        occupied = inflate_obstacles_3d(raw, radius_m=radius,
                                       resolution_m=resolution, resolution_z_m=resolution_z)
        occupied = occupied[pad:-pad, pad:-pad, pad:-pad].astype(np.uint8)
        # Outside configured xyz bounds is unavailable even if the rounded grid extends beyond it.
        xs = frame.x_min + (np.arange(frame.cols) + .5) * resolution
        ys = frame.y_min + (np.arange(frame.rows) + .5) * resolution
        zs = frame.z_min + (np.arange(frame.layers) + .5) * resolution_z
        occupied[:, :, (xs-resolution/2 < self.lower[0]-1e-9) | (xs+resolution/2 > self.upper[0]+1e-9)] = 1
        occupied[:, (ys-resolution/2 < self.lower[1]-1e-9) | (ys+resolution/2 > self.upper[1]+1e-9), :] = 1
        occupied[(zs-resolution_z/2 < self.lower[2]-1e-9) | (zs+resolution_z/2 > self.upper[2]+1e-9), :, :] = 1
        return frame, occupied, np.asarray(effective), clipped

    @staticmethod
    def remaining_curve(coefficients, durations, elapsed):
        """Shift the exact remaining local cubics without refitting or resampling."""
        shifted, remaining = [], []
        for c, duration in zip(coefficients, durations):
            if not math.isfinite(duration) or duration <= 0.:
                raise ValueError('invalid trajectory segment duration')
            if elapsed >= duration:
                elapsed -= duration
                continue
            a, b, d, e = np.asarray(c).T
            shifted.append(np.array([a+b*elapsed+d*elapsed**2+e*elapsed**3,
                                     b+2*d*elapsed+3*e*elapsed**2, d+3*e*elapsed, e]).T)
            remaining.append(duration-elapsed)
            elapsed = 0.
        return shifted, remaining

    def revoke_curve(self, status, detail, target):
        """Replace an unusable success with an empty failure before actual-state recovery."""
        self.valid_curve = None
        self.previous = []
        self.source = 'failure'
        self.transactions = self.hardware_transactions = 0
        self.last_attempt = 0.0
        self.result_pub.publish(self.outcome(status, detail, target))

    def terminal_curve_clear(self, now):
        """Certify the remaining old curve, or only its stationary goal after it expires."""
        target, coefficients, durations, started = self.valid_curve
        shifted, remaining = self.remaining_curve(coefficients, durations, max(0., now-started))
        if not shifted:
            # This is the existing final reference, not a business-arrival decision.
            stationary = np.zeros((3, 4))
            stationary[:, 0] = target[2]
            shifted, remaining = [stationary], [1.0]
        frame, occupied, _, _ = self.grid(self.odom[0], target[2], self.cloud[0], clip=False)
        return executable_segments_clear(shifted, remaining, occupied, frame)

    def query(self, request, response):
        """Check a full line or remaining exact curve against this backend's same map."""
        response.planner_session = self.session
        try:
            if request.header.frame_id != self.frame:
                response.result = self.outcome(PlanResult.FRAME_MISMATCH, 'query frame mismatch')
                return response
            received = time.monotonic()
            stamp = request.header.stamp.sec*1_000_000_000 + request.header.stamp.nanosec
            start, goal = self.xyz(request.start), self.xyz(request.goal)
            self.xyz(request.velocity)
            if stamp <= 0:
                raise ValueError('invalid query timestamp')
            age = (self.get_clock().now().nanoseconds-stamp)/1e9

            def request_fresh():
                """Service requests use ROS timestamps plus steady processing elapsed time."""
                return age >= -.05 and age+time.monotonic()-received <= self.timeout

            if not request_fresh():
                response.result = self.outcome(PlanResult.STALE_INPUT, 'query request expired or future stamped')
                return response
            reason = self.input_reason()
            if reason or not self.board_ready():
                response.result = self.outcome(PlanResult.STALE_INPUT, reason or self.board_error or 'board unavailable')
                return response
            frame, occupied, _, _ = self.grid(start, goal, self.cloud[0], clip=False)
            if request.mode == PlanMotion.Request.CHECK_LINE:
                coefficients = np.zeros((1, 3, 4))
                coefficients[0, :, 0], coefficients[0, :, 1] = start, goal - start
                durations = [1.0]
            elif request.mode == PlanMotion.Request.VALIDATE_TRAJECTORY:
                elapsed = float(request.trajectory_elapsed)
                if not math.isfinite(elapsed) or elapsed < 0. or len(request.segments) > 256:
                    raise ValueError('invalid trajectory elapsed time or segment count')
                coefficients, durations = self.remaining_curve(
                    [np.array([segment.x, segment.y, segment.z]) for segment in request.segments],
                    [segment.duration for segment in request.segments], elapsed)
                if not coefficients:
                    raise ValueError('empty remaining trajectory')
            else:
                raise ValueError('PLAN uses persistent SetGoal, unsupported query mode')
            clear = executable_segments_clear(coefficients, durations, occupied, frame)
            reason = self.input_reason()
            if reason or not self.board_ready() or not request_fresh():
                response.result = self.outcome(PlanResult.STALE_INPUT, reason or 'query inputs, board or request expired during validation')
                return response
            response.result = self.outcome(PlanResult.REACH_END if clear else PlanResult.NO_PATH,
                                           'current inflated voxel map clear' if clear else 'curve intersects occupied/outside voxel')
        except (ValueError, RuntimeError, IndexError) as exc:
            response.result = self.outcome(PlanResult.INVALID_INPUT, str(exc))
        return response

    def solve(self, target, odom, points):
        """Probe real hardware and call the original object's executable trajectory entry."""
        cpu_started = time.process_time()
        deadline = time.monotonic() + self.budget
        self.solver.ping(require_current_runtime=True)
        if target is None:
            return None
        position, velocity, _ = odom
        frame, occupied, effective, clipped = self.grid(position, target[2], points)
        start_cell, goal_cell = xyz_to_cell3d(*position, frame), xyz_to_cell3d(*effective, frame)
        payload = dict(self.config, start_vel_xyz=velocity.tolist(), goal_vel_xyz=[0., 0., 0.],
                       previous_path_xyz=self.previous,
                       max_vel_mps=max(self.limits[:2]), max_acc_mps2=max(self.limits[2:]))
        result = self.planner.plan_executable(
            occupied, start_cell, goal_cell, frame, reaches_goal=not clipped,
            exact_start_xyz=position, exact_goal_xyz=effective,
            max_vel_xy=self.limits[0], max_vel_z=self.limits[1],
            max_acc_xy=self.limits[2], max_acc_z=self.limits[3],
            cancel_callback=lambda: (self.target is None or self.target[:2] != target[:2] or
                                     time.monotonic() > deadline),
            trusted_binary_grid=True, **kino_kwargs_from_request(payload))
        # A geometry/quality fallback cannot conceal a board lost during the solver transaction.
        self.solver.ping(require_current_runtime=True)
        result.details['backend_cpu_ms'] = (time.process_time()-cpu_started)*1000.
        return result

    def tick(self):
        """Drain one worker and publish only outcomes from the still-current target generation."""
        now = time.monotonic()
        target = self.target
        reason = self.input_reason()
        if target is not None and reason:
            if self.valid_curve is not None or now-self.last_attempt > .5:
                self.revoke_curve(PlanResult.STALE_INPUT, reason, target)
                self.last_attempt = now
            if self.future is not None and self.job[0] is not None and self.job[0][:2] == target[:2]:
                self.job_expired = True
        holding = (self.valid_curve is not None and target is not None and
                   self.valid_curve[0][:2] == target[:2] and self.odom is not None and
                   np.linalg.norm(target[2]-self.odom[0]) < self.near_goal_distance)
        if holding:
            try:
                if not self.board_ready():
                    raise RuntimeError(self.board_error or 'board probe expired during terminal hold')
                if not self.terminal_curve_clear(now):
                    raise RuntimeError('remaining terminal curve intersects latest occupied/outside voxel')
                reason = self.input_reason()
                if reason:
                    self.revoke_curve(PlanResult.STALE_INPUT, reason, target)
                    holding = False
            except (ValueError, RuntimeError, IndexError) as exc:
                self.revoke_curve(PlanResult.NO_PATH, str(exc), target)
                holding = False
            if holding and self.future is not None and self.job[0] is not None and self.job[0][:2] == target[:2]:
                # A replan submitted before entering the terminal zone must not
                # replace the already valid final curve when it eventually returns.
                self.job_expired = True
        if self.future is not None:
            target, stamp, started = self.job
            current = (target is None or (self.target is not None and target[:2] == self.target[:2]))
            if target is not None and current and not self.job_expired and now-started > self.budget:
                self.result_pub.publish(self.outcome(PlanResult.TIMEOUT, 'FPGA planning deadline exceeded', target, stamp))
                self.source = 'timeout'
                self.transactions = self.hardware_transactions = 0
                self.valid_curve = None
                self.previous = []
                self.job_expired = True
            if not self.future.done():
                return
            try:
                result = self.future.result()
                self.board_error = ''
                self.board_time = now
                if result is not None and current and not self.job_expired:
                    reason = self.input_reason()
                    if reason:
                        raise RuntimeError(reason)
                    coefficients = result.polynomial_coefficients_xyz
                    durations = result.segment_durations_s
                    # Clouds can arrive during solving; recheck the whole candidate on the latest input.
                    frame, occupied, _, _ = self.grid(self.odom[0], target[2], self.cloud[0])
                    if not executable_segments_clear(coefficients, durations, occupied, frame):
                        raise RuntimeError('candidate invalid against latest cloud')
                    self.source = ('mock:' if self.mock else '') + result.source
                    self.transactions = int(result.details.get('solver_transactions', 0))
                    self.hardware_transactions = int(result.details.get('hardware_transactions', 0))
                    self.solve_ms = (now-started)*1000.
                    self.cpu_ms = float(result.details.get('backend_cpu_ms', 0.0))
                    message = self.outcome(PlanResult.REACH_END if result.reaches_goal else PlanResult.REACH_HORIZON,
                                           json.dumps({'source': self.source, 'transactions': self.transactions,
                                                       'fallback': result.details.get('fallback_reason', '')}), target, stamp)
                    message.planning_ms = (now-started)*1000.
                    message.segments = [PolynomialSegment(duration=float(duration), x=list(c[0]),
                                                           y=list(c[1]), z=list(c[2]))
                                        for c, duration in zip(coefficients, durations)]
                    self.result_pub.publish(message)
                    self.previous = list(result.waypoints_xyz)
                    self.valid_curve = (target, coefficients, durations, started) if result.reaches_goal else None
            except Exception as exc:
                if not self.job_expired and (target is None or current):
                    self.source = 'failure'
                    self.transactions = self.hardware_transactions = 0
                    self.board_error = str(exc)
                    affected = self.target if target is None else target
                    if affected is not None:
                        self.revoke_curve(PlanResult.NO_PATH, str(exc), affected)
            self.future = self.job = None
            self.job_expired = False
        reason = self.input_reason()
        target = self.target
        if target is not None and reason:
            if now-self.last_attempt > .5:
                self.result_pub.publish(self.outcome(PlanResult.STALE_INPUT, reason, target))
                self.last_attempt = now
            return
        holding = (self.valid_curve is not None and target is not None and
                   self.valid_curve[0][:2] == target[:2] and
                   np.linalg.norm(target[2]-self.odom[0]) < self.near_goal_distance)
        if holding:
            if now-self.board_time < 1.0:
                return
            # Keep probing the board without refitting the valid zero-speed finish.
            self.job = (None, self.get_clock().now().to_msg(), now)
            self.future = self.worker.submit(self.solve, None, None, None)
            return
        if target is None:
            if now-self.board_time < 1.0:
                return
        else:
            interval = float(self.config.get('replan_interval', .2))
            distance = float(self.config.get('replan_distance', 1.0))
            moved = self.last_position is None or np.linalg.norm(self.odom[0]-self.last_position) >= distance
            if now-self.last_attempt < interval and not moved:
                return
        self.last_attempt = now
        self.last_position = self.odom[0].copy() if self.odom else None
        stamp = self.get_clock().now().to_msg()
        self.job = (target, stamp, now)
        odom = self.odom
        points = self.cloud[0] if self.cloud else None
        self.future = self.worker.submit(self.solve, target, odom, points)

    def heartbeat(self):
        """Advertise identity and honest solver provenance without publishing flight commands."""
        reason = self.input_reason() or self.board_error
        ready = not reason and self.board_ready()
        data = dict(process_id=os.getpid(), ready=ready, mock=self.mock, source=self.source,
                    error=reason, transactions=self.transactions, planner_session=self.session,
                    hardware_transactions=self.hardware_transactions, solve_ms=self.solve_ms,
                    cpu_ms=self.cpu_ms,
                    hardware_participation=(not self.mock and self.hardware_transactions > 0))
        self.status_pub.publish(String(data=json.dumps(data)))
        message = self.outcome(PlanResult.WAITING_FOR_INPUT, reason or 'backend idle/capable')
        message.sequence = 0
        self.result_pub.publish(message)

    def close(self):
        """Invalidate active work and let the launcher own bounded process termination."""
        self.target = None
        self.valid_curve = None
        self.worker.shutdown(wait=False, cancel_futures=True)
        if self.transport.session:
            self.transport.session.close()


def main():
    """Read only the explicitly enabled module's YAML and keep ROS arguments separate."""
    parser = argparse.ArgumentParser()
    parser.add_argument('--config', type=Path, required=True)
    args, ros_args = parser.parse_known_args()
    config = yaml.safe_load(args.config.read_text(encoding='utf-8'))
    if not isinstance(config, dict):
        raise ValueError('FPGA backend config must be a mapping')
    rclpy.init(args=ros_args)
    node = FlightBackend(config)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
