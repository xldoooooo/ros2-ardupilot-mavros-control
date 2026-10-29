#!/usr/bin/env python3
"""Embedded FPGA kinodynamic planner ROS node (no Windows HTTP bridge).

The whole QUBO solver stack (protocol / qubo_model / fpga_uav_solver /
dyn_ising_adapter / uav_kino_planner, vendored under scripts/fpga_solver/)
runs inside this node, which talks UDP directly to the FPGA.  It subscribes
the same topics as the original dyn_obs_planner A* node and publishes the same
outputs (kino_path, /search_node/trajectory_position, path_size, markers), so
it can replace both the original planner and the old Windows HTTP bridge.

Topics in (defaults):
  /mavros/odometry/out        nav_msgs/Odometry
  /cloud_registered           sensor_msgs/PointCloud2
  /goal                       geometry_msgs/PoseStamped
Topics out (defaults):
  kino_path                   nav_msgs/Path (dense 0.01 s samples)
  /search_node/trajectory_position   nav_msgs/Path (latched, optional)
  /search_node/trajectory_path_size  std_msgs/Int64 (optional)
  /fpga_ising/trajectory_position    nav_msgs/Path (alias for comparison)
  /fpga_ising/service_ms      std_msgs/Float64
  /fpga_ising/metrics         std_msgs/String
  /fpga_ising/path_marker     visualization_msgs/Marker
"""
from __future__ import annotations

import gc
import json
import math
import os
import sys
import threading
import time

import numpy as np  # noqa: F401  (imported before solver modules)

_SOLVER_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "fpga_solver")
if _SOLVER_DIR not in sys.path:
    sys.path.insert(0, _SOLVER_DIR)

from . import rospy_compat as rospy  # noqa: E402
from geometry_msgs.msg import PoseStamped  # noqa: E402
from nav_msgs.msg import Odometry, Path  # noqa: E402
from sensor_msgs_py import point_cloud2  # noqa: E402
from sensor_msgs.msg import PointCloud2, PointField  # noqa: E402
from std_msgs.msg import Float64, Int64, String  # noqa: E402
from visualization_msgs.msg import Marker  # noqa: E402

from dyn_ising_adapter import kino_kwargs_from_request, problem3d_from_request  # noqa: E402
from fpga_uav_solver import FpgaUavSolver  # noqa: E402
from qubo_model import N_HW  # noqa: E402
from uav_kino_planner import FpgaKinoPathPlanner  # noqa: E402
try:
    import native_fastpath as _native_fastpath  # noqa: E402
except Exception:  # pragma: no cover - optional accelerator
    _native_fastpath = None


PLANNER_PROFILE_VERSION = "uav-general-runtime-abi3301"

def _read_proc_cpu_ticks() -> dict[int, tuple[int, int]]:
    """Return per-CPU (busy_ticks,total_ticks) from /proc/stat; empty on failure."""
    out: dict[int, tuple[int, int]] = {}
    try:
        with open("/proc/stat", "r", encoding="utf-8", errors="ignore") as fh:
            for line in fh:
                parts = line.split()
                if not parts or not parts[0].startswith("cpu") or parts[0] == "cpu":
                    continue
                suffix = parts[0][3:]
                if not suffix.isdigit():
                    continue
                vals = [int(x) for x in parts[1:]]
                if len(vals) < 4:
                    continue
                total = sum(vals)
                idle = vals[3] + (vals[4] if len(vals) > 4 else 0)
                out[int(suffix)] = (total - idle, total)
    except Exception:
        return {}
    return out


def _choose_least_busy_cpu(
    allowed: set[int], before: dict[int, tuple[int, int]], after: dict[int, tuple[int, int]]
) -> tuple[int, float]:
    """Choose the least busy allowed CPU. Prefer non-zero CPUs and higher id on ties."""
    candidates = sorted(int(c) for c in allowed if int(c) in before and int(c) in after)
    if len(candidates) > 1 and 0 in candidates:
        candidates.remove(0)
    best_cpu = -1
    best_load = float("inf")
    for cpu in candidates:
        b_busy, b_total = before[cpu]
        a_busy, a_total = after[cpu]
        dt = max(0, a_total - b_total)
        db = max(0, a_busy - b_busy)
        load = (float(db) / float(dt)) if dt > 0 else 1.0
        if (load < best_load - 1.0e-12) or (abs(load - best_load) <= 1.0e-12 and cpu > best_cpu):
            best_cpu = cpu
            best_load = load
    return best_cpu, (best_load if best_cpu >= 0 else float("nan"))



def _param_bool(value: object) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)):
        return bool(value)
    if isinstance(value, str):
        normalized = value.strip().lower()
        if normalized in {"1", "true", "yes", "on", "y"}:
            return True
        if normalized in {"0", "false", "no", "off", "n", ""}:
            return False
    return bool(value)


def _pointcloud_xyz_numpy(
    msg: PointCloud2,
    *,
    stride: int,
    max_points: int,
    center: tuple[float, float, float] | None,
    radius_m: float,
) -> np.ndarray:
    """Extract an N×3 cloud with a zero-Python-loop fast path when possible.

    The bag used by this project stores ordinary little-endian FLOAT32 x/y/z
    fields.  ``sensor_msgs.point_cloud2.read_points`` yields one Python tuple
    per point and can become a noticeable callback cost at 10 Hz.  A structured
    NumPy view reads those fields directly from ``PointCloud2.data``; unusual
    layouts transparently fall back to the ROS helper.
    """
    stride = max(1, int(stride))
    max_points = max(1, int(max_points))
    field_map = {field.name: field for field in msg.fields}
    xyz_fields = [field_map.get(name) for name in ("x", "y", "z")]
    contiguous_rows = int(msg.row_step) == int(msg.point_step) * int(msg.width)
    fast_layout = (
        not bool(msg.is_bigendian)
        and contiguous_rows
        and all(field is not None for field in xyz_fields)
        and all(field.count == 1 for field in xyz_fields if field is not None)
        and all(
            field.datatype in (PointField.FLOAT32, PointField.FLOAT64)
            for field in xyz_fields if field is not None
        )
    )
    if fast_layout:
        formats = [
            "<f4" if field.datatype == PointField.FLOAT32 else "<f8"
            for field in xyz_fields  # type: ignore[union-attr]
        ]
        dtype = np.dtype({
            "names": ["x", "y", "z"],
            "formats": formats,
            "offsets": [int(field.offset) for field in xyz_fields],  # type: ignore[union-attr]
            "itemsize": int(msg.point_step),
        })
        count = int(msg.width) * int(msg.height)
        view = np.frombuffer(msg.data, dtype=dtype, count=count)[::stride]
        xyz = np.column_stack((view["x"], view["y"], view["z"])).astype(np.float64, copy=False)
    else:
        points: list[tuple[float, float, float]] = []
        for index, point in enumerate(point_cloud2.read_points(
                msg, field_names=("x", "y", "z"), skip_nans=True)):
            if index % stride:
                continue
            points.append((float(point[0]), float(point[1]), float(point[2])))
            # Keep a little headroom for the later radius filter.
            if len(points) >= max_points * 4:
                break
        xyz = np.asarray(points, dtype=np.float64)
        if xyz.size == 0:
            return np.empty((0, 3), dtype=np.float64)

    finite = np.all(np.isfinite(xyz), axis=1)
    xyz = xyz[finite]
    if center is not None and xyz.size:
        c = np.asarray(center, dtype=np.float64)
        delta = xyz - c[None, :]
        xyz = xyz[np.einsum("ij,ij->i", delta, delta) <= float(radius_m) ** 2]
    if len(xyz) > max_points:
        # Uniform decimation preserves the full field of view better than
        # taking only the first max_points records from a spinning LiDAR scan.
        indices = np.linspace(0, len(xyz) - 1, max_points, dtype=np.int64)
        xyz = xyz[indices]
    return np.ascontiguousarray(xyz, dtype=np.float64)


class FpgaKinoPlannerNode:
    def __init__(self) -> None:
        # ---- FPGA link ----
        self.fpga_ip = rospy.get_param("~fpga_ip", os.environ.get("FPGA_IP", "192.168.2.10"))
        self.bind_ip = rospy.get_param("~bind_ip", os.environ.get("FPGA_BIND_IP", "192.168.2.100"))
        self.udp_port = int(rospy.get_param("~udp_port", int(os.environ.get("FPGA_PORT", "4000"))))
        # V19: bounded connected-UDP busy-poll budget for WSL2.  Set 0 to
        # disable.  Windows broadcast mode is unaffected because the session
        # only spins on connected unicast sockets.
        self.udp_spin_wait_us = int(rospy.get_param("~udp_spin_wait_us", 4500))
        self.udp_spin_adaptive = _param_bool(rospy.get_param("~udp_spin_adaptive", True))
        self.udp_spin_min_us = int(rospy.get_param("~udp_spin_min_us", 800))
        self.udp_blocking_poll_us = int(rospy.get_param("~udp_blocking_poll_us", 250))
        # Real hardware has a known unicast address.  Connected UDP is required
        # by the ABI-2702 native receive helper; the old hard-coded broadcast
        # fallback prevented connect() and silently disabled that fastpath.
        self.udp_broadcast_fallback = _param_bool(rospy.get_param(
            "~udp_broadcast_fallback",
            os.environ.get("FPGA_BROADCAST_FALLBACK", "0"),
        ))
        # Optional C++17 acceleration for deterministic host kernels.  The
        # September FPGA drop extends ABI 2702 with bulk candidate generation;
        # Python/NumPy remains the exact fallback.
        self.native_cpp_accel = _param_bool(rospy.get_param("~native_cpp_accel", True))
        if _native_fastpath is not None:
            _native_fastpath.set_enabled(self.native_cpp_accel)
        self.candidate_cpp_accel = _param_bool(rospy.get_param("~candidate_cpp_accel", True))
        self.incremental_qubo = _param_bool(rospy.get_param("~incremental_qubo", True))
        self.incremental_qubo_cache_size = max(1, min(128, int(
            rospy.get_param("~incremental_qubo_cache_size", 128)
        )))
        self.mock = _param_bool(rospy.get_param("~mock", False))
        # ---- ROS topics ----
        self.frame_id = rospy.get_param("~frame_id", "camera_init")
        self.cloud_topic = rospy.get_param("~cloud_topic", "/cloud_registered")
        self.odom_topic = rospy.get_param("~odom_topic", "/mavros/odometry/out")
        self.goal_topic = rospy.get_param("~goal_topic", "/goal")
        self.output_path_topic = rospy.get_param("~output_path_topic", "kino_path")
        self.publish_search_node = _param_bool(rospy.get_param("~publish_search_node", False))
        self.comparison_path_topic = rospy.get_param("~comparison_path_topic", "/fpga_ising/trajectory_position")
        # ---- window / cloud / solver knobs (same names as the old bridge) ----
        self.resolution_m = float(rospy.get_param("~resolution_m", 0.2))
        self.resolution_z_m = float(rospy.get_param("~resolution_z_m", 0.2))
        self.window_margin_m = float(rospy.get_param("~window_margin_m", 2.0))
        self.window_radius_m = float(rospy.get_param("~window_radius_m", 8.0))
        self.z_margin_m = float(rospy.get_param("~z_margin_m", 1.2))
        self.max_axis_cells = int(rospy.get_param("~max_axis_cells", 120))
        self.max_z_cells = int(rospy.get_param("~max_z_cells", 24))
        self.uav_radius_m = float(rospy.get_param("~uav_radius_m", 0.45))
        self.safety_margin_m = float(rospy.get_param("~safety_margin_m", 0.05))
        # Manual ceilings.  Adaptive profiles normally use much less
        # (e.g. 40 spins -> 480x2), but keeping the ceilings moderate prevents
        # a large-map fallback from silently returning to the old 2000x16 cost.
        self.iterations = int(rospy.get_param("~iterations", 1400))
        self.restarts = int(rospy.get_param("~restarts", 8))
        self.waypoint_count = int(rospy.get_param("~waypoint_count", 32))
        self.candidates_per_layer = int(rospy.get_param("~candidates_per_layer", 8))
        self.dt_s = float(rospy.get_param("~dt_s", 0.0))
        self.dt_auto = _param_bool(rospy.get_param("~dt_auto", True))
        self.cruise_speed_mps = float(rospy.get_param("~cruise_speed_mps", 1.5))
        self.dt_sample_s = float(rospy.get_param("~dt_sample_s", 0.02))
        self.max_vel_mps = float(rospy.get_param("~max_vel_mps", 2.0))
        self.max_acc_mps2 = float(rospy.get_param("~max_acc_mps2", 2.0))
        self.publish_speed_ratio = min(1.0, max(0.75, float(
            rospy.get_param("~publish_speed_ratio", 0.96)
        )))
        self.qubo_dt_quantum_s = max(0.0, min(0.10, float(
            rospy.get_param("~qubo_dt_quantum_s", 0.02)
        )))
        self.strict_spine_latency_fuse_ms = max(0.0, float(
            rospy.get_param("~strict_spine_latency_fuse_ms", 3.5)
        ))
        self.w_acc = float(rospy.get_param("~w_acc", 2.0))
        self.w_spine = float(rospy.get_param("~w_spine", 0.45))
        self.w_len = float(rospy.get_param("~w_len", 0.35))
        self.a_mult = float(rospy.get_param("~a_mult", 3.0))
        self.safe_clearance_m = float(rospy.get_param("~safe_clearance_m", 0.3))
        # Latency-first adaptive controls, aligned with the final standalone
        # UAV-route solver: short rolling replans use a small Active-N problem
        # and only escalate annealing effort when quality/feasibility requires.
        self.adaptive_problem_size = _param_bool(rospy.get_param("~adaptive_problem_size", True))
        self.adaptive_solver_profile = _param_bool(rospy.get_param("~adaptive_solver_profile", True))
        self.quality_gap_ratio = float(rospy.get_param("~quality_gap_ratio", 0.05))
        self.quality_guard = _param_bool(rospy.get_param("~quality_guard", True))
        self.adaptive_budget = _param_bool(rospy.get_param("~adaptive_budget", True))
        self.fpga_early_restart_stop = _param_bool(rospy.get_param("~fpga_early_restart_stop", True))
        self.reuse_previous_path = _param_bool(rospy.get_param("~reuse_previous_path", True))
        # V12 rolling policy: refresh by elapsed wall time and route state, not
        # by every N replans.  Normal dynamic-obstacle response remains local
        # and per-frame; global A* is a rare topology reset rather than a reflex
        # after every successful micro-bypass.
        self.reuse_refresh_interval = max(0, int(rospy.get_param("~reuse_refresh_interval", 0)))
        self.reuse_refresh_period_s = max(0.0, float(rospy.get_param("~reuse_refresh_period_s", 60.0)))
        self.reuse_refresh_min_interval_s = max(0.0, float(rospy.get_param("~reuse_refresh_min_interval_s", 2.0)))
        self.reuse_refresh_on_bypass = _param_bool(rospy.get_param("~reuse_refresh_on_bypass", True))
        self.reuse_refresh_bypass_count = max(1, int(rospy.get_param("~reuse_refresh_bypass_count", 6)))
        self.reuse_refresh_shortcut_ratio = min(1.0, max(0.0, float(
            rospy.get_param("~reuse_refresh_shortcut_ratio", 0.0)
        )))
        self.shortcut_reused_spine = _param_bool(rospy.get_param("~shortcut_reused_spine", True))
        self.near_goal_direct_m = max(0.0, float(rospy.get_param("~near_goal_direct_m", 4.0)))
        self.near_goal_tiny_m = max(0.0, float(rospy.get_param("~near_goal_tiny_m", 4.0)))
        self.near_goal_direct_cross_speed_mps = max(0.0, float(
            rospy.get_param("~near_goal_direct_cross_speed_mps", 0.8)
        ))
        self.near_goal_direct_endpoint_grace_m = max(0.0, float(
            rospy.get_param("~near_goal_direct_endpoint_grace_m", 0.4)
        ))
        self.near_hysteresis_distance_m = max(0.0, float(
            rospy.get_param("~near_hysteresis_distance_m", 4.5)
        ))
        self.near_hysteresis_detour_ratio = max(1.0, float(
            rospy.get_param("~near_hysteresis_detour_ratio", 1.06)
        ))
        self.near_hysteresis_improvement_ratio = min(0.25, max(0.0, float(
            rospy.get_param("~near_hysteresis_improvement_ratio", 0.02)
        )))
        self.cloud_radius_m = float(rospy.get_param("~cloud_radius_m", 20.0))
        self.cloud_stride = max(1, int(rospy.get_param("~cloud_stride", 2)))
        self.max_cloud_points = max(1, int(rospy.get_param("~max_cloud_points", 6000)))
        self.require_cloud = _param_bool(rospy.get_param("~require_cloud", True))
        self.max_cloud_age_s = max(0.0, float(rospy.get_param("~max_cloud_age_s", 1.0)))
        self.replan_on_cloud = _param_bool(rospy.get_param("~replan_on_cloud", True))
        self.cloud_replan_min_period_s = max(
            0.0, float(rospy.get_param("~cloud_replan_min_period_s", 0.10))
        )
        # Do not perform heavy PointCloud2 -> NumPy filtering
        # concurrently with the solver critical path.  If a cloud arrives while
        # solving, retain only the newest message and process it immediately
        # after the current solve.  The current solve already uses a snapshot,
        # so intermediate clouds cannot affect its result and were already
        # coalesced by the pending-replan policy.
        self.cloud_defer_during_solve = _param_bool(rospy.get_param("~cloud_defer_during_solve", True))
        self.skip_unchanged_replan = _param_bool(rospy.get_param("~skip_unchanged_replan", True))
        # ---- rolling replan ----
        self.replan_period_s = float(rospy.get_param("~replan_period_s", 0.0))
        self.replan_distance_m = float(rospy.get_param("~replan_distance_m", 0.25))
        # Host scheduling guard: reuse one daemon solver thread rather
        # than creating a new Python thread for every rolling replan.  The GC
        # guard only disables cyclic collection while a solve batch is active;
        # reference counting remains unchanged and GC is restored immediately
        # afterwards.  CPU affinity is intentionally opt-in because Jetson CPU
        # topology differs by SKU and power mode.
        self.solver_persistent_worker = _param_bool(rospy.get_param("~solver_persistent_worker", True))
        self.solver_gc_guard = _param_bool(rospy.get_param("~solver_gc_guard", True))
        self.solver_preflight_ping = _param_bool(rospy.get_param("~solver_preflight_ping", True))
        self.clearance_cache_policy = str(rospy.get_param("~clearance_cache_policy", "versioned")).strip().lower()
        if self.clearance_cache_policy not in {"auto", "live", "versioned"}:
            rospy.logwarn("unknown clearance_cache_policy=%s; using versioned", self.clearance_cache_policy)
            self.clearance_cache_policy = "versioned"
        self.solver_cpu_affinity = int(rospy.get_param(
            "~solver_cpu_affinity", os.environ.get("FPGA_SOLVER_CPU", "-1")
        ))

        self._lock = threading.Lock()
        self._latest_odom: Odometry | None = None
        self._latest_goal: PoseStamped | None = None
        self._latest_points = np.empty((0, 3), dtype=np.float64)
        self._cloud_generation = 0
        self._cloud_received_monotonic = 0.0
        self._last_cloud_schedule_monotonic = 0.0
        self._deferred_cloud = None
        self._cloud_deferred_count = 0
        self._cloud_coalesced_count = 0
        self._last_cloud_processing_ms = 0.0
        self._last_cloud_processing_deferred = False
        self._last_solve_cloud_deferred = 0
        self._last_solve_cloud_coalesced = 0
        self._last_solve_cloud_pending = False
        self._solver_affinity_auto = bool(self.solver_cpu_affinity == -2)
        self._solver_affinity_selected_cpu = -1
        self._solver_affinity_selected_load = float("nan")
        self._solver_affinity_sample_ms = 0.0
        self._last_solved_cloud_generation = -1
        self._last_solved_goal_signature: tuple[float, float, float, str] | None = None
        self._solve_seq = 0
        self._solving = False
        self._pending = False
        self._solve_event = threading.Event()
        self._solver_thread: threading.Thread | None = None
        self._solver_thread_native_id = 0
        self._solver_affinity_applied = False
        self._solver_affinity_error = ""
        self._last_solver_gc_guard_applied = False
        self._solver_preflight_done = False
        self._solver_preflight_ok = False
        self._solver_preflight_ms = 0.0
        self._solver_preflight_error = ""
        self._solver_host_warmup_done = False
        self._solver_host_warmup_ms = 0.0
        self._last_solve_pose: tuple[float, float, float] | None = None
        self._previous_path_xyz: list[list[float]] = []
        self._reuse_streak = 0
        self._bypass_since_refresh = 0
        self._last_global_refresh_monotonic = 0.0
        self._refresh_requested = False
        self._session = None  # created lazily on first FPGA transact
        self._fpga_problem_crc = None
        self._fpga_j_crc = None
        self._fpga_uav_problem_crc = None
        self._fpga_uav_j_crc = None

        # ---- solver stack (embedded) ----
        self._solver = self._make_mock_solver() if self.mock else FpgaUavSolver(
            self,
            adaptive_budget=self.adaptive_budget,
            early_restart_stop=self.fpga_early_restart_stop,
        )
        self._planner = FpgaKinoPathPlanner(self._solver, clearance_cache_policy=self.clearance_cache_policy)
        if self.native_cpp_accel and _native_fastpath is not None:
            native_status = _native_fastpath.status()
            if native_status.get("available"):
                rospy.loginfo(
                    "FPGA native C++ fastpath enabled (ABI=%s)",
                    native_status.get("version"),
                )
            else:
                rospy.logwarn(
                    "native_cpp_accel requested but native library is unavailable; "
                    "falling back to the V23 Python/NumPy kernels (%s)",
                    native_status.get("error"),
                )

        # ---- publishers ----
        self.path_pub = rospy.Publisher(self.output_path_topic, Path, queue_size=1, latch=True)
        self.search_pub = (
            rospy.Publisher("/search_node/trajectory_position", Path, queue_size=1, latch=True)
            if self.publish_search_node
            else None
        )
        self.size_pub = rospy.Publisher("/fpga_ising/trajectory_path_size", Int64, queue_size=1, latch=True)
        self.search_size_pub = (
            rospy.Publisher("/search_node/trajectory_path_size", Int64, queue_size=1, latch=True)
            if self.publish_search_node
            else None
        )
        self.comparison_pub = rospy.Publisher(self.comparison_path_topic, Path, queue_size=1, latch=True)
        self.marker_pub = rospy.Publisher("/fpga_ising/path_marker", Marker, queue_size=1, latch=True)
        self.metrics_pub = rospy.Publisher("/fpga_ising/service_ms", Float64, queue_size=1, latch=True)
        self.metrics_details_pub = rospy.Publisher("/fpga_ising/metrics", String, queue_size=1, latch=True)
        # V15: planner failures are observable as structured diagnostics.  The
        # compare node records them as fpga_error rows without contaminating
        # normal A*/FPGA path statistics.
        self.error_metrics_pub = rospy.Publisher("/fpga_ising/error_metrics", String, queue_size=10)

        # ---- subscribers ----
        rospy.Subscriber(self.odom_topic, Odometry, self._odom_cb, queue_size=1)
        rospy.Subscriber(self.cloud_topic, PointCloud2, self._cloud_cb, queue_size=1)
        rospy.Subscriber(self.goal_topic, PoseStamped, self._goal_cb, queue_size=1)
        if self.replan_period_s > 0.0:
            rospy.Timer(rospy.Duration(self.replan_period_s), self._periodic_replan_cb)
        if self.solver_persistent_worker:
            self._solver_thread = threading.Thread(
                target=self._persistent_solver_loop,
                name="fpga-kino-solver",
                daemon=True,
            )
            self._solver_thread.start()
        rospy.loginfo(
            "FPGA kinodynamic planner node: mode=%s fpga=%s:%d bind=%s udp_spin=%dus goal=%s cloud=%s odom=%s "
            "waypoints_cap=%d cand_cap=%d a_mult=%.1f adaptive_size=%s adaptive_profile=%s "
            "adaptive_budget=%s early_restart_stop=%s reuse_path=%s quality_guard=%s "
            "replan_period=%ss cloud_trigger=%s cloud_defer=%s persistent_worker=%s gc_guard=%s preflight_ping=%s solver_cpu=%d",
            "MOCK" if self.mock else "REAL", self.fpga_ip, self.udp_port, self.bind_ip or "(auto)",
            self.udp_spin_wait_us, self.goal_topic, self.cloud_topic, self.odom_topic,
            self.waypoint_count, self.candidates_per_layer, self.a_mult,
            self.adaptive_problem_size, self.adaptive_solver_profile, self.adaptive_budget,
            self.fpga_early_restart_stop, self.reuse_previous_path, self.quality_guard,
            self.replan_period_s, self.replan_on_cloud, self.cloud_defer_during_solve, self.solver_persistent_worker,
            self.solver_gc_guard, self.solver_preflight_ping, self.solver_cpu_affinity,
        )

    # ---------- FPGA transactor (host for FpgaUavSolver) ----------
    def log(self, message: str) -> None:
        rospy.loginfo(message)

    def append(self, message: str) -> None:
        rospy.loginfo(message)

    def transact(self, cmd: int, payload: bytes = b"", timeout: float = 3.0):
        from protocol import UdpSession
        if self._session is None:
            self._session = UdpSession(
                fpga_ip=self.fpga_ip, port=self.udp_port,
                bind_ip=self.bind_ip or "0.0.0.0",
                broadcast_fallback=self.udp_broadcast_fallback,
                spin_wait_us=self.udp_spin_wait_us,
                adaptive_spin=self.udp_spin_adaptive,
                spin_wait_min_us=self.udp_spin_min_us,
                blocking_poll_us=self.udp_blocking_poll_us,
                # ABI 2702 contains a native connected-UDP receive helper.
                # The previous ROS2 wrapper enabled the native kernels but
                # never requested this receive path, so the advertised UDP
                # fastpath was silently unused on real hardware.
                native_receive=self.native_cpp_accel,
            )
            rospy.loginfo(
                "FPGA UDP session ready: profile=%s connected_unicast=%s broadcast_fallback=%s "
                "native_requested=%s native_applied=%s native_abi=%s",
                PLANNER_PROFILE_VERSION,
                bool(getattr(self._session, "_connected_udp", False)),
                bool(self.udp_broadcast_fallback),
                bool(getattr(self._session, "native_receive_requested", False)),
                bool(getattr(self._session, "native_receive_applied", False)),
                int(getattr(self._session, "native_receive_version", 0)),
            )
        response, milliseconds = self._session.transact(cmd, payload, timeout=timeout)
        return response, milliseconds, "UDP"

    def _decode_ping_payload(self, payload: bytes) -> dict[str, int]:
        if len(payload) < 20:
            raise ValueError("PING返回不足20字节")
        import struct
        status, counts, build_id, version, hardware = struct.unpack_from("<IIIII", payload)
        transport_flags = struct.unpack_from("<I", payload, 20)[0] if len(payload) >= 24 else 0
        return {"status": status, "counts": counts, "build_id": build_id,
                "version": version, "hardware": hardware,
                "transport_flags": transport_flags}

    def _invalidate_fpga_cache(self) -> None:
        # Both generic and UAV-compact solver paths cache payload fingerprints.
        # A PS/PL reset invalidates board state even though host CRCs survive;
        # clearing all four values lets the retry path resend the full problem.
        self._fpga_problem_crc = None
        self._fpga_j_crc = None
        self._fpga_uav_problem_crc = None
        self._fpga_uav_j_crc = None

    # ---------- mock solver ----------
    def _make_mock_solver(self):
        from optimizer_bridge import solve_local_sa
        from qubo_model import qubo_value
        from fpga_uav_solver import FpgaBatchResult, FpgaRuntimeStatus, FpgaSolveResult

        class MockSolver:
            def ping(self, require_current_runtime=True):
                return FpgaRuntimeStatus(status=1, h_writes=0, j_commits=0, build_id=0x44325257,
                                         version=0x04010351, node_count=N_HW, serial_rtt_ms=0.0,
                                         transport="MOCK")

            def solve_runs(self, problem, iterations, restarts=1, seed=None, warm_starts=()):
                # Mirror the real firmware: one call performs the whole
                # multi-start batch and returns the global best solution.
                requested = max(1, int(restarts))
                base = int(seed or 0x12802026) & 0xFFFFFFFF
                started = time.perf_counter()
                raw = solve_local_sa(problem, int(iterations), requested, base)
                spins = tuple(int(x) for x in raw.spins)
                binary = tuple(1 if x > 0 else 0 for x in spins)
                elapsed_us = int((time.perf_counter() - started) * 1_000_000)
                run = FpgaSolveResult(
                    status=4, energy_raw=0, energy=float(qubo_value(problem, spins) or 0.0),
                    node_count=N_HW, iterations=int(iterations), elapsed_us=elapsed_us,
                    spins=spins, binary=binary, packed_spins=0,
                    serial_rtt_ms=elapsed_us / 1000.0, restarts=requested,
                    best_restart=int(getattr(raw, "best_restart", 0) or 0), transport="MOCK",
                    load_mode="MOCK-BATCH", profile_pl_solve_us=elapsed_us,
                    issued_iteration_budget=int(iterations) * requested)
                return FpgaBatchResult(
                    FpgaRuntimeStatus(status=1, h_writes=0, j_commits=0, build_id=0x44325257,
                                      version=0x04010351, node_count=N_HW, serial_rtt_ms=0.0,
                                      transport="MOCK"),
                    (run,))
        return MockSolver()

    # ---------- callbacks ----------
    def _odom_cb(self, msg: Odometry) -> None:
        with self._lock:
            self._latest_odom = msg
            moved = False
            if (self.replan_distance_m > 0.0 and self._last_solve_pose is not None
                    and not self._solving and not self._pending):
                p = msg.pose.pose.position
                dx = p.x - self._last_solve_pose[0]
                dy = p.y - self._last_solve_pose[1]
                dz = p.z - self._last_solve_pose[2]
                moved = dx * dx + dy * dy + dz * dz >= self.replan_distance_m * self.replan_distance_m
        if moved:
            self._schedule_solve()

    def _periodic_replan_cb(self, _event: object) -> None:
        self._schedule_solve()

    def _process_cloud_msg(
        self,
        msg: PointCloud2,
        center: tuple[float, float, float] | None,
        received_monotonic: float,
        *,
        deferred: bool,
        schedule: bool,
    ) -> bool:
        processing_started = time.perf_counter()
        points = _pointcloud_xyz_numpy(
            msg,
            stride=self.cloud_stride,
            max_points=self.max_cloud_points,
            center=center,
            radius_m=self.cloud_radius_m,
        )
        now = time.monotonic()
        processing_ms = (time.perf_counter() - processing_started) * 1000.0
        with self._lock:
            self._latest_points = points
            self._cloud_generation += 1
            self._cloud_received_monotonic = float(received_monotonic)
            self._last_cloud_processing_ms = float(processing_ms)
            self._last_cloud_processing_deferred = bool(deferred)
            ready = self._latest_goal is not None and self._latest_odom is not None
            should_trigger = (
                self.replan_on_cloud
                and ready
                and now - self._last_cloud_schedule_monotonic >= self.cloud_replan_min_period_s
            )
            if should_trigger:
                self._last_cloud_schedule_monotonic = now
        if should_trigger and schedule:
            self._schedule_solve()
        return bool(should_trigger)

    def _cloud_cb(self, msg: PointCloud2) -> None:
        received_monotonic = time.monotonic()
        with self._lock:
            odom = self._latest_odom
            center = None
            if odom is not None:
                center = (
                    odom.pose.pose.position.x,
                    odom.pose.pose.position.y,
                    odom.pose.pose.position.z,
                )
            if self.cloud_defer_during_solve and self._solving:
                if self._deferred_cloud is not None:
                    self._cloud_coalesced_count += 1
                self._deferred_cloud = (msg, center, received_monotonic)
                self._cloud_deferred_count += 1
                return
        self._process_cloud_msg(
            msg, center, received_monotonic, deferred=False, schedule=True
        )

    def _goal_cb(self, msg: PoseStamped) -> None:
        with self._lock:
            old = self._latest_goal
            if old is not None:
                a = old.pose.position
                b = msg.pose.position
                if math.dist((a.x, a.y, a.z), (b.x, b.y, b.z)) > max(self.resolution_m, self.resolution_z_m):
                    self._previous_path_xyz = []
            self._latest_goal = msg
        self._schedule_solve()

    def _schedule_solve(self) -> None:
        with self._lock:
            # V16: startup readiness gate.  Cloud callbacks arrive before the
            # latched demo goal in the benchmark bag; scheduling a worker in
            # that window only produces expected ``no goal`` error rows.  Wait
            # until all mandatory state is present before starting a planner
            # thread.  This does not delay the first real solve because the goal
            # callback calls _schedule_solve() again as soon as it arrives.
            if self._latest_goal is None or self._latest_odom is None:
                return
            if self.require_cloud and self._cloud_generation <= 0:
                return
            if self.skip_unchanged_replan and not self._solving and not self._pending:
                goal = self._latest_goal
                odom = self._latest_odom
                if goal is not None and odom is not None and self._last_solve_pose is not None:
                    p = odom.pose.pose.position
                    moved2 = (
                        (p.x - self._last_solve_pose[0]) ** 2
                        + (p.y - self._last_solve_pose[1]) ** 2
                        + (p.z - self._last_solve_pose[2]) ** 2
                    )
                    goal_sig = (
                        float(goal.pose.position.x), float(goal.pose.position.y),
                        float(goal.pose.position.z), goal.header.frame_id or self.frame_id,
                    )
                    cloud_changed = self._cloud_generation != self._last_solved_cloud_generation
                    goal_changed = goal_sig != self._last_solved_goal_signature
                    move_gate = max(0.05, min(self.replan_distance_m, 0.20))
                    if not cloud_changed and not goal_changed and moved2 < move_gate * move_gate:
                        return
            if self._solving:
                self._pending = True
                return
            self._solving = True
        if self.solver_persistent_worker:
            self._solve_event.set()
        else:
            threading.Thread(
                target=self._solver_worker_entry,
                name="fpga-kino-solver",
                daemon=True,
            ).start()

    def _apply_solver_thread_affinity(self) -> None:
        self._solver_thread_native_id = int(threading.get_native_id())
        self._solver_affinity_applied = False
        self._solver_affinity_error = ""
        self._solver_affinity_selected_cpu = -1
        self._solver_affinity_selected_load = float("nan")
        self._solver_affinity_sample_ms = 0.0
        cpu = int(self.solver_cpu_affinity)
        if cpu == -1:
            return
        try:
            if not hasattr(os, "sched_getaffinity") or not hasattr(os, "sched_setaffinity"):
                raise RuntimeError("sched_getaffinity/sched_setaffinity is unavailable on this platform")
            allowed = set(int(c) for c in os.sched_getaffinity(0))
            if not allowed:
                raise RuntimeError("empty allowed CPU affinity set")
            if cpu == -2:
                sample_started = time.perf_counter()
                before = _read_proc_cpu_ticks()
                time.sleep(0.080)
                after = _read_proc_cpu_ticks()
                self._solver_affinity_sample_ms = (time.perf_counter() - sample_started) * 1000.0
                cpu, load = _choose_least_busy_cpu(allowed, before, after)
                if cpu < 0:
                    # Safe deterministic fallback: highest allowed non-zero CPU.
                    ordered = sorted(allowed)
                    nonzero = [c for c in ordered if c != 0]
                    cpu = (nonzero[-1] if nonzero else ordered[-1])
                self._solver_affinity_selected_load = float(load)
            elif cpu < -2:
                return
            if cpu not in allowed:
                raise ValueError(f"CPU {cpu} is not in allowed affinity set {sorted(allowed)}")
            os.sched_setaffinity(self._solver_thread_native_id, {cpu})
            self._solver_affinity_selected_cpu = int(cpu)
            self._solver_affinity_applied = True
            if self._solver_affinity_auto:
                rospy.loginfo(
                    "FPGA solver worker auto-pinned to CPU %d (tid=%d, sample=%.1f ms, load=%.3f)",
                    cpu, self._solver_thread_native_id, self._solver_affinity_sample_ms,
                    self._solver_affinity_selected_load if math.isfinite(self._solver_affinity_selected_load) else -1.0,
                )
            else:
                rospy.loginfo("FPGA solver worker pinned to CPU %d (tid=%d)", cpu, self._solver_thread_native_id)
        except Exception as exc:  # noqa: BLE001
            self._solver_affinity_error = str(exc)
            rospy.logwarn("FPGA solver CPU affinity not applied: %s", exc)

    def _solver_worker_entry(self) -> None:
        self._apply_solver_thread_affinity()
        self._solve_worker()

    def _persistent_solver_loop(self) -> None:
        self._apply_solver_thread_affinity()
        # Preflight the real FPGA runtime/session before the
        # first benchmarked solve. FpgaUavSolver.solve_runs() performs a PING
        # lazily when _last_runtime is None, and that one-time socket/native
        # receive initialization used to appear as a large first-solve
        # "unaccounted" gap because it happens before host_started.
        if self.solver_preflight_ping and not self.mock:
            preflight_started = time.perf_counter()
            try:
                if self.solver_gc_guard and gc.isenabled():
                    gc.collect()
                self._solver.ping(require_current_runtime=True)
                warm_started = time.perf_counter()
                warm = getattr(self._solver, "preflight_host_path", None)
                if callable(warm):
                    warm()
                self._solver_host_warmup_ms = (time.perf_counter() - warm_started) * 1000.0
                self._solver_host_warmup_done = True
                self._solver_preflight_ok = True
                rospy.loginfo(
                    "FPGA solver preflight ready: %.3f ms (host warmup %.3f ms, tid=%d)",
                    (time.perf_counter() - preflight_started) * 1000.0,
                    self._solver_host_warmup_ms,
                    int(self._solver_thread_native_id),
                )
            except Exception as exc:  # noqa: BLE001
                self._solver_preflight_error = str(exc)
                rospy.logwarn("FPGA solver preflight PING failed; first solve will retry: %s", exc)
            finally:
                self._solver_preflight_ms = (time.perf_counter() - preflight_started) * 1000.0
                self._solver_preflight_done = True
        else:
            self._solver_preflight_done = True
            self._solver_preflight_ok = bool(self.mock or not self.solver_preflight_ping)
        while not rospy.is_shutdown():
            if not self._solve_event.wait(timeout=0.25):
                continue
            self._solve_event.clear()
            self._solve_worker()

    def _snapshot(self):
        with self._lock:
            odom = self._latest_odom
            goal = self._latest_goal
            # Cloud arrays are immutable after assignment; keeping the reference
            # avoids copying thousands of nested Python lists on every replan.
            points = self._latest_points
            cloud_generation = self._cloud_generation
            cloud_received = self._cloud_received_monotonic
        if odom is None:
            raise RuntimeError("no odometry has been received")
        if goal is None:
            raise RuntimeError("no goal has been received")
        if self.require_cloud and cloud_generation <= 0:
            raise RuntimeError("no point cloud has been received")
        if self.max_cloud_age_s > 0.0 and cloud_received > 0.0:
            age = time.monotonic() - cloud_received
            if age > self.max_cloud_age_s:
                raise RuntimeError(f"point cloud is stale ({age:.3f}s > {self.max_cloud_age_s:.3f}s)")
        return odom, goal, points, cloud_generation

    # ---------- solve ----------
    def _solve_worker(self) -> None:
        try:
            while not rospy.is_shutdown():
                with self._lock:
                    self._pending = False
                    solve_cloud_deferred_start = int(self._cloud_deferred_count)
                    solve_cloud_coalesced_start = int(self._cloud_coalesced_count)
                gc_was_enabled = bool(gc.isenabled())
                gc_guard_applied = bool(self.solver_gc_guard and gc_was_enabled)
                self._last_solver_gc_guard_applied = gc_guard_applied
                if gc_guard_applied:
                    gc.disable()
                try:
                    odom, goal, points, cloud_generation = self._snapshot()
                    with self._lock:
                        self._solve_seq = (int(self._solve_seq) + 1) & 0xFFFFFFFF
                        solve_seq = int(self._solve_seq)
                    request_started = time.perf_counter()
                    payload = self._build_payload(odom, goal, points)
                    problem = problem3d_from_request(payload)
                    kwargs = kino_kwargs_from_request(payload)
                    planner_started = time.perf_counter()
                    preprocess_ms = (planner_started - request_started) * 1000.0
                    # Provenance-based clearance cache identity for exact reuse without full-grid CRC.
                    # A matching token means the same processed cloud snapshot
                    # and the same discretisation geometry/start/goal were used,
                    # so exact cache reuse needs no whole-grid CRC scan.
                    clearance_cache_token = (
                        int(cloud_generation),
                        tuple(int(v) for v in problem.grid3d.shape),
                        tuple(int(v) for v in problem.start),
                        tuple(int(v) for v in problem.goal),
                        round(float(problem.frame.x_min), 9),
                        round(float(problem.frame.y_min), 9),
                        round(float(problem.frame.z_min), 9),
                        round(float(problem.frame.resolution_m), 9),
                        round(float(problem.frame.resolution_z_m), 9),
                        round(float(self.uav_radius_m), 9),
                        round(float(self.safety_margin_m), 9),
                    )
                    result = self._planner.plan(
                        problem.grid3d, problem.start, problem.goal, problem.frame,
                        raw_grid=problem.raw_grid3d,
                        trusted_binary_grid=True,
                        clearance_cache_token=clearance_cache_token,
                        exact_start_xyz=payload.get("start_xyz"),
                        exact_goal_xyz=(problem.metadata or {}).get("effective_goal_xyz", payload.get("goal_xyz")),
                        **kwargs,
                    )
                    planner_ms = (time.perf_counter() - planner_started) * 1000.0
                    service_ms = (time.perf_counter() - request_started) * 1000.0
                    with self._lock:
                        self._last_solve_cloud_deferred = max(0, int(self._cloud_deferred_count) - solve_cloud_deferred_start)
                        self._last_solve_cloud_coalesced = max(0, int(self._cloud_coalesced_count) - solve_cloud_coalesced_start)
                        self._last_solve_cloud_pending = self._deferred_cloud is not None
                    self._publish_result(result, service_ms, goal, preprocess_ms, planner_ms, solve_seq)
                    with self._lock:
                        p = odom.pose.pose.position
                        self._last_solve_pose = (p.x, p.y, p.z)
                        self._last_solved_cloud_generation = cloud_generation
                        self._last_solved_goal_signature = (
                            float(goal.pose.position.x), float(goal.pose.position.y),
                            float(goal.pose.position.z), goal.header.frame_id or self.frame_id,
                        )
                except Exception as exc:  # noqa: BLE001
                    rospy.logerr("FPGA kinodynamic solve failed: %s", exc)
                    try:
                        elapsed_ms = (
                            (time.perf_counter() - request_started) * 1000.0
                            if "request_started" in locals() else float("nan")
                        )
                        direct_distance = None
                        if "odom" in locals() and "goal" in locals() and odom is not None and goal is not None:
                            p = odom.pose.pose.position
                            g = goal.pose.position
                            direct_distance = math.dist((p.x, p.y, p.z), (g.x, g.y, g.z))
                        err = String()
                        err.data = json.dumps({
                            "source": "error",
                            "solve_seq": int(locals().get("solve_seq", 0) or 0),
                            "service_ms": round(float(elapsed_ms), 3) if math.isfinite(float(elapsed_ms)) else None,
                            "planner_error_type": type(exc).__name__,
                            "planner_error_text": str(exc),
                            "cloud_generation": int(locals().get("cloud_generation", -1) or -1),
                            "direct_distance_m": direct_distance,
                            "planner_profile_version": PLANNER_PROFILE_VERSION,
                        }, ensure_ascii=False)
                        self.error_metrics_pub.publish(err)
                    except Exception:
                        pass
                finally:
                    if gc_guard_applied:
                        gc.enable()

                # Serialize heavy PointCloud processing after the solve.
                # Keep _solving=True while draining one latest-only deferred
                # cloud so callbacks arriving during this short step are also
                # coalesced instead of racing the next solver batch.
                deferred_cloud = None
                if self.cloud_defer_during_solve:
                    with self._lock:
                        deferred_cloud = self._deferred_cloud
                        self._deferred_cloud = None
                if deferred_cloud is not None:
                    msg, center, received_monotonic = deferred_cloud
                    try:
                        deferred_trigger = self._process_cloud_msg(
                            msg, center, received_monotonic, deferred=True, schedule=False
                        )
                        if deferred_trigger:
                            with self._lock:
                                self._pending = True
                    except Exception as exc:  # noqa: BLE001
                        rospy.logwarn_throttle(2.0, "deferred cloud processing failed: %s", exc)

                with self._lock:
                    if self._pending:
                        continue
                    self._solving = False
                    return
        finally:
            with self._lock:
                self._solving = False

    def _should_force_spine_refresh(self) -> bool:
        if not self.reuse_previous_path or not self._previous_path_xyz:
            return False
        now = time.monotonic()
        age = (
            now - self._last_global_refresh_monotonic
            if self._last_global_refresh_monotonic > 0.0 else float("inf")
        )
        by_count = self.reuse_refresh_interval > 0 and self._reuse_streak >= self.reuse_refresh_interval
        by_time = self.reuse_refresh_period_s > 0.0 and age >= self.reuse_refresh_period_s
        # V7: state-triggered refreshes obey a short cooldown.  V6 set a refresh
        # request whenever string-pulling shortened a dense previous trajectory.
        # That condition was true on almost every reuse frame and produced an
        # A*/reuse ping-pong (~50% fresh A*), increasing both latency and route
        # length.  Time-triggered refresh still remains authoritative.
        request_ready = self._refresh_requested and age >= self.reuse_refresh_min_interval_s
        return bool(by_count or by_time or request_ready)

    def _build_payload(self, odom: Odometry, goal: PoseStamped, points: np.ndarray) -> dict:
        start = odom.pose.pose.position
        twist = odom.twist.twist.linear
        target = goal.pose.position
        return {
            "request_id": "kino-%.3f" % time.time(),
            "planning_mode": "kinodynamic",
            "frame_id": goal.header.frame_id or self.frame_id,
            "start_xyz": [start.x, start.y, start.z],
            "goal_xyz": [target.x, target.y, target.z],
            "start_vel_xyz": [twist.x, twist.y, twist.z],
            "start_acc_xyz": [0.0, 0.0, 0.0],
            "goal_vel_xyz": [0.0, 0.0, 0.0],
            "points_xyz": points,
            # The PointCloud2 callback already removes non-finite points and
            # stores a contiguous float64 array.  Mark that contract so the
            # embedded adapter can skip two redundant O(N) finite scans.
            "_points_validated_numpy": True,
            "native_cpp_accel": bool(self.native_cpp_accel),
            "candidate_cpp_accel": bool(self.candidate_cpp_accel),
            "incremental_qubo": bool(self.incremental_qubo),
            "incremental_qubo_cache_size": int(self.incremental_qubo_cache_size),
            "resolution_m": self.resolution_m,
            "resolution_z_m": self.resolution_z_m,
            "window_margin_m": self.window_margin_m,
            "window_radius_m": self.window_radius_m,
            "z_margin_m": self.z_margin_m,
            "max_axis_cells": self.max_axis_cells,
            "max_z_cells": self.max_z_cells,
            "uav_radius_m": self.uav_radius_m,
            "safety_margin_m": self.safety_margin_m,
            "iterations": self.iterations,
            "restarts": self.restarts,
            "waypoint_count": self.waypoint_count,
            "candidates_per_layer": self.candidates_per_layer,
            "dt_s": self.dt_s,
            "dt_auto": self.dt_auto,
            "cruise_speed_mps": self.cruise_speed_mps,
            "dt_sample_s": self.dt_sample_s,
            "max_vel_mps": self.max_vel_mps,
            "max_acc_mps2": self.max_acc_mps2,
            "publish_speed_ratio": self.publish_speed_ratio,
            "qubo_dt_quantum_s": self.qubo_dt_quantum_s,
            "strict_spine_latency_fuse_ms": self.strict_spine_latency_fuse_ms,
            "w_acc": self.w_acc,
            "w_spine": self.w_spine,
            "w_len": self.w_len,
            "a_mult": self.a_mult,
            "safe_clearance_m": self.safe_clearance_m,
            "adaptive_problem_size": self.adaptive_problem_size,
            "adaptive_solver_profile": self.adaptive_solver_profile,
            "quality_gap_ratio": self.quality_gap_ratio,
            "quality_guard": self.quality_guard,
            "previous_path_xyz": (list(self._previous_path_xyz) if self.reuse_previous_path else []),
            "force_spine_refresh": bool(self._should_force_spine_refresh()),
            "shortcut_reused_spine": bool(self.shortcut_reused_spine),
            "near_goal_direct_m": float(self.near_goal_direct_m),
            "near_goal_tiny_m": float(self.near_goal_tiny_m),
            "near_goal_direct_cross_speed_mps": float(self.near_goal_direct_cross_speed_mps),
            "near_goal_direct_endpoint_grace_m": float(self.near_goal_direct_endpoint_grace_m),
            "near_hysteresis_distance_m": float(self.near_hysteresis_distance_m),
            "near_hysteresis_detour_ratio": float(self.near_hysteresis_detour_ratio),
            "near_hysteresis_improvement_ratio": float(self.near_hysteresis_improvement_ratio),
        }

    def _publish_result(
        self, result, service_ms: float, goal: PoseStamped,
        preprocess_ms: float = 0.0, planner_ms: float = 0.0,
        solve_seq: int = 0,
    ) -> None:
        path = Path()
        path.header.stamp = rospy.Time.now()
        path.header.frame_id = goal.header.frame_id or self.frame_id
        for xyz in result.path_xyz:
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = float(xyz[0])
            pose.pose.position.y = float(xyz[1])
            pose.pose.position.z = float(xyz[2])
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)
        terminal_goal_error_m = None
        terminal_goal_anchored = False
        if path.poses:
            end = path.poses[-1].pose.position
            g = goal.pose.position
            terminal_goal_error_m = math.dist((end.x, end.y, end.z), (g.x, g.y, g.z))
            terminal_goal_anchored = terminal_goal_error_m <= 1.0e-6
        # Publish timing/diagnostic data *before* the path.  The comparison
        # node samples service_ms when a path callback arrives; the old order
        # occasionally associated a new path with the previous solve latency.
        service_msg = Float64()
        service_msg.data = float(service_ms)
        self.metrics_pub.publish(service_msg)
        detail_msg = String()
        detail_msg.data = json.dumps({
            "source": result.source,
            "solve_seq": int(solve_seq),
            "terminal_goal_error_m": terminal_goal_error_m,
            "terminal_goal_anchored": bool(terminal_goal_anchored),
            "exact_terminal_goal": bool(result.details.get("exact_terminal_goal", False)),
            "fallback_exact_endpoint_anchored": bool(
                result.details.get("fallback_exact_endpoint_anchored", terminal_goal_anchored)
            ),
            "fallback_exact_start_anchored": bool(
                result.details.get("fallback_exact_start_anchored", terminal_goal_anchored)
            ),
            "fallback_exact_goal_anchored": bool(
                result.details.get("fallback_exact_goal_anchored", terminal_goal_anchored)
            ),
            # ROS 2 removed Header.seq.  The path timestamp is a transport-only
            # correlation key; solve_seq remains the original diagnostic ID.
            "path_stamp_ns": rospy.stamp_to_nanoseconds(path.header.stamp),
            "service_ms": round(service_ms, 3),
            "preprocess_ms": round(preprocess_ms, 3),
            "planner_ms": round(planner_ms, 3),
            "internal_planning_ms": round(float(result.planning_elapsed_ms), 3),
            "fpga_ms": result.fpga_elapsed_ms,
            "variables": result.variables,
            "fpga_starts": result.fpga_starts,
            "valid_starts": result.valid_starts,
            "strict_safe": result.details.get("strict_safe"),
            "size_profile": result.details.get("size_profile"),
            "single_candidate_layers": result.details.get("single_candidate_layers", 0),
            "spine_source": result.details.get("spine_source"),
            "solver_profiles": result.details.get("solver_profiles"),
            "avg_off_layers": result.details.get("avg_off_layers"),
            "fallback_reason": result.details.get("fallback_reason"),
            "inflated_blocked_segments": result.details.get("inflated_blocked_segments"),
            "retry_inflated_gate_required": result.details.get("retry_inflated_gate_required"),
            "max_vel_mps": result.max_vel_mps,
            "max_acc_mps2": result.max_acc_mps2,
            "dense_fd_max_acc_mps2": result.details.get("dense_fd_max_acc_mps2"),
            "board_load_decode_ms": result.details.get("board_load_decode_ms"),
            "board_load_pl_ms": result.details.get("board_load_pl_ms"),
            "board_load_total_ms": result.details.get("board_load_total_ms"),
            "board_transport_rx_ms": result.details.get("board_transport_rx_ms"),
            "board_pl_solve_ms": result.details.get("board_pl_solve_ms"),
            "board_result_read_ms": result.details.get("board_result_read_ms"),
            "board_restart_overhead_ms": result.details.get("board_restart_overhead_ms"),
            "issued_iteration_budget": result.details.get("issued_iteration_budget"),
            "reduced_budgets": result.details.get("reduced_budgets"),
            "early_restart_stopped": result.details.get("early_restart_stopped"),
            "quality_guard": result.details.get("quality_guard"),
            "quality_tolerance_m": result.details.get("quality_tolerance_m"),
            "spine_geom_len_m": result.details.get("spine_geom_len_m"),
            "raw_fpga_geom_len_m": result.details.get("raw_fpga_geom_len_m"),
            "pre_shortcut_quality_gap": result.details.get("pre_shortcut_quality_gap", False),
            "quality_guard_rescued": result.details.get("quality_guard_rescued", False),
            "selected_geom_len_m": result.details.get("selected_geom_len_m"),
            "published_waypoint_geom_len_m": result.details.get("published_waypoint_geom_len_m"),
            "quality_recovery_used": result.details.get("quality_recovery_used", False),
            "quality_recovery_ms": result.details.get("quality_recovery_ms", 0.0),
            "quality_recovery_geom_len_m": result.details.get("quality_recovery_geom_len_m"),
            "post_shortcut_applied": result.details.get("post_shortcut_applied", False),
            "post_shortcut_ratio": result.details.get("post_shortcut_ratio", 1.0),
            "post_shortcut_ms": result.details.get("post_shortcut_ms", 0.0),
            "spine_turn_ratio": result.details.get("spine_turn_ratio"),
            "spine_source": result.details.get("spine_source"),
            "force_spine_refresh": result.details.get("force_spine_refresh"),
            "spine_shortcut_applied": result.details.get("spine_shortcut_applied"),
            "spine_shortcut_ratio": result.details.get("spine_shortcut_ratio"),
            "spine_nodes_before_shortcut": result.details.get("spine_nodes_before_shortcut"),
            "near_hysteresis_refresh": result.details.get("near_hysteresis_refresh", False),
            "reuse_streak": int(self._reuse_streak),
            "reuse_refresh_period_s": float(self.reuse_refresh_period_s),
            "reuse_refresh_min_interval_s": float(self.reuse_refresh_min_interval_s),
            "reuse_refresh_bypass_count": int(self.reuse_refresh_bypass_count),
            "bypass_since_refresh": int(self._bypass_since_refresh),
            "refresh_requested": bool(self._refresh_requested),
            "near_goal_direct": result.details.get("near_goal_direct", False),
            "near_goal_direct_safety_mode": result.details.get("near_goal_direct_safety_mode"),
            "near_goal_direct_cross_speed_actual_mps": result.details.get("near_goal_direct_cross_speed_actual_mps"),
            "direct_distance_m": result.details.get("direct_distance_m"),
            "near_hysteresis_distance_m": result.details.get("near_hysteresis_distance_m", self.near_hysteresis_distance_m),
            "near_hysteresis_detour_ratio": result.details.get("near_hysteresis_detour_ratio", self.near_hysteresis_detour_ratio),
            "near_hysteresis_improvement_ratio": result.details.get("near_hysteresis_improvement_ratio", self.near_hysteresis_improvement_ratio),
            "retime_expansions": result.details.get("retime_expansions", 0),
            "w_len": result.details.get("w_len", self.w_len),
            "clearance_ms": result.details.get("clearance_ms"),
            "pre_clearance_setup_ms": result.details.get("pre_clearance_setup_ms"),
            "clearance_compute_ms": result.details.get("clearance_compute_ms"),
            "clearance_crc_ms": result.details.get("clearance_crc_ms", 0.0),
            "clearance_kernel_ms": result.details.get("clearance_kernel_ms", 0.0),
            "grid_validation_ms": result.details.get("grid_validation_ms", 0.0),
            "raw_grid_validation_ms": result.details.get("raw_grid_validation_ms", 0.0),
            "grid_validation_copy_avoided": result.details.get("grid_validation_copy_avoided", False),
            "raw_grid_validation_copy_avoided": result.details.get("raw_grid_validation_copy_avoided", False),
            "trusted_binary_grid": result.details.get("trusted_binary_grid", False),
            "clearance_cache_probe": result.details.get("clearance_cache_probe", False),
            "clearance_cache_bypassed": result.details.get("clearance_cache_bypassed", False),
            "clearance_cache_bypass_remaining": result.details.get("clearance_cache_bypass_remaining", 0),
            "clearance_cache_miss_streak": result.details.get("clearance_cache_miss_streak", 0),
            "clearance_cache_permanent_bypass": result.details.get("clearance_cache_permanent_bypass", False),
            "clearance_cache_identity_mode": result.details.get("clearance_cache_identity_mode", "unknown"),
            "clearance_cache_token_present": result.details.get("clearance_cache_token_present", False),
            "clearance_cache_token_hit": result.details.get("clearance_cache_token_hit", False),
            "spine_ms": result.details.get("spine_ms"),
            "candidate_ms": result.details.get("candidate_ms"),
            "segment_ms": result.details.get("segment_ms"),
            "qubo_ms": result.details.get("qubo_ms"),
            "solve_host_ms": result.details.get("solve_host_ms"),
            "retime_ms": result.details.get("retime_ms"),
            "dense_collision_ms": result.details.get("dense_collision_ms"),
            "host_payload_ms": result.details.get("host_payload_ms"),
            "host_fingerprint_ms": result.details.get("host_fingerprint_ms"),
            "host_payload_prepared_bytes": result.details.get("host_payload_prepared_bytes"),
            "host_payload_cache_fastpath": result.details.get("host_payload_cache_fastpath"),
            "host_payload_build_modes": result.details.get("host_payload_build_modes"),
            "native_wire_payload": result.details.get("native_wire_payload", False),
            "native_wire_payload_version": result.details.get("native_wire_payload_version", 0),
            "host_transport_wall_ms": result.details.get("host_transport_wall_ms"),
            "host_decode_ms": result.details.get("host_decode_ms"),
            # V19 transport diagnostics.  Only meaningful when this plan
            # actually issued an FPGA transaction; direct mode leaves them
            # empty rather than reusing the previous session values.
            "udp_spin_wait_us": self.udp_spin_wait_us,
            "udp_spin_budget_us": (
                float(getattr(self._session, "last_spin_budget_us", self.udp_spin_wait_us))
                if result.details.get("solver_transactions", 0) else 0.0
            ),
            "udp_spin_adaptive": bool(self.udp_spin_adaptive),
            "udp_blocking_poll_us": int(self.udp_blocking_poll_us),
            "udp_broadcast_fallback": bool(self.udp_broadcast_fallback),
            "udp_connected_unicast": bool(
                getattr(self._session, "_connected_udp", False)
            ) if result.details.get("solver_transactions", 0) else False,
            "udp_spin_hit": (
                bool(getattr(self._session, "last_spin_hit", False))
                if result.details.get("solver_transactions", 0) else False
            ),
            "udp_spin_elapsed_us": (
                float(getattr(self._session, "last_spin_elapsed_us", 0.0))
                if result.details.get("solver_transactions", 0) else 0.0
            ),
            "udp_receive_phase": (
                str(getattr(self._session, "last_receive_phase", "none"))
                if result.details.get("solver_transactions", 0) else "none"
            ),
            "host_solver_total_ms": result.details.get("host_solver_total_ms"),
            "solve_decode_score_ms": result.details.get("solve_decode_score_ms", 0.0),
            "solve_host_unaccounted_ms": result.details.get("solve_host_unaccounted_ms", 0.0),
            "host_payload_bytes": result.details.get("host_payload_bytes"),
            "host_load_modes": result.details.get("host_load_modes"),
            "solver_transactions": result.details.get("solver_transactions"),
            "solver_profiles": result.details.get("solver_profiles"),
            "solve_dt_s": result.details.get("solve_dt_s"),
            "dt_unquantized_s": result.details.get("dt_unquantized_s"),
            "qubo_dt_quantum_s": result.details.get("qubo_dt_quantum_s", self.qubo_dt_quantum_s),
            "publish_speed_ratio": result.details.get("publish_speed_ratio", self.publish_speed_ratio),
            "strict_spine_latency_fuse_ms": result.details.get(
                "spine_latency_fuse_ms", self.strict_spine_latency_fuse_ms
            ),
            "spine_latency_fuse_triggered": bool(
                result.details.get("spine_latency_fuse_triggered", False)
            ),
            "trajectory_dt_s": result.details.get("trajectory_dt_s"),
            "attempts": result.attempts,
            "native_cpp_accel": bool(result.details.get("native_cpp_accel", False)),
            "native_cpp_requested": bool(self.native_cpp_accel),
            "candidate_native_cpp": bool(result.details.get("candidate_native_cpp", False)),
            "candidate_cpp_requested": bool(self.candidate_cpp_accel),
            "incremental_qubo": bool(result.details.get("incremental_qubo", self.incremental_qubo)),
            "native_qubo": bool(result.details.get("native_qubo", False)),
            "native_qubo_version": int(result.details.get("native_qubo_version", 0) or 0),
            "incremental_qubo_static_hit": bool(result.details.get("incremental_qubo_static_hit", False)),
            "incremental_qubo_static_build_ms": result.details.get("incremental_qubo_static_build_ms", 0.0),
            "incremental_qubo_cache_entries": result.details.get("incremental_qubo_cache_entries", 0),
            "incremental_qubo_cache_size": int(self.incremental_qubo_cache_size),
            "clearance_cache_hit": bool(result.details.get("clearance_cache_hit", False)),
            "native_clearance": bool(result.details.get("native_clearance", False)),
            "native_chain_collision": bool(result.details.get("native_chain_collision", False)),
            "native_spine_segments": bool(result.details.get("native_spine_segments", False)),
            "native_retime_dense": bool(result.details.get("native_retime_dense", False)),
            "native_clearance_cap2": bool(result.details.get("native_clearance_cap2", False)),
            "native_cpp_version": (
                int(_native_fastpath.status().get("version", 0))
                if _native_fastpath is not None and self.native_cpp_accel else 0
            ),
            "udp_native_receive_requested": bool(
                getattr(self._session, "native_receive_requested", False)
            ) if result.details.get("solver_transactions", 0) else False,
            "udp_native_receive_applied": bool(
                getattr(self._session, "native_receive_applied", False)
            ) if result.details.get("solver_transactions", 0) else False,
            "udp_native_receive_version": int(
                getattr(self._session, "native_receive_version", 0)
            ) if result.details.get("solver_transactions", 0) else 0,
            "udp_native_receive_fallbacks": int(
                getattr(self._session, "native_receive_fallbacks", 0)
            ) if result.details.get("solver_transactions", 0) else 0,
            "solver_persistent_worker": bool(self.solver_persistent_worker),
            "solver_gc_guard": bool(self.solver_gc_guard),
            "solver_gc_guard_applied": bool(self._last_solver_gc_guard_applied),
            "solver_preflight_ping": bool(self.solver_preflight_ping),
            "solver_preflight_done": bool(self._solver_preflight_done),
            "solver_preflight_ok": bool(self._solver_preflight_ok),
            "solver_preflight_ms": float(self._solver_preflight_ms),
            "solver_preflight_error": str(self._solver_preflight_error),
            "solver_host_warmup_done": bool(self._solver_host_warmup_done),
            "solver_host_warmup_ms": float(self._solver_host_warmup_ms),
            "clearance_cache_policy": str(self.clearance_cache_policy),
            "solver_thread_native_id": int(self._solver_thread_native_id),
            "solver_cpu_affinity": int(self.solver_cpu_affinity),
            "solver_affinity_auto": bool(self._solver_affinity_auto),
            "solver_affinity_selected_cpu": int(self._solver_affinity_selected_cpu),
            "solver_affinity_selected_load": float(self._solver_affinity_selected_load),
            "solver_affinity_sample_ms": float(self._solver_affinity_sample_ms),
            "solver_affinity_applied": bool(self._solver_affinity_applied),
            "solver_affinity_error": str(self._solver_affinity_error),
            "cloud_defer_during_solve": bool(self.cloud_defer_during_solve),
            "cloud_deferred_count": int(self._cloud_deferred_count),
            "cloud_coalesced_count": int(self._cloud_coalesced_count),
            "cloud_deferred_during_solve": int(self._last_solve_cloud_deferred),
            "cloud_coalesced_during_solve": int(self._last_solve_cloud_coalesced),
            "cloud_pending_at_publish": bool(self._last_solve_cloud_pending),
            "cloud_processing_ms": float(self._last_cloud_processing_ms),
            "cloud_processing_deferred": bool(self._last_cloud_processing_deferred),
            "planner_profile_version": PLANNER_PROFILE_VERSION,
            "replan_on_cloud": bool(self.replan_on_cloud),
            "cloud_replan_min_period_s": float(self.cloud_replan_min_period_s),
            "replan_period_s": float(self.replan_period_s),
            "replan_distance_m": float(self.replan_distance_m),
            "dt_s": result.dt_s,
            "dt_sample_s": result.dt_sample_s,
        })
        self.metrics_details_pub.publish(detail_msg)

        self.path_pub.publish(path)
        self.comparison_pub.publish(path)
        if self.search_pub is not None:
            self.search_pub.publish(path)
        size_msg = Int64()
        size_msg.data = int(result.path_size)
        self.size_pub.publish(size_msg)
        if self.search_size_pub is not None:
            self.search_size_pub.publish(size_msg)
        self.marker_pub.publish(self._path_marker(path))
        if self.reuse_previous_path:
            with self._lock:
                # V7 reuses the sparse selected waypoints instead of the dense
                # 50-Hz trajectory.  Reusing dense Hermite samples made the V6
                # shortcut ratio artificially small even after a fresh A*, which
                # repeatedly requested another global refresh on the next frame.
                reuse_geometry = result.waypoints_xyz if len(result.waypoints_xyz) >= 2 else result.path_xyz
                self._previous_path_xyz = [[float(x), float(y), float(z)] for x, y, z in reuse_geometry]
                spine_src = str(result.details.get("spine_source") or "")
                shortcut_ratio = float(result.details.get("spine_shortcut_ratio", 1.0) or 1.0)
                if spine_src in ("previous-safe", "previous-raw", "micro-bypass", "micro-bypass-raw"):
                    self._reuse_streak += 1
                    if spine_src in ("micro-bypass", "micro-bypass-raw"):
                        self._bypass_since_refresh += 1
                        # V12: a single local bypass already repairs the current
                        # dynamic obstacle and should not automatically force an
                        # expensive global A* on the next frame.  Request a full
                        # refresh only after several bypasses in the same route
                        # epoch, which preserves topology recovery without the
                        # V11 bypass->fresh-A* latency spikes.
                        if (
                            self.reuse_refresh_on_bypass
                            and self._bypass_since_refresh >= self.reuse_refresh_bypass_count
                        ):
                            self._refresh_requested = True
                    # Disabled by default. If explicitly enabled,
                    # require several reuse frames before a shortcut-quality
                    # trigger can request a global A* refresh.
                    if (
                        self.reuse_refresh_shortcut_ratio > 0.0
                        and self._reuse_streak >= 3
                        and shortcut_ratio < self.reuse_refresh_shortcut_ratio
                    ):
                        self._refresh_requested = True
                else:
                    self._reuse_streak = 0
                    # Fresh A*/near-direct results establish a new route epoch.
                    self._last_global_refresh_monotonic = time.monotonic()
                    self._refresh_requested = False
                    self._bypass_since_refresh = 0
        # V9: keep normal operation logs readable without formatting/flushing
        # one INFO line for every ~3--10 Hz trajectory publication.
        rospy.loginfo_throttle(
            2.0,
            "Kinodynamic path published: nodes=%d path_size=%d source=%s service_ms=%.1f fpga_ms=%.1f "
            "vars=%d profile=%s max_v=%.2f max_a=%.2f",
            len(path.poses), result.path_size, result.source, service_ms,
            result.fpga_elapsed_ms, result.variables, result.details.get("size_profile"),
            result.max_vel_mps, result.max_acc_mps2,
        )

    @staticmethod
    def _path_marker(path: Path) -> Marker:
        marker = Marker()
        marker.header = path.header
        marker.ns = "fpga_ising_3d_path"
        marker.id = 1
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.08
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 0.65
        marker.color.b = 0.95
        marker.points = [pose.pose.position for pose in path.poses]
        return marker


def main() -> None:
    rospy.init_node("fpga_kino_planner_node")
    FpgaKinoPlannerNode()
    rospy.spin()


if __name__ == "__main__":
    main()
