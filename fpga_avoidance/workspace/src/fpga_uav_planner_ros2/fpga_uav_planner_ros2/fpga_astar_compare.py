#!/usr/bin/env python3
"""FPGA (kinodynamic QUBO) vs original kinodynamic A* path comparison node."""
from __future__ import annotations

import csv
import json
import math
import os
import statistics
import time

import numpy as np
from . import rospy_compat as rospy
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Odometry, Path
from sensor_msgs_py import point_cloud2
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float64, String
from visualization_msgs.msg import Marker

GOAL_TOL_M = 0.5


class FpgaAstarCompare:
    def __init__(self) -> None:
        self.astar_topic = rospy.get_param("~astar_path_topic", "/search_node/trajectory_position")
        self.fpga_topic = rospy.get_param("~fpga_path_topic", "/fpga_ising/trajectory_position")
        self.cloud_topic = rospy.get_param("~cloud_topic", "/cloud_registered")
        self.odom_topic = rospy.get_param("~odom_topic", "/mavros/odometry/out")
        self.goal_topic = rospy.get_param("~goal_topic", "/goal")
        self.safe_dist_m = float(rospy.get_param("~safe_dist_m", 0.45))
        self.output_csv = rospy.get_param("~output_csv", os.path.expanduser("~/fpga_vs_astar.csv"))
        self.summary_csv = rospy.get_param("~summary_csv", os.path.expanduser("~/fpga_vs_astar_summary.csv"))
        self.summary_period_s = float(rospy.get_param("~summary_period_s", 10.0))
        self.append_csv = bool(rospy.get_param("~append_csv", False))
        self.max_cloud_points = int(rospy.get_param("~max_cloud_points", 2000))
        self.cloud_stride = max(1, int(rospy.get_param("~cloud_stride", 3)))
        self.astar_dt_sample_s = max(1.0e-4, float(rospy.get_param("~astar_dt_sample_s", 0.01)))
        self.fpga_dt_sample_s = max(1.0e-4, float(rospy.get_param("~fpga_dt_sample_s", 0.02)))
        self.bag_rate = float(rospy.get_param("~bag_rate", float("nan")))
        self.metrics_wait_ms = max(0.0, float(rospy.get_param("~metrics_wait_ms", 20.0)))

        self._cloud: list[tuple[float, float, float]] = []
        self._goal: PoseStamped | None = None
        self._astar_stats: dict[str, object] = {}
        self._fpga_stats: dict[str, object] = {}
        self._rows: list[list[object]] = []
        self._last_seen: dict[str, float] = {}
        self._fpga_service_ms = float("nan")
        self._astar_service_ms = float("nan")
        self._astar_search_ms = float("nan")
        self._fpga_metrics: dict[str, object] = {}
        self._fpga_metrics_by_seq: dict[int, dict[str, object]] = {}
        self._fpga_metrics_by_stamp: dict[int, dict[str, object]] = {}
        self._csv_handle = None
        self._csv_writer = None
        self._csv_header: list[str] = []

        self.marker_pub_astar = rospy.Publisher("/fpga_astar_compare/astar_marker", Marker, queue_size=1, latch=True)
        self.marker_pub_fpga = rospy.Publisher("/fpga_astar_compare/fpga_marker", Marker, queue_size=1, latch=True)
        self.marker_pub_astar_halo = rospy.Publisher("/fpga_astar_compare/astar_halo", Marker, queue_size=1, latch=True)
        self.marker_pub_astar_points = rospy.Publisher("/fpga_astar_compare/astar_points", Marker, queue_size=1, latch=True)
        self.marker_pub_fpga_points = rospy.Publisher("/fpga_astar_compare/fpga_points", Marker, queue_size=1, latch=True)
        self.marker_pub_difference = rospy.Publisher("/fpga_astar_compare/difference_links", Marker, queue_size=1, latch=True)
        self.marker_pub_environment = rospy.Publisher("/fpga_astar_compare/environment_marker", Marker, queue_size=1, latch=True)
        self.marker_pub_start = rospy.Publisher("/fpga_astar_compare/start_marker", Marker, queue_size=1, latch=True)
        self.marker_pub_goal = rospy.Publisher("/fpga_astar_compare/goal_marker", Marker, queue_size=1, latch=True)
        self.marker_pub_text = rospy.Publisher("/fpga_astar_compare/comparison_text", Marker, queue_size=1, latch=True)
        self._latest_path_points: dict[str, list[Point]] = {"astar": [], "fpga": []}
        self._path_deviation_mean_m = float("nan")
        self._path_deviation_max_m = float("nan")

        rospy.Subscriber(self.astar_topic, Path, lambda msg: self._path_cb("astar", msg), queue_size=1)
        rospy.Subscriber(self.fpga_topic, Path, lambda msg: self._path_cb("fpga", msg), queue_size=1)
        rospy.Subscriber(self.cloud_topic, PointCloud2, self._cloud_cb, queue_size=1)
        rospy.Subscriber(self.goal_topic, PoseStamped, self._goal_cb, queue_size=1)
        rospy.Subscriber("/fpga_ising/service_ms", Float64, self._service_ms_cb, queue_size=1)
        rospy.Subscriber("/search_node/service_ms", Float64, self._astar_service_ms_cb, queue_size=1)
        rospy.Subscriber("/search_node/search_ms", Float64, self._astar_search_ms_cb, queue_size=1)
        rospy.Subscriber("/fpga_ising/metrics", String, self._metrics_cb, queue_size=1)
        rospy.Subscriber("/fpga_ising/error_metrics", String, self._error_metrics_cb, queue_size=10)

        rospy.Timer(rospy.Duration(2.0), self._summary_cb)
        rospy.Timer(rospy.Duration(self.summary_period_s), self._periodic_stats_cb)
        rospy.on_shutdown(self._write_summary)
        rospy.loginfo("fpga_astar_compare: astar=%s fpga=%s cloud=%s csv=%s summary=%s",
                      self.astar_topic, self.fpga_topic, self.cloud_topic, self.output_csv, self.summary_csv)

    # ---------- helpers ----------
    def _ensure_csv(self) -> None:
        if self._csv_writer is not None:
            return
        first = (not self.append_csv) or (not os.path.exists(self.output_csv))
        self._csv_handle = open(self.output_csv, "a" if self.append_csv else "w", newline="")
        self._csv_writer = csv.writer(self._csv_handle)
        if first:
            header = [
                "time_s", "source", "path_length_m", "max_step_speed_mps",
                "min_obstacle_dist_m", "publish_latency_ms", "service_ms",
                "replan_interval_ms", "reached_goal",
                "terminal_goal_error_m", "terminal_goal_anchored", "exact_terminal_goal",
                "fallback_exact_endpoint_anchored", "fallback_exact_start_anchored", "fallback_exact_goal_anchored",
                # FPGA pipeline diagnostics; A* rows keep these fields empty.
                "solve_seq", "metrics_matched", "variables", "size_profile", "source_mode", "fallback_reason",
                "preprocess_ms", "planner_ms", "internal_planning_ms", "fpga_ms",
                "board_load_total_ms", "board_transport_rx_ms", "board_pl_solve_ms",
                "board_result_read_ms", "board_restart_overhead_ms",
                "issued_iteration_budget", "early_restart_stopped",
                "spine_source", "retry_inflated_gate_required",
                "force_spine_refresh", "spine_shortcut_applied",
                "spine_shortcut_ratio", "spine_nodes_before_shortcut",
                "near_hysteresis_refresh", "reuse_streak",
                "clearance_ms", "pre_clearance_setup_ms", "clearance_compute_ms",
                "clearance_crc_ms", "clearance_kernel_ms",
                "grid_validation_ms", "raw_grid_validation_ms",
                "grid_validation_copy_avoided", "raw_grid_validation_copy_avoided",
                "trusted_binary_grid",
                "clearance_cache_probe", "clearance_cache_bypassed",
                "clearance_cache_bypass_remaining", "clearance_cache_miss_streak",
                "clearance_cache_permanent_bypass", "clearance_cache_policy",
                "clearance_cache_identity_mode", "clearance_cache_token_present",
                "clearance_cache_token_hit",
                "spine_ms", "candidate_ms",
                "segment_ms", "qubo_ms", "solve_host_ms",
                "retime_ms", "dense_collision_ms",
                "host_payload_ms", "host_fingerprint_ms", "host_payload_prepared_bytes",
                "host_payload_cache_fastpath", "host_payload_build_modes",
                "native_wire_payload", "native_wire_payload_version",
                "host_transport_wall_ms", "host_decode_ms",
                "udp_spin_wait_us", "udp_spin_hit", "udp_spin_elapsed_us",
                "udp_receive_phase",
                "host_solver_total_ms", "solve_decode_score_ms", "solve_host_unaccounted_ms",
                "host_payload_bytes", "host_load_modes",
                "solver_transactions", "solver_profiles",
                "attempts", "solve_dt_s", "trajectory_dt_s",
                "dt_s", "dt_sample_s",
                "astar_search_ms", "near_goal_direct", "direct_distance_m",
                "retime_expansions", "w_len", "reuse_refresh_period_s",
                "refresh_requested", "bag_rate",
                "replan_on_cloud", "cloud_replan_min_period_s", "replan_period_s", "replan_distance_m",
                "cloud_defer_during_solve", "cloud_deferred_count", "cloud_coalesced_count",
                "cloud_deferred_during_solve", "cloud_coalesced_during_solve", "cloud_pending_at_publish",
                "cloud_processing_ms", "cloud_processing_deferred",
                "reuse_refresh_min_interval_s", "near_goal_direct_safety_mode",
                "near_goal_direct_cross_speed_actual_mps",
                "quality_tolerance_m", "spine_geom_len_m", "raw_fpga_geom_len_m",
                "pre_shortcut_quality_gap", "quality_guard_rescued",
                "selected_geom_len_m", "published_waypoint_geom_len_m",
                "quality_recovery_used", "quality_recovery_ms",
                "quality_recovery_geom_len_m",
                "post_shortcut_applied", "post_shortcut_ratio", "post_shortcut_ms",
                "reuse_refresh_bypass_count",
                "bypass_since_refresh", "near_hysteresis_distance_m",
                "near_hysteresis_detour_ratio", "near_hysteresis_improvement_ratio",
                "single_candidate_layers",
                "planner_profile_version", "udp_spin_budget_us", "udp_spin_adaptive",
                "udp_broadcast_fallback", "udp_connected_unicast",
                "dt_unquantized_s", "qubo_dt_quantum_s", "publish_speed_ratio",
                "udp_blocking_poll_us",
                "strict_spine_latency_fuse_ms", "spine_latency_fuse_triggered",
                "native_cpp_accel", "native_cpp_requested", "native_cpp_version",
                "candidate_native_cpp", "candidate_cpp_requested", "incremental_qubo",
                "native_qubo", "native_qubo_version",
                "incremental_qubo_static_hit", "incremental_qubo_static_build_ms",
                "incremental_qubo_cache_entries", "incremental_qubo_cache_size",
                "clearance_cache_hit", "native_clearance", "native_chain_collision",
                "native_spine_segments", "native_retime_dense", "native_clearance_cap2",
                "wsl2_detected", "wsl2_tuning", "wsl2_solver_cpu", "wsl2_cloud_cpu",
                "wsl2_main_affinity_applied", "wsl2_solver_affinity_applied",
                "wsl2_cloud_affinity_applied", "wsl2_persistent_solver_worker",
                "wsl2_async_cloud_worker", "wsl2_gc_guard", "wsl2_realtime_priority",
                "wsl2_realtime_applied", "wsl2_cloud_dropped_frames",
                "udp_incoming_cpu_applied", "udp_kernel_busy_poll_us",
                "udp_kernel_busy_poll_applied",
                "wsl2_timer_slack_ns", "wsl2_timer_slack_applied",
                "udp_native_receive_requested", "udp_native_receive_applied",
                "udp_native_receive_version", "udp_native_receive_fallbacks",
                "solver_persistent_worker", "solver_gc_guard", "solver_gc_guard_applied",
                "solver_preflight_ping", "solver_preflight_done", "solver_preflight_ok",
                "solver_preflight_ms", "solver_preflight_error",
                "solver_host_warmup_done", "solver_host_warmup_ms",
                "solver_thread_native_id", "solver_cpu_affinity",
                "solver_affinity_auto", "solver_affinity_selected_cpu", "solver_affinity_selected_load",
                "solver_affinity_sample_ms", "solver_affinity_applied", "solver_affinity_error",
                "planner_error_type", "planner_error_text",
            ]
            self._csv_header = header
            self._csv_writer.writerow(header)
        elif not self._csv_header:
            # Current code always starts a new CSV in benchmark runs; this
            # fallback preserves error-row indexing for append mode.
            self._csv_header = [
                "time_s", "source", "path_length_m", "max_step_speed_mps",
                "min_obstacle_dist_m", "publish_latency_ms", "service_ms",
                "replan_interval_ms", "reached_goal"
            ]

    def _path_length(self, msg: Path) -> tuple[float, float]:
        total = 0.0
        max_step = 0.0
        prev = None
        for pose in msg.poses:
            p = pose.pose.position
            if prev is not None:
                step = math.sqrt((p.x - prev[0]) ** 2 + (p.y - prev[1]) ** 2 + (p.z - prev[2]) ** 2)
                total += step
                max_step = max(max_step, step)
            prev = (p.x, p.y, p.z)
        return total, max_step

    def _min_clearance(self, msg: Path) -> float:
        if not self._cloud:
            return float("nan")
        path_points = np.asarray([
            (pose.pose.position.x, pose.pose.position.y, pose.pose.position.z)
            for pose in msg.poses[::5]
        ], dtype=np.float64)
        cloud = np.asarray(self._cloud, dtype=np.float64)
        if path_points.size == 0 or cloud.size == 0:
            return float("nan")
        # Chunk to bound temporary memory while removing the old nested Python
        # loop that could itself perturb measured replan intervals.
        best2 = float("inf")
        for begin in range(0, len(path_points), 64):
            chunk = path_points[begin:begin + 64]
            d2 = np.sum((chunk[:, None, :] - cloud[None, :, :]) ** 2, axis=2)
            best2 = min(best2, float(np.min(d2)))
        return math.sqrt(best2) if math.isfinite(best2) else float("nan")

    def _reached_goal(self, msg: Path) -> bool:
        if self._goal is None or not msg.poses:
            return False
        end = msg.poses[-1].pose.position
        g = self._goal.pose.position
        return math.dist((end.x, end.y, end.z), (g.x, g.y, g.z)) <= GOAL_TOL_M

    # ---------- callbacks ----------
    def _cloud_cb(self, msg: PointCloud2) -> None:
        points: list[tuple[float, float, float]] = []
        for index, point in enumerate(point_cloud2.read_points(
                msg, field_names=("x", "y", "z"), skip_nans=True)):
            if index % self.cloud_stride:
                continue
            points.append((float(point[0]), float(point[1]), float(point[2])))
            if len(points) >= self.max_cloud_points:
                break
        self._cloud = points
        # The comparison RViz displays /cloud_registered directly with AxisColor.
        # Keep only the sampled XYZ copy here for clearance metrics; do not replace
        # the source cloud with a monochrome Marker.

    def _goal_cb(self, msg: PoseStamped) -> None:
        self._goal = msg
        self._publish_goal_marker(msg)

    def _publish_environment_marker(self, header, points: list[tuple[float, float, float]]) -> None:
        """Legacy sampled-cloud publisher; the final RViz view uses raw coloured PointCloud2."""
        marker = Marker()
        marker.header = header
        marker.ns = "fpga_astar_environment"
        marker.id = 0
        marker.type = Marker.POINTS
        marker.action = Marker.ADD
        marker.scale.x = 0.035
        marker.scale.y = 0.035
        marker.color.r = 0.62
        marker.color.g = 0.62
        marker.color.b = 0.62
        marker.color.a = 0.32
        marker.points = [Point(x=x, y=y, z=z) for x, y, z in points]
        self.marker_pub_environment.publish(marker)

    def _publish_goal_marker(self, msg: PoseStamped) -> None:
        marker = Marker()
        marker.header = msg.header
        marker.ns = "fpga_astar_context"
        marker.id = 2
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose = msg.pose
        marker.scale.x = marker.scale.y = marker.scale.z = 0.18
        marker.color.r = 1.0
        marker.color.g = 0.85
        marker.color.b = 0.0
        marker.color.a = 1.0
        self.marker_pub_goal.publish(marker)

    def _publish_start_marker(self, msg: Path) -> None:
        if not msg.poses:
            return
        marker = Marker()
        marker.header = msg.header
        marker.ns = "fpga_astar_context"
        marker.id = 1
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose = msg.poses[0].pose
        marker.scale.x = marker.scale.y = marker.scale.z = 0.18
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.25
        marker.color.a = 1.0
        self.marker_pub_start.publish(marker)

    @staticmethod
    def _sample_marker_points(msg: Path, count: int = 18) -> list[Point]:
        if not msg.poses:
            return []
        n = len(msg.poses)
        if n <= count:
            return [pose.pose.position for pose in msg.poses]
        ids = sorted({int(round(i * (n - 1) / float(count - 1))) for i in range(count)})
        return [msg.poses[i].pose.position for i in ids]

    def _publish_comparison_text(self, header) -> None:
        if not self._astar_stats and not self._fpga_stats:
            return
        if self._goal is not None:
            x = float(self._goal.pose.position.x)
            y = float(self._goal.pose.position.y)
            z = float(self._goal.pose.position.z) + 0.55
        elif self._cloud:
            x = max(p[0] for p in self._cloud)
            y = min(p[1] for p in self._cloud)
            z = max(p[2] for p in self._cloud) + 0.5
        else:
            return

        def line(name: str, stats: dict[str, object]) -> str:
            if not stats:
                return f"{name}: waiting"
            clear = float(stats.get("clearance", float("nan")))
            service = float(stats.get("service_ms", float("nan")))
            clear_s = f"{clear:.2f}m" if math.isfinite(clear) else "n/a"
            service_s = f"{service:.1f}ms" if math.isfinite(service) else "n/a"
            return f"{name}: L={float(stats.get('length', 0.0)):.2f}m  clear={clear_s}  service={service_s}"

        marker = Marker()
        marker.header = header
        marker.ns = "fpga_astar_context"
        marker.id = 3
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z
        marker.pose.orientation.w = 1.0
        marker.scale.z = 0.16
        marker.color.r = marker.color.g = marker.color.b = 1.0
        marker.color.a = 0.95
        marker.text = line("A* [RED]", self._astar_stats) + "\n" + line("FPGA [BLUE]", self._fpga_stats)
        if self._astar_stats and self._fpga_stats:
            a = float(self._astar_stats.get("length", 0.0))
            f = float(self._fpga_stats.get("length", 0.0))
            if a > 1.0e-9:
                marker.text += f"\nFPGA-A*: path {((f / a) - 1.0) * 100.0:+.1f}%"
            if math.isfinite(self._path_deviation_mean_m):
                marker.text += (f"  |  deviation mean={self._path_deviation_mean_m:.2f}m"
                                f" max={self._path_deviation_max_m:.2f}m")
        self.marker_pub_text.publish(marker)

    def _service_ms_cb(self, msg: Float64) -> None:
        self._fpga_service_ms = float(msg.data)

    def _astar_service_ms_cb(self, msg: Float64) -> None:
        self._astar_service_ms = float(msg.data)

    def _astar_search_ms_cb(self, msg: Float64) -> None:
        self._astar_search_ms = float(msg.data)

    def _metrics_cb(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
            self._fpga_metrics = data
            seq = int(data.get("solve_seq", -1))
            if seq >= 0:
                self._fpga_metrics_by_seq[seq] = data
                # Bounded history protects long rosbag loops while preserving
                # exact path/metrics association across independent ROS topics.
                if len(self._fpga_metrics_by_seq) > 64:
                    for old_seq in sorted(self._fpga_metrics_by_seq)[:-32]:
                        self._fpga_metrics_by_seq.pop(old_seq, None)
            stamp_ns = int(data.get("path_stamp_ns", -1))
            if stamp_ns >= 0:
                self._fpga_metrics_by_stamp[stamp_ns] = data
                if len(self._fpga_metrics_by_stamp) > 64:
                    for old_stamp in sorted(self._fpga_metrics_by_stamp)[:-32]:
                        self._fpga_metrics_by_stamp.pop(old_stamp, None)
        except Exception:
            pass

    def _error_metrics_cb(self, msg: String) -> None:
        """Record planner exceptions without mixing them into path statistics."""
        try:
            data = json.loads(msg.data)
        except Exception:
            return
        self._ensure_csv()
        if not self._csv_header or self._csv_writer is None:
            return
        row: list[object] = [""] * len(self._csv_header)
        index = {name: i for i, name in enumerate(self._csv_header)}
        def put(name: str, value: object) -> None:
            if name in index:
                row[index[name]] = value
        put("time_s", round(time.time(), 3))
        put("source", "fpga_error")
        put("service_ms", data.get("service_ms", ""))
        put("solve_seq", data.get("solve_seq", ""))
        put("metrics_matched", 1)
        put("direct_distance_m", data.get("direct_distance_m", ""))
        put("bag_rate", round(self.bag_rate, 3) if math.isfinite(self.bag_rate) else "")
        put("planner_error_type", data.get("planner_error_type", ""))
        put("planner_error_text", data.get("planner_error_text", ""))
        self._csv_writer.writerow(row)
        self._csv_handle.flush()
        self._rows.append(row)

    def _path_cb(self, source: str, msg: Path) -> None:
        if not msg.poses:
            return
        now = time.time()
        length, max_step = self._path_length(msg)
        dt_sample = self.fpga_dt_sample_s if source == "fpga" else self.astar_dt_sample_s
        max_speed = max_step / dt_sample if max_step > 0 else 0.0
        latency_ms = (rospy.time_now_seconds() - rospy.stamp_to_seconds(msg.header.stamp)) * 1000.0
        previous_seen = self._last_seen.get(source)
        interval_ms = (now - previous_seen) * 1000.0 if previous_seen is not None else float("nan")
        self._last_seen[source] = now
        metrics: dict[str, object] = {}
        if source == "fpga":
            # ROS 2 removed Header.seq; correlate diagnostics with the exact
            # Path timestamp published in the planner's JSON metrics.
            path_stamp_ns = rospy.stamp_to_nanoseconds(msg.header.stamp)
            metrics = self._fpga_metrics_by_stamp.pop(path_stamp_ns, {})
            solve_seq = int(metrics.get("solve_seq", -1)) if metrics else -1
            if not metrics and int(self._fpga_metrics.get("path_stamp_ns", -1)) == path_stamp_ns:
                metrics = self._fpga_metrics
                solve_seq = int(metrics.get("solve_seq", -1))
            if not metrics and self.metrics_wait_ms > 0.0:
                deadline = time.perf_counter() + self.metrics_wait_ms / 1000.0
                while time.perf_counter() < deadline and not rospy.is_shutdown():
                    time.sleep(0.0005)
                    metrics = self._fpga_metrics_by_stamp.pop(path_stamp_ns, {})
                    if metrics:
                        solve_seq = int(metrics.get("solve_seq", -1))
                        break
                    if int(self._fpga_metrics.get("path_stamp_ns", -1)) == path_stamp_ns:
                        metrics = self._fpga_metrics
                        solve_seq = int(metrics.get("solve_seq", -1))
                        break
            try:
                service_ms = float(metrics.get("service_ms", self._fpga_service_ms))
            except Exception:
                service_ms = float(self._fpga_service_ms)
        else:
            solve_seq = -1
            service_ms = float(getattr(self, "_astar_service_ms", float("nan")))
        stats = {
            "length": length,
            "max_speed": max_speed,
            "clearance": self._min_clearance(msg),
            "latency_ms": latency_ms,
            "service_ms": service_ms,
            "interval_ms": interval_ms,
            "reached": self._reached_goal(msg),
            "points": len(msg.poses),
        }
        if source == "astar":
            self._astar_stats = stats
        else:
            self._fpga_stats = stats
        self._ensure_csv()
        def _m(name: str, default=""):
            value = metrics.get(name, default) if isinstance(metrics, dict) else default
            if isinstance(value, float) and math.isfinite(value):
                return round(value, 3)
            return value
        row = [
            round(now, 3), source,
            round(stats["length"], 4), round(stats["max_speed"], 4),
            round(stats["clearance"], 4) if math.isfinite(stats["clearance"]) else "nan",
            round(stats["latency_ms"], 1), round(stats["service_ms"], 1),
            round(stats["interval_ms"], 1), int(stats["reached"]),
            _m("terminal_goal_error_m"), _m("terminal_goal_anchored"), _m("exact_terminal_goal"),
            _m("fallback_exact_endpoint_anchored"), _m("fallback_exact_start_anchored"), _m("fallback_exact_goal_anchored"),
            (solve_seq if source == "fpga" else ""),
            (int(bool(metrics)) if source == "fpga" else ""),
            _m("variables"), _m("size_profile"), _m("source"), _m("fallback_reason"),
            _m("preprocess_ms"), _m("planner_ms"), _m("internal_planning_ms"), _m("fpga_ms"),
            _m("board_load_total_ms"), _m("board_transport_rx_ms"), _m("board_pl_solve_ms"),
            _m("board_result_read_ms"), _m("board_restart_overhead_ms"),
            _m("issued_iteration_budget"), _m("early_restart_stopped"),
            _m("spine_source"), _m("retry_inflated_gate_required"),
            _m("force_spine_refresh"), _m("spine_shortcut_applied"),
            _m("spine_shortcut_ratio"), _m("spine_nodes_before_shortcut"),
            _m("near_hysteresis_refresh"), _m("reuse_streak"),
            _m("clearance_ms"), _m("pre_clearance_setup_ms"), _m("clearance_compute_ms"),
            _m("clearance_crc_ms"), _m("clearance_kernel_ms"),
            _m("grid_validation_ms"), _m("raw_grid_validation_ms"),
            _m("grid_validation_copy_avoided"), _m("raw_grid_validation_copy_avoided"),
            _m("trusted_binary_grid"),
            _m("clearance_cache_probe"), _m("clearance_cache_bypassed"),
            _m("clearance_cache_bypass_remaining"), _m("clearance_cache_miss_streak"),
            _m("clearance_cache_permanent_bypass"), _m("clearance_cache_policy"),
            _m("clearance_cache_identity_mode"), _m("clearance_cache_token_present"),
            _m("clearance_cache_token_hit"),
            _m("spine_ms"), _m("candidate_ms"),
            _m("segment_ms"), _m("qubo_ms"), _m("solve_host_ms"),
            _m("retime_ms"), _m("dense_collision_ms"),
            _m("host_payload_ms"), _m("host_fingerprint_ms"), _m("host_payload_prepared_bytes"),
            _m("host_payload_cache_fastpath"), _m("host_payload_build_modes"),
            _m("native_wire_payload"), _m("native_wire_payload_version"),
            _m("host_transport_wall_ms"), _m("host_decode_ms"),
            _m("udp_spin_wait_us"), _m("udp_spin_hit"), _m("udp_spin_elapsed_us"),
            _m("udp_receive_phase"),
            _m("host_solver_total_ms"), _m("solve_decode_score_ms"), _m("solve_host_unaccounted_ms"),
            _m("host_payload_bytes"), _m("host_load_modes"),
            _m("solver_transactions"), _m("solver_profiles"),
            _m("attempts"), _m("solve_dt_s"), _m("trajectory_dt_s"),
            _m("dt_s"), _m("dt_sample_s", dt_sample),
            (round(float(self._astar_search_ms), 3) if source == "astar" and math.isfinite(float(self._astar_search_ms)) else ""),
            _m("near_goal_direct"), _m("direct_distance_m"), _m("retime_expansions"),
            _m("w_len"), _m("reuse_refresh_period_s"), _m("refresh_requested"),
            (round(self.bag_rate, 3) if math.isfinite(self.bag_rate) else ""),
            _m("replan_on_cloud"), _m("cloud_replan_min_period_s"), _m("replan_period_s"), _m("replan_distance_m"),
            _m("cloud_defer_during_solve"), _m("cloud_deferred_count"), _m("cloud_coalesced_count"),
            _m("cloud_deferred_during_solve"), _m("cloud_coalesced_during_solve"), _m("cloud_pending_at_publish"),
            _m("cloud_processing_ms"), _m("cloud_processing_deferred"),
            _m("reuse_refresh_min_interval_s"),
            _m("near_goal_direct_safety_mode"),
            _m("near_goal_direct_cross_speed_actual_mps"),
            _m("quality_tolerance_m"), _m("spine_geom_len_m"), _m("raw_fpga_geom_len_m"),
            _m("pre_shortcut_quality_gap"), _m("quality_guard_rescued"),
            _m("selected_geom_len_m"), _m("published_waypoint_geom_len_m"),
            _m("quality_recovery_used"), _m("quality_recovery_ms"),
            _m("quality_recovery_geom_len_m"),
            _m("post_shortcut_applied"), _m("post_shortcut_ratio"), _m("post_shortcut_ms"),
            _m("reuse_refresh_bypass_count"),
            _m("bypass_since_refresh"), _m("near_hysteresis_distance_m"),
            _m("near_hysteresis_detour_ratio"), _m("near_hysteresis_improvement_ratio"),
            _m("single_candidate_layers"),
            _m("planner_profile_version"), _m("udp_spin_budget_us"), _m("udp_spin_adaptive"),
            _m("udp_broadcast_fallback"), _m("udp_connected_unicast"),
            _m("dt_unquantized_s"), _m("qubo_dt_quantum_s"), _m("publish_speed_ratio"),
            _m("udp_blocking_poll_us"),
            _m("strict_spine_latency_fuse_ms"), _m("spine_latency_fuse_triggered"),
            _m("native_cpp_accel"), _m("native_cpp_requested"), _m("native_cpp_version"),
            _m("candidate_native_cpp"), _m("candidate_cpp_requested"), _m("incremental_qubo"),
            _m("native_qubo"), _m("native_qubo_version"),
            _m("incremental_qubo_static_hit"), _m("incremental_qubo_static_build_ms"),
            _m("incremental_qubo_cache_entries"), _m("incremental_qubo_cache_size"),
            _m("clearance_cache_hit"), _m("native_clearance"), _m("native_chain_collision"),
            _m("native_spine_segments"), _m("native_retime_dense"), _m("native_clearance_cap2"),
            _m("wsl2_detected"), _m("wsl2_tuning"), _m("wsl2_solver_cpu"), _m("wsl2_cloud_cpu"),
            _m("wsl2_main_affinity_applied"), _m("wsl2_solver_affinity_applied"),
            _m("wsl2_cloud_affinity_applied"), _m("wsl2_persistent_solver_worker"),
            _m("wsl2_async_cloud_worker"), _m("wsl2_gc_guard"), _m("wsl2_realtime_priority"),
            _m("wsl2_realtime_applied"), _m("wsl2_cloud_dropped_frames"),
            _m("udp_incoming_cpu_applied"), _m("udp_kernel_busy_poll_us"),
            _m("udp_kernel_busy_poll_applied"),
            _m("wsl2_timer_slack_ns"), _m("wsl2_timer_slack_applied"),
            _m("udp_native_receive_requested"), _m("udp_native_receive_applied"),
            _m("udp_native_receive_version"), _m("udp_native_receive_fallbacks"),
            _m("solver_persistent_worker"), _m("solver_gc_guard"), _m("solver_gc_guard_applied"),
            _m("solver_preflight_ping"), _m("solver_preflight_done"), _m("solver_preflight_ok"),
            _m("solver_preflight_ms"), _m("solver_preflight_error"),
            _m("solver_host_warmup_done"), _m("solver_host_warmup_ms"),
            _m("solver_thread_native_id"), _m("solver_cpu_affinity"),
            _m("solver_affinity_auto"), _m("solver_affinity_selected_cpu"), _m("solver_affinity_selected_load"),
            _m("solver_affinity_sample_ms"), _m("solver_affinity_applied"), _m("solver_affinity_error"),
            "", "",
        ]
        self._csv_writer.writerow(row)
        self._csv_handle.flush()
        self._rows.append(row)
        self._publish_marker(source, msg)
        rospy.loginfo(
            "[%s] len=%.2fm speed=%.2fm/s clear=%.2fm lat=%.0fms service=%.0fms interval=%.0fms reached=%s",
            source, stats["length"], stats["max_speed"], stats["clearance"],
            stats["latency_ms"], stats["service_ms"], stats["interval_ms"], stats["reached"],
        )

    def _publish_difference_links(self, header) -> None:
        """Highlight geometric separation between FPGA and A* trajectories.

        Sparse FPGA samples are connected to their nearest A* point.  Links below
        5 cm are suppressed so coincident sections remain clean.  This is a
        visualization-only comparison and does not alter either published path.
        """
        astar = self._latest_path_points.get("astar", [])
        fpga = self._latest_path_points.get("fpga", [])
        marker = Marker()
        marker.header = header
        marker.ns = "fpga_astar_difference"
        marker.id = 30
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.scale.x = 0.010
        marker.color.r = 1.0
        marker.color.g = 0.92
        marker.color.b = 0.05
        marker.color.a = 0.90

        if not astar or not fpga:
            marker.points = []
            self.marker_pub_difference.publish(marker)
            return

        a = np.asarray([(p.x, p.y, p.z) for p in astar], dtype=np.float64)
        f = np.asarray([(p.x, p.y, p.z) for p in fpga], dtype=np.float64)
        # At most 14 links keeps the 3-D view readable even for dense paths.
        ids = np.unique(np.linspace(0, len(f) - 1, min(14, len(f))).round().astype(int))
        distances = []
        links = []
        for idx in ids:
            q = f[idx]
            d2 = np.sum((a - q) ** 2, axis=1)
            j = int(np.argmin(d2))
            d = float(math.sqrt(float(d2[j])))
            distances.append(d)
            if d >= 0.05:
                links.append(Point(x=float(q[0]), y=float(q[1]), z=float(q[2])))
                links.append(Point(x=float(a[j, 0]), y=float(a[j, 1]), z=float(a[j, 2])))

        if distances:
            self._path_deviation_mean_m = float(np.mean(distances))
            self._path_deviation_max_m = float(np.max(distances))
        marker.points = links
        self.marker_pub_difference.publish(marker)

    def _publish_marker(self, source: str, msg: Path) -> None:
        path_points = [pose.pose.position for pose in msg.poses]
        self._latest_path_points[source] = path_points

        # A* gets a white halo so the red reference path stays visible against
        # the restored coloured environment.  Both comparison paths use slim
        # line widths; the FPGA path remains slightly narrower so overlap is clear.
        if source == "astar":
            halo = Marker()
            halo.header = msg.header
            halo.ns = "fpga_astar_halo"
            halo.id = 0
            halo.type = Marker.LINE_STRIP
            halo.action = Marker.ADD
            halo.scale.x = 0.055
            halo.color.r = halo.color.g = halo.color.b = 1.0
            halo.color.a = 0.92
            halo.points = path_points
            self.marker_pub_astar_halo.publish(halo)

        line = Marker()
        line.header = msg.header
        line.ns = "fpga_astar_compare"
        line.id = 1 if source == "astar" else 2
        line.type = Marker.LINE_STRIP
        line.action = Marker.ADD
        line.color.a = 1.0
        if source == "astar":
            line.scale.x = 0.035
            line.color.r = 1.0
            line.color.g = 0.02
            line.color.b = 0.02
        else:
            line.scale.x = 0.025
            line.color.r = 0.0
            line.color.g = 0.20
            line.color.b = 1.0
        line.points = path_points
        (self.marker_pub_astar if source == "astar" else self.marker_pub_fpga).publish(line)

        # Distinct sample shapes remain visible when the two lines nearly overlap.
        samples = Marker()
        samples.header = msg.header
        samples.ns = "fpga_astar_samples"
        samples.id = 11 if source == "astar" else 12
        samples.type = Marker.SPHERE_LIST if source == "astar" else Marker.CUBE_LIST
        samples.action = Marker.ADD
        samples.scale.x = samples.scale.y = samples.scale.z = 0.060
        samples.color.a = 1.0
        if source == "astar":
            samples.color.r = 1.0
            samples.color.g = 0.02
            samples.color.b = 0.02
        else:
            samples.color.r = 0.0
            samples.color.g = 0.20
            samples.color.b = 1.0
        samples.points = self._sample_marker_points(msg, count=16)
        (self.marker_pub_astar_points if source == "astar" else self.marker_pub_fpga_points).publish(samples)

        self._publish_difference_links(msg.header)
        self._publish_start_marker(msg)
        self._publish_comparison_text(msg.header)

    def _aggregate(self, rows: list[list[object]]) -> dict[str, float | int]:
        if not rows:
            return {}
        keys = {
            "path_length_m": 2, "max_step_speed_mps": 3, "min_obstacle_dist_m": 4,
            "publish_latency_ms": 5, "service_ms": 6, "replan_interval_ms": 7,
            "variables": 11, "preprocess_ms": 15, "planner_ms": 16,
            "internal_planning_ms": 17, "fpga_ms": 18,
            "board_load_total_ms": 19, "board_transport_rx_ms": 20,
            "board_pl_solve_ms": 21, "board_result_read_ms": 22,
            "board_restart_overhead_ms": 23, "issued_iteration_budget": 24,
        }
        out: dict[str, float | int] = {"count": len(rows)}
        for name, col in keys.items():
            values = []
            for row in rows:
                try:
                    value = float(row[col])
                except (TypeError, ValueError):
                    continue
                if math.isfinite(value):
                    values.append(value)
            if not values:
                continue
            out[f"{name}_mean"] = statistics.mean(values)
            out[f"{name}_p50"] = statistics.median(values)
            values_sorted = sorted(values)
            out[f"{name}_p95"] = values_sorted[min(len(values_sorted) - 1, int(len(values_sorted) * 0.95))]
        reached = sum(1 for row in rows if int(row[8]) == 1)
        out["reached_count"] = reached
        return out

    def _write_summary(self) -> None:
        rows = [row for row in self._rows if row[1] == "astar"]
        fpga_rows = [row for row in self._rows if row[1] == "fpga"]
        with open(self.summary_csv, "w", newline="") as handle:
            writer = csv.writer(handle)
            writer.writerow(["source", "metric", "count", "mean", "p50", "p95", "reached"])
            for name, group in (("astar", rows), ("fpga", fpga_rows)):
                agg = self._aggregate(group)
                if not agg:
                    continue
                for metric in ("path_length_m", "max_step_speed_mps", "min_obstacle_dist_m",
                               "publish_latency_ms", "service_ms", "replan_interval_ms"):
                    mean = agg.get(f"{metric}_mean")
                    p50 = agg.get(f"{metric}_p50")
                    p95 = agg.get(f"{metric}_p95")
                    if mean is None:
                        continue
                    writer.writerow([name, metric, agg["count"],
                                     round(mean, 3), round(p50, 3), round(p95, 3), agg.get("reached_count", 0)])
        rospy.loginfo("SUMMARY written to %s (%d rows)", self.summary_csv, len(self._rows))

    def _summary_cb(self, _event: object) -> None:
        def fmt(stats: dict[str, object]) -> str:
            if not stats:
                return "no path yet"
            return ("len=%.2fm speed=%.2fm/s clear=%.2fm lat=%.0fms service=%.0fms reached=%s"
                    % (stats["length"], stats["max_speed"], stats["clearance"],
                       stats["latency_ms"], stats.get("service_ms", 0.0), stats["reached"]))
        rospy.loginfo("SUMMARY  A* : %s", fmt(self._astar_stats))
        rospy.loginfo("SUMMARY  FPGA: %s", fmt(self._fpga_stats))

    def _periodic_stats_cb(self, _event: object) -> None:
        for name, group in (("astar", "astar"), ("fpga", "fpga")):
            rows = [row for row in self._rows if row[1] == group]
            agg = self._aggregate(rows)
            if not agg:
                continue
            rospy.loginfo(
                "STATS %s: n=%d len mean/p95=%.2f/%.2f m  lat mean/p95=%.0f/%.0f ms  "
                "service mean/p95=%.0f/%.0f ms  interval mean/p95=%.0f/%.0f ms",
                name, agg["count"],
                agg.get("path_length_m_mean", 0.0), agg.get("path_length_m_p95", 0.0),
                agg.get("publish_latency_ms_mean", 0.0), agg.get("publish_latency_ms_p95", 0.0),
                agg.get("service_ms_mean", 0.0), agg.get("service_ms_p95", 0.0),
                agg.get("replan_interval_ms_mean", 0.0), agg.get("replan_interval_ms_p95", 0.0),
            )
        self._write_summary()


def main() -> None:
    rospy.init_node("fpga_astar_compare")
    FpgaAstarCompare()
    rospy.spin()


if __name__ == "__main__":
    main()
