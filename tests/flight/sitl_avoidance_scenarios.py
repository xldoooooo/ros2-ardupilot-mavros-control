#!/usr/bin/env python3
"""真实ArduPilot避障闭环验证，含最近点接入回归：仅domain231/LOCALHOST仿真。"""
from __future__ import annotations

import argparse
import json
import math
import os
import shlex
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from ground_station_core.environment import EnvironmentInitializer
from ground_station_core.event_log import EventLog
from ground_station_core.models import WaypointFlightStrategy as Strategy
from ground_station_core.models import WaypointReferenceGenerator as Generator
from ground_station_core.models import WaypointTrackingController as Tracking
from ground_station_core.ros_controller import GroundStationRosController


class ScanScene:
    """真实几何射线：有限前向FOV、盒子内壁、可移走实心竖直圆柱，遮挡取首交点。"""
    def __init__(self, context):
        import rclpy
        from rclpy.qos import qos_profile_sensor_data
        from geometry_msgs.msg import PoseStamped
        from guided_interfaces.msg import VideoCapture, ControlStatus, AvoidanceResult
        from nav_msgs.msg import Odometry, Path as RosPath
        from sensor_msgs.msg import PointCloud2, PointField

        self.node = rclpy.create_node("task38_geometric_lidar", context=context)
        from rclpy.executors import SingleThreadedExecutor
        self.executor = SingleThreadedExecutor(context=context)
        self.executor.add_node(self.node)
        self.pose = None
        self.pole = False
        self.direction = 0.0
        self.points = None
        self.origin = np.zeros(3)
        self.pole_center = np.array([1.5, 0.0])
        self.captures = []
        self.paths = []
        self.statuses = []
        self.avoidance_results = []
        self.node.create_subscription(AvoidanceResult, "/onboard_control/avoidance_result",
                                      self.avoidance_results.append, 10)
        self.node.create_subscription(PoseStamped, "/mavros/local_position/pose",
                                      lambda m: setattr(self, "pose", m), qos_profile_sensor_data)
        self.node.create_subscription(VideoCapture, "/video_service/capture",
                                      self.captures.append, 20)
        self.node.create_subscription(RosPath, "/onboard_control/avoidance_path", self.paths.append, 1)
        self.node.create_subscription(ControlStatus, "/onboard_control/status", self.statuses.append,
                                      qos_profile_sensor_data)
        self.odom = self.node.create_publisher(Odometry, "/task38/scan_odom", qos_profile_sensor_data)
        self.cloud = self.node.create_publisher(PointCloud2, "/task38/scan", qos_profile_sensor_data)
        self.Odometry, self.PointCloud2, self.PointField = Odometry, PointCloud2, PointField
        a, e = np.meshgrid(np.radians(np.arange(-55., 55.1, 1.0)),
                            np.radians(np.arange(-35., 35.1, 1.0)))
        self.directions = np.column_stack((np.cos(e).ravel() * np.cos(a).ravel(),
                                          np.cos(e).ravel() * np.sin(a).ravel(), np.sin(e).ravel()))
        self.running = True
        self.thread = threading.Thread(target=self.spin, daemon=True)
        self.thread.start()

    def spin(self):
        import rclpy
        next_scan = 0.0
        while self.running:
            self.executor.spin_once(timeout_sec=0.01)
            if self.pose is None or time.monotonic() < next_scan:
                continue
            next_scan = time.monotonic() + .1
            p = self.pose.pose.position
            origin = np.array([p.x, p.y, p.z])
            c, s = math.cos(self.direction), math.sin(self.direction)
            rotation = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
            rays = self.directions @ rotation.T
            # Map z=0 is the FCU's grounded origin, not the floor surface beneath the vehicle.
            # Explicit finite enclosure stays within the production map's 300000-cell cap.
            lower = self.origin + np.array([-2.5, -2.5, -0.25])
            upper = self.origin + np.array([6., 2.5, 3.5])
            with np.errstate(divide="ignore", invalid="ignore"):
                hits = np.where(rays > 0, (upper-origin)/rays, (lower-origin)/rays)
                hits[hits <= 0] = np.inf
                distance = np.min(hits, axis=1)
            if self.pole:
                offset = origin[:2] - self.pole_center
                aa = np.sum(rays[:, :2]**2, axis=1)
                bb = 2 * (rays[:, :2] @ offset)
                cc = offset @ offset - .18**2
                disc = bb**2 - 4 * aa * cc
                valid = disc >= 0
                t = np.full(len(rays), np.inf)
                t[valid] = (-bb[valid] - np.sqrt(disc[valid])) / (2 * aa[valid])
                with np.errstate(invalid="ignore"):
                    hit_z = origin[2] + t * rays[:, 2]
                valid &= (t > 0) & (hit_z >= -.25) & (hit_z <= 3.5)
                distance[valid] = np.minimum(distance[valid], t[valid])
            points = origin + distance[:, None] * rays
            points = points[np.isfinite(points).all(axis=1)].astype("<f4")
            stamp = self.node.get_clock().now().to_msg()
            odom = self.Odometry()
            odom.header.frame_id = "map"
            odom.header.stamp = stamp
            odom.child_frame_id = "lidar"
            odom.pose.pose = self.pose.pose
            self.odom.publish(odom)
            cloud = self.PointCloud2()
            cloud.header = odom.header
            cloud.height = 1
            cloud.width = len(points)
            cloud.point_step = 12
            cloud.row_step = cloud.width * 12
            cloud.is_dense = True
            cloud.fields = [self.PointField(name=n, offset=i*4, datatype=7, count=1)
                            for i, n in enumerate(("x", "y", "z"))]
            cloud.data = points.tobytes()
            self.points = points
            self.cloud.publish(cloud)

    def stop(self):
        self.running = False
        self.thread.join(3)
        self.executor.remove_node(self.node)
        self.executor.shutdown()
        self.node.destroy_node()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--scenario", choices=["straight", "hover_recover", "avoid", "avoid_recover", "timeout", "fault_cancel"], required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    os.environ.update(QT_QPA_PLATFORM="offscreen", ROS_DOMAIN_ID="231",
                      ROS_AUTOMATIC_DISCOVERY_RANGE="LOCALHOST")
    import rclpy
    from rclpy.context import Context
    context = Context()
    rclpy.init(context=context, domain_id=231)
    events = EventLog(max_events=20000)
    controller = GroundStationRosController(source_id="task38-isolated-sitl", event_log=events)
    env = EnvironmentInitializer(controller, event_log=events)
    scene = None
    bridge = None
    samples = []
    report = {"scenario": args.scenario, "success": False, "domain": 231, "geometry": "raycast .18m radius vertical cylinder and box; first surface occludes back surfaces"}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    log = args.output.with_suffix(".planner.log").open("w")

    def wait(predicate, timeout=30., label="condition"):
        deadline = time.monotonic()+timeout
        while time.monotonic()<deadline:
            snap = controller.snapshot()
            samples.append({"t": time.monotonic(), "x": snap.x, "y": snap.y, "z": snap.z,
                            "vx": snap.vx, "vy": snap.vy, "vz": snap.vz,
                            "tx": snap.target_x, "ty": snap.target_y, "tz": snap.target_z,
                            "tvx": snap.target_vx, "tvy": snap.target_vy, "tvz": snap.target_vz,
                            "tax": snap.target_ax, "tay": snap.target_ay, "taz": snap.target_az,
                            "state": snap.avoidance_state, "detail": snap.avoidance_detail,
                            "remaining": snap.avoidance_wait_remaining_seconds,
                            "armed": snap.armed, "mode": snap.active_mode.name,
                            "waypoint": snap.waypoint_index, "deadline_misses": snap.deadline_miss_count})
            if predicate(): return
            time.sleep(.05)
        raise TimeoutError(label + ": " + str(controller.snapshot()))

    def final(ticket, timeout=80.):
        result = None
        def completed():
            nonlocal result
            result = controller.wait_for_result(ticket, timeout=.005)
            return result is not None
        wait(completed, timeout, f"command {ticket} final")
        report.setdefault("commands", []).append({"command": result.command, "success": result.success, "message": result.message})
        if not result.success: raise RuntimeError(result.message)
        return result

    try:
        done = threading.Event()
        init = {}
        env.initialize_simulation(lambda level, m: print(m, flush=True),
                                  lambda ok, m: (init.update(ok=ok, message=m), done.set()),
                                  avoidance_demo=False)
        if not done.wait(150) or not init.get("ok"): raise RuntimeError(str(init))
        # Explicit isolation gate before issuing any arm/takeoff-capable simulation command.
        assert controller.domain_id == 231
        scene = ScanScene(context)
        wait(lambda: scene.pose is not None, 10, "SITL pose")
        snap = controller.snapshot()
        scene.origin = np.array([snap.x, snap.y, 0.0])
        scene.pole_center = np.array([snap.x+1.5, snap.y])
        if args.scenario == "avoid_recover":
            # Put a real occupied surface at the goal: there is no feasible route until it moves.
            scene.pole_center[0] = snap.x+3.0
        strategy = Strategy.STRAIGHT if args.scenario == "straight" else (
            Strategy.AVOID if args.scenario in ("avoid", "avoid_recover") else Strategy.HOVER_ON_OBSTACLE)
        if strategy is not Strategy.STRAIGHT:
            launch_env = os.environ.copy()
            planner_setup = shlex.quote(str(ROOT.parent / "dyn_small_obs_avoidance-ros2/install/setup.bash"))
            command = f"source /opt/ros/jazzy/setup.bash && source {planner_setup} && source install/setup.bash && exec ros2 launch avoidance_bridge avoidance.launch.py coordinate_mode:=identity cloud:=/task38/scan odom:=/task38/scan_odom"
            bridge = subprocess.Popen(["bash", "-c", command], cwd=ROOT, env=launch_env,
                                      stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            wait(lambda: controller.snapshot().avoidance_ready, 30, "planner readiness")
        final(controller.request_takeoff(1.5, strategy, Generator.TRAPEZOIDAL_PROFILE,
                                         Tracking.TRAJECTORY_PD_DOB), 60)
        wait(lambda: abs(controller.snapshot().z-1.5)<.08 and
             math.hypot(controller.snapshot().vx,controller.snapshot().vy)<.07, 30, "settled")
        # Observe the whole future region before inserting the moving obstacle; no hidden free-space fixture.
        time.sleep(2)
        snap = controller.snapshot()
        scene.pole = args.scenario != "straight"
        time.sleep(1.2)
        start = np.array([snap.x,snap.y,snap.z])
        target = (start[0]+3.0, start[1], 1.5, 0.0)
        capture_start = len(scene.captures)
        ticket = controller.request_waypoints((target,), strategy,
            reference_generator=Generator.TRAPEZOIDAL_PROFILE, tracking_controller=Tracking.TRAJECTORY_PD_DOB)
        if args.scenario in ("hover_recover", "avoid_recover", "timeout", "fault_cancel"):
            wait(lambda: controller.snapshot().avoidance_state in (1,3) and
                 controller.snapshot().waypoint_count==1, 10, "blocked waiting")
            time.sleep(2)
            report["blocked_drift_m"] = math.hypot(controller.snapshot().x-start[0],controller.snapshot().y-start[1])
            report["blocked_capture_count"] = len(scene.captures)-capture_start
            assert report["blocked_capture_count"] == 0
            assert report["blocked_drift_m"] < .25
            if args.scenario in ("hover_recover", "avoid_recover"):
                scene.pole = False
                removed = time.monotonic()
                wait(lambda: controller.snapshot().avoidance_state==2, 6, "obstacle removed recovery")
                report["recovery_seconds"] = time.monotonic()-removed
                final(ticket)
            elif args.scenario == "fault_cancel":
                final(controller.request_hover())
                scene.pole = False
                time.sleep(2)
                assert controller.snapshot().waypoint_count==0
                assert controller.snapshot().avoidance_state==0
                report["cancelled_task_did_not_resume"] = True
            else:
                failed = None
                def failure():
                    nonlocal failed
                    failed=controller.wait_for_result(ticket,timeout=.005)
                    return failed is not None
                wait(failure, 22, "avoidance timeout failure")
                assert not failed.success
                report["expected_failure"] = failed.message
                wait(lambda: not controller.snapshot().armed, 40, "actual LAND disarm")
                report["land_confirmed_disarmed"] = True
        else:
            final(ticket)
        if args.scenario not in ("timeout","fault_cancel"):
            # 任务终态与拍照事件来自不同 DDS 话题，等待异步订阅线程收到事件再核对次数。
            wait(lambda: len(scene.captures) > capture_start, 3, "waypoint capture event")
            report["waypoint_capture_count"] = len(scene.captures)-capture_start
            assert report["waypoint_capture_count"] == 1, report["waypoint_capture_count"]
        if args.scenario == "avoid":
            route = [s for s in samples if s["state"] == 2 and s["mode"]=="WAYPOINT"]
            assert route, "no executed avoidance references"
            distances = [math.hypot(s["x"]-scene.pole_center[0],s["y"]-scene.pole_center[1])-.18 for s in route]
            report["actual_min_cylinder_clearance_m"] = min(distances)
            report["actual_max_lateral_offset_m"] = max(abs(s["y"]-start[1]) for s in route)
            assert report["actual_max_lateral_offset_m"] > .4
            assert report["actual_min_cylinder_clearance_m"] > .35
        report["max_reference_horizontal_speed"] = max(math.hypot(s["tvx"],s["tvy"]) for s in samples)
        report["max_actual_horizontal_speed"] = max(math.hypot(s["vx"],s["vy"]) for s in samples)
        if controller.snapshot().armed:
            final(controller.request_land(),40)
            wait(lambda: not controller.snapshot().armed, 5, "land status propagated")
        report["success"] = True
    except Exception as exc:
        report["error"] = f"{type(exc).__name__}: {exc}"
        print("FAILED", str(exc), flush=True)
    finally:
        if controller.snapshot().armed:
            try: final(controller.request_land(),40)
            except Exception as exc: report["cleanup_land_error"] = str(exc)
        if scene:
            report["raw_status_count"] = len(scene.statuses)
            report["nonempty_path_count"] = sum(bool(p.poses) for p in scene.paths)
            report["empty_path_count"] = sum(not p.poses for p in scene.paths)
            report["avoidance_results"] = [{"id": r.request_id, "status": r.status,
                "ready": r.ready, "detail": r.detail} for r in scene.avoidance_results]
            scene.stop()
        if bridge:
            os.killpg(bridge.pid,signal.SIGINT)
            try: bridge.wait(5)
            except subprocess.TimeoutExpired: os.killpg(bridge.pid,signal.SIGTERM)
        env.cleanup()
        controller.stop()
        rclpy.shutdown(context=context)
        log.close()
        args.output.write_text(json.dumps(report,ensure_ascii=False,indent=2))
        args.output.with_suffix(".samples.json").write_text(json.dumps(samples,ensure_ascii=False))
        print(json.dumps(report,ensure_ascii=False,indent=2),flush=True)
    return 0 if report["success"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
