#!/usr/bin/env python3
"""
地面站控制台 — Ground Station Control Panel
- 启动 MAVROS (含自动设置消息频率: local_pos 100Hz, imu 100Hz)
- 启动 odin1 (占位)
- 启动 Rviz (自动 source 工作空间 + 静态 TF 兜底)
- 起飞 / 降落
- 方向速度控制 (rclpy 内嵌节点, 发布到 /mavros/setpoint_velocity/cmd_vel_unstamped)
  按住按钮持续发布，松开归零。
- 悬停 (PD+DOB, 对齐 keyboard_vel_controller.cpp MODE_HOVER_CUSTOM,
  发布 AttitudeTarget 到 /mavros/setpoint_raw/attitude, 复用其 YAML 增益参数)
"""

import atexit
import os
import queue
import subprocess
import sys
import tempfile
import threading
import time
import tkinter as tk
from tkinter import ttk, messagebox

# --- 路径配置 ---
PROJECT_ROOT = os.path.dirname(os.path.abspath(__file__))
SHFILES_DIR = os.path.join(PROJECT_ROOT, "shfiles")
INSTALL_SETUP = os.path.join(PROJECT_ROOT, "install", "setup.bash")

# --- 参数 ---
VELOCITY_SCALE = 0.2
PUBLISH_TOPIC = "/mavros/setpoint_velocity/cmd_vel_unstamped"
PUBLISH_RATE_HZ = 100
TAKEOFF_ALTITUDE = 0.3  # 起飞高度 (米)

GRAVITY_ACC = 9.8  # 重力加速度 (m/s²)

# PD+DOB 悬停增益 —— 与 src/guided_sim/params/keyboard_vel_controller.yaml 一致。
# 运行时优先读取该 YAML 复用参数，读取失败时回退到这些默认值。
HOVER_PARAM_DEFAULTS = {
    "hover_wn_xy": 2.236,    # 悬停XY: 固有频率 ωₙ (rad/s)
    "hover_zeta_xy": 0.8,    # 悬停XY: 阻尼比 ζ
    "hover_wn_z": 2.236,     # 悬停Z:  固有频率 ωₙ (rad/s)
    "hover_zeta_z": 0.6,     # 悬停Z:  阻尼比 ζ
    "dob_L_xy": 1.5,         # DOB 观测器增益 (XY轴)
    "dob_L_z": 0.6,          # DOB 观测器增益 (Z轴)
    "hover_throttle": 0.2,   # 悬停油门 (归一化)
    "thrust_ratio": 2.5,     # 最大推重比
    "uav_weight": 1.7,       # 无人机重量 (kg)
}


def _load_hover_params():
    """复用 keyboard_vel_controller.yaml 中的 PD+DOB 悬停增益参数。"""
    candidates = [
        os.path.join(PROJECT_ROOT, "src", "guided_sim", "params",
                     "keyboard_vel_controller.yaml"),
        os.path.join(PROJECT_ROOT, "install", "guided_sim", "share",
                     "guided_sim", "params", "keyboard_vel_controller.yaml"),
    ]
    for path in candidates:
        if not os.path.isfile(path):
            continue
        try:
            import yaml
            with open(path, "r", encoding="utf-8") as f:
                data = yaml.safe_load(f) or {}
            root = data.get("/**", data) if isinstance(data, dict) else {}
            params = root.get("ros__parameters", root) if isinstance(root, dict) else {}
            merged = dict(HOVER_PARAM_DEFAULTS)
            for k in HOVER_PARAM_DEFAULTS:
                v = params.get(k)
                if v is not None:
                    merged[k] = float(v)
            return merged
        except Exception as e:
            print(f"[GS] 读取悬停参数失败 ({path}): {e}", flush=True)
    return dict(HOVER_PARAM_DEFAULTS)


# ============================================================
# 终端工具
# ============================================================
_background_procs = []


def _source_cmd():
    if os.path.isfile(INSTALL_SETUP):
        return f"source {INSTALL_SETUP} 2>/dev/null; "
    return ""


_tmp_files = set()


def run_in_bg(command: str, title: str = ""):
    """后台静默运行命令，不弹终端窗口。输出写入 /tmp 日志文件。"""
    log_file = f"/tmp/gs_{title.replace(' ', '_').lower()}.log" if title else "/tmp/gs_bg.log"
    script_body = (
        "#!/usr/bin/env bash\n"
        + _source_cmd()
        + f"echo '--- {title or 'BG'} started at' $(date) '---' >> {log_file}\n"
        + command + f" >> {log_file} 2>&1\n"
        + f'echo "--- Finished (exit $?) at $(date) ---" >> {log_file}\n'
    )
    tmp = tempfile.NamedTemporaryFile(
        mode='w', suffix='.sh', delete=False, dir='/tmp',
    )
    tmp.write(script_body)
    tmp.close()
    os.chmod(tmp.name, 0o755)
    _tmp_files.add(tmp.name)

    try:
        proc = subprocess.Popen(
            ["bash", tmp.name],
            cwd=PROJECT_ROOT,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            start_new_session=True,
        )
        _background_procs.append(proc)
        return True
    except Exception as e:
        messagebox.showerror("启动失败", f"无法启动后台进程:\n{e}")
        return False


# ============================================================
# ROS2 后台节点: 速度发布 + 服务调用 + 静态 TF 兜底
# ============================================================


# ============================================================
# ROS2 后台节点: 速度发布 + 服务调用 + 静态 TF 兜底
# ============================================================
class ROS2Node:
    """后台线程运行 rclpy 节点。"""

    def __init__(self):
        self._lock = threading.Lock()
        self._vel_x = 0.0
        self._vel_y = 0.0
        self._vel_z = 0.0
        self._vel_yaw = 0.0
        self._pos_x = 0.0
        self._pos_y = 0.0
        self._pos_z = 0.0
        self._pos_yaw = 0.0
        self._running = False
        self._thread = None
        self._ready = False
        self._error = None
        self._fc_connected = False
        self._cmd_queue = queue.Queue()
        self._last_result = None
        self._result_lock = threading.Lock()

        # 航点任务状态
        self._wp_mission_active = False
        self._waypoints = []        # list of (x, y, z, yaw_rad)
        self._wp_index = 0
        self._wp_tolerance = 0.3    # 到达判定距离 (米)
        self._wp_arrive_time = None # 到达当前航点的时刻
        self._wp_hold_time = 1.0    # 到达后悬停时间 (秒)
        self._wp_hovering = False   # 任务完成后悬停
        self._wp_hover_target = None

        # PD+DOB 悬停状态（对齐 keyboard_vel_controller.cpp MODE_HOVER_CUSTOM）
        self._dob_hover_active = False
        self._dob_target = None          # 悬停目标 (x, y, z, yaw)，首帧抓拍
        self._dob_z_x = 0.0
        self._dob_z_y = 0.0
        self._dob_z_z = 0.0
        self._dob_d_x_hat = 0.0          # 扰动估计 d̂
        self._dob_d_y_hat = 0.0
        self._dob_d_z_hat = 0.0
        self._dob_u_x = 0.0              # 上一拍控制量 u
        self._dob_u_y = 0.0
        self._dob_u_z = 0.0
        self._dob_last_time = 0.0
        self._dob_first_frame = True   # 首次进入 DOB 控制时重置观测器
        # 当前速度反馈（/mavros/local_position/velocity_local）
        self._fb_vx = 0.0
        self._fb_vy = 0.0
        self._fb_vz = 0.0
        # DOB 悬停增益（复用 keyboard_vel_controller.yaml）
        _hp = _load_hover_params()
        self._hover_wn_xy = _hp["hover_wn_xy"]
        self._hover_zeta_xy = _hp["hover_zeta_xy"]
        self._hover_wn_z = _hp["hover_wn_z"]
        self._hover_zeta_z = _hp["hover_zeta_z"]
        self._dob_L_xy = _hp["dob_L_xy"]
        self._dob_L_z = _hp["dob_L_z"]
        self._hover_throttle = _hp["hover_throttle"]
        self._thrust_ratio = _hp["thrust_ratio"]
        self._uav_weight = _hp["uav_weight"]

    @property
    def ready(self):
        return self._ready

    @property
    def error(self):
        return self._error

    @property
    def fc_connected(self):
        return self._fc_connected

    @property
    def last_result(self):
        with self._result_lock:
            return self._last_result

    # ---- 速度 ----
    def add_velocity(self, x, y, z, yaw):
        """累加模型：每次按键 += scale，永不自动归零（对齐 C++ keyboard_listener）"""
        with self._lock:
            self._vel_x += x
            self._vel_y += y
            self._vel_z += z
            self._vel_yaw += yaw

    def zero_velocity(self):
        """悬停：所有速度分量归零"""
        with self._lock:
            self._vel_x = 0.0
            self._vel_y = 0.0
            self._vel_z = 0.0
            self._vel_yaw = 0.0

    def request_dob_hover(self):
        """进入 PD+DOB 悬停：抓拍当前位置为悬停目标，退出航点任务。"""
        with self._lock:
            self._dob_hover_active = True
            self._dob_target = (self._pos_x, self._pos_y, self._pos_z, self._pos_yaw)
            self._dob_first_frame = True
            self._wp_hovering = False          # 退出航点悬停
            self._wp_mission_active = False

    def exit_dob_hover(self):
        """退出 PD+DOB 悬停（速度控制分支重新接管）。"""
        with self._lock:
            self._dob_hover_active = False
            self._dob_target = None

    def get_position(self):
        """读取当前位姿 (x, y, z, yaw)，从 /mavros/local_position/pose 回调更新"""
        return (self._pos_x, self._pos_y, self._pos_z, self._pos_yaw)

    # ---- 命令 ----
    def request_takeoff(self, altitude=TAKEOFF_ALTITUDE):
        self._cmd_queue.put(("takeoff", altitude))

    def request_land(self):
        self._cmd_queue.put(("land", None))

    def request_set_rates(self):
        """设置 MAVROS 消息频率 (local_pos=100, imu=100, imu_raw=100)。"""
        self._cmd_queue.put(("set_rates", None))

    def request_waypoints(self, waypoints):
        """启动航点任务。waypoints: list of (x, y, z, yaw_rad)"""
        self._cmd_queue.put(("waypoints", list(waypoints)))

    def request_set_gp_origin(self, lat, lon, alt):
        """设置 GPS 原点 (global_position/set_gp_origin)。"""
        self._cmd_queue.put(("set_gp_origin", (lat, lon, alt)))

    # ---- 生命周期 ----
    def start(self):
        if self._running:
            return
        self._running = True
        self._thread = threading.Thread(target=self._spin, daemon=True)
        self._thread.start()
        for _ in range(30):
            if self._ready or self._error:
                break
            time.sleep(0.1)

    def stop(self):
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=3.0)

    def _spin(self):
        try:
            import rclpy
            from geometry_msgs.msg import Twist, TwistStamped, TransformStamped, PoseStamped
            from mavros_msgs.srv import CommandTOL, SetMode, CommandBool, MessageInterval
            from mavros_msgs.msg import AttitudeTarget, State
            from tf2_ros import StaticTransformBroadcaster

            rclpy.init(args=["--ros-args", "--log-level", "warn"])
            node = rclpy.create_node("ground_station_node")

            # 发布器
            vel_pub = node.create_publisher(Twist, PUBLISH_TOPIC, 10)
            pos_pub = node.create_publisher(PoseStamped, "/mavros/setpoint_position/local", 10)
            att_pub = node.create_publisher(AttitudeTarget, "/mavros/setpoint_raw/attitude", 10)

            # 服务客户端
            takeoff_client = node.create_client(CommandTOL, "/mavros/cmd/takeoff")
            set_mode_client = node.create_client(SetMode, "/mavros/set_mode")
            arming_client = node.create_client(CommandBool, "/mavros/cmd/arming")
            msg_interval_client = node.create_client(MessageInterval, "/mavros/set_message_interval")

            # 飞控状态跟踪（持久订阅，供起飞逻辑查询）
            self._fc_armed = False
            self._fc_mode = ""
            self._fc_connected = False
            def _state_cb(msg: State):
                self._fc_armed = msg.armed
                self._fc_mode = msg.mode
                self._fc_connected = msg.connected
            node.create_subscription(State, "/mavros/state", _state_cb, 10)

            # 位姿跟踪（持久订阅，供 GUI 显示）
            import math as _math
            def _pose_cb(msg):
                self._pos_x = msg.pose.position.x
                self._pos_y = msg.pose.position.y
                self._pos_z = msg.pose.position.z
                q = msg.pose.orientation
                siny = 2.0 * (q.w * q.z + q.x * q.y)
                cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
                self._pos_yaw = _math.atan2(siny, cosy)
            pos_qos = rclpy.qos.QoSProfile(
                depth=1, reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT)
            node.create_subscription(PoseStamped, "/mavros/local_position/pose", _pose_cb, pos_qos)

            # 速度反馈（PD+DOB 悬停需要），与位姿同源 /mavros/local_position/*
            def _vel_cb(msg):
                self._fb_vx = msg.twist.linear.x
                self._fb_vy = msg.twist.linear.y
                self._fb_vz = msg.twist.linear.z
            node.create_subscription(
                TwistStamped, "/mavros/local_position/velocity_local", _vel_cb, pos_qos)

            # 静态 TF 兜底: map → base_link 置于原点
            # 保证 Rviz 中 TF 链完整 (即使 MAVROS 未运行)
            # MAVROS 运行时 pose_to_tf 节点发出的动态 /tf 会自动覆盖
            static_broadcaster = StaticTransformBroadcaster(node)
            static_tf = TransformStamped()
            static_tf.header.stamp = node.get_clock().now().to_msg()
            static_tf.header.frame_id = "map"
            static_tf.child_frame_id = "base_link"
            static_tf.transform.translation.x = 0.0
            static_tf.transform.translation.y = 0.0
            static_tf.transform.translation.z = 0.0
            static_tf.transform.rotation.w = 1.0
            static_broadcaster.sendTransform(static_tf)

            self._ready = True
            self._vel_enabled = False  # 起飞成功后才开启速度发布
            period = 1.0 / PUBLISH_RATE_HZ

            while self._running and rclpy.ok():
                # 1) 处理服务命令
                self._process_commands(
                    node, takeoff_client, set_mode_client,
                    arming_client, msg_interval_client,
                )
                # 2) 航点任务（优先级高于速度控制，PD+DOB 跟踪）
                if self._wp_mission_active and self._wp_index < len(self._waypoints):
                    wp = self._waypoints[self._wp_index]
                    self._publish_dob_setpoint(
                        node, att_pub, pos_pub, (wp[0], wp[1], wp[2], wp[3]))
                    self._advance_waypoint()
                elif self._wp_hovering and self._wp_hover_target is not None:
                    # 任务完成后悬停在最后航点（PD+DOB）
                    wp = self._wp_hover_target
                    self._publish_dob_setpoint(
                        node, att_pub, pos_pub, (wp[0], wp[1], wp[2], wp[3]))
                elif self._dob_hover_active and self._fc_armed and self._fc_mode == "GUIDED":
                    # PD+DOB 悬停（对齐 keyboard_vel_controller.cpp MODE_HOVER_CUSTOM）
                    self._publish_dob_setpoint(node, att_pub, pos_pub, self._dob_target)
                elif self._vel_enabled:
                    # 3) 发布速度（仅起飞成功后且无航点任务）
                    with self._lock:
                        vx, vy, vz, vyaw = self._vel_x, self._vel_y, self._vel_z, self._vel_yaw
                    msg = Twist()
                    msg.linear.x = vx
                    msg.linear.y = vy
                    msg.linear.z = vz
                    msg.angular.z = vyaw
                    vel_pub.publish(msg)

                rclpy.spin_once(node, timeout_sec=period)

            # 退出前归零
            msg = Twist()
            vel_pub.publish(msg)
            time.sleep(0.05)

            node.destroy_node()
            rclpy.shutdown()
        except Exception as e:
            import traceback
            print(f"[GS] ROS2 THREAD CRASHED: {e}", flush=True)
            traceback.print_exc()
            self._error = str(e)

    def _heartbeat(self):
        now = time.time()
        if not hasattr(self, '_hb_time'):
            self._hb_time = 0.0
        if now - self._hb_time > 5.0:
            print(f"[GS] LOOP alive | vel_enabled={self._vel_enabled} | v=({self._vel_x:.2f},{self._vel_y:.2f},{self._vel_z:.2f},{self._vel_yaw:.2f})", flush=True)
            self._hb_time = now

    def _process_commands(self, node, takeoff, set_mode, arming, msg_int):
        from mavros_msgs.srv import SetMode, MessageInterval

        try:
            cmd, arg = self._cmd_queue.get_nowait()
        except queue.Empty:
            return

        if cmd == "takeoff":
            self._do_takeoff(arg)

        elif cmd == "land":
            req = SetMode.Request()
            req.custom_mode = "LAND"
            future = set_mode.call_async(req)
            if self._wait_service(node, future, 3.0, "Land"):
                ok = future.result() and future.result().mode_sent
                self._set_result(ok, "降落指令已发送" if ok else "设置 LAND 模式失败")
            else:
                self._set_result(False, "降落超时")

        elif cmd == "set_rates":
            self._do_set_rates(node, msg_int)

        elif cmd == "waypoints":
            self._do_waypoints(node, arg)

        elif cmd == "set_gp_origin":
            self._do_set_gp_origin(node, arg)

    def _wait_service(self, node, future, timeout_sec=5.0, step_name=""):
        """手动 spin 等待 future 完成，替代不稳定的 spin_until_future_complete。"""
        import rclpy
        start = time.time()
        while not future.done():
            rclpy.spin_once(node, timeout_sec=0.01)
            if time.time() - start > timeout_sec:
                print(f"[GS]    {step_name} future timed out after {timeout_sec}s", flush=True)
                return False
        return True

    def _do_takeoff(self, altitude=1.0):
        """起飞：独立子进程跑 test_takeoff5.py（已验证成功）。"""
        if not self._fc_connected:
            self._set_result(False, "飞控未连接，请先点击「启动 MAVROS」并等待「飞控: 已连接」")
            return

        self._set_result(False, f"起飞中 (目标 {altitude:.1f}m)...")
        try:
            script = os.path.join(PROJECT_ROOT, "test_takeoff5.py")
            proc = subprocess.Popen(
                ["python3", script, str(altitude)],
                cwd=PROJECT_ROOT,
                stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                text=True,
            )
            out_lines = []
            start = time.time()
            while proc.poll() is None:
                line = proc.stdout.readline()
                if line:
                    print(f"[TAKEOFF] {line.rstrip()}", flush=True)
                    out_lines.append(line)
                    if "SUCCESS!" in line or "FAIL" in line:
                        break
                if time.time() - start > 60:
                    proc.kill()
                    self._set_result(False, "起飞超时 (60s)")
                    return
                time.sleep(0.05)
            for line in proc.stdout:
                print(f"[TAKEOFF] {line.rstrip()}", flush=True)
                out_lines.append(line)
            if "SUCCESS!" in "".join(out_lines):
                self._vel_enabled = True
                self._set_result(True, "起飞成功 — 速度控制已启用")
            else:
                self._set_result(False, f"起飞失败")
        except Exception as e:
            self._set_result(False, f"起飞异常: {e}")

    def _do_set_rates(self, node, msg_int):
        """设置 MAVROS 消息频率。"""
        from mavros_msgs.srv import MessageInterval

        rates = [(32, 100.0), (31, 100.0), (105, 100.0)]
        success = True
        for msg_id, rate in rates:
            req = MessageInterval.Request()
            req.message_id = msg_id
            req.message_rate = rate
            future = msg_int.call_async(req)
            if not self._wait_service(node, future, 3.0, f"set_rates-{msg_id}"):
                success = False
                break
            if not future.result() or not future.result().success:
                success = False
                break
            time.sleep(0.1)
        self._set_result(success, "消息频率已设置 (100Hz)" if success else "设置消息频率失败")

    def _do_set_gp_origin(self, node, _unused):
        """发布 GPS 原点 (global_position/set_gp_origin)。"""
        lat, lon, alt = _unused
        try:
            from geographic_msgs.msg import GeoPointStamped
            import rclpy
            pub = node.create_publisher(GeoPointStamped, "/mavros/global_position/set_gp_origin", 10)
            msg = GeoPointStamped()
            msg.header.stamp = node.get_clock().now().to_msg()
            msg.header.frame_id = "map"
            msg.position.latitude = lat
            msg.position.longitude = lon
            msg.position.altitude = alt
            # 发布几次确保送达 (QoS unreliable)
            for _ in range(5):
                pub.publish(msg)
                time.sleep(0.05)
            node.destroy_publisher(pub)
            self._set_result(True, f"GPS 原点已设置 (lat={lat:.6f}, lon={lon:.6f}, alt={alt:.1f})")
        except Exception as e:
            self._set_result(False, f"设置 GPS 原点失败: {e}")

    def _do_waypoints(self, node, waypoints):
        """启动航点任务：切换到 GUIDED 模式，按序飞越航点。"""
        if not self._fc_connected:
            self._set_result(False, "飞控未连接，无法执行航点任务")
            return

        if not waypoints:
            self._set_result(False, "航点列表为空")
            return

        from mavros_msgs.srv import SetMode, CommandBool
        import rclpy

        # 确保 GUIDED 模式
        set_mode_client = node.create_client(SetMode, "/mavros/set_mode")
        if set_mode_client.wait_for_service(timeout_sec=3.0):
            req = SetMode.Request()
            req.custom_mode = "GUIDED"
            future = set_mode_client.call_async(req)
            if self._wait_service(node, future, 5.0, "GUIDED for waypoints"):
                if not (future.result() and future.result().mode_sent):
                    self._set_result(False, "无法切换到 GUIDED 模式")
                    return
            else:
                self._set_result(False, "GUIDED 模式切换超时")
                return

        # 确保已武装
        if not self._fc_armed:
            arming_client = node.create_client(CommandBool, "/mavros/cmd/arming")
            if arming_client.wait_for_service(timeout_sec=3.0):
                req = CommandBool.Request()
                req.value = True
                future = arming_client.call_async(req)
                if self._wait_service(node, future, 5.0, "Arm for waypoints"):
                    if not (future.result() and future.result().success):
                        self._set_result(False, "无法武装")
                        return
                else:
                    self._set_result(False, "武装超时")
                    return
            # 等待武装状态
            for _ in range(50):
                rclpy.spin_once(node, timeout_sec=0.1)
                if self._fc_armed:
                    break
            if not self._fc_armed:
                self._set_result(False, "武装等待超时")
                return

        # 启动航点任务
        self._waypoints = waypoints
        self._wp_index = 0
        self._wp_arrive_time = None
        self._wp_mission_active = True
        self._wp_hovering = False      # 重置悬停状态
        self._wp_hover_target = None
        self._dob_hover_active = False  # 退出 DOB 悬停
        self._dob_first_frame = True    # 进入 DOB 航点跟踪时重置观测器
        self._vel_enabled = False      # 暂停速度控制
        self._set_result(
            True,
            f"航点任务已启动 — 共 {len(waypoints)} 个航点"
        )

    def _advance_waypoint(self):
        """检测当前航点是否到达，到达并保持后推进到下一航点。由 spin 循环调用。"""
        import math
        wp = self._waypoints[self._wp_index]
        dx = self._pos_x - wp[0]
        dy = self._pos_y - wp[1]
        dz = self._pos_z - wp[2]
        dist = math.sqrt(dx*dx + dy*dy + dz*dz)

        if dist < self._wp_tolerance:
            now = time.time()
            if self._wp_arrive_time is None:
                self._wp_arrive_time = now
            elif now - self._wp_arrive_time >= self._wp_hold_time:
                # 进入下一航点
                self._wp_index += 1
                self._wp_arrive_time = None
                if self._wp_index >= len(self._waypoints):
                    # 任务完成，PD+DOB 悬停在最后一个航点
                    self._wp_mission_active = False
                    self._wp_hover_target = wp  # 保存最后航点作为悬停目标
                    self._wp_hovering = True
                    self._set_result(
                        True,
                        f"航点任务完成 — 已到达全部 {len(self._waypoints)} 个航点，悬停中"
                    )
                else:
                    self._set_result(
                        True,
                        f"航点 {self._wp_index}/{len(self._waypoints)}: "
                        f"前往 ({wp[0]:.1f}, {wp[1]:.1f}, {wp[2]:.1f})"
                    )
        else:
            self._wp_arrive_time = None  # 离开到达范围则重置

    @staticmethod
    def _quat_from_rotmat(r):
        """旋转矩阵 → 四元数 (x, y, z, w)，Shepperd 方法。"""
        import math
        r00, r01, r02 = r[0]
        r10, r11, r12 = r[1]
        r20, r21, r22 = r[2]
        trace = r00 + r11 + r22
        if trace > 0.0:
            s = math.sqrt(trace + 1.0) * 2.0
            w = 0.25 * s
            x = (r21 - r12) / s
            y = (r02 - r20) / s
            z = (r10 - r01) / s
        elif r00 > r11 and r00 > r22:
            s = math.sqrt(1.0 + r00 - r11 - r22) * 2.0
            w = (r21 - r12) / s
            x = 0.25 * s
            y = (r01 + r10) / s
            z = (r02 + r20) / s
        elif r11 > r22:
            s = math.sqrt(1.0 + r11 - r00 - r22) * 2.0
            w = (r02 - r20) / s
            x = (r01 + r10) / s
            y = 0.25 * s
            z = (r12 + r21) / s
        else:
            s = math.sqrt(1.0 + r22 - r00 - r11) * 2.0
            w = (r10 - r01) / s
            x = (r02 + r20) / s
            y = (r12 + r21) / s
            z = 0.25 * s
        n = math.sqrt(w * w + x * x + y * y + z * z)
        return (x / n, y / n, z / n, w / n)

    def _publish_dob_setpoint(self, node, att_pub, pos_pub, target):
        """PD+DOB 位置跟踪 —— 移植自 keyboard_vel_controller.cpp MODE_HOVER_CUSTOM。

        悬停时 target 为抓拍的固定点，航点飞行时 target 为当前航点。
        位置环:  acc = ωₙ²·err − 2ζωₙ·vel
        DOB:     ż = −L·z − L²·vel − L·u,  d̂ = z + L·vel（一阶低通 L/(s+L)，DC 增益 1）
        输出:    u = acc_pid − d̂，总加速度 = u + g，由其方向生成姿态四元数并映射油门。
        """
        import math
        from geometry_msgs.msg import PoseStamped
        from mavros_msgs.msg import AttitudeTarget

        G = GRAVITY_ACC

        # 首次进入 DOB 控制：重置观测器，发布一次位置设定点
        if self._dob_first_frame:
            self._dob_first_frame = False
            self._dob_z_x = self._dob_z_y = self._dob_z_z = 0.0
            self._dob_d_x_hat = self._dob_d_y_hat = self._dob_d_z_hat = 0.0
            self._dob_u_x = self._dob_u_y = self._dob_u_z = 0.0
            self._dob_last_time = time.time()
            tx, ty, tz, tyaw = target
            p = PoseStamped()
            p.header.stamp = node.get_clock().now().to_msg()
            p.header.frame_id = "map"
            p.pose.position.x = tx
            p.pose.position.y = ty
            p.pose.position.z = tz
            p.pose.orientation.z = math.sin(tyaw / 2.0)
            p.pose.orientation.w = math.cos(tyaw / 2.0)
            pos_pub.publish(p)
            return

        tx, ty, tz, tyaw = target

        # 位置环 PD（增益由固有频率 ωₙ 与阻尼比 ζ 换算）
        err_x = tx - self._pos_x
        err_y = ty - self._pos_y
        err_z = tz - self._pos_z
        kp_xy = self._hover_wn_xy * self._hover_wn_xy
        kd_xy = 2.0 * self._hover_zeta_xy * self._hover_wn_xy
        kp_z = self._hover_wn_z * self._hover_wn_z
        kd_z = 2.0 * self._hover_zeta_z * self._hover_wn_z
        acc_pid_x = kp_xy * err_x - kd_xy * self._fb_vx
        acc_pid_y = kp_xy * err_y - kd_xy * self._fb_vy
        acc_pid_z = kp_z * err_z - kd_z * self._fb_vz

        # DOB 观测器更新（dt 阈值 5ms，与 C++ 一致）
        now = time.time()
        dt = now - self._dob_last_time
        l_xy, l_z = self._dob_L_xy, self._dob_L_z
        if dt > 0.005:
            den_xy = l_xy * dt + 1.0
            den_z = l_z * dt + 1.0
            self._dob_z_x = (self._dob_z_x - l_xy * l_xy * dt * self._fb_vx
                             - l_xy * self._dob_u_x * dt) / den_xy
            self._dob_z_y = (self._dob_z_y - l_xy * l_xy * dt * self._fb_vy
                             - l_xy * self._dob_u_y * dt) / den_xy
            self._dob_z_z = (self._dob_z_z - l_z * l_z * dt * self._fb_vz
                             - l_z * self._dob_u_z * dt) / den_z
            self._dob_d_x_hat = self._dob_z_x + l_xy * self._fb_vx
            self._dob_d_y_hat = self._dob_z_y + l_xy * self._fb_vy
            self._dob_d_z_hat = self._dob_z_z + l_z * self._fb_vz
            self._dob_last_time = now

        u_x = acc_pid_x - self._dob_d_x_hat
        u_y = acc_pid_y - self._dob_d_y_hat
        u_z = acc_pid_z - self._dob_d_z_hat
        self._dob_u_x, self._dob_u_y, self._dob_u_z = u_x, u_y, u_z

        # 总期望加速度 = 运动加速度 + 重力补偿
        an_x, an_y, an_z = u_x, u_y, u_z + G
        acc_norm = math.sqrt(an_x * an_x + an_y * an_y + an_z * an_z)

        # 由总加速度方向（机体 z 轴）与期望偏航构造姿态四元数
        eps = 1e-6
        if acc_norm < eps:
            qx, qy, qz, qw = 0.0, 0.0, math.sin(tyaw / 2.0), math.cos(tyaw / 2.0)
        else:
            bx, by, bz = an_x / acc_norm, an_y / acc_norm, an_z / acc_norm
            t_x, t_y = math.cos(tyaw), math.sin(tyaw)
            dot = t_x * bx + t_y * by
            cx, cy = t_x - dot * bx, t_y - dot * by
            cn = math.sqrt(cx * cx + cy * cy)
            if cn < eps:
                cx, cy, cn = 1.0, 0.0, 1.0
            cx, cy = cx / cn, cy / cn
            # Y_c = b_c × X_c
            yx, yy, yz = -bz * cy, bz * cx, bx * cy - by * cx
            yn = math.sqrt(yx * yx + yy * yy + yz * yz)
            if yn < eps:
                yx, yy, yz, yn = 0.0, 1.0, 0.0, 1.0
            yx, yy, yz = yx / yn, yy / yn, yz / yn
            qx, qy, qz, qw = self._quat_from_rotmat(
                [(cx, yx, bx), (cy, yy, by), (0.0, yz, bz)])

        # 油门映射（与 C++ 一致：悬停以下线性到 hover_throttle，以上线性到 1.0）
        weight_n = self._uav_weight * G
        thrust_n = self._uav_weight * acc_norm
        max_n = weight_n * self._thrust_ratio
        if thrust_n <= weight_n:
            throttle = self._hover_throttle * (thrust_n / weight_n)
        else:
            throttle = self._hover_throttle + (1.0 - self._hover_throttle) * (
                thrust_n - weight_n) / (max_n - weight_n)
        throttle = max(0.0, min(1.0, throttle))

        msg = AttitudeTarget()
        msg.orientation.x = qx
        msg.orientation.y = qy
        msg.orientation.z = qz
        msg.orientation.w = qw
        msg.thrust = throttle
        msg.type_mask = 1 + 2 + 4
        msg.header.stamp = node.get_clock().now().to_msg()
        att_pub.publish(msg)

    def _set_result(self, success, message):
        with self._result_lock:
            self._last_result = (success, message)


# ============================================================
# 主界面
# ============================================================
class GroundStationApp:
    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title("地面站控制台 — Ground Station")
        self.root.geometry("960x740")
        self.root.resizable(True, True)
        self.root.configure(bg="#1c1c1c")
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

        self._ros_node = ROS2Node()
        self._ros_node.start()

        self._build_ui()
        self._setup_keyboard()
        self._update_ros_status()
        self._update_pos_display()
        self._poll_result()

    def _update_ros_status(self):
        if self._ros_node.ready:
            self._ros_status.config(text="ROS2: 已连接", fg="#00cc66")
            fc = self._ros_node.fc_connected
            self._fc_status.config(
                text=f"飞控: {'已连接' if fc else '未连接'}",
                fg="#00cc66" if fc else "#ccaa00")
        elif self._ros_node.error:
            self._ros_status.config(
                text=f"ROS2: 错误 — {self._ros_node.error[:60]}", fg="#cc3333")
        else:
            self._ros_status.config(text="ROS2: 连接中...", fg="#ccaa00")
        self.root.after(1000, self._update_ros_status)

    def _poll_result(self):
        result = self._ros_node.last_result
        if result is not None:
            success, message = result
            self._main_status.config(
                text=message, fg="#00cc66" if success else "#cc3333")
            # 航点相关状态同步
            if "航点任务" in message:
                self._wp_status.config(
                    text=message, fg="#00cc66" if success else "#cc3333")
                if "完成" in message or "失败" in message or "无法" in message:
                    self._wp_send_btn.config(state="normal", text="▶ 发送航点")
        self.root.after(500, self._poll_result)

    # ---- UI ----

    def _build_ui(self):
        # ---- 主容器：左右双栏 ----
        main_frame = tk.Frame(self.root, bg="#1c1c1c")
        main_frame.pack(fill="both", expand=True)

        # 左栏 — 原有控制面板
        self._left = tk.Frame(main_frame, bg="#1c1c1c")
        self._left.pack(side="left", fill="both", expand=True)

        # 垂直分隔线
        tk.Frame(main_frame, width=2, bg="#444444").pack(
            side="left", fill="y", padx=(0, 4))

        # 右栏 — 航点面板
        right = tk.Frame(main_frame, bg="#1c1c1c")
        right.pack(side="left", fill="both", expand=True)

        # ================================================
        # 左栏内容
        # ================================================
        left = self._left

        # 标题
        tk.Label(
            left, text="地面站控制台",
            font=("Helvetica", 16, "bold"), fg="#ffffff", bg="#1c1c1c",
        ).pack(pady=(10, 0))

        self._ros_status = tk.Label(
            left, text="ROS2: 连接中...",
            font=("Helvetica", 9), fg="#ccaa00", bg="#1c1c1c",
        )
        self._ros_status.pack(pady=(0, 1))

        self._fc_status = tk.Label(
            left, text="飞控: --",
            font=("Helvetica", 9), fg="#ccaa00", bg="#1c1c1c",
        )
        self._fc_status.pack(pady=(0, 6))

        btn_cfg = {
            "font": ("Helvetica", 11),
            "relief": "flat", "cursor": "hand2",
            "padx": 10, "pady": 5,
        }

        # === 启动按钮区 ===
        tk.Label(
            left, text="── 启动 / Launch ──",
            font=("Helvetica", 9, "bold"), fg="#888888", bg="#1c1c1c",
        ).pack(pady=(0, 4))

        self._add_btn("启动 MAVROS (+频率设置)", self._start_mavros,
                      "#2d5a27", "#3d7a33", btn_cfg)
        self._add_btn("启动 odin1", self._start_odin1,
                      "#3a3a3a", "#4a4a4a", btn_cfg)
        self._add_btn("启动 Rviz", self._start_rviz,
                      "#2d5a27", "#3d7a33", btn_cfg)

        # 分隔
        self._sep()

        # === GPS 原点设置区 ===
        tk.Label(
            left, text="── GPS 原点 / Origin ──",
            font=("Helvetica", 9, "bold"), fg="#888888", bg="#1c1c1c",
        ).pack(pady=(0, 2))

        gp_frame = tk.Frame(left, bg="#1c1c1c")
        gp_frame.pack(pady=(0, 4))

        gp_labels = [("Lat:", "30.2489634"), ("Lon:", "120.2052342"), ("Alt:", "488.0")]
        self._gp_entries = {}
        for i, (lbl, default) in enumerate(gp_labels):
            tk.Label(
                gp_frame, text=lbl, font=("Helvetica", 9),
                fg="#aaaaaa", bg="#1c1c1c",
            ).pack(side="left", padx=(6 if i == 0 else 2, 1))
            ent = tk.Entry(
                gp_frame, width=12 if i < 2 else 7, font=("Helvetica", 9),
                bg="#2a2a2a", fg="#ffffff", insertbackground="#ffffff",
                relief="flat",
            )
            ent.insert(0, default)
            ent.pack(side="left", padx=(0, 4))
            self._gp_entries[lbl] = ent

        tk.Button(
            gp_frame, text="设置原点",
            font=("Helvetica", 9), bg="#1a5a8a", fg="#ffffff",
            activebackground="#2070a0", relief="flat",
            padx=8, pady=3, cursor="hand2",
            command=self._do_set_origin,
        ).pack(side="left", padx=(4, 0))

        self._sep()

        # === 起飞/降落区 ===
        tk.Label(
            left, text="── 飞行 / Flight ──",
            font=("Helvetica", 9, "bold"), fg="#888888", bg="#1c1c1c",
        ).pack(pady=(0, 4))

        flight_frame = tk.Frame(left, bg="#1c1c1c")
        flight_frame.pack()

        self._sep()

        flight_btn = {
            "font": ("Helvetica", 12, "bold"),
            "width": 12, "height": 1,
            "relief": "flat", "cursor": "hand2",
        }
        tk.Button(
            flight_frame, text="起飞", **flight_btn,
            bg="#1a6a3a", fg="#ffffff", activebackground="#208a40",
            command=self._do_takeoff,
        ).pack(side="left", padx=6)

        tk.Button(
            flight_frame, text="降落", **flight_btn,
            bg="#8b3a1a", fg="#ffffff", activebackground="#a04020",
            command=self._do_land,
        ).pack(side="left", padx=6)

        self._sep()

        # === 方向控制区 ===
        tk.Label(
            left, text="── 速度方向控制 / Velocity ──",
            font=("Helvetica", 9, "bold"), fg="#888888", bg="#1c1c1c",
        ).pack(pady=(0, 2))

        tk.Label(
            left,
            text=f"Topic: {PUBLISH_TOPIC}  |  步长: {VELOCITY_SCALE} m/s  |  {PUBLISH_RATE_HZ} Hz",
            font=("Helvetica", 8), fg="#555555", bg="#1c1c1c",
        ).pack(pady=(0, 4))

        # 方向网格
        dir_frame = tk.Frame(left, bg="#1c1c1c")
        dir_frame.pack()

        # 上
        self._mk_dir(dir_frame, "↑ 上 W", 0, 1,
                     0, 0, VELOCITY_SCALE, 0, "#2d5a27", "#3d7a33")
        # 左
        self._mk_dir(dir_frame, "← 左 J", 1, 0,
                     0, VELOCITY_SCALE, 0, 0, "#2d5a27", "#3d7a33")
        # 悬停
        tk.Button(
            dir_frame, text="■\n悬停",
            font=("Helvetica", 12, "bold"), width=6, height=1,
            relief="flat", cursor="hand2",
            bg="#8b6f1a", fg="#ffffff", activebackground="#a08020",
            command=self._hover,
        ).grid(row=1, column=1, padx=3, pady=2)
        # 右
        self._mk_dir(dir_frame, "→ 右 L", 1, 2,
                     0, -VELOCITY_SCALE, 0, 0, "#2d5a27", "#3d7a33")
        # 下
        self._mk_dir(dir_frame, "↓ 下 S", 2, 1,
                     0, 0, -VELOCITY_SCALE, 0, "#2d5a27", "#3d7a33")

        # 前后
        fb_frame = tk.Frame(left, bg="#1c1c1c")
        fb_frame.pack(pady=(4, 0))
        self._mk_dir(fb_frame, "▲ 前 I", 0, 0,
                     VELOCITY_SCALE, 0, 0, 0, "#1a5a8a", "#2070a0")
        self._mk_dir(fb_frame, "▼ 后 K", 0, 1,
                     -VELOCITY_SCALE, 0, 0, 0, "#1a5a8a", "#2070a0")

        # 偏航
        yaw_frame = tk.Frame(left, bg="#1c1c1c")
        yaw_frame.pack(pady=(4, 0))
        self._mk_dir(yaw_frame, "↺ 左转 A", 0, 0,
                     0, 0, 0, VELOCITY_SCALE, "#5a275a", "#6a306a")
        self._mk_dir(yaw_frame, "↻ 右转 D", 0, 1,
                     0, 0, 0, -VELOCITY_SCALE, "#5a275a", "#6a306a")

        # 速度状态
        self._vel_status = tk.Label(
            left,
            text="Vx:  0.00  Vy:  0.00  Vz:  0.00  Yaw:  0.00",
            font=("Courier", 11, "bold"), fg="#00cc66", bg="#1c1c1c",
        )
        self._vel_status.pack(pady=(8, 2))

        # 位姿状态
        self._pos_status = tk.Label(
            left,
            text="X:  --.--  Y:  --.--  Z:  --.--  Yaw:  --.-°",
            font=("Courier", 11, "bold"), fg="#00aacc", bg="#1c1c1c",
        )
        self._pos_status.pack(pady=(0, 6))

        tk.Label(
            left, text="点击累加速度  |  Space 悬停(PD+DOB)  |  W/S上下  I/K前后  J/L左右  A/D偏航",
            font=("Helvetica", 8), fg="#444444", bg="#1c1c1c",
        ).pack(pady=(0, 2))

        self._sep()

        # 关闭所有 / 退出
        close_frame = tk.Frame(left, bg="#1c1c1c")
        close_frame.pack(pady=(0, 4))

        tk.Button(
            close_frame, text="关闭所有进程",
            font=("Helvetica", 10), bg="#8b3a1a", fg="#ffffff",
            activebackground="#a04020", relief="flat",
            padx=8, pady=4, cursor="hand2",
            command=self._terminate_all,
        ).pack(side="left", padx=4)

        tk.Button(
            close_frame, text="退出界面",
            font=("Helvetica", 10), bg="#333333", fg="#888888",
            activebackground="#444444", relief="flat",
            padx=8, pady=4, cursor="hand2",
            command=self._on_close,
        ).pack(side="left", padx=4)

        self._main_status = tk.Label(
            left, text="就绪",
            font=("Helvetica", 9), fg="#555555", bg="#1c1c1c",
        )
        self._main_status.pack(pady=(6, 4))

        # ================================================
        # 右栏内容 — 航点面板
        # ================================================
        self._build_waypoint_panel(right)

    # ---- 辅助 UI 方法 ----

    def _add_btn(self, text, cmd, bg, abg, cfg):
        tk.Button(
            self._left, text=text, bg=bg, fg="#ffffff",
            activebackground=abg, command=cmd, **cfg,
        ).pack(fill="x", padx=20, pady=(0, 4))

    def _sep(self):
        tk.Frame(self._left, height=1, bg="#444444").pack(
            fill="x", padx=20, pady=(4, 6))

    # ---- 航点面板 ----

    def _build_waypoint_panel(self, parent):
        """构建右侧航点输入面板。"""
        import math

        # 标题
        tk.Label(
            parent, text="航点管理 / Waypoints",
            font=("Helvetica", 13, "bold"), fg="#ffffff", bg="#1c1c1c",
        ).pack(pady=(10, 6))

        # ── 输入区 ──
        entry_frame = tk.Frame(parent, bg="#1c1c1c")
        entry_frame.pack(pady=(0, 4))

        # 网格: 4列 — X, Y, Z, Yaw(°)
        labels = ["X (m):", "Y (m):", "Z (m):", "Yaw (°):"]
        self._wp_entries = {}
        for i, lbl in enumerate(labels):
            tk.Label(
                entry_frame, text=lbl, font=("Helvetica", 9),
                fg="#aaaaaa", bg="#1c1c1c",
            ).grid(row=0, column=i*2, padx=(4, 2), pady=2)
            ent = tk.Entry(
                entry_frame, width=8, font=("Helvetica", 11),
                bg="#2a2a2a", fg="#ffffff", insertbackground="#ffffff",
                relief="flat",
            )
            ent.grid(row=0, column=i*2+1, padx=(0, 6), pady=2)
            self._wp_entries[lbl] = ent

        # 默认值
        self._wp_entries["X (m):"].insert(0, "0.0")
        self._wp_entries["Y (m):"].insert(0, "0.0")
        self._wp_entries["Z (m):"].insert(0, "1.0")
        self._wp_entries["Yaw (°):"].insert(0, "0.0")

        # 添加按钮
        tk.Button(
            parent, text="＋ 添加航点",
            font=("Helvetica", 10, "bold"),
            bg="#1a5a8a", fg="#ffffff", activebackground="#2070a0",
            relief="flat", padx=12, pady=4, cursor="hand2",
            command=self._add_waypoint,
        ).pack(pady=(0, 4))

        # ── 飞行模式选择 ──
        mode_frame = tk.Frame(parent, bg="#1c1c1c")
        mode_frame.pack(pady=(0, 4))

        tk.Label(
            mode_frame, text="飞行模式:",
            font=("Helvetica", 9, "bold"), fg="#aaaaaa", bg="#1c1c1c",
        ).pack(side="left", padx=(0, 6))

        self._wp_flight_mode = tk.StringVar(value="直接飞行")
        flight_modes = ["自动避障", "遇到障碍悬停", "直接飞行"]
        self._wp_mode_combo = ttk.Combobox(
            mode_frame,
            textvariable=self._wp_flight_mode,
            values=flight_modes,
            state="readonly",
            width=16,
            font=("Helvetica", 9),
        )
        self._wp_mode_combo.pack(side="left")

        # ── 航点列表 ──
        list_label = tk.Label(
            parent, text="已添加航点列表:",
            font=("Helvetica", 9, "bold"), fg="#888888", bg="#1c1c1c",
        )
        list_label.pack(anchor="w", padx=8, pady=(0, 1))

        list_frame = tk.Frame(parent, bg="#1c1c1c")
        list_frame.pack(fill="both", expand=True, padx=10)

        scrollbar = tk.Scrollbar(list_frame)
        scrollbar.pack(side="right", fill="y")

        self._wp_listbox = tk.Listbox(
            list_frame,
            font=("Courier", 10),
            bg="#2a2a2a", fg="#00cc66",
            selectbackground="#1a5a8a", selectforeground="#ffffff",
            relief="flat", height=6,
            yscrollcommand=scrollbar.set,
        )
        self._wp_listbox.pack(side="left", fill="both", expand=True)
        scrollbar.config(command=self._wp_listbox.yview)

        # ── 列表操作按钮 ──
        list_btn_frame = tk.Frame(parent, bg="#1c1c1c")
        list_btn_frame.pack(pady=(4, 0))

        btn_sm = {
            "font": ("Helvetica", 9),
            "relief": "flat", "cursor": "hand2",
            "padx": 8, "pady": 4,
        }
        tk.Button(
            list_btn_frame, text="删除选中", bg="#5a2727", fg="#ffffff",
            activebackground="#703030", command=self._remove_waypoint, **btn_sm,
        ).pack(side="left", padx=3)
        tk.Button(
            list_btn_frame, text="上移", bg="#3a3a3a", fg="#cccccc",
            activebackground="#4a4a4a", command=self._move_waypoint_up, **btn_sm,
        ).pack(side="left", padx=3)
        tk.Button(
            list_btn_frame, text="下移", bg="#3a3a3a", fg="#cccccc",
            activebackground="#4a4a4a", command=self._move_waypoint_down, **btn_sm,
        ).pack(side="left", padx=3)
        tk.Button(
            list_btn_frame, text="清空全部", bg="#5a2727", fg="#ffffff",
            activebackground="#703030", command=self._clear_waypoints, **btn_sm,
        ).pack(side="left", padx=3)

        # ── 发送按钮 ──
        tk.Frame(parent, height=1, bg="#444444").pack(
            fill="x", padx=10, pady=(6, 6))

        self._wp_send_btn = tk.Button(
            parent, text="▶ 发送航点",
            font=("Helvetica", 12, "bold"),
            bg="#1a6a3a", fg="#ffffff", activebackground="#208a40",
            relief="flat", padx=16, pady=6, cursor="hand2",
            command=self._send_waypoints,
        )
        self._wp_send_btn.pack()

        # 航点状态
        self._wp_status = tk.Label(
            parent, text="就绪 — 请添加航点",
            font=("Helvetica", 9), fg="#555555", bg="#1c1c1c",
        )
        self._wp_status.pack(pady=(6, 6))

    def _add_waypoint(self):
        """从输入框读取并添加航点到列表"""
        import math
        try:
            x = float(self._wp_entries["X (m):"].get())
            y = float(self._wp_entries["Y (m):"].get())
            z = float(self._wp_entries["Z (m):"].get())
            yaw_deg = float(self._wp_entries["Yaw (°):"].get())
            yaw_rad = math.radians(yaw_deg)
        except ValueError:
            messagebox.showwarning("输入错误", "请确保 X, Y, Z, Yaw 均为有效数字。")
            return

        idx = self._wp_listbox.size() + 1
        label = f"#{idx}: X={x:+.1f}  Y={y:+.1f}  Z={z:+.1f}  Yaw={yaw_deg:+.0f}°"
        self._wp_listbox.insert(tk.END, label)
        # 将数据存储为 item 的隐藏属性
        if not hasattr(self, '_wp_data'):
            self._wp_data = []
        self._wp_data.append((x, y, z, yaw_rad))

        self._wp_status.config(
            text=f"已添加 {len(self._wp_data)} 个航点", fg="#00cc66")

    def _remove_waypoint(self):
        """删除选中的航点"""
        sel = self._wp_listbox.curselection()
        if not sel:
            return
        idx = sel[0]
        self._wp_listbox.delete(idx)
        if hasattr(self, '_wp_data') and idx < len(self._wp_data):
            self._wp_data.pop(idx)
        # 重新编号
        self._refresh_waypoint_labels()
        self._wp_status.config(
            text=f"剩余 {len(self._wp_data) if hasattr(self, '_wp_data') else 0} 个航点",
            fg="#ccaa00")

    def _clear_waypoints(self):
        """清空所有航点"""
        self._wp_listbox.delete(0, tk.END)
        if hasattr(self, '_wp_data'):
            self._wp_data.clear()
        self._wp_status.config(text="已清空 — 请添加航点", fg="#555555")

    def _move_waypoint_up(self):
        """上移选中航点"""
        sel = self._wp_listbox.curselection()
        if not sel or sel[0] == 0:
            return
        idx = sel[0]
        self._wp_listbox.selection_clear(0, tk.END)
        # 交换数据
        if hasattr(self, '_wp_data') and idx < len(self._wp_data):
            self._wp_data[idx-1], self._wp_data[idx] = \
                self._wp_data[idx], self._wp_data[idx-1]
        self._refresh_waypoint_labels()
        self._wp_listbox.selection_set(idx - 1)

    def _move_waypoint_down(self):
        """下移选中航点"""
        sel = self._wp_listbox.curselection()
        if not sel:
            return
        idx = sel[0]
        if hasattr(self, '_wp_data') and idx >= len(self._wp_data) - 1:
            return
        self._wp_listbox.selection_clear(0, tk.END)
        if hasattr(self, '_wp_data') and idx < len(self._wp_data) - 1:
            self._wp_data[idx+1], self._wp_data[idx] = \
                self._wp_data[idx], self._wp_data[idx+1]
        self._refresh_waypoint_labels()
        self._wp_listbox.selection_set(idx + 1)

    def _refresh_waypoint_labels(self):
        """重新生成列表项标签以反映编号变化"""
        import math
        self._wp_listbox.delete(0, tk.END)
        if not hasattr(self, '_wp_data'):
            return
        for i, wp in enumerate(self._wp_data):
            label = (
                f"#{i+1}: X={wp[0]:+.1f}  Y={wp[1]:+.1f}  "
                f"Z={wp[2]:+.1f}  Yaw={math.degrees(wp[3]):+.0f}°"
            )
            self._wp_listbox.insert(tk.END, label)

    def _send_waypoints(self):
        """发送航点任务到无人机"""
        if not hasattr(self, '_wp_data') or not self._wp_data:
            messagebox.showwarning("无航点", "请先添加至少一个航点。")
            return

        self._wp_send_btn.config(state="disabled", text="发送中...")
        self._wp_status.config(text="正在发送航点任务...", fg="#ccaa00")
        self._main_status.config(text="航点任务执行中...", fg="#ccaa00")

        self._ros_node.request_waypoints(self._wp_data)

    def _mk_dir(self, parent, text, row, col, vx, vy, vz, vyaw, bg, abg):
        btn = tk.Button(
            parent, text=text,
            font=("Helvetica", 11, "bold"), width=6, height=1,
            relief="flat", cursor="hand2",
            bg=bg, fg="#ffffff", activebackground=abg,
        )
        btn.grid(row=row, column=col, padx=2, pady=1)
        # 累加模型：每次点击（含长按自动重复）累加速度，对齐 C++ keyboard_listener
        btn.configure(command=lambda x=vx, y=vy, z=vz, yw=vyaw: self._on_click(x, y, z, yw))

    # ---- 速度控制 ----

    def _on_click(self, vx, vy, vz, vyaw):
        """累加模型：每次点击 += scale（对齐 C++ keyboard_listener）；同时退出 DOB 悬停"""
        self._ros_node._wp_hovering = False  # 退出航点悬停
        self._ros_node._wp_mission_active = False
        self._ros_node.exit_dob_hover()
        self._ros_node._vel_enabled = True
        self._ros_node.add_velocity(vx, vy, vz, vyaw)
        self._update_vel_display()

    def _hover(self):
        """悬停：进入 PD+DOB 姿态环（对齐 keyboard_vel_controller.cpp MODE_HOVER_CUSTOM）"""
        self._ros_node.request_dob_hover()
        self._ros_node.zero_velocity()  # 清掉累积速度，避免恢复速度模式时残留
        self._update_vel_display()
        self._main_status.config(text="悬停 — PD+DOB 控制中", fg="#888888")

    def _update_vel_display(self):
        vx = self._ros_node._vel_x
        vy = self._ros_node._vel_y
        vz = self._ros_node._vel_z
        vyaw = self._ros_node._vel_yaw
        self._vel_status.config(
            text=f"Vx: {vx:+.2f}  Vy: {vy:+.2f}  Vz: {vz:+.2f}  Yaw: {vyaw:+.2f}")

    def _update_pos_display(self):
        """每 200ms 刷新位姿显示"""
        if self._ros_node.ready:
            px, py, pz, pyaw = self._ros_node.get_position()
            import math
            self._pos_status.config(
                text=f"X: {px:+.2f}  Y: {py:+.2f}  Z: {pz:+.2f}  Yaw: {math.degrees(pyaw):+.1f}°")
        self.root.after(200, self._update_pos_display)

    # ---- 键盘控制（累加模型，对齐 C++ keyboard_listener） ----
    def _setup_keyboard(self):
        self.root.bind("<KeyPress-w>", lambda e: self._on_click(0, 0, VELOCITY_SCALE, 0))
        self.root.bind("<KeyPress-s>", lambda e: self._on_click(0, 0, -VELOCITY_SCALE, 0))
        self.root.bind("<KeyPress-i>", lambda e: self._on_click(VELOCITY_SCALE, 0, 0, 0))
        self.root.bind("<KeyPress-k>", lambda e: self._on_click(-VELOCITY_SCALE, 0, 0, 0))
        self.root.bind("<KeyPress-j>", lambda e: self._on_click(0, VELOCITY_SCALE, 0, 0))
        self.root.bind("<KeyPress-l>", lambda e: self._on_click(0, -VELOCITY_SCALE, 0, 0))
        self.root.bind("<KeyPress-a>", lambda e: self._on_click(0, 0, 0, VELOCITY_SCALE))
        self.root.bind("<KeyPress-d>", lambda e: self._on_click(0, 0, 0, -VELOCITY_SCALE))
        self.root.bind("<KeyPress-space>", lambda e: self._hover())

    # ---- 起飞/降落 ----

    def _do_takeoff(self):
        self._main_status.config(text="正在发送起飞指令...", fg="#ccaa00")
        self._ros_node.request_takeoff(TAKEOFF_ALTITUDE)

    def _do_land(self):
        self._main_status.config(text="正在发送降落指令...", fg="#ccaa00")
        self._ros_node.request_land()

    def _do_set_origin(self):
        """从输入框读取 GPS 坐标并发布原点。"""
        try:
            lat = float(self._gp_entries["Lat:"].get())
            lon = float(self._gp_entries["Lon:"].get())
            alt = float(self._gp_entries["Alt:"].get())
        except ValueError:
            messagebox.showwarning("输入错误", "GPS 坐标必须为有效数字。")
            return
        self._main_status.config(text=f"正在设置 GPS 原点 ({lat:.6f}, {lon:.6f})...", fg="#ccaa00")
        self._ros_node.request_set_gp_origin(lat, lon, alt)

    # ---- 启动按钮 ----

    def _start_mavros(self):
        """启动 MAVROS (apm.launch) 并自动设置消息频率。先杀残留，保证干净启动。"""
        mavros_cmd = 'ros2 launch mavros apm.launch fcu_url:=/dev/ttyTHS1:460800'
        # 合并: 杀残留 → 后台启动 MAVROS → 等待服务就绪 → 设置频率
        full_cmd = (
            'pkill -f "mavros" 2>/dev/null; sleep 1; '
            'pkill -9 -f "mavros" 2>/dev/null; true; '
            'echo "--- 启动 MAVROS ---"; '
            + f'{mavros_cmd} & '
            "MAVROS_PID=$!; "
            "echo '等待 MAVROS 就绪...'; "
            "for i in $(seq 1 15); do "
            "  if ros2 service list 2>/dev/null | grep -q '/mavros/set_message_interval'; then "
            "    echo 'MAVROS 已就绪, 设置消息频率...'; "
            "    ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "
            "      '{message_id: 32, message_rate: 100.0}'; "
            "    ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "
            "      '{message_id: 31, message_rate: 100.0}'; "
            "    ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "
            "      '{message_id: 105, message_rate: 100.0}'; "
            "    echo '消息频率设置完成 (local_pos=100Hz, imu=100Hz, imu_raw=100Hz)'; "
            "    break; "
            "  fi; "
            "  echo \"等待中... ($i/15)\"; "
            "  sleep 2; "
            "done; "
            "wait $MAVROS_PID"
        )
        self._main_status.config(text="正在启动 MAVROS...", fg="#ccaa00")
        ok = run_in_bg(full_cmd, title="MAVROS")
        self._main_status.config(
            text="MAVROS 已在新终端启动" if ok else "启动失败",
            fg="#00cc66" if ok else "#cc3333",
        )

    def _start_odin1(self):
        """启动 odin1 全功能脚本 (odin驱动 + MAVROS + extnav桥接 + 频率设置)。"""
        script = os.path.join(PROJECT_ROOT, "odin1.sh")
        if os.path.isfile(script):
            self._main_status.config(text="正在启动 odin1 (含 MAVROS + extnav)...", fg="#ccaa00")
            os.chmod(script, 0o755)
            run_in_bg(script, title="odin1")
            self._main_status.config(text="odin1 全功能已在新终端启动", fg="#00cc66")
        else:
            messagebox.showinfo("占位", "odin1.sh 尚未创建，此功能预留。")

    def _start_rviz(self):
        cmd = "ros2 launch guided_sim visualize.launch.py"
        self._main_status.config(text="正在启动 Rviz...", fg="#ccaa00")
        ok = run_in_bg(cmd, title="Rviz")
        self._main_status.config(
            text="Rviz 已在新终端启动" if ok else "启动失败",
            fg="#00cc66" if ok else "#cc3333",
        )

    # ---- 关闭 ----

    def _on_close(self):
        self._terminate_all()
        self._ros_node.zero_velocity()
        self._ros_node.stop()
        self.root.quit()

    def _terminate_all(self):
        """关闭所有后台进程（MAVROS, Rviz 等）。"""
        global _background_procs
        killed = 0
        for proc in _background_procs:
            try:
                # SIGTERM 整个进程组
                import signal
                os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
                killed += 1
            except Exception:
                try:
                    proc.terminate()
                    killed += 1
                except Exception:
                    pass
        _background_procs.clear()
        self._main_status.config(
            text=f"已终止 {killed} 个后台进程", fg="#888888")


def _cleanup_tmp():
    for f in list(_tmp_files):
        if os.path.isfile(f):
            os.unlink(f)


def main():
    atexit.register(_cleanup_tmp)

    # 启动前清理残留 MAVROS（Ctrl+C 退出时可能没清干净）
    os.system('pkill -f "mavros" 2>/dev/null; sleep 1; pkill -9 -f "mavros" 2>/dev/null; true')

    root = tk.Tk()
    app = GroundStationApp(root)
    root.mainloop()
    _cleanup_tmp()


if __name__ == "__main__":
    main()
