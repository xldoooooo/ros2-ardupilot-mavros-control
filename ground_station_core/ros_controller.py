"""ROS 2 后台桥接：状态订阅、服务命令队列与互斥飞行模式输出。"""

from __future__ import annotations

import queue
import threading
import time
from collections import deque
from dataclasses import replace

from .config import MESSAGE_INTERVALS, PUBLISH_RATE_HZ, PUBLISH_TOPIC, load_hover_params
from .dob_controller import DobGains
from .flight_modes import KeyboardControlMode, TakeoffLandMode, WaypointFlightMode
from .mode_manager import FlightModeManager
from .models import CommandRequest, CommandResult, FlightMode, VehicleSnapshot
from .ros_services import RosServiceHelper


class _VehicleStateStore:
    """合并多个 ROS 回调，并让 GUI 获取一致的状态快照。"""

    _CONNECTION_STALE_SECONDS = 2.5

    def __init__(self) -> None:
        self._lock = threading.RLock()
        self._snapshot = VehicleSnapshot()
        self._last_state_time = 0.0
        self._last_pose_time = 0.0

    def update_fcu(self, *, connected: bool, armed: bool, mode: str) -> None:
        """更新飞控状态并记录消息新鲜度。"""
        with self._lock:
            self._snapshot = replace(
                self._snapshot,
                connected=connected,
                armed=armed,
                autopilot_mode=mode,
            )
            self._last_state_time = time.monotonic()

    def update_pose(self, *, x: float, y: float, z: float, yaw: float) -> None:
        """更新本地 ENU 位姿。"""
        with self._lock:
            self._snapshot = replace(
                self._snapshot,
                x=x,
                y=y,
                z=z,
                yaw=yaw,
                local_position_valid=True,
            )
            self._last_pose_time = time.monotonic()

    def update_velocity(self, *, vx: float, vy: float, vz: float) -> None:
        """更新本地 ENU 速度。"""
        with self._lock:
            self._snapshot = replace(self._snapshot, vx=vx, vy=vy, vz=vz)

    def mark_disconnected(self) -> None:
        """环境清理后立即清除连接/武装状态，避免使用旧回调数据。"""
        with self._lock:
            self._snapshot = replace(
                self._snapshot,
                connected=False,
                armed=False,
                autopilot_mode="",
                local_position_valid=False,
            )
            self._last_state_time = 0.0
            self._last_pose_time = 0.0

    def snapshot(self) -> VehicleSnapshot:
        """返回快照；若状态话题已超时则将连接状态视为断开。"""
        with self._lock:
            snapshot = self._snapshot
            last_state_time = self._last_state_time
            last_pose_time = self._last_pose_time
        if (
            snapshot.connected
            and time.monotonic() - last_state_time > self._CONNECTION_STALE_SECONDS
        ):
            snapshot = replace(
                snapshot, connected=False, armed=False, autopilot_mode=""
            )
        if (
            snapshot.local_position_valid
            and time.monotonic() - last_pose_time > self._CONNECTION_STALE_SECONDS
        ):
            snapshot = replace(snapshot, local_position_valid=False)
        return snapshot


class GroundStationRosController:
    """在独立线程运行 rclpy，并将三个飞行模式暴露为稳定的 GUI API。"""

    def __init__(self) -> None:
        """初始化状态存储、模式模块、命令队列与结果通道。"""
        self._state = _VehicleStateStore()
        self._mode_manager = FlightModeManager()
        gains = DobGains.from_mapping(load_hover_params())
        self._keyboard_mode = KeyboardControlMode(self._mode_manager, gains)
        self._takeoff_land_mode = TakeoffLandMode(self._mode_manager)
        self._waypoint_mode = WaypointFlightMode(
            self._mode_manager, gains, self._emit_waypoint_result
        )
        self._mode_manager.add_listener(self._on_mode_changed)

        self._running = False
        self._ready = False
        self._error: str | None = None
        self._thread: threading.Thread | None = None
        self._command_queue: queue.Queue[CommandRequest] = queue.Queue()
        self._ticket_lock = threading.Lock()
        self._next_ticket = 1
        self._flight_action_lock = threading.RLock()
        self._flight_action_sequence = 0

        self._result_condition = threading.Condition()
        self._result_sequence = 0
        self._results: deque[CommandResult] = deque(maxlen=256)
        self._ticket_results: dict[int, CommandResult] = {}
        self._zero_velocity_pending = threading.Event()

    @property
    def ready(self) -> bool:
        """指示 ROS 节点是否创建完成。"""
        return self._ready

    @property
    def error(self) -> str | None:
        """返回 ROS 后台线程最后一次致命错误。"""
        return self._error

    @property
    def active_mode(self) -> FlightMode:
        """返回当前互斥飞行模式。"""
        return self._mode_manager.current

    @property
    def velocity(self) -> tuple[float, float, float, float]:
        """返回键盘模式当前累加速度。"""
        return self._keyboard_mode.velocity

    def snapshot(self) -> VehicleSnapshot:
        """返回最新飞行器状态快照。"""
        return self._state.snapshot()

    def start(self, timeout: float = 5.0) -> None:
        """启动 ROS 后台线程，并短暂等待节点就绪或失败。"""
        if self._running:
            return
        self._running = True
        self._ready = False
        self._error = None
        self._thread = threading.Thread(
            target=self._spin, name="ground-station-ros", daemon=True
        )
        self._thread.start()
        deadline = time.monotonic() + timeout
        while not self._ready and self._error is None and time.monotonic() < deadline:
            time.sleep(0.05)

    def stop(self) -> None:
        """停止发布、关闭 rclpy 上下文并等待后台线程退出。"""
        self.reset_controls()
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=5.0)
        self._ready = False
        self._state.mark_disconnected()

    def mark_environment_stopped(self) -> None:
        """由环境管理器在关闭外部 ROS 进程后复位飞行状态。"""
        self.reset_controls()
        self._state.mark_disconnected()

    def reset_controls(self) -> None:
        """取消模式、丢弃未执行命令并安排一次零速度发布。"""
        with self._flight_action_lock:
            self._flight_action_sequence += 1
            self._mode_manager.clear()
            self._keyboard_mode.reset()
            self._waypoint_mode.reset()
        self._zero_velocity_pending.set()
        while True:
            try:
                self._command_queue.get_nowait()
            except queue.Empty:
                break

    # ---- GUI 直接模式按键 ----

    def adjust_velocity(self, vx: float, vy: float, vz: float, yaw_rate: float) -> None:
        """由方向按钮激活键盘模式并累加速度。"""
        with self._flight_action_lock:
            self._flight_action_sequence += 1
            self._keyboard_mode.adjust(vx, vy, vz, yaw_rate)

    def request_hover(self) -> None:
        """由悬停按钮激活键盘模式并抓取当前位置。"""
        with self._flight_action_lock:
            self._flight_action_sequence += 1
            self._keyboard_mode.hover(self.snapshot())

    # ---- 后台服务命令 ----

    def request_takeoff(self, altitude: float) -> int:
        """排队执行起飞流程并返回结果票据。"""
        return self._enqueue("takeoff", float(altitude), flight_mode=True)

    def request_land(self) -> int:
        """排队发送 LAND 模式。"""
        return self._enqueue("land", flight_mode=True)

    def request_set_rates(self) -> int:
        """排队设置 MAVLink 消息频率。"""
        return self._enqueue("set_rates")

    def request_waypoints(self, waypoints) -> int:
        """排队启动航点任务。"""
        return self._enqueue("waypoints", tuple(waypoints), flight_mode=True)

    def request_set_gp_origin(self, latitude: float, longitude: float, altitude: float) -> int:
        """排队发布 GPS 原点。"""
        return self._enqueue("set_gp_origin", (latitude, longitude, altitude))

    def _enqueue(self, name: str, argument=None, *, flight_mode: bool = False) -> int:
        """生成唯一票据并投递命令。"""
        with self._ticket_lock:
            ticket = self._next_ticket
            self._next_ticket += 1
        if flight_mode:
            with self._flight_action_lock:
                self._flight_action_sequence += 1
                action = self._flight_action_sequence
                self._command_queue.put(
                    CommandRequest(ticket, name, argument, flight_action=action)
                )
        else:
            self._command_queue.put(CommandRequest(ticket, name, argument))
        return ticket

    def _flight_action_is_current(self, sequence: int) -> bool:
        """判断排队命令是否仍是用户最后一次飞行模式输入。"""
        with self._flight_action_lock:
            return sequence == self._flight_action_sequence

    def _claim_flight_action(self, command: CommandRequest, activate) -> bool:
        """原子检查输入顺序并激活模式，消除旧队列命令反向覆盖新按键的竞态。"""
        with self._flight_action_lock:
            if command.flight_action != self._flight_action_sequence:
                return False
            activate()
            return True

    def results_after(self, sequence: int) -> list[CommandResult]:
        """返回指定序号之后尚未被 GUI 消费的结果。"""
        with self._result_condition:
            return [result for result in self._results if result.sequence > sequence]

    def wait_for_result(
        self, ticket: int, timeout: float, require_final: bool = True
    ) -> CommandResult | None:
        """供初始化流程和 CLI 测试等待指定票据结果。"""
        deadline = time.monotonic() + timeout
        with self._result_condition:
            while True:
                result = self._ticket_results.get(ticket)
                if result is not None and (result.final or not require_final):
                    return result
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    return None
                self._result_condition.wait(timeout=remaining)

    def _emit_result(
        self,
        ticket: int,
        command: str,
        success: bool,
        message: str,
        final: bool = True,
    ) -> None:
        """保存结果、唤醒同步等待者，并供 GUI 增量读取。"""
        with self._result_condition:
            self._result_sequence += 1
            result = CommandResult(
                self._result_sequence, ticket, command, success, message, final
            )
            self._results.append(result)
            self._ticket_results[ticket] = result
            self._result_condition.notify_all()

    def _emit_waypoint_result(
        self, ticket: int, success: bool, message: str, final: bool
    ) -> None:
        """接收航点模式的异步进度与完成事件。"""
        self._emit_result(ticket, "waypoints", success, message, final)

    def _on_mode_changed(self, previous: FlightMode, current: FlightMode) -> None:
        """模式覆盖时安排零速度帧，清除 MAVROS 中可能残留的速度设定点。"""
        if previous is FlightMode.KEYBOARD and current is not FlightMode.KEYBOARD:
            self._zero_velocity_pending.set()

    # ---- ROS 线程 ----

    def _spin(self) -> None:
        """创建 ROS 实体并循环处理服务命令及当前模式输出。"""
        context = None
        node = None
        try:
            import math
            import rclpy
            from geometry_msgs.msg import PoseStamped, Twist, TwistStamped
            from mavros_msgs.msg import AttitudeTarget, State
            from mavros_msgs.srv import CommandBool, CommandTOL, MessageInterval, SetMode

            # 地面站进程只创建这一个 rclpy 上下文；使用默认上下文可确保
            # rclpy.spin_once() 与节点使用同一个全局执行器（Jazzy 要求）。
            rclpy.init(args=["--ros-args", "--log-level", "warn"])
            context = rclpy.get_default_context()
            node = rclpy.create_node("ground_station_node")
            velocity_publisher = node.create_publisher(Twist, PUBLISH_TOPIC, 10)
            position_publisher = node.create_publisher(
                PoseStamped, "/mavros/setpoint_position/local", 10
            )
            attitude_publisher = node.create_publisher(
                AttitudeTarget, "/mavros/setpoint_raw/attitude", 10
            )

            clients = {
                "takeoff": node.create_client(CommandTOL, "/mavros/cmd/takeoff"),
                "set_mode": node.create_client(SetMode, "/mavros/set_mode"),
                "arming": node.create_client(CommandBool, "/mavros/cmd/arming"),
                "message_interval": node.create_client(
                    MessageInterval, "/mavros/set_message_interval"
                ),
            }

            def state_callback(message: State) -> None:
                self._state.update_fcu(
                    connected=message.connected,
                    armed=message.armed,
                    mode=message.mode,
                )

            def pose_callback(message: PoseStamped) -> None:
                orientation = message.pose.orientation
                sine = 2.0 * (
                    orientation.w * orientation.z + orientation.x * orientation.y
                )
                cosine = 1.0 - 2.0 * (
                    orientation.y * orientation.y + orientation.z * orientation.z
                )
                self._state.update_pose(
                    x=message.pose.position.x,
                    y=message.pose.position.y,
                    z=message.pose.position.z,
                    yaw=math.atan2(sine, cosine),
                )

            def velocity_callback(message: TwistStamped) -> None:
                self._state.update_velocity(
                    vx=message.twist.linear.x,
                    vy=message.twist.linear.y,
                    vz=message.twist.linear.z,
                )

            node.create_subscription(State, "/mavros/state", state_callback, 10)
            best_effort = rclpy.qos.QoSProfile(
                depth=1,
                reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            )
            node.create_subscription(
                PoseStamped,
                "/mavros/local_position/pose",
                pose_callback,
                best_effort,
            )
            node.create_subscription(
                TwistStamped,
                "/mavros/local_position/velocity_local",
                velocity_callback,
                best_effort,
            )

            self._ready = True
            period = 1.0 / PUBLISH_RATE_HZ
            while self._running and context.ok():
                self._process_one_command(node, clients)
                snapshot = self.snapshot()

                if self._zero_velocity_pending.is_set():
                    velocity_publisher.publish(Twist())
                    self._zero_velocity_pending.clear()

                active_mode = self._mode_manager.current
                if active_mode is FlightMode.KEYBOARD:
                    self._keyboard_mode.publish(
                        node,
                        velocity_publisher,
                        attitude_publisher,
                        position_publisher,
                        snapshot,
                    )
                elif active_mode is FlightMode.WAYPOINT:
                    self._waypoint_mode.publish(
                        node, attitude_publisher, position_publisher, snapshot
                    )
                rclpy.spin_once(node, timeout_sec=period)

            velocity_publisher.publish(Twist())
            rclpy.spin_once(node, timeout_sec=0.05)
        except Exception as exc:  # ROS 初始化/运行失败必须传回 GUI，而不是静默退出。
            import traceback

            self._error = str(exc)
            print(f"[GS] ROS2 后台线程异常: {exc}", flush=True)
            traceback.print_exc()
        finally:
            self._ready = False
            if node is not None:
                try:
                    node.destroy_node()
                except Exception:
                    pass
            if context is not None:
                try:
                    import rclpy

                    if context.ok():
                        rclpy.shutdown()
                except Exception:
                    pass

    def _process_one_command(self, node, clients: dict[str, object]) -> None:
        """每个主循环最多执行一条命令，避免 GUI 请求饿死订阅回调。"""
        try:
            command = self._command_queue.get_nowait()
        except queue.Empty:
            return

        helper = RosServiceHelper(node, self.snapshot, lambda: self._running)
        try:
            if command.name == "takeoff":
                if not self._claim_flight_action(
                    command, self._takeoff_land_mode.activate
                ):
                    self._emit_result(
                        command.ticket,
                        command.name,
                        False,
                        "起飞命令已被后续飞行模式按键覆盖",
                    )
                    return
                mode_helper = RosServiceHelper(
                    node,
                    self.snapshot,
                    lambda: self._running
                    and self._mode_manager.current is FlightMode.TAKEOFF_LAND
                    and self._flight_action_is_current(command.flight_action),
                )
                success, message = self._takeoff_land_mode.execute_takeoff(
                    command.argument,
                    mode_helper,
                    self.snapshot,
                    clients["takeoff"],
                    clients["set_mode"],
                    clients["arming"],
                )
                self._emit_result(
                    command.ticket, command.name, success, message, final=True
                )
            elif command.name == "land":
                if not self._claim_flight_action(
                    command, self._takeoff_land_mode.activate
                ):
                    self._emit_result(
                        command.ticket,
                        command.name,
                        False,
                        "降落命令已被后续飞行模式按键覆盖",
                    )
                    return
                mode_helper = RosServiceHelper(
                    node,
                    self.snapshot,
                    lambda: self._running
                    and self._mode_manager.current is FlightMode.TAKEOFF_LAND
                    and self._flight_action_is_current(command.flight_action),
                )
                success, message = self._takeoff_land_mode.execute_land(
                    mode_helper, clients["set_mode"]
                )
                self._emit_result(command.ticket, command.name, success, message)
            elif command.name == "set_rates":
                success, message = self._set_message_rates(
                    helper, clients["message_interval"]
                )
                self._emit_result(command.ticket, command.name, success, message)
            elif command.name == "set_gp_origin":
                success, message = self._set_gp_origin(node, command.argument)
                self._emit_result(command.ticket, command.name, success, message)
            elif command.name == "waypoints":
                self._start_waypoint_command(command, node, clients)
            else:
                self._emit_result(
                    command.ticket, command.name, False, f"未知命令: {command.name}"
                )
        except Exception as exc:
            self._emit_result(
                command.ticket, command.name, False, f"{command.name} 执行异常: {exc}"
            )

    def _set_message_rates(self, helper: RosServiceHelper, client) -> tuple[bool, str]:
        """依次设置本地位置、姿态和高频 IMU 的 MAVLink 消息频率。"""
        from mavros_msgs.srv import MessageInterval

        if not client.wait_for_service(timeout_sec=5.0):
            return False, "MAVROS 消息频率服务不可用"
        for message_id, rate in MESSAGE_INTERVALS:
            request = MessageInterval.Request()
            request.message_id = message_id
            request.message_rate = rate
            future = client.call_async(request)
            if not helper.wait_future(future, 5.0, f"设置消息 {message_id} 频率"):
                return False, f"消息 {message_id} 频率设置超时"
            response = future.result()
            if response is None or not response.success:
                return False, f"飞控拒绝消息 {message_id} 的频率设置"
        return True, "消息频率已设置 (local position/attitude/IMU = 100Hz)"

    def _set_gp_origin(self, node, origin) -> tuple[bool, str]:
        """多次发布 GeographicLib GPS 原点，降低发现阶段丢包概率。"""
        import rclpy
        from geographic_msgs.msg import GeoPointStamped

        latitude, longitude, altitude = (float(value) for value in origin)
        publisher = node.create_publisher(
            GeoPointStamped, "/mavros/global_position/set_gp_origin", 10
        )
        try:
            message = GeoPointStamped()
            message.header.frame_id = "map"
            message.position.latitude = latitude
            message.position.longitude = longitude
            message.position.altitude = altitude
            for _ in range(5):
                message.header.stamp = node.get_clock().now().to_msg()
                publisher.publish(message)
                rclpy.spin_once(node, timeout_sec=0.05)
        finally:
            node.destroy_publisher(publisher)
        return (
            True,
            f"GPS 原点已设置 (lat={latitude:.7f}, lon={longitude:.7f}, alt={altitude:.1f})",
        )

    def _start_waypoint_command(self, command: CommandRequest, node, clients) -> None:
        """确保 GUIDED/武装后启动航点模式，并允许键盘按键中途覆盖。"""
        if not self._claim_flight_action(
            command, lambda: self._mode_manager.activate(FlightMode.WAYPOINT)
        ):
            self._emit_result(
                command.ticket,
                command.name,
                False,
                "航点命令已被后续飞行模式按键覆盖",
            )
            return
        if not command.argument:
            self._mode_manager.clear()
            self._emit_result(command.ticket, command.name, False, "航点列表为空")
            return
        if not self.snapshot().connected:
            self._mode_manager.clear()
            self._emit_result(command.ticket, command.name, False, "飞控未连接，无法执行航点任务")
            return

        helper = RosServiceHelper(
            node,
            self.snapshot,
            lambda: self._running
            and self._mode_manager.current is FlightMode.WAYPOINT
            and self._flight_action_is_current(command.flight_action),
        )
        success, message = helper.ensure_guided(clients["set_mode"])
        if success:
            success, message = helper.ensure_armed(clients["arming"])
        action_is_current = self._flight_action_is_current(command.flight_action)
        if (
            not success
            or self._mode_manager.current is not FlightMode.WAYPOINT
            or not action_is_current
        ):
            if (
                self._mode_manager.current is FlightMode.WAYPOINT
                and action_is_current
            ):
                self._mode_manager.clear()
            message = (
                "航点任务已被其他飞行模式覆盖"
                if not action_is_current
                or (self._mode_manager.current is not FlightMode.IDLE and success)
                else message
            )
            self._emit_result(command.ticket, command.name, False, message)
            return

        self._waypoint_mode.start(command.ticket, command.argument)
        self._emit_result(
            command.ticket,
            command.name,
            True,
            f"航点任务已启动 — 共 {len(command.argument)} 个航点",
            final=False,
        )
