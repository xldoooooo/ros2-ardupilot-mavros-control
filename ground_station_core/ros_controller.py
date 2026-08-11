"""地面站 ROS 2 薄客户端：租约、命令协议、机载状态与结果桥接。"""

from __future__ import annotations

import os
import queue
import socket
import threading
import time
import uuid
from collections import deque
from dataclasses import replace

from .config import (
    COMMAND_TTL_MS,
    HARDWARE_DISCOVERY_RANGE,
    HARDWARE_DOMAIN_ID,
    HEARTBEAT_PERIOD_SECONDS,
    INTERFACE_PREFIX,
    INTERFACE_VERSION,
    LEASE_DURATION_MS,
    detect_ros_distro,
)
from .event_log import EventLog
from .models import CommandRequest, CommandResult, FlightMode, VehicleSnapshot


_REMOTE_MODE_MAP = {
    0: FlightMode.IDLE,
    1: FlightMode.TAKEOFF,
    2: FlightMode.KEYBOARD,
    3: FlightMode.HOVER,
    4: FlightMode.WAYPOINT,
    5: FlightMode.LAND,
    6: FlightMode.FAILSAFE,
}

# 连续多个图查询周期均发现重复状态发布者才锁定冲突，过滤 DDS 发现瞬态。
_ENDPOINT_CONFLICT_OBSERVATIONS = 10

# These optional ROS discovery settings may explicitly point at the aircraft.
# Simulation temporarily removes them and restores the exact launch-time values
# when the same GUI switches back to the hardware transport.
_EXPLICIT_DISCOVERY_ENVIRONMENT = (
    "ROS_STATIC_PEERS",
    "ROS_DISCOVERY_SERVER",
)


class _VehicleStateStore:
    """将机载聚合状态转换为线程安全快照，并处理链路新鲜度。"""

    _STATUS_STALE_SECONDS = 2.5

    def __init__(self, source_id: str) -> None:
        """创建权威快照及用于链路诊断的有界接收统计。"""
        self._source_id = source_id
        self._lock = threading.RLock()
        self._snapshot = VehicleSnapshot()
        self._last_status_time = 0.0
        self._status_count = 0
        self._endpoint_conflict = False
        # 仅保留最近 1024 个本地单调时钟时间戳，足够覆盖数十秒 10 Hz 状态流。
        self._status_times: deque[float] = deque(maxlen=1024)

    def update(self, message) -> None:
        """用一条 ControlStatus 原子替换完整快照。"""
        mode = _REMOTE_MODE_MAP.get(int(message.control_mode), FlightMode.IDLE)
        with self._lock:
            endpoint_conflict = self._endpoint_conflict
        snapshot = VehicleSnapshot(
            onboard_available=True,
            interface_version=message.interface_version,
            connected=message.fcu_connected,
            armed=message.armed,
            autopilot_mode=message.autopilot_mode,
            x=message.position.x,
            y=message.position.y,
            z=message.position.z,
            yaw=message.yaw,
            vx=message.velocity.x,
            vy=message.velocity.y,
            vz=message.velocity.z,
            roll=message.roll,
            pitch=message.pitch,
            battery_valid=message.battery_valid,
            battery_voltage=message.battery_voltage,
            battery_current=message.battery_current,
            battery_percentage=message.battery_percentage,
            local_position_valid=message.local_position_valid,
            active_mode=mode,
            controller_active=message.controller_active,
            target_x=message.target_position.x,
            target_y=message.target_position.y,
            target_z=message.target_position.z,
            target_yaw=message.target_yaw,
            target_vx=message.target_velocity.x,
            target_vy=message.target_velocity.y,
            target_vz=message.target_velocity.z,
            target_yaw_rate=message.target_yaw_rate,
            lease_owner=message.lease_owner,
            lease_active=message.lease_active,
            control_authority=(
                not endpoint_conflict
                and message.lease_active
                and message.lease_owner == self._source_id
            ),
            waypoint_index=message.waypoint_index,
            waypoint_count=message.waypoint_count,
            message_rates_configured=message.message_rates_configured,
            thrust_mode_verified=message.thrust_mode_verified,
            hover_throttle=message.hover_throttle,
            endpoint_conflict=endpoint_conflict,
            setpoint_conflict=message.setpoint_conflict,
            failsafe_reason=message.failsafe_reason,
            status_message=message.status_message,
            control_rate_hz=message.control_rate_hz,
            max_jitter_ms=message.max_jitter_ms,
            deadline_miss_count=message.deadline_miss_count,
        )
        received_at = time.monotonic()
        with self._lock:
            self._snapshot = snapshot
            self._last_status_time = received_at
            self._status_count += 1
            self._status_times.append(received_at)

    def mark_disconnected(self) -> None:
        """本地仿真清理后立即清除旧机载/飞控连接状态。"""
        with self._lock:
            self._endpoint_conflict = False
            self._snapshot = VehicleSnapshot()
            self._last_status_time = 0.0
            self._status_count = 0
            self._status_times.clear()

    def set_endpoint_conflict(self, conflict: bool) -> None:
        """记录同一状态接口存在多个发布者，并立即反映到权威快照。"""
        with self._lock:
            self._endpoint_conflict = bool(conflict)
            self._snapshot = replace(
                self._snapshot,
                endpoint_conflict=self._endpoint_conflict,
                control_authority=(
                    self._snapshot.control_authority and not self._endpoint_conflict
                ),
            )

    def observation(self) -> tuple[int, tuple[float, ...]]:
        """返回累计状态数与近期接收时刻，供纯订阅链路检测计算频率/间隔。"""
        with self._lock:
            return self._status_count, tuple(self._status_times)

    def snapshot(self) -> VehicleSnapshot:
        """返回快照；状态话题超时后不继续显示虚假的连接或控制权。"""
        with self._lock:
            snapshot = self._snapshot
            last_status_time = self._last_status_time
            status_times = tuple(self._status_times)
        now = time.monotonic()
        status_age = max(0.0, now - last_status_time) if last_status_time else 0.0
        recent_times = tuple(
            received_at
            for received_at in status_times
            if received_at >= now - 5.0
        )
        status_rate = 0.0
        if len(recent_times) >= 2 and recent_times[-1] > recent_times[0]:
            status_rate = (len(recent_times) - 1) / (
                recent_times[-1] - recent_times[0]
            )
        snapshot = replace(
            snapshot,
            status_rate_hz=status_rate,
            status_age_seconds=status_age,
        )
        if snapshot.onboard_available and status_age > self._STATUS_STALE_SECONDS:
            return replace(
                snapshot,
                onboard_available=False,
                connected=False,
                armed=False,
                local_position_valid=False,
                active_mode=FlightMode.IDLE,
                controller_active=False,
                lease_active=False,
                control_authority=False,
            )
        return snapshot


class GroundStationRosController:
    """仅发送高层意图并显示机载结果，不创建任何 MAVROS setpoint 发布器。"""

    def __init__(
        self, source_id: str | None = None, event_log: EventLog | None = None
    ) -> None:
        """创建客户端身份、命令/结果队列与状态存储。"""
        default_source = (
            f"gcs-{socket.gethostname()}-{os.getpid()}-{uuid.uuid4().hex[:8]}"
        )
        self._source_id = source_id or os.environ.get(
            "GROUND_STATION_SOURCE_ID", default_source
        )
        self._events = event_log or EventLog()
        self._state = _VehicleStateStore(self._source_id)
        self._running = False
        self._ready = False
        self._error: str | None = None
        self._lease_error = ""
        self._thread: threading.Thread | None = None
        self._lifecycle_lock = threading.RLock()
        self._active_domain_id: int | None = None
        self._active_discovery_range = ""
        self._hardware_discovery_environment = {
            name: os.environ.get(name)
            for name in _EXPLICIT_DISCOVERY_ENVIRONMENT
        }
        self._command_queue: queue.Queue[CommandRequest] = queue.Queue()
        self._ticket_lock = threading.Lock()
        self._next_ticket = 1
        # source_id 在一个 GUI 进程内稳定，因此租约序号也必须跨 ROS context
        # 单调递增；仿真/实机切换时回到 1 会被真机判为重复或乱序。
        self._lease_sequence_lock = threading.Lock()
        self._lease_sequence = 0

        self._result_condition = threading.Condition()
        self._result_sequence = 0
        self._results: deque[CommandResult] = deque(maxlen=512)
        self._ticket_results: dict[int, CommandResult] = {}

        self._release_requested = threading.Event()
        self._release_finished = threading.Event()
        # ROS 客户端默认只观察；完整仿真/实机工作流明确开启后才允许申请租约。
        self._control_enabled = threading.Event()
        # 远端 rosout 仅在实机会话或通讯检测中进入 GUI，避免本地仿真日志重复。
        self._remote_logs_enabled = threading.Event()

    @property
    def source_id(self) -> str:
        """返回本次地面站进程的唯一控制来源标识。"""
        return self._source_id

    @property
    def event_log(self) -> EventLog:
        """返回 ROS 客户端与 GUI 共用的结构化日志总线。"""
        return self._events

    @property
    def ready(self) -> bool:
        """指示本地 ROS 2 客户端节点是否已创建。"""
        return self._ready

    @property
    def error(self) -> str | None:
        """返回客户端线程最后一次致命错误。"""
        return self._error

    @property
    def domain_id(self) -> int | None:
        """返回当前 ROS context 的 domain；未启动时返回 None。"""
        return self._active_domain_id

    @property
    def discovery_range(self) -> str:
        """返回当前 ROS context 的发现范围。"""
        return self._active_discovery_range

    @property
    def lease_error(self) -> str:
        """返回最近一次控制权申请失败原因。"""
        return self._lease_error

    @property
    def control_enabled(self) -> bool:
        """指示客户端是否获准申请/续租；新客户端默认仅观察。"""
        return self._control_enabled.is_set()

    @property
    def active_mode(self) -> FlightMode:
        """返回机载服务报告的实际控制模式。"""
        return self.snapshot().active_mode

    @property
    def velocity(self) -> tuple[float, float, float, float]:
        """返回机载运动参考，而非地面站保存的持续控制状态。"""
        snapshot = self.snapshot()
        return (
            snapshot.target_vx,
            snapshot.target_vy,
            snapshot.target_vz,
            snapshot.target_yaw_rate,
        )

    def snapshot(self) -> VehicleSnapshot:
        """返回最新机载聚合状态。"""
        return self._state.snapshot()

    def status_observation(self) -> tuple[int, tuple[float, ...]]:
        """返回状态接收统计；读取本地内存，不产生任何 ROS 传输。"""
        return self._state.observation()

    def start(
        self,
        timeout: float = 5.0,
        *,
        domain_id: int | None = None,
        discovery_range: str | None = None,
    ) -> None:
        """按指定传输隔离启动客户端；domain 变化时安全重建 ROS context。"""
        desired_domain = int(
            os.environ.get("ROS_DOMAIN_ID", HARDWARE_DOMAIN_ID)
            if domain_id is None
            else domain_id
        )
        desired_discovery = str(
            os.environ.get(
                "ROS_AUTOMATIC_DISCOVERY_RANGE", HARDWARE_DISCOVERY_RANGE
            )
            if discovery_range is None
            else discovery_range
        ).strip().upper()
        if not 0 <= desired_domain <= 232:
            raise ValueError("ROS domain 必须在 [0, 232] 范围内")
        if desired_discovery not in {"LOCALHOST", "SUBNET"}:
            raise ValueError("ROS 发现范围只能是 LOCALHOST 或 SUBNET")

        with self._lifecycle_lock:
            same_transport = (
                self._running
                and self._active_domain_id == desired_domain
                and self._active_discovery_range == desired_discovery
            )
            old_thread_alive = self._thread is not None and self._thread.is_alive()
            if not same_transport and (self._running or old_thread_alive):
                self._events.info(
                    "ros",
                    f"正在从 domain {self._active_domain_id} 切换到 "
                    f"domain {desired_domain}",
                )
                self.stop()
                if self._thread is not None and self._thread.is_alive():
                    raise RuntimeError("旧 ROS context 未能在时限内停止，拒绝跨 domain 启动")

            if not same_transport:
                self._apply_transport_environment(
                    desired_domain, desired_discovery
                )
                self._state.mark_disconnected()
                self._discard_queued_commands("ROS 传输环境已切换，旧命令已取消")
                self._active_domain_id = desired_domain
                self._active_discovery_range = desired_discovery
                self._events.info(
                    "ros",
                    f"正在启动 ROS 2 客户端：{self._source_id}；"
                    f"domain={desired_domain}，发现={desired_discovery}",
                )
                self._running = True
                self._ready = False
                self._error = None
                self._release_requested.clear()
                self._release_finished.clear()
                self._control_enabled.clear()
                self._remote_logs_enabled.clear()
                self._thread = threading.Thread(
                    target=self._spin,
                    args=(desired_domain,),
                    name="ground-station-ros-client",
                    daemon=True,
                )
                self._thread.start()

        deadline = time.monotonic() + timeout
        while not self._ready and self._error is None and time.monotonic() < deadline:
            time.sleep(0.05)
        if self._ready:
            self._events.info(
                "ros",
                f"ROS 2 客户端已就绪（domain {desired_domain}），等待机载控制服务",
            )
        elif self._error is None:
            self._events.warn("ros", f"ROS 2 客户端在 {timeout:.1f}s 内未就绪")

    def stop(self) -> None:
        """先主动释放远端控制租约，再关闭本地 ROS 客户端。"""
        with self._lifecycle_lock:
            if not self._running and (
                self._thread is None or not self._thread.is_alive()
            ):
                return
            self._events.info("ros", "正在释放控制租约并停止 ROS 2 客户端")
            self.release_control(timeout=1.5)
            self._running = False
            if self._thread is not None:
                self._thread.join(timeout=5.0)
            self._ready = False
            self._remote_logs_enabled.clear()
            self._control_enabled.clear()
            self._state.mark_disconnected()
            if self._thread is None or not self._thread.is_alive():
                self._active_domain_id = None
                self._active_discovery_range = ""
                self._events.info("ros", "ROS 2 客户端已停止")
            else:
                self._error = "ROS 2 客户端线程停止超时"
                self._events.error("ros", self._error)

    def _apply_transport_environment(
        self, domain_id: int, discovery_range: str
    ) -> None:
        """让当前 context 及随后启动的本地 ROS 子进程继承同一隔离策略。"""
        os.environ["ROS_DOMAIN_ID"] = str(domain_id)
        if detect_ros_distro() == "humble":
            os.environ.pop("ROS_AUTOMATIC_DISCOVERY_RANGE", None)
            os.environ["ROS_LOCALHOST_ONLY"] = (
                "1" if discovery_range == "LOCALHOST" else "0"
            )
        else:
            os.environ["ROS_AUTOMATIC_DISCOVERY_RANGE"] = discovery_range
            os.environ.pop("ROS_LOCALHOST_ONLY", None)
        if discovery_range == "LOCALHOST":
            # domain 隔离 + LOCALHOST 发现范围强制仿真 DDS 只走回环；同时
            # 清除可能直指真机的显式 peer/server，避免绕过自动发现范围。
            for name in _EXPLICIT_DISCOVERY_ENVIRONMENT:
                os.environ.pop(name, None)
        else:
            for name, value in self._hardware_discovery_environment.items():
                if value is None:
                    os.environ.pop(name, None)
                else:
                    os.environ[name] = value

    def _discard_queued_commands(self, reason: str) -> None:
        """跨 domain 时拒绝旧队列，防止仿真命令进入后续实机 context。"""
        while True:
            try:
                command = self._command_queue.get_nowait()
            except queue.Empty:
                return
            self._emit_result(
                command.ticket, command.name, False, reason, final=True
            )

    def enable_control(self) -> None:
        """允许客户端自动申请控制租约；完整仿真/实机工作流显式调用。"""
        self._release_finished.clear()
        self._release_requested.clear()
        self._control_enabled.set()
        self._events.debug("lease", "已允许自动申请机载控制租约")

    def enable_remote_logs(self) -> None:
        """在实机会话或通讯检测中接收远端 ROS 日志，不改变远端状态。"""
        self._remote_logs_enabled.set()
        self._events.debug("ros", "已启用实机 ROS 日志只读接收")

    def disable_remote_logs(self) -> None:
        """停止把后续远端 ROS 日志写入 GUI 事件总线。"""
        self._remote_logs_enabled.clear()

    def release_control(self, timeout: float = 1.5) -> bool:
        """主动释放租约但保持状态订阅运行，实机进程不受影响。"""
        if not self._running:
            self._control_enabled.clear()
            return True
        was_enabled = self._control_enabled.is_set()
        self._control_enabled.clear()
        if not was_enabled and not self.snapshot().control_authority:
            # 仅观察会话从未持有租约，断开时不得发送多余 release 请求。
            self._release_finished.set()
            self._events.debug("lease", "仅观察会话没有控制租约，无需发送释放请求")
            return True
        self._events.info("lease", "正在主动释放机载控制租约")
        self._release_finished.clear()
        self._release_requested.set()
        released = self._release_finished.wait(timeout=timeout)
        if not released:
            self._events.warn("lease", "等待控制租约释放确认超时")
        return released

    def mark_environment_stopped(self) -> None:
        """本地仿真被终止后立刻丢弃其旧状态。"""
        self._state.mark_disconnected()

    def reset_controls(self) -> None:
        """请求机载端取消当前任务；实际安全状态仍由机载端裁决。"""
        self._enqueue("cancel")

    def adjust_velocity(
        self, vx: float, vy: float, vz: float, yaw_rate: float
    ) -> int:
        """发送一次有序速度增量意图，累加与限幅均在机载端执行。"""
        return self._enqueue("motion", (vx, vy, vz, yaw_rate))

    def request_hover(self) -> int:
        """请求机载端抓取当前位置并切换统一 PD+DOB 悬停。"""
        return self._enqueue("hover")

    def request_takeoff(self, altitude: float) -> int:
        """请求机载端完成 GUIDED、武装、起飞与高度确认。"""
        return self._enqueue("takeoff", float(altitude))

    def request_land(self) -> int:
        """请求机载端切换 LAND。"""
        return self._enqueue("land")

    def request_set_rates(self) -> int:
        """请求机载 MAVROS 配置必要高频消息。"""
        return self._enqueue("set_rates")

    def request_waypoints(self, waypoints, strategy: int | object = 0) -> int:
        """上传航点副本与飞行策略；进度/到达/终点由机载端维护。

        strategy 对齐 ExecuteWaypoints.flight_strategy；未实现的策略机载会按直线飞行。
        """
        from .models import WaypointFlightStrategy

        strategy_value = int(WaypointFlightStrategy.from_value(strategy))
        return self._enqueue(
            "waypoints",
            {"waypoints": tuple(waypoints), "strategy": strategy_value},
        )

    def request_set_gp_origin(
        self, latitude: float, longitude: float, altitude: float
    ) -> int:
        """请求机载端向其本机 MAVROS 发布 GPS 原点。"""
        return self._enqueue("set_gp_origin", (latitude, longitude, altitude))

    def _enqueue(self, name: str, argument=None) -> int:
        """为所有高层输入分配同一单调序号并投递给 ROS 线程。"""
        with self._ticket_lock:
            ticket = self._next_ticket
            self._next_ticket += 1
        self._command_queue.put(CommandRequest(ticket, name, argument))
        self._events.debug("command", f"命令已排队：{name} (ticket={ticket})")
        return ticket

    def results_after(self, sequence: int) -> list[CommandResult]:
        """返回指定本地结果序号之后的增量事件。"""
        with self._result_condition:
            return [result for result in self._results if result.sequence > sequence]

    def wait_for_result(
        self, ticket: int, timeout: float, require_final: bool = True
    ) -> CommandResult | None:
        """供初始化流程与 CLI 等待机载命令的终态或进度。"""
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
        """统一保存本地传输错误与远端可靠命令结果。"""
        with self._result_condition:
            self._result_sequence += 1
            result = CommandResult(
                self._result_sequence, ticket, command, success, message, final
            )
            self._results.append(result)
            self._ticket_results[ticket] = result
            self._result_condition.notify_all()
        detail = f"{command} (ticket={ticket})：{message}"
        if not success:
            self._events.error("command", detail)
        elif final or command != "motion":
            self._events.info("command", detail)
        else:
            self._events.debug("command", detail)

    def _spin(self, domain_id: int) -> None:
        """在独立 ROS context 中处理租约、服务 future 和状态订阅。"""
        context = None
        node = None
        executor = None
        try:
            import rclpy
            from guided_interfaces.msg import (
                CommandResult as RemoteCommandResult,
                ControlHeartbeat,
                ControlStatus,
                MotionIntent,
                Waypoint,
            )
            from guided_interfaces.srv import (
                AcquireControl,
                ExecuteWaypoints,
                FlightCommand,
                SetGpsOrigin,
            )
            from rcl_interfaces.msg import Log as RosLog
            from rclpy.executors import SingleThreadedExecutor

            context = rclpy.Context()
            rclpy.init(
                args=["--ros-args", "--log-level", "warn"],
                context=context,
                domain_id=domain_id,
                signal_handler_options=rclpy.SignalHandlerOptions.NO,
            )
            node = rclpy.create_node(
                "ground_station_client", context=context
            )
            executor = SingleThreadedExecutor(context=context)
            executor.add_node(node)

            heartbeat_publisher = node.create_publisher(
                ControlHeartbeat, f"{INTERFACE_PREFIX}/heartbeat", 10
            )
            motion_publisher = node.create_publisher(
                MotionIntent, f"{INTERFACE_PREFIX}/motion_intent", 20
            )
            clients = {
                "lease": node.create_client(
                    AcquireControl, f"{INTERFACE_PREFIX}/acquire_control"
                ),
                "flight": node.create_client(
                    FlightCommand, f"{INTERFACE_PREFIX}/flight_command"
                ),
                "waypoints": node.create_client(
                    ExecuteWaypoints, f"{INTERFACE_PREFIX}/execute_waypoints"
                ),
                "origin": node.create_client(
                    SetGpsOrigin, f"{INTERFACE_PREFIX}/set_gps_origin"
                ),
            }

            def status_callback(message: ControlStatus) -> None:
                self._state.update(message)

            def result_callback(message: RemoteCommandResult) -> None:
                if message.source_id != self._source_id:
                    return
                successful = message.status in (
                    RemoteCommandResult.STATUS_RUNNING,
                    RemoteCommandResult.STATUS_SUCCEEDED,
                )
                self._emit_result(
                    int(message.sequence),
                    message.command,
                    successful,
                    message.message,
                    message.final,
                )

            def rosout_callback(message: RosLog) -> None:
                self._ingest_remote_rosout(message)

            status_qos = rclpy.qos.QoSProfile(
                depth=1,
                reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            )
            rosout_qos = rclpy.qos.QoSProfile(
                depth=1000,
                reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
                durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL,
            )
            status_topic = f"{INTERFACE_PREFIX}/status"
            subscriptions = (
                node.create_subscription(
                    ControlStatus,
                    status_topic,
                    status_callback,
                    status_qos,
                ),
                node.create_subscription(
                    RemoteCommandResult,
                    f"{INTERFACE_PREFIX}/command_result",
                    result_callback,
                    50,
                ),
            )
            rosout_subscription = None

            ros_entities = {
                "node": node,
                "clients": clients,
                "motion_publisher": motion_publisher,
                "heartbeat_publisher": heartbeat_publisher,
                "AcquireControl": AcquireControl,
                "ControlHeartbeat": ControlHeartbeat,
                "MotionIntent": MotionIntent,
                "Waypoint": Waypoint,
                "FlightCommand": FlightCommand,
                "ExecuteWaypoints": ExecuteWaypoints,
                "SetGpsOrigin": SetGpsOrigin,
                "subscriptions": subscriptions,
            }
            pending_services: dict[int, tuple] = {}
            lease_state = {
                "sequence": 0,
                "future": None,
                "last_attempt": 0.0,
                "last_heartbeat": 0.0,
                "granted_hint": False,
                "grant_hint_deadline": 0.0,
                "release_future": None,
            }
            self._ready = True
            last_logged_snapshot = VehicleSnapshot()
            endpoint_conflict_observations = 0

            while self._running and context.ok():
                # 仅实机会话/通讯检测创建 rosout 订阅；TRANSIENT_LOCAL 会补发远端
                # 节点启动日志，仿真会话则完全不订阅以避免本地 tee 日志重复。
                if self._remote_logs_enabled.is_set() and rosout_subscription is None:
                    rosout_subscription = node.create_subscription(
                        RosLog,
                        "/rosout",
                        rosout_callback,
                        rosout_qos,
                    )
                elif (
                    not self._remote_logs_enabled.is_set()
                    and rosout_subscription is not None
                ):
                    node.destroy_subscription(rosout_subscription)
                    rosout_subscription = None
                executor.spin_once(timeout_sec=0.02)
                if node.count_publishers(status_topic) > 1:
                    endpoint_conflict_observations += 1
                else:
                    endpoint_conflict_observations = 0
                self._state.set_endpoint_conflict(
                    endpoint_conflict_observations
                    >= _ENDPOINT_CONFLICT_OBSERVATIONS
                )
                current_snapshot = self.snapshot()
                self._log_status_transitions(last_logged_snapshot, current_snapshot)
                last_logged_snapshot = current_snapshot
                self._poll_pending_services(pending_services)
                self._update_lease(ros_entities, lease_state)
                self._process_one_command(ros_entities, pending_services)

            self._release_finished.set()
        except Exception as exc:
            import traceback

            self._error = str(exc)
            self._events.error("ros", f"ROS 2 客户端线程异常：{exc}")
            print(f"[GS] ROS2 客户端线程异常: {exc}", flush=True)
            traceback.print_exc()
        finally:
            self._running = False
            self._ready = False
            self._release_finished.set()
            if executor is not None:
                try:
                    executor.shutdown(timeout_sec=1.0)
                except Exception:
                    pass
            if node is not None:
                try:
                    node.destroy_node()
                except Exception:
                    pass
            if context is not None:
                try:
                    if context.ok():
                        context.shutdown()
                except Exception:
                    pass

    def _log_status_transitions(
        self, previous: VehicleSnapshot, current: VehicleSnapshot
    ) -> None:
        """在 ROS 源端将关键权威状态变化转换为已分级日志事件。"""
        if previous.onboard_available != current.onboard_available:
            if current.onboard_available:
                self._events.info(
                    "onboard",
                    f"机载控制服务已发现，接口版本 {current.interface_version or '--'}",
                )
            else:
                self._events.warn("onboard", "机载状态超时，连接已标记为不可用")
        if (
            current.interface_version
            and current.interface_version != INTERFACE_VERSION
            and current.interface_version != previous.interface_version
        ):
            self._events.error(
                "onboard",
                f"接口版本不兼容：地面站 {INTERFACE_VERSION} / "
                f"机载端 {current.interface_version}",
            )
        if previous.connected != current.connected:
            if current.connected:
                self._events.info("flight-controller", "飞控链路已连接")
            else:
                self._events.warn("flight-controller", "飞控链路已断开")
        if previous.control_authority != current.control_authority:
            if current.control_authority:
                self._events.info("lease", "已获得机载控制权")
            else:
                owner = current.lease_owner or "无"
                self._events.warn("lease", f"已失去机载控制权（当前持有者：{owner}）")
        if previous.endpoint_conflict != current.endpoint_conflict:
            if current.endpoint_conflict:
                self._events.error(
                    "ros",
                    "检测到多个 /onboard_control/status 发布者，已停止全部控制传输",
                )
            else:
                self._events.info("ros", "机载状态发布者冲突已解除")
        if previous.armed != current.armed:
            if current.armed:
                self._events.warn("flight-controller", "飞行器已武装")
            else:
                self._events.info("flight-controller", "飞行器已解除武装")
        if previous.active_mode is not current.active_mode:
            if current.active_mode is FlightMode.FAILSAFE:
                self._events.error("onboard", "机载控制进入失联保护模式")
            else:
                self._events.info("onboard", f"控制模式：{current.active_mode.value}")
        if not previous.setpoint_conflict and current.setpoint_conflict:
            self._events.error("onboard", "检测到姿态 setpoint 多发布者冲突")
        if current.failsafe_reason and current.failsafe_reason != previous.failsafe_reason:
            self._events.error("onboard", f"失联保护原因：{current.failsafe_reason}")
        if current.status_message and current.status_message != previous.status_message:
            self._events.debug("onboard", current.status_message)
        if current.deadline_miss_count > previous.deadline_miss_count:
            self._events.warn(
                "controller",
                f"控制周期超期累计 {current.deadline_miss_count} 次，"
                f"最大抖动 {current.max_jitter_ms:.3f} ms",
            )

    def _ingest_remote_rosout(self, message) -> None:
        """原样接收实机会话 rosout 文本，并映射 ROS 原生严重度。"""
        if not self._remote_logs_enabled.is_set():
            return
        logger_name = str(getattr(message, "name", "") or "unknown").strip("/")
        if logger_name == "ground_station_client":
            return
        ros_level = int(getattr(message, "level", 20))
        if ros_level >= 40:
            level = 40
        elif ros_level >= 30:
            level = 30
        elif ros_level >= 20:
            level = 20
        else:
            level = 10
        self._events.emit(
            level,
            f"remote-rosout:{logger_name}",
            str(getattr(message, "msg", "")),
        )

    def _update_lease(self, ros_entities: dict[str, object], state: dict) -> None:
        """仅在明确授权后申请/续租，并在关闭前主动释放。"""
        node = ros_entities["node"]
        client = ros_entities["clients"]["lease"]
        now = time.monotonic()

        release_future = state["release_future"]
        if release_future is not None and release_future.done():
            state["granted_hint"] = False
            state["grant_hint_deadline"] = 0.0
            self._release_finished.set()
            self._release_requested.clear()
            state["release_future"] = None

        snapshot = self.snapshot()
        compatible = (
            snapshot.onboard_available
            and snapshot.interface_version == INTERFACE_VERSION
            and not snapshot.endpoint_conflict
        )
        if snapshot.interface_version and snapshot.interface_version != INTERFACE_VERSION:
            self._lease_error = (
                f"接口版本不兼容：地面站 {INTERFACE_VERSION} / "
                f"机载端 {snapshot.interface_version}"
            )
            return

        lease_future = state["future"]
        if lease_future is not None and lease_future.done():
            try:
                response = lease_future.result()
                state["granted_hint"] = bool(response and response.granted)
                state["grant_hint_deadline"] = (
                    now + 1.0 if state["granted_hint"] else 0.0
                )
                self._lease_error = "" if state["granted_hint"] else response.message
            except Exception as exc:
                state["granted_hint"] = False
                state["grant_hint_deadline"] = 0.0
                self._lease_error = f"控制权申请异常: {exc}"
            state["future"] = None

        if snapshot.endpoint_conflict:
            self._lease_error = "检测到多个机载状态发布者，控制传输已禁用"
            self._control_enabled.clear()
            state["granted_hint"] = False
            state["grant_hint_deadline"] = 0.0
            self._release_requested.clear()
            self._release_finished.set()
            return

        # 服务响应先于状态消息到达时短暂相信 granted；若机载进程已重启且
        # 一秒后仍未报告本客户端持权，则重新申请租约，避免永久只发无效心跳。
        if (
            state["granted_hint"]
            and snapshot.onboard_available
            and not snapshot.control_authority
            and now >= state.get("grant_hint_deadline", 0.0)
        ):
            state["granted_hint"] = False
            state["grant_hint_deadline"] = 0.0

        if self._release_requested.is_set():
            # 等待正在途中的 acquire 完成，避免刚获租约便在本地误判为无需释放。
            if state["future"] is not None:
                return
            owns_control = snapshot.control_authority or state["granted_hint"]
            if (
                owns_control
                and state["release_future"] is None
                and client.service_is_ready()
            ):
                request = ros_entities["AcquireControl"].Request()
                state["sequence"] = self._next_lease_sequence(state["sequence"])
                request.stamp = node.get_clock().now().to_msg()
                request.source_id = self._source_id
                request.sequence = state["sequence"]
                request.lease_duration_ms = LEASE_DURATION_MS
                request.release = True
                state["release_future"] = client.call_async(request)
            elif state["release_future"] is None:
                state["granted_hint"] = False
                state["grant_hint_deadline"] = 0.0
                self._release_requested.clear()
                self._release_finished.set()
            return

        # 默认仅观察：即便发现兼容服务，也不申请租约或发布心跳。
        if not self._control_enabled.is_set():
            return

        owns_control = snapshot.control_authority or state["granted_hint"]
        if owns_control and now - state["last_heartbeat"] >= HEARTBEAT_PERIOD_SECONDS:
            heartbeat = ros_entities["ControlHeartbeat"]()
            state["sequence"] = self._next_lease_sequence(state["sequence"])
            heartbeat.header.stamp = node.get_clock().now().to_msg()
            heartbeat.source_id = self._source_id
            heartbeat.sequence = state["sequence"]
            heartbeat.lease_duration_ms = LEASE_DURATION_MS
            ros_entities["heartbeat_publisher"].publish(heartbeat)
            state["last_heartbeat"] = now

        if (
            compatible
            and not owns_control
            and state["future"] is None
            and client.service_is_ready()
            and now - state["last_attempt"] >= 1.0
        ):
            request = ros_entities["AcquireControl"].Request()
            state["sequence"] = self._next_lease_sequence(state["sequence"])
            request.stamp = node.get_clock().now().to_msg()
            request.source_id = self._source_id
            request.sequence = state["sequence"]
            request.lease_duration_ms = LEASE_DURATION_MS
            request.release = False
            state["future"] = client.call_async(request)
            state["last_attempt"] = now

    def _next_lease_sequence(self, context_sequence: int = 0) -> int:
        """分配跨 context 单调租约序号，并兼容现有上下文的局部计数。"""
        with self._lease_sequence_lock:
            self._lease_sequence = max(
                self._lease_sequence, int(context_sequence)
            ) + 1
            return self._lease_sequence

    def _process_one_command(
        self, ros_entities: dict[str, object], pending_services: dict[int, tuple]
    ) -> None:
        """每轮最多发送一个高层请求，保持客户端事件循环可响应。"""
        try:
            command = self._command_queue.get_nowait()
        except queue.Empty:
            return

        snapshot = self.snapshot()
        if not snapshot.onboard_available:
            self._emit_result(
                command.ticket, command.name, False, "机载控制服务不可用"
            )
            return
        if snapshot.endpoint_conflict:
            self._emit_result(
                command.ticket,
                command.name,
                False,
                "检测到多个机载状态发布者，拒绝发送任何命令",
            )
            return
        if snapshot.interface_version != INTERFACE_VERSION:
            self._emit_result(
                command.ticket,
                command.name,
                False,
                f"接口版本不兼容：期望 {INTERFACE_VERSION}，收到 "
                f"{snapshot.interface_version or '--'}",
            )
            return
        if not self._control_enabled.is_set():
            self._emit_result(
                command.ticket,
                command.name,
                False,
                "客户端尚未启用控制会话",
            )
            return
        if not snapshot.control_authority:
            owner = snapshot.lease_owner or "无"
            self._emit_result(
                command.ticket,
                command.name,
                False,
                f"地面站未持有控制权（当前持有者: {owner}）",
            )
            return

        node = ros_entities["node"]
        clients = ros_entities["clients"]
        try:
            if command.name == "motion":
                message = ros_entities["MotionIntent"]()
                message.header.stamp = node.get_clock().now().to_msg()
                message.source_id = self._source_id
                message.sequence = command.ticket
                message.ttl_ms = COMMAND_TTL_MS
                (
                    message.velocity_delta.x,
                    message.velocity_delta.y,
                    message.velocity_delta.z,
                    message.yaw_rate_delta,
                ) = (float(value) for value in command.argument)
                ros_entities["motion_publisher"].publish(message)
                self._emit_result(
                    command.ticket,
                    command.name,
                    True,
                    "运动意图已发送，等待机载确认",
                    final=False,
                )
                return

            if command.name == "waypoints":
                client = clients["waypoints"]
                if not client.service_is_ready():
                    raise RuntimeError("机载航点服务不可用")
                # 兼容旧调用：纯航点序列；新调用：{waypoints, strategy}。
                payload = command.argument
                if isinstance(payload, dict):
                    waypoint_values = payload.get("waypoints", ())
                    strategy_value = int(payload.get("strategy", 0))
                else:
                    waypoint_values = payload
                    strategy_value = 0
                request = ros_entities["ExecuteWaypoints"].Request()
                request.stamp = node.get_clock().now().to_msg()
                request.source_id = self._source_id
                request.sequence = command.ticket
                request.ttl_ms = COMMAND_TTL_MS
                request.flight_strategy = strategy_value
                for values in waypoint_values:
                    waypoint = ros_entities["Waypoint"]()
                    waypoint.position.x = float(values[0])
                    waypoint.position.y = float(values[1])
                    waypoint.position.z = float(values[2])
                    waypoint.yaw = float(values[3])
                    request.waypoints.append(waypoint)
                future = client.call_async(request)
            elif command.name == "set_gp_origin":
                client = clients["origin"]
                if not client.service_is_ready():
                    raise RuntimeError("机载 GPS 原点服务不可用")
                request = ros_entities["SetGpsOrigin"].Request()
                request.stamp = node.get_clock().now().to_msg()
                request.source_id = self._source_id
                request.sequence = command.ticket
                request.ttl_ms = COMMAND_TTL_MS
                (
                    request.origin.latitude,
                    request.origin.longitude,
                    request.origin.altitude,
                ) = (float(value) for value in command.argument)
                future = client.call_async(request)
            else:
                client = clients["flight"]
                if not client.service_is_ready():
                    raise RuntimeError("机载飞行命令服务不可用")
                command_codes = {
                    "takeoff": ros_entities["FlightCommand"].Request.COMMAND_TAKEOFF,
                    "land": ros_entities["FlightCommand"].Request.COMMAND_LAND,
                    "hover": ros_entities["FlightCommand"].Request.COMMAND_HOVER,
                    "cancel": ros_entities["FlightCommand"].Request.COMMAND_CANCEL,
                    "set_rates": (
                        ros_entities["FlightCommand"].Request.COMMAND_CONFIGURE_RATES
                    ),
                }
                if command.name not in command_codes:
                    raise RuntimeError(f"未知命令: {command.name}")
                request = ros_entities["FlightCommand"].Request()
                request.stamp = node.get_clock().now().to_msg()
                request.source_id = self._source_id
                request.sequence = command.ticket
                request.ttl_ms = COMMAND_TTL_MS
                request.command = command_codes[command.name]
                request.value = float(command.argument or 0.0)
                future = client.call_async(request)

            pending_services[command.ticket] = (
                future,
                command,
                time.monotonic() + 6.0,
            )
        except Exception as exc:
            self._emit_result(
                command.ticket, command.name, False, f"发送 {command.name} 失败: {exc}"
            )

    def _poll_pending_services(self, pending_services: dict[int, tuple]) -> None:
        """将服务级拒绝/超时转换为 GUI 可见终态；执行结果由机载话题返回。"""
        now = time.monotonic()
        for ticket, (future, command, deadline) in tuple(pending_services.items()):
            if future.done():
                try:
                    response = future.result()
                    accepted = bool(response and response.accepted)
                    if not accepted:
                        message = response.message if response else "机载服务无响应"
                        self._emit_result(
                            ticket, command.name, False, message, final=True
                        )
                except Exception as exc:
                    self._emit_result(
                        ticket,
                        command.name,
                        False,
                        f"机载服务调用异常: {exc}",
                        final=True,
                    )
                pending_services.pop(ticket, None)
            elif now >= deadline:
                self._emit_result(
                    ticket,
                    command.name,
                    False,
                    "等待机载服务接收确认超时",
                    final=True,
                )
                pending_services.pop(ticket, None)
