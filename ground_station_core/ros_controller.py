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
    HEARTBEAT_PERIOD_SECONDS,
    INTERFACE_PREFIX,
    INTERFACE_VERSION,
    LEASE_DURATION_MS,
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


class _VehicleStateStore:
    """将机载聚合状态转换为线程安全快照，并处理链路新鲜度。"""

    _STATUS_STALE_SECONDS = 2.5

    def __init__(self, source_id: str) -> None:
        self._source_id = source_id
        self._lock = threading.RLock()
        self._snapshot = VehicleSnapshot()
        self._last_status_time = 0.0

    def update(self, message) -> None:
        """用一条 ControlStatus 原子替换完整快照。"""
        mode = _REMOTE_MODE_MAP.get(int(message.control_mode), FlightMode.IDLE)
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
                message.lease_active and message.lease_owner == self._source_id
            ),
            waypoint_index=message.waypoint_index,
            waypoint_count=message.waypoint_count,
            message_rates_configured=message.message_rates_configured,
            thrust_mode_verified=message.thrust_mode_verified,
            hover_throttle=message.hover_throttle,
            setpoint_conflict=message.setpoint_conflict,
            failsafe_reason=message.failsafe_reason,
            status_message=message.status_message,
            control_rate_hz=message.control_rate_hz,
            max_jitter_ms=message.max_jitter_ms,
            deadline_miss_count=message.deadline_miss_count,
        )
        with self._lock:
            self._snapshot = snapshot
            self._last_status_time = time.monotonic()

    def mark_disconnected(self) -> None:
        """本地仿真清理后立即清除旧机载/飞控连接状态。"""
        with self._lock:
            self._snapshot = VehicleSnapshot()
            self._last_status_time = 0.0

    def snapshot(self) -> VehicleSnapshot:
        """返回快照；状态话题超时后不继续显示虚假的连接或控制权。"""
        with self._lock:
            snapshot = self._snapshot
            last_status_time = self._last_status_time
        if snapshot.onboard_available and (
            time.monotonic() - last_status_time > self._STATUS_STALE_SECONDS
        ):
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
        self._command_queue: queue.Queue[CommandRequest] = queue.Queue()
        self._ticket_lock = threading.Lock()
        self._next_ticket = 1

        self._result_condition = threading.Condition()
        self._result_sequence = 0
        self._results: deque[CommandResult] = deque(maxlen=512)
        self._ticket_results: dict[int, CommandResult] = {}

        self._release_requested = threading.Event()
        self._release_finished = threading.Event()

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
    def lease_error(self) -> str:
        """返回最近一次控制权申请失败原因。"""
        return self._lease_error

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

    def start(self, timeout: float = 5.0) -> None:
        """启动常驻客户端线程，并等待节点就绪或明确失败。"""
        if self._running:
            return
        self._events.info("ros", f"正在启动 ROS 2 客户端：{self._source_id}")
        self._running = True
        self._ready = False
        self._error = None
        self._release_requested.clear()
        self._release_finished.clear()
        self._thread = threading.Thread(
            target=self._spin, name="ground-station-ros-client", daemon=True
        )
        self._thread.start()
        deadline = time.monotonic() + timeout
        while not self._ready and self._error is None and time.monotonic() < deadline:
            time.sleep(0.05)
        if self._ready:
            self._events.info("ros", "ROS 2 客户端已就绪，等待机载控制服务")
        elif self._error is None:
            self._events.warn("ros", f"ROS 2 客户端在 {timeout:.1f}s 内未就绪")

    def stop(self) -> None:
        """先主动释放远端控制租约，再关闭本地 ROS 客户端。"""
        if not self._running:
            return
        self._events.info("ros", "正在释放控制租约并停止 ROS 2 客户端")
        self.release_control(timeout=1.5)
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=5.0)
        self._ready = False
        self._state.mark_disconnected()
        self._events.info("ros", "ROS 2 客户端已停止")

    def enable_control(self) -> None:
        """允许客户端自动申请控制租约，用于开始仿真或连接实机。"""
        self._release_finished.clear()
        self._release_requested.clear()
        self._events.debug("lease", "已允许自动申请机载控制租约")

    def release_control(self, timeout: float = 1.5) -> bool:
        """主动释放租约但保持状态订阅运行，实机进程不受影响。"""
        if not self._running:
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

    def request_waypoints(self, waypoints) -> int:
        """上传航点副本；进度、到达保持和终点状态均由机载端维护。"""
        return self._enqueue("waypoints", tuple(waypoints))

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

    def _spin(self) -> None:
        """创建高层协议实体，处理租约、服务 future 和状态订阅。"""
        context = None
        node = None
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

            rclpy.init(args=["--ros-args", "--log-level", "warn"])
            context = rclpy.get_default_context()
            node = rclpy.create_node("ground_station_client")

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

            status_qos = rclpy.qos.QoSProfile(
                depth=1,
                reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            )
            subscriptions = (
                node.create_subscription(
                    ControlStatus,
                    f"{INTERFACE_PREFIX}/status",
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
                "release_future": None,
            }
            self._ready = True
            last_logged_snapshot = VehicleSnapshot()

            while self._running and context.ok():
                rclpy.spin_once(node, timeout_sec=0.02)
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
            self._ready = False
            self._release_finished.set()
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

    def _update_lease(self, ros_entities: dict[str, object], state: dict) -> None:
        """自动申请/续租控制权，并在关闭前主动释放。"""
        node = ros_entities["node"]
        client = ros_entities["clients"]["lease"]
        now = time.monotonic()

        release_future = state["release_future"]
        if release_future is not None and release_future.done():
            state["granted_hint"] = False
            self._release_finished.set()
            state["release_future"] = None

        if self._release_requested.is_set():
            if state["release_future"] is None and client.service_is_ready():
                request = ros_entities["AcquireControl"].Request()
                state["sequence"] += 1
                request.stamp = node.get_clock().now().to_msg()
                request.source_id = self._source_id
                request.sequence = state["sequence"]
                request.lease_duration_ms = LEASE_DURATION_MS
                request.release = True
                state["release_future"] = client.call_async(request)
            elif state["release_future"] is None:
                state["granted_hint"] = False
                self._release_finished.set()
            return

        snapshot = self.snapshot()
        compatible = (
            snapshot.onboard_available
            and snapshot.interface_version == INTERFACE_VERSION
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
                self._lease_error = "" if state["granted_hint"] else response.message
            except Exception as exc:
                state["granted_hint"] = False
                self._lease_error = f"控制权申请异常: {exc}"
            state["future"] = None

        owns_control = snapshot.control_authority or state["granted_hint"]
        if owns_control and now - state["last_heartbeat"] >= HEARTBEAT_PERIOD_SECONDS:
            heartbeat = ros_entities["ControlHeartbeat"]()
            state["sequence"] += 1
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
            state["sequence"] += 1
            request.stamp = node.get_clock().now().to_msg()
            request.source_id = self._source_id
            request.sequence = state["sequence"]
            request.lease_duration_ms = LEASE_DURATION_MS
            request.release = False
            state["future"] = client.call_async(request)
            state["last_attempt"] = now

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
        if snapshot.interface_version != INTERFACE_VERSION:
            self._emit_result(
                command.ticket,
                command.name,
                False,
                f"接口版本不兼容：期望 {INTERFACE_VERSION}，收到 "
                f"{snapshot.interface_version or '--'}",
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
                request = ros_entities["ExecuteWaypoints"].Request()
                request.stamp = node.get_clock().now().to_msg()
                request.source_id = self._source_id
                request.sequence = command.ticket
                request.ttl_ms = COMMAND_TTL_MS
                for values in command.argument:
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
