"""Small ROS 1 ``rospy`` surface implemented with ROS 2 ``rclpy``.

This compatibility layer keeps the V24 planner wrapper structurally close to
the validated ROS 1 source.  It is deliberately limited to APIs used by the
three adapted Python nodes; it is not a general rospy replacement.
"""
from __future__ import annotations

import math
import signal
import threading
import time as _time
from typing import Any, Callable

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

_node: Node | None = None
_shutdown_hooks: list[Callable[[], Any]] = []
_throttle_last: dict[str, float] = {}
_throttle_lock = threading.Lock()


def init_node(name: str) -> None:
    global _node
    if not rclpy.ok():
        rclpy.init()
    if _node is not None:
        raise RuntimeError("rospy_compat.init_node() may only be called once")
    _node = Node(name, automatically_declare_parameters_from_overrides=True)


def _require_node() -> Node:
    if _node is None:
        raise RuntimeError("rospy_compat.init_node() must be called first")
    return _node


def _parameter_name(name: str) -> str:
    if name.startswith("~"):
        name = name[1:]
    return name.replace("/", ".")


def get_param(name: str, default: Any = None) -> Any:
    node = _require_node()
    normalized = _parameter_name(name)
    if not node.has_parameter(normalized):
        node.declare_parameter(normalized, default)
    value = node.get_parameter(normalized).value
    return default if value is None else value


class Publisher:
    def __init__(self, topic: str, msg_type: type, queue_size: int = 10, latch: bool = False):
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=max(1, int(queue_size)),
            reliability=ReliabilityPolicy.RELIABLE,
            durability=(DurabilityPolicy.TRANSIENT_LOCAL if latch else DurabilityPolicy.VOLATILE),
        )
        self._publisher = _require_node().create_publisher(msg_type, topic, qos)

    def publish(self, message: Any) -> None:
        self._publisher.publish(message)

    def get_num_connections(self) -> int:
        return int(self._publisher.get_subscription_count())


class Subscriber:
    def __init__(self, topic: str, msg_type: type, callback: Callable[[Any], Any], queue_size: int = 10):
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=max(1, int(queue_size)),
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._subscription = _require_node().create_subscription(msg_type, topic, callback, qos)


class Duration:
    def __init__(self, seconds: float = 0.0):
        self.seconds = float(seconds)


class Timer:
    def __init__(self, duration: Duration, callback: Callable[[object], Any]):
        self._timer = _require_node().create_timer(duration.seconds, lambda: callback(None))


class Time:
    @staticmethod
    def now():
        return _require_node().get_clock().now().to_msg()


def stamp_to_seconds(stamp: Any) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1.0e-9


def stamp_to_nanoseconds(stamp: Any) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def time_now_seconds() -> float:
    return _require_node().get_clock().now().nanoseconds * 1.0e-9


def sleep(seconds: float) -> None:
    deadline = _time.monotonic() + max(0.0, float(seconds))
    while rclpy.ok() and _time.monotonic() < deadline:
        _time.sleep(min(0.05, deadline - _time.monotonic()))


def spin_once(timeout_sec: float = 0.1) -> None:
    """Process one callback for helpers that must wait on observed ROS state."""
    rclpy.spin_once(_require_node(), timeout_sec=max(0.0, float(timeout_sec)))


def is_shutdown() -> bool:
    return not rclpy.ok()


def on_shutdown(callback: Callable[[], Any]) -> None:
    _shutdown_hooks.append(callback)


def _format(message: Any, args: tuple[Any, ...]) -> str:
    text = str(message)
    if not args:
        return text
    try:
        return text % args
    except Exception:
        return " ".join((text, *(str(arg) for arg in args)))


def loginfo(message: Any, *args: Any) -> None:
    if not rclpy.ok():
        return
    _require_node().get_logger().info(_format(message, args))


def logwarn(message: Any, *args: Any) -> None:
    if not rclpy.ok():
        return
    _require_node().get_logger().warning(_format(message, args))


def logerr(message: Any, *args: Any) -> None:
    if not rclpy.ok():
        return
    _require_node().get_logger().error(_format(message, args))


def loginfo_throttle(period: float, message: Any, *args: Any) -> None:
    key = str(message)
    now = _time.monotonic()
    with _throttle_lock:
        previous = _throttle_last.get(key, -math.inf)
        if now - previous < float(period):
            return
        _throttle_last[key] = now
    loginfo(message, *args)


def spin() -> None:
    node = _require_node()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        try:
            signal.signal(signal.SIGINT, signal.SIG_IGN)
        except ValueError:  # pragma: no cover - only possible off main thread
            pass
        for callback in reversed(_shutdown_hooks):
            try:
                callback()
            except KeyboardInterrupt:
                # ROS launch may forward the same SIGINT while a shutdown
                # hook is running.  Shutdown remains idempotent.
                pass
            except Exception as exc:  # pragma: no cover - shutdown best effort
                node.get_logger().error(f"shutdown hook failed: {exc}")
        try:
            executor.remove_node(node)
            node.destroy_node()
        except BaseException:
            pass
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except BaseException:
                pass
