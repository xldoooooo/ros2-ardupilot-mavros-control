#!/usr/bin/env python3
"""只订阅机载状态与 rosout，量化任务 09 局域网 DDS 完整性和稳定性。"""

from __future__ import annotations

import argparse
import json
import math
import statistics
import time
from collections import Counter

import rclpy
from guided_interfaces.msg import ControlStatus
from rcl_interfaces.msg import Log as RosLog
from mavros_msgs.msg import AttitudeTarget, State
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


EXPECTED_STATUS_HZ = 10.0  # control.yaml 中机载聚合状态的标称发布频率。


class ReadOnlyCommunicationProbe(Node):
    """仅创建两个订阅，不创建发布者、客户端或参数服务。"""

    def __init__(self, *, subscribe_rosout: bool = True) -> None:
        super().__init__(
            "task09_readonly_communication_probe",
            enable_rosout=False,
            start_parameter_services=False,
        )
        self.receive_times: list[float] = []
        self.publisher_stamps: list[float] = []
        self.semantic_errors: list[str] = []
        self.versions: Counter[str] = Counter()
        self.armed_values: Counter[bool] = Counter()
        self.connected_values: Counter[bool] = Counter()
        self.lease_values: Counter[bool] = Counter()
        self.lease_owners: Counter[str] = Counter()
        self.active_sequences: Counter[int] = Counter()
        self.status_messages: Counter[str] = Counter()
        self.rosout_messages: list[dict[str, object]] = []
        self.mavros_states: list[dict[str, object]] = []
        self.attitude_setpoint_count = 0

        status_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        rosout_qos = QoSProfile(
            depth=1000,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.status_subscription = self.create_subscription(
            ControlStatus,
            "/onboard_control/status",
            self._on_status,
            status_qos,
        )
        self.rosout_subscription = (
            self.create_subscription(
                RosLog,
                "/rosout",
                self._on_rosout,
                rosout_qos,
            )
            if subscribe_rosout
            else None
        )
        self.mavros_state_subscription = self.create_subscription(
            State,
            "/mavros/state",
            self._on_mavros_state,
            status_qos,
        )
        self.attitude_subscription = self.create_subscription(
            AttitudeTarget,
            "/mavros/setpoint_raw/attitude",
            self._on_attitude_setpoint,
            status_qos,
        )

    def _on_status(self, message: ControlStatus) -> None:
        """记录每条完整解码后的状态及发布/接收时间。"""
        self.receive_times.append(time.monotonic())
        stamp = float(message.header.stamp.sec) + (
            float(message.header.stamp.nanosec) / 1_000_000_000.0
        )
        self.publisher_stamps.append(stamp)
        self.versions[message.interface_version] += 1
        self.armed_values[bool(message.armed)] += 1
        self.connected_values[bool(message.fcu_connected)] += 1
        self.lease_values[bool(message.lease_active)] += 1
        self.lease_owners[message.lease_owner or "<none>"] += 1
        self.active_sequences[int(message.active_command_sequence)] += 1
        self.status_messages[message.status_message or "<empty>"] += 1
        self.semantic_errors.extend(self._validate_status(message))

    @staticmethod
    def _validate_status(message: ControlStatus) -> list[str]:
        """访问协议全部关键字段并拒绝非有限数值或越界枚举。"""
        errors: list[str] = []
        numeric_values = {
            "position.x": message.position.x,
            "position.y": message.position.y,
            "position.z": message.position.z,
            "velocity.x": message.velocity.x,
            "velocity.y": message.velocity.y,
            "velocity.z": message.velocity.z,
            "yaw": message.yaw,
            "target_position.x": message.target_position.x,
            "target_position.y": message.target_position.y,
            "target_position.z": message.target_position.z,
            "target_velocity.x": message.target_velocity.x,
            "target_velocity.y": message.target_velocity.y,
            "target_velocity.z": message.target_velocity.z,
            "target_yaw": message.target_yaw,
            "target_yaw_rate": message.target_yaw_rate,
            "hover_throttle": message.hover_throttle,
            "control_rate_hz": message.control_rate_hz,
            "max_jitter_ms": message.max_jitter_ms,
        }
        for name, value in numeric_values.items():
            if not math.isfinite(float(value)):
                errors.append(f"{name}=non-finite")
        if int(message.control_mode) not in range(7):
            errors.append(f"control_mode={message.control_mode}")
        if int(message.waypoint_index) > int(message.waypoint_count):
            errors.append(
                f"waypoint_index={message.waypoint_index}>count={message.waypoint_count}"
            )
        if not message.interface_version:
            errors.append("interface_version=<empty>")
        return errors

    def _on_rosout(self, message: RosLog) -> None:
        """保存远端日志原始来源、等级和文本，供 GUI 事件逐字比对。"""
        logger_name = str(message.name or "unknown").strip("/")
        if logger_name == self.get_name():
            return
        self.rosout_messages.append(
            {
                "name": logger_name,
                "level": int(message.level),
                "message": str(message.msg),
            }
        )

    def _on_mavros_state(self, message: State) -> None:
        """记录 FCU 原始状态，独立核对机载聚合字段。"""
        self.mavros_states.append(
            {
                "connected": bool(message.connected),
                "armed": bool(message.armed),
                "guided": bool(message.guided),
                "mode": str(message.mode),
                "system_status": int(message.system_status),
            }
        )

    def _on_attitude_setpoint(self, _message: AttitudeTarget) -> None:
        """统计姿态 setpoint；未武装只读测试中必须始终为零。"""
        self.attitude_setpoint_count += 1


def _positive_float(value: str) -> float:
    """为 argparse 校验严格正浮点参数。"""
    parsed = float(value)
    if parsed <= 0.0:
        raise argparse.ArgumentTypeError("必须大于 0")
    return parsed


def _gaps(values: list[float]) -> list[float]:
    """返回相邻时间值的差；样本不足时返回空列表。"""
    return [current - previous for previous, current in zip(values, values[1:])]


def _percentile(values: list[float], fraction: float) -> float:
    """用最近秩方法返回小样本也稳定的百分位值。"""
    if not values:
        return 0.0
    ordered = sorted(values)
    index = max(0, min(len(ordered) - 1, math.ceil(len(ordered) * fraction) - 1))
    return ordered[index]


def _counter_dict(counter: Counter) -> dict[str, int]:
    """把布尔/整数键稳定转换为 JSON 字符串键。"""
    return {str(key): int(value) for key, value in counter.items()}


def summarize(
    probe: ReadOnlyCommunicationProbe,
    *,
    duration: float,
    min_delivery_ratio: float,
    max_gap_seconds: float,
    expect_fcu: str,
    require_no_lease: bool,
    require_rosout: bool,
    require_mavros_state: bool,
    require_zero_setpoints: bool,
) -> tuple[dict[str, object], list[str]]:
    """生成可审计指标，并给出不美化结果的失败原因。"""
    publisher_gaps = _gaps(probe.publisher_stamps)
    receive_gaps = _gaps(probe.receive_times)
    count = len(probe.publisher_stamps)
    publisher_span = (
        probe.publisher_stamps[-1] - probe.publisher_stamps[0] if count >= 2 else 0.0
    )
    expected_count = (
        int(round(publisher_span * EXPECTED_STATUS_HZ)) + 1 if count >= 2 else 0
    )
    delivery_ratio = (
        min(1.0, count / expected_count) if expected_count > 0 else 0.0
    )
    unique_errors = sorted(set(probe.semantic_errors))
    failures: list[str] = []

    if count < max(5, int(expected_count * 0.8)):
        failures.append(f"状态样本不足：{count}")
    if delivery_ratio < min_delivery_ratio:
        failures.append(
            f"估算送达率 {delivery_ratio:.4f} < {min_delivery_ratio:.4f}"
        )
    if publisher_gaps and max(publisher_gaps) > max_gap_seconds:
        failures.append(
            f"发布戳最大间隔 {max(publisher_gaps):.4f}s > {max_gap_seconds:.4f}s"
        )
    if receive_gaps and max(receive_gaps) > max_gap_seconds:
        failures.append(
            f"接收最大间隔 {max(receive_gaps):.4f}s > {max_gap_seconds:.4f}s"
        )
    if set(probe.versions) != {"2.0"}:
        failures.append(f"接口版本不唯一为 2.0：{dict(probe.versions)}")
    if probe.armed_values.get(True, 0):
        failures.append(f"观察到 armed=true 共 {probe.armed_values[True]} 条")
    if expect_fcu == "connected" and probe.connected_values.get(False, 0):
        failures.append("要求 FCU connected，但观察到断开状态")
    if expect_fcu == "disconnected" and probe.connected_values.get(True, 0):
        failures.append("要求 FCU disconnected，但观察到连接状态")
    if require_no_lease and probe.lease_values.get(True, 0):
        failures.append("只读测试期间观察到活动控制租约")
    if unique_errors:
        failures.append(f"状态字段语义错误：{unique_errors}")
    if require_rosout and not probe.rosout_messages:
        failures.append("未收到远端 /rosout 消息")
    if require_mavros_state:
        if not probe.mavros_states:
            failures.append("未收到远端 /mavros/state 消息")
        elif any(not item["connected"] for item in probe.mavros_states):
            failures.append("原始 /mavros/state 曾报告 connected=false")
        if any(item["armed"] for item in probe.mavros_states):
            failures.append("原始 /mavros/state 曾报告 armed=true")
    if require_zero_setpoints and probe.attitude_setpoint_count:
        failures.append(
            f"只读窗口收到 {probe.attitude_setpoint_count} 条姿态 setpoint"
        )

    summary = {
        "probe_mode": "subscriptions-only",
        "requested_duration_seconds": duration,
        "status_count": count,
        "publisher_span_seconds": publisher_span,
        "expected_count_from_10hz": expected_count,
        "estimated_delivery_ratio": delivery_ratio,
        "publisher_period_mean_seconds": (
            statistics.fmean(publisher_gaps) if publisher_gaps else 0.0
        ),
        "publisher_period_p95_seconds": _percentile(publisher_gaps, 0.95),
        "publisher_gap_max_seconds": max(publisher_gaps, default=0.0),
        "receive_period_mean_seconds": (
            statistics.fmean(receive_gaps) if receive_gaps else 0.0
        ),
        "receive_period_p95_seconds": _percentile(receive_gaps, 0.95),
        "receive_gap_max_seconds": max(receive_gaps, default=0.0),
        "interface_versions": dict(probe.versions),
        "armed_values": _counter_dict(probe.armed_values),
        "fcu_connected_values": _counter_dict(probe.connected_values),
        "lease_active_values": _counter_dict(probe.lease_values),
        "lease_owners": dict(probe.lease_owners),
        "active_command_sequences": _counter_dict(probe.active_sequences),
        "status_messages": dict(probe.status_messages),
        "semantic_errors": unique_errors,
        "rosout_count": len(probe.rosout_messages),
        "rosout_samples": probe.rosout_messages[:10],
        "mavros_state_count": len(probe.mavros_states),
        "mavros_state_values": {
            "connected": dict(
                Counter(str(item["connected"]) for item in probe.mavros_states)
            ),
            "armed": dict(Counter(str(item["armed"]) for item in probe.mavros_states)),
            "guided": dict(
                Counter(str(item["guided"]) for item in probe.mavros_states)
            ),
            "mode": dict(Counter(str(item["mode"]) for item in probe.mavros_states)),
        },
        "attitude_setpoint_count": probe.attitude_setpoint_count,
        "result": "PASS" if not failures else "FAIL",
        "failures": failures,
    }
    return summary, failures


def main() -> int:
    """运行有界只读观察并以 JSON 输出全部指标。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration", type=_positive_float, default=30.0)
    parser.add_argument("--min-delivery-ratio", type=float, default=0.98)
    parser.add_argument("--max-gap-seconds", type=_positive_float, default=0.35)
    parser.add_argument(
        "--expect-fcu",
        choices=("any", "connected", "disconnected"),
        default="any",
    )
    parser.add_argument("--require-no-lease", action="store_true")
    parser.add_argument("--require-rosout", action="store_true")
    parser.add_argument("--require-mavros-state", action="store_true")
    parser.add_argument("--require-zero-setpoints", action="store_true")
    parser.add_argument(
        "--skip-rosout",
        action="store_true",
        help="不创建 /rosout 订阅，用于隔离跨发行版类型兼容问题",
    )
    args = parser.parse_args()

    rclpy.init(args=None)
    probe = ReadOnlyCommunicationProbe(subscribe_rosout=not args.skip_rosout)
    deadline = time.monotonic() + args.duration
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(probe, timeout_sec=0.1)
    finally:
        summary, failures = summarize(
            probe,
            duration=args.duration,
            min_delivery_ratio=args.min_delivery_ratio,
            max_gap_seconds=args.max_gap_seconds,
            expect_fcu=args.expect_fcu,
            require_no_lease=args.require_no_lease,
            require_rosout=args.require_rosout,
            require_mavros_state=args.require_mavros_state,
            require_zero_setpoints=args.require_zero_setpoints,
        )
        probe.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    print(json.dumps(summary, ensure_ascii=False, indent=2, sort_keys=True))
    return 1 if failures else 0


if __name__ == "__main__":
    raise SystemExit(main())
