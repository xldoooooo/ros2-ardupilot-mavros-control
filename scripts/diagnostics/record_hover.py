#!/usr/bin/env python3
"""Manually enable PD+DOB diagnostics by subscribing; record CSV until Ctrl+C."""

import argparse
import csv
import json
import math
from pathlib import Path
import time


def flatten(message, prefix=""):
    """Flatten ROS scalar/vector fields without rounding integer session IDs or timestamps."""
    result = {}
    for name in message.get_fields_and_field_types():
        value = getattr(message, name)
        key = prefix + name
        if hasattr(value, "get_fields_and_field_types"):
            result.update(flatten(value, key + "_"))
        else:
            result[key] = int(value) if isinstance(value, bool) else value
    return result


def sample_row(message):
    """Preserve raw diagnostics and add reference-minus-measurement tracking errors."""
    row = {"schema_version": 1, "receive_wall_s": time.time(),
           "receive_monotonic_s": time.monotonic()}
    row.update(flatten(message))
    for axis in "xyz":
        row["position_error_" + axis] = (
            getattr(message.reference_position, axis) - getattr(message.position, axis))
        row["velocity_error_" + axis] = (
            getattr(message.reference_velocity, axis) - getattr(message.velocity, axis))
    row["position_error_xy"] = math.hypot(row["position_error_x"], row["position_error_y"])
    return row


def main(argv=None):
    """Subscribe only; stream to a new CSV and preserve partial data on Ctrl+C."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True, help="New CSV path; never overwritten")
    parser.add_argument("--prefix", default="/onboard_control", help="Aircraft interface namespace")
    parser.add_argument("--duration", type=float, default=0., help="Wall seconds from launch; 0 until Ctrl+C")
    args, ros_args = parser.parse_known_args(argv)
    if not math.isfinite(args.duration) or args.duration < 0:
        parser.error("--duration must be finite and nonnegative")
    try:
        import rclpy
        from rclpy.executors import ExternalShutdownException
        from rclpy.qos import QoSProfile, ReliabilityPolicy
        from guided_interfaces.msg import TrackingSample
    except ImportError as error:
        parser.exit(2, f"ROS/TrackingSample unavailable: {error}. Source Jazzy and rebuilt install/setup.bash.\n")

    args.output.parent.mkdir(parents=True, exist_ok=True)
    started = last_flush = last_notice = time.monotonic()
    metadata = {"schema_version": 1, "topic": args.prefix.rstrip("/") + "/tracking_sample",
                "started_wall_s": time.time(), "samples": 0, "sessions": [],
                "timing": "Latest-only asynchronous diagnostic publication; sequence gaps expose skipped snapshots."}
    rclpy.init(args=ros_args)
    node = rclpy.create_node("hover_tracking_recorder")
    try:
        with args.output.open("x", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=list(sample_row(TrackingSample())))
            writer.writeheader()
            sessions = set()

            def receive(message):
                """All disk IO stays in the recorder process."""
                row = sample_row(message)
                writer.writerow(row)
                metadata["samples"] += 1
                sessions.add(message.session_id)

            node.create_subscription(TrackingSample, metadata["topic"], receive,
                                     QoSProfile(depth=200, reliability=ReliabilityPolicy.BEST_EFFORT))
            print(f"Recording {metadata['topic']} → {args.output}; Ctrl+C to finish.", flush=True)
            try:
                while rclpy.ok() and (not args.duration or time.monotonic() - started < args.duration):
                    rclpy.spin_once(node, timeout_sec=.1)
                    now = time.monotonic()
                    if now - last_flush >= 1.:
                        stream.flush()
                        last_flush = now
                    if now - last_notice >= 10.:
                        print(f"Samples: {metadata['samples']}. " + ("" if metadata["samples"] else
                              "Waiting: check active control, rebuilt node, namespace and ROS domain."), flush=True)
                        last_notice = now
            except (KeyboardInterrupt, ExternalShutdownException):
                pass
            finally:
                metadata["sessions"] = sorted(sessions)
                metadata["elapsed_s"] = time.monotonic() - started
                args.output.with_suffix(args.output.suffix + ".json").write_text(
                    json.dumps(metadata, ensure_ascii=False, indent=2) + "\n")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    print(f"Saved {metadata['samples']} samples from {len(metadata['sessions'])} source sessions.")
    return 0 if metadata["samples"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
