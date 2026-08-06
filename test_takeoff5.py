#!/usr/bin/env python3
"""共享起飞模式的命令行回归入口，不再复制地面站起飞实现。"""

from __future__ import annotations

import argparse
import sys
import time

from ground_station_core.ros_controller import GroundStationRosController


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    """解析目标高度与连接等待时间。"""
    parser = argparse.ArgumentParser(description="测试地面站共享起飞模式")
    parser.add_argument("altitude", nargs="?", type=float, default=1.0)
    parser.add_argument("--connection-timeout", type=float, default=20.0)
    parser.add_argument("--takeoff-timeout", type=float, default=65.0)
    return parser.parse_args(argv)


def wait_for_connection(
    controller: GroundStationRosController, timeout: float
) -> bool:
    """等待地面站节点收到新鲜的 MAVROS connected 状态。"""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if controller.snapshot().connected:
            return True
        time.sleep(0.1)
    return controller.snapshot().connected


def main(argv: list[str] | None = None) -> int:
    """设置消息频率并执行与 GUI 完全相同的起飞流程。"""
    args = parse_args(argv)
    if args.altitude <= 0.0:
        print("[TEST] FAIL: altitude must be greater than zero", flush=True)
        return 2

    controller = GroundStationRosController()
    controller.start()
    try:
        if not controller.ready:
            print(f"[TEST] FAIL: ROS node unavailable: {controller.error}", flush=True)
            return 1
        print("[TEST] waiting for MAVROS connection...", flush=True)
        if not wait_for_connection(controller, args.connection_timeout):
            print("[TEST] FAIL: MAVROS flight controller not connected", flush=True)
            return 1

        rate_ticket = controller.request_set_rates()
        rate_result = controller.wait_for_result(rate_ticket, timeout=20.0)
        if rate_result is None or not rate_result.success:
            detail = rate_result.message if rate_result is not None else "timeout"
            print(f"[TEST] FAIL: message intervals: {detail}", flush=True)
            return 1
        print(f"[TEST] {rate_result.message}", flush=True)

        print(f"[TEST] takeoff target={args.altitude:.2f}m", flush=True)
        takeoff_ticket = controller.request_takeoff(args.altitude)
        result = controller.wait_for_result(
            takeoff_ticket, timeout=args.takeoff_timeout
        )
        if result is None:
            print("[TEST] FAIL: takeoff result timeout", flush=True)
            return 1
        print(f"[TEST] {result.message}", flush=True)
        if not result.success:
            print("[TEST] FAIL", flush=True)
            return 1
        print("[TEST] SUCCESS!", flush=True)
        return 0
    finally:
        controller.stop()


if __name__ == "__main__":
    sys.exit(main())
