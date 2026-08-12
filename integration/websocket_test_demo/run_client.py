#!/usr/bin/env python3
"""运行我方单飞机地面站 WebSocket client 通讯演示。"""

from __future__ import annotations

import argparse
import asyncio
import json
from pathlib import Path
import sys
from typing import Any, Mapping


DEMO_ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(DEMO_ROOT))

from ws_demo.client import OurGroundStationClient  # noqa: E402


EVENT_LABELS = {
    "system": "连接",
    "subscribed": "订阅",
    "command_broadcast_received": "收到 JAR 信封",
    "command_received": "收到命令",
    "command_ack_sent": "发送确认",
    "status_sent": "发送状态",
    "simulated_action": "模拟动作",
    "error": "错误",
}


def show_event(event: str, payload: Mapping[str, Any]) -> None:
    """以紧凑中文格式展示每次协议收发。"""

    body = json.dumps(dict(payload), ensure_ascii=False, separators=(",", ":"))
    print(f"[{EVENT_LABELS.get(event, event)}] {body}", flush=True)


def parse_args() -> argparse.Namespace:
    """解析独立客户端参数。"""

    parser = argparse.ArgumentParser(
        description="连接甲方 JAR；只模拟通讯，不连接或控制真实飞机。"
    )
    parser.add_argument("--url", default="ws://127.0.0.1:8581/ws")
    parser.add_argument("--client-no", default="UAV01001")
    parser.add_argument("--power", type=float, default=55.6)
    parser.add_argument("--x", type=float, default=55.6)
    parser.add_argument("--y", type=float, default=55.6)
    parser.add_argument("--z", type=float, default=5.0)
    parser.add_argument("--telemetry-interval", type=float, default=1.0)
    return parser.parse_args()


async def async_main(args: argparse.Namespace) -> None:
    """保持客户端在线，直到操作者按 Ctrl+C。"""

    client = OurGroundStationClient(
        args.url,
        args.client_no,
        telemetry_interval=args.telemetry_interval,
        power=args.power,
        position=(args.x, args.y, args.z),
        on_event=show_event,
    )
    await client.start()
    print("客户端已就绪；按 Ctrl+C 退出。所有飞行动作仅打印模拟。", flush=True)
    try:
        await asyncio.Future()
    finally:
        await client.stop()


def main() -> int:
    """命令行入口。"""

    args = parse_args()
    try:
        asyncio.run(async_main(args))
    except KeyboardInterrupt:
        print("\n客户端已安全退出。")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
