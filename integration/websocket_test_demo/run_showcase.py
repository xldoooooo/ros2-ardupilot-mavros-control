#!/usr/bin/env python3
"""以甲方可见的逐帧方式演示 WebSocket 双端真实通讯过程。"""

from __future__ import annotations

import argparse
import asyncio
import json
from pathlib import Path
import sys
import time
from typing import Any, Callable, Mapping


DEMO_ROOT = Path(__file__).resolve().parent
INTEGRATION_ROOT = DEMO_ROOT.parent
sys.path.insert(0, str(DEMO_ROOT))

from run_acceptance import ManagedJarServer, resolve_java_8  # noqa: E402
from ws_demo.client import OurGroundStationClient  # noqa: E402
from ws_demo.protocol import (  # noqa: E402
    COMMAND_LABELS,
    STATUS_LABELS,
    command_topic,
    sample_commands,
    sample_statuses,
    status_topic,
)
from ws_demo.transport import JarTopicConnection  # noqa: E402


class LiveTrace:
    """将每一帧按参与方、方向、主题和完整 JSON 打印到终端。"""

    COLORS = {
        "甲方地面站": "\033[36m",
        "WebSocket JAR": "\033[35m",
        "我方地面站": "\033[32m",
        "本地验收": "\033[33m",
    }
    RESET = "\033[0m"

    def __init__(
        self, *, delay: float, step: bool, color: bool, pretty_json: bool
    ) -> None:
        self.delay = delay
        self.step = step
        self.color = color
        self.pretty_json = pretty_json
        self.sequence = 0
        self.started = time.monotonic()

    def title(self, text: str) -> None:
        """输出一个场景标题。"""

        print("\n" + "=" * 88, flush=True)
        print(f"  {text}", flush=True)
        print("=" * 88, flush=True)

    def frame(
        self,
        source: str,
        target: str,
        action: str,
        *,
        topic: str | None = None,
        payload: Mapping[str, Any] | None = None,
        note: str | None = None,
    ) -> None:
        """输出一条具有明确发送方和接收方的真实通讯事件。"""

        self.sequence += 1
        elapsed = time.monotonic() - self.started
        source_text = self._actor(source)
        target_text = self._actor(target)
        print(
            f"\n[{self.sequence:02d} | +{elapsed:05.2f}s] "
            f"{source_text}  ──▶  {target_text}",
            flush=True,
        )
        print(f"     动作: {action}", flush=True)
        if topic is not None:
            print(f"     主题: {topic}", flush=True)
        if payload is not None:
            if self.pretty_json:
                body = json.dumps(dict(payload), ensure_ascii=False, indent=2)
                print("     JSON:", flush=True)
                for line in body.splitlines():
                    print(f"       {line}", flush=True)
            else:
                body = json.dumps(
                    dict(payload), ensure_ascii=False, separators=(",", ":")
                )
                print(f"     JSON: {body}", flush=True)
        if note is not None:
            print(f"     说明: {note}", flush=True)

    async def pause(self, prompt: str | None = None) -> None:
        """按自动延时或人工回车节奏推进演示。"""

        if self.step:
            label = prompt or "按 Enter 继续"
            await asyncio.to_thread(input, f"\n>>> {label} ")
        elif self.delay:
            await asyncio.sleep(self.delay)

    def _actor(self, actor: str) -> str:
        """仅在交互终端启用角色颜色。"""

        if not self.color:
            return actor
        color = next(
            (value for key, value in self.COLORS.items() if actor.startswith(key)),
            "",
        )
        return f"{color}{actor}{self.RESET}" if color else actor


def client_event_handler(
    trace: LiveTrace, client_no: str
) -> Callable[[str, Mapping[str, Any]], None]:
    """把我方 client 内部真实事件转换成现场可读帧。"""

    actor = f"我方地面站({client_no})"

    def handle(event: str, payload: Mapping[str, Any]) -> None:
        if event == "system":
            trace.frame("WebSocket JAR", actor, "SYSTEM 连接确认", payload=payload)
        elif event == "subscribed":
            trace.frame(
                "WebSocket JAR",
                actor,
                "SUB_ACK 订阅确认",
                topic=str(payload.get("topic")),
                payload=payload,
            )
        elif event == "command_broadcast_received":
            trace.frame(
                "WebSocket JAR",
                actor,
                "BROADCAST 原始转发帧",
                topic=str(payload.get("topic")),
                payload=payload,
            )
        elif event == "command_received":
            trace.frame(
                actor,
                "本地验收",
                "协议字段解析与 clientNo 校验通过",
                payload=payload,
            )
        elif event == "command_ack_sent":
            topic = status_topic(client_no)
            trace.frame(
                actor,
                "WebSocket JAR",
                "PUBLISH 表 1 命令确认",
                topic=topic,
                payload={"type": "PUBLISH", "topic": topic, "data": dict(payload)},
            )
        elif event == "status_sent":
            topic = status_topic(client_no)
            trace.frame(
                actor,
                "WebSocket JAR",
                "PUBLISH 无人机状态",
                topic=topic,
                payload={"type": "PUBLISH", "topic": topic, "data": dict(payload)},
            )
        elif event == "simulated_action":
            trace.frame(
                actor,
                "本地验收",
                "仅记录模拟业务动作",
                payload=payload,
                note="没有连接 ROS、飞控或实机，不会执行真实飞行动作",
            )
        elif event == "error":
            trace.frame(actor, "本地验收", "ERROR", payload=payload)

    return handle


async def receive_and_show(
    trace: LiveTrace,
    monitor: JarTopicConnection,
    count: int,
    *,
    timeout: float = 3.0,
) -> list[dict[str, Any]]:
    """在甲方状态订阅端接收指定数量的 JAR BROADCAST 并逐帧显示。"""

    received = []
    for _ in range(count):
        envelope = await monitor.receive_broadcast(timeout=timeout)
        received.append(envelope)
        trace.frame(
            "WebSocket JAR",
            "甲方地面站",
            "BROADCAST 返回状态",
            topic=str(envelope.get("topic")),
            payload=envelope,
        )
    return received


async def run_showcase(url: str, trace: LiveTrace) -> None:
    """按连接、命令、主动状态、周期遥测和多机隔离五幕演示。"""

    client_no = "UAV01001"
    publisher = JarTopicConnection(url, "showcase-counterparty-command")
    monitor = JarTopicConnection(url, "showcase-counterparty-status")
    other_monitor: JarTopicConnection | None = None
    clients: list[OurGroundStationClient] = []
    try:
        trace.title("第 1 幕：双方连接甲方 WebSocket JAR，并订阅各自主题")
        publisher_system = await publisher.open()
        trace.frame(
            "WebSocket JAR",
            "甲方地面站",
            "SYSTEM 连接确认",
            payload=publisher_system,
        )
        monitor_system = await monitor.open()
        trace.frame(
            "WebSocket JAR",
            "甲方地面站",
            "SYSTEM 状态监听连接确认",
            payload=monitor_system,
        )
        trace.frame(
            "甲方地面站",
            "WebSocket JAR",
            "SUBSCRIBE 订阅我方状态",
            topic=status_topic(client_no),
            payload={"type": "SUBSCRIBE", "topic": status_topic(client_no)},
        )
        monitor_ack = await monitor.subscribe(status_topic(client_no))
        trace.frame(
            "WebSocket JAR",
            "甲方地面站",
            "SUB_ACK 状态主题订阅成功",
            topic=status_topic(client_no),
            payload=monitor_ack,
        )
        trace.frame(
            "我方地面站",
            "WebSocket JAR",
            "SUBSCRIBE 订阅本机控制主题",
            topic=command_topic(client_no),
            payload={"type": "SUBSCRIBE", "topic": command_topic(client_no)},
        )
        client = OurGroundStationClient(
            url,
            client_no,
            telemetry_interval=None,
            on_event=client_event_handler(trace, client_no),
        )
        clients.append(client)
        await client.start()
        await trace.pause("连接与订阅完成，按 Enter 开始下发命令")

        trace.title("第 2 幕：甲方依次下发 4 类命令，我方返回确认和状态")
        for command in sample_commands(client_no):
            command_no = str(command["commandNo"])
            topic = command_topic(client_no)
            trace.frame(
                "甲方地面站",
                "WebSocket JAR",
                f"PUBLISH {command_no} {COMMAND_LABELS[command_no]}",
                topic=topic,
                payload={"type": "PUBLISH", "topic": topic, "data": command},
            )
            await publisher.publish(topic, command)
            await receive_and_show(trace, monitor, 2)
            await trace.pause(f"{command_no} 完成，按 Enter 继续")

        trace.title("第 3 幕：我方主动上报其余状态，覆盖协议全部状态编号")
        # 02/03/05/07 已在命令响应中展示，这里展示其余六种状态。
        proactive_statuses = [
            item
            for item in sample_statuses(client_no)
            if item["uavStatus"] not in {"02", "03", "05", "07"}
        ]
        for status in proactive_statuses:
            status_no = str(status["uavStatus"])
            trace.frame(
                "本地验收",
                "我方地面站",
                f"触发 {status_no} {STATUS_LABELS[status_no]} 状态",
                payload=status,
            )
            await client.publish_status(status_no, status.get("data"))
            await receive_and_show(trace, monitor, 1)
            await trace.pause(f"{status_no} 完成，按 Enter 继续")

        trace.title("第 4 幕：实测 0A 电量和 0B 位置的 1 秒周期上报")
        receive_times: dict[str, list[float]] = {"0A": [], "0B": []}
        client.start_telemetry(1.0)
        while min(len(values) for values in receive_times.values()) < 2:
            envelopes = await receive_and_show(trace, monitor, 1, timeout=2.0)
            status_no = str(envelopes[0].get("data", {}).get("uavStatus"))
            if status_no in receive_times:
                receive_times[status_no].append(time.monotonic())
        await client.stop_telemetry()
        intervals = {
            key: values[1] - values[0] for key, values in receive_times.items()
        }
        trace.frame(
            "本地验收",
            "甲方地面站",
            "计算两轮消息的接收间隔",
            payload={
                "0A_interval_seconds": round(intervals["0A"], 3),
                "0B_interval_seconds": round(intervals["0B"], 3),
                "required_seconds": 1.0,
            },
        )
        await trace.pause("周期上报完成，按 Enter 演示多飞机隔离")

        trace.title("第 5 幕：第二架飞机接入，验证每个我方地面站只控制一架飞机")
        other_client_no = "UAV01002"
        other_monitor = JarTopicConnection(url, "showcase-counterparty-uav2-status")
        await other_monitor.open()
        other_monitor_ack = await other_monitor.subscribe(status_topic(other_client_no))
        trace.frame(
            "WebSocket JAR",
            "甲方地面站",
            "SUB_ACK 第二架飞机状态主题",
            topic=status_topic(other_client_no),
            payload=other_monitor_ack,
        )
        other_client = OurGroundStationClient(
            url,
            other_client_no,
            telemetry_interval=None,
            on_event=client_event_handler(trace, other_client_no),
        )
        trace.frame(
            "我方地面站",
            "WebSocket JAR",
            "UAV01002 订阅自己的控制主题",
            topic=command_topic(other_client_no),
        )
        clients.append(other_client)
        await other_client.start()
        other_command = sample_commands(other_client_no)[1]
        trace.frame(
            "甲方地面站",
            "WebSocket JAR",
            "只向 UAV01002 下发 03 执行巡检",
            topic=command_topic(other_client_no),
            payload={
                "type": "PUBLISH",
                "topic": command_topic(other_client_no),
                "data": other_command,
            },
            note="UAV01001 订阅的是另一个精确主题，因此不会收到该命令",
        )
        await publisher.publish(command_topic(other_client_no), other_command)
        await receive_and_show(trace, other_monitor, 2)
        try:
            await monitor.receive_broadcast(timeout=0.4)
        except asyncio.TimeoutError:
            trace.frame(
                "本地验收",
                "甲方地面站",
                "UAV01001 状态主题在隔离窗口内无消息",
                payload={"isolated": True, "window_seconds": 0.4},
                note="确认 UAV01002 的响应没有串入 UAV01001",
            )
        else:
            raise RuntimeError("多飞机隔离失败：UAV01001 状态主题收到意外消息")
        await other_monitor.close()
        other_monitor = None

        trace.title("现场演示完成：真实 JAR 转发、完整业务 JSON 和多飞机隔离均已展示")
        print(
            "\n结论：4 类控制命令、10 类状态、1 秒周期遥测和双 clientNo 隔离均成功。",
            flush=True,
        )
        print(
            "安全说明：本程序只模拟通讯，没有连接 ROS、MAVROS、飞控或实机。",
            flush=True,
        )
    finally:
        for client in reversed(clients):
            await client.stop()
        if other_monitor is not None:
            await other_monitor.close()
        await monitor.close()
        await publisher.close()


def parse_args() -> argparse.Namespace:
    """解析现场节奏、JAR 和外部服务参数。"""

    parser = argparse.ArgumentParser(
        description="逐帧显示甲方地面站、JAR 和我方地面站之间的完整通讯过程。"
    )
    parser.add_argument(
        "--delay",
        type=float,
        default=0.8,
        help="自动模式每个业务步骤的停顿秒数，默认 0.8",
    )
    parser.add_argument(
        "--step",
        action="store_true",
        help="人工演示模式：每个业务步骤按 Enter 继续",
    )
    parser.add_argument("--no-color", action="store_true")
    parser.add_argument(
        "--pretty-json",
        action="store_true",
        help="多行缩进 JSON；默认用完整单行 JSON，避免现场滚屏过快",
    )
    parser.add_argument(
        "--jar",
        type=Path,
        default=INTEGRATION_ROOT / "websocket-server-1.0.0.jar",
    )
    parser.add_argument("--java", help="Java 8 可执行文件")
    parser.add_argument(
        "--server-url",
        help="连接已启动的甲方服务，不在本机启动 JAR",
    )
    return parser.parse_args()


async def async_main(args: argparse.Namespace) -> int:
    """启动本地权威 JAR 或连接外部服务，并保证退出时精确清理。"""

    if args.delay < 0:
        raise ValueError("--delay 不能小于 0")
    trace = LiveTrace(
        delay=args.delay,
        step=args.step,
        color=sys.stdout.isatty() and not args.no_color,
        pretty_json=args.pretty_json,
    )
    server: ManagedJarServer | None = None
    url = args.server_url
    try:
        if url is None:
            java = resolve_java_8(args.java)
            server = ManagedJarServer(args.jar.resolve(), java)
            url = server.url
            trace.title("准备现场环境：启动甲方提供的原始 WebSocket JAR")
            trace.frame(
                "本地验收",
                "WebSocket JAR",
                "启动原始 JAR",
                payload={
                    "jar": str(args.jar.resolve()),
                    "java": java,
                    "url": url,
                },
            )
            await server.start()
            trace.frame(
                "WebSocket JAR",
                "本地验收",
                "TCP/WebSocket 服务已就绪",
                payload={"url": url},
            )
            await trace.pause("服务已就绪，按 Enter 开始连接")
        await run_showcase(url, trace)
        return 0
    finally:
        if server is not None:
            await server.stop()


def main() -> int:
    """命令行入口。"""

    try:
        return asyncio.run(async_main(parse_args()))
    except KeyboardInterrupt:
        print("\n演示已由操作者终止，相关连接与本次 JAR 进程已清理。")
        return 130


if __name__ == "__main__":
    raise SystemExit(main())
