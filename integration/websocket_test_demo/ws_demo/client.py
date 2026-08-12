"""我方地面站 WebSocket client 的独立通讯演示实现。

本模块只模拟协议收发，不连接 ROS、飞控或任何真实飞机，也不会执行解锁、
起飞、返航或停机动作。
"""

from __future__ import annotations

import asyncio
import contextlib
from typing import Any, Callable, Mapping

from .protocol import (
    COMMAND_LABELS,
    command_ack,
    command_topic,
    make_status,
    status_topic,
    validate_command,
)
from .transport import JarTopicConnection


EventCallback = Callable[[str, Mapping[str, Any]], None]


class OurGroundStationClient:
    """一个 clientNo 对应一架飞机的我方地面站通讯客户端。"""

    def __init__(
        self,
        url: str,
        client_no: str,
        *,
        telemetry_interval: float | None = 1.0,
        power: float = 55.6,
        position: tuple[float, float, float] = (55.6, 55.6, 5.0),
        on_event: EventCallback | None = None,
    ) -> None:
        self.client_no = client_no
        self.connection = JarTopicConnection(url, f"our-ground-{client_no}")
        self.telemetry_interval = telemetry_interval
        self.power = power
        self.position = position
        self.on_event = on_event
        self.processed_commands: list[dict[str, Any]] = []
        self.errors: asyncio.Queue[Exception] = asyncio.Queue()
        self._command_task: asyncio.Task[None] | None = None
        self._telemetry_task: asyncio.Task[None] | None = None

    async def start(self) -> None:
        """连接 JAR、订阅本机控制主题，并按配置启动 1 秒遥测。"""

        system = await self.connection.open()
        self._emit("system", system)
        ack = await self.connection.subscribe(command_topic(self.client_no))
        self._emit("subscribed", ack)
        self._command_task = asyncio.create_task(
            self._command_loop(), name=f"command-loop-{self.client_no}"
        )
        if self.telemetry_interval is not None:
            self.start_telemetry(self.telemetry_interval)

    async def stop(self) -> None:
        """停止模拟任务并关闭当前 WebSocket，不影响 JAR 或其他客户端。"""

        for task in (self._telemetry_task, self._command_task):
            if task is not None:
                task.cancel()
        for task in (self._telemetry_task, self._command_task):
            if task is not None:
                with contextlib.suppress(asyncio.CancelledError):
                    await task
        self._telemetry_task = None
        self._command_task = None
        await self.connection.close()

    def start_telemetry(self, interval: float = 1.0) -> None:
        """启动协议规定的电量和位置周期上报。"""

        if interval <= 0:
            raise ValueError("遥测周期必须大于 0")
        if self._telemetry_task is not None and not self._telemetry_task.done():
            raise RuntimeError("遥测上报已经启动")
        self.telemetry_interval = interval
        self._telemetry_task = asyncio.create_task(
            self._telemetry_loop(), name=f"telemetry-loop-{self.client_no}"
        )

    async def stop_telemetry(self) -> None:
        """仅停止周期遥测，保持 WebSocket 和命令订阅在线。"""

        if self._telemetry_task is not None:
            self._telemetry_task.cancel()
            with contextlib.suppress(asyncio.CancelledError):
                await self._telemetry_task
        self._telemetry_task = None

    async def publish_status(
        self, status: str, data: Mapping[str, Any] | None = None
    ) -> dict[str, Any]:
        """向本机状态主题发布一条已校验的协议状态。"""

        payload = make_status(self.client_no, status, data)
        await self.connection.publish(status_topic(self.client_no), payload)
        self._emit("status_sent", payload)
        return payload

    async def _command_loop(self) -> None:
        """接收命令，发送表 1 确认，再上报对应的业务状态。"""

        try:
            while True:
                envelope = await self.connection.receive_broadcast(timeout=3600.0)
                self._emit("command_broadcast_received", envelope)
                payload = validate_command(envelope.get("data"), self.client_no)
                self.processed_commands.append(payload)
                self._emit("command_received", payload)

                # 文档表 1 的确认就是 clientNo + commandNo，没有额外 ACK 字段。
                ack = command_ack(payload)
                await self.connection.publish(status_topic(self.client_no), ack)
                self._emit("command_ack_sent", ack)

                status = payload["commandNo"]
                await self.publish_status(status)
                self._emit(
                    "simulated_action",
                    {"commandNo": status, "label": COMMAND_LABELS[status]},
                )
        except asyncio.CancelledError:
            raise
        except Exception as exc:
            await self.errors.put(exc)
            self._emit("error", {"message": str(exc)})

    async def _telemetry_loop(self) -> None:
        """每个周期连续发布一次电量 0A 和一次位置 0B。"""

        assert self.telemetry_interval is not None
        try:
            while True:
                await self.publish_status("0A", {"uavPower": self.power})
                x, y, z = self.position
                await self.publish_status("0B", {"X": x, "Y": y, "Z": z})
                await asyncio.sleep(self.telemetry_interval)
        except asyncio.CancelledError:
            raise
        except Exception as exc:
            await self.errors.put(exc)
            self._emit("error", {"message": str(exc)})

    def _emit(self, event: str, payload: Mapping[str, Any]) -> None:
        """把可选的可视化事件回调隔离在通讯主路径之外。"""

        if self.on_event is not None:
            self.on_event(event, payload)
