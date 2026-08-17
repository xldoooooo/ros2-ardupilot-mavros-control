"""甲方 JAR 的 SUBSCRIBE/PUBLISH 原生 WebSocket 传输适配器。"""

from __future__ import annotations

import asyncio
import contextlib
import json
from typing import Any, Mapping

import websockets


class TransportError(RuntimeError):
    """WebSocket 握手、连接或 JAR 信封异常。"""


class JarTopicConnection:
    """一个与甲方 JAR 对接的独立 WebSocket 会话。"""

    def __init__(self, url: str, name: str) -> None:
        self.url = url
        self.name = name
        self.system_message: dict[str, Any] | None = None
        self._ws: Any = None
        self._reader_task: asyncio.Task[None] | None = None
        self._broadcasts: asyncio.Queue[dict[str, Any]] = asyncio.Queue()
        self._subscription_acks: asyncio.Queue[dict[str, Any]] = asyncio.Queue()
        self._heartbeat_acks: asyncio.Queue[dict[str, Any]] = asyncio.Queue()
        self._errors: asyncio.Queue[dict[str, Any]] = asyncio.Queue()

    async def open(self, timeout: float = 5.0) -> dict[str, Any]:
        """连接 `/ws`，并验证 JAR 首帧 SYSTEM 握手。"""

        self._ws = await asyncio.wait_for(
            websockets.connect(
                self.url,
                ping_interval=20,
                ping_timeout=20,
                close_timeout=2,
                max_size=2 * 1024 * 1024,
            ),
            timeout=timeout,
        )
        raw = await asyncio.wait_for(self._ws.recv(), timeout=timeout)
        message = self._decode(raw)
        if message.get("type") != "SYSTEM":
            await self._ws.close()
            raise TransportError(f"期待 SYSTEM 握手，实际收到: {message}")
        self.system_message = message
        self._reader_task = asyncio.create_task(
            self._reader(), name=f"jar-topic-reader-{self.name}"
        )
        return message

    async def close(self) -> None:
        """只关闭当前会话及其读取任务。"""

        if self._ws is not None:
            await self._ws.close()
        if self._reader_task is not None:
            self._reader_task.cancel()
            with contextlib.suppress(asyncio.CancelledError):
                await self._reader_task
        self._reader_task = None
        self._ws = None

    async def subscribe(self, topic: str, timeout: float = 3.0) -> dict[str, Any]:
        """订阅主题并等待 JAR 的 SUB_ACK。"""

        await self.send_json({"type": "SUBSCRIBE", "topic": topic})
        while True:
            ack = await asyncio.wait_for(self._subscription_acks.get(), timeout)
            if ack.get("topic") == topic:
                return ack

    async def publish(self, topic: str, data: Mapping[str, Any]) -> None:
        """按 JAR 要求用 PUBLISH 信封发送一条业务 JSON。"""

        await self.send_json({"type": "PUBLISH", "topic": topic, "data": dict(data)})

    async def heartbeat(self, timeout: float = 3.0) -> dict[str, Any]:
        """发送 JAR 心跳并等待 HEARTBEAT_ACK。"""

        await self.send_json({"type": "HEARTBEAT"})
        return await asyncio.wait_for(self._heartbeat_acks.get(), timeout)

    async def send_json(self, payload: Mapping[str, Any]) -> None:
        """发送 JSON 文本帧，中文保持可读。"""

        if self._ws is None:
            raise TransportError("WebSocket 尚未连接")
        await self._ws.send(json.dumps(dict(payload), ensure_ascii=False))

    async def send_text(self, payload: str) -> None:
        """发送原始文本；仅用于验证 JAR 的错误响应。"""

        if self._ws is None:
            raise TransportError("WebSocket 尚未连接")
        await self._ws.send(payload)

    async def receive_broadcast(self, timeout: float = 3.0) -> dict[str, Any]:
        """等待下一条 BROADCAST。"""

        return await asyncio.wait_for(self._broadcasts.get(), timeout)

    async def receive_error(self, timeout: float = 3.0) -> dict[str, Any]:
        """等待 JAR 的 ERROR 信封。"""

        return await asyncio.wait_for(self._errors.get(), timeout)

    async def _reader(self) -> None:
        """单消费者读取帧，再按 JAR 消息类型分流，避免并发 recv。"""

        try:
            async for raw in self._ws:
                message = self._decode(raw)
                message_type = message.get("type")
                if message_type == "BROADCAST":
                    await self._broadcasts.put(message)
                elif message_type == "SUB_ACK":
                    await self._subscription_acks.put(message)
                elif message_type == "HEARTBEAT_ACK":
                    await self._heartbeat_acks.put(message)
                elif message_type == "ERROR":
                    await self._errors.put(message)
                else:
                    await self._errors.put(
                        {
                            "type": "ERROR",
                            "msg": f"未知 JAR 响应类型: {message_type!r}",
                        }
                    )
        except asyncio.CancelledError:
            raise
        except Exception as exc:
            await self._errors.put({"type": "ERROR", "msg": str(exc)})

    @staticmethod
    def _decode(raw: Any) -> dict[str, Any]:
        """把文本帧解析为 JSON 对象。"""

        try:
            value = json.loads(raw)
        except (TypeError, json.JSONDecodeError) as exc:
            raise TransportError(f"JAR 返回了无效 JSON: {raw!r}") from exc
        if not isinstance(value, dict):
            raise TransportError(f"JAR 返回值不是 JSON 对象: {value!r}")
        return value

