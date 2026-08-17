"""可独立连接、断开与重启的上位机 WebSocket 通讯服务。"""

from __future__ import annotations

import asyncio
import contextlib
import json
import os
import threading
import time
from collections.abc import Callable, Mapping
from typing import Any
from urllib.parse import urlparse

from ..event_log import EventLog
from ..models import CommandResult, VehicleSnapshot
from .journal import RawFrameJournal
from .mapping import (
    COMMAND_MAPPINGS,
    UpstreamProtocolError,
    parse_command,
)
from .models import UpstreamCommand, UpstreamConnectionSnapshot
from .protocol import (
    command_ack,
    command_topic,
    decode_object,
    publish_envelope,
    subscribe_envelope,
)
from .status_projector import UpstreamStatusProjector


class UpstreamCommunicationService:
    """把上位机主题协议隔离在专用 asyncio 线程中的插件式服务。"""

    _RECONNECT_SECONDS = 3.0
    _HANDSHAKE_TIMEOUT_SECONDS = 5.0
    _OUTBOUND_LIMIT = 256

    def __init__(
        self,
        *,
        event_log: EventLog,
        on_command: Callable[[UpstreamCommand], None],
        url: str | None = None,
        client_no: str | None = None,
        auto_connect: bool | None = None,
    ) -> None:
        """读取可覆盖配置；构造本身不创建线程或网络连接。"""
        configured_url = url or os.environ.get(
            "UPSTREAM_WS_URL", "ws://127.0.0.1:8581/ws"
        )
        configured_client = client_no or os.environ.get(
            "UPSTREAM_CLIENT_NO", "UAV01001"
        )
        if auto_connect is None:
            auto_connect = os.environ.get(
                "UPSTREAM_WS_AUTO_CONNECT", "1"
            ).strip() not in {
                "0",
                "false",
                "False",
                "no",
                "NO",
            }
        self._events = event_log
        try:
            self._validate_configuration(configured_url, configured_client)
        except (TypeError, ValueError, UpstreamProtocolError) as exc:
            # 配置错误只禁用自动连接，原地面站必须仍可完整启动。
            self._events.warn(
                "upstream",
                f"上位机环境配置无效，已回退为断开状态：{exc}",
            )
            configured_url = "ws://127.0.0.1:8581/ws"
            configured_client = "UAV01001"
            auto_connect = False
        self._on_command = on_command
        self._journal = RawFrameJournal()
        self._state_lock = threading.RLock()
        self._url = configured_url.strip()
        self._client_no = configured_client.strip()
        self._desired_connected = bool(auto_connect)
        self._connected = False
        self._state = "未启动"
        self._detail = "通讯线程尚未启动"
        self._revision = 0
        self._last_connection_warning = ""
        self._last_connection_warning_at = 0.0
        self._stopping = False
        self._thread: threading.Thread | None = None
        self._loop: asyncio.AbstractEventLoop | None = None
        self._wake_event: asyncio.Event | None = None
        self._outbound: asyncio.Queue[dict[str, Any]] | None = None
        self._projector = UpstreamStatusProjector(
            self._current_client_no, self.publish_status, self._events
        )

    @property
    def journal(self) -> RawFrameJournal:
        """返回只供独立面板读取的原始报文缓冲。"""
        return self._journal

    @property
    def command_mappings(self):
        """返回独立映射文件中的只读命令表。"""
        return COMMAND_MAPPINGS

    def start(self) -> None:
        """启动专用线程；默认连接失败不会阻塞或关闭地面站。"""
        with self._state_lock:
            if self._thread is not None and self._thread.is_alive():
                return
            self._stopping = False
            self._state = "启动中"
            self._detail = "正在创建上位机通讯线程"
            self._thread = threading.Thread(
                target=self._thread_main,
                name="ground-station-upstream-websocket",
                daemon=True,
            )
            self._thread.start()
        self._events.info("upstream", "上位机通讯模块已随地面站启动")
        # TODO(上位机状态): 01 待机状态需等异常恢复判定逻辑明确后实现。
        self._events.info("upstream", "状态 01 当前未实现，不会发送待机状态")
        # TODO(上位机媒体): FTP、RTSP、云台、拍照及媒体路径暂不实现。
        self._events.info("upstream", "相机、云台、照片、FTP 与媒体路径当前未接入")

    def connect(self, url: str, client_no: str) -> None:
        """应用面板配置并请求连接；可在地面站会话外独立执行。"""
        self._validate_configuration(url, client_no)
        with self._state_lock:
            self._url = url.strip()
            self._client_no = client_no.strip()
            self._desired_connected = True
            self._connected = False
            self._state = "连接中"
            self._revision += 1
            self._detail = "已请求连接上位机"
            needs_start = self._thread is None or not self._thread.is_alive()
        if needs_start:
            self.start()
        else:
            self._signal_loop()

    def disconnect(self) -> None:
        """只关闭上位机连接，不触碰 ROS、仿真或机载控制租约。"""
        with self._state_lock:
            self._desired_connected = False
            self._connected = False
            self._revision += 1
            self._detail = "已请求断开上位机"
        self._signal_loop()

    def restart(self, url: str, client_no: str) -> None:
        """应用配置并强制重建一个全新的 WebSocket 会话。"""
        self.connect(url, client_no)
        self._events.info("upstream", "已请求重启上位机通讯连接")

    def stop(self, timeout: float = 3.0) -> bool:
        """在地面站退出时限时停止插件；失败也不阻断主体清理。"""
        with self._state_lock:
            thread = self._thread
            if thread is None:
                return True
            self._stopping = True
            self._desired_connected = False
            self._revision += 1
        self._signal_loop()
        thread.join(timeout=max(0.0, float(timeout)))
        stopped = not thread.is_alive()
        if stopped:
            with self._state_lock:
                self._thread = None
                self._state = "已停止"
                self._detail = "上位机通讯线程已停止"
        else:
            self._events.warn("upstream", "上位机通讯线程未在退出时限内停止")
        return stopped

    def snapshot(self) -> UpstreamConnectionSnapshot:
        """返回面板可无锁使用的连接配置与状态副本。"""
        with self._state_lock:
            return UpstreamConnectionSnapshot(
                url=self._url,
                client_no=self._client_no,
                desired_connected=self._desired_connected,
                connected=self._connected,
                state=self._state,
                detail=self._detail,
            )

    def publish_status(self, payload: Mapping[str, Any]) -> bool:
        """只在当前连接有效时排队状态，防止断线旧消息重新发送。"""
        with self._state_lock:
            loop = self._loop
            queue = self._outbound
            connected = self._connected
            client_no = self._client_no
        if loop is None or queue is None or not connected or loop.is_closed():
            return False
        envelope = publish_envelope(client_no, payload)
        loop.call_soon_threadsafe(self._queue_outbound, envelope)
        return True

    def report_waypoints_staged(self) -> bool:
        """转发 GUI 成功替换航点的 02 事件。"""
        return self._projector.report_waypoints_staged()

    def begin_mission(
        self, ticket: int, kind: str, point_indexes: tuple[int, ...]
    ) -> None:
        """绑定由上位机触发的航点任务与本地可靠命令 ticket。"""
        self._projector.begin_mission(ticket, kind, point_indexes)

    def begin_landing(self, ticket: int) -> None:
        """绑定由上位机触发的降落与本地可靠命令 ticket。"""
        self._projector.begin_landing(ticket)

    def observe_vehicle(self, snapshot: VehicleSnapshot, connection_mode: str) -> None:
        """由 Qt 刷新循环提供当前会话权威快照。"""
        self._projector.observe_vehicle(snapshot, connection_mode)

    def observe_result(self, result: CommandResult) -> None:
        """由 GUI 消费同一可靠结果，生成无重复的任务状态。"""
        self._projector.observe_result(result)

    def reset_runtime(self) -> None:
        """仿真或实机会话断开后清空任务事件关联。"""
        self._projector.reset_runtime()

    def _thread_main(self) -> None:
        """在线程内部拥有并完整销毁 asyncio 事件循环。"""
        try:
            asyncio.run(self._supervise())
        except Exception as exc:  # noqa: BLE001 - 线程边界必须隔离所有插件故障。
            self._set_state("故障", f"通讯线程异常：{exc}", connected=False)
            self._events.error("upstream", f"上位机通讯线程异常：{exc}")
        finally:
            with self._state_lock:
                self._loop = None
                self._wake_event = None
                self._outbound = None
                self._connected = False

    async def _supervise(self) -> None:
        """按期望状态监督会话，并支持配置变更、断开和限速重连。"""
        loop = asyncio.get_running_loop()
        wake_event = asyncio.Event()
        outbound: asyncio.Queue[dict[str, Any]] = asyncio.Queue(
            maxsize=self._OUTBOUND_LIMIT
        )
        with self._state_lock:
            self._loop = loop
            self._wake_event = wake_event
            self._outbound = outbound
        while True:
            with self._state_lock:
                stopping = self._stopping
                desired = self._desired_connected
                url = self._url
                client_no = self._client_no
                revision = self._revision
            if stopping:
                break
            if not desired:
                self._set_state("已断开", "未连接上位机", connected=False)
                await wake_event.wait()
                wake_event.clear()
                continue

            self._clear_outbound()
            self._set_state("连接中", f"正在连接 {url}", connected=False)
            session = asyncio.create_task(self._run_session(url, client_no))
            wake = asyncio.create_task(wake_event.wait())
            done, _ = await asyncio.wait(
                (session, wake), return_when=asyncio.FIRST_COMPLETED
            )
            if wake in done:
                wake_event.clear()
                session.cancel()
                with contextlib.suppress(asyncio.CancelledError, Exception):
                    await session
            else:
                wake.cancel()
                with contextlib.suppress(asyncio.CancelledError):
                    await wake
                try:
                    await session
                except asyncio.CancelledError:
                    raise
                except ModuleNotFoundError:
                    self._set_state(
                        "依赖缺失",
                        "缺少 websockets，请安装 requirements-gui.txt",
                        connected=False,
                    )
                    self._events.warn(
                        "upstream",
                        "缺少 websockets 依赖；原地面站继续运行，上位机通讯不可用",
                    )
                    with self._state_lock:
                        self._desired_connected = False
                except Exception as exc:  # noqa: BLE001 - 网络库异常需统一重连。
                    self._set_state("等待重连", str(exc), connected=False)
                    self._log_connection_warning(str(exc))
            self._clear_outbound()
            with self._state_lock:
                still_desired = self._desired_connected
                unchanged = self._revision == revision
            if still_desired and unchanged:
                try:
                    await asyncio.wait_for(
                        wake_event.wait(), timeout=self._RECONNECT_SECONDS
                    )
                    wake_event.clear()
                except TimeoutError:
                    pass

        self._set_state("已停止", "上位机通讯线程已停止", connected=False)

    async def _run_session(self, url: str, client_no: str) -> None:
        """完成 SYSTEM/SUB_ACK 握手，再运行单读单写任务。"""
        # 延迟导入保证依赖缺失时原地面站仍能 100% 启动和退出。
        import websockets

        websocket = await asyncio.wait_for(
            websockets.connect(
                url,
                ping_interval=20,
                ping_timeout=20,
                close_timeout=2,
                max_size=2 * 1024 * 1024,
            ),
            timeout=self._HANDSHAKE_TIMEOUT_SECONDS,
        )
        reader: asyncio.Task[None] | None = None
        writer: asyncio.Task[None] | None = None
        try:
            raw = await asyncio.wait_for(
                websocket.recv(), timeout=self._HANDSHAKE_TIMEOUT_SECONDS
            )
            self._record_rx(raw)
            system = decode_object(raw)
            if system.get("type") != "SYSTEM":
                raise UpstreamProtocolError("WebSocket 首帧不是 SYSTEM")
            await self._send_json(websocket, subscribe_envelope(client_no))
            while True:
                raw = await asyncio.wait_for(
                    websocket.recv(), timeout=self._HANDSHAKE_TIMEOUT_SECONDS
                )
                self._record_rx(raw)
                message = decode_object(raw)
                if message.get("type") == "SUB_ACK" and message.get(
                    "topic"
                ) == command_topic(client_no):
                    break
                if message.get("type") == "ERROR":
                    raise UpstreamProtocolError(
                        f"订阅控制主题失败：{message.get('msg', '未知错误')}"
                    )
            self._set_state(
                "已连接",
                f"已订阅 {command_topic(client_no)}",
                connected=True,
            )
            self._events.info("upstream", f"已连接上位机并订阅无人机 {client_no}")
            with self._state_lock:
                self._last_connection_warning = ""
            reader = asyncio.create_task(self._reader(websocket, client_no))
            writer = asyncio.create_task(self._writer(websocket))
            done, pending = await asyncio.wait(
                (reader, writer), return_when=asyncio.FIRST_COMPLETED
            )
            for task in pending:
                task.cancel()
            for task in pending:
                with contextlib.suppress(asyncio.CancelledError):
                    await task
            for task in done:
                task.result()
        finally:
            # 监督器因断开/重启取消本会话时，也必须回收内部读写任务；
            # 否则旧 writer 会从新会话的共享队列抢走首条确认消息。
            for task in (reader, writer):
                if task is not None and not task.done():
                    task.cancel()
            for task in (reader, writer):
                if task is not None:
                    with contextlib.suppress(asyncio.CancelledError, Exception):
                        await task
            self._set_state("已断开", "WebSocket 会话已关闭", connected=False)
            await websocket.close()

    async def _reader(self, websocket: Any, client_no: str) -> None:
        """作为唯一 recv 消费者解析 BROADCAST 并投递已映射命令。"""
        async for raw in websocket:
            self._record_rx(raw)
            try:
                envelope = decode_object(raw)
                message_type = envelope.get("type")
                if message_type == "BROADCAST":
                    if envelope.get("topic") != command_topic(client_no):
                        continue
                    data = envelope.get("data")
                    command = parse_command(data, client_no)
                    self._queue_outbound(
                        publish_envelope(
                            client_no,
                            command_ack(client_no, command.command_no),
                        )
                    )
                    self._events.info(
                        "upstream",
                        f"收到上位机命令 {command.command_no}（{command.label}），已确认接收",
                    )
                    try:
                        self._on_command(command)
                    except Exception as exc:  # noqa: BLE001 - 回调不得终止网络线程。
                        self._events.error(
                            "upstream", f"上位机命令投递到地面站失败：{exc}"
                        )
                elif message_type == "ERROR":
                    self._events.warn(
                        "upstream",
                        f"上位机主题服务返回错误：{envelope.get('msg', '--')}",
                    )
                elif message_type not in {"HEARTBEAT_ACK"}:
                    self._events.debug(
                        "upstream", f"忽略上位机信封类型 {message_type!r}"
                    )
            except UpstreamProtocolError as exc:
                self._events.warn("upstream", f"拒绝无效上位机报文：{exc}")

    async def _writer(self, websocket: Any) -> None:
        """串行发送确认和状态，保证同一 WebSocket 不发生并发 send。"""
        queue = self._outbound
        if queue is None:
            raise RuntimeError("上位机发送队列未初始化")
        while True:
            envelope = await queue.get()
            await self._send_json(websocket, envelope)

    async def _send_json(self, websocket: Any, payload: Mapping[str, Any]) -> None:
        """发送紧凑 UTF-8 JSON，并把精确文本只写入原始报文日志。"""
        raw = json.dumps(dict(payload), ensure_ascii=False, separators=(",", ":"))
        await websocket.send(raw)
        self._journal.append("TX", raw)

    def _record_rx(self, raw: Any) -> None:
        """保留精确文本；二进制帧使用可识别占位表示。"""
        payload = raw if isinstance(raw, str) else f"<binary {len(raw)} bytes>"
        self._journal.append("RX", payload)

    def _queue_outbound(self, envelope: dict[str, Any]) -> None:
        """在事件循环内以丢旧保新的方式写入有界发送队列。"""
        queue = self._outbound
        if queue is None:
            return
        if queue.full():
            with contextlib.suppress(asyncio.QueueEmpty):
                queue.get_nowait()
        with contextlib.suppress(asyncio.QueueFull):
            queue.put_nowait(envelope)

    def _clear_outbound(self) -> None:
        """会话边界丢弃旧状态，满足不积压过期业务消息的要求。"""
        queue = self._outbound
        if queue is None:
            return
        while True:
            try:
                queue.get_nowait()
            except asyncio.QueueEmpty:
                return

    def _signal_loop(self) -> None:
        """从 Qt 或退出线程安全唤醒 asyncio 监督器。"""
        with self._state_lock:
            loop = self._loop
            wake_event = self._wake_event
        if loop is not None and wake_event is not None and not loop.is_closed():
            loop.call_soon_threadsafe(wake_event.set)

    def _set_state(self, state: str, detail: str, *, connected: bool) -> None:
        """原子更新面板状态，不在网络线程触碰任何 Qt 对象。"""
        with self._state_lock:
            self._state = state
            self._detail = detail
            self._connected = connected

    def _current_client_no(self) -> str:
        """为状态投影器读取当前配置的无人机编号。"""
        with self._state_lock:
            return self._client_no

    def _log_connection_warning(self, message: str) -> None:
        """相同断线原因每分钟至多写一次人类日志，面板状态仍实时更新。"""
        now = time.monotonic()
        with self._state_lock:
            should_log = (
                message != self._last_connection_warning
                or now - self._last_connection_warning_at >= 60.0
            )
            if should_log:
                self._last_connection_warning = message
                self._last_connection_warning_at = now
        if should_log:
            self._events.warn("upstream", f"上位机连接中断：{message}")

    @staticmethod
    def _validate_configuration(url: str, client_no: str) -> None:
        """在进入网络线程前拒绝不完整 URL 与非法主题编号。"""
        if not isinstance(url, str):
            raise TypeError("WebSocket URL 必须是字符串")
        parsed = urlparse(url.strip())
        if parsed.scheme not in {"ws", "wss"} or not parsed.netloc:
            raise ValueError("WebSocket URL 必须使用 ws:// 或 wss:// 并包含主机")
        command_topic(client_no)
