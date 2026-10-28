"""后台读取地面媒体目录，向状态投影和界面提供独立的只读快照。"""

from __future__ import annotations

import json
import os
import threading
from collections.abc import Callable, Mapping
from pathlib import Path, PurePosixPath
from typing import Any
from urllib.request import Request, urlopen


def media_api_url(config_path: str | Path | None = None) -> str:
    """读取独立地面配置；未配置服务地址时使用本机默认入口。"""
    target = Path(config_path or os.environ.get("GROUND_MEDIA_CONFIG") or (
        Path(__file__).resolve().parent.parent / "config" / "ground" / "media.json"
    ))
    try:
        config = json.loads(target.read_text(encoding="utf-8"))
    except FileNotFoundError:
        config = {}
    return str(config.get("api_url") or "http://127.0.0.1:8090").rstrip("/")


def processed_download(record: Mapping[str, Any]) -> str:
    """返回已公开处理文件的 FTP 根目录路径，不向甲方发送 HTTP URL。"""
    if record.get("variant") not in {"experiment", "rebuilt"}:
        return ""
    if record.get("state") not in {"ready", "incomplete"}:
        return ""
    value = str(record.get("ftp_path") or "").strip()
    path = PurePosixPath(value)
    return value if path.is_absolute() and not value.startswith("//") and ".." not in path.parts else ""


class GroundMediaReader:
    """用独立线程轮询地面目录，网络故障不阻塞 GUI 或飞控。"""

    def __init__(
        self,
        on_snapshot: Callable[[tuple[dict[str, Any], ...], str], None],
        *,
        config_path: str | Path | None = None,
        interval: float = 1.0,
        timeout: float = 1.5,
    ) -> None:
        """保存回调与短网络超时；构造时不进行文件或网络访问。"""
        self._on_snapshot = on_snapshot
        self._config_path = config_path
        self._interval = interval
        self._timeout = timeout
        self._stop = threading.Event()
        self._wake = threading.Event()
        self._thread: threading.Thread | None = None
        self._pending_lock = threading.Lock()
        self._pending_failures: dict[tuple[str, str, str], dict[str, Any]] = {}
        self._pending_revision = 0
        self._persisted_revision = 0
        explicit_config = config_path or os.environ.get("GROUND_MEDIA_CONFIG")
        if explicit_config:
            self._spool_path = Path(explicit_config).parent / "media-delivery-failures.json"
        else:
            state_root = Path(os.environ.get("XDG_STATE_HOME") or Path.home() / ".local" / "state")
            self._spool_path = state_root / "ros2-ardupilot-ground-station" / "media-delivery-failures.json"

    def fail_delivery(self, payload: Mapping[str, Any]) -> None:
        """只在内存排队交付终态，HTTP持久化由轮询线程完成。"""
        value = {**payload, "action": "fail"}
        key = tuple(str(value.get(name, "")) for name in (
            "aircraft_id", "source_id", "mission_sequence"
        ))
        with self._pending_lock:
            self._pending_failures[key] = value
            self._pending_revision += 1
        self._wake.set()

    def _restore_failures(self) -> None:
        """恢复尚未确认的失败通知，新会话不恢复任何自动视频操作。"""
        try:
            values = json.loads(self._spool_path.read_text(encoding="utf-8"))
        except FileNotFoundError:
            return
        if not isinstance(values, list):
            raise ValueError("媒体失败通知文件必须为JSON列表")
        with self._pending_lock:
            for value in values:
                if not isinstance(value, dict):
                    continue
                key = tuple(str(value.get(name, "")) for name in (
                    "aircraft_id", "source_id", "mission_sequence"
                ))
                self._pending_failures.setdefault(key, value)

    def _persist_failures(self) -> None:
        """后台原子保存待确认终态，进程退出重启后继续通知停止。"""
        with self._pending_lock:
            revision = self._pending_revision
            if revision == self._persisted_revision:
                return
            pending = list(self._pending_failures.values())
        self._spool_path.parent.mkdir(parents=True, exist_ok=True)
        temporary = self._spool_path.with_name(f".{self._spool_path.name}.{os.getpid()}.tmp")
        temporary.write_text(json.dumps(pending, ensure_ascii=False), encoding="utf-8")
        temporary.replace(self._spool_path)
        self._persisted_revision = revision

    def _flush_failures(self, api_url: str) -> None:
        """可靠重试终态通知；网络恢复后禁止后台继续该任务的视频处理。"""
        with self._pending_lock:
            pending = tuple(self._pending_failures.items())
        for key, payload in pending:
            request = Request(
                api_url + "/api/delivery",
                data=json.dumps(payload).encode("utf-8"),
                headers={"Content-Type": "application/json"}, method="POST",
            )
            with urlopen(request, timeout=self._timeout) as response:
                json.load(response)
            with self._pending_lock:
                self._pending_failures.pop(key, None)
                self._pending_revision += 1
            self._persist_failures()

    def start(self) -> None:
        """启动幂等的后台目录读取。"""
        if self._thread is not None and self._thread.is_alive():
            return
        self._stop.clear()
        self._thread = threading.Thread(
            target=self._run, name="ground-media-reader", daemon=True
        )
        self._thread.start()

    def stop(self, timeout: float = 2.0) -> bool:
        """唤醒轮询等待并限时结束线程。"""
        self._stop.set()
        self._wake.set()
        if self._thread is not None:
            self._thread.join(timeout=max(0.0, timeout))
        return self._thread is None or not self._thread.is_alive()

    def _run(self) -> None:
        """读取完整快照，失败时清空可发布目录并报告可见错误。"""
        try:
            self._restore_failures()
        except Exception as exc:  # noqa: BLE001 - 损坏的本地记录不得关闭GUI。
            self._on_snapshot((), f"恢复媒体失败通知失败：{exc}")
        try:
            self._poll()
        finally:
            try:
                self._persist_failures()
            except OSError as exc:
                self._on_snapshot((), f"保存媒体失败通知失败：{exc}")

    def _poll(self) -> None:
        """轮询前先持久化失败，断网只重试终态通知而不恢复媒体处理。"""
        while not self._stop.is_set():
            records: tuple[dict[str, Any], ...] = ()
            error = ""
            try:
                self._persist_failures()
                api_url = media_api_url(self._config_path)
                self._flush_failures(api_url)
                with urlopen(api_url + "/api/media", timeout=self._timeout) as response:
                    payload = json.load(response)
                values = payload.get("records")
                if not isinstance(values, list):
                    raise ValueError("地面媒体响应缺少 records 列表")
                deliveries = payload.get("deliveries") or []
                records = tuple(dict(value) for value in values if isinstance(value, dict))
                records += tuple(
                    {**value, "kind": "delivery",
                     "metadata": {"source_id": value.get("source_id", "")}}
                    for value in deliveries if isinstance(value, dict)
                )
                records += ({"kind": "delivery_policy", "delivery_timeout_seconds":
                             payload.get("delivery_timeout_seconds", 600.0)},)
                error = str(payload.get("error") or "")
            except Exception as exc:  # noqa: BLE001 - 媒体功能独立降级。
                error = str(exc)
            if self._stop.is_set():
                return
            self._on_snapshot(records, error)
            self._wake.wait(self._interval)
            self._wake.clear()
