"""后台读取地面媒体目录，向状态投影和界面提供独立的只读快照。"""

from __future__ import annotations

import json
import os
import threading
from collections.abc import Callable, Mapping
from pathlib import Path, PurePosixPath
from typing import Any
from urllib.request import urlopen


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
        self._thread: threading.Thread | None = None

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
        if self._thread is not None:
            self._thread.join(timeout=max(0.0, timeout))
        return self._thread is None or not self._thread.is_alive()

    def _run(self) -> None:
        """读取完整快照，失败时清空可发布目录并报告可见错误。"""
        while not self._stop.is_set():
            records: tuple[dict[str, Any], ...] = ()
            error = ""
            try:
                url = media_api_url(self._config_path) + "/api/media"
                with urlopen(url, timeout=self._timeout) as response:
                    payload = json.load(response)
                values = payload.get("records")
                if not isinstance(values, list):
                    raise ValueError("地面媒体响应缺少 records 列表")
                records = tuple(dict(value) for value in values if isinstance(value, dict))
                error = str(payload.get("error") or "")
            except Exception as exc:  # noqa: BLE001 - 媒体功能独立降级。
                error = str(exc)
            if self._stop.is_set():
                return
            self._on_snapshot(records, error)
            self._stop.wait(self._interval)
