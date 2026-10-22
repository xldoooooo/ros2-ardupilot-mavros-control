"""Share local ground-relay warnings with the camera preview panel."""

from __future__ import annotations

import hashlib
import json
import os
from pathlib import Path
import tempfile
from urllib.parse import urlsplit


def _status_directory() -> Path:
    """Locate the current user's lightweight relay status directory."""
    runtime = os.environ.get("XDG_RUNTIME_DIR")
    if runtime:
        return Path(runtime) / "ros2-ardupilot-relay"
    return Path(tempfile.gettempdir()) / f"ros2-ardupilot-relay-{os.getuid()}"


def status_path(output: str) -> Path:
    """Give each output its own status file, including non-RTSP outputs."""
    digest = hashlib.sha256(output.encode()).hexdigest()[:16]
    return _status_directory() / f"{digest}.json"


def write_status(path: Path, payload: dict) -> None:
    """Replace the status atomically so readers never observe partial JSON."""
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(
            mode="w", encoding="utf-8", dir=path.parent, delete=False
        ) as stream:
            temporary = Path(stream.name)
            json.dump(payload, stream, ensure_ascii=False)
            stream.write("\n")
        temporary.replace(path)
    finally:
        if temporary is not None:
            temporary.unlink(missing_ok=True)


def _endpoint(url: str) -> tuple | None:
    """Compare RTSP destinations without credentials or loopback spelling."""
    try:
        parsed = urlsplit(url)
        if parsed.scheme not in {"rtsp", "rtsps"} or not parsed.hostname:
            return None
        host = parsed.hostname.lower()
        if host in {"localhost", "127.0.0.1", "::1"}:
            host = "localhost"
        port = parsed.port or (322 if parsed.scheme == "rtsps" else 554)
        return parsed.scheme, host, port, parsed.path or "/", parsed.query
    except ValueError:
        return None


def _process_alive(pid: object) -> bool:
    """Check liveness without imposing a heartbeat freshness requirement."""
    try:
        process_id = int(pid)
        if process_id <= 0:
            return False
        os.kill(process_id, 0)
        return True
    except PermissionError:
        return True
    except (TypeError, ValueError, OverflowError, OSError):
        return False


def preview_notice(url: str) -> tuple[str, str]:
    """Return an independent notice for the local relay feeding this URL."""
    endpoint = _endpoint(url)
    if endpoint is None:
        return "", ""
    matches = []
    try:
        for path in _status_directory().glob("*.json"):
            try:
                payload = json.loads(path.read_text(encoding="utf-8"))
                if isinstance(payload, dict) and _endpoint(str(payload.get("output", ""))) == endpoint:
                    matches.append((path.stat().st_mtime_ns, payload))
            except (OSError, ValueError, UnicodeError):
                continue
    except OSError:
        return "", ""
    if not matches:
        return "", ""
    # Equivalent loopback URLs may have different filenames; prefer the latest writer.
    payload = max(matches, key=lambda item: item[0])[1]
    state = str(payload.get("state", ""))
    if state in {"completed", "stopped"}:
        return "", ""
    warning = str(payload.get("warning") or "")
    detail = str(payload.get("detail") or "")
    tooltip = "\n".join(part for part in (warning, detail) if part)
    if state == "error":
        return "地面站视频处理失败" + (f"：{warning}" if warning else "，请查看详情。"), tooltip
    if not _process_alive(payload.get("pid")):
        return "地面站视频处理已停止，请检查中继进程。", tooltip
    if state == "starting":
        return "地面站视频处理正在启动…" + (f" {warning}" if warning else ""), tooltip
    if str(payload.get("backend", "")).lower().startswith("cpu"):
        notice = "地面站使用 CPU 兼容模式，处理速度可能下降。"
        return notice + (f" {warning}" if warning else ""), tooltip
    return (f"地面站视频处理提醒：{warning}" if warning else ""), tooltip
