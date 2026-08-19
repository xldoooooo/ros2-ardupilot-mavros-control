"""摄像头后台服务的本机 Unix Socket JSON 控制协议。"""

from __future__ import annotations

import json
import os
from pathlib import Path
import socket
import socketserver
import stat
import threading
import time
from typing import Any

from .config import RuntimePaths
from .controller import CameraController, CameraServiceError


_MAX_MESSAGE_BYTES = 1024 * 1024


class CameraServiceClient:
    """面板和命令行共用的同步本机控制客户端。"""

    def __init__(
        self,
        *,
        paths: RuntimePaths | None = None,
        timeout: float = 2.0,
    ) -> None:
        self.paths = paths or RuntimePaths.discover()
        self.timeout = timeout

    def request(
        self,
        command: str,
        payload: dict[str, Any] | None = None,
        *,
        timeout: float | None = None,
    ) -> dict[str, Any]:
        """发送一条请求并把服务端错误转成可展示异常。"""
        message = json.dumps(
            {"command": command, "payload": payload or {}},
            ensure_ascii=False,
        ).encode("utf-8") + b"\n"
        if len(message) > _MAX_MESSAGE_BYTES:
            raise CameraServiceError("摄像头控制请求过大")

        connection = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        connection.settimeout(timeout if timeout is not None else self.timeout)
        try:
            connection.connect(str(self.paths.socket_file))
            connection.sendall(message)
            response = self._receive_line(connection)
        except (FileNotFoundError, ConnectionRefusedError) as exc:
            raise CameraServiceError("摄像头后台服务尚未启动") from exc
        except socket.timeout as exc:
            raise CameraServiceError("摄像头后台服务响应超时") from exc
        except OSError as exc:
            raise CameraServiceError(f"摄像头后台通信失败：{exc}") from exc
        finally:
            connection.close()

        try:
            decoded = json.loads(response.decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError) as exc:
            raise CameraServiceError("摄像头后台返回了无效响应") from exc
        if not isinstance(decoded, dict):
            raise CameraServiceError("摄像头后台响应类型无效")
        if not decoded.get("ok"):
            raise CameraServiceError(str(decoded.get("error") or "摄像头请求失败"))
        result = decoded.get("result", {})
        return result if isinstance(result, dict) else {"value": result}

    @staticmethod
    def _receive_line(connection: socket.socket) -> bytes:
        """有界接收单行JSON，防止服务异常时无限占用内存。"""
        chunks = bytearray()
        while len(chunks) <= _MAX_MESSAGE_BYTES:
            block = connection.recv(65536)
            if not block:
                break
            chunks.extend(block)
            if b"\n" in block:
                break
        if len(chunks) > _MAX_MESSAGE_BYTES:
            raise CameraServiceError("摄像头后台响应过大")
        return bytes(chunks).partition(b"\n")[0]

    def wait_until_ready(self, timeout: float = 5.0) -> dict[str, Any]:
        """在拉起独立服务后有界等待Socket可用。"""
        deadline = time.monotonic() + timeout
        last_error = ""
        while time.monotonic() < deadline:
            try:
                return self.request("status", timeout=0.4)
            except CameraServiceError as exc:
                last_error = str(exc)
                time.sleep(0.08)
        raise CameraServiceError(f"摄像头后台服务启动超时：{last_error}")


class _CameraRequestHandler(socketserver.StreamRequestHandler):
    """每条本机连接处理一个换行结尾的JSON请求。"""

    def handle(self) -> None:
        raw = self.rfile.readline(_MAX_MESSAGE_BYTES + 1)
        if len(raw) > _MAX_MESSAGE_BYTES:
            self._send({"ok": False, "error": "请求过大"})
            return
        try:
            request = json.loads(raw.decode("utf-8"))
            if not isinstance(request, dict):
                raise ValueError("请求根节点必须是对象")
            command = str(request.get("command", ""))
            payload = request.get("payload", {})
            if not isinstance(payload, dict):
                raise ValueError("payload必须是对象")
            server = self.server
            result = server.dispatch(command, payload)  # type: ignore[attr-defined]
            self._send({"ok": True, "result": result})
        except (CameraServiceError, ValueError) as exc:
            self._send({"ok": False, "error": str(exc)})
        except Exception as exc:
            self._send({"ok": False, "error": f"后台内部错误：{exc}"})

    def _send(self, payload: dict[str, Any]) -> None:
        """返回单行 UTF-8 JSON，不在协议中传输二进制视频。"""
        encoded = json.dumps(payload, ensure_ascii=False).encode("utf-8") + b"\n"
        self.wfile.write(encoded)


class CameraServiceServer(socketserver.ThreadingUnixStreamServer):
    """只绑定当前用户私有运行目录的多请求摄像头控制服务。"""

    daemon_threads = True
    allow_reuse_address = False

    def __init__(
        self,
        controller: CameraController,
        *,
        paths: RuntimePaths | None = None,
    ) -> None:
        self.controller = controller
        self.paths = paths or controller.paths
        self.stop_requested = threading.Event()
        self.paths.ensure_directories()
        self._prepare_socket_path()
        super().__init__(str(self.paths.socket_file), _CameraRequestHandler)
        os.chmod(self.paths.socket_file, 0o600)
        self.paths.pid_file.write_text(f"{os.getpid()}\n", encoding="utf-8")
        os.chmod(self.paths.pid_file, 0o600)

    def dispatch(self, command: str, payload: dict[str, Any]) -> dict[str, Any]:
        """把有限命令集合映射到控制器，拒绝任意代码或shell执行。"""
        if command == "status":
            return self.controller.status()
        if command == "probe":
            device = payload.get("device")
            return self.controller.probe(str(device) if device else None)
        if command == "configure":
            config = payload.get("config", payload)
            if not isinstance(config, dict):
                raise CameraServiceError("config必须是对象")
            return self.controller.configure(config)
        if command == "start":
            config = payload.get("config")
            if config is not None and not isinstance(config, dict):
                raise CameraServiceError("config必须是对象")
            return self.controller.start(config)
        if command == "stop":
            return self.controller.stop()
        if command == "snapshot":
            return self.controller.request_snapshot(
                kind=str(payload.get("kind", "manual")),
                photo_no=str(payload.get("photo_no", "")),
            )
        if command == "shutdown":
            if self.controller.status()["running"]:
                raise CameraServiceError("请先关闭摄像头，再退出后台服务")
            self.stop_requested.set()
            return {"shutting_down": True}
        raise CameraServiceError(f"未知摄像头命令：{command}")

    def serve_until_stopped(self) -> None:
        """以短超时处理请求，使POSIX信号能及时结束主循环。"""
        self.timeout = 0.4
        while not self.stop_requested.is_set():
            self.handle_request()

    def close_and_cleanup(self) -> None:
        """关闭监听并仅删除本服务创建且仍属于当前PID的运行文件。"""
        self.server_close()
        self._unlink_if_owned(self.paths.pid_file, require_pid=True)
        self._unlink_if_owned(self.paths.socket_file, require_pid=False)

    def _prepare_socket_path(self) -> None:
        """拒绝覆盖活跃Socket或运行目录中的任意普通文件。"""
        socket_path = self.paths.socket_file
        if len(os.fsencode(socket_path)) >= 104:
            raise CameraServiceError(f"Unix Socket路径过长：{socket_path}")
        try:
            file_status = socket_path.lstat()
        except FileNotFoundError:
            return
        if not stat.S_ISSOCK(file_status.st_mode):
            raise CameraServiceError(f"运行路径已存在且不是Socket：{socket_path}")
        probe = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        probe.settimeout(0.25)
        try:
            probe.connect(str(socket_path))
        except OSError:
            socket_path.unlink()
        else:
            raise CameraServiceError("摄像头后台服务已经运行")
        finally:
            probe.close()

    def _unlink_if_owned(self, path: Path, *, require_pid: bool) -> None:
        """清理前复核PID文件内容，避免删除另一个新服务的状态。"""
        try:
            if require_pid:
                content = path.read_text(encoding="utf-8").strip()
                if content != str(os.getpid()):
                    return
            path.unlink(missing_ok=True)
        except OSError:
            pass
