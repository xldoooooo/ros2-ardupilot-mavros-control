#!/usr/bin/env python3
"""独立摄像头后台服务及无需GUI的控制命令入口。"""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import signal
import sys
from typing import Any

from camera_app.config import RuntimePaths
from camera_app.controller import CameraController, CameraServiceError
from camera_app.ipc import CameraServiceClient, CameraServiceServer


def _serve() -> int:
    """在当前用户私有Unix Socket上运行摄像头状态机。"""
    paths = RuntimePaths.discover()
    controller = CameraController(paths=paths)
    try:
        server = CameraServiceServer(controller, paths=paths)
    except CameraServiceError as exc:
        print(f"camera-service: {exc}", file=sys.stderr)
        controller.close()
        return 1

    def request_shutdown(_signum: int, _frame: object) -> None:
        server.stop_requested.set()

    def request_snapshot(_signum: int, _frame: object) -> None:
        controller.signal_snapshot()

    for handled in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(handled, request_shutdown)
    signal.signal(signal.SIGUSR1, request_snapshot)

    try:
        server.serve_until_stopped()
    finally:
        controller.close()
        server.close_and_cleanup()
    return 0


def _load_json_argument(value: str | None) -> dict[str, Any]:
    """读取CLI内联JSON或@文件，并保证根节点为对象。"""
    if not value:
        return {}
    try:
        if value.startswith("@"):
            raw = json.loads(Path(value[1:]).read_text(encoding="utf-8"))
        else:
            raw = json.loads(value)
    except (OSError, json.JSONDecodeError) as exc:
        raise CameraServiceError(f"无法读取JSON配置：{exc}") from exc
    if not isinstance(raw, dict):
        raise CameraServiceError("JSON配置根节点必须是对象")
    return raw


def _request(command: str, config_argument: str | None) -> int:
    """向已有后台发送一条命令并打印稳定JSON结果。"""
    client = CameraServiceClient(timeout=15.0)
    payload: dict[str, Any] = {}
    if command in {"start", "configure"} and config_argument:
        payload["config"] = _load_json_argument(config_argument)
    try:
        result = client.request(command, payload, timeout=20.0)
    except CameraServiceError as exc:
        print(f"camera-service: {exc}", file=sys.stderr)
        return 1
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0


def _build_parser() -> argparse.ArgumentParser:
    """创建serve/status/start/stop/snapshot等有限命令解析器。"""
    parser = argparse.ArgumentParser(description="独立USB摄像头RTSP与录像服务")
    parser.add_argument(
        "command",
        choices=(
            "serve",
            "status",
            "probe",
            "configure",
            "start",
            "stop",
            "snapshot",
            "shutdown",
        ),
    )
    parser.add_argument(
        "--config-json",
        help="内联JSON，或以@开头引用JSON文件；用于start/configure",
    )
    return parser


def main() -> int:
    """运行后台服务或向已运行后台发送一次控制请求。"""
    arguments = _build_parser().parse_args()
    if arguments.command == "serve":
        return _serve()
    return _request(arguments.command, arguments.config_json)


if __name__ == "__main__":
    raise SystemExit(main())
