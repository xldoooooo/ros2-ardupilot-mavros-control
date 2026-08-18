"""上位机协议 V2.0 的主题信封、确认、状态及面板示例。"""

from __future__ import annotations

import json
from collections.abc import Mapping
from copy import deepcopy
from typing import Any

from .mapping import UpstreamProtocolError, _client_no

STATUS_LABELS = {
    "01": "待机",
    "02": "接收航线任务",
    "03": "巡检中",
    "05": "返航",
    "07": "降落",
    "08": "任务执行完毕",
    "09": "到达巡检点位",
    "0A": "电量",
    "0B": "无人机位置",
    "0C": "巡检电量不足",
}


def command_topic(client_no: str) -> str:
    """返回指定无人机的上位机控制主题。"""
    return f"drone/{_client_no(client_no)}/command"


def status_topic(client_no: str) -> str:
    """返回指定无人机的上位机状态主题。"""
    return f"drone/{_client_no(client_no)}/status"


def subscribe_envelope(client_no: str) -> dict[str, str]:
    """构造 JAR 主题服务要求的订阅信封。"""
    return {"type": "SUBSCRIBE", "topic": command_topic(client_no)}


def publish_envelope(client_no: str, data: Mapping[str, Any]) -> dict[str, Any]:
    """构造向状态主题发布业务对象的信封。"""
    return {
        "type": "PUBLISH",
        "topic": status_topic(client_no),
        "data": deepcopy(dict(data)),
    }


def command_ack(client_no: str, command_no: str) -> dict[str, str]:
    """按协议表 1 返回只含无人机编号与命令编号的确认。"""
    return {"clientNo": _client_no(client_no), "commandNo": str(command_no)}


def make_status(
    client_no: str, status: str, data: Mapping[str, Any] | None = None
) -> dict[str, Any]:
    """构造一条已知状态。"""
    if status not in STATUS_LABELS:
        raise UpstreamProtocolError(f"不支持的 uavStatus: {status!r}")
    payload: dict[str, Any] = {
        "clientNo": _client_no(client_no),
        "uavStatus": status,
    }
    if data is not None:
        payload["data"] = deepcopy(dict(data))
    return payload


def decode_object(raw: Any) -> dict[str, Any]:
    """把 WebSocket 文本帧严格解析为 JSON 对象。"""
    if not isinstance(raw, str):
        raise UpstreamProtocolError("仅支持 UTF-8 JSON 文本帧")
    try:
        value = json.loads(raw)
    except json.JSONDecodeError as exc:
        raise UpstreamProtocolError(f"收到无效 JSON：{exc.msg}") from exc
    if not isinstance(value, dict):
        raise UpstreamProtocolError("WebSocket 消息必须是 JSON 对象")
    return value


def json_examples(client_no: str) -> tuple[tuple[str, dict[str, Any]], ...]:
    """返回通讯面板展示的全部命令与已实现状态信封示例。"""
    task = {
        "clientNo": client_no,
        "commandNo": "02",
        "taskPoints": [
            {
                "index": 1,
                "x": 3.4,
                "y": -0.549,
                "z": 1.2,
                "forwardAngle": -90,
                "cameraAngle": 1495,
                "photoNo": 1,
            }
        ],
    }
    command_payloads = (
        {"clientNo": client_no, "commandNo": "01"},
        task,
        {"clientNo": client_no, "commandNo": "03"},
        {"clientNo": client_no, "commandNo": "05"},
        {"clientNo": client_no, "commandNo": "06"},
        {"clientNo": client_no, "commandNo": "07"},
    )
    examples: list[tuple[str, dict[str, Any]]] = [
        ("订阅控制主题", subscribe_envelope(client_no))
    ]
    for payload in command_payloads:
        command_no = str(payload["commandNo"])
        examples.append(
            (
                f"接收命令 {command_no}（BROADCAST）",
                {
                    "type": "BROADCAST",
                    "topic": command_topic(client_no),
                    "data": deepcopy(payload),
                },
            )
        )
    examples.extend(
        (
            (
                "命令确认（PUBLISH）",
                publish_envelope(client_no, command_ack(client_no, "02")),
            ),
            (
                "状态 01 待机",
                publish_envelope(client_no, make_status(client_no, "01")),
            ),
            (
                "状态 02 接收航线任务",
                publish_envelope(client_no, make_status(client_no, "02")),
            ),
            (
                "状态 03 巡检中",
                publish_envelope(client_no, make_status(client_no, "03")),
            ),
            (
                "状态 05 返航",
                publish_envelope(client_no, make_status(client_no, "05")),
            ),
            (
                "状态 07 降落",
                publish_envelope(client_no, make_status(client_no, "07")),
            ),
            (
                "状态 08 任务执行完毕",
                publish_envelope(
                    client_no,
                    make_status(client_no, "08", {"videoPath": "", "JPGPath": ""}),
                ),
            ),
            (
                "状态 09 到达巡检点位",
                publish_envelope(
                    client_no,
                    make_status(
                        client_no,
                        "09",
                        {
                            "pointNo": "1",
                            "pointName": "巡检点位 1",
                            "pointPic": "",
                        },
                    ),
                ),
            ),
            (
                "状态 0A 电量（仿真百分比）",
                publish_envelope(
                    client_no, make_status(client_no, "0A", {"uavPower": 55.6})
                ),
            ),
            (
                "状态 0A 电量（实机电压 V）",
                publish_envelope(
                    client_no, make_status(client_no, "0A", {"uavPower": 22.8})
                ),
            ),
            (
                "状态 0B 位置",
                publish_envelope(
                    client_no,
                    make_status(client_no, "0B", {"X": 1.0, "Y": 2.0, "Z": 3.0}),
                ),
            ),
            (
                "状态 0C 巡检电量不足",
                publish_envelope(client_no, make_status(client_no, "0C")),
            ),
        )
    )
    return tuple(examples)
