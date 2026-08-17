"""协议 V2.0 的业务 JSON、主题名和最小字段校验。

这里忠实实现文档已有字段，不补充消息编号、时间戳、重试或额外 ACK。
"""

from __future__ import annotations

from copy import deepcopy
from typing import Any, Mapping


COMMAND_LABELS = {
    "02": "巡检任务下发",
    "03": "执行巡检任务",
    "05": "一键返航",
    "07": "紧急停机",
}

# 0C 只出现在权威 DOCX 的表 2 中，复制版 TXT 漏掉了这一行。
STATUS_LABELS = {
    "01": "待机",
    "02": "接收航线任务",
    "03": "巡检中",
    "05": "返航",
    "07": "降落",
    "08": "任务执行完毕",
    "09": "到达巡检点位",
    "0A": "电量百分比",
    "0B": "无人机位置",
    "0C": "巡检电量不足，暂停巡检，返航充电",
}

WAYPOINT_FIELDS = (
    "index",
    "x",
    "y",
    "z",
    "forwardAngle",
    "cameraAngle",
    "photoNo",
)


class ProtocolError(ValueError):
    """业务消息不符合协议 V2.0 已声明字段时抛出。"""


def command_topic(client_no: str) -> str:
    """返回指定飞机的控制主题。"""

    return f"drone/{_client_no(client_no)}/command"


def status_topic(client_no: str) -> str:
    """返回指定飞机的状态主题。"""

    return f"drone/{_client_no(client_no)}/status"


def _client_no(value: Any) -> str:
    """校验协议中提前约定的无人机编号。"""

    if not isinstance(value, str) or not value.strip():
        raise ProtocolError("clientNo 必须是非空字符串")
    return value


def _number(value: Any, field: str) -> None:
    """校验 JSON 数值并排除 Python 中属于 int 子类的布尔值。"""

    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ProtocolError(f"{field} 必须是数值")


def validate_command(
    payload: Mapping[str, Any], expected_client_no: str | None = None
) -> dict[str, Any]:
    """校验四类下发命令，返回与输入等值的普通字典副本。"""

    if not isinstance(payload, Mapping):
        raise ProtocolError("命令必须是 JSON 对象")
    client_no = _client_no(payload.get("clientNo"))
    if expected_client_no is not None and client_no != expected_client_no:
        raise ProtocolError(
            f"命令 clientNo={client_no!r} 与当前飞机 {expected_client_no!r} 不一致"
        )
    command_no = payload.get("commandNo")
    if command_no not in COMMAND_LABELS:
        raise ProtocolError(f"不支持的 commandNo: {command_no!r}")
    if command_no == "02":
        _validate_task_points(payload.get("taskPoints"))
    return deepcopy(dict(payload))


def _validate_task_points(value: Any) -> None:
    """校验巡检任务中的航点数组和文档列出的七个字段。"""

    if not isinstance(value, list) or not value:
        raise ProtocolError("commandNo=02 的 taskPoints 必须是非空数组")
    for position, point in enumerate(value, start=1):
        if not isinstance(point, Mapping):
            raise ProtocolError(f"taskPoints[{position}] 必须是 JSON 对象")
        missing = [field for field in WAYPOINT_FIELDS if field not in point]
        if missing:
            raise ProtocolError(
                f"taskPoints[{position}] 缺少字段: {', '.join(missing)}"
            )
        if isinstance(point["index"], bool) or not isinstance(point["index"], int):
            raise ProtocolError(f"taskPoints[{position}].index 必须是整数")
        if isinstance(point["photoNo"], bool) or not isinstance(
            point["photoNo"], int
        ):
            raise ProtocolError(f"taskPoints[{position}].photoNo 必须是整数")
        for field in ("x", "y", "z", "forwardAngle", "cameraAngle"):
            _number(point[field], f"taskPoints[{position}].{field}")


def command_ack(command: Mapping[str, Any]) -> dict[str, str]:
    """按表 1 生成命令确认；协议没有定义额外 ACK 包装。"""

    checked = validate_command(command)
    return {
        "clientNo": checked["clientNo"],
        "commandNo": checked["commandNo"],
    }


def make_status(
    client_no: str, status: str, data: Mapping[str, Any] | None = None
) -> dict[str, Any]:
    """构造并校验协议表 2 的状态消息。"""

    payload: dict[str, Any] = {
        "clientNo": _client_no(client_no),
        "uavStatus": status,
    }
    if data is not None:
        payload["data"] = deepcopy(dict(data))
    return validate_status(payload)


def validate_status(
    payload: Mapping[str, Any], expected_client_no: str | None = None
) -> dict[str, Any]:
    """校验十种状态消息及其文档中声明的数据字段。"""

    if not isinstance(payload, Mapping):
        raise ProtocolError("状态必须是 JSON 对象")
    client_no = _client_no(payload.get("clientNo"))
    if expected_client_no is not None and client_no != expected_client_no:
        raise ProtocolError(
            f"状态 clientNo={client_no!r} 与当前飞机 {expected_client_no!r} 不一致"
        )
    status = payload.get("uavStatus")
    if status not in STATUS_LABELS:
        raise ProtocolError(f"不支持的 uavStatus: {status!r}")

    required_fields: dict[str, tuple[str, ...]] = {
        "08": ("videoPath", "JPGPath"),
        "09": ("pointNo", "pointName", "pointPic"),
        "0A": ("uavPower",),
        "0B": ("X", "Y", "Z"),
    }
    if status in required_fields:
        data = payload.get("data")
        if not isinstance(data, Mapping):
            raise ProtocolError(f"uavStatus={status} 必须包含 data 对象")
        missing = [field for field in required_fields[status] if field not in data]
        if missing:
            raise ProtocolError(
                f"uavStatus={status} 的 data 缺少字段: {', '.join(missing)}"
            )
        if status in ("08", "09"):
            for field in required_fields[status]:
                if not isinstance(data[field], str):
                    raise ProtocolError(f"uavStatus={status}.data.{field} 必须是字符串")
        elif status == "0A":
            _number(data["uavPower"], "uavStatus=0A.data.uavPower")
        else:
            for field in ("X", "Y", "Z"):
                _number(data[field], f"uavStatus=0B.data.{field}")
    return deepcopy(dict(payload))


def sample_commands(client_no: str = "UAV01001") -> list[dict[str, Any]]:
    """返回覆盖表 1 的四条可直接发送的示例命令。"""

    return [
        {
            "clientNo": client_no,
            "commandNo": "02",
            "taskPoints": [
                {
                    "index": 1,
                    "x": 3.4,
                    "y": -0.549,
                    "z": 0.221,
                    "forwardAngle": 0,
                    "cameraAngle": 1495,
                    "photoNo": 1,
                },
                {
                    "index": 2,
                    "x": 14.67,
                    "y": -0.548,
                    "z": 0.219,
                    "forwardAngle": -90,
                    "cameraAngle": 1495,
                    "photoNo": 2,
                },
            ],
        },
        {"clientNo": client_no, "commandNo": "03"},
        {"clientNo": client_no, "commandNo": "05"},
        {"clientNo": client_no, "commandNo": "07"},
    ]


def sample_statuses(client_no: str = "UAV01001") -> list[dict[str, Any]]:
    """返回覆盖权威 DOCX 表 2（含 0C）的十条状态。"""

    return [
        make_status(client_no, "01"),
        make_status(client_no, "02"),
        make_status(client_no, "03"),
        make_status(client_no, "05"),
        make_status(client_no, "07"),
        make_status(
            client_no,
            "08",
            {"videoPath": "/home/share/xx.mp4", "JPGPath": "/home/share/jpg"},
        ),
        make_status(
            client_no,
            "09",
            {
                "pointNo": "02",
                "pointName": "xx巡检点位",
                "pointPic": "/home/share/jpg/xx.jpg",
            },
        ),
        make_status(client_no, "0A", {"uavPower": 55.6}),
        make_status(client_no, "0B", {"X": 55.6, "Y": 55.6, "Z": 5}),
        make_status(client_no, "0C"),
    ]

