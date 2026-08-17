"""上位机协议命令到现有地面站动作的集中映射与字段转换。"""

from __future__ import annotations

import math
from collections.abc import Mapping
from copy import deepcopy
from dataclasses import dataclass
from typing import Any

from ..config import MAX_WAYPOINT_COUNT, WAYPOINT_HORIZONTAL_LIMIT_METERS
from .models import UpstreamAction, UpstreamCommand


class UpstreamProtocolError(ValueError):
    """业务 JSON 无法安全映射为地面站动作时抛出。"""


@dataclass(frozen=True)
class CommandMapping:
    """一条可由面板直接展示的稳定命令映射。"""

    command_no: str
    label: str
    action: UpstreamAction
    local_effect: str


# 后续对接调整只需修改本文件；Qt 和 WebSocket 核心不复制业务编号。
COMMAND_MAPPINGS: tuple[CommandMapping, ...] = (
    CommandMapping("01", "起飞", UpstreamAction.TAKEOFF, "执行地面站起飞操作"),
    CommandMapping(
        "02",
        "巡检任务下发",
        UpstreamAction.STAGE_WAYPOINTS,
        "原子替换地面站航点列表",
    ),
    CommandMapping(
        "03",
        "执行巡检任务",
        UpstreamAction.EXECUTE_WAYPOINTS,
        "执行当前地面站航点任务",
    ),
    CommandMapping(
        "05",
        "一键返航",
        UpstreamAction.RETURN_HOME,
        "以当前航点配置飞至 (0, 0, 起飞高度)",
    ),
    CommandMapping("06", "降落", UpstreamAction.LAND, "执行地面站降落操作"),
    CommandMapping("07", "紧急停机", UpstreamAction.EMERGENCY_LAND, "立即原地降落"),
)
_MAPPINGS_BY_NO = {item.command_no: item for item in COMMAND_MAPPINGS}

_REQUIRED_POINT_FIELDS = (
    "index",
    "x",
    "y",
    "z",
    "forwardAngle",
    "cameraAngle",
    "photoNo",
)


def parse_command(
    payload: Mapping[str, Any], expected_client_no: str
) -> UpstreamCommand:
    """校验业务 JSON，并把角度制航向转换为本地弧度制航点。"""
    if not isinstance(payload, Mapping):
        raise UpstreamProtocolError("命令必须是 JSON 对象")
    client_no = _client_no(payload.get("clientNo"))
    if client_no != _client_no(expected_client_no):
        raise UpstreamProtocolError(
            f"命令 clientNo={client_no!r} 与当前无人机编号不一致"
        )
    command_no = payload.get("commandNo")
    mapping = _MAPPINGS_BY_NO.get(command_no)
    if mapping is None:
        raise UpstreamProtocolError(f"不支持的 commandNo: {command_no!r}")

    waypoints: tuple[tuple[float, float, float, float], ...] = ()
    point_indexes: tuple[int, ...] = ()
    ignored = False
    if command_no == "02":
        waypoints, point_indexes = _parse_task_points(payload.get("taskPoints"))
        ignored = True
    return UpstreamCommand(
        client_no=client_no,
        command_no=command_no,
        label=mapping.label,
        action=mapping.action,
        raw_payload=deepcopy(dict(payload)),
        waypoints=waypoints,
        point_indexes=point_indexes,
        ignored_camera_fields=ignored,
    )


def _parse_task_points(
    value: Any,
) -> tuple[tuple[tuple[float, float, float, float], ...], tuple[int, ...]]:
    """校验航点数组并保留协议顺序；index 仅作为状态回报编号。"""
    if not isinstance(value, list) or not value:
        raise UpstreamProtocolError("commandNo=02 的 taskPoints 必须是非空数组")
    if len(value) > MAX_WAYPOINT_COUNT:
        raise UpstreamProtocolError(f"taskPoints 最多允许 {MAX_WAYPOINT_COUNT} 个航点")

    normalized: list[tuple[float, float, float, float]] = []
    indexes: list[int] = []
    for position, point in enumerate(value, start=1):
        if not isinstance(point, Mapping):
            raise UpstreamProtocolError(f"taskPoints[{position}] 必须是 JSON 对象")
        missing = [field for field in _REQUIRED_POINT_FIELDS if field not in point]
        if missing:
            raise UpstreamProtocolError(
                f"taskPoints[{position}] 缺少字段: {', '.join(missing)}"
            )
        index = point["index"]
        photo_no = point["photoNo"]
        if isinstance(index, bool) or not isinstance(index, int) or index < 1:
            raise UpstreamProtocolError(f"taskPoints[{position}].index 必须是正整数")
        if index in indexes:
            raise UpstreamProtocolError(f"taskPoints index={index} 重复")
        if isinstance(photo_no, bool) or not isinstance(photo_no, int):
            raise UpstreamProtocolError(f"taskPoints[{position}].photoNo 必须是整数")
        x = _finite_number(point["x"], f"taskPoints[{position}].x")
        y = _finite_number(point["y"], f"taskPoints[{position}].y")
        z = _finite_number(point["z"], f"taskPoints[{position}].z")
        angle = _finite_number(
            point["forwardAngle"], f"taskPoints[{position}].forwardAngle"
        )
        _finite_number(point["cameraAngle"], f"taskPoints[{position}].cameraAngle")
        if abs(x) > WAYPOINT_HORIZONTAL_LIMIT_METERS or abs(y) > (
            WAYPOINT_HORIZONTAL_LIMIT_METERS
        ):
            raise UpstreamProtocolError(f"taskPoints[{position}] 的 X/Y 超出地面站范围")
        # 机载 ExecuteWaypoints 服务的既有边界是 [0.1, 50] m。
        if not 0.1 <= z <= 50.0:
            raise UpstreamProtocolError(
                f"taskPoints[{position}].z 必须在 [0.1, 50] m 范围内"
            )
        normalized_angle = ((angle + 180.0) % 360.0) - 180.0
        normalized.append((x, y, z, math.radians(normalized_angle)))
        indexes.append(index)
    return tuple(normalized), tuple(indexes)


def _client_no(value: Any) -> str:
    """校验无人机编号可安全嵌入主题路径。"""
    if not isinstance(value, str) or not value.strip():
        raise UpstreamProtocolError("clientNo 必须是非空字符串")
    cleaned = value.strip()
    if any(character in cleaned for character in ("/", "\\", "#", "+")):
        raise UpstreamProtocolError("clientNo 不能包含主题路径或通配符字符")
    return cleaned


def _finite_number(value: Any, field: str) -> float:
    """解析有限 JSON 数值并明确排除布尔值、NaN 与无穷大。"""
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise UpstreamProtocolError(f"{field} 必须是数值")
    result = float(value)
    if not math.isfinite(result):
        raise UpstreamProtocolError(f"{field} 必须是有限数值")
    return result
