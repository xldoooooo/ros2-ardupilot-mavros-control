"""按本地 YAML 为上位机航线补充航点行为，保留原始业务点号和照片编号。"""

from __future__ import annotations

import json
import math
from collections.abc import Mapping
from dataclasses import dataclass
from pathlib import Path

import yaml

from ..config import MAX_WAYPOINT_COUNT, PROJECT_ROOT
from ..waypoint_io import Waypoint, WaypointCollection, normalize_waypoint

# 默认文件负责未匹配的航线；其他 YAML 可通过 route_ids 声明专用航线。
PROFILE_DIRECTORY = PROJECT_ROOT / "config" / "ground" / "route_profiles"


@dataclass(frozen=True)
class AdaptedRoute:
    """一次接收生成的不可变执行副本；点号 0 仅标记不回报的本地插入点。"""

    name: str
    waypoints: WaypointCollection
    point_indexes: tuple[int, ...]
    photo_nos: tuple[str, ...]


def _load_profile(route_id: object, directory: Path) -> Mapping:
    """选取唯一专用配置，否则使用 default.yaml；配置错误明确返回给操作者。"""
    selected = None
    for path in sorted(directory.glob("*.yaml")):
        if path.name == "default.yaml":
            continue
        profile = yaml.safe_load(path.read_text(encoding="utf-8"))
        if not isinstance(profile, Mapping):
            raise ValueError(f"{path.name} 必须是 YAML 对象")
        if str(route_id) in tuple(str(value) for value in profile.get("route_ids", ())):
            if selected is not None:
                raise ValueError(f"航线 {route_id!r} 匹配多份适配配置")
            selected = profile
    if selected is None:
        selected = yaml.safe_load((directory / "default.yaml").read_text(encoding="utf-8"))
    if not isinstance(selected, Mapping):
        raise ValueError("航线适配配置必须是 YAML 对象")
    return selected


def _attributes(point: Waypoint, values: Mapping) -> Waypoint:
    """只覆盖显式行为字段；当前飞行执行仅支持 stop 到点方式。"""
    if values.get("arrival_mode", "stop") != "stop":
        raise ValueError("航线适配暂不支持非 stop 的 arrival_mode")
    if "is_inspection" in values:
        point = point._replace(is_inspection=values["is_inspection"])
    if "calibration" in values:
        calibration = values["calibration"]
        if calibration is None or calibration is False:
            point = point._replace(is_calibration=False, continue_window=False)
        elif isinstance(calibration, Mapping):
            point = point._replace(
                is_calibration=True,
                calibration_tag_id=calibration["tag_id"],
                continue_window=calibration.get("continue_window", False),
            )
        else:
            raise ValueError("calibration 必须是包含 tag_id 的对象，或 false")
    return normalize_waypoint(point)


def adapt_route(
    waypoints: WaypointCollection,
    point_indexes: tuple[int, ...],
    photo_nos: tuple[str, ...],
    route_id: object = "",
    *,
    directory: Path | None = None,
) -> AdaptedRoute:
    """从原始列表生成一次执行副本；所有锚点始终引用原始 index，first 表示首行。"""
    try:
        profile = _load_profile(route_id, directory or PROFILE_DIRECTORY)
        original = list(waypoints)
        if not original or len(original) != len(point_indexes) or len(original) != len(photo_nos):
            raise ValueError("原始航点、业务点号和照片编号必须逐点对应")
        positions = {index: row for row, index in enumerate(point_indexes)}

        def locate(value: object) -> int:
            """定位原始点，插入操作不会影响后续规则引用。"""
            if value == "first":
                return 0
            if value not in positions:
                raise ValueError(f"适配配置引用了不存在的原始点号 {value!r}")
            return positions[value]

        defaults = profile.get("defaults", {})
        original = [_attributes(point, defaults) for point in original]
        for override in profile.get("point_overrides", ()):
            row = locate(override["point_index"])
            original[row] = _attributes(original[row], override)

        before: dict[int, list] = {}
        after: dict[int, list] = {}
        for insertion in profile.get("insert_points", ()):
            if ("after_point" in insertion) == ("before_point" in insertion):
                raise ValueError("插入点必须且只能指定 after_point 或 before_point")
            anchor = "after_point" if "after_point" in insertion else "before_point"
            row = locate(insertion[anchor])
            x, y, z = insertion["position"]
            point = _attributes(Waypoint(x, y, z, math.radians(insertion.get("yaw_deg", 0))), insertion)
            index, photo = 0, ""
            if point.is_inspection:
                index = insertion["point_index"]
                if isinstance(index, bool) or not isinstance(index, int) or index <= 0:
                    raise ValueError("插入巡检点必须提供正整数 point_index")
                photo = insertion["photo_no"]
                if not isinstance(photo, str):
                    photo = json.dumps(photo, ensure_ascii=False, separators=(",", ":"))
            target = after if anchor == "after_point" else before
            target.setdefault(row, []).append((point, index, photo))

        entries = []
        for row, point in enumerate(original):
            entries.extend(before.get(row, ()))
            entries.append((point, point_indexes[row], photo_nos[row]))
            entries.extend(after.get(row, ()))
        if len(entries) > MAX_WAYPOINT_COUNT:
            raise ValueError(f"适配后的航点数超过 {MAX_WAYPOINT_COUNT}")
        return AdaptedRoute(
            name=str(profile.get("name", "本地航线适配")),
            waypoints=tuple(entry[0] for entry in entries),
            point_indexes=tuple(entry[1] for entry in entries),
            photo_nos=tuple(entry[2] for entry in entries),
        )
    except (OSError, yaml.YAMLError, KeyError, TypeError, ValueError) as exc:
        raise ValueError(f"航线适配失败：{exc}") from exc
