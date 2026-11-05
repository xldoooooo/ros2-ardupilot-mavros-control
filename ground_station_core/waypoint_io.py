"""读写地面站本地 ENU 航点文件，保持导入与导出的 CSV 语义一致。"""

from __future__ import annotations

import csv
import math
from collections.abc import Callable, Iterable, Mapping
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import NamedTuple

from .config import (
    MAX_WAYPOINT_COUNT,
    WAYPOINT_HORIZONTAL_LIMIT_METERS,
    WAYPOINT_YAW_LIMIT_DEGREES,
    WAYPOINT_Z_MAX_METERS,
    WAYPOINT_Z_MIN_METERS,
)


class Waypoint(NamedTuple):
    """保留前四项坐标索引与旧构造方式，追加独立的航点行为标志。"""

    x: float
    y: float
    z: float
    yaw: float
    is_inspection: bool = False
    is_calibration: bool = False
    calibration_tag_id: int = 0
    continue_window: bool = False


# 导入结果为不可变的具名航点集合。
WaypointCollection = tuple[Waypoint, ...]

# ROS Waypoint.calibration_tag_id 使用 int32；各入口统一拒绝超范围 ID。
MAX_CALIBRATION_TAG_ID = 2**31 - 1

# 历史 CSV 的五列表头仍按普通航点导入。
_LEGACY_CSV_HEADER = ("index", "x", "y", "z", "yaw")
# 新 CSV 追加行为字段；yaw 仍以角度存储。
_CSV_HEADER = _LEGACY_CSV_HEADER + Waypoint._fields[4:]


class WaypointImportError(ValueError):
    """表示文件格式、内容或安全边界不符合航点导入要求。"""


class WaypointExportError(ValueError):
    """表示导出路径、航点内容或文件写入不符合导出要求。"""


@dataclass(frozen=True)
class WaypointFileFormat:
    """描述一种航点文件格式；以后加入 Excel 时只需注册新的加载器。"""

    name: str
    suffixes: tuple[str, ...]
    loader: Callable[[Path], WaypointCollection]


def _parse_float(
    value: str,
    *,
    field: str,
    line_number: int,
    minimum: float,
    maximum: float,
) -> float:
    """解析有限浮点数并按与手动编辑器一致的范围拒绝越界值。"""
    try:
        number = float(value.strip())
    except ValueError as exc:
        raise WaypointImportError(
            f"第 {line_number} 行的 {field} 不是有效数字：{value!r}"
        ) from exc
    if not math.isfinite(number):
        raise WaypointImportError(f"第 {line_number} 行的 {field} 必须是有限数字")
    if not minimum <= number <= maximum:
        raise WaypointImportError(
            f"第 {line_number} 行的 {field} 超出允许范围 "
            f"[{minimum:g}, {maximum:g}]"
        )
    return number


def _parse_bool(value: str, *, field: str, line_number: int) -> bool:
    """接受明确的 true/false 或 1/0，禁止用非空字符串推断布尔值。"""
    normalized = value.strip().lower()
    if normalized in ("true", "1"):
        return True
    if normalized in ("false", "0"):
        return False
    raise WaypointImportError(
        f"第 {line_number} 行的 {field} 必须是布尔值 true/false 或 1/0"
    )


def _parse_tag_id(value: str, *, line_number: int) -> int:
    """校准 Tag ID 只接受非负整数，不接受小数或隐式转换。"""
    normalized = value.strip()
    if not normalized.isdecimal():
        raise WaypointImportError(
            f"第 {line_number} 行的 calibration_tag_id 必须是非负整数"
        )
    try:
        tag_id = int(normalized)
    except ValueError as exc:
        raise WaypointImportError(
            f"第 {line_number} 行的 calibration_tag_id 不是有效整数"
        ) from exc
    if tag_id > MAX_CALIBRATION_TAG_ID:
        raise WaypointImportError(
            f"第 {line_number} 行的 calibration_tag_id 必须是非负32位整数"
        )
    return tag_id


def normalize_waypoint(values: Iterable[object]) -> Waypoint:
    """校验完整四/八项记录，统一导出与提交入口，禁止截断或补齐非法字段。"""
    if isinstance(values, (str, bytes, Mapping)):
        raise ValueError("航点必须为四项坐标或八项完整记录")
    try:
        fields = tuple(values)
    except TypeError as exc:
        raise ValueError("航点必须为四项坐标或八项完整记录") from exc
    if len(fields) not in (4, 8):
        raise ValueError("航点必须为四项坐标或八项完整记录")
    try:
        coordinates = tuple(float(value) for value in fields[:4])
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError("航点坐标必须包含四个有限数值") from exc
    if any(isinstance(value, bool) for value in fields[:4]) or not all(
        math.isfinite(value) for value in coordinates
    ):
        raise ValueError("航点坐标必须包含四个有限数值")
    x, y, z, yaw = coordinates
    if not (
        abs(x) <= WAYPOINT_HORIZONTAL_LIMIT_METERS
        and abs(y) <= WAYPOINT_HORIZONTAL_LIMIT_METERS
        and WAYPOINT_Z_MIN_METERS <= z <= WAYPOINT_Z_MAX_METERS
        and abs(math.degrees(yaw)) <= WAYPOINT_YAW_LIMIT_DEGREES
    ):
        raise ValueError("航点坐标超出地面站允许范围")
    point = Waypoint(*coordinates, *fields[4:])
    for field in ("is_inspection", "is_calibration", "continue_window"):
        if not isinstance(getattr(point, field), bool):
            raise ValueError(f"{field} 必须是布尔值")
    if (
        isinstance(point.calibration_tag_id, bool)
        or not isinstance(point.calibration_tag_id, int)
        or not 0 <= point.calibration_tag_id <= MAX_CALIBRATION_TAG_ID
    ):
        raise ValueError("calibration_tag_id 必须是非负32位整数")
    return point


def _load_csv(path: Path) -> WaypointCollection:
    """读取新九列或历史五列 UTF-8 CSV，保持行顺序与追加字段。"""
    waypoints: list[Waypoint] = []
    try:
        with path.open("r", encoding="utf-8-sig", newline="") as stream:
            reader = csv.reader(stream, strict=True)
            try:
                header = next(reader)
            except StopIteration as exc:
                raise WaypointImportError("CSV 文件为空，缺少表头") from exc

            normalized_header = tuple(cell.strip().lower() for cell in header)
            if normalized_header not in (_CSV_HEADER, _LEGACY_CSV_HEADER):
                raise WaypointImportError(
                    f"CSV 表头必须依次为 {','.join(_CSV_HEADER)}；"
                    "也支持旧表头 index,x,y,z,yaw"
                )

            for line_number, row in enumerate(reader, start=2):
                # 允许编辑器在文件末尾留下纯空行，其他缺列/多列情况均拒绝。
                if not row or (len(row) == 1 and not row[0].strip()):
                    continue
                if len(row) != len(normalized_header):
                    raise WaypointImportError(
                        f"第 {line_number} 行应有 {len(normalized_header)} 列，"
                        f"实际为 {len(row)} 列"
                    )
                if len(waypoints) >= MAX_WAYPOINT_COUNT:
                    raise WaypointImportError(
                        f"单次最多导入 {MAX_WAYPOINT_COUNT} 个航点"
                    )

                expected_index = len(waypoints) + 1
                try:
                    actual_index = int(row[0].strip())
                except ValueError as exc:
                    raise WaypointImportError(
                        f"第 {line_number} 行的 index 必须是整数"
                    ) from exc
                if actual_index != expected_index:
                    raise WaypointImportError(
                        f"第 {line_number} 行的 index 应为 {expected_index}，"
                        f"实际为 {actual_index}"
                    )

                x = _parse_float(
                    row[1],
                    field="x",
                    line_number=line_number,
                    minimum=-WAYPOINT_HORIZONTAL_LIMIT_METERS,
                    maximum=WAYPOINT_HORIZONTAL_LIMIT_METERS,
                )
                y = _parse_float(
                    row[2],
                    field="y",
                    line_number=line_number,
                    minimum=-WAYPOINT_HORIZONTAL_LIMIT_METERS,
                    maximum=WAYPOINT_HORIZONTAL_LIMIT_METERS,
                )
                z = _parse_float(
                    row[3],
                    field="z",
                    line_number=line_number,
                    minimum=WAYPOINT_Z_MIN_METERS,
                    maximum=WAYPOINT_Z_MAX_METERS,
                )
                yaw_degrees = _parse_float(
                    row[4],
                    field="yaw",
                    line_number=line_number,
                    minimum=-WAYPOINT_YAW_LIMIT_DEGREES,
                    maximum=WAYPOINT_YAW_LIMIT_DEGREES,
                )
                waypoint = Waypoint(x, y, z, math.radians(yaw_degrees))
                if normalized_header == _CSV_HEADER:
                    waypoint = waypoint._replace(
                        is_inspection=_parse_bool(
                            row[5], field="is_inspection", line_number=line_number
                        ),
                        is_calibration=_parse_bool(
                            row[6], field="is_calibration", line_number=line_number
                        ),
                        calibration_tag_id=_parse_tag_id(row[7], line_number=line_number),
                        continue_window=_parse_bool(
                            row[8], field="continue_window", line_number=line_number
                        ),
                    )
                waypoints.append(waypoint)
    except UnicodeDecodeError as exc:
        raise WaypointImportError("CSV 文件必须使用 UTF-8 编码") from exc
    except csv.Error as exc:
        raise WaypointImportError(f"CSV 语法错误：{exc}") from exc
    except OSError as exc:
        raise WaypointImportError(f"无法读取文件：{exc}") from exc

    if not waypoints:
        raise WaypointImportError("CSV 文件没有可导入的航点数据")
    return tuple(waypoints)


# 格式分派表是未来 Excel 支持的扩展点；当前任务只注册 CSV。
_WAYPOINT_FILE_FORMATS = (
    WaypointFileFormat("CSV 文件", (".csv",), _load_csv),
)


def waypoint_file_dialog_filter() -> str:
    """返回与已注册格式同步的 Qt 单文件选择器过滤器。"""
    patterns = " ".join(
        f"*{suffix}"
        for file_format in _WAYPOINT_FILE_FORMATS
        for suffix in file_format.suffixes
    )
    format_filters = ";;".join(
        f"{file_format.name} "
        f"({' '.join(f'*{suffix}' for suffix in file_format.suffixes)})"
        for file_format in _WAYPOINT_FILE_FORMATS
    )
    return f"支持的航点文件 ({patterns});;{format_filters}"


def waypoint_export_dialog_filter() -> str:
    """返回只允许 CSV 的 Qt 保存文件选择器过滤器。"""
    return "CSV 文件 (*.csv)"


def default_waypoint_export_path(moment: datetime | None = None) -> Path:
    """按本地时间生成项目 export 子目录中的默认 CSV 文件名。"""
    timestamp = (moment or datetime.now()).strftime("%Y%m%d-%H%M%S")
    return Path("export") / f"waypoints-export-{timestamp}.csv"


def _format_csv_number(value: float) -> str:
    """以足够回读精度输出有限浮点数，同时避免无意义的尾随零。"""
    return format(value, ".15g")


def save_waypoint_file(
    path: str | Path, waypoints: Iterable[Waypoint | tuple[float, float, float, float]]
) -> Path:
    """把地面站当前列表保存为可由 :func:`load_waypoint_file` 回读的 CSV。"""
    destination = Path(path).expanduser()
    if not destination.suffix:
        destination = destination.with_suffix(".csv")
    elif destination.suffix.lower() != ".csv":
        raise WaypointExportError("航点只能导出为 .csv 文件")

    normalized: list[Waypoint] = []
    for index, waypoint in enumerate(waypoints, start=1):
        try:
            normalized.append(normalize_waypoint(waypoint))
        except ValueError as exc:
            raise WaypointExportError(f"航点 #{index}：{exc}") from exc

    if not normalized:
        raise WaypointExportError("航点列表为空，无法导出")
    if len(normalized) > MAX_WAYPOINT_COUNT:
        raise WaypointExportError(f"最多导出 {MAX_WAYPOINT_COUNT} 个航点")

    try:
        with destination.open("w", encoding="utf-8", newline="") as stream:
            writer = csv.writer(stream, lineterminator="\n")
            writer.writerow(_CSV_HEADER)
            for index, point in enumerate(normalized, start=1):
                writer.writerow(
                    (
                        index,
                        _format_csv_number(point.x),
                        _format_csv_number(point.y),
                        _format_csv_number(point.z),
                        _format_csv_number(math.degrees(point.yaw)),
                        str(point.is_inspection).lower(),
                        str(point.is_calibration).lower(),
                        point.calibration_tag_id,
                        str(point.continue_window).lower(),
                    )
                )
    except OSError as exc:
        raise WaypointExportError(f"无法写入文件：{exc}") from exc
    return destination


def load_waypoint_file(path: str | Path) -> WaypointCollection:
    """按扩展名选择加载器，并返回内部使用的弧度偏航航点副本。"""
    source = Path(path).expanduser()
    suffix = source.suffix.lower()
    for file_format in _WAYPOINT_FILE_FORMATS:
        if suffix in file_format.suffixes:
            return file_format.loader(source)
    supported = ", ".join(
        suffix
        for file_format in _WAYPOINT_FILE_FORMATS
        for suffix in file_format.suffixes
    )
    raise WaypointImportError(
        f"不支持的航点文件格式 {suffix or '（无扩展名）'}；当前支持：{supported}"
    )
