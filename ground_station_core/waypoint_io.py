"""读写地面站本地 ENU 航点文件，保持导入与导出的 CSV 语义一致。"""

from __future__ import annotations

import csv
from dataclasses import dataclass
from datetime import datetime
import math
from pathlib import Path
from typing import Callable, Iterable

from .config import (
    MAX_WAYPOINT_COUNT,
    WAYPOINT_HORIZONTAL_LIMIT_METERS,
    WAYPOINT_YAW_LIMIT_DEGREES,
    WAYPOINT_Z_MAX_METERS,
    WAYPOINT_Z_MIN_METERS,
)


Waypoint = tuple[float, float, float, float]
WaypointCollection = tuple[Waypoint, ...]

_CSV_HEADER = ("index", "x", "y", "z", "yaw")


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


def _load_csv(path: Path) -> WaypointCollection:
    """严格按 index,x,y,z,yaw 表头读取 UTF-8 CSV，保持文件行顺序。"""
    waypoints: list[Waypoint] = []
    try:
        with path.open("r", encoding="utf-8-sig", newline="") as stream:
            reader = csv.reader(stream)
            try:
                header = next(reader)
            except StopIteration as exc:
                raise WaypointImportError("CSV 文件为空，缺少表头") from exc

            normalized_header = tuple(cell.strip().lower() for cell in header)
            if normalized_header != _CSV_HEADER:
                raise WaypointImportError(
                    "CSV 表头必须依次为 index,x,y,z,yaw"
                )

            for line_number, row in enumerate(reader, start=2):
                # 允许编辑器在文件末尾留下纯空行，其他缺列/多列情况均拒绝。
                if not row or all(not cell.strip() for cell in row):
                    continue
                if len(row) != len(_CSV_HEADER):
                    raise WaypointImportError(
                        f"第 {line_number} 行应有 5 列，实际为 {len(row)} 列"
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
                waypoints.append((x, y, z, math.radians(yaw_degrees)))
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
    path: str | Path, waypoints: Iterable[Waypoint]
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
            values = tuple(float(value) for value in waypoint)
        except (TypeError, ValueError) as exc:
            raise WaypointExportError(f"航点 #{index} 包含无效数值") from exc
        if len(values) != 4 or not all(math.isfinite(value) for value in values):
            raise WaypointExportError(f"航点 #{index} 必须包含四个有限数值")
        x, y, z, yaw = values
        yaw_degrees = math.degrees(yaw)
        if not (
            -WAYPOINT_HORIZONTAL_LIMIT_METERS
            <= x
            <= WAYPOINT_HORIZONTAL_LIMIT_METERS
            and -WAYPOINT_HORIZONTAL_LIMIT_METERS
            <= y
            <= WAYPOINT_HORIZONTAL_LIMIT_METERS
            and WAYPOINT_Z_MIN_METERS <= z <= WAYPOINT_Z_MAX_METERS
            and -WAYPOINT_YAW_LIMIT_DEGREES
            <= yaw_degrees
            <= WAYPOINT_YAW_LIMIT_DEGREES
        ):
            raise WaypointExportError(f"航点 #{index} 超出地面站允许范围")
        normalized.append((x, y, z, yaw))

    if not normalized:
        raise WaypointExportError("航点列表为空，无法导出")
    if len(normalized) > MAX_WAYPOINT_COUNT:
        raise WaypointExportError(f"最多导出 {MAX_WAYPOINT_COUNT} 个航点")

    try:
        with destination.open("w", encoding="utf-8", newline="") as stream:
            writer = csv.writer(stream, lineterminator="\n")
            writer.writerow(_CSV_HEADER)
            for index, (x, y, z, yaw) in enumerate(normalized, start=1):
                writer.writerow(
                    (
                        index,
                        _format_csv_number(x),
                        _format_csv_number(y),
                        _format_csv_number(z),
                        _format_csv_number(math.degrees(yaw)),
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
