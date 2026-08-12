"""CSV 航点格式分派、内容校验、数量上限与示例文件回归。"""

from __future__ import annotations

import math
from pathlib import Path

import pytest

from ground_station_core.config import MAX_WAYPOINT_COUNT, PROJECT_ROOT
from ground_station_core.waypoint_io import (
    WaypointImportError,
    load_waypoint_file,
    waypoint_file_dialog_filter,
)


def test_example_csv_preserves_order_and_converts_yaw_to_radians() -> None:
    """交付示例可直接读取，位置不做相对偏移，偏航由角度转为弧度。"""
    waypoints = load_waypoint_file(PROJECT_ROOT / "examples/waypoints-example.csv")

    assert len(waypoints) == 5
    assert waypoints[0] == (0.0, 0.0, 1.5, 0.0)
    assert waypoints[2][:3] == (2.0, 2.0, 1.8)
    assert math.isclose(waypoints[2][3], math.pi / 2.0)
    assert math.isclose(waypoints[-1][3], -math.pi / 2.0)


def test_csv_accepts_utf8_bom_trimmed_header_and_trailing_blank_lines(
    tmp_path: Path,
) -> None:
    """常见 UTF-8 BOM 和表头空格不会破坏严格的五列语义。"""
    source = tmp_path / "trimmed.CSV"
    source.write_text(
        "\ufeff INDEX , X , y ,z, yaw \n"
        "1, 1.25, -2.5, 3.75, 180\n\n",
        encoding="utf-8",
    )

    assert load_waypoint_file(source) == (
        (1.25, -2.5, 3.75, math.pi),
    )


@pytest.mark.parametrize(
    ("body", "message"),
    (
        ("x,index,y,z,yaw\n1,0,0,1,0\n", "表头"),
        ("index,x,y,z,yaw\n2,0,0,1,0\n", "index 应为 1"),
        ("index,x,y,z,yaw\n1,nan,0,1,0\n", "有限数字"),
        ("index,x,y,z,yaw\n1,0,0,1,181\n", "yaw 超出允许范围"),
        ("index,x,y,z,yaw\n1,0,0,1\n", "应有 5 列"),
        ("index,x,y,z,yaw\n", "没有可导入"),
    ),
)
def test_csv_rejects_ambiguous_or_unsafe_rows(
    tmp_path: Path, body: str, message: str
) -> None:
    """格式或数值不合法时整体失败，禁止返回部分航点。"""
    source = tmp_path / "invalid.csv"
    source.write_text(body, encoding="utf-8")

    with pytest.raises(WaypointImportError, match=message):
        load_waypoint_file(source)


def test_csv_import_limit_matches_onboard_executor_limit(tmp_path: Path) -> None:
    """256 个航点可导入，第 257 个航点会使整个文件失败。"""
    valid = tmp_path / "maximum.csv"
    rows = ["index,x,y,z,yaw"]
    rows.extend(f"{index},{index},0,1,0" for index in range(1, MAX_WAYPOINT_COUNT + 1))
    valid.write_text("\n".join(rows) + "\n", encoding="utf-8")
    assert len(load_waypoint_file(valid)) == MAX_WAYPOINT_COUNT

    too_many = tmp_path / "too-many.csv"
    rows.append(f"{MAX_WAYPOINT_COUNT + 1},0,0,1,0")
    too_many.write_text("\n".join(rows) + "\n", encoding="utf-8")
    with pytest.raises(WaypointImportError, match="单次最多导入 256"):
        load_waypoint_file(too_many)


def test_file_format_dispatcher_exposes_csv_and_reserves_other_formats(
    tmp_path: Path,
) -> None:
    """选择器由格式表生成，尚未注册的 Excel 扩展名被明确拒绝。"""
    assert "*.csv" in waypoint_file_dialog_filter()
    assert "*.xlsx" not in waypoint_file_dialog_filter()

    with pytest.raises(WaypointImportError, match="当前支持：.csv"):
        load_waypoint_file(tmp_path / "future.xlsx")
