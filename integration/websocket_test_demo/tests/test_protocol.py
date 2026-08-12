"""协议 V2.0 所有命令、状态、主题和必填字段的单元测试。"""

from __future__ import annotations

from copy import deepcopy

import pytest

from ws_demo.protocol import (
    COMMAND_LABELS,
    STATUS_LABELS,
    ProtocolError,
    command_ack,
    command_topic,
    sample_commands,
    sample_statuses,
    status_topic,
    validate_command,
    validate_status,
)


def test_topics_use_exact_client_number() -> None:
    """状态和命令主题严格使用协议路径。"""

    assert command_topic("UAV01001") == "drone/UAV01001/command"
    assert status_topic("UAV01001") == "drone/UAV01001/status"


@pytest.mark.parametrize("command", sample_commands())
def test_all_documented_commands_are_valid(command: dict[str, object]) -> None:
    """表 1 的 02、03、05、07 全部可解析并生成原样确认。"""

    checked = validate_command(command, "UAV01001")
    assert checked["commandNo"] in COMMAND_LABELS
    assert command_ack(checked) == {
        "clientNo": "UAV01001",
        "commandNo": checked["commandNo"],
    }


@pytest.mark.parametrize("status", sample_statuses())
def test_all_documented_statuses_are_valid(status: dict[str, object]) -> None:
    """表 2 十种状态全部可解析，包含 DOCX 独有的 0C。"""

    checked = validate_status(status, "UAV01001")
    assert checked["uavStatus"] in STATUS_LABELS


def test_status_catalog_contains_docx_only_0c() -> None:
    """防止后续按不完整 TXT 误删低电量 0C。"""

    assert STATUS_LABELS["0C"] == "巡检电量不足，暂停巡检，返航充电"


def test_route_command_requires_all_waypoint_fields() -> None:
    """航线点缺少文档字段时必须显式报错。"""

    command = deepcopy(sample_commands()[0])
    del command["taskPoints"][0]["cameraAngle"]  # type: ignore[index]
    with pytest.raises(ProtocolError, match="cameraAngle"):
        validate_command(command)


def test_non_route_command_does_not_invent_task_points_requirement() -> None:
    """03/05/07 不擅自增加 taskPoints。"""

    assert validate_command({"clientNo": "UAV01001", "commandNo": "03"})


@pytest.mark.parametrize(
    ("status_no", "data", "missing"),
    [
        ("08", {"videoPath": "x"}, "JPGPath"),
        ("09", {"pointNo": "1", "pointName": "x"}, "pointPic"),
        ("0A", {}, "uavPower"),
        ("0B", {"X": 1, "Y": 2}, "Z"),
    ],
)
def test_data_statuses_require_documented_fields(
    status_no: str, data: dict[str, object], missing: str
) -> None:
    """08/09/0A/0B 的 data 缺字段时给出可定位错误。"""

    with pytest.raises(ProtocolError, match=missing):
        validate_status(
            {"clientNo": "UAV01001", "uavStatus": status_no, "data": data}
        )


def test_client_number_isolation_is_validated() -> None:
    """单地面站只接受自身 clientNo 的命令。"""

    with pytest.raises(ProtocolError, match="不一致"):
        validate_command(sample_commands("UAV01002")[1], "UAV01001")

