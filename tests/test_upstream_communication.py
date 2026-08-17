"""上位机映射、状态投影和独立 WebSocket 会话的回归测试。"""

from __future__ import annotations

import asyncio
import json
import math
import time
from dataclasses import replace

import pytest

from ground_station_core.event_log import EventLog
from ground_station_core.models import CommandResult, FlightMode, VehicleSnapshot
from ground_station_core.upstream.mapping import (
    UpstreamProtocolError,
    parse_command,
)
from ground_station_core.upstream.protocol import command_topic, status_topic
from ground_station_core.upstream.service import UpstreamCommunicationService
from ground_station_core.upstream.status_projector import UpstreamStatusProjector


def _task_payload(client_no: str = "UAV01001") -> dict[str, object]:
    """返回覆盖角度转换与暂忽略相机字段的两点任务。"""
    return {
        "clientNo": client_no,
        "commandNo": "02",
        "taskPoints": [
            {
                "index": 10,
                "x": 1.0,
                "y": -2.0,
                "z": 1.2,
                "forwardAngle": 270.0,
                "cameraAngle": 1495,
                "photoNo": 3,
            },
            {
                "index": 20,
                "x": 3.0,
                "y": 4.0,
                "z": 2.5,
                "forwardAngle": 45.0,
                "cameraAngle": 1200,
                "photoNo": 4,
            },
        ],
    }


def _telemetry_snapshot() -> VehicleSnapshot:
    """返回已连接且遥测有效的仿真权威快照。"""
    return VehicleSnapshot(
        onboard_available=True,
        connected=True,
        armed=True,
        x=1.2345,
        y=-2.3456,
        z=3.4567,
        battery_valid=True,
        battery_voltage=21.9,
        battery_percentage=0.19,
        local_position_valid=True,
        active_mode=FlightMode.HOVER,
    )


def test_command_mapping_converts_yaw_and_covers_added_interfaces() -> None:
    """02 保留点号并做角度到弧度转换，01/06 使用同一业务格式。"""
    command = parse_command(_task_payload(), "UAV01001")
    assert command.command_no == "02"
    assert command.point_indexes == (10, 20)
    assert command.ignored_camera_fields
    assert command.waypoints[0][:3] == (1.0, -2.0, 1.2)
    assert math.isclose(command.waypoints[0][3], -math.pi / 2.0)
    assert math.isclose(command.waypoints[1][3], math.pi / 4.0)

    assert (
        parse_command(
            {"clientNo": "UAV01001", "commandNo": "01"}, "UAV01001"
        ).action.value
        == "takeoff"
    )
    assert (
        parse_command(
            {"clientNo": "UAV01001", "commandNo": "06"}, "UAV01001"
        ).action.value
        == "land"
    )
    assert command_topic("UAV01001") == "drone/UAV01001/command"
    assert status_topic("UAV01001") == "drone/UAV01001/status"


@pytest.mark.parametrize(
    ("mutation", "message"),
    (
        (lambda value: value.update(clientNo="UAV99999"), "编号不一致"),
        (
            lambda value: value["taskPoints"][0].update(z=0.05),  # type: ignore[index]
            "[0.1, 50]",
        ),
        (
            lambda value: value["taskPoints"][0].update(x=float("nan")),  # type: ignore[index]
            "有限数值",
        ),
        (
            lambda value: value["taskPoints"][1].update(index=10),  # type: ignore[index]
            "重复",
        ),
    ),
)
def test_command_mapping_rejects_unsafe_or_ambiguous_values(
    mutation, message: str
) -> None:
    """协议边界错误在进入 Qt/ROS 前被完整拒绝。"""
    payload = _task_payload()
    mutation(payload)
    with pytest.raises(UpstreamProtocolError, match=message):
        parse_command(payload, "UAV01001")


def test_status_projector_matches_gui_progress_and_telemetry_rules() -> None:
    """03/09/08 与已完成格数一致，遥测 1 Hz 且 0C 只发一次边沿。"""
    sent: list[dict[str, object]] = []
    events = EventLog()
    projector = UpstreamStatusProjector(
        lambda: "UAV01001", lambda payload: not sent.append(dict(payload)), events
    )
    snapshot = _telemetry_snapshot()

    projector.observe_vehicle(snapshot, "simulation", now=10.0)
    projector.observe_vehicle(snapshot, "simulation", now=10.5)
    statuses = [payload["uavStatus"] for payload in sent]
    assert statuses == ["0A", "0B", "0C"]
    assert sent[0]["data"] == {"uavPower": 19.0}
    assert sent[1]["data"] == {"X": 1.234, "Y": -2.346, "Z": 3.457}

    projector.observe_vehicle(snapshot, "simulation", now=11.1)
    assert [payload["uavStatus"] for payload in sent].count("0C") == 1
    assert [payload["uavStatus"] for payload in sent].count("0A") == 2
    assert [payload["uavStatus"] for payload in sent].count("0B") == 2

    projector.begin_mission(7, "inspection", (10, 20))
    flying_to_first = replace(
        snapshot,
        active_mode=FlightMode.WAYPOINT,
        waypoint_index=1,
        waypoint_count=2,
    )
    projector.observe_vehicle(flying_to_first, "simulation", now=11.2)
    assert [payload["uavStatus"] for payload in sent][-1] == "03"
    assert "09" not in [payload["uavStatus"] for payload in sent]

    flying_to_second = replace(flying_to_first, waypoint_index=2)
    projector.observe_vehicle(flying_to_second, "simulation", now=11.3)
    first_point = sent[-1]
    assert first_point["uavStatus"] == "09"
    assert first_point["data"] == {
        "pointNo": "10",
        "pointName": "巡检点位 10",
        "pointPic": "",
    }

    projector.observe_result(
        CommandResult(1, 7, "waypoints", True, "航点任务完成", True)
    )
    tail = [payload["uavStatus"] for payload in sent[-2:]]
    assert tail == ["09", "08"]
    assert sent[-1]["data"] == {"videoPath": "", "JPGPath": ""}
    assert "01" not in [payload["uavStatus"] for payload in sent]


def test_status_projector_uses_voltage_for_hardware_and_reports_landing_once() -> None:
    """实机 0A 使用电压；06/07 共用一次进入 LAND 的 07 状态。"""
    sent: list[dict[str, object]] = []
    projector = UpstreamStatusProjector(
        lambda: "UAV01001",
        lambda payload: not sent.append(dict(payload)),
        EventLog(),
    )
    snapshot = replace(_telemetry_snapshot(), battery_voltage=22.1)
    projector.observe_vehicle(snapshot, "hardware", now=5.0)
    assert sent[0] == {
        "clientNo": "UAV01001",
        "uavStatus": "0A",
        "data": {"uavPower": 22.1},
    }
    assert any(payload["uavStatus"] == "0C" for payload in sent)

    projector.begin_landing(9)
    landing = replace(snapshot, active_mode=FlightMode.LAND)
    projector.observe_vehicle(landing, "hardware", now=5.1)
    projector.observe_vehicle(landing, "hardware", now=5.2)
    assert [payload["uavStatus"] for payload in sent].count("07") == 1


def test_websocket_service_uses_topic_envelopes_and_separate_raw_journal() -> None:
    """真实 WebSocket 握手、订阅、BROADCAST、确认和独立重连均可工作。"""
    import websockets

    async def scenario() -> tuple[
        list[dict[str, object]],
        list[object],
        EventLog,
        tuple[object, ...],
        int,
    ]:
        acknowledgements: list[dict[str, object]] = []
        commands: list[object] = []
        connection_count = 0

        async def handler(websocket) -> None:
            nonlocal connection_count
            connection_count += 1
            await websocket.send(json.dumps({"type": "SYSTEM", "session": "test"}))
            subscription = json.loads(await websocket.recv())
            assert subscription == {
                "type": "SUBSCRIBE",
                "topic": "drone/UAV01001/command",
            }
            await websocket.send(
                json.dumps(
                    {
                        "type": "SUB_ACK",
                        "topic": "drone/UAV01001/command",
                    }
                )
            )
            await websocket.send(
                json.dumps(
                    {
                        "type": "BROADCAST",
                        "topic": "drone/UAV01001/command",
                        "data": {"clientNo": "UAV01001", "commandNo": "06"},
                    }
                )
            )
            raw_ack = await asyncio.wait_for(websocket.recv(), timeout=3.0)
            acknowledgements.append(json.loads(raw_ack))
            with contextlib.suppress(Exception):
                await websocket.wait_closed()

        import contextlib

        server = await websockets.serve(handler, "127.0.0.1", 0)
        port = server.sockets[0].getsockname()[1]
        events = EventLog()
        service = UpstreamCommunicationService(
            event_log=events,
            on_command=commands.append,
            url=f"ws://127.0.0.1:{port}/ws",
            client_no="UAV01001",
            auto_connect=True,
        )
        service.start()

        async def wait_until(predicate, timeout: float = 5.0) -> None:
            deadline = time.monotonic() + timeout
            while not predicate():
                if time.monotonic() >= deadline:
                    raise AssertionError("等待 WebSocket 条件超时")
                await asyncio.sleep(0.02)

        try:
            await wait_until(lambda: len(commands) >= 1)
            await wait_until(lambda: len(acknowledgements) >= 1)
            assert service.snapshot().connected
            service.disconnect()
            await wait_until(lambda: not service.snapshot().connected)
            service.connect(f"ws://127.0.0.1:{port}/ws", "UAV01001")
            await wait_until(lambda: connection_count >= 2)
            await wait_until(lambda: len(acknowledgements) >= 2)
            frames = service.journal.snapshot()
        finally:
            await asyncio.to_thread(service.stop, 3.0)
            server.close()
            await server.wait_closed()
        return acknowledgements, commands, events, frames, connection_count

    acknowledgements, commands, events, frames, connection_count = asyncio.run(
        scenario()
    )
    assert connection_count >= 2
    assert len(commands) >= 2
    assert acknowledgements[0] == {
        "type": "PUBLISH",
        "topic": "drone/UAV01001/status",
        "data": {"clientNo": "UAV01001", "commandNo": "06"},
    }
    assert {frame.direction for frame in frames} == {"RX", "TX"}
    assert any('"type":"PUBLISH"' in frame.payload for frame in frames)
    # 人类维护日志保留语义摘要，不复制原始传输信封。
    human_messages = "\n".join(event.message for event in events.snapshot())
    assert "收到上位机命令 06" in human_messages
    assert '"type":"PUBLISH"' not in human_messages
