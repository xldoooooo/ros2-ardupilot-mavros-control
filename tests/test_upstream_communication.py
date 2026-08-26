"""上位机映射、状态投影和独立 WebSocket 会话的回归测试。"""

from __future__ import annotations

import asyncio
import json
import math
import time
from dataclasses import replace

import pytest

from ground_station_core.event_log import EventLog
from ground_station_core.models import (
    CommandResult,
    FlightMode,
    VideoCaptureEvent,
    VideoServiceSnapshot,
    VehicleSnapshot,
)
from ground_station_core.upstream.mapping import (
    UpstreamProtocolError,
    parse_command,
)
from ground_station_core.upstream.models import UpstreamStandbyPolicy
from ground_station_core.upstream.protocol import command_topic, status_topic
from ground_station_core.upstream.service import UpstreamCommunicationService
from ground_station_core.upstream.status_projector import (
    DEFAULT_JPG_PATH,
    DEFAULT_VIDEO_PATH,
    UpstreamStatusProjector,
)


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
    assert command.photo_nos == ("3", "4")
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


def test_command_mapping_preserves_duplicate_negative_photo_numbers() -> None:
    """photoNo 只作为文件标识传递，不校正重复、负数或先后关系。"""
    payload = _task_payload()
    payload["taskPoints"][0]["photoNo"] = -7  # type: ignore[index]
    payload["taskPoints"][1]["photoNo"] = -7  # type: ignore[index]

    command = parse_command(payload, "UAV01001")

    assert command.photo_nos == ("-7", "-7")


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
    """03/09/08 与已完成格数一致，遥测 1 Hz 且 0C 只上报一次。"""
    sent: list[dict[str, object]] = []
    events = EventLog()
    projector = UpstreamStatusProjector(
        lambda: "UAV01001", lambda payload: not sent.append(dict(payload)), events
    )
    snapshot = _telemetry_snapshot()

    assert projector.observe_vehicle(snapshot, "simulation", now=10.0)
    projector.observe_vehicle(snapshot, "simulation", now=10.5)
    statuses = [payload["uavStatus"] for payload in sent]
    assert statuses == ["0A", "0B", "0C"]
    assert sent[0]["data"] == {"uavPower": 19.0}
    assert sent[1]["data"] == {"X": 1.234, "Y": -2.346, "Z": 3.457}

    assert projector.observe_vehicle(snapshot, "simulation", now=11.1)
    assert [payload["uavStatus"] for payload in sent].count("0C") == 1
    assert [payload["uavStatus"] for payload in sent].count("0A") == 2
    assert [payload["uavStatus"] for payload in sent].count("0B") == 2

    projector.begin_mission(7, "inspection", (10, 20))
    flying_to_first = replace(
        snapshot,
        active_mode=FlightMode.WAYPOINT,
        active_command_sequence=7,
        waypoint_index=1,
        waypoint_count=2,
    )
    projector.observe_vehicle(flying_to_first, "simulation", now=11.2)
    assert [payload["uavStatus"] for payload in sent][-1] == "03"
    assert "09" not in [payload["uavStatus"] for payload in sent]

    flying_to_second = replace(flying_to_first, waypoint_index=2)
    projector.observe_vehicle(flying_to_second, "simulation", now=11.3)
    assert [payload["uavStatus"] for payload in sent][-1] == "03"
    projector.observe_video_capture(
        VideoCaptureEvent(
            1,
            "UAV01001",
            101,
            True,
            2,
            7,
            1,
            "3",
            "/home/share/jpg/point-3.jpg",
            "完成",
        )
    )
    first_point = sent[-1]
    assert first_point["uavStatus"] == "09"
    assert first_point["data"] == {
        "pointNo": "10",
        "pointName": "巡检点位 10",
        "pointPic": "/home/share/jpg/point-3.jpg",
    }

    projector.observe_result(
        CommandResult(1, 7, "waypoints", True, "航点任务完成", True)
    )
    projector.observe_video_capture(
        VideoCaptureEvent(
            2,
            "UAV01001",
            102,
            True,
            2,
            7,
            2,
            "4",
            "/home/share/jpg/point-4.jpg",
            "完成",
        )
    )
    projector.begin_landing(8)
    projector.observe_result(CommandResult(2, 8, "land", True, "降落完成", True))
    assert [payload["uavStatus"] for payload in sent][-1] == "07"
    projector.observe_video_status(
        VideoServiceSnapshot(
            service_available=True,
            interface_version="3.2",
            running=False,
            state="stopped",
            video_directory="/home/share",
            image_directory="/home/share/jpg",
            last_video_path="/home/share/recording.mp4",
            last_image_path="/home/share/jpg/point-4.jpg",
            age_seconds=0.0,
        )
    )
    mission_statuses = [payload["uavStatus"] for payload in sent]
    assert mission_statuses.count("09") == 2
    assert mission_statuses[-2:] == ["07", "08"]
    assert sent[-1]["data"] == {
        "videoPath": "/home/share/recording.mp4",
        "JPGPath": "/home/share/jpg",
    }
    assert "01" not in [payload["uavStatus"] for payload in sent]


def test_status_projector_standby_hangar_blocking_and_return_completion_rules() -> None:
    """01 只在可起飞、入库且未锁定时发一次，返航绝不发 08。"""
    sent: list[dict[str, object]] = []
    policy = UpstreamStandbyPolicy(
        x_tolerance_meters=1.0,
        y_tolerance_meters=1.0,
        z_tolerance_meters=0.5,
        inspection_delay_seconds=60.0,
    )
    projector = UpstreamStatusProjector(
        lambda: "UAV01001",
        lambda payload: not sent.append(dict(payload)),
        EventLog(),
        policy,
    )
    ready = VehicleSnapshot(
        local_position_valid=True,
        x=0.99,
        y=-0.99,
        z=0.49,
    )

    projector.observe_vehicle(ready, "simulation", now=0.0, can_takeoff=True)
    projector.observe_vehicle(ready, "simulation", now=0.5, can_takeoff=True)
    assert [payload["uavStatus"] for payload in sent] == ["01"]

    # 武装后产生新待机边沿，但任务锁定期仍不可发送。
    projector.observe_vehicle(
        replace(ready, armed=True), "simulation", now=0.6, can_takeoff=False
    )
    projector.block_standby()
    projector.observe_vehicle(ready, "simulation", now=0.7, can_takeoff=True)
    assert [payload["uavStatus"] for payload in sent].count("01") == 1
    projector.release_standby()
    projector.observe_vehicle(ready, "simulation", now=0.8, can_takeoff=True)
    assert [payload["uavStatus"] for payload in sent].count("01") == 2

    outside = replace(ready, x=1.0)
    projector.observe_vehicle(
        replace(ready, armed=True), "simulation", now=0.9, can_takeoff=False
    )
    projector.observe_vehicle(outside, "simulation", now=1.0, can_takeoff=True)
    assert [payload["uavStatus"] for payload in sent].count("01") == 2

    projector.begin_mission(12, "return", (1,))
    returning = replace(
        ready,
        armed=True,
        active_mode=FlightMode.WAYPOINT,
        active_command_sequence=12,
        waypoint_index=1,
        waypoint_count=1,
    )
    projector.observe_vehicle(returning, "simulation", now=1.1)
    projector.observe_result(
        CommandResult(1, 12, "waypoints", True, "返航点完成", True)
    )
    statuses = [payload["uavStatus"] for payload in sent]
    assert statuses.count("05") == 1
    assert "08" not in statuses
    assert "09" not in statuses


def test_status_projector_never_reports_unfinalized_recording_as_video_path() -> None:
    """媒体等待超时时，运行中的预留文件名不能冒充已封装录像。"""
    sent: list[dict[str, object]] = []
    projector = UpstreamStatusProjector(
        lambda: "UAV01001",
        lambda payload: not sent.append(dict(payload)),
        EventLog(),
    )
    projector.MEDIA_RESULT_WAIT_SECONDS = 0.0
    projector.begin_mission(21, "inspection", (5,))
    projector.observe_result(
        CommandResult(1, 21, "waypoints", True, "航点任务完成", True)
    )
    projector.observe_video_capture(
        VideoCaptureEvent(
            1,
            "UAV01001",
            1,
            True,
            2,
            21,
            1,
            "raw",
            "/home/share/jpg/point.jpg",
            "完成",
        )
    )
    projector.observe_video_status(
        VideoServiceSnapshot(
            service_available=True,
            interface_version="3.2",
            running=True,
            state="running",
            image_directory="/home/share/jpg",
            current_video_path="/home/share/not-finalized.mp4",
        )
    )
    projector.begin_landing(22)
    projector.observe_result(
        CommandResult(2, 22, "land", True, "降落完成", True)
    )

    assert sent[-1]["uavStatus"] == "08"
    assert sent[-1]["data"] == {
        "videoPath": DEFAULT_VIDEO_PATH,
        "JPGPath": "/home/share/jpg",
    }


def test_status_projector_uses_default_media_paths_when_video_is_unavailable() -> None:
    """视频服务无可用路径时，08 使用协议约定的非空占位路径。"""
    sent: list[dict[str, object]] = []
    projector = UpstreamStatusProjector(
        lambda: "UAV01001",
        lambda payload: not sent.append(dict(payload)),
        EventLog(),
    )
    projector.MEDIA_RESULT_WAIT_SECONDS = 0.0
    projector.begin_mission(31, "inspection", ())
    projector.observe_result(
        CommandResult(1, 31, "waypoints", True, "航点任务完成", True)
    )
    projector.begin_landing(32)
    projector.observe_result(
        CommandResult(2, 32, "land", True, "降落完成", True)
    )

    assert sent[-1]["uavStatus"] == "08"
    assert sent[-1]["data"] == {
        "videoPath": DEFAULT_VIDEO_PATH,
        "JPGPath": DEFAULT_JPG_PATH,
    }


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


def test_connection_configuration_persists_between_service_instances(
    tmp_path, monkeypatch
) -> None:
    """URL 与无人机编号保存后成为下次面板初始值。"""
    config_path = tmp_path / "upstream.json"
    monkeypatch.delenv("UPSTREAM_WS_URL", raising=False)
    monkeypatch.delenv("UPSTREAM_CLIENT_NO", raising=False)
    first = UpstreamCommunicationService(
        event_log=EventLog(),
        on_command=lambda _command: None,
        auto_connect=False,
        config_path=config_path,
    )
    first.save_configuration("wss://example.test:9443/ws", "UAV02002")

    restored = UpstreamCommunicationService(
        event_log=EventLog(),
        on_command=lambda _command: None,
        auto_connect=False,
        config_path=config_path,
    ).snapshot()
    assert restored.url == "wss://example.test:9443/ws"
    assert restored.client_no == "UAV02002"


def test_disconnected_service_never_requests_low_power_return(tmp_path) -> None:
    """状态投影可继续计算，但 WebSocket 断线时服务不得开放动作触发。"""
    service = UpstreamCommunicationService(
        event_log=EventLog(),
        on_command=lambda _command: None,
        auto_connect=False,
        config_path=tmp_path / "upstream-offline.json",
    )

    assert service.snapshot().connected is False
    assert not service.observe_vehicle(_telemetry_snapshot(), "simulation")


def test_handshake_timeout_backoff_is_bounded_and_resets_after_success(
    tmp_path,
) -> None:
    """连续失败采用15/20/25/30秒，成功握手后的下一次重连恢复15秒。"""
    observed: list[float] = []
    service = UpstreamCommunicationService(
        event_log=EventLog(),
        on_command=lambda _command: None,
        url="ws://127.0.0.1:1/ws",
        auto_connect=True,
        config_path=tmp_path / "upstream-backoff.json",
    )
    service._RECONNECT_SECONDS = 0.01

    async def fail_session(
        _url: str,
        _client_no: str,
        timeout_seconds: float,
        handshake_completed: asyncio.Event,
    ) -> None:
        observed.append(timeout_seconds)
        if len(observed) == 5:
            handshake_completed.set()
        raise TimeoutError("测试握手失败")

    service._run_session = fail_session  # type: ignore[method-assign]
    service.start()
    deadline = time.monotonic() + 2.0
    try:
        while len(observed) < 6 and time.monotonic() < deadline:
            time.sleep(0.01)
    finally:
        service.stop(3.0)

    assert observed[:6] == [15.0, 20.0, 25.0, 30.0, 30.0, 15.0]


def test_websocket_service_uses_topic_envelopes_and_separate_raw_journal(
    tmp_path,
) -> None:
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
            config_path=tmp_path / "upstream-websocket.json",
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
