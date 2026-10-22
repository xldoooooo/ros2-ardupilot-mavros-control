"""验证甲方媒体只发布地面处理版，飞行完成不等待上传。"""

from __future__ import annotations

import io
import json
import threading

from ground_station_core.event_log import EventLog
from ground_station_core.ground_media_reader import GroundMediaReader, processed_download
from ground_station_core.models import CommandResult, VideoCaptureEvent
from ground_station_core.upstream.status_projector import UpstreamStatusProjector


def _record(kind: str = "image", **changes) -> dict:
    """构造含客户端、任务与抓拍身份的地面处理版目录记录。"""
    return {
        "id": kind, "aircraft_id": "refresh", "session_id": "session",
        "mission_sequence": 8, "waypoint_index": 1, "photo_no": "-3",
        "kind": kind, "variant": "experiment", "state": "ready",
        "download_url": f"ftp://ground/refresh/session/{kind}",
        "download_directory": "ftp://ground/refresh/session/",
        "metadata": {"source_id": "gui-one", "sequence": 42},
        **changes,
    }


def _projector(sent: list) -> UpstreamStatusProjector:
    """建立真实投影器并绑定一次带身份的巡检任务。"""
    projector = UpstreamStatusProjector(
        lambda: "UAV01001", lambda payload: not sent.append(dict(payload)), EventLog()
    )
    projector.begin_mission(8, "inspection", (10,), source_id="gui-one", aircraft_id="refresh")
    return projector


def _complete(projector: UpstreamStatusProjector) -> None:
    """交付可靠飞行与降落终态。"""
    projector.observe_result(CommandResult(1, 8, "waypoints", True, "完成"))
    projector.begin_landing(9)
    projector.observe_result(CommandResult(2, 9, "land", True, "完成"))


def test_flight_completion_precedes_upload_and_late_media_survives_disconnect() -> None:
    """原片路径不外发；断开飞机后，地面入库仍可补发媒体结果。"""
    sent = []
    projector = _projector(sent)
    event = VideoCaptureEvent(1, "gui-one", 42, True, 2, 8, 1, "-3", "/aircraft/raw.jpg", "原片已保存")
    projector.observe_video_capture(event)
    _complete(projector)
    assert [item["uavStatus"] for item in sent] == ["07", "08"]
    assert sent[-1]["data"]["videoPath"] == sent[-1]["data"]["JPGPath"] == ""
    assert sent[-1]["data"]["mediaStatus"] == "processing"
    assert not sent[-1]["data"]["mediaUpdate"]
    projector.reset_runtime()
    projector.observe_ground_media((_record(), _record("video")))
    assert [item["uavStatus"] for item in sent][-2:] == ["09", "08"]
    assert sent[-1]["data"]["videoPath"] == "ftp://ground/refresh/session/video"
    assert sent[-1]["data"]["JPGPath"] == "ftp://ground/refresh/session/"
    assert sent[-1]["data"]["mediaStatus"] == "ready"
    assert sent[-1]["data"]["mediaUpdate"]
    count = len(sent)
    projector.observe_ground_media((_record(), _record("video")))
    assert len(sent) == count


def test_media_identity_rejects_other_aircraft_source_task_and_photo() -> None:
    """相同任务序号或相同 photoNo 不能覆盖实际客户端与航点身份。"""
    sent = []
    projector = _projector(sent)
    projector.observe_video_capture(VideoCaptureEvent(1, "gui-one", 42, True, 2, 8, 1, "-3", "", ""))
    _complete(projector)
    projector.observe_ground_media((
        _record(aircraft_id="new"),
        _record(metadata={"source_id": "old-gui", "sequence": 42}),
        _record(mission_sequence=7), _record(photo_no="4"),
        _record(waypoint_index=2),
        _record(metadata={"source_id": "gui-one", "sequence": 43}),
        _record(variant="original"),
    ))
    assert not any(item["uavStatus"] == "09" for item in sent)
    assert sent[-1]["data"]["JPGPath"] == ""
    projector.observe_ground_media((_record(),))
    assert any(item["uavStatus"] == "09" for item in sent)


def test_known_incomplete_record_is_downloadable_and_rebuilt_can_replace_it() -> None:
    """实验版已知中断保留说明，整段重建完成后可更新为 ready。"""
    sent = []
    projector = _projector(sent)
    projector.observe_video_capture(VideoCaptureEvent(1, "gui-one", 42, True, 2, 8, 1, "-3", "", ""))
    _complete(projector)
    image = _record()
    video = _record("video", state="incomplete", error="流中断")
    projector.observe_ground_media((image, video))
    assert sent[-1]["data"]["mediaStatus"] == "incomplete"
    assert sent[-1]["data"]["mediaError"] == "流中断"
    rebuilt = _record("video", variant="rebuilt", download_url="ftp://ground/rebuilt.mp4")
    projector.observe_ground_media((image, video, rebuilt))
    assert sent[-1]["data"]["videoPath"] == "ftp://ground/rebuilt.mp4"
    assert sent[-1]["data"]["mediaStatus"] == "ready"


def test_processing_and_local_paths_are_never_downloads() -> None:
    """暂存、处理失败、内部原片以及本地路径均不能充当下载地址。"""
    for change in (
        {"state": "processing"}, {"state": "error"}, {"variant": "original"},
        {"download_url": "/home/share/fake.jpg"}, {"download_url": ""},
    ):
        assert processed_download(_record(**change)) == ""
    assert processed_download(_record(state="incomplete"))


def test_failed_capture_reports_error_without_aircraft_path() -> None:
    """明确失败仍上报点位状态，媒体失败不改变飞行完成结果。"""
    sent = []
    projector = _projector(sent)
    projector.observe_video_capture(VideoCaptureEvent(1, "gui-one", 42, False, 2, 8, 1, "-3", "/aircraft/fake", "相机错误"))
    _complete(projector)
    point = next(item for item in sent if item["uavStatus"] == "09")
    assert point["data"]["pointPic"] == ""
    assert point["data"]["mediaStatus"] == "error"
    assert sent[-1]["uavStatus"] == "08"
    assert sent[-1]["data"]["mediaStatus"] == "error"


def test_processing_failures_retry_and_service_warning_preserve_truthful_media_state() -> None:
    """终态处理失败可上报点位，retry继续等待，容量告警不否定可下载文件。"""
    for state in ("processing_error", "deleted", "missing", "retry"):
        sent = []
        projector = _projector(sent)
        projector.observe_video_capture(VideoCaptureEvent(1, "gui-one", 42, True, 2, 8, 1, "-3", "", ""))
        _complete(projector)
        message = f"ground media {state}"
        projector.observe_ground_media((
            _record(state=state, error=message, download_url=""), _record("video"),
        ))
        points = [item for item in sent if item["uavStatus"] == "09"]
        if state == "retry":
            assert not points
            assert sent[-1]["data"]["mediaStatus"] == "retry"
        else:
            assert points[-1]["data"]["mediaStatus"] == state
            assert points[-1]["data"]["mediaError"] == message
            assert points[-1]["data"]["pointPic"] == ""
            assert sent[-1]["data"]["mediaStatus"] == "error"
        assert sent[-1]["data"]["images"][0]["state"] == state
        assert sent[-1]["data"]["mediaError"] == message
        projector.observe_ground_media((_record(), _record("video")), "storage capacity exceeded")
        assert sent[-1]["data"]["mediaStatus"] == "ready"
        assert sent[-1]["data"]["videoPath"]
        assert sent[-1]["data"]["mediaError"] == "storage capacity exceeded"


def test_reader_network_and_callback_run_outside_caller(tmp_path, monkeypatch) -> None:
    """慢网络不会阻塞启动；配置覆盖与目录回调均在后台执行。"""
    config = tmp_path / "media.json"
    config.write_text(json.dumps({"api_url": "http://media.test:8090"}))
    monkeypatch.setenv("GROUND_MEDIA_CONFIG", str(config))
    entered, release, completed = threading.Event(), threading.Event(), threading.Event()
    observed = []

    def open_media(url, timeout):
        """在模拟网络中停留，确认主线程已经从 start 返回。"""
        observed.append((url, threading.get_ident()))
        entered.set()
        release.wait(2.0)
        return io.StringIO(json.dumps({"records": [_record()], "error": "storage capacity exceeded"}))

    def on_snapshot(records, error):
        """记录后台目录快照与调用线程。"""
        observed.append((records, error, threading.get_ident()))
        completed.set()

    monkeypatch.setattr("ground_station_core.ground_media_reader.urlopen", open_media)
    reader = GroundMediaReader(on_snapshot, interval=10.0)
    main_thread = threading.get_ident()
    reader.start()
    try:
        assert entered.wait(1.0)
        assert not completed.is_set()
        release.set()
        assert completed.wait(1.0)
        assert observed[0] == ("http://media.test:8090/api/media", observed[0][1])
        assert observed[0][1] != main_thread
        assert observed[1][0][0]["metadata"]["source_id"] == "gui-one"
        assert observed[1][1] == "storage capacity exceeded"
        assert observed[1][2] != main_thread
    finally:
        release.set()
        assert reader.stop()
