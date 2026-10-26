"""验证串行巡检只上报一次带真实FTP路径的旧协议08/09。"""

from __future__ import annotations

import io
import json
import threading

from ground_station_core.event_log import EventLog
from ground_station_core.ground_media_reader import GroundMediaReader, processed_download
from ground_station_core.models import CommandResult, VideoCaptureEvent, VideoServiceSnapshot
from ground_station_core.upstream.status_projector import UpstreamStatusProjector


def _record(kind: str = "image", **changes) -> dict:
    """构造含内部任务身份和FTP公开路径的已处理媒体。"""
    return {
        "id": kind, "aircraft_id": "refresh", "session_id": "session",
        "mission_sequence": 8, "waypoint_index": 1, "photo_no": "-3",
        "kind": kind, "variant": "rebuilt" if kind == "video" else "experiment",
        "state": "ready", "ftp_path": f"/refresh/session/{kind}",
        "download_url": f"http://ground/api/file?id={kind}",
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


def test_finished_flight_waits_for_media_and_reports_completion_only_once() -> None:
    """落地不发送空08，成品可下载后一次交付，重建和轮询不补发。"""
    sent = []
    projector = _projector(sent)
    projector.observe_video_capture(VideoCaptureEvent(1, "gui-one", 42, True, 2, 8, 1, "-3", "/aircraft/raw.jpg", "已保存"))
    _complete(projector)
    assert [item["uavStatus"] for item in sent] == ["07"]
    projector.reset_runtime()
    projector.observe_ground_media((_record(), _record("video")))
    assert sent[-2:] == [
        {"clientNo": "UAV01001", "uavStatus": "09", "data": {
            "pointNo": "10", "pointName": "巡检点位 10", "pointPic": "/refresh/session/image",
        }},
        {"clientNo": "UAV01001", "uavStatus": "08", "data": {
            "videoPath": "/refresh/session/video", "JPGPath": "/refresh/session/",
        }},
    ]
    count = len(sent)
    projector.observe_ground_media((_record(), _record("video")))
    projector.observe_ground_media((_record(), _record("video", ftp_path="/refresh/session/new-rebuild.mp4")))
    assert len(sent) == count
    assert not projector._completed_missions


def test_media_identity_rejects_other_aircraft_source_task_and_photo() -> None:
    """相同任务序号或photoNo不得冒充本轮，HTTP路径也不能交给FTP客户端。"""
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
        _record("video", metadata={"source_id": "old-gui"}),
    ))
    assert [item["uavStatus"] for item in sent] == ["07"]
    projector.observe_ground_media((_record(),))
    assert sent[-1]["uavStatus"] == "09"
    assert not any(item["uavStatus"] == "08" for item in sent)
    projector.observe_ground_media((_record(), _record("video")))
    assert sent[-1]["uavStatus"] == "08"


def test_live_fragments_cannot_replace_the_complete_task_video() -> None:
    """实时片段只供内部查看，08等待完整原片生成的单个成品。"""
    sent = []
    projector = _projector(sent)
    _complete(projector)
    fragment = _record("video", variant="experiment", state="incomplete", error="流中断")
    projector.observe_ground_media((_record(), fragment))
    assert not any(item["uavStatus"] == "08" for item in sent)
    rebuilt = _record("video", ftp_path="/refresh/session/full.mp4")
    projector.observe_ground_media((_record(), fragment, rebuilt))
    assert sent[-1]["data"] == {"videoPath": "/refresh/session/full.mp4", "JPGPath": "/refresh/session/"}


def test_processing_and_local_paths_are_never_downloads() -> None:
    """暂存、原片和HTTP URL都不能当作FTP虚拟根文件路径。"""
    for change in (
        {"state": "processing"}, {"state": "error"}, {"variant": "original"},
        {"ftp_path": ""}, {"ftp_path": "http://ground/api/file?id=image"},
        {"ftp_path": "relative/image.jpg"}, {"ftp_path": "/../internal/original.jpg"},
    ):
        assert processed_download(_record(**change)) == ""
    assert processed_download(_record()) == "/refresh/session/image"


def test_failed_capture_logs_locally_without_unsupported_failure_payload() -> None:
    """照片失败只记本地日志，不发空09；有效视频仍可一次交付。"""
    sent = []
    projector = _projector(sent)
    error = VideoServiceSnapshot(last_error="视频节点离线")
    projector.observe_video_status(error)
    projector.observe_video_status(error)
    assert sum("视频节点离线" in event.message for event in projector._events.snapshot()) == 1
    projector.observe_video_capture(VideoCaptureEvent(1, "gui-one", 42, False, 2, 8, 1, "-3", "", "相机错误"))
    _complete(projector)
    assert [item["uavStatus"] for item in sent] == ["07"]
    projector.observe_ground_media((_record("video"),))
    assert [item["uavStatus"] for item in sent] == ["07", "08"]
    assert any("相机错误" in event.message for event in projector._events.snapshot())
    assert set(sent[-1]["data"]) == {"videoPath", "JPGPath"}


def test_processing_failures_are_local_and_retry_does_not_create_empty_completion() -> None:
    """处理失败留痕，重试继续等真实文件，绝不发送带空路径的08/09。"""
    for state in ("processing_error", "deleted", "missing", "retry"):
        sent = []
        projector = _projector(sent)
        _complete(projector)
        message = f"ground media {state}"
        projector.observe_ground_media((
            _record(state=state, error=message, ftp_path=""), _record("video"),
        ))
        if state == "retry":
            assert [item["uavStatus"] for item in sent] == ["07"]
        else:
            assert [item["uavStatus"] for item in sent] == ["07", "08"]
            assert any(message in event.message for event in projector._events.snapshot())
        projector.observe_ground_media((_record(), _record("video")))
        assert [item["uavStatus"] for item in sent].count("08") == 1


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
