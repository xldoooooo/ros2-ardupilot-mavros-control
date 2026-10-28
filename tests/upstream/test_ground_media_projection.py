"""验证实时录像优先交付、失败终结及晚到照片的08/09关联。"""

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


def test_playable_incomplete_live_recording_is_delivered_without_waiting_for_rebuild() -> None:
    """可播放实时录像即使缺段也交付，后备成品到达不补发08。"""
    sent = []
    projector = _projector(sent)
    _complete(projector)
    fragment = _record("video", variant="experiment", state="incomplete", error="流中断")
    projector.observe_ground_media((_record(), fragment))
    assert sent[-1]["data"] == {"videoPath": "/refresh/session/video", "JPGPath": "/refresh/session/"}
    assert any("不完整" in event.message for event in projector._events.snapshot())
    count = len(sent)
    rebuilt = _record("video", ftp_path="/refresh/session/full.mp4")
    projector.observe_ground_media((_record(), fragment, rebuilt))
    assert len(sent) == count


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


def test_photo_processing_or_retry_does_not_block_video_completion() -> None:
    """图片重试与失败不阻塞已就绪视频08，重试成功后仍可补报09。"""
    for state in ("processing_error", "deleted", "missing", "retry"):
        sent = []
        projector = _projector(sent)
        _complete(projector)
        message = f"ground media {state}"
        projector.observe_ground_media((
            _record(state=state, error=message, ftp_path=""), _record("video"),
        ))
        assert [item["uavStatus"] for item in sent] == ["07", "08"]
        if state != "retry":
            assert any(message in event.message for event in projector._events.snapshot())
        projector.observe_ground_media((_record(), _record("video")))
        assert [item["uavStatus"] for item in sent].count("08") == 1
        if state == "retry":
            assert sent[-1]["uavStatus"] == "09"


def test_live_recording_precedes_rebuild_and_broken_live_uses_rebuild() -> None:
    """同时存在时首选实时录像；实时文件不可交付时允许后备成品。"""
    for state, expected in (("ready", "live.mp4"), ("error", "full.mp4")):
        sent = []
        projector = _projector(sent)
        _complete(projector)
        projector.observe_ground_media((
            _record("video", variant="experiment", state=state,
                    ftp_path="/refresh/session/live.mp4"),
            _record("video", ftp_path="/refresh/session/full.mp4"),
        ))
        assert sent[-1]["data"]["videoPath"] == f"/refresh/session/{expected}"


def test_authoritative_delivery_waits_for_selected_merged_record() -> None:
    """整轮合并结束前不抢发ready片段，只交付媒体中心指定的成品。"""
    sent = []
    projector = _projector(sent)
    _complete(projector)
    fragment = _record("video", id="fragment", variant="experiment", ftp_path="/refresh/session/part.mkv")
    merged = _record("video", id="merged", variant="experiment", ftp_path="/refresh/session/merged.mkv")
    for state in ("waiting", "fallback"):
        projector.observe_ground_media((fragment, merged, _record("delivery", state=state)))
        assert not any(item["uavStatus"] == "08" for item in sent)
    projector.observe_ground_media((fragment, merged, _record("delivery", state="ready", record_id="merged")))
    assert sent[-1]["data"]["videoPath"] == "/refresh/session/merged.mkv"


def test_failed_delivery_sends_empty_completion_and_releases_next_mission() -> None:
    """明确失败发空08并释放后轮；晚到视频或图片不能复活失败轮。"""
    sent, failures = [], []
    projector = _projector(sent)
    projector._fail_delivery = failures.append
    _complete(projector)
    projector.begin_mission(10, "inspection", (), source_id="gui-one", aircraft_id="refresh")
    projector.observe_result(CommandResult(3, 10, "waypoints", True, "完成"))
    projector.begin_landing(11)
    projector.observe_result(CommandResult(4, 11, "land", True, "完成"))
    projector.observe_ground_media((
        _record("delivery", state="failed", error="存储权限需要人工处理"),
        _record("video", mission_sequence=10),
    ))
    completed = [item for item in sent if item["uavStatus"] == "08"]
    assert [item["data"]["videoPath"] for item in completed] == ["", "/refresh/session/video"]
    assert completed[0]["data"] == {"videoPath": "", "JPGPath": ""}
    assert failures == [{
        "aircraft_id": "refresh", "source_id": "gui-one", "mission_sequence": 8,
        "session_id": "session", "error": "存储权限需要人工处理",
    }]
    count = len(sent)
    projector.observe_ground_media((_record(), _record("video")))
    assert len(sent) == count
    assert not projector._completed_missions


def test_landing_timeout_without_media_freezes_empty_completion_while_offline(monkeypatch) -> None:
    """无媒体目录也从落地开始超时，离线仍通知停止且重连后只发一次空08。"""
    now = [100.0]
    monkeypatch.setattr("ground_station_core.upstream.status_projector.time.monotonic", lambda: now[0])
    sent, failures = [], []
    projector = _projector(sent)
    projector._fail_delivery = failures.append
    projector.observe_ground_media(({"kind": "delivery_policy", "delivery_timeout_seconds": 2},))
    projector.observe_result(CommandResult(1, 8, "waypoints", True, "完成"))
    now[0] = 1000.0
    projector.observe_ground_media(())
    assert not failures  # 飞行结束尚未落地，不能开始交付超时。
    projector.begin_landing(9)
    projector.observe_result(CommandResult(2, 9, "land", True, "完成"))
    projector._emit = lambda payload: False
    now[0] = 1001.9
    projector.observe_ground_media((), "地面站不可达")
    assert not failures
    now[0] = 1002.0
    projector.observe_ground_media((), "地面站不可达")
    assert len(failures) == 1
    assert failures[0]["source_id"] == "gui-one"
    assert failures[0]["mission_sequence"] == 8
    assert "session_id" not in failures[0]
    projector.observe_ground_media((_record(), _record("video")))
    assert len(failures) == 1
    projector._emit = lambda payload: not sent.append(dict(payload))
    projector.observe_ground_media((_record(), _record("video")))
    assert sent[-1]["data"] == {"videoPath": "", "JPGPath": ""}
    assert [item["uavStatus"] for item in sent].count("08") == 1
    assert not any(item["uavStatus"] == "09" for item in sent)


def test_invalid_timeout_uses_ten_minutes(monkeypatch) -> None:
    """非法非正超时配置恢复600秒，避免误终止任务。"""
    sent = []
    projector = _projector(sent)
    for value in (0, -1, "invalid", float("nan"), float("inf")):
        projector.observe_ground_media(({"kind": "delivery_policy", "delivery_timeout_seconds": value},))
        assert projector._delivery_timeout_seconds == 600.0
    now = [50.0]
    monkeypatch.setattr("ground_station_core.upstream.status_projector.time.monotonic", lambda: now[0])
    _complete(projector)
    now[0] = 649.9
    projector.observe_ground_media(())
    assert not any(item["uavStatus"] == "08" for item in sent)
    now[0] = 650.0
    projector.observe_ground_media(())
    assert sent[-1]["uavStatus"] == "08"
    assert sent[-1]["data"] == {"videoPath": "", "JPGPath": ""}


def test_late_photo_after_success_does_not_block_later_completion() -> None:
    """首轮照片晚到仍补09，第二轮08无需等待该照片。"""
    sent = []
    projector = _projector(sent)
    _complete(projector)
    projector.observe_ground_media((_record("video"),))
    assert sent[-1]["uavStatus"] == "08"
    projector.begin_mission(10, "inspection", (), source_id="gui-one", aircraft_id="refresh")
    projector.observe_result(CommandResult(3, 10, "waypoints", True, "完成"))
    projector.begin_landing(11)
    projector.observe_result(CommandResult(4, 11, "land", True, "完成"))
    projector.observe_ground_media((_record("video", mission_sequence=10),))
    assert [item["uavStatus"] for item in sent].count("08") == 2
    projector.observe_ground_media((_record(),))
    assert sent[-1]["uavStatus"] == "09"
    assert sent[-1]["data"]["pointNo"] == "10"
    assert not projector._photo_missions


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


def test_reader_posts_terminal_failure_in_background_and_projects_delivery(tmp_path, monkeypatch) -> None:
    """失败终态只在后台POST，任务身份和超时配置一并送入投影器。"""
    posted, snapshots = [], []
    completed = threading.Event()

    def open_media(target, timeout):
        """返回任务状态，并记录POST发生的线程和完整身份。"""
        if not isinstance(target, str):
            posted.append((json.loads(target.data), threading.get_ident()))
            return io.StringIO('{"ok":true}')
        return io.StringIO(json.dumps({
            "records": [], "delivery_timeout_seconds": 15,
            "deliveries": [{
                "aircraft_id": "refresh", "session_id": "session", "source_id": "gui-one",
                "mission_sequence": 8, "state": "failed", "error": "原片丢失",
            }],
        }))

    def on_snapshot(records, error):
        """保存完整快照用于验证synthetic记录。"""
        snapshots.extend(records)
        completed.set()

    monkeypatch.setattr("ground_station_core.ground_media_reader.urlopen", open_media)
    reader = GroundMediaReader(on_snapshot, config_path=tmp_path / "media.json", interval=10.0)
    payload = {"source_id": "gui-one", "mission_sequence": 8, "error": "交付超时"}
    reader.fail_delivery(payload)
    assert not posted
    reader.start()
    try:
        assert completed.wait(1.0)
        assert posted[0][0] == {**payload, "action": "fail"}
        assert posted[0][1] != threading.get_ident()
        delivery = next(record for record in snapshots if record["kind"] == "delivery")
        assert delivery["metadata"] == {"source_id": "gui-one"}
        assert delivery["state"] == "failed"
        assert delivery["session_id"] == "session"
        policy = next(record for record in snapshots if record["kind"] == "delivery_policy")
        assert policy["delivery_timeout_seconds"] == 15
    finally:
        assert reader.stop()


def test_reader_restores_unsent_failure_after_restart(tmp_path, monkeypatch) -> None:
    """离线退出前保存终态通知，重启后先通知后台终止再读取媒体。"""
    config = tmp_path / "media.json"
    config.write_text('{"api_url":"http://media.test"}')
    failed = threading.Event()

    def offline(target, timeout):
        """模拟失败通知尚未送达后台。"""
        failed.set()
        raise OSError("网络中断")

    monkeypatch.setattr("ground_station_core.ground_media_reader.urlopen", offline)
    reader = GroundMediaReader(lambda records, error: None, config_path=config, interval=10.0)
    payload = {"aircraft_id": "refresh", "source_id": "gui-one", "mission_sequence": 8,
               "error": "落地后交付超时"}
    reader.fail_delivery(payload)
    reader.start()
    assert failed.wait(1.0)
    assert reader.stop()
    spool = tmp_path / "media-delivery-failures.json"
    assert json.loads(spool.read_text()) == [{**payload, "action": "fail"}]
    restored, completed = [], threading.Event()

    def online(target, timeout):
        """验证重启后的第一项网络操作就是终态通知。"""
        restored.append(target)
        return io.StringIO('{"ok":true}' if not isinstance(target, str) else '{"records":[]}')

    monkeypatch.setattr("ground_station_core.ground_media_reader.urlopen", online)
    resumed = GroundMediaReader(lambda records, error: completed.set(), config_path=config, interval=10.0)
    resumed.start()
    try:
        assert completed.wait(1.0)
        assert json.loads(restored[0].data) == {**payload, "action": "fail"}
        assert restored[1] == "http://media.test/api/media"
        assert json.loads(spool.read_text()) == []
    finally:
        assert resumed.stop()
