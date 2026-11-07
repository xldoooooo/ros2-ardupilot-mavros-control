"""验证仿真媒体配置、真实路径、进程取消及隔离端口上的 RTSP 解码。"""

import json
from pathlib import Path
import shutil
import socket
import subprocess
import sys
import threading
import time

import pytest

from ground_station_core.event_log import EventLog
from ground_station_core.upstream import simulation_media as module
from ground_station_core.upstream.simulation_media import SimulationMedia


def _make_media(tmp_path, monkeypatch, **config):
    """建立临时公开目录与配置，不读取或写入现场媒体配置。"""
    ground = tmp_path / "ground.json"
    ground.write_text(json.dumps({"root": str(tmp_path), "rtsp_port": 9854}))
    monkeypatch.setenv("GROUND_MEDIA_CONFIG", str(ground))
    simulation = tmp_path / "simulation.json"
    simulation.write_text(json.dumps(config))
    for directory in ("videos", "pictures"):
        (tmp_path / "public" / "examples" / directory).mkdir(parents=True, exist_ok=True)
    return SimulationMedia(EventLog(), simulation)


def test_config_precedence_and_template_is_not_loaded(tmp_path, monkeypatch):
    """默认配置独立于工作目录；现场配置优先，example 不参与运行。"""
    monkeypatch.setattr(module, "_PROJECT", tmp_path)
    monkeypatch.delenv("GROUND_MEDIA_CONFIG", raising=False)
    directory = tmp_path / "config" / "ground"
    directory.mkdir(parents=True)
    (directory / "media.json").write_text('{"root": "store", "rtsp_port": 9654}')
    (directory / "simulation_media.example.json").write_text('{"root": "WRONG"}')
    media = SimulationMedia(EventLog())
    assert media.public_root == tmp_path / "store" / "public"
    assert media.rtsp_url == "rtsp://127.0.0.1:9654/uav002/rectified"
    (directory / "simulation_media.json").write_text('{"root": "override", "rtsp_port": 9754}')
    media = SimulationMedia(EventLog())
    assert media.public_root == tmp_path / "override" / "public"
    assert media.rtsp_url == "rtsp://127.0.0.1:9754/uav002/rectified"
    explicit = tmp_path / "chosen.json"
    explicit.write_text('{"public_root": "shared", "videos_dir": "v", "rtsp_url": "rtsp://localhost/demo"}')
    media = SimulationMedia(EventLog(), explicit)
    assert media.public_root == tmp_path / "shared"
    assert media.videos_dir == tmp_path / "shared" / "v"
    assert media.rtsp_url == "rtsp://localhost/demo"
    (directory / "media.json").write_text(json.dumps({
        "remote_ftp": {"enabled": True, "path_prefix": "/ftp/s12/"},
    }))
    prefixed = SimulationMedia(EventLog(), explicit)
    assert prefixed._ftp_path(prefixed.public_root / "uav002" / "video.mp4") == "/ftp/s12/uav002/video.mp4"
    (directory / "media.json").unlink()
    explicit.write_text('[]')
    media = SimulationMedia(EventLog(), explicit)
    assert media.public_root == Path.home() / "Media" / "ardupilot" / "public"
    assert any("配置读取失败" in event.message for event in media._events.snapshot())


def test_existing_paths_random_choice_and_missing_resources(tmp_path, monkeypatch):
    """只交付 public 下非空现有文件；删除资源后不返回缓存假路径。"""
    media = _make_media(tmp_path, monkeypatch)
    video = media.videos_dir / "sample.mp4"
    video.write_bytes(b"sample")
    first = media.pictures_dir / "a.JPG"
    second = media.pictures_dir / "中文 b.png"
    first.write_bytes(b"sample")
    second.write_bytes(b"sample")
    (media.pictures_dir / "empty.jpg").touch()
    outside = tmp_path / "private.jpg"
    outside.write_bytes(b"private")
    (media.pictures_dir / "outside.jpg").symlink_to(outside)
    monkeypatch.setattr(module.random, "choice", lambda files: files[-1])
    assert media.photo_path() == "/examples/pictures/中文 b.png"
    assert media.completion_payload() == {
        "videoPath": "/examples/videos/sample.mp4", "JPGPath": "/examples/pictures/",
    }
    monkeypatch.setattr(module.random, "choice", lambda files: files[0])
    assert media.photo_path() == "/examples/pictures/a.JPG"
    for path in (video, first, second):
        path.unlink()
    assert media.photo_path() == ""
    assert media.completion_payload() == {"videoPath": "", "JPGPath": ""}
    media._play(threading.Event())
    assert any("没有仿真视频" in event.message for event in media._events.snapshot())


def test_playlist_order_and_restart_from_first(tmp_path, monkeypatch):
    """单进程循环有序列表，取消后重启也从第一个视频开始。"""
    media = _make_media(tmp_path, monkeypatch)
    for name in ("b.mp4", "a.mp4", "c.mp4"):
        (media.videos_dir / name).write_bytes(b"sample")
    played = []

    def execute(args, stop, *, probe=False):
        """检查真实生成的concat列表与输入循环参数，不连接媒体服务。"""
        if probe:
            return 0, "h264\n", ""
        assert args[args.index("-stream_loop") + 1] == "-1"
        assert args[args.index("-f") + 1] == "concat"
        playlist = Path(args[args.index("-i") + 1]).read_text().splitlines()
        assert playlist == ["ffconcat version 1.0"] + [
            f"file '{media.videos_dir / name}'" for name in ("a.mp4", "b.mp4", "c.mp4")
        ]
        played.append(playlist)
        stop.set()
        return 0, "", ""

    monkeypatch.setattr(media, "_execute", execute)
    for _ in range(2):
        media._play(threading.Event())
    assert len(played) == 2
    assert played[0] == played[1]


@pytest.mark.integration
def test_stop_during_spawn_restart_and_process_ownership(tmp_path, monkeypatch):
    """stop 不等待 Popen 或 join；快速重启串行接续且不杀无关进程。"""
    media = _make_media(tmp_path, monkeypatch)
    original_popen = subprocess.Popen
    command = [sys.executable, "-c", "import time; time.sleep(30)"]
    unrelated = original_popen(command)
    spawned, resumed, release = threading.Event(), threading.Event(), threading.Event()
    processes = []

    def delayed_popen(*args, **kwargs):
        """仅阻塞第一次进程登记，模拟 GUI 取消时仍在创建进程。"""
        process = original_popen(*args, **kwargs)
        processes.append(process)
        if len(processes) == 1:
            spawned.set()
            release.wait(5)
        else:
            resumed.set()
        return process

    def play(cancel):
        """用真实休眠进程验证取消，而不连接任何媒体服务。"""
        media._execute(command, cancel)

    monkeypatch.setattr(module.subprocess, "Popen", delayed_popen)
    monkeypatch.setattr(media, "_play", play)
    try:
        before = time.monotonic()
        media.start()
        assert time.monotonic() - before < 1
        assert spawned.wait(3)
        worker = media._thread
        media.start()
        assert media._thread is worker
        before = time.monotonic()
        media.stop()
        assert time.monotonic() - before < 0.5
        media.start()
        release.set()
        assert resumed.wait(3)
        before = time.monotonic()
        media.stop()
        assert time.monotonic() - before < 0.5
        media.close()
        assert len(processes) == 2
        assert all(process.poll() is not None for process in processes)
        assert unrelated.poll() is None
        assert not worker.is_alive()
        media.close()
        media.start()
        assert media._thread is worker and not worker.is_alive()
    finally:
        release.set()
        media.close()
        unrelated.kill()
        unrelated.wait()


def test_publish_failure_is_logged_and_retry_is_cancellable(tmp_path, monkeypatch):
    """发布错误不冒充成功，长重试等待也能立即取消。"""
    media = _make_media(tmp_path, monkeypatch)
    video = media.videos_dir / "sample.mp4"
    video.write_bytes(b"sample")
    failed = threading.Event()

    def execute(args, cancel, *, probe=False):
        """模拟可探测视频但 RTSP 拒绝发布，不连接默认目标。"""
        if probe:
            return 0, "h264\n", ""
        assert args[args.index("-c:v") + 1] == "copy"
        assert "-re" in args and "-vf" not in args
        failed.set()
        return 1, "", "publish rejected"

    monkeypatch.setattr(media, "_execute", execute)
    monkeypatch.setattr(module, "_RETRY_SECONDS", 30)
    assert not media._publish([video], threading.Event())
    assert any("publish rejected" in event.message for event in media._events.snapshot())
    failed.clear()
    try:
        media.start()
        assert failed.wait(3)
        before = time.monotonic()
        media.close()
        assert time.monotonic() - before < 1
    finally:
        media.close()


@pytest.mark.parametrize("source_codec,fps", [("libx264", None), ("mpeg4", None), ("libx264", 30)])
@pytest.mark.skipif(not shutil.which("ffmpeg") or not shutil.which("ffprobe"), reason="需要 FFmpeg")
@pytest.mark.integration
def test_real_rtsp_decodes_on_isolated_port(tmp_path, monkeypatch, source_codec, fps):
    """同一个RTSP接收连接跨片段和两轮循环，逐帧核对颜色顺序，无黑帧。"""
    with socket.socket() as sock:
        sock.bind(("127.0.0.1", 0))
        port = sock.getsockname()[1]
    url = f"rtsp://127.0.0.1:{port}/uav002/rectified"
    media = _make_media(tmp_path, monkeypatch, rtsp_url=url, fps=fps, bitrate="10M")
    multiplier = 3 if fps else 1
    for name, color in (("b.mp4", "blue"), ("a.mp4", "red")):
        subprocess.run([
            "ffmpeg", "-v", "error", "-f", "lavfi", "-i", f"color={color}:size=128x96:rate=10",
            "-t", "0.6", "-an", "-c:v", source_codec, "-threads", "1", str(media.videos_dir / name),
        ], check=True, capture_output=True, timeout=15)
    # 保留含B帧输入的启动负PTS，不让测试接收器默认CFR丢掉首帧。
    receiver = subprocess.Popen([
        "ffmpeg", "-hide_banner", "-loglevel", "info" if fps else "error", "-nostdin",
        "-rtsp_flags", "listen", "-rtsp_transport", "tcp", "-listen_timeout", "8",
        "-copyts", "-i", url, "-frames:v", str(27 * multiplier), "-threads", "1", "-vf", "scale=1:1",
        "-fps_mode", "passthrough", "-pix_fmt", "rgb24", "-f", "rawvideo", "-",
    ], stdin=subprocess.DEVNULL, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    try:
        media.start()
        output, error = receiver.communicate(timeout=15)
        assert receiver.returncode == 0, error.decode(errors="replace")
        assert len(output) == 27 * multiplier * 3, (len(output), error)
        colors = ["red" if output[i] > output[i + 2] else "blue" for i in range(0, len(output), 3)]
        assert colors == (["red"] * (6 * multiplier) + ["blue"] * (6 * multiplier)) * 2 + ["red"] * (3 * multiplier)
        assert all(max(output[i:i + 3]) > 200 for i in range(0, len(output), 3))
        expected = "copy" if source_codec == "libx264" and not fps else "libx264"
        assert any(f"（{expected}）" in event.message for event in media._events.snapshot())
        if fps:
            assert "1920x1080" in error.decode(errors="replace")
            assert "30 fps" in error.decode(errors="replace")
    finally:
        media.close()
        if receiver.poll() is None:
            receiver.kill()
        receiver.communicate()
