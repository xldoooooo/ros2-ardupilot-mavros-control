"""独立摄像头配置、命令构造、IPC 与面板生命周期回归测试。"""

from __future__ import annotations

from dataclasses import replace
import json
import os
from pathlib import Path
import subprocess
import tempfile
import threading
import time
from urllib.parse import unquote

# 必须在首次导入 Qt 前选择无显示平台，保证 CI 不依赖桌面会话。
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import pytest
from PySide6.QtCore import QEvent, QPoint, QPointF, Qt
from PySide6.QtGui import QWheelEvent
from PySide6.QtMultimedia import QMediaPlayer
from PySide6.QtWidgets import QApplication, QLabel

from camera_app import config as config_module
from camera_app import controller as controller_module
from camera_app.config import (
    CameraConfig,
    CameraMode,
    RuntimePaths,
    detect_lan_ipv4,
    load_config,
    parse_v4l2_formats,
    save_config,
)
from camera_app.controller import CameraController, CameraServiceError
from camera_app.ipc import CameraServiceClient, CameraServiceServer
from camera_app.onboard_config import load_lens_controls, load_onboard_settings
from camera_app.panel import (
    DESKTOP_APPLICATION_NAME,
    CameraPanelWindow,
    NoWheelComboBox,
    NoWheelSpinBox,
    PANEL_STYLE_SHEET,
    SourceMode,
)


def _runtime_paths(root: Path) -> RuntimePaths:
    """为单个测试创建完全隔离的配置、Socket 和日志路径。"""
    runtime = root / "run"
    state = root / "state"
    return RuntimePaths(
        config_file=root / "config" / "camera.json",
        runtime_dir=runtime,
        state_dir=state,
        socket_file=runtime / "camera-service.sock",
        pid_file=runtime / "camera-service.pid",
        mediamtx_config=runtime / "mediamtx.yml",
        mediamtx_log=state / "mediamtx.log",
        ffmpeg_log=state / "ffmpeg.log",
    )


def _camera_config(root: Path, **changes: object) -> CameraConfig:
    """返回不依赖当前用户目录、适合命令和持久化测试的配置。"""
    base = CameraConfig(
        device="/dev/video-test",
        codec="h264",
        width=1920,
        height=1080,
        fps=30.0,
        rtsp_ip="192.168.10.20",
        rtsp_port=18554,
        rtsp_path="camera/primary",
        container="mp4",
        video_directory=str(root / "videos"),
        image_directory=str(root / "images"),
    )
    return replace(base, **changes)


def _application() -> QApplication:
    """复用 Qt 全局应用实例。"""
    return QApplication.instance() or QApplication([])


class _FakeOnboardVideoClient:
    """不创建 DDS participant 的三模式面板测试替身。"""

    def __init__(self) -> None:
        self.started = 0
        self.closed = 0
        self.state_requests: list[bool] = []
        self.snapshot_requests = 0
        self.current_status: dict[str, object] = {
            "interface_version": "3.2",
            "service_available": True,
            "running": True,
            "state": "running",
            "rtsp_url": "rtsp://aircraft.test:8554/camera",
        }

    def start(self) -> None:
        self.started += 1

    def close(self) -> None:
        self.closed += 1

    def status(self) -> dict[str, object]:
        return dict(self.current_status)

    def request_state(self, enabled: bool, callback=None) -> None:
        self.state_requests.append(enabled)
        if callback is not None:
            callback({"accepted": True}, "")

    def request_snapshot(self, callback=None) -> None:
        self.snapshot_requests += 1
        if callback is not None:
            callback({"published": True}, "")


def test_panel_role_buttons_use_ground_station_disabled_style() -> None:
    """彩色角色按钮禁用后必须显式回到主GUI的灰色样式。"""
    disabled_selectors = (
        'QPushButton[role="primary"]:disabled,\n'
        'QPushButton[role="success"]:disabled,\n'
        'QPushButton[role="danger"]:disabled'
    )

    assert disabled_selectors in PANEL_STYLE_SHEET
    assert "color: #98a4b1; background: #edf0f2;" in PANEL_STYLE_SHEET
    assert PANEL_STYLE_SHEET.index(disabled_selectors) > PANEL_STYLE_SHEET.index(
        'QPushButton[role="danger"]'
    )


def test_panel_desktop_name_is_ascii() -> None:
    """Dock应用名和窗口标题必须使用同一个无编码歧义的英文名称。"""
    application = _application()
    window = CameraPanelWindow(auto_bootstrap=False)

    assert DESKTOP_APPLICATION_NAME == "ROS 2 ArduPilot Camera Panel"
    assert DESKTOP_APPLICATION_NAME.isascii()
    assert window.windowTitle() == DESKTOP_APPLICATION_NAME

    window.close()
    window.deleteLater()
    application.sendPostedEvents(None, QEvent.Type.DeferredDelete)
    application.processEvents()


def test_parse_v4l2_formats_keeps_native_h264_and_mjpeg_modes() -> None:
    """V4L2 离散能力应原样成为 GUI 可选组合。"""
    output = """
        [0]: 'H264' (H.264, compressed)
            Size: Discrete 1920x1080
                Interval: Discrete 0.033s (30.000 fps)
            Size: Discrete 1280x720
                Interval: Discrete 0.008s (120.000 fps)
        [1]: 'MJPG' (Motion-JPEG, compressed)
            Size: Discrete 1280x720
                Interval: Discrete 0.008s (120.000 fps)
        [2]: 'YUYV' (YUYV 4:2:2)
            Size: Discrete 640x480
                Interval: Discrete 0.033s (30.000 fps)
    """

    modes = parse_v4l2_formats(output)

    assert {(item.codec, item.width, item.height, item.fps) for item in modes} == {
        ("h264", 1920, 1080, 30.0),
        ("h264", 1280, 720, 120.0),
        ("mjpeg", 1280, 720, 120.0),
    }


def test_lan_address_prefers_main_default_route_over_tun(monkeypatch) -> None:
    """默认展示地址取主路由源 IP，不被策略代理 TUN 路由劫持。"""
    routes = [
        {
            "dst": "default",
            "dev": "wlp3s0",
            "prefsrc": "192.168.112.176",
            "metric": 600,
        }
    ]
    completed = subprocess.CompletedProcess(
        args=["ip"], returncode=0, stdout=json.dumps(routes), stderr=""
    )
    monkeypatch.setattr(
        config_module.subprocess, "run", lambda *args, **kwargs: completed
    )

    assert detect_lan_ipv4() == "192.168.112.176"


def test_camera_config_round_trip_and_rejects_tee_metacharacters(
    tmp_path: Path,
) -> None:
    """配置原子往返，且路径不能破坏 FFmpeg tee 输出语法。"""
    paths = _runtime_paths(tmp_path)
    config = _camera_config(tmp_path)

    save_config(config, paths)

    assert load_config(paths) == config
    assert paths.config_file.stat().st_mode & 0o777 == 0o600
    with pytest.raises(ValueError, match="不支持的字符"):
        replace(config, video_directory=str(tmp_path / "bad|path")).validate()
    with pytest.raises(ValueError, match="不支持的字符"):
        replace(config, image_directory=str(tmp_path / "bad[path")).validate()


@pytest.mark.parametrize("container", ["mp4", "mkv", "avi"])
def test_ffmpeg_command_uses_one_capture_fixed_timestamps_and_tee(
    tmp_path: Path,
    container: str,
) -> None:
    """所有保存格式都复用一次采集，并同时输出 RTSP 与录像。"""
    paths = _runtime_paths(tmp_path)
    config = _camera_config(tmp_path, container=container)
    controller = CameraController(paths=paths)
    try:
        command = controller.build_ffmpeg_command(
            config, tmp_path / f"recording.partial.{container}"
        )
    finally:
        controller.close()

    assert command.count("-i") == 1
    assert command[command.index("-input_format") + 1] == "h264"
    assert command[command.index("-c:v") + 1] == "copy"
    assert command[command.index("-bsf:v") + 1] == (
        "setts=pts=N/(30*TB):dts=N/(30*TB)"
    )
    tee_output = command[-1]
    assert "f=rtsp:rtsp_transport=tcp" in tee_output
    assert config.local_rtsp_url in tee_output
    expected_muxer = {"mp4": "f=mp4", "mkv": "f=matroska", "avi": "f=avi"}
    assert expected_muxer[container] in tee_output


def test_mjpeg_command_remains_zero_transcode_fallback(tmp_path: Path) -> None:
    """MJPEG 回退仍使用摄像头原生码流，不额外引入编码器负载。"""
    paths = _runtime_paths(tmp_path)
    config = _camera_config(
        tmp_path,
        codec="mjpeg",
        width=1280,
        height=720,
        fps=120.0,
        container="mkv",
    )
    controller = CameraController(paths=paths)
    try:
        command = controller.build_ffmpeg_command(
            config, tmp_path / "recording.partial.mkv"
        )
    finally:
        controller.close()

    assert command[command.index("-input_format") + 1] == "mjpeg"
    assert command[command.index("-c:v") + 1] == "copy"
    assert command[command.index("-bsf:v") + 1] == (
        "setts=pts=N/(120*TB):dts=N/(120*TB)"
    )


def test_dri_mjpeg_normalizes_only_rtsp_and_preserves_recording_copy(
    tmp_path: Path,
) -> None:
    """含DRI的MJPEG只规范化RTSP分支，原始录像不能被二次编码。"""
    paths = _runtime_paths(tmp_path)
    config = _camera_config(
        tmp_path,
        codec="mjpeg",
        width=1920,
        height=1080,
        fps=30.0,
        container="mp4",
    )
    recording = tmp_path / "recording.partial.mp4"
    controller = CameraController(paths=paths)
    try:
        command = controller.build_ffmpeg_command(
            config,
            recording,
            normalize_mjpeg_for_rtsp=True,
        )
    finally:
        controller.close()

    codec_positions = [
        index for index, argument in enumerate(command) if argument == "-c:v"
    ]
    assert command.count("-i") == 1
    assert [command[index + 1] for index in codec_positions] == ["copy", "mjpeg"]
    assert str(recording) in command
    assert config.local_rtsp_url == command[-1]
    assert command[command.index("-pix_fmt") + 1] == "yuvj420p"
    assert command[command.index("-huffman") + 1] == "default"
    assert "-force_duplicated_matrix" not in command


@pytest.mark.parametrize("has_dri", [False, True])
def test_mjpeg_restart_interval_detection_reads_only_jpeg_header(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    has_dri: bool,
) -> None:
    """仅JPEG扫描数据前的DRI标记触发兼容路径，避免误判压缩数据。"""
    header = b"\xff\xd8\xff\xdb\x00\x04\x00\x00"
    if has_dri:
        header += b"\xff\xdd\x00\x04\x00\x78"
    frame = header + b"\xff\xda\x00\x02payload\xff\xdd\xff\xd9"
    completed = subprocess.CompletedProcess(
        args=["ffmpeg"], returncode=0, stdout=frame, stderr=b""
    )
    monkeypatch.setattr(
        controller_module.subprocess, "run", lambda *args, **kwargs: completed
    )
    controller = CameraController(
        paths=_runtime_paths(tmp_path), initial_config=_camera_config(tmp_path)
    )
    try:
        detected = controller._mjpeg_has_restart_interval(
            _camera_config(tmp_path, codec="mjpeg")
        )
    finally:
        controller.close()

    assert detected is has_dri


def test_stream_copy_progress_derives_packet_count_from_media_time(
    tmp_path: Path,
) -> None:
    """FFmpeg stream-copy 的 frame=0 时仍能展示真实处理帧数与吞吐。"""
    controller = CameraController(paths=_runtime_paths(tmp_path))
    try:
        controller._config = _camera_config(tmp_path, fps=30.0)
        controller._progress = {"frame": "0", "out_time_us": "5000000"}
        controller._started_monotonic = time.monotonic() - 5.0
        controller._progress_baseline_media = 1.0
        controller._progress_baseline_monotonic = time.monotonic() - 4.0
        status = controller.status()
    finally:
        controller.close()

    assert status["media_elapsed_seconds"] == 5.0
    assert status["frame"] == 150
    assert 29.0 <= status["measured_fps"] <= 30.0


def test_generated_mediamtx_config_is_rtsp_tcp_only(tmp_path: Path) -> None:
    """生成配置只开放 RTSP/TCP，并仅允许本机向指定路径发布。"""
    paths = _runtime_paths(tmp_path)
    paths.ensure_directories()
    config = _camera_config(tmp_path)
    controller = CameraController(paths=paths)
    try:
        controller._write_mediamtx_config(config)
        text = paths.mediamtx_config.read_text(encoding="utf-8")
    finally:
        controller.close()

    assert "rtspTransports: [tcp]" in text
    assert "writeQueueSize: 1024" in text
    assert 'rtspAddress: "0.0.0.0:18554"' in text
    assert 'ips: ["127.0.0.1", "::1"]' in text
    for protocol in ("rtmp", "hls", "webrtc", "srt", "moq"):
        assert f"{protocol}: false" in text


def test_unix_socket_service_configures_without_ground_station() -> None:
    """独立 Socket 后台可由 CLI/面板控制，不依赖地面站对象。"""
    # Linux AF_UNIX 路径上限约 108 字节，显式使用短根目录验证真实协议。
    with tempfile.TemporaryDirectory(prefix="camera-ipc-", dir="/tmp") as root:
        temporary_root = Path(root)
        paths = _runtime_paths(temporary_root)
        save_config(_camera_config(temporary_root), paths)
        controller = CameraController(paths=paths)
        server = CameraServiceServer(controller, paths=paths)
        worker = threading.Thread(target=server.serve_until_stopped, daemon=True)
        worker.start()
        client = CameraServiceClient(paths=paths, timeout=1.0)
        try:
            status = client.wait_until_ready(2.0)
            assert status["state"] == "stopped"
            configured = client.request(
                "configure", {"config": {"rtsp_path": "camera/secondary"}}
            )
            assert configured["rtsp_path"] == "camera/secondary"
            with pytest.raises(CameraServiceError, match="未知摄像头命令"):
                client.request("run-shell")
            assert client.request("shutdown") == {"shutting_down": True}
            worker.join(timeout=2.0)
            assert not worker.is_alive()
        finally:
            server.stop_requested.set()
            server.close_and_cleanup()
            controller.close()

        assert not paths.socket_file.exists()
        assert not paths.pid_file.exists()


def test_probe_hides_mjpeg_modes_above_rtp_jpeg_limit(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """超过RFC 2435尺寸字段的MJPEG不能进入RTSP画质列表。"""
    modes = [
        CameraMode("mjpeg", 2560, 1920, 30.0),
        CameraMode("mjpeg", 1920, 1080, 30.0),
        CameraMode("h264", 3840, 2160, 30.0),
    ]
    monkeypatch.setattr(controller_module, "discover_camera_devices", lambda: [])
    monkeypatch.setattr(controller_module, "probe_camera_modes", lambda _device: modes)
    controller = CameraController(paths=_runtime_paths(tmp_path))
    try:
        result = controller.probe("/dev/video-test")
        with pytest.raises(CameraServiceError, match="RFC 2435"):
            controller._preflight(
                _camera_config(
                    tmp_path,
                    codec="mjpeg",
                    width=2560,
                    height=1920,
                    fps=30.0,
                )
            )
    finally:
        controller.close()

    supported = {
        (item["codec"], item["width"], item["height"])
        for item in result["modes"]
    }
    assert supported == {
        ("mjpeg", 1920, 1080),
        ("h264", 3840, 2160),
    }
    assert result["excluded_modes"] == [
        {
            "codec": "mjpeg",
            "width": 2560,
            "height": 1920,
            "fps": 30.0,
        }
    ]


def test_panel_close_does_not_send_camera_stop(tmp_path: Path) -> None:
    """关闭面板不发 stop，且本机状态不再自动驱动 RTSP 播放器。"""
    application = _application()
    paths = _runtime_paths(tmp_path)
    client = CameraServiceClient(paths=paths)
    window = CameraPanelWindow(client=client, auto_bootstrap=False)
    window.show()
    application.processEvents()
    config = _camera_config(tmp_path).to_dict()
    window._service_ready = True
    window._apply_status(
        {
            "state": "stopped",
            "running": False,
            "config": config,
            "rtsp_url": config["rtsp_url"],
        }
    )
    window._apply_probe(
        {
            "devices": [
                {"label": "Test Camera", "path": config["device"]}
            ],
            "modes": [
                {
                    "codec": "h264",
                    "width": 1920,
                    "height": 1080,
                    "fps": 30.0,
                }
            ],
            "error": "",
        }
    )
    application.processEvents()

    labels = {label.text() for label in window.findChildren(QLabel)}
    assert not hasattr(window, "state_chip")
    assert not hasattr(window, "preview_state")
    forbidden_labels = {
        "独立 RTSP 推流、录像与截图服务 · 不依赖飞控或 ROS",
        "RTSP实时预览",
        "服务固定监听所有本机网卡；这里的IP用于生成局域网拉流地址。",
        "H.264默认零转码并修正固定帧率时间戳；MJPEG作为兼容回退。",
        "摄像头、RTSP和录像已启动。",
    }
    assert labels.isdisjoint(forbidden_labels)
    assert window.device_label.x() < window.device_combo.x()
    assert window.refresh_devices_button.x() > (
        window.device_combo.x() + window.device_combo.width()
    )
    assert window.device_combo.width() > window.refresh_devices_button.width()
    assert window.refresh_devices_button.width() == 88
    assert window.start_button.isEnabled()
    assert not window.stop_button.isEnabled()

    preview_urls: list[str] = []
    window._ensure_preview = preview_urls.append  # type: ignore[method-assign]
    custom_url = "rtsp://example.test:8554/operator-selected"
    window.rtsp_url_input.setText(custom_url)
    window._mark_url_edited(custom_url)
    window._apply_status(
        {
            "state": "running",
            "running": True,
            "config": config,
            "rtsp_url": config["rtsp_url"],
            "recording_file": str(tmp_path / "recording.mp4"),
            "elapsed_seconds": 5.0,
            "frame": 150,
            "measured_fps": 30.0,
        }
    )

    assert preview_urls == []
    assert window.rtsp_url_input.text() == custom_url
    assert not window.codec_combo.isEnabled()
    assert not window.start_button.isEnabled()
    assert window.stop_button.isEnabled()
    assert window.snapshot_button.isEnabled()
    assert window.fps_value.text() == "0.0 fps"
    assert window.elapsed_value.text() == "00:00:00"
    assert not hasattr(window, "recording_value")
    assert not hasattr(window, "frame_value")
    assert window._request_in_flight == set()

    window.close()
    window.deleteLater()
    application.sendPostedEvents(None, QEvent.Type.DeferredDelete)
    application.processEvents()
    assert window._request_in_flight == set()


def test_panel_probe_keeps_requested_device_and_matching_modes(
    tmp_path: Path,
) -> None:
    """双摄像头探测结果不能被磁盘中旧设备覆盖成混合配置。"""
    application = _application()
    window = CameraPanelWindow(
        client=CameraServiceClient(paths=_runtime_paths(tmp_path)),
        auto_bootstrap=False,
    )
    hp_device = "/dev/v4l/by-id/usb-Quanta_HP-video-index0"
    wasintek_device = "/dev/v4l/by-id/usb-Wasintek-camera-video-index0"
    config = _camera_config(tmp_path, device=hp_device).to_dict()
    window._service_ready = True
    window._apply_status(
        {
            "state": "stopped",
            "running": False,
            "config": config,
            "rtsp_url": config["rtsp_url"],
        }
    )

    window._apply_probe(
        {
            "devices": [
                {"label": "HP 5MP Camera", "path": hp_device},
                {"label": "Wasintek camera", "path": wasintek_device},
            ],
            "selected_device": wasintek_device,
            "modes": [
                {
                    "codec": "h264",
                    "width": 1280,
                    "height": 720,
                    "fps": 120.0,
                }
            ],
            "excluded_modes": [
                {
                    "codec": "mjpeg",
                    "width": 2560,
                    "height": 1920,
                    "fps": 30.0,
                }
            ],
            "error": "",
        }
    )

    assert window.device_combo.currentData() == wasintek_device
    assert window.device_path_label.text() == wasintek_device
    assert window.mode_combo.count() == 1
    assert "已隐藏 1 个" in window.operation_message.text()
    assert window._configuration_from_fields() == {
        "device": wasintek_device,
        "codec": "h264",
        "width": 1280,
        "height": 720,
        "fps": 120.0,
        "rtsp_ip": "192.168.10.20",
        "rtsp_port": 18554,
        "rtsp_path": "camera/primary",
        "container": "mp4",
        "video_directory": str(tmp_path / "videos"),
        "image_directory": str(tmp_path / "images"),
    }

    window.close()
    window.deleteLater()
    application.sendPostedEvents(None, QEvent.Type.DeferredDelete)
    application.processEvents()


def test_preview_recreates_player_before_reusing_same_rtsp_url(
    tmp_path: Path,
) -> None:
    """停止、换地址或显式重连必须换新播放器。"""
    application = _application()
    window = CameraPanelWindow(
        client=CameraServiceClient(paths=_runtime_paths(tmp_path)),
        auto_bootstrap=False,
    )
    window.show()
    application.processEvents()
    url = "rtsp://127.0.0.1:65534/camera"

    first_player = window.player
    window._ensure_preview(url)
    assert window.player.source().toString() == url
    window._stop_preview("等待摄像头启动")
    second_player = window.player

    assert second_player is not first_player
    assert not second_player.source().isValid()
    assert window._active_preview_url == ""

    window._ensure_preview(url)
    assert window.player is second_player
    assert window.player.source().toString() == url
    window.rtsp_url_input.setText("rtsp://127.0.0.1:65533/changed")
    window._toggle_preview()
    third_player = window.player
    assert third_player is not second_player
    assert window.player.source().toString().endswith("/changed")
    window._reconnect_preview()
    assert window.player is not third_player
    assert window.player.source().toString().endswith("/changed")

    window.close()
    window.deleteLater()
    application.sendPostedEvents(None, QEvent.Type.DeferredDelete)
    application.processEvents()


def test_snapshot_command_and_setts_are_ffmpeg_44_compatible(tmp_path: Path) -> None:
    """机载 Jammy 命令不得使用 FFmpeg 4.4 尚不存在的选项。"""
    controller = CameraController(paths=_runtime_paths(tmp_path))
    config = _camera_config(tmp_path)
    try:
        snapshot = controller.build_snapshot_command(config, tmp_path / "image.jpg")
        stream = controller.build_ffmpeg_command(config, tmp_path / "video.mp4")
    finally:
        controller.close()

    assert "-fps_mode" not in snapshot
    assert snapshot[snapshot.index("-vsync") + 1] == "passthrough"
    timestamp = stream[stream.index("-bsf:v") + 1]
    assert timestamp == "setts=pts=N/(30*TB):dts=N/(30*TB)"
    assert "duration=" not in timestamp
    assert "time_base=" not in timestamp


def test_onboard_configs_preserve_exact_camera_and_lens_values() -> None:
    """机载配置使用协议目录/JPG，并原值保存同型号镜头要求。"""
    settings = load_onboard_settings()
    controls = dict(load_lens_controls(settings.lens_config))

    assert settings.camera.video_directory == "/home/share"
    assert settings.camera.image_directory == "/home/share/jpg"
    assert settings.camera.container == "mp4"
    assert (settings.camera.width, settings.camera.height, settings.camera.fps) == (
        1280,
        720,
        120.0,
    )
    assert controls == {
        "auto_exposure": "1",
        "exposure_time_absolute": "25",
        "gain": "200",
        "brightness": "6",
        "contrast": "6",
        "saturation": "6",
        "hue": "0",
        "sharpness": "6",
        "power_line_frequency": "1",
        "zoom_absolute": "10",
    }


def test_lens_controls_apply_after_stream_settle_sequentially_and_verify(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """流稳定一秒后须分步写控制、等待关键项并逐项读回。"""
    controls = load_lens_controls()
    calls: list[list[str]] = []
    sleeps: list[float] = []

    def run(command, **_kwargs):
        calls.append(list(command))
        if any(argument.startswith("--get-ctrl=") for argument in command):
            output = "\n".join(
                f"{name}: {value}"
                + (" (Manual Mode)" if name == "auto_exposure" else "")
                for name, value in controls
            )
            return subprocess.CompletedProcess(command, 0, output)
        return subprocess.CompletedProcess(command, 0, "")

    monkeypatch.setattr(controller_module.subprocess, "run", run)
    monkeypatch.setattr(controller_module.time, "sleep", sleeps.append)
    controller = CameraController(
        paths=_runtime_paths(tmp_path), initial_config=_camera_config(tmp_path)
    )
    try:
        monkeypatch.setattr(controller, "_ffmpeg_running", lambda: True)
        controller._apply_lens_controls(_camera_config(tmp_path))
    finally:
        monkeypatch.setattr(controller, "_ffmpeg_running", lambda: False)
        controller.close()

    set_calls = [call for call in calls if "--set-ctrl" in call]
    assert len(set_calls) == len(controls)
    assert all(call[1:3] == ["--device", "/dev/video-test"] for call in calls)
    assert [call[call.index("--set-ctrl") + 1] for call in set_calls[:3]] == [
        "auto_exposure=1",
        "exposure_time_absolute=25",
        "gain=200",
    ]
    assert set_calls[-1][-1] == "zoom_absolute=10"
    assert calls[-1][-1].startswith("--get-ctrl=auto_exposure,")
    assert sleeps == [1.0, 0.2, 0.2]


def test_start_sets_capture_mode_before_post_stream_lens_controls(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """格式由 FFmpeg 打开设备时确定，镜头参数只能在 RTSP 验证后应用。"""
    events: list[str] = []
    controller = CameraController(
        paths=_runtime_paths(tmp_path),
        initial_config=_camera_config(tmp_path),
        persist_config=False,
    )
    monkeypatch.setattr(
        controller, "_preflight", lambda _config: events.append("preflight")
    )
    monkeypatch.setattr(
        controller,
        "_write_mediamtx_config",
        lambda _config: events.append("write_mediamtx"),
    )
    monkeypatch.setattr(
        controller, "_start_mediamtx", lambda _config: events.append("mediamtx")
    )
    monkeypatch.setattr(
        controller,
        "_start_ffmpeg",
        lambda _config, _recording, **_kwargs: events.append("ffmpeg"),
    )
    monkeypatch.setattr(
        controller,
        "_wait_for_rtsp_stream",
        lambda _config: events.append("rtsp_ready"),
    )
    monkeypatch.setattr(
        controller,
        "_apply_lens_controls",
        lambda _config: events.append("lens"),
    )
    try:
        controller.start()
    finally:
        controller._state = "stopped"
        controller.close()

    assert events.index("preflight") < events.index("ffmpeg")
    assert events.index("ffmpeg") < events.index("rtsp_ready")
    assert events.index("rtsp_ready") < events.index("lens")


def test_snapshot_names_include_time_source_and_original_photo_no(tmp_path: Path) -> None:
    """三类 JPG 均含时间；甲方 photoNo 经可逆文件名转义后仍可恢复。"""
    controller = CameraController(paths=_runtime_paths(tmp_path))
    try:
        manual = controller._snapshot_filename(kind="manual", photo_no="")
        gcs = controller._snapshot_filename(kind="gcs", photo_no="")
        upstream = controller._snapshot_filename(
            kind="upstream", photo_no="巡检/原值 01"
        )
    finally:
        controller.close()

    assert manual.startswith("snapshot-") and manual.endswith("-manual-1.jpg")
    assert gcs.startswith("snapshot-") and gcs.endswith("-gcs-1.jpg")
    assert upstream.startswith("snapshot-") and upstream.endswith(".jpg")
    assert "photoNo-巡检/原值 01" in unquote(upstream)


def test_panel_modes_gate_commands_but_keep_manual_viewer_available(
    tmp_path: Path,
) -> None:
    """三模式只改变命令目标和本机配置门控，RTSP 查看器始终可用。"""
    application = _application()
    onboard = _FakeOnboardVideoClient()
    window = CameraPanelWindow(
        client=CameraServiceClient(paths=_runtime_paths(tmp_path)),
        onboard_client=onboard,
        auto_bootstrap=False,
    )
    config = _camera_config(tmp_path).to_dict()
    window._service_ready = True
    window._apply_status(
        {"state": "stopped", "running": False, "config": config, "rtsp_url": config["rtsp_url"]}
    )
    window._apply_probe(
        {
            "devices": [{"label": "Camera", "path": config["device"]}],
            "modes": [{"codec": "h264", "width": 1920, "height": 1080, "fps": 30.0}],
            "error": "",
        }
    )
    assert window.source_mode is SourceMode.LOCAL
    assert window.device_group.isEnabled()
    assert window.start_button.isEnabled()

    window.rtsp_url_input.clear()
    window._url_user_edited = False
    window.source_combo.setCurrentIndex(window.source_combo.findData(SourceMode.ONBOARD.value))
    application.processEvents()
    assert window.source_mode is SourceMode.ONBOARD
    assert not window.device_group.isEnabled()
    assert window.rtsp_url_input.text() == "rtsp://aircraft.test:8554/camera"
    assert window.rtsp_url_input.isEnabled()
    assert window.play_pause_button.isEnabled()
    window.start_button.click()
    application.processEvents()
    window.stop_button.click()
    application.processEvents()
    window.snapshot_button.click()
    application.processEvents()
    assert onboard.state_requests == [True, False]
    assert onboard.snapshot_requests == 1

    window.source_combo.setCurrentIndex(window.source_combo.findData(SourceMode.EXTERNAL.value))
    application.processEvents()
    assert not window.start_button.isEnabled()
    assert not window.stop_button.isEnabled()
    assert not window.snapshot_button.isEnabled()
    assert not window.storage_group.isEnabled()
    assert window.rtsp_url_input.isEnabled()

    window.close()
    assert onboard.closed == 1
    assert onboard.state_requests == [True, False]
    window.deleteLater()
    application.sendPostedEvents(None, QEvent.Type.DeferredDelete)
    application.processEvents()


def test_onboard_command_failure_is_shown_explicitly(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """真机启停端点缺失时不得只在底部静默显示错误。"""
    application = _application()
    window = CameraPanelWindow(
        client=CameraServiceClient(paths=_runtime_paths(tmp_path)),
        onboard_client=_FakeOnboardVideoClient(),
        auto_bootstrap=False,
    )
    shown: list[tuple[str, str]] = []
    monkeypatch.setattr(
        window, "_show_error", lambda title, message: shown.append((title, message))
    )

    window._action_busy = True
    window._on_request_completed(
        "onboard-start", {}, "独立机载视频状态接口尚未发现"
    )

    assert not window._action_busy
    assert window.operation_message.text() == "独立机载视频状态接口尚未发现"
    assert shown == [
        ("真机摄像头操作失败", "独立机载视频状态接口尚未发现")
    ]
    window.close()
    window.deleteLater()
    application.sendPostedEvents(None, QEvent.Type.DeferredDelete)
    application.processEvents()


def test_typed_rtsp_url_survives_config_poll_stop_and_backend_failure(
    tmp_path: Path,
) -> None:
    """推流字段和本机服务状态永远不能覆盖或停止手填查看器。"""
    application = _application()
    window = CameraPanelWindow(
        client=CameraServiceClient(paths=_runtime_paths(tmp_path)),
        auto_bootstrap=False,
    )
    custom = "rtsp://viewer.example:9554/manual"
    window.rtsp_url_input.setText(custom)
    window._mark_url_edited(custom)
    window.ip_input.setText("10.0.0.2")
    window.port_input.setValue(18554)
    window.path_input.setText("publisher")
    window._apply_status(
        {
            "state": "running",
            "running": True,
            "config": _camera_config(tmp_path).to_dict(),
            "rtsp_url": "rtsp://different.invalid/camera",
            "elapsed_seconds": 99,
            "frame": 999,
            "measured_fps": 88.0,
        }
    )
    player = window.player
    window._set_service_failure("本机后台离线")

    assert window.rtsp_url_input.text() == custom
    assert window.player is player
    assert window.elapsed_value.text() == "00:00:00"
    assert window.fps_value.text() == "0.0 fps"

    window.close()
    window.deleteLater()
    application.sendPostedEvents(None, QEvent.Type.DeferredDelete)
    application.processEvents()


def test_preview_icons_copy_validation_and_accessibility(tmp_path: Path) -> None:
    """播放/复制使用有效图标，复制原值，无效 scheme 不创建媒体源。"""
    application = _application()
    window = CameraPanelWindow(
        client=CameraServiceClient(paths=_runtime_paths(tmp_path)),
        auto_bootstrap=False,
    )
    assert not window.play_pause_button.icon().isNull()
    assert not window.copy_url_button.icon().isNull()
    assert window.play_pause_button.toolTip()
    assert window.play_pause_button.accessibleName()
    assert window.copy_url_button.toolTip()
    assert window.copy_url_button.accessibleName()
    assert not window.play_pause_button.text()
    assert not window.copy_url_button.text()

    window.rtsp_url_input.setText("http://example.test/not-rtsp")
    window._toggle_preview()
    assert not window.player.source().isValid()
    assert "rtsp://" in window.operation_message.text()
    valid = "rtsp://example.test:8554/path"
    window.rtsp_url_input.setText(valid)
    window._copy_rtsp_url()
    assert QApplication.clipboard().text() == valid

    window.close()
    window.deleteLater()
    application.sendPostedEvents(None, QEvent.Type.DeferredDelete)
    application.processEvents()


def test_playback_metrics_freeze_on_pause_and_ignore_service_metrics(
    tmp_path: Path,
) -> None:
    """统计只累计 Playing 区间及有效 sink 帧，暂停期间完全冻结。"""
    application = _application()
    window = CameraPanelWindow(
        client=CameraServiceClient(paths=_runtime_paths(tmp_path)),
        auto_bootstrap=False,
    )

    class PlaybackStub:
        state = QMediaPlayer.PlaybackState.StoppedState

        def playbackState(self):
            return self.state

    real_player = window.player
    stub = PlaybackStub()
    window.player = stub  # type: ignore[assignment]
    now = [10.0]
    window._clock = lambda: now[0]
    stub.state = QMediaPlayer.PlaybackState.PlayingState
    window._player_state_changed(stub.state)

    class Frame:
        def isValid(self) -> bool:
            return True

    for _ in range(20):
        window._count_video_frame(Frame())
    now[0] = 20.0
    window._refresh_playback_metrics()
    assert window.elapsed_value.text() == "00:00:10"
    assert window.fps_value.text() == "2.0 fps"

    stub.state = QMediaPlayer.PlaybackState.PausedState
    window._player_state_changed(stub.state)
    now[0] = 35.0
    window._refresh_playback_metrics()
    assert window.elapsed_value.text() == "00:00:10"
    assert window.fps_value.text() == "2.0 fps"

    window.player = real_player
    labels = {label.text() for label in window.findChildren(QLabel)}
    assert {"播放时长", "平均帧数"}.issubset(labels)
    assert {"当前录像", "已采集帧", "录制时长", "实测平均"}.isdisjoint(labels)
    window.close()
    window.deleteLater()
    application.sendPostedEvents(None, QEvent.Type.DeferredDelete)
    application.processEvents()


def test_every_panel_combo_and_spinbox_ignores_wheel(tmp_path: Path) -> None:
    """来源、设备、编码、模式、格式与端口都不会被滚轮误改。"""
    application = _application()
    window = CameraPanelWindow(
        client=CameraServiceClient(paths=_runtime_paths(tmp_path)),
        auto_bootstrap=False,
    )
    combos = window.findChildren(NoWheelComboBox)
    spins = window.findChildren(NoWheelSpinBox)
    assert len(combos) == 5
    assert spins == [window.port_input]

    for widget in [*combos, *spins]:
        before = widget.currentIndex() if isinstance(widget, NoWheelComboBox) else widget.value()
        event = QWheelEvent(
            QPointF(5, 5),
            QPointF(5, 5),
            QPoint(),
            QPoint(0, 120),
            Qt.MouseButton.NoButton,
            Qt.KeyboardModifier.NoModifier,
            Qt.ScrollPhase.ScrollUpdate,
            False,
        )
        widget.wheelEvent(event)
        after = widget.currentIndex() if isinstance(widget, NoWheelComboBox) else widget.value()
        assert not event.isAccepted()
        assert after == before

    window.close()
    window.deleteLater()
    application.sendPostedEvents(None, QEvent.Type.DeferredDelete)
    application.processEvents()
