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

# 必须在首次导入 Qt 前选择无显示平台，保证 CI 不依赖桌面会话。
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import pytest
from PySide6.QtCore import QEvent
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
from camera_app.panel import (
    DESKTOP_APPLICATION_NAME,
    CameraPanelWindow,
    PANEL_STYLE_SHEET,
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
        "setts=pts=N:dts=N:duration=1:time_base=1/30"
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
    assert command[command.index("-bsf:v") + 1].endswith("time_base=1/120")


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
    assert command[command.index("-force_duplicated_matrix") + 1] == "1"


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
    controller = CameraController(paths=_runtime_paths(tmp_path))
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
    """关闭独立面板只关预览；正在运行的后台状态不被改变。"""
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

    assert preview_urls == [config["rtsp_url"]]
    assert not window.codec_combo.isEnabled()
    assert not window.start_button.isEnabled()
    assert window.stop_button.isEnabled()
    assert window.snapshot_button.isEnabled()
    assert window.fps_value.text() == "30.0 fps"
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
    """停止或手动重连后必须换新播放器，不能复用已结束的FFmpeg会话。"""
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
    window._ensure_preview(url)
    assert window.player is second_player
    window._reconnect_preview()
    assert window.player is not second_player
    assert window.player.source().toString() == url

    window.close()
    window.deleteLater()
    application.sendPostedEvents(None, QEvent.Type.DeferredDelete)
    application.processEvents()
