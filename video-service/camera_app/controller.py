"""独立管理 MediaMTX、FFmpeg、录像封装、进度和按需 JPG 截图。"""

from __future__ import annotations

from datetime import datetime
import json
import os
from pathlib import Path
import shutil
import signal
import socket
import subprocess
import threading
import time
from typing import Any, TextIO

from .config import (
    BUNDLED_MEDIAMTX,
    RTP_JPEG_MAX_DIMENSION,
    CameraConfig,
    RuntimePaths,
    discover_camera_devices,
    load_config,
    probe_camera_modes,
    save_config,
)


# MediaMTX 官方对“reader is too slow”建议至少使用 1024 个出站包槽位。
MEDIAMTX_WRITE_QUEUE_SIZE = 1024

# RFC 2435 的常规 RTP/JPEG 只携带标准 Huffman 表；质量 2 保持预览清晰度。
RTP_MJPEG_QUALITY = 2


class CameraServiceError(RuntimeError):
    """摄像头服务无法完成请求时返回给 GUI/CLI 的明确错误。"""


class CameraController:
    """摄像头服务唯一状态机；不导入 ROS 或地面站进程管理器。"""

    _START_TIMEOUT_SECONDS = 8.0
    _STOP_TIMEOUT_SECONDS = 10.0
    _SNAPSHOT_TIMEOUT_SECONDS = 6.0

    def __init__(
        self,
        *,
        paths: RuntimePaths | None = None,
        mediamtx_binary: Path = BUNDLED_MEDIAMTX,
        ffmpeg_binary: str | None = None,
        ffprobe_binary: str | None = None,
    ) -> None:
        """加载持久配置并启动只在收到信号时工作的截图线程。"""
        self.paths = paths or RuntimePaths.discover()
        self.mediamtx_binary = Path(mediamtx_binary)
        self.ffmpeg_binary = ffmpeg_binary or shutil.which("ffmpeg") or "ffmpeg"
        self.ffprobe_binary = ffprobe_binary or shutil.which("ffprobe") or "ffprobe"

        self._state_lock = threading.RLock()
        self._operation_lock = threading.Lock()
        self._snapshot_lock = threading.Lock()
        self._state = "stopped"
        self._last_error = ""
        try:
            self._config = load_config(self.paths)
        except ValueError as exc:
            self._config = CameraConfig.defaults()
            self._last_error = str(exc)

        self._mediamtx: subprocess.Popen[str] | None = None
        self._ffmpeg: subprocess.Popen[str] | None = None
        self._mediamtx_log_stream: TextIO | None = None
        self._ffmpeg_log_stream: TextIO | None = None
        self._ffmpeg_monitor: threading.Thread | None = None
        self._stop_requested = False
        self._started_at: datetime | None = None
        self._started_monotonic: float | None = None
        self._partial_recording: Path | None = None
        self._current_recording: Path | None = None
        self._last_recording: Path | None = None
        self._last_snapshot: Path | None = None
        self._progress: dict[str, str] = {}
        self._progress_baseline_media: float | None = None
        self._progress_baseline_monotonic: float | None = None
        self._normalize_mjpeg_for_rtsp = False

        self._snapshot_requested = threading.Event()
        self._shutdown_requested = threading.Event()
        self._snapshot_worker = threading.Thread(
            target=self._snapshot_worker_loop,
            name="camera-service-snapshot",
            daemon=True,
        )
        self._snapshot_worker.start()

    @property
    def config(self) -> CameraConfig:
        """返回不可变配置对象，调用方无法绕过校验原地修改。"""
        with self._state_lock:
            return self._config

    def configure(self, raw: dict[str, Any]) -> dict[str, Any]:
        """仅在摄像头停止时验证并原子保存完整或部分配置。"""
        with self._operation_lock:
            with self._state_lock:
                if self._state in {"starting", "running", "stopping"}:
                    raise CameraServiceError("摄像头运行期间不能修改配置，请先关闭")
                merged = self._config.to_dict()
                merged.update(raw)
            config = CameraConfig.from_dict(merged)
            save_config(config, self.paths)
            with self._state_lock:
                self._config = config
                self._last_error = ""
                if self._state == "error":
                    self._state = "stopped"
            return config.to_dict()

    def probe(self, device: str | None = None) -> dict[str, Any]:
        """枚举视频采集节点，并返回指定设备声明的离散模式。"""
        devices = discover_camera_devices()
        selected = device or self.config.device
        modes: list[dict[str, Any]] = []
        excluded_modes: list[dict[str, Any]] = []
        error = ""
        try:
            for item in probe_camera_modes(selected):
                target = (
                    excluded_modes
                    if item.codec == "mjpeg"
                    and (
                        item.width > RTP_JPEG_MAX_DIMENSION
                        or item.height > RTP_JPEG_MAX_DIMENSION
                    )
                    else modes
                )
                target.append(item.to_dict())
        except (OSError, RuntimeError) as exc:
            error = str(exc)
        return {
            "devices": devices,
            "selected_device": selected,
            "modes": modes,
            "excluded_modes": excluded_modes,
            "error": error,
        }

    def start(self, raw_config: dict[str, Any] | None = None) -> dict[str, Any]:
        """启动独立 MediaMTX，并按帧格式选择零转码或兼容发布路径。"""
        with self._operation_lock:
            with self._state_lock:
                if self._state in {"starting", "running"}:
                    return self.status()
                if self._state == "stopping":
                    raise CameraServiceError("摄像头正在关闭，请稍后重试")
                if raw_config is not None:
                    merged = self._config.to_dict()
                    merged.update(raw_config)
                    config = CameraConfig.from_dict(merged)
                else:
                    config = self._config
                self._state = "starting"
                self._last_error = ""
                self._stop_requested = False
                self._progress = {}
                self._progress_baseline_media = None
                self._progress_baseline_monotonic = None
                self._started_at = None
                self._started_monotonic = None
                self._current_recording = None
                self._partial_recording = None
                self._normalize_mjpeg_for_rtsp = False

            try:
                self._preflight(config)
                normalize_mjpeg_for_rtsp = (
                    config.codec == "mjpeg"
                    and self._mjpeg_has_restart_interval(config)
                )
                save_config(config, self.paths)
                with self._state_lock:
                    # 启动失败时仍保留用户刚提交的有效配置，便于面板修正后重试。
                    self._config = config
                    self._normalize_mjpeg_for_rtsp = normalize_mjpeg_for_rtsp
                self.paths.ensure_directories()
                video_directory = Path(config.video_directory).expanduser()
                image_directory = Path(config.image_directory).expanduser()
                video_directory.mkdir(parents=True, exist_ok=True)
                image_directory.mkdir(parents=True, exist_ok=True)

                self._write_mediamtx_config(config)
                self._start_mediamtx(config)
                final_recording, partial_recording = self._recording_paths(config)
                with self._state_lock:
                    # 从采集进程拉起前计时，包含RTSP就绪探测期间已采集的帧。
                    self._started_at = datetime.now()
                    self._started_monotonic = time.monotonic()
                self._start_ffmpeg(
                    config,
                    partial_recording,
                    normalize_mjpeg_for_rtsp=normalize_mjpeg_for_rtsp,
                )
                self._wait_for_rtsp_stream(config)
            except Exception as exc:
                self._stop_processes(finalize_recording=False)
                message = str(exc) or exc.__class__.__name__
                with self._state_lock:
                    self._state = "error"
                    self._last_error = message
                    self._started_at = None
                    self._started_monotonic = None
                    self._current_recording = None
                    self._partial_recording = None
                    self._normalize_mjpeg_for_rtsp = False
                raise CameraServiceError(message) from exc

            with self._state_lock:
                self._config = config
                self._current_recording = final_recording
                self._partial_recording = partial_recording
                self._state = "running"
            return self.status()

    def stop(self) -> dict[str, Any]:
        """幂等停止视频链路，先让 FFmpeg封装录像，再停止RTSP服务。"""
        with self._operation_lock:
            with self._state_lock:
                if self._state == "stopped" and not self._has_live_processes():
                    return self.status()
                self._state = "stopping"
                self._stop_requested = True

            self._stop_processes(finalize_recording=True)
            with self._state_lock:
                self._state = "stopped"
                self._last_error = ""
                self._started_at = None
                self._started_monotonic = None
                self._current_recording = None
                self._partial_recording = None
                self._progress = {}
                self._progress_baseline_media = None
                self._progress_baseline_monotonic = None
                self._normalize_mjpeg_for_rtsp = False
            return self.status()

    def request_snapshot(self) -> dict[str, Any]:
        """同步从本机 RTSP解码一帧并原子保存为带时间戳的JPG。"""
        with self._snapshot_lock:
            with self._state_lock:
                if self._state != "running" or not self._ffmpeg_running():
                    raise CameraServiceError("摄像头未运行，无法保存图片")
                config = self._config

            destination_dir = Path(config.image_directory).expanduser()
            destination_dir.mkdir(parents=True, exist_ok=True)
            stamp = datetime.now().strftime("%Y%m%d-%H%M%S-%f")
            destination = destination_dir / f"snapshot-{stamp}.jpg"
            temporary = destination_dir / f".snapshot-{stamp}.tmp.jpg"
            command = self.build_snapshot_command(config, temporary)
            try:
                result = subprocess.run(
                    command,
                    check=False,
                    text=True,
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.PIPE,
                    timeout=self._SNAPSHOT_TIMEOUT_SECONDS,
                    start_new_session=True,
                )
            except subprocess.TimeoutExpired as exc:
                self._remove_runtime_file(temporary)
                raise CameraServiceError("保存JPG超时") from exc
            if result.returncode != 0 or not temporary.is_file():
                self._remove_runtime_file(temporary)
                detail = self._last_nonempty_line(result.stderr)
                raise CameraServiceError(f"保存JPG失败：{detail}")
            os.replace(temporary, destination)
            with self._state_lock:
                self._last_snapshot = destination
            return {"path": str(destination)}

    def signal_snapshot(self) -> None:
        """供 SIGUSR1处理器使用：只设置事件，不在信号回调中执行I/O。"""
        self._snapshot_requested.set()

    def status(self) -> dict[str, Any]:
        """返回 GUI轮询所需的纯数据快照，不阻塞视频进程。"""
        with self._state_lock:
            elapsed = 0.0
            if self._started_monotonic is not None:
                elapsed = max(0.0, time.monotonic() - self._started_monotonic)
            frame = self._integer_progress("frame")
            media_elapsed = self._integer_progress("out_time_us") / 1_000_000.0
            if frame <= 0 and media_elapsed > 0.0:
                # FFmpeg stream-copy 时 frame 恒为 0；媒体时间由逐包 setts 生成，
                # 因而可稳定还原已处理包数，同时仍用墙钟时间衡量实际吞吐。
                frame = int(round(media_elapsed * self._config.fps))
            measured_fps = 0.0
            if (
                self._progress_baseline_media is not None
                and self._progress_baseline_monotonic is not None
                and media_elapsed > self._progress_baseline_media
            ):
                sample_elapsed = max(
                    0.0, time.monotonic() - self._progress_baseline_monotonic
                )
                if sample_elapsed > 0.2:
                    media_delta = media_elapsed - self._progress_baseline_media
                    measured_fps = media_delta * self._config.fps / sample_elapsed
            elif frame and elapsed > 0.2:
                measured_fps = frame / elapsed
            return {
                "state": self._state,
                "running": self._state == "running" and self._ffmpeg_running(),
                "last_error": self._last_error,
                "config": self._config.to_dict(),
                "rtsp_url": self._config.rtsp_url,
                "local_rtsp_url": self._config.local_rtsp_url,
                "recording_file": (
                    str(self._current_recording) if self._current_recording else ""
                ),
                "partial_recording_file": (
                    str(self._partial_recording) if self._partial_recording else ""
                ),
                "last_recording_file": (
                    str(self._last_recording) if self._last_recording else ""
                ),
                "last_snapshot_file": (
                    str(self._last_snapshot) if self._last_snapshot else ""
                ),
                "started_at": (
                    self._started_at.isoformat(timespec="seconds")
                    if self._started_at
                    else ""
                ),
                "elapsed_seconds": elapsed,
                "media_elapsed_seconds": media_elapsed,
                "frame": frame,
                "measured_fps": measured_fps,
                "bitrate": self._progress.get("bitrate", ""),
                "speed": self._progress.get("speed", ""),
                "ffmpeg_pid": self._ffmpeg.pid if self._ffmpeg_running() else 0,
                "mediamtx_pid": (
                    self._mediamtx.pid if self._mediamtx_running() else 0
                ),
                "service_pid": os.getpid(),
                "timestamp_policy": "fixed-cfr-setts",
                "rtsp_mjpeg_normalization": self._normalize_mjpeg_for_rtsp,
            }

    def close(self) -> None:
        """关闭服务进程拥有的所有资源；可被信号和正常退出重复调用。"""
        try:
            self.stop()
        finally:
            self._shutdown_requested.set()
            self._snapshot_requested.set()
            if self._snapshot_worker.is_alive():
                self._snapshot_worker.join(timeout=1.0)

    def build_ffmpeg_command(
        self,
        config: CameraConfig,
        partial_recording: Path,
        *,
        normalize_mjpeg_for_rtsp: bool = False,
    ) -> list[str]:
        """构造单次采集命令；录像始终保留摄像头原始压缩码流。"""
        codec_input = "h264" if config.codec == "h264" else "mjpeg"
        frame_rate = self._format_fps_argument(config.fps)
        timestamp_filter = (
            "setts=pts=N:dts=N:duration=1:"
            f"time_base={config.ffmpeg_time_base}"
        )
        rtsp_output = (
            "[f=rtsp:rtsp_transport=tcp:onfail=abort]"
            f"{config.local_rtsp_url}"
        )
        if config.container == "mp4":
            record_options = (
                "f=mp4:movflags=+frag_keyframe+empty_moov+default_base_moof:"
                "onfail=abort"
            )
        elif config.container == "mkv":
            record_options = "f=matroska:onfail=abort"
        else:
            record_options = "f=avi:onfail=abort"
        record_output = f"[{record_options}]{partial_recording}"
        common = [
            self.ffmpeg_binary,
            "-hide_banner",
            "-loglevel",
            "warning",
            "-nostats",
            "-thread_queue_size",
            "512",
            "-f",
            "v4l2",
            "-input_format",
            codec_input,
            "-video_size",
            f"{config.width}x{config.height}",
            "-framerate",
            frame_rate,
            "-i",
            config.device,
        ]
        if normalize_mjpeg_for_rtsp:
            # 部分 UVC MJPEG 帧含 DRI；FFmpeg 6/7 的 RTP/JPEG 发送端会丢失
            # restart interval。只重编码 RTSP 分支，录像仍然 stream-copy。
            record_muxer = {
                "mp4": [
                    "-f",
                    "mp4",
                    "-movflags",
                    "+frag_keyframe+empty_moov+default_base_moof",
                ],
                "mkv": ["-f", "matroska"],
                "avi": ["-f", "avi"],
            }[config.container]
            return common + [
                "-progress",
                "pipe:1",
                "-map",
                "0:v:0",
                "-an",
                "-c:v",
                "copy",
                "-bsf:v",
                timestamp_filter,
                *record_muxer,
                str(partial_recording),
                "-map",
                "0:v:0",
                "-an",
                "-c:v",
                "mjpeg",
                "-pix_fmt",
                "yuvj420p",
                "-q:v",
                str(RTP_MJPEG_QUALITY),
                "-huffman",
                "default",
                "-force_duplicated_matrix",
                "1",
                "-bsf:v",
                timestamp_filter,
                "-f",
                "rtsp",
                "-rtsp_transport",
                "tcp",
                config.local_rtsp_url,
            ]
        return common + [
            "-map",
            "0:v:0",
            "-an",
            "-c:v",
            "copy",
            "-bsf:v",
            timestamp_filter,
            "-progress",
            "pipe:1",
            "-f",
            "tee",
            f"{rtsp_output}|{record_output}",
        ]

    def _mjpeg_has_restart_interval(self, config: CameraConfig) -> bool:
        """读取一帧 MJPEG 头，判断是否需绕过 FFmpeg 的 RTP/DRI 缺陷。"""
        command = [
            self.ffmpeg_binary,
            "-nostdin",
            "-hide_banner",
            "-loglevel",
            "error",
            "-f",
            "v4l2",
            "-input_format",
            "mjpeg",
            "-video_size",
            f"{config.width}x{config.height}",
            "-framerate",
            self._format_fps_argument(config.fps),
            "-i",
            config.device,
            "-map",
            "0:v:0",
            "-frames:v",
            "1",
            "-c:v",
            "copy",
            "-f",
            "image2pipe",
            "pipe:1",
        ]
        try:
            result = subprocess.run(
                command,
                check=False,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                timeout=self._START_TIMEOUT_SECONDS,
            )
        except subprocess.TimeoutExpired as exc:
            raise CameraServiceError("读取MJPEG兼容性信息超时") from exc
        frame = result.stdout
        scan_header = frame.find(b"\xff\xda")
        if result.returncode != 0 or scan_header < 0:
            detail = self._last_nonempty_line(
                result.stderr.decode("utf-8", errors="replace")
            )
            raise CameraServiceError(
                "无法读取MJPEG兼容性信息" + (f"：{detail}" if detail else "")
            )
        return b"\xff\xdd" in frame[:scan_header]

    def build_snapshot_command(
        self, config: CameraConfig, destination: Path
    ) -> list[str]:
        """构造低探测延迟、仅解码首个关键帧的 JPG截图命令。"""
        return [
            self.ffmpeg_binary,
            "-hide_banner",
            "-loglevel",
            "error",
            "-rtsp_transport",
            "tcp",
            "-analyzeduration",
            "0",
            "-probesize",
            "32768",
            "-fflags",
            "nobuffer",
            "-skip_frame",
            "nokey",
            "-i",
            config.local_rtsp_url,
            "-map",
            "0:v:0",
            "-frames:v",
            "1",
            "-fps_mode",
            "passthrough",
            "-q:v",
            "2",
            "-y",
            str(destination),
        ]

    def _preflight(self, config: CameraConfig) -> None:
        """在启动任何子进程前验证依赖、设备模式、端口与目录。"""
        config.validate()
        if config.codec == "mjpeg" and (
            config.width > RTP_JPEG_MAX_DIMENSION
            or config.height > RTP_JPEG_MAX_DIMENSION
        ):
            raise CameraServiceError(
                "RTSP/JPEG受RFC 2435限制，宽和高都不能超过"
                f" {RTP_JPEG_MAX_DIMENSION} 像素；请选择1080p或更低模式"
            )
        if not self.mediamtx_binary.is_file() or not os.access(
            self.mediamtx_binary, os.X_OK
        ):
            raise CameraServiceError(f"MediaMTX不可执行：{self.mediamtx_binary}")
        for name, executable in (
            ("FFmpeg", self.ffmpeg_binary),
            ("FFprobe", self.ffprobe_binary),
        ):
            if not Path(executable).is_file() and shutil.which(executable) is None:
                raise CameraServiceError(f"未找到{name}：{executable}")
        device = Path(config.device)
        if not device.exists():
            raise CameraServiceError(f"摄像头设备不存在：{config.device}")

        modes = probe_camera_modes(config.device)
        supported = any(
            mode.codec == config.codec
            and mode.width == config.width
            and mode.height == config.height
            and abs(mode.fps - config.fps) < 0.02
            for mode in modes
        )
        if not supported:
            raise CameraServiceError(
                "摄像头不支持所选组合："
                f"{config.codec} {config.width}×{config.height} @ {config.fps:g} fps"
            )
        if self._port_accepting_connections(config.rtsp_port):
            raise CameraServiceError(f"RTSP端口 {config.rtsp_port} 已被其他服务占用")

    def _write_mediamtx_config(self, config: CameraConfig) -> None:
        """生成只开启RTSP/TCP且只允许本机发布的最小MediaMTX配置。"""
        path_key = json.dumps(config.normalized_rtsp_path)
        text = (
            "# Camera service generated MediaMTX v1.20.0 configuration.\n"
            "logLevel: info\n"
            "logDestinations: [stdout]\n"
            f"writeQueueSize: {MEDIAMTX_WRITE_QUEUE_SIZE}\n"
            "authMethod: internal\n"
            "authInternalUsers:\n"
            "  - user: any\n"
            "    pass:\n"
            "    ips: [\"127.0.0.1\", \"::1\"]\n"
            "    permissions:\n"
            "      - action: publish\n"
            f"        path: {path_key}\n"
            "  - user: any\n"
            "    pass:\n"
            "    ips: []\n"
            "    permissions:\n"
            "      - action: read\n"
            f"        path: {path_key}\n"
            "api: false\n"
            "metrics: false\n"
            "pprof: false\n"
            "playback: false\n"
            "rtsp: true\n"
            "rtspTransports: [tcp]\n"
            f"rtspAddress: \"0.0.0.0:{config.rtsp_port}\"\n"
            "rtmp: false\n"
            "hls: false\n"
            "webrtc: false\n"
            "srt: false\n"
            "moq: false\n"
            "pathDefaults:\n"
            "  source: publisher\n"
            "  overridePublisher: false\n"
            "paths:\n"
            f"  {path_key}:\n"
        )
        temporary = self.paths.mediamtx_config.with_suffix(".yml.tmp")
        temporary.write_text(text, encoding="utf-8")
        os.chmod(temporary, 0o600)
        os.replace(temporary, self.paths.mediamtx_config)

    def _start_mediamtx(self, config: CameraConfig) -> None:
        """启动当前服务私有的MediaMTX并等待TCP监听就绪。"""
        self._mediamtx_log_stream = self.paths.mediamtx_log.open(
            "a", encoding="utf-8", buffering=1
        )
        self._write_log_header(self._mediamtx_log_stream, "MediaMTX")
        self._mediamtx = subprocess.Popen(
            [str(self.mediamtx_binary), str(self.paths.mediamtx_config)],
            stdin=subprocess.DEVNULL,
            stdout=self._mediamtx_log_stream,
            stderr=subprocess.STDOUT,
            text=True,
            start_new_session=True,
        )
        deadline = time.monotonic() + self._START_TIMEOUT_SECONDS
        while time.monotonic() < deadline:
            if self._mediamtx.poll() is not None:
                detail = self._tail_log(self.paths.mediamtx_log)
                raise CameraServiceError(f"MediaMTX启动失败：{detail}")
            if self._port_accepting_connections(config.rtsp_port):
                return
            time.sleep(0.05)
        raise CameraServiceError("MediaMTX启动超时")

    def _start_ffmpeg(
        self,
        config: CameraConfig,
        partial_recording: Path,
        *,
        normalize_mjpeg_for_rtsp: bool = False,
    ) -> None:
        """启动FFmpeg并用独立线程解析机器可读进度。"""
        self._ffmpeg_log_stream = self.paths.ffmpeg_log.open(
            "a", encoding="utf-8", buffering=1
        )
        self._write_log_header(self._ffmpeg_log_stream, "FFmpeg")
        command = self.build_ffmpeg_command(
            config,
            partial_recording,
            normalize_mjpeg_for_rtsp=normalize_mjpeg_for_rtsp,
        )
        self._ffmpeg_log_stream.write("command: " + " ".join(command) + "\n")
        self._ffmpeg = subprocess.Popen(
            command,
            stdin=subprocess.DEVNULL,
            stdout=subprocess.PIPE,
            stderr=self._ffmpeg_log_stream,
            text=True,
            bufsize=1,
            start_new_session=True,
        )
        self._partial_recording = partial_recording
        self._ffmpeg_monitor = threading.Thread(
            target=self._monitor_ffmpeg,
            args=(self._ffmpeg,),
            name="camera-service-ffmpeg-monitor",
            daemon=True,
        )
        self._ffmpeg_monitor.start()

    def _wait_for_rtsp_stream(self, config: CameraConfig) -> None:
        """用FFprobe验证发布端真正可被RTSP客户端读取。"""
        deadline = time.monotonic() + self._START_TIMEOUT_SECONDS
        last_detail = ""
        while time.monotonic() < deadline:
            if not self._ffmpeg_running():
                detail = self._tail_log(self.paths.ffmpeg_log)
                raise CameraServiceError(f"FFmpeg启动失败：{detail}")
            command = [
                self.ffprobe_binary,
                "-v",
                "error",
                "-rtsp_transport",
                "tcp",
                "-select_streams",
                "v:0",
                "-show_entries",
                "stream=codec_name,width,height",
                "-of",
                "json",
                config.local_rtsp_url,
            ]
            try:
                result = subprocess.run(
                    command,
                    check=False,
                    text=True,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    timeout=1.5,
                )
            except subprocess.TimeoutExpired:
                last_detail = "RTSP探测超时"
                continue
            if result.returncode == 0:
                try:
                    streams = json.loads(result.stdout).get("streams", [])
                except json.JSONDecodeError:
                    streams = []
                if streams:
                    return
            last_detail = self._last_nonempty_line(result.stderr)
            time.sleep(0.1)
        raise CameraServiceError(f"RTSP流未在时限内就绪：{last_detail}")

    def _recording_paths(self, config: CameraConfig) -> tuple[Path, Path]:
        """为每次启动生成不会覆盖旧文件的最终与录制中路径。"""
        directory = Path(config.video_directory).expanduser()
        stamp = datetime.now().strftime("%Y%m%d-%H%M%S-%f")
        final = directory / f"recording-{stamp}.{config.container}"
        partial = directory / f"recording-{stamp}.partial.{config.container}"
        return final, partial

    def _stop_processes(self, *, finalize_recording: bool) -> None:
        """按FFmpeg INT→TERM→KILL、MediaMTX TERM→KILL顺序精确清理。"""
        ffmpeg = self._ffmpeg
        if ffmpeg is not None:
            self._terminate_process(
                ffmpeg,
                first_signal=signal.SIGINT,
                timeout=self._STOP_TIMEOUT_SECONDS,
            )
        if self._ffmpeg_monitor is not None and self._ffmpeg_monitor.is_alive():
            self._ffmpeg_monitor.join(timeout=1.0)

        if finalize_recording:
            self._finalize_recording()
        mediamtx = self._mediamtx
        if mediamtx is not None:
            self._terminate_process(
                mediamtx,
                first_signal=signal.SIGTERM,
                timeout=3.0,
            )

        self._ffmpeg = None
        self._mediamtx = None
        self._ffmpeg_monitor = None
        self._close_log_streams()

    def _finalize_recording(self) -> None:
        """FFmpeg正常退出后把可恢复的partial文件原子命名为最终录像。"""
        partial = self._partial_recording
        final = self._current_recording
        if partial is None or final is None or not partial.is_file():
            return
        try:
            if partial.stat().st_size <= 0:
                return
            os.replace(partial, final)
        except OSError as exc:
            with self._state_lock:
                self._last_error = f"录像完成但重命名失败：{exc}"
            return
        with self._state_lock:
            self._last_recording = final

    def _monitor_ffmpeg(self, process: subprocess.Popen[str]) -> None:
        """解析-progress输出，并在非主动退出时准确标记服务错误。"""
        output = process.stdout
        if output is not None:
            for line in output:
                key, separator, value = line.strip().partition("=")
                if separator and key:
                    with self._state_lock:
                        self._progress[key] = value
                        if (
                            key == "out_time_us"
                            and self._progress_baseline_media is None
                        ):
                            try:
                                media_seconds = int(value) / 1_000_000.0
                            except ValueError:
                                media_seconds = 0.0
                            if media_seconds > 0.0:
                                self._progress_baseline_media = media_seconds
                                self._progress_baseline_monotonic = time.monotonic()
        return_code = process.wait()
        with self._state_lock:
            if process is not self._ffmpeg or self._stop_requested:
                return
            if self._state in {"starting", "running"}:
                detail = self._tail_log(self.paths.ffmpeg_log)
                self._state = "error"
                self._last_error = (
                    f"FFmpeg意外退出（{return_code}）：{detail}"
                )

    def _snapshot_worker_loop(self) -> None:
        """把SIGUSR1事件转为普通线程内截图，连续信号自动合并。"""
        while not self._shutdown_requested.is_set():
            self._snapshot_requested.wait(timeout=0.5)
            if not self._snapshot_requested.is_set():
                continue
            self._snapshot_requested.clear()
            if self._shutdown_requested.is_set():
                break
            try:
                self.request_snapshot()
            except CameraServiceError as exc:
                with self._state_lock:
                    self._last_error = str(exc)

    def _terminate_process(
        self,
        process: subprocess.Popen[str],
        *,
        first_signal: signal.Signals,
        timeout: float,
    ) -> None:
        """只向本控制器创建的进程组发送有界升级信号。"""
        if process.poll() is not None:
            return
        try:
            os.killpg(process.pid, first_signal)
            process.wait(timeout=timeout)
            return
        except (ProcessLookupError, subprocess.TimeoutExpired):
            pass
        try:
            os.killpg(process.pid, signal.SIGTERM)
            process.wait(timeout=2.0)
            return
        except (ProcessLookupError, subprocess.TimeoutExpired):
            pass
        try:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait(timeout=1.0)
        except (ProcessLookupError, subprocess.TimeoutExpired):
            pass

    def _has_live_processes(self) -> bool:
        """返回本控制器拥有的任一子进程是否仍存活。"""
        return self._ffmpeg_running() or self._mediamtx_running()

    def _ffmpeg_running(self) -> bool:
        """检查当前FFmpeg对象而不扫描系统进程。"""
        return self._ffmpeg is not None and self._ffmpeg.poll() is None

    def _mediamtx_running(self) -> bool:
        """检查当前MediaMTX对象而不碰用户其他服务。"""
        return self._mediamtx is not None and self._mediamtx.poll() is None

    @staticmethod
    def _port_accepting_connections(port: int) -> bool:
        """用回环TCP连接判断端口是否已有监听。"""
        try:
            with socket.create_connection(("127.0.0.1", port), timeout=0.15):
                return True
        except OSError:
            return False

    @staticmethod
    def _format_fps_argument(fps: float) -> str:
        """生成FFmpeg接受且不引入无意义小数的帧率参数。"""
        return str(int(fps)) if float(fps).is_integer() else f"{fps:g}"

    def _integer_progress(self, key: str) -> int:
        """容错读取FFmpeg进度整数值。"""
        try:
            return int(self._progress.get(key, "0"))
        except ValueError:
            return 0

    @staticmethod
    def _write_log_header(stream: TextIO, component: str) -> None:
        """在追加日志中建立清晰的每次启动边界。"""
        timestamp = datetime.now().isoformat(timespec="seconds")
        stream.write(f"\n--- {component} started {timestamp} ---\n")

    @staticmethod
    def _tail_log(path: Path, limit: int = 8) -> str:
        """返回日志末尾非空行，避免把整份外部进程日志送入GUI。"""
        try:
            content = path.read_text(encoding="utf-8", errors="replace")
            lines = [line.strip() for line in content.splitlines() if line.strip()]
        except OSError:
            return "无日志"
        return " | ".join(lines[-limit:]) or "无日志"

    @staticmethod
    def _last_nonempty_line(text: str) -> str:
        """提取命令错误输出中最后一条有效信息。"""
        lines = [line.strip() for line in text.splitlines() if line.strip()]
        return lines[-1] if lines else "无详细信息"

    @staticmethod
    def _remove_runtime_file(path: Path) -> None:
        """只清理由本次截图明确创建的临时文件。"""
        try:
            path.unlink(missing_ok=True)
        except OSError:
            pass

    def _close_log_streams(self) -> None:
        """子进程退出后关闭本控制器持有的追加日志句柄。"""
        for attribute in ("_ffmpeg_log_stream", "_mediamtx_log_stream"):
            stream = getattr(self, attribute)
            if stream is not None:
                try:
                    stream.close()
                except OSError:
                    pass
                setattr(self, attribute, None)
