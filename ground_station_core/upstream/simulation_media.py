"""独立仿真媒体：循环发布样例 RTSP，并返回现有文件的 FTP 虚拟路径。

默认读取 config/ground/simulation_media.json，example 仅为模板。
不启动媒体服务、不去畸变，也不把样例视频宣称为本轮飞行录像。
"""

from __future__ import annotations

import json
import os
from pathlib import Path
import random
import subprocess
import threading
from tempfile import TemporaryDirectory
from urllib.parse import urlsplit

from ..event_log import EventLog

# 相对配置路径以仓库为基准，不受 GUI 的工作目录影响。
_PROJECT = Path(__file__).resolve().parents[2]
# 只枚举常见视频容器，避免误报临时文件与元数据。
_VIDEO_SUFFIXES = {".mp4", ".mkv", ".mov", ".avi", ".m4v", ".ts"}
# 照片上报允许现有 JPG 与其他常见静态图片格式。
_PICTURE_SUFFIXES = {".jpg", ".jpeg", ".png", ".webp", ".bmp"}
# 发布失败稍后尝试下一片段；取消事件可立即打断等待。
_RETRY_SECONDS = 2.0
# 探测本地媒体允许较宽松超时；真正播放不限制片长。
_PROBE_SECONDS = 15.0


def _path(value: str | Path, base: Path | None = None) -> Path:
    """展开用户目录、环境变量与相对路径。"""
    path = Path(os.path.expandvars(str(value))).expanduser()
    return (path if path.is_absolute() else (base or _PROJECT) / path).resolve()


class SimulationMedia:
    """仅由父任务控制启停，不依赖飞控、ROS 或 GUI。"""

    def __init__(self, event_log: EventLog, config_path: str | Path | None = None) -> None:
        """读取配置；不启动线程、进程或网络连接。"""
        self._events = event_log
        self._condition = threading.Condition()
        self._thread: threading.Thread | None = None
        self._process: subprocess.Popen | None = None
        self._cancel = threading.Event()
        self._running = False
        self._closed = False
        media = self._read_config(_path(os.environ.get(
            "GROUND_MEDIA_CONFIG", "config/ground/media.json",
        )))
        config = self._read_config(_path(config_path or "config/ground/simulation_media.json"))
        # root 与 ground-media 语义相同；public_root 可直接覆盖 FTP 根。
        root = _path(config.get("root", media.get("root", "~/Media/ardupilot")))
        self.public_root = _path(config.get("public_root", root / "public"))
        self.videos_dir = _path(config.get("videos_dir", "examples/videos"), self.public_root)
        self.pictures_dir = _path(config.get("pictures_dir", "examples/pictures"), self.public_root)
        port = config.get("rtsp_port", media.get("rtsp_port", 8554))
        self.rtsp_url = str(config.get("rtsp_url") or f"rtsp://127.0.0.1:{port}/uav002/rectified")

    def _read_config(self, path: Path) -> dict:
        """配置缺省使用默认值，损坏时仅告警，不影响地面站其他功能。"""
        try:
            value = json.loads(path.read_text(encoding="utf-8"))
            if not isinstance(value, dict):
                raise ValueError("配置顶层必须是 JSON 对象")
            return value
        except FileNotFoundError:
            return {}
        except (OSError, ValueError) as exc:
            self._events.warn("simulation-media", f"配置读取失败 {path}：{exc}")
            return {}

    def start(self) -> None:
        """非阻塞开始循环发布；重复调用不重复创建发布者。"""
        with self._condition:
            if self._closed or self._running:
                return
            self._cancel = threading.Event()
            self._running = True
            if self._thread is None:
                self._thread = threading.Thread(
                    target=self._run, name="simulation-media", daemon=True,
                )
                self._thread.start()
            self._condition.notify_all()

    def stop(self) -> None:
        """立即取消并杀掉自己的探测或发布进程，不在 GUI 线程等待。"""
        with self._condition:
            self._running = False
            self._cancel.set()
            self._kill_process()

    def close(self) -> None:
        """退出时取消并等待工作线程回收；关闭后不能再次启动。"""
        with self._condition:
            self._closed = True
            self._running = False
            self._cancel.set()
            self._kill_process()
            self._condition.notify_all()
            thread = self._thread
        if thread is not None and thread is not threading.current_thread():
            thread.join()

    def _kill_process(self) -> None:
        """在状态锁内只操作已持有的子进程，绝不按名称或端口杀进程。"""
        if self._process is not None:
            try:
                self._process.kill()
            except ProcessLookupError:
                pass

    def _files(self, directory: Path, suffixes: set[str]) -> list[Path]:
        """仅选取 FTP 根内非空现有文件，排除越界软链接。"""
        try:
            directory.relative_to(self.public_root)
            files = []
            for candidate in sorted(directory.rglob("*")):
                if candidate.suffix.lower() not in suffixes:
                    continue
                try:
                    path = candidate.resolve(strict=True)
                    path.relative_to(self.public_root)
                    if path.is_file() and path.stat().st_size:
                        files.append(path)
                except (OSError, ValueError, RuntimeError):
                    continue
            return files
        except (OSError, ValueError) as exc:
            self._events.warn("simulation-media", f"无法读取公开媒体目录 {directory}：{exc}")
            return []

    def _ftp_path(self, path: Path) -> str:
        """转换已检查的本地文件为 FTP 虚拟根路径，不添加主机或协议。"""
        return "/" + "/".join(path.relative_to(self.public_root).parts)

    def photo_path(self) -> str:
        """随机选择一张现有图片；没有图片时仅告警并返回空字符串。"""
        pictures = self._files(self.pictures_dir, _PICTURE_SUFFIXES)
        if not pictures:
            self._events.warn("simulation-media", f"没有仿真图片：{self.pictures_dir}")
            return ""
        return self._ftp_path(random.choice(pictures))

    def completion_payload(self) -> dict:
        """返回样例视频文件与非空图片目录的 FTP 路径；缺失项为空。"""
        videos = self._files(self.videos_dir, _VIDEO_SUFFIXES)
        pictures = self._files(self.pictures_dir, _PICTURE_SUFFIXES)
        if not videos or not pictures:
            self._events.warn("simulation-media", "仿真交付资源不完整，缺失项返回空路径")
        return {
            "videoPath": self._ftp_path(random.choice(videos)) if videos else "",
            "JPGPath": self._ftp_path(self.pictures_dir).rstrip("/") + "/" if pictures else "",
        }

    def _execute(
        self, args: list[str], cancel: threading.Event, *, probe: bool = False,
    ) -> tuple[int, str, str]:
        """后台执行一个自有进程；覆盖 stop 与 Popen 交错时的取消窗口。"""
        if cancel.is_set():
            return -1, "", ""
        process = subprocess.Popen(
            args, stdin=subprocess.DEVNULL,
            stdout=subprocess.PIPE if probe else subprocess.DEVNULL,
            stderr=subprocess.PIPE, text=True, encoding="utf-8", errors="replace",
        )
        with self._condition:
            self._process = process
            # stop 可能发生在进程创建期间，旧轮次不能逃逸或覆盖新轮次。
            if cancel.is_set():
                self._kill_process()
        try:
            try:
                output, error = process.communicate(timeout=_PROBE_SECONDS if probe else None)
            except subprocess.TimeoutExpired:
                process.kill()
                output, error = process.communicate()
                error = f"媒体探测超时；{error}"
            return process.returncode, output or "", error or ""
        finally:
            with self._condition:
                if self._process is process:
                    self._process = None

    def _publish(self, videos: list[Path], cancel: threading.Event) -> bool:
        """同编码及流参数的片段用单进程循环；H.264 复用，其他编码转为 H.264。"""
        video = videos[0]
        code, codec, error = self._execute([
            "ffprobe", "-v", "error", "-select_streams", "v:0",
            "-show_entries", "stream=codec_name", "-of", "csv=p=0", str(video),
        ], cancel, probe=True)
        if cancel.is_set():
            return False
        if code or not codec.strip():
            self._events.warn("simulation-media", f"无法探测视频 {video.name}：{error[-2000:]}")
            return False
        encoding = ["-c:v", "copy"] if codec.strip() == "h264" else [
            "-c:v", "libx264", "-preset", "veryfast", "-tune", "zerolatency",
            "-pix_fmt", "yuv420p", "-threads", "2",
        ]
        self._events.info(
            "simulation-media",
            f"尝试连续发布 {len(videos)} 个仿真片段，从 {video.name} 开始（{encoding[1]}）",
        )
        # concat 与输入循环保持同一 RTSP 连接，切片和回到首片均不重建发布者。
        with TemporaryDirectory(prefix="simulation-media-") as directory:
            playlist = Path(directory) / "playlist.ffconcat"
            entries = ["file '" + str(path).replace("'", "'\\''") + "'" for path in videos]
            playlist.write_text("ffconcat version 1.0\n" + "\n".join(entries) + "\n", encoding="utf-8")
            code, _, error = self._execute([
                "ffmpeg", "-hide_banner", "-loglevel", "error", "-nostdin",
                "-re", "-stream_loop", "-1", "-f", "concat", "-safe", "0", "-i", str(playlist),
                "-map", "0:v:0", "-an", "-sn", "-dn",
                *encoding, "-f", "rtsp", "-rtsp_transport", "tcp", self.rtsp_url,
            ], cancel)
        if code and not cancel.is_set():
            self._events.warn("simulation-media", f"RTSP 发布失败 {video.name}：{error[-2000:]}")
        return code == 0

    def _play(self, cancel: threading.Event) -> None:
        """每次从首个视频按路径排序连续循环播放；仅发布失败时重新尝试。"""
        url = urlsplit(self.rtsp_url)
        if url.scheme not in {"rtsp", "rtsps"} or not url.hostname:
            raise ValueError("rtsp_url 必须是 RTSP 地址")
        while not cancel.is_set():
            videos = self._files(self.videos_dir, _VIDEO_SUFFIXES)
            if not videos:
                self._events.warn("simulation-media", f"没有仿真视频：{self.videos_dir}")
                return
            if not self._publish(videos, cancel):
                if cancel.wait(_RETRY_SECONDS):
                    return

    def _run(self) -> None:
        """串行管理各轮发布；停止后保留空闲线程，快速重启也不并发抢流。"""
        while True:
            with self._condition:
                self._condition.wait_for(lambda: self._running or self._closed)
                if self._closed:
                    return
                cancel = self._cancel
            try:
                self._play(cancel)
            except (OSError, ValueError) as exc:
                if not cancel.is_set():
                    self._events.warn("simulation-media", f"仿真发布停止：{exc}")
            finally:
                with self._condition:
                    if self._cancel is cancel:
                        self._running = False
