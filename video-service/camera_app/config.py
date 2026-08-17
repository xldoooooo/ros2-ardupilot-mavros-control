"""摄像头配置、运行路径、V4L2 能力探测与持久化。"""

from __future__ import annotations

from dataclasses import asdict, dataclass
from fractions import Fraction
import ipaddress
import json
import os
from pathlib import Path
import re
import socket
import subprocess
from typing import Any


VIDEO_SERVICE_ROOT = Path(__file__).resolve().parent.parent
PROJECT_ROOT = VIDEO_SERVICE_ROOT.parent
BUNDLED_MEDIAMTX = VIDEO_SERVICE_ROOT / "bin" / "mediamtx" / "mediamtx"
CONFIG_SCHEMA_VERSION = 1

SUPPORTED_CODECS = ("h264", "mjpeg")
SUPPORTED_CONTAINERS = ("mp4", "mkv", "avi")
CODEC_LABELS = {"h264": "H.264（摄像头原生）", "mjpeg": "MJPEG（兼容回退）"}
CONTAINER_LABELS = {"mp4": "MP4（推荐）", "mkv": "MKV", "avi": "AVI"}
_FOURCC_TO_CODEC = {"H264": "h264", "MJPG": "mjpeg", "JPEG": "mjpeg"}
_PATH_SEGMENT = re.compile(r"^[A-Za-z0-9_.~-]+$")


@dataclass(frozen=True, order=True)
class CameraMode:
    """摄像头明确声明支持的一组编码、分辨率和帧率。"""

    codec: str
    width: int
    height: int
    fps: float

    @property
    def label(self) -> str:
        """返回适合 GUI 展示的紧凑模式名称。"""
        fps_text = format_fps(self.fps)
        return f"{self.width}×{self.height}  ·  {fps_text} fps"

    def to_dict(self) -> dict[str, Any]:
        """转换为可通过本地 JSON IPC 传输的字典。"""
        return asdict(self)


@dataclass(frozen=True)
class RuntimePaths:
    """摄像头服务配置、日志、Socket 与生成配置的宿主路径。"""

    config_file: Path
    runtime_dir: Path
    state_dir: Path
    socket_file: Path
    pid_file: Path
    mediamtx_config: Path
    mediamtx_log: Path
    ffmpeg_log: Path

    @classmethod
    def discover(cls) -> "RuntimePaths":
        """按显式环境、XDG 目录和安全的用户临时目录依次选址。"""
        home = Path.home()
        config_file = Path(
            os.environ.get(
                "VIDEO_SERVICE_CONFIG",
                home / ".config" / "ros2-ardupilot-camera" / "config.json",
            )
        ).expanduser()

        configured_runtime = os.environ.get("VIDEO_SERVICE_RUNTIME_DIR")
        xdg_runtime = os.environ.get("XDG_RUNTIME_DIR")
        if configured_runtime:
            runtime_dir = Path(configured_runtime).expanduser()
        elif xdg_runtime:
            runtime_dir = Path(xdg_runtime) / "ros2-ardupilot-camera"
        else:
            runtime_dir = Path("/tmp") / f"ros2-ardupilot-camera-{os.getuid()}"

        state_dir = Path(
            os.environ.get(
                "VIDEO_SERVICE_STATE_DIR",
                home / ".local" / "state" / "ros2-ardupilot-camera",
            )
        ).expanduser()
        return cls(
            config_file=config_file,
            runtime_dir=runtime_dir,
            state_dir=state_dir,
            socket_file=runtime_dir / "camera-service.sock",
            pid_file=runtime_dir / "camera-service.pid",
            mediamtx_config=runtime_dir / "mediamtx.yml",
            mediamtx_log=state_dir / "mediamtx.log",
            ffmpeg_log=state_dir / "ffmpeg.log",
        )

    def ensure_directories(self) -> None:
        """创建私有运行目录、状态目录与配置文件父目录。"""
        self.runtime_dir.mkdir(parents=True, exist_ok=True, mode=0o700)
        self.state_dir.mkdir(parents=True, exist_ok=True, mode=0o700)
        self.config_file.parent.mkdir(parents=True, exist_ok=True, mode=0o700)
        try:
            self.runtime_dir.chmod(0o700)
            self.state_dir.chmod(0o700)
        except OSError:
            # 某些共享文件系统不支持 chmod；目录仍由宿主 umask 保护。
            pass


@dataclass(frozen=True)
class CameraConfig:
    """摄像头采集、RTSP发布和本地保存的唯一权威配置。"""

    device: str
    codec: str = "h264"
    width: int = 1920
    height: int = 1080
    fps: float = 30.0
    rtsp_ip: str = "127.0.0.1"
    rtsp_port: int = 8554
    rtsp_path: str = "camera"
    container: str = "mp4"
    video_directory: str = ""
    image_directory: str = ""

    @classmethod
    def defaults(cls) -> "CameraConfig":
        """从当前用户目录和已连接设备生成可直接使用的默认配置。"""
        return cls(
            device=default_camera_device(),
            rtsp_ip=detect_lan_ipv4(),
            video_directory=str(
                Path.home() / "Videos" / "ros2-ardupilot-camera"
            ),
            image_directory=str(
                Path.home() / "Pictures" / "ros2-ardupilot-camera"
            ),
        )

    @classmethod
    def from_dict(cls, raw: dict[str, Any]) -> "CameraConfig":
        """从不可信 JSON 字典提取已知字段并执行完整校验。"""
        defaults = cls.defaults()
        values = {
            field: raw.get(field, getattr(defaults, field))
            for field in cls.__dataclass_fields__
        }
        try:
            config = cls(
                device=str(values["device"]),
                codec=str(values["codec"]).lower(),
                width=int(values["width"]),
                height=int(values["height"]),
                fps=float(values["fps"]),
                rtsp_ip=str(values["rtsp_ip"]),
                rtsp_port=int(values["rtsp_port"]),
                rtsp_path=str(values["rtsp_path"]),
                container=str(values["container"]).lower(),
                video_directory=str(values["video_directory"]),
                image_directory=str(values["image_directory"]),
            )
        except (TypeError, ValueError) as exc:
            raise ValueError(f"摄像头配置字段类型无效：{exc}") from exc
        config.validate()
        return config

    def validate(self) -> None:
        """拒绝不支持、不可安全拼入 FFmpeg/MediaMTX 的配置值。"""
        if not self.device.strip():
            raise ValueError("摄像头设备不能为空")
        if self.codec not in SUPPORTED_CODECS:
            raise ValueError(f"不支持的摄像头编码：{self.codec}")
        if self.container not in SUPPORTED_CONTAINERS:
            raise ValueError(f"不支持的保存格式：{self.container}")
        if not 16 <= self.width <= 8192 or not 16 <= self.height <= 8192:
            raise ValueError("分辨率超出 16～8192 像素范围")
        if not 0.5 <= self.fps <= 240.0:
            raise ValueError("帧率超出 0.5～240 fps 范围")
        if not 1024 <= self.rtsp_port <= 65535:
            raise ValueError("RTSP端口必须位于 1024～65535")
        try:
            address = ipaddress.ip_address(self.rtsp_ip)
        except ValueError as exc:
            raise ValueError("拉流IP必须是有效的 IPv4 地址") from exc
        if address.version != 4 or address.is_unspecified:
            raise ValueError("拉流IP必须是具体的 IPv4 地址，不能使用 0.0.0.0")

        segments = self.rtsp_path.strip("/").split("/")
        valid_segments = all(
            segment and _PATH_SEGMENT.fullmatch(segment) for segment in segments
        )
        if not valid_segments:
            raise ValueError("RTSP路径只能包含字母、数字、点、下划线、波浪线和连字符")
        for label, directory in (
            ("视频保存路径", self.video_directory),
            ("图片保存路径", self.image_directory),
        ):
            if not directory.strip():
                raise ValueError(f"{label}不能为空")
            if any(
                character in directory
                for character in ("\n", "\r", "|", "[", "]", "\\")
            ):
                raise ValueError(f"{label}包含不支持的字符")

    @property
    def normalized_rtsp_path(self) -> str:
        """返回不带首尾斜线的 MediaMTX path。"""
        return self.rtsp_path.strip("/")

    @property
    def rtsp_url(self) -> str:
        """返回提供给局域网设备和 GUI 预览的完整地址。"""
        return (
            f"rtsp://{self.rtsp_ip}:{self.rtsp_port}/"
            f"{self.normalized_rtsp_path}"
        )

    @property
    def local_rtsp_url(self) -> str:
        """返回 FFmpeg 发布和本机截图使用的回环 RTSP 地址。"""
        return (
            f"rtsp://127.0.0.1:{self.rtsp_port}/"
            f"{self.normalized_rtsp_path}"
        )

    @property
    def fps_fraction(self) -> Fraction:
        """返回稳定的有理帧率，供固定包时间戳使用。"""
        return Fraction(str(self.fps)).limit_denominator(1001)

    @property
    def ffmpeg_time_base(self) -> str:
        """返回一帧一个 tick 的 FFmpeg setts 时间基。"""
        rate = self.fps_fraction
        return f"{rate.denominator}/{rate.numerator}"

    def to_dict(self) -> dict[str, Any]:
        """转换为配置文件与本地 IPC 使用的稳定字典。"""
        result = asdict(self)
        result["schema_version"] = CONFIG_SCHEMA_VERSION
        result["rtsp_url"] = self.rtsp_url
        return result


def format_fps(fps: float) -> str:
    """整数帧率不显示小数，其余帧率最多保留三位。"""
    if float(fps).is_integer():
        return str(int(fps))
    return f"{fps:.3f}".rstrip("0").rstrip(".")


def default_camera_device() -> str:
    """优先返回稳定的 USB by-id 视频节点，再回退到 /dev/video0。"""
    by_id = Path("/dev/v4l/by-id")
    candidates = sorted(by_id.glob("*-video-index0")) if by_id.is_dir() else []
    if candidates:
        return str(candidates[0])
    return "/dev/video0"


def detect_lan_ipv4() -> str:
    """优先使用主路由源地址，避免代理 TUN 地址被误当作局域网地址。"""
    try:
        result = subprocess.run(
            ["ip", "-json", "-4", "route", "show", "default"],
            check=False,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            timeout=1.0,
        )
        routes = json.loads(result.stdout) if result.returncode == 0 else []
        if isinstance(routes, list):
            routes.sort(
                key=lambda item: int(item.get("metric", 0))
                if isinstance(item, dict)
                else 0
            )
            for route in routes:
                if not isinstance(route, dict):
                    continue
                address = str(route.get("prefsrc", ""))
                if _is_concrete_ipv4(address):
                    return address
    except (
        OSError,
        subprocess.TimeoutExpired,
        json.JSONDecodeError,
        ValueError,
    ):
        pass

    # 非 Linux 或路由条目没有 prefsrc 时，保留无实际发包的 UDP 推导回退。
    probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        probe.connect(("192.0.2.1", 9))
        address = str(probe.getsockname()[0])
        if _is_concrete_ipv4(address):
            return address
    except OSError:
        pass
    finally:
        probe.close()

    try:
        for item in socket.getaddrinfo(socket.gethostname(), None, socket.AF_INET):
            address = item[4][0]
            if not ipaddress.ip_address(address).is_loopback:
                return address
    except OSError:
        pass
    return "127.0.0.1"


def _is_concrete_ipv4(value: str) -> bool:
    """判断字符串是否为可展示给局域网客户端的具体 IPv4 地址。"""
    try:
        address = ipaddress.ip_address(value)
    except ValueError:
        return False
    return address.version == 4 and not address.is_unspecified


def load_config(paths: RuntimePaths | None = None) -> CameraConfig:
    """读取用户配置；不存在时返回硬件友好的默认值。"""
    resolved = paths or RuntimePaths.discover()
    if not resolved.config_file.is_file():
        return CameraConfig.defaults()
    try:
        raw = json.loads(resolved.config_file.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise ValueError(f"无法读取摄像头配置：{exc}") from exc
    if not isinstance(raw, dict):
        raise ValueError("摄像头配置根节点必须是 JSON 对象")
    return CameraConfig.from_dict(raw)


def save_config(config: CameraConfig, paths: RuntimePaths | None = None) -> None:
    """通过同目录临时文件和原子替换持久化已验证配置。"""
    config.validate()
    resolved = paths or RuntimePaths.discover()
    resolved.ensure_directories()
    temporary = resolved.config_file.with_suffix(".json.tmp")
    payload = config.to_dict()
    payload.pop("rtsp_url", None)
    temporary.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    os.chmod(temporary, 0o600)
    os.replace(temporary, resolved.config_file)


def discover_camera_devices() -> list[dict[str, str]]:
    """列出真正具备视频采集格式的稳定设备节点，过滤元数据节点。"""
    candidates: list[Path] = []
    by_id = Path("/dev/v4l/by-id")
    if by_id.is_dir():
        candidates.extend(sorted(by_id.glob("*-video-index0")))
    if not candidates:
        candidates.extend(sorted(Path("/dev").glob("video*")))

    devices: list[dict[str, str]] = []
    seen_targets: set[Path] = set()
    for candidate in candidates:
        try:
            target = candidate.resolve(strict=True)
        except OSError:
            continue
        if target in seen_targets:
            continue
        try:
            modes = probe_camera_modes(str(candidate))
        except (OSError, RuntimeError):
            continue
        if not modes:
            continue
        seen_targets.add(target)
        devices.append(
            {
                "path": str(candidate),
                "target": str(target),
                "label": _device_label(candidate),
            }
        )
    return devices


def _device_label(path: Path) -> str:
    """把稳定 UVC symlink 文件名转成可读设备名称。"""
    name = path.name
    if name.startswith("usb-"):
        name = name[4:]
    name = name.removesuffix("-video-index0").replace("_", " ")
    return name or str(path)


def probe_camera_modes(device: str) -> list[CameraMode]:
    """调用 v4l2-ctl 并解析离散编码、尺寸和帧率组合。"""
    command = ["v4l2-ctl", "--device", device, "--list-formats-ext"]
    try:
        result = subprocess.run(
            command,
            check=False,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            timeout=5.0,
        )
    except FileNotFoundError as exc:
        raise RuntimeError("缺少 v4l2-ctl，请安装 v4l-utils") from exc
    except subprocess.TimeoutExpired as exc:
        raise RuntimeError("读取摄像头能力超时") from exc
    if result.returncode != 0:
        detail = result.stdout.strip() or f"退出码 {result.returncode}"
        raise RuntimeError(f"无法读取摄像头能力：{detail}")
    return parse_v4l2_formats(result.stdout)


def parse_v4l2_formats(output: str) -> list[CameraMode]:
    """解析 v4l2-ctl --list-formats-ext 的离散模式输出。"""
    codec: str | None = None
    size: tuple[int, int] | None = None
    modes: set[CameraMode] = set()
    format_pattern = re.compile(r"\[\d+\]:\s+'([^']+)'")
    size_pattern = re.compile(r"Size:\s+Discrete\s+(\d+)x(\d+)")
    fps_pattern = re.compile(r"\((\d+(?:\.\d+)?)\s+fps\)")

    for line in output.splitlines():
        format_match = format_pattern.search(line)
        if format_match:
            codec = _FOURCC_TO_CODEC.get(format_match.group(1).upper())
            size = None
            continue
        size_match = size_pattern.search(line)
        if size_match:
            size = (int(size_match.group(1)), int(size_match.group(2)))
            continue
        fps_match = fps_pattern.search(line)
        if codec and size and fps_match:
            modes.add(
                CameraMode(
                    codec=codec,
                    width=size[0],
                    height=size[1],
                    fps=float(fps_match.group(1)),
                )
            )
    return sorted(modes, key=lambda item: (item.codec, -item.width, -item.fps))
