"""读取机载视频与镜头 INI 配置，不把运行参数复制进飞控配置。"""

from __future__ import annotations

import configparser
import os
from dataclasses import dataclass
from pathlib import Path

from .config import (
    DEFAULT_MEDIAMTX_BINARY,
    CameraConfig,
    VIDEO_SERVICE_ROOT,
    default_camera_device,
    detect_lan_ipv4,
)


DEFAULT_CAMERA_CONFIG = VIDEO_SERVICE_ROOT / "config" / "camera.conf"
DEFAULT_LENS_CONFIG = VIDEO_SERVICE_ROOT / "config" / "lens.conf"


@dataclass(frozen=True)
class OnboardVideoSettings:
    """机载节点一次启动摄像头所需的完整不可变配置。"""

    camera: CameraConfig
    mediamtx_binary: Path
    lens_config: Path
    status_period_seconds: float


def load_onboard_settings(path: Path | None = None) -> OnboardVideoSettings:
    """从带注释的 INI 文件读取参数；不修正硬件组合或镜头值。"""
    config_path = Path(
        path
        or os.environ.get("VIDEO_SERVICE_ONBOARD_CONFIG", DEFAULT_CAMERA_CONFIG)
    ).expanduser()
    parser = configparser.ConfigParser(interpolation=None)
    if not parser.read(config_path, encoding="utf-8"):
        raise ValueError(f"无法读取机载摄像头配置：{config_path}")

    device_value = parser.get("camera", "device", fallback="auto").strip()
    ip_value = parser.get("rtsp", "advertise_ip", fallback="auto").strip()
    image_format = parser.get("storage", "image_format", fallback="jpg").strip().lower()
    if image_format not in {"jpg", "jpeg"}:
        raise ValueError("甲方协议要求 image_format=jpg")
    camera = CameraConfig(
        device=default_camera_device() if device_value.lower() == "auto" else device_value,
        codec=parser.get("camera", "codec", fallback="h264").strip().lower(),
        width=parser.getint("camera", "width", fallback=1280),
        height=parser.getint("camera", "height", fallback=720),
        fps=parser.getfloat("camera", "fps", fallback=120.0),
        rtsp_ip=detect_lan_ipv4() if ip_value.lower() == "auto" else ip_value,
        rtsp_port=parser.getint("rtsp", "port", fallback=8554),
        rtsp_path=parser.get("rtsp", "path", fallback="camera").strip(),
        container=parser.get("storage", "video_format", fallback="mp4").strip().lower(),
        video_directory=parser.get(
            "storage", "video_directory", fallback="/home/share"
        ).strip(),
        image_directory=parser.get(
            "storage", "image_directory", fallback="/home/share/jpg"
        ).strip(),
    )
    camera.validate()

    binary_value = parser.get(
        "runtime", "mediamtx_binary", fallback="/usr/local/bin/mediamtx"
    ).strip()
    mediamtx_binary = (
        DEFAULT_MEDIAMTX_BINARY
        if binary_value.lower() == "auto"
        else Path(binary_value).expanduser()
    )
    lens_config = Path(
        os.environ.get("VIDEO_SERVICE_LENS_CONFIG", DEFAULT_LENS_CONFIG)
    ).expanduser()
    status_period = parser.getfloat(
        "runtime", "status_period_seconds", fallback=1.0
    )
    if not 0.2 <= status_period <= 10.0:
        raise ValueError("status_period_seconds 必须位于 0.2～10 秒")
    return OnboardVideoSettings(
        camera=camera,
        mediamtx_binary=mediamtx_binary,
        lens_config=lens_config,
        status_period_seconds=status_period,
    )


def load_lens_controls(path: Path | None = None) -> list[tuple[str, str]]:
    """按文件顺序读取镜头控制名和值，不做范围夹紧或组合修正。"""
    config_path = Path(path or DEFAULT_LENS_CONFIG).expanduser()
    parser = configparser.ConfigParser(interpolation=None)
    parser.optionxform = str
    if not parser.read(config_path, encoding="utf-8"):
        raise ValueError(f"无法读取镜头配置：{config_path}")
    if not parser.has_section("controls"):
        raise ValueError(f"镜头配置缺少 [controls]：{config_path}")
    return [(name.strip(), value.strip()) for name, value in parser.items("controls")]
