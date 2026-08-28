"""严格加载相机标定、Tag 世界位姿和修正任务运行参数。"""

from __future__ import annotations

import configparser
import csv
import math
import re
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np
import yaml

_ROS_NAME = re.compile(r"^[A-Za-z_][A-Za-z0-9_]*$")


@dataclass(frozen=True)
class Intrinsics:
    """固定分辨率下的 OpenCV 针孔内参与畸变。"""

    width: int
    height: int
    camera_matrix: np.ndarray
    distortion: np.ndarray
    calibration_rms_px: float


@dataclass(frozen=True)
class TagPose:
    """Tag 坐标系在车间世界系中的绝对位姿和检测边长。"""

    tag_id: int
    x: float
    y: float
    z: float
    yaw_rad: float
    size_m: float


@dataclass(frozen=True)
class CameraSettings:
    """只含采集所需的相机和外部 PTS 驱动参数。"""

    device: str
    width: int
    height: int
    fps: int
    pixel_format: str
    frame_id: str
    image_topic: str
    max_capture_age_ms: float
    driver_package: str
    driver_executable: str


@dataclass(frozen=True)
class DetectionSettings:
    """AprilTag 检测与单帧几何质量阈值。"""

    tag_family: str
    processing_rate_hz: float
    image_scale: float
    corner_refinement: bool
    max_reprojection_error_px: float
    max_correction_tilt_deg: float
    reject_multiple_tags: bool


@dataclass(frozen=True)
class SynchronizationSettings:
    """图像时间戳与 Odin 历史样本匹配限制。"""

    odometry_history_seconds: float
    max_header_delta_ms: float
    max_arrival_delta_ms: float
    header_epoch_tolerance_seconds: float


@dataclass(frozen=True)
class QualitySettings:
    """滚动稳健估计的收敛和发散阈值。"""

    minimum_samples: int
    rolling_window_samples: int
    minimum_span_seconds: float
    max_position_std_m: float
    max_yaw_std_rad: float
    max_position_range_m: float
    max_yaw_range_rad: float
    divergence_position_range_m: float
    divergence_yaw_range_rad: float
    mad_outlier_scale: float


@dataclass(frozen=True)
class TimeoutSettings:
    """相机、首个 Tag、extnav ACK 与地面任务生命周期限制。"""

    camera_start_seconds: float
    first_tag_seconds: float
    extnav_apply_seconds: float
    ground_max_runtime_seconds: float


@dataclass(frozen=True)
class LoggingSettings:
    """模块内日志目录与轮转配置。"""

    directory: Path
    service_log_max_bytes: int
    service_log_backups: int
    sample_summary_period_seconds: float


@dataclass(frozen=True)
class ServiceConfig:
    """correction_service 的完整、已验证配置快照。"""

    config_dir: Path
    interface_version: str
    topics: dict[str, str]
    intrinsics: Intrinsics
    t_imu_camera: np.ndarray
    tags: dict[int, TagPose]
    camera: CameraSettings
    lens_controls: dict[str, int]
    detection: DetectionSettings
    synchronization: SynchronizationSettings
    quality: QualitySettings
    timeouts: TimeoutSettings
    logging: LoggingSettings


def _mapping(value: Any, name: str) -> dict[str, Any]:
    """要求 YAML 节点为映射并保留可读错误路径。"""
    if not isinstance(value, dict):
        raise ValueError(f"{name} 必须是映射")
    return value


def _finite(value: Any, name: str) -> float:
    """解析一个有限浮点数。"""
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"{name} 必须是有限数")
    return result


def _positive(value: Any, name: str, *, allow_zero: bool = False) -> float:
    """解析正数；仅明确配置的总时限允许为零。"""
    result = _finite(value, name)
    if result < 0.0 or (result == 0.0 and not allow_zero):
        relation = "非负" if allow_zero else "正"
        raise ValueError(f"{name} 必须为{relation}数")
    return result


def _load_yaml(path: Path) -> dict[str, Any]:
    """读取标准 YAML，拒绝空文件和非映射根节点。"""
    try:
        loaded = yaml.safe_load(path.read_text(encoding="utf-8"))
    except (OSError, yaml.YAMLError) as exc:
        raise ValueError(f"无法读取配置 {path}: {exc}") from exc
    return _mapping(loaded, str(path))


def _matrix(spec: Any, rows: int, cols: int, name: str) -> np.ndarray:
    """从 rows/cols/data 结构构造固定形状矩阵。"""
    mapping = _mapping(spec, name)
    if int(mapping.get("rows", -1)) != rows or int(mapping.get("cols", -1)) != cols:
        raise ValueError(f"{name} 必须是 {rows}x{cols} 矩阵")
    data = np.asarray(mapping.get("data", []), dtype=np.float64)
    if data.size != rows * cols or not np.isfinite(data).all():
        raise ValueError(f"{name}.data 数量或有限性无效")
    return data.reshape(rows, cols)


def _load_intrinsics(config_dir: Path) -> Intrinsics:
    """加载并验证分辨率、焦距矩阵和畸变向量。"""
    data = _load_yaml(config_dir / "intrinsics.yaml")
    width = int(data.get("image_width", 0))
    height = int(data.get("image_height", 0))
    matrix = _matrix(data.get("camera_matrix"), 3, 3, "camera_matrix")
    distortion_spec = _mapping(
        data.get("distortion_coefficients"), "distortion_coefficients"
    )
    distortion = np.asarray(distortion_spec.get("data", []), dtype=np.float64).reshape(
        -1
    )
    if width <= 0 or height <= 0:
        raise ValueError("相机标定分辨率必须为正整数")
    if distortion.size not in (4, 5, 8, 12, 14) or not np.isfinite(distortion).all():
        raise ValueError("畸变系数数量或有限性无效")
    if (
        matrix[0, 0] <= 0.0
        or matrix[1, 1] <= 0.0
        or not np.allclose(matrix[2], (0, 0, 1))
    ):
        raise ValueError("camera_matrix 不是有效针孔内参")
    return Intrinsics(
        width=width,
        height=height,
        camera_matrix=matrix,
        distortion=distortion,
        calibration_rms_px=_positive(
            data.get("rms_reprojection_error"), "rms_reprojection_error"
        ),
    )


def _load_extrinsics(config_dir: Path) -> np.ndarray:
    """加载 T_imu_camera，并检查刚体旋转正交性和齐次末行。"""
    data = _load_yaml(config_dir / "extrinsics.yaml")
    transform = _matrix(data.get("matrix"), 4, 4, "T_imu_camera")
    rotation = transform[:3, :3]
    if not np.allclose(transform[3], (0, 0, 0, 1), atol=1e-9):
        raise ValueError("T_imu_camera 齐次末行无效")
    if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-6):
        raise ValueError("T_imu_camera 旋转矩阵不正交")
    if not math.isclose(float(np.linalg.det(rotation)), 1.0, abs_tol=1e-6):
        raise ValueError("T_imu_camera 旋转矩阵行列式不为 1")
    return transform


def _load_tags(config_dir: Path) -> dict[int, TagPose]:
    """加载人工维护的 Tag 世界坐标，拒绝重复 ID 与非正边长。"""
    path = config_dir / "tag_pose.csv"
    expected = ("tag_id", "x", "y", "z", "yaw_deg", "size_m")
    tags: dict[int, TagPose] = {}
    try:
        with path.open("r", encoding="utf-8", newline="") as stream:
            reader = csv.DictReader(stream)
            if tuple(reader.fieldnames or ()) != expected:
                raise ValueError(f"tag_pose.csv 表头必须为 {','.join(expected)}")
            for line_number, row in enumerate(reader, start=2):
                tag_id = int(row["tag_id"])
                if tag_id < 0 or tag_id in tags:
                    raise ValueError(f"tag_pose.csv 第 {line_number} 行 ID 重复或为负")
                size = _positive(row["size_m"], f"tag_pose.csv:{line_number}:size_m")
                tags[tag_id] = TagPose(
                    tag_id=tag_id,
                    x=_finite(row["x"], f"tag_pose.csv:{line_number}:x"),
                    y=_finite(row["y"], f"tag_pose.csv:{line_number}:y"),
                    z=_finite(row["z"], f"tag_pose.csv:{line_number}:z"),
                    yaw_rad=math.radians(
                        _finite(row["yaw_deg"], f"tag_pose.csv:{line_number}:yaw_deg")
                    ),
                    size_m=size,
                )
    except OSError as exc:
        raise ValueError(f"无法读取 {path}: {exc}") from exc
    if not tags:
        raise ValueError("tag_pose.csv 至少需要一个 Tag")
    return tags


def _load_ini(path: Path) -> configparser.ConfigParser:
    """读取带注释的 INI 配置并禁用隐式字符串插值。"""
    parser = configparser.ConfigParser(interpolation=None)
    try:
        with path.open("r", encoding="utf-8") as stream:
            parser.read_file(stream)
    except (OSError, configparser.Error) as exc:
        raise ValueError(f"无法读取配置 {path}: {exc}") from exc
    return parser


def _load_camera(config_dir: Path) -> CameraSettings:
    """加载采集节点配置；命令始终以参数列表启动，不经过 shell。"""
    parser = _load_ini(config_dir / "camera.conf")
    package = parser.get("driver", "package", fallback="").strip()
    executable = parser.get("driver", "executable", fallback="").strip()
    if not _ROS_NAME.fullmatch(package) or not _ROS_NAME.fullmatch(executable):
        raise ValueError("相机 ROS package/executable 名称无效")
    settings = CameraSettings(
        device=parser.get("camera", "device", fallback="").strip(),
        width=parser.getint("camera", "width", fallback=0),
        height=parser.getint("camera", "height", fallback=0),
        fps=parser.getint("camera", "fps", fallback=0),
        pixel_format=parser.get("camera", "pixel_format", fallback="").strip().lower(),
        frame_id=parser.get("camera", "frame_id", fallback="").strip(),
        image_topic=parser.get("camera", "image_topic", fallback="").strip(),
        max_capture_age_ms=parser.getfloat(
            "camera", "max_capture_age_ms", fallback=0.0
        ),
        driver_package=package,
        driver_executable=executable,
    )
    if not settings.device.startswith("/dev/"):
        raise ValueError("camera.device 必须是明确的 /dev 路径")
    if settings.width <= 0 or settings.height <= 0 or settings.fps <= 0:
        raise ValueError("camera width/height/fps 必须为正整数")
    if settings.pixel_format != "mjpeg":
        raise ValueError("当前 PTS 相机节点只支持已验收的 mjpeg 管线")
    if not settings.image_topic.startswith("/") or not settings.frame_id:
        raise ValueError("camera image_topic/frame_id 无效")
    _positive(settings.max_capture_age_ms, "camera.max_capture_age_ms")
    return settings


def _load_lens(config_dir: Path) -> dict[str, int]:
    """只允许已知且经标定使用的 V4L2 整数控制项。"""
    parser = _load_ini(config_dir / "lens.conf")
    allowed = {
        "brightness",
        "auto_exposure",
        "exposure_time_absolute",
        "gain",
        "zoom_absolute",
    }
    controls = {key: int(value) for key, value in parser.items("controls")}
    unknown = set(controls) - allowed
    if unknown or not controls:
        raise ValueError(f"lens.conf 含未知项或为空：{sorted(unknown)}")
    return controls


def load_config(config_dir: str | Path) -> ServiceConfig:
    """一次性加载全部配置；任何歧义都在节点创建服务前失败。"""
    root = Path(config_dir).expanduser().resolve()
    general = _load_yaml(root / "general_settings.yaml")
    topics = {
        str(key): str(value)
        for key, value in _mapping(general.get("topics"), "topics").items()
    }
    required_topics = {
        "raw_odometry",
        "corrected_odometry",
        "camera_image",
        "correction_status",
        "correction_result",
        "extnav_status",
        "start_service",
        "stop_service",
        "set_correction_service",
    }
    if set(topics) != required_topics or any(
        not value.startswith("/") for value in topics.values()
    ):
        raise ValueError("topics 必须完整且全部为绝对 ROS 名称")

    detection = _mapping(general.get("detection"), "detection")
    sync = _mapping(general.get("synchronization"), "synchronization")
    quality = _mapping(general.get("quality"), "quality")
    timeouts = _mapping(general.get("timeouts"), "timeouts")
    logging = _mapping(general.get("logging"), "logging")
    tag_family = str(detection.get("tag_family", "")).strip()
    if tag_family != "tag36h11":
        raise ValueError("首版只允许 tag36h11")

    minimum_samples = int(quality.get("minimum_samples", 0))
    rolling_samples = int(quality.get("rolling_window_samples", 0))
    if minimum_samples < 3 or rolling_samples < minimum_samples:
        raise ValueError("质量窗口必须不少于 minimum_samples >= 3")

    intrinsics = _load_intrinsics(root)
    camera = _load_camera(root)
    if (camera.width, camera.height) != (intrinsics.width, intrinsics.height):
        raise ValueError("camera.conf 分辨率必须与 intrinsics.yaml 完全一致")

    log_path = Path(str(logging.get("directory", "../log"))).expanduser()
    if not log_path.is_absolute():
        log_path = (root / log_path).resolve()

    return ServiceConfig(
        config_dir=root,
        interface_version=str(general.get("interface_version", "")).strip(),
        topics=topics,
        intrinsics=intrinsics,
        t_imu_camera=_load_extrinsics(root),
        tags=_load_tags(root),
        camera=camera,
        lens_controls=_load_lens(root),
        detection=DetectionSettings(
            tag_family=tag_family,
            processing_rate_hz=_positive(
                detection.get("processing_rate_hz"), "processing_rate_hz"
            ),
            image_scale=_positive(detection.get("image_scale"), "image_scale"),
            corner_refinement=bool(detection.get("corner_refinement", True)),
            max_reprojection_error_px=_positive(
                detection.get("max_reprojection_error_px"), "max_reprojection_error_px"
            ),
            max_correction_tilt_deg=_positive(
                detection.get("max_correction_tilt_deg"), "max_correction_tilt_deg"
            ),
            reject_multiple_tags=bool(detection.get("reject_multiple_tags", True)),
        ),
        synchronization=SynchronizationSettings(
            odometry_history_seconds=_positive(
                sync.get("odometry_history_seconds"), "odometry_history_seconds"
            ),
            max_header_delta_ms=_positive(
                sync.get("max_header_delta_ms"), "max_header_delta_ms"
            ),
            max_arrival_delta_ms=_positive(
                sync.get("max_arrival_delta_ms"), "max_arrival_delta_ms"
            ),
            header_epoch_tolerance_seconds=_positive(
                sync.get("header_epoch_tolerance_seconds"),
                "header_epoch_tolerance_seconds",
            ),
        ),
        quality=QualitySettings(
            minimum_samples=minimum_samples,
            rolling_window_samples=rolling_samples,
            minimum_span_seconds=_positive(
                quality.get("minimum_span_seconds"), "minimum_span_seconds"
            ),
            max_position_std_m=_positive(
                quality.get("max_position_std_m"), "max_position_std_m"
            ),
            max_yaw_std_rad=math.radians(
                _positive(quality.get("max_yaw_std_deg"), "max_yaw_std_deg")
            ),
            max_position_range_m=_positive(
                quality.get("max_position_range_m"), "max_position_range_m"
            ),
            max_yaw_range_rad=math.radians(
                _positive(quality.get("max_yaw_range_deg"), "max_yaw_range_deg")
            ),
            divergence_position_range_m=_positive(
                quality.get("divergence_position_range_m"),
                "divergence_position_range_m",
            ),
            divergence_yaw_range_rad=math.radians(
                _positive(
                    quality.get("divergence_yaw_range_deg"), "divergence_yaw_range_deg"
                )
            ),
            mad_outlier_scale=_positive(
                quality.get("mad_outlier_scale"), "mad_outlier_scale"
            ),
        ),
        timeouts=TimeoutSettings(
            camera_start_seconds=_positive(
                timeouts.get("camera_start_seconds"), "camera_start_seconds"
            ),
            first_tag_seconds=_positive(
                timeouts.get("first_tag_seconds"), "first_tag_seconds"
            ),
            extnav_apply_seconds=_positive(
                timeouts.get("extnav_apply_seconds"), "extnav_apply_seconds"
            ),
            ground_max_runtime_seconds=_positive(
                timeouts.get("ground_max_runtime_seconds", 0.0),
                "ground_max_runtime_seconds",
                allow_zero=True,
            ),
        ),
        logging=LoggingSettings(
            directory=log_path,
            service_log_max_bytes=int(
                _positive(logging.get("service_log_max_bytes"), "service_log_max_bytes")
            ),
            service_log_backups=int(
                _positive(logging.get("service_log_backups"), "service_log_backups")
            ),
            sample_summary_period_seconds=_positive(
                logging.get("sample_summary_period_seconds"),
                "sample_summary_period_seconds",
            ),
        ),
    )
