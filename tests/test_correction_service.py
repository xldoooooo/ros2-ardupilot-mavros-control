"""AprilTag-Odin 配置、SE(3)、同步、稳健估计与 extnav 数学回归测试。"""

from __future__ import annotations

import importlib.util
import json
import math
from pathlib import Path
from types import SimpleNamespace
from typing import Any

import cv2
import numpy as np
from correction_service.camera_process import parse_v4l2_control_value
from correction_service.detector import AprilTagDetector
from correction_service.estimator import CorrectionEstimator, CorrectionSample
from correction_service.geometry import (
    compute_planar_correction,
    configured_tag_from_standard,
    homogeneous,
    planar_transform,
    rotation_z,
    world_tag_transform,
)
from correction_service.journal import JobJournal
from correction_service.synchronizer import (
    OdometrySynchronizer,
    is_apply_time_source_safe,
)

from correction_service.config import load_config

PROJECT_ROOT = Path(__file__).resolve().parent.parent
CONFIG_DIR = PROJECT_ROOT / "correction_service" / "config"


def test_job_journal_emits_strict_json_for_unavailable_metrics(tmp_path: Path) -> None:
    """失败任务尚无候选时应记录 null，不能把 NaN 写进 JSONL。"""
    journal = JobJournal(tmp_path, "strict-json")

    journal.write("failed", x_m=math.nan, nested={"quality": math.inf})
    record = json.loads(journal.path.read_text(encoding="utf-8"))

    assert record["x_m"] is None
    assert record["nested"]["quality"] is None


def test_v4l2_readback_accepts_numeric_value_with_enum_label() -> None:
    """Jetson v4l2-ctl 会在 exposure 枚举值后附加可读标签。"""
    assert parse_v4l2_control_value("auto_exposure: 1 (Manual Mode)\n") == 1
    assert parse_v4l2_control_value("gain: 240\n") == 240


def _stamp(nanoseconds: int) -> SimpleNamespace:
    """构造只含 sec/nanosec 的测试时间。"""
    return SimpleNamespace(
        sec=nanoseconds // 1_000_000_000,
        nanosec=nanoseconds % 1_000_000_000,
    )


def _odom(header_ns: int, name: str = "sample") -> SimpleNamespace:
    """构造同步器需要的最小 odometry 替身。"""
    return SimpleNamespace(
        header=SimpleNamespace(stamp=_stamp(header_ns)),
        name=name,
    )


def test_aircraft_calibration_config_is_loaded_exactly() -> None:
    """飞机内参、外参、Tag 尺寸和按需相机配置不得被默认值替代。"""
    config = load_config(CONFIG_DIR)

    assert config.interface_version == "1.0"
    assert (config.intrinsics.width, config.intrinsics.height) == (1920, 1080)
    assert math.isclose(config.intrinsics.camera_matrix[0, 0], 1143.4239585813639)
    assert math.isclose(config.t_imu_camera[0, 3], 0.04780285496559549)
    assert config.tags[0].size_m == 0.170
    assert config.camera.image_topic == "/correction_service/image_raw"
    assert config.camera.driver_package == "wasintek_gst_camera"
    assert config.lens_controls == {
        "brightness": 10,
        "auto_exposure": 1,
        "exposure_time_absolute": 25,
        "gain": 240,
        "zoom_absolute": 10,
    }


def test_full_se3_chain_recovers_known_planar_world_from_odin() -> None:
    """先完整组合倾斜相机/IMU，再提取的 SE(2) 必须恢复已知真值。"""
    config = load_config(CONFIG_DIR)
    tag = config.tags[0]
    desired = planar_transform(1.25, -0.73, math.radians(37.0))
    odin_from_imu = homogeneous(
        rotation_z(math.radians(-21.0)),
        np.array((0.42, 0.17, 0.31)),
    )
    world_from_imu = desired @ odin_from_imu
    world_from_camera = world_from_imu @ config.t_imu_camera
    camera_from_tag_configured = np.linalg.inv(world_from_camera) @ world_tag_transform(
        tag
    )
    camera_from_tag_standard = camera_from_tag_configured @ np.linalg.inv(
        configured_tag_from_standard()
    )

    correction = compute_planar_correction(
        camera_from_tag_standard,
        tag,
        config.t_imu_camera,
        odin_from_imu,
    )

    assert np.allclose(correction.world_from_odin, desired, atol=1e-10)
    assert math.isclose(correction.x_m, 1.25, abs_tol=1e-10)
    assert math.isclose(correction.y_m, -0.73, abs_tol=1e-10)
    assert math.isclose(correction.yaw_rad, math.radians(37.0), abs_tol=1e-10)
    assert correction.tilt_rad < 1e-8


def test_detector_pose_solver_uses_metric_tag_size_and_scaled_intrinsics() -> None:
    """已知投影角点经 IPPE 解算后应恢复相机前方的公制 Tag 位姿。"""
    config = load_config(CONFIG_DIR)
    detector = AprilTagDetector(config.intrinsics, config.detection)
    half = config.tags[0].size_m / 2.0
    object_points = np.array(
        (
            (-half, half, 0.0),
            (half, half, 0.0),
            (half, -half, 0.0),
            (-half, -half, 0.0),
        ),
        dtype=np.float64,
    )
    rotation_vector = np.array((0.08, -0.05, 0.23), dtype=np.float64)
    translation = np.array((0.04, -0.02, 0.82), dtype=np.float64)
    projected, _ = cv2.projectPoints(
        object_points,
        rotation_vector,
        translation,
        detector.camera_matrix,
        config.intrinsics.distortion,
    )

    result = detector._estimate_pose(  # noqa: SLF001 - deterministic solver test.
        0, projected.reshape(1, 4, 2), config.tags[0].size_m
    )

    assert result is not None
    assert np.allclose(result.camera_from_tag_standard[:3, 3], translation, atol=1e-5)
    assert result.reprojection_error_px < 1e-4


def test_synchronizer_never_hides_bad_same_epoch_header_with_arrival_time() -> None:
    """同 epoch 的 header 超限时必须拒绝，不能因接收时刻接近而伪装匹配。"""
    config = load_config(CONFIG_DIR)
    synchronizer = OdometrySynchronizer(config.synchronization)
    image_ns = 100_000_000_000
    synchronizer.add(_odom(image_ns + 100_000_000), image_ns)

    assert synchronizer.match(image_ns) is None


def test_synchronizer_explicitly_reports_epoch_mismatch_fallback() -> None:
    """相机 header 与 Odin header 不同 epoch 时才允许接收时钟有限回退。"""
    config = load_config(CONFIG_DIR)
    synchronizer = OdometrySynchronizer(config.synchronization)
    image_ns = 100_000_000_000
    message = _odom(8_000_000_000_000, "fallback")
    synchronizer.add(message, image_ns + 4_000_000)

    matched = synchronizer.match(image_ns)

    assert matched is not None
    assert matched.message.name == "fallback"
    assert matched.time_source == "arrival_history"
    assert math.isclose(matched.delta_ms, 4.0)
    assert is_apply_time_source_safe(matched.time_source)
    assert is_apply_time_source_safe("header")
    assert not is_apply_time_source_safe("mixed")


def test_estimator_converges_across_yaw_wrap_only_after_all_gates() -> None:
    """±180° 圆角样本不能产生 360° 离散度，且样本数/时长门都必须满足。"""
    config = load_config(CONFIG_DIR)
    estimator = CorrectionEstimator(config.quality)
    snapshot = None
    for index in range(30):
        jitter = (index % 5 - 2) * 0.0003
        yaw = math.radians(179.9 + (index % 3 - 1) * 0.03)
        if index % 2:
            yaw -= 2.0 * math.pi
        snapshot = estimator.add(
            CorrectionSample(
                stamp_ns=index * 100_000_000,
                x_m=0.4 + jitter,
                y_m=-0.2 - jitter,
                yaw_rad=yaw,
                tilt_rad=math.radians(1.2),
                reprojection_error_px=0.4,
                odom_match_error_ms=3.0,
                odom_time_source="header",
                processing_time_ms=8.0,
            )
        )

    assert snapshot is not None and snapshot.converged
    assert snapshot.inlier_samples >= config.quality.minimum_samples
    assert snapshot.span_seconds >= config.quality.minimum_span_seconds
    assert snapshot.yaw_std_rad < math.radians(0.1)
    assert abs(abs(math.degrees(snapshot.yaw_rad)) - 179.9) < 0.1


def _load_extnav_module() -> Any:
    """从受版本控制的部署源加载 extnav 数学，不依赖外部工作区路径。"""
    path = (
        PROJECT_ROOT
        / "correction_service"
        / "extnav_patch"
        / "extnav_to_vision_pose.py"
    )
    spec = importlib.util.spec_from_file_location("task27_extnav_patch", path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_extnav_planar_correction_left_multiplies_pose_and_rotates_velocity() -> None:
    """实际 extnav 修正必须发布新消息并一致旋转位置、姿态、水平速度。"""
    from nav_msgs.msg import Odometry

    extnav = _load_extnav_module()
    raw = Odometry()
    raw.pose.pose.position.x = 2.0
    raw.pose.pose.position.y = 1.0
    raw.pose.pose.position.z = 0.7
    raw.pose.pose.orientation.w = 1.0
    raw.twist.twist.linear.x = 1.0
    raw.twist.twist.linear.y = 0.0
    corrected = extnav.apply_planar_correction(raw, 0.5, -0.25, math.pi / 2.0)

    assert corrected is not raw
    assert math.isclose(corrected.pose.pose.position.x, -0.5, abs_tol=1e-12)
    assert math.isclose(corrected.pose.pose.position.y, 1.75, abs_tol=1e-12)
    assert corrected.pose.pose.position.z == 0.7
    assert math.isclose(corrected.pose.pose.orientation.z, math.sqrt(0.5))
    assert math.isclose(corrected.pose.pose.orientation.w, math.sqrt(0.5))
    assert math.isclose(corrected.twist.twist.linear.x, 0.0, abs_tol=1e-12)
    assert math.isclose(corrected.twist.twist.linear.y, 1.0, abs_tol=1e-12)
    assert raw.pose.pose.position.x == 2.0
