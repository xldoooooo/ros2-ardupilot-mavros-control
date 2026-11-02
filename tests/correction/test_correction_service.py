"""AprilTag-Odin 配置、SE(3)、同步、稳健估计与 extnav 数学回归测试。"""

from __future__ import annotations

import csv
import importlib.util
import json
import logging
import math
import subprocess
import shutil
from pathlib import Path
from types import SimpleNamespace
from typing import Any

import cv2
import numpy as np
import pytest
import yaml
from correction_service.camera_process import CameraProcess, parse_v4l2_control_value
from correction_service.detector import AprilTagDetector
from correction_service.estimator import CorrectionEstimator, CorrectionSample
from correction_service.geometry import (
    compute_planar_correction,
    configured_tag_from_standard,
    homogeneous,
    planar_transform,
    project_fcu_reference_pose,
    rotation_z,
    world_tag_transform,
)
from correction_service.journal import JobJournal
from correction_service.synchronizer import (
    OdometrySynchronizer,
    is_apply_time_source_safe,
)

from correction_service.config import load_config

PROJECT_ROOT = Path(__file__).resolve().parents[2]
CONFIG_DIR = PROJECT_ROOT / "config" / "aircraft" / "simulation" / "correction"


def test_aircraft_correction_configs_are_complete() -> None:
    """Each aircraft owns one directly readable correction configuration."""
    required = {"camera.conf", "intrinsics.yaml", "extrinsics.yaml", "lens.conf",
                "general_settings.yaml", "tag_pose.csv"}
    for aircraft_id in ("simulation", "uav001", "uav002"):
        profile = PROJECT_ROOT / "config" / "aircraft" / aircraft_id / "correction"
        assert required <= {path.name for path in profile.iterdir()}
        shared = yaml.safe_load((profile.parent / "odin_extrinsics.yaml").read_text())
        parameters = shared["/**"]["ros__parameters"]
        assert np.allclose(load_config(profile).fcu_reference.lever_arm_m,
                           [parameters[f"odin_{axis}"] for axis in "xyz"])


def test_shared_odin_geometry_changes_reference_and_fingerprint(tmp_path: Path) -> None:
    """Odin source edits affect correction geometry and invalidate saved windows."""
    root = tmp_path / "correction"
    shutil.copytree(CONFIG_DIR, root)
    odin_path = tmp_path / "odin_extrinsics.yaml"
    shutil.copy2(CONFIG_DIR.parent / "odin_extrinsics.yaml", odin_path)
    before = load_config(root)
    odin = yaml.safe_load(odin_path.read_text())
    odin["/**"]["ros__parameters"]["odin_x"] = 0.12
    odin_path.write_text(yaml.safe_dump(odin))
    after = load_config(root)
    assert np.allclose(after.fcu_reference.lever_arm_m, (0.12, -0.04, 0.05))
    assert after.config_fingerprint != before.config_fingerprint
    odin_path.unlink()
    with pytest.raises(ValueError, match="odin_extrinsics.yaml"):
        load_config(root)


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


def test_lens_controls_apply_in_file_order_then_verify_final_values(
    tmp_path: Path, monkeypatch: Any
) -> None:
    """下视相机须先稳定曝光模式，再在全部写入后核验最终值。"""
    config = load_config(CONFIG_DIR)
    calls: list[list[str]] = []
    sleeps: list[float] = []

    def run(command: list[str], **_kwargs: Any) -> subprocess.CompletedProcess[str]:
        calls.append(list(command))
        if "--get-ctrl" in command:
            name = command[-1]
            value = config.lens_controls[name]
            suffix = " (Manual Mode)" if name == "auto_exposure" else ""
            return subprocess.CompletedProcess(
                command, 0, f"{name}: {value}{suffix}\n", ""
            )
        return subprocess.CompletedProcess(command, 0, "", "")

    monkeypatch.setattr(
        "correction_service.camera_process.shutil.which",
        lambda _name: "/usr/bin/v4l2-ctl",
    )
    monkeypatch.setattr("correction_service.camera_process.subprocess.run", run)
    monkeypatch.setattr("correction_service.camera_process.time.sleep", sleeps.append)
    camera = CameraProcess(
        config.camera,
        config.lens_controls,
        tmp_path / "camera.log",
        logging.getLogger("test-correction-camera"),
    )

    readback = camera.apply_lens_controls()

    set_calls = [call for call in calls if "--set-ctrl" in call]
    get_calls = [call for call in calls if "--get-ctrl" in call]
    expected_names = list(config.lens_controls)
    assert [call[-1].split("=", 1)[0] for call in set_calls] == expected_names
    assert [call[-1] for call in get_calls] == expected_names
    assert calls.index(get_calls[0]) > calls.index(set_calls[-1])
    assert expected_names[:3] == [
        "auto_exposure",
        "white_balance_automatic",
        "focus_automatic_continuous",
    ]
    assert sleeps == [0.2]
    assert readback == config.lens_controls


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

    assert config.interface_version == "2.0"
    assert config.window.default_size == 5
    assert config.window.maximum_size == 20
    assert len(config.config_fingerprint) == 64
    assert (config.intrinsics.width, config.intrinsics.height) == (1920, 1080)
    assert math.isclose(config.intrinsics.camera_matrix[0, 0], 942.1240709375937)
    assert math.isclose(config.intrinsics.calibration_rms_px, 0.8863085437393902)
    assert np.allclose(config.intrinsics.distortion,
                       [0.03626268524324754, -0.056048601794863075,
                        0.0010844756398792788, -0.0010052529149732062, 0.00958122664915688])
    # simulation 保留原物理相机位置，改写到 FCU 后需加原 FCU→IMU 位移。
    assert np.allclose(config.t_fcu_camera[:3, 3],
                       np.array((41 + 5.57, 50 - 21.03, -46 - 12 - 9.26)) / 1000
                       + config.fcu_reference.lever_arm_m)
    assert np.array_equal(config.t_fcu_camera[:3, 2], (0, 0, -1))
    assert np.array_equal(config.t_fcu_camera[:3, :3] @ (0, -1, 0), (1, 0, 0))
    # Tag 布设由现场 CSV 决定；换纸后仍须逐项精确加载，不能锁死历史17cm。
    with (CONFIG_DIR / "tag_pose.csv").open() as stream:
        rows = list(csv.DictReader(stream))
    assert set(config.tags) == {int(row["tag_id"]) for row in rows}
    for row in rows:
        tag = config.tags[int(row["tag_id"])]
        assert (tag.x, tag.y, tag.z, tag.size_m) == tuple(
            float(row[key]) for key in ("x", "y", "z", "size_m")
        )
        assert tag.yaw_rad == math.radians(float(row["yaw_deg"]))
    assert config.camera.image_topic == "/correction_service/image_raw"
    assert config.camera.device.endswith("UQ212_UQ212-video-index0")
    assert config.camera.fps == 120
    assert config.camera.driver_package == "correction_service"
    assert config.lens_controls == {
        "auto_exposure": 3,
        "white_balance_automatic": 1,
        "focus_automatic_continuous": 0,
        "brightness": 0,
        "contrast": 34,
        "saturation": 60,
        "hue": 0,
        "gamma": 120,
        "sharpness": 2,
        "backlight_compensation": 1,
        "power_line_frequency": 1,
        "zoom_absolute": 0,
    }


def test_full_se3_chain_recovers_known_planar_world_from_odin() -> None:
    """先完整组合相机/FCU，再提取的 SE(2) 必须恢复已知真值。"""
    config = load_config(CONFIG_DIR)
    tag = config.tags[0]
    desired = planar_transform(1.25, -0.73, math.radians(37.0))
    odin_from_fcu = homogeneous(
        rotation_z(math.radians(-21.0)),
        np.array((0.42, 0.17, 0.31)),
    )
    world_from_fcu = desired @ odin_from_fcu
    world_from_camera = world_from_fcu @ config.t_fcu_camera
    camera_from_tag_configured = np.linalg.inv(world_from_camera) @ world_tag_transform(
        tag
    )
    camera_from_tag_standard = camera_from_tag_configured @ np.linalg.inv(
        configured_tag_from_standard()
    )

    correction = compute_planar_correction(
        camera_from_tag_standard,
        tag,
        config.t_fcu_camera,
        odin_from_fcu,
    )

    assert np.allclose(correction.world_from_odin, desired, atol=1e-10)
    assert math.isclose(correction.x_m, 1.25, abs_tol=1e-10)
    assert math.isclose(correction.y_m, -0.73, abs_tol=1e-10)
    assert math.isclose(correction.yaw_rad, math.radians(37.0), abs_tol=1e-10)
    assert correction.tilt_rad < 1e-8
    expected_odin_from_tag = (
        odin_from_fcu @ config.t_fcu_camera @ camera_from_tag_configured
    )
    assert np.allclose(correction.odin_from_tag, expected_odin_from_tag, atol=1e-10)
    assert np.allclose(
        (correction.odin_tag_x_m, correction.odin_tag_y_m),
        expected_odin_from_tag[:2, 3],
        atol=1e-10,
    )


def test_tilted_full_chain_preserves_task27_planar_projection() -> None:
    """非零 tilt 时首次仍直接提取 C_full 的 x/y/yaw，不能改成单点锚定。"""
    config = load_config(CONFIG_DIR)
    tag = config.tags[0]
    yaw = math.radians(-18.0)
    pitch = math.radians(8.0)
    pitch_rotation = np.array(
        (
            (math.cos(pitch), 0.0, math.sin(pitch)),
            (0.0, 1.0, 0.0),
            (-math.sin(pitch), 0.0, math.cos(pitch)),
        ),
        dtype=np.float64,
    )
    desired_full = homogeneous(
        rotation_z(yaw) @ pitch_rotation,
        np.array((0.08, -0.04, 0.72)),
    )
    odin_from_fcu = homogeneous(
        rotation_z(math.radians(11.0)), np.array((0.25, -0.31, 0.72))
    )
    world_from_fcu = desired_full @ odin_from_fcu
    world_from_camera = world_from_fcu @ config.t_fcu_camera
    camera_from_tag_configured = np.linalg.inv(world_from_camera) @ world_tag_transform(
        tag
    )
    camera_from_tag_standard = camera_from_tag_configured @ np.linalg.inv(
        configured_tag_from_standard()
    )

    correction = compute_planar_correction(
        camera_from_tag_standard,
        tag,
        config.t_fcu_camera,
        odin_from_fcu,
    )

    assert np.allclose(correction.world_from_odin, desired_full, atol=1e-10)
    assert math.isclose(correction.x_m, desired_full[0, 3], abs_tol=1e-10)
    assert math.isclose(correction.y_m, desired_full[1, 3], abs_tol=1e-10)
    assert math.isclose(correction.yaw_rad, yaw, abs_tol=1e-10)
    assert math.isclose(correction.tilt_rad, pitch, abs_tol=1e-10)

    # Task27 的 planar_xy_yaw(C_full) 保留坐标系原点平移。丢弃 tilt 后，
    # 单个三维点通常不再严格满足二维 P=RQ+t；该差值不是重算首次平移的依据。
    q_odin = correction.odin_from_tag[:2, 3]
    predicted_world = rotation_z(correction.yaw_rad)[:2, :2] @ q_odin + np.array(
        (correction.x_m, correction.y_m)
    )
    assert np.linalg.norm(predicted_world - np.array((tag.x, tag.y))) > 0.05


def test_nonzero_installation_preserves_full_tag_chain_and_final_center() -> None:
    """30°安装的新旧完整链等价；驱动中心输出经 Tag 修正后回到真实 FCU。"""
    config = load_config(PROJECT_ROOT / "config/aircraft/uav002/correction")
    tag = config.tags[0]
    pitch = -math.pi / 6.0
    fcu_from_imu = homogeneous(
        np.array(((math.cos(pitch), 0, math.sin(pitch)),
                  (0, 1, 0), (-math.sin(pitch), 0, math.cos(pitch)))),
        np.array((0.100, -0.029, 0.060)),
    )
    imu_from_camera = np.linalg.inv(fcu_from_imu) @ config.t_fcu_camera
    desired = planar_transform(-0.8, 1.6, math.radians(-42.0))
    # 非零机体倾斜和偏航，与固定安装角分开构造。
    body_rotation, _ = cv2.Rodrigues(np.array((0.12, -0.08, 0.33)))
    odin_from_fcu = homogeneous(body_rotation, np.array((2.0, -0.7, 0.4)))
    odin_from_imu = odin_from_fcu @ fcu_from_imu
    world_from_fcu = desired @ odin_from_fcu
    world_from_imu = world_from_fcu @ fcu_from_imu
    camera_from_tag_configured = np.linalg.inv(
        world_from_fcu @ config.t_fcu_camera
    ) @ world_tag_transform(tag)
    camera_from_tag_standard = camera_from_tag_configured @ np.linalg.inv(
        configured_tag_from_standard()
    )

    correction = compute_planar_correction(
        camera_from_tag_standard, tag, config.t_fcu_camera, odin_from_fcu
    )
    old_world_from_odin = world_from_imu @ np.linalg.inv(odin_from_imu)
    old_odin_from_tag = odin_from_imu @ imu_from_camera @ camera_from_tag_configured
    assert np.allclose(correction.world_fcu, world_from_fcu, atol=1e-12)
    assert np.allclose(correction.world_from_odin, old_world_from_odin, atol=1e-12)
    assert np.allclose(correction.world_from_odin, desired, atol=1e-12)
    assert np.allclose(correction.odin_from_tag, old_odin_from_tag, atol=1e-12)

    center = homogeneous(np.eye(3), np.array((tag.x, tag.y, 0.70)))
    raw_center = np.linalg.inv(desired) @ center
    projected = project_fcu_reference_pose(
        raw_center, (correction.x_m, correction.y_m, correction.yaw_rad),
        config.fcu_reference.lever_arm_m,
    )
    assert np.allclose(projected.position[:2], (tag.x, tag.y), atol=1e-12)
    assert np.allclose(projected.rotation, np.eye(3), atol=1e-12)
    assert math.isclose(projected.position[2],
                        raw_center[2, 3] + config.fcu_reference.lever_arm_m[2])


def test_detector_pose_solver_uses_metric_tag_size_and_scaled_intrinsics() -> None:
    """不同真实边长的 Tag 都必须用各自尺寸恢复公制位姿。"""
    config = load_config(CONFIG_DIR)
    detector = AprilTagDetector(config.intrinsics, config.detection)
    rotation_vector = np.array((0.08, -0.05, 0.23), dtype=np.float64)
    translation = np.array((0.04, -0.02, 0.82), dtype=np.float64)
    for tag_id, size_m in ((0, config.tags[0].size_m), (91, 0.245)):
        half = size_m / 2.0
        object_points = np.array(
            (
                (-half, half, 0.0),
                (half, half, 0.0),
                (half, -half, 0.0),
                (-half, -half, 0.0),
            ),
            dtype=np.float64,
        )
        projected, _ = cv2.projectPoints(
            object_points,
            rotation_vector,
            translation,
            detector.camera_matrix,
            config.intrinsics.distortion,
        )

        result = detector._estimate_pose(tag_id, projected.reshape(1, 4, 2), size_m)

        assert result is not None
        assert np.allclose(
            result.camera_from_tag_standard[:3, 3], translation, atol=1e-5
        )
        assert result.reprojection_error_px < 1e-4


def test_detector_pnp_accounts_for_distortion_without_double_correction() -> None:
    """原始畸变角点配 K/D 应等价于先去畸变角点再配 K/零畸变。"""
    config = load_config(CONFIG_DIR)
    detector = AprilTagDetector(config.intrinsics, config.detection)
    size_m = config.tags[0].size_m
    half = size_m / 2.0
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
    translation = np.array((0.25, -0.15, 0.82), dtype=np.float64)
    distorted_points, _ = cv2.projectPoints(
        object_points,
        rotation_vector,
        translation,
        detector.camera_matrix,
        config.intrinsics.distortion,
    )
    detected = detector._estimate_pose(0, distorted_points, size_m)
    undistorted_points = cv2.undistortPoints(
        distorted_points,
        detector.camera_matrix,
        config.intrinsics.distortion,
        P=detector.camera_matrix,
    )
    solved, explicit_rotation, explicit_translation = cv2.solvePnP(
        object_points,
        undistorted_points,
        detector.camera_matrix,
        np.zeros_like(config.intrinsics.distortion),
        flags=cv2.SOLVEPNP_IPPE_SQUARE,
    )

    assert detected is not None and solved
    assert np.allclose(
        detected.camera_from_tag_standard[:3, 3],
        explicit_translation.reshape(3),
        atol=1e-10,
    )
    detected_rotation, _ = cv2.Rodrigues(explicit_rotation)
    assert np.allclose(
        detected.camera_from_tag_standard[:3, :3], detected_rotation, atol=1e-10
    )


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


def test_extnav_planar_correction_left_multiplies_pose_and_keeps_body_velocity() -> None:
    """世界修正左乘位姿；FCU 机体系速度不随世界系 yaw 改变。"""
    from nav_msgs.msg import Odometry

    extnav = _load_extnav_module()
    raw = Odometry()
    raw.header.frame_id = "odom"
    raw.child_frame_id = "fcu"
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
    assert corrected.twist.twist.linear.x == 1.0
    assert corrected.twist.twist.linear.y == 0.0
    assert raw.pose.pose.position.x == 2.0
