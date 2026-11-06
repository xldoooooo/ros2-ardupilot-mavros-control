"""用官方 Tag 像素和独立物理场景验证方向，避免同一错误矩阵自生成、自验证。"""

import math
from dataclasses import replace
from pathlib import Path

import cv2
import numpy as np
import pytest
import _correction_native as native
from correction_service.detector import AprilTagDetector
from correction_service.geometry import (
    compute_planar_correction,
    homogeneous,
    project_fcu_reference_pose,
    rotation_z,
)

from correction_service.config import load_config

CONFIG_DIR = Path(__file__).resolve().parents[2] / "config/aircraft/simulation/correction"
# AprilRobotics 官方 PNG 的 8×8 黑边及数据位，按原文件上方排列；不是由待测
# OpenCV 字典或几何函数生成。白色保护边不计入 0.170m 的检测边长。
# https://github.com/AprilRobotics/apriltag-imgs/blob/master/tag36h11/tag36_11_00000.png
OFFICIAL_TAG0 = (
    np.array(
        [
            [0, 0, 0, 0, 0, 0, 0, 0],
            [0, 1, 1, 0, 1, 0, 1, 0],
            [0, 0, 1, 1, 1, 0, 1, 0],
            [0, 0, 1, 1, 0, 0, 0, 0],
            [0, 1, 0, 1, 0, 0, 0, 0],
            [0, 0, 1, 0, 1, 1, 0, 0],
            [0, 0, 0, 0, 1, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0],
        ],
        dtype=np.uint8,
    )
    * 255
)


@pytest.mark.parametrize(
    "fcu_xy,yaw_deg,tag_yaw_deg",
    [
        ((-0.20, 0.02), -25.0, 0.0),  # 用户描述：在纸张后方约20cm，向右偏航。
        ((0.08, -0.09), 15.0, 0.0),
        ((-0.12, 0.08), 80.0, 90.0),
        ((0.02, 0.05), -110.0, -90.0),
    ],
)
def test_official_print_to_first_correction_and_return_to_center(
    fcu_xy: tuple[float, float],
    yaw_deg: float,
    tag_yaw_deg: float,
) -> None:
    """真实图案→检测→PnP→首次修正→移回原点，水平位置与航向都必须正确。"""
    cfg = load_config(CONFIG_DIR)
    # Independently build the supported UQ212 mechanical installation scene.
    physical = homogeneous(
        np.array(((0, -1, 0), (-1, 0, 0), (0, 0, -1))),
        np.array((141 + 5.57, 10 - 21.03, -(46 + 12 + 9.26) + 50)) / 1000,
    )
    # 历史独立真值场景固定17cm；现场换纸不得改变此场景的像素尺寸与精度断言。
    tag = replace(cfg.tags[0], size_m=0.170, yaw_rad=math.radians(tag_yaw_deg))
    # 这里使用无畸变合成镜头，以单独检查角点方向；畸变另有公制回归覆盖。
    intrinsics = replace(cfg.intrinsics, distortion=np.zeros(5))
    detector = AprilTagDetector(intrinsics, cfg.detection)
    wf = homogeneous(rotation_z(math.radians(yaw_deg)), np.array((*fcu_xy, 0.70)))
    wc = wf @ physical
    wt = homogeneous(rotation_z(tag.yaw_rad), np.zeros(3))
    ct = np.linalg.inv(wc) @ wt
    half = tag.size_m / 2
    # 官方纸张物理 TL/TR/BR/BL，+X上/+Y左/+Z朝上；不使用生产坐标转换。
    corners_tag = np.array(
        ((half, half, 0), (half, -half, 0), (-half, -half, 0), (-half, half, 0)),
        dtype=float,
    )
    rvec, _ = cv2.Rodrigues(ct[:3, :3])
    pixels, _ = cv2.projectPoints(
        corners_tag, rvec, ct[:3, 3], intrinsics.camera_matrix, intrinsics.distortion
    )
    marker = cv2.resize(OFFICIAL_TAG0, (400, 400), interpolation=cv2.INTER_NEAREST)
    source = np.array(((0, 0), (399, 0), (399, 399), (0, 399)), dtype=np.float32)
    transform = cv2.getPerspectiveTransform(
        source, pixels.reshape(4, 2).astype(np.float32)
    )
    frame = cv2.warpPerspective(marker, transform, (1920, 1080), borderValue=255)
    batch = detector.detect(frame, {0: tag})
    transferred = native.resize_gray(frame, cfg.detection.image_scale)
    direct = AprilTagDetector(intrinsics, cfg.detection, prescaled=True).detect(transferred, {0: tag})
    assert direct.marker_ids == batch.marker_ids
    assert len(direct.detections) == len(batch.detections)
    for before, after in zip(batch.detections, direct.detections):
        assert np.array_equal(before.corners, after.corners)
        assert np.array_equal(before.camera_from_tag_standard, after.camera_from_tag_standard)
        assert before.reprojection_error_px == after.reprojection_error_px
    assert batch.marker_ids == (0,)
    assert len(batch.detections) == 1
    # Odin 起始自建系归零，随后按真实刚体位移变化。
    of_start = np.eye(4)
    measured = compute_planar_correction(
        batch.detections[0].camera_from_tag_standard, tag, cfg.t_fcu_camera, of_start
    )
    candidate = (measured.x_m, measured.y_m, measured.yaw_rad)
    start = project_fcu_reference_pose(of_start, candidate, cfg.fcu_reference.lever_arm_m)
    assert np.allclose(start.position[:2], fcu_xy, atol=0.008)
    assert abs(
        math.remainder(measured.yaw_rad - math.radians(yaw_deg), 2 * math.pi)
    ) < math.radians(0.3)
    wf_center = homogeneous(np.eye(3), np.array((0.0, 0.0, 0.70)))
    of_center = np.linalg.inv(wf) @ wf_center
    final = project_fcu_reference_pose(of_center, candidate, cfg.fcu_reference.lever_arm_m)
    assert np.linalg.norm(final.position[:2]) < 0.008


def test_official_tag_top_is_opposite_opencv_dictionary_top() -> None:
    """锁定第三方库角点约定：官方原图上方是 OpenCV 点2→点3的一侧。"""
    cfg = load_config(CONFIG_DIR)
    detector = AprilTagDetector(cfg.intrinsics, cfg.detection)
    marker = cv2.resize(OFFICIAL_TAG0, (400, 400), interpolation=cv2.INTER_NEAREST)
    frame = np.full((1080, 1920), 255, dtype=np.uint8)
    frame[340:740, 760:1160] = marker
    batch = detector.detect(frame, cfg.tags)
    assert batch.marker_ids == (0,)
    corners = batch.detections[0].corners
    # 缩放后纸张黑边 x≈[380,580]、y≈[170,370]，顺序是 BR/BL/TL/TR。
    assert np.allclose(
        corners,
        ((579.5, 369.5), (379.5, 369.5), (379.5, 169.5), (579.5, 169.5)),
        atol=1.0,
    )


def _roi_scene():
    """Place the independent official print on a full-resolution pixel grid."""
    cfg = load_config(CONFIG_DIR)
    image = np.full((cfg.intrinsics.height, cfg.intrinsics.width), 255, np.uint8)
    image[390:670, 820:1100] = cv2.resize(OFFICIAL_TAG0, (280, 280), interpolation=cv2.INTER_NEAREST)
    return cfg, image


def test_roi_tracks_hover_rotation_and_translation_without_changing_full_frame_pose():
    """Full-resolution ROI keeps full-image perimeter gates, coordinates and subpixel context."""
    cfg, original = _roi_scene()
    settings = replace(cfg.detection, image_scale=1., roi_enabled=True)
    tracker = AprilTagDetector(cfg.intrinsics, settings)
    reference = AprilTagDetector(cfg.intrinsics, replace(settings, roi_enabled=False))
    matrix = cfg.intrinsics.camera_matrix
    motions = [(0, 0, 0, 0, 0), (10, -8, 1, 2, 2), (-40, 25, -3, -2, -5),
               (80, 30, 4, 3, 8), (-80, -30, -4, -3, -8), (0, 0, 0, 0, 0)]
    used_roi = False
    for x, y, roll, pitch, yaw in motions:
        rotation, _ = cv2.Rodrigues(np.radians((float(roll), float(pitch), float(yaw))))
        translate = np.array(((1., 0., x), (0., 1., y), (0., 0., 1.)))
        homography = translate @ matrix @ rotation @ np.linalg.inv(matrix)
        image = cv2.warpPerspective(original, homography, (original.shape[1], original.shape[0]), borderValue=255)
        expected = reference.detect(image, {0: cfg.tags[0]})
        actual = tracker.detect(image, {0: cfg.tags[0]})
        assert actual.marker_ids == expected.marker_ids == (0,)
        assert len(actual.detections) == len(expected.detections) == 1
        assert np.allclose(actual.detections[0].corners, expected.detections[0].corners, atol=1e-5, rtol=0)
        assert np.allclose(actual.detections[0].camera_from_tag_standard,
                           expected.detections[0].camera_from_tag_standard, atol=1e-6, rtol=0)
        used_roi |= actual.roi_used
    assert used_roi


def test_roi_recovers_large_motion_on_same_frame_and_never_reuses_missing_target():
    """Unexpected hover movement, loss and reacquisition cannot silently reuse an old pose."""
    cfg, original = _roi_scene()
    tracker = AprilTagDetector(cfg.intrinsics, replace(cfg.detection, image_scale=1., roi_enabled=True))
    tags = {0: cfg.tags[0]}
    assert tracker.detect(original, tags).marker_ids == (0,)
    assert tracker.detect(original, tags).roi_used
    moved = cv2.warpAffine(original, np.array(((1., 0., 600.), (0., 1., 0.))),
                          (original.shape[1], original.shape[0]), borderValue=255)
    recovered = tracker.detect(moved, tags)
    assert recovered.roi_used and recovered.roi_fallback
    assert recovered.marker_ids == (0,) and len(recovered.detections) == 1
    lost = tracker.detect(np.full_like(original, 255), tags)
    assert lost.roi_fallback and not lost.marker_ids and not lost.detections
    assert tracker.detect(original, tags).marker_ids == (0,)
    tracker.reset()
    assert not tracker.detect(original, tags).roi_used


def test_native_selected_jpeg_preserves_decode_pixels_and_original_coordinates():
    """The compressed handoff decodes exactly once and keeps full-image pose coordinates."""
    cfg, original = _roi_scene()
    ok, jpeg = cv2.imencode('.jpg', original)
    assert ok
    settings = replace(cfg.detection, image_scale=1., roi_enabled=True)
    tracker = AprilTagDetector(cfg.intrinsics, settings)
    reference = AprilTagDetector(cfg.intrinsics, replace(settings, roi_enabled=False))
    decoded = cv2.imdecode(jpeg, cv2.IMREAD_GRAYSCALE)
    expected = reference.detect(decoded, {0: cfg.tags[0]})
    for _ in range(2):
        result = tracker.detect_compressed(jpeg, {0: cfg.tags[0]})
        assert result.decode_time_ms > 0
        assert result.marker_ids == expected.marker_ids == (0,)
        assert np.allclose(result.detections[0].camera_from_tag_standard,
                           expected.detections[0].camera_from_tag_standard, atol=1e-6, rtol=0)
