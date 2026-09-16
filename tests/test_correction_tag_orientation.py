"""用官方 Tag 像素和独立物理场景验证方向，避免同一错误矩阵自生成、自验证。"""

import math
from dataclasses import replace
from pathlib import Path

import cv2
import numpy as np
import pytest
import yaml
from correction_service.detector import AprilTagDetector
from correction_service.geometry import (
    compute_planar_correction,
    homogeneous,
    project_fcu_reference_pose,
    rotation_z,
)

from correction_service.config import load_config, _load_intrinsics

CONFIG_DIR = Path(__file__).resolve().parents[1] / "correction_service/config"
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


def _physical_camera_in_imu() -> np.ndarray:
    """独立冻结 2026-08-27 原始标定，禁止从待测配置反推物理真值。"""
    return np.array(
        [
            [
                0.07572468395269205,
                -0.9965466361414317,
                -0.03406720175568133,
                0.04780285496559549,
            ],
            [
                -0.9969178615240186,
                -0.07496181266164714,
                -0.023140959722069636,
                0.027639069129542938,
            ],
            [
                0.02050730637219694,
                0.035714543783696824,
                -0.9991516009833945,
                -0.07853750991857861,
            ],
            [0.0, 0.0, 0.0, 1.0],
        ]
    )


@pytest.mark.parametrize("camera_profile", ("Wasintek", "UQ212"))
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
    camera_profile: str,
    fcu_xy: tuple[float, float],
    yaw_deg: float,
    tag_yaw_deg: float,
) -> None:
    """真实图案→检测→PnP→首次修正→移回原点，水平位置与航向都必须正确。"""
    cfg = load_config(CONFIG_DIR)
    physical = _physical_camera_in_imu()
    if camera_profile == "Wasintek":
        # 历史图案方向回归仍使用旧相机档案，与独立冻结的历史矩阵核对。
        archive = yaml.safe_load((CONFIG_DIR / "Wasintek/extrinsics.yaml").read_text())
        cfg = replace(cfg, t_imu_camera=np.array(archive["matrix"]["data"]).reshape(4, 4),
                      intrinsics=_load_intrinsics(CONFIG_DIR / "Wasintek"))
    else:
        # 独立构造本次机械安装场景，不从待测矩阵生成期望真值。
        physical = homogeneous(
            np.array(((0, -1, 0), (-1, 0, 0), (0, 0, -1))),
            np.array((41 + 5.57, 50 - 21.03, -(46 + 12 + 9.26))) / 1000,
        )
    tag = replace(cfg.tags[0], yaw_rad=math.radians(tag_yaw_deg))
    # 这里使用无畸变合成镜头，以单独检查角点方向；畸变另有公制回归覆盖。
    intrinsics = replace(cfg.intrinsics, distortion=np.zeros(5))
    detector = AprilTagDetector(intrinsics, cfg.detection)
    fi = homogeneous(np.eye(3), np.array((0.06, -0.03, 0.05)))
    wf = homogeneous(rotation_z(math.radians(yaw_deg)), np.array((*fcu_xy, 0.70)))
    wi = wf @ fi
    wc = wi @ physical
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
    assert batch.marker_ids == (0,)
    assert len(batch.detections) == 1
    # Odin 起始自建系归零，随后按真实刚体位移变化。
    oi_start = np.eye(4)
    measured = compute_planar_correction(
        batch.detections[0].camera_from_tag_standard, tag, cfg.t_imu_camera, oi_start
    )
    candidate = (measured.x_m, measured.y_m, measured.yaw_rad)
    start = project_fcu_reference_pose(oi_start, candidate, fi[:3, 3])
    assert np.allclose(start.position[:2], fcu_xy, atol=0.008)
    assert abs(
        math.remainder(measured.yaw_rad - math.radians(yaw_deg), 2 * math.pi)
    ) < math.radians(0.3)
    wf_center = homogeneous(np.eye(3), np.array((0.0, 0.0, 0.70)))
    oi_center = np.linalg.inv(wi) @ wf_center @ fi
    final = project_fcu_reference_pose(oi_center, candidate, fi[:3, 3])
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
