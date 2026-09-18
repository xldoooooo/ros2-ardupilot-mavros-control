"""印刷外框不能吞掉真实 Tag；检测角点仍必须位于编码黑边。"""

from dataclasses import replace
from pathlib import Path

import cv2
import numpy as np
import pytest
from correction_service.detector import AprilTagDetector

from correction_service.config import load_config


@pytest.mark.parametrize("tag_id", [0, 1])
@pytest.mark.parametrize("gap", [10, 12, 14])
def test_printed_outer_frame_preserves_tag_and_metric_corners(tag_id, gap):
    """复现现场额外黑框；旧 .05 筛选会丢失内部可解码候选。"""
    cfg = load_config(Path(__file__).resolve().parents[2] / "correction_service/config")
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    draw = getattr(cv2.aruco, "generateImageMarker", None) or cv2.aruco.drawMarker
    frame = np.full((1080, 1920), 255, np.uint8)
    # 120px 编码边长，外围白色间隔与8px装饰框均不属于公制边长。
    frame[312-gap:448+gap, 812-gap:948+gap] = 0
    frame[320-gap:440+gap, 820-gap:940+gap] = 255
    frame[320:440, 820:940] = draw(dictionary, tag_id, 120)
    tag = replace(cfg.tags[0], tag_id=tag_id)
    detector = AprilTagDetector(cfg.intrinsics, cfg.detection)
    batch = detector.detect(frame, {tag_id: tag})
    assert batch.marker_ids == (tag_id,)
    assert len(batch.detections) == 1
    assert np.allclose(batch.detections[0].corners,
                       [[409.5, 159.5], [469.5, 159.5],
                        [469.5, 219.5], [409.5, 219.5]], atol=1)
    assert batch.detections[0].reprojection_error_px < 1


def test_blank_frame_does_not_decode_with_two_configured_tags():
    """减少候选合并距离不应把纯背景识别为配置中的 Tag。"""
    cfg = load_config(Path(__file__).resolve().parents[2] / "correction_service/config")
    tags = {i: replace(cfg.tags[0], tag_id=i) for i in (0, 1)}
    detector = AprilTagDetector(cfg.intrinsics, cfg.detection)
    assert detector.detect(np.full((1080, 1920), 180, np.uint8), tags).marker_ids == ()
