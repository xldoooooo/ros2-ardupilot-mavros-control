"""使用 OpenCV AprilTag 36h11 检测器生成带重投影残差的相机<-Tag 位姿。"""

from __future__ import annotations

import time
from dataclasses import dataclass

import cv2
import numpy as np
import _correction_native as native

from .config import DetectionSettings, Intrinsics, TagPose


@dataclass(frozen=True)
class PoseDetection:
    """单个已配置 Tag 的 PnP 位姿与像素质量。"""

    tag_id: int
    camera_from_tag_standard: np.ndarray
    reprojection_error_px: float
    corners: np.ndarray


@dataclass(frozen=True)
class DetectionBatch:
    """一帧中全部解码 ID、可用位姿和实际处理耗时。"""

    marker_ids: tuple[int, ...]
    detections: tuple[PoseDetection, ...]
    processing_time_ms: float


class AprilTagDetector:
    """保留原Python接口，原生执行同一36h11搜索、亚像素精修和PnP。"""

    def __init__(
        self, intrinsics: Intrinsics, settings: DetectionSettings, *, prescaled: bool = False
    ) -> None:
        """派生原有检测内参；编译和运行时OpenCV必须与Python一致。"""
        if settings.tag_family != "tag36h11":
            raise ValueError("只支持 tag36h11")
        if native.opencv_version != cv2.__version__:
            raise RuntimeError("correction_native 与 Python OpenCV 版本不一致，请原生重建")
        self._scale = settings.image_scale
        self._camera_matrix = intrinsics.camera_matrix.copy()
        self._camera_matrix[0, :] *= self._scale
        self._camera_matrix[1, :] *= self._scale
        self._distortion = intrinsics.distortion.copy()
        # 相机已按相同INTER_AREA缩图时，仅缩放内参，不再二次缩像素。
        self._detector = native.Detector(
            1.0 if prescaled else self._scale, settings.corner_refinement
        )

    @property
    def camera_matrix(self) -> np.ndarray:
        """返回缩放图像对应内参副本，供合成测试与诊断使用。"""
        return self._camera_matrix.copy()

    def detect(self, gray: np.ndarray, tags: dict[int, TagPose]) -> DetectionBatch:
        """检测一帧；未知 Tag 仍出现在 marker_ids 中但不会被计算为候选。"""
        started = time.perf_counter()
        image = np.asarray(gray)
        if image.ndim != 2 or image.dtype != np.uint8:
            raise ValueError("AprilTag 输入必须是 mono8 图像")
        ids, corners = self._detector.detect(np.ascontiguousarray(image), set(tags))
        marker_ids = tuple(ids)
        detections: list[PoseDetection] = []
        for marker_id, marker_corners in zip(marker_ids, corners):
            tag = tags.get(marker_id)
            if tag is None:
                continue
            detection = self._estimate_pose(marker_id, marker_corners, tag.size_m)
            if detection is not None:
                detections.append(detection)
        return DetectionBatch(
            marker_ids=marker_ids,
            detections=tuple(detections),
            processing_time_ms=(time.perf_counter() - started) * 1000.0,
        )

    def _estimate_pose(
        self,
        marker_id: int,
        marker_corners: np.ndarray,
        size_m: float,
    ) -> PoseDetection | None:
        """以标准 +X右/+Y上/+Z朝观察者 Tag 系执行 IPPE square PnP。"""
        image_points = np.asarray(marker_corners, dtype=np.float64).reshape(4, 2)
        result = native.estimate_pose(
            image_points, size_m, self._camera_matrix, self._distortion,
        )
        if result is None:
            return None
        transform, reprojection_rms = result
        return PoseDetection(
            tag_id=marker_id,
            camera_from_tag_standard=transform,
            reprojection_error_px=reprojection_rms,
            corners=image_points,
        )
