"""使用 OpenCV AprilTag 36h11 检测器生成带重投影残差的相机<-Tag 位姿。"""

from __future__ import annotations

import time
from dataclasses import dataclass

import cv2
import numpy as np

from .config import DetectionSettings, Intrinsics, TagPose
from .geometry import homogeneous


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
    """兼容开发机 OpenCV 4.6 与 Jetson OpenCV 4.14 的 36h11 检测器。"""

    def __init__(self, intrinsics: Intrinsics, settings: DetectionSettings) -> None:
        if settings.tag_family != "tag36h11":
            raise ValueError("只支持 tag36h11")
        self._scale = settings.image_scale
        self._camera_matrix = intrinsics.camera_matrix.copy()
        self._camera_matrix[0, :] *= self._scale
        self._camera_matrix[1, :] *= self._scale
        self._distortion = intrinsics.distortion.copy()
        self._corner_refinement = settings.corner_refinement
        self._dictionary = cv2.aruco.getPredefinedDictionary(
            cv2.aruco.DICT_APRILTAG_36h11
        )
        # Ubuntu 24.04 的 OpenCV 4.6 Python 绑定虽然暴露构造器，但直接调用会
        # 段错误；旧版必须走工厂函数，Jetson 4.14 则使用新构造器。
        parameter_factory = getattr(cv2.aruco, "DetectorParameters_create", None)
        parameters = (
            parameter_factory()
            if parameter_factory is not None
            else cv2.aruco.DetectorParameters()
        )
        parameters.markerBorderBits = 1
        parameters.minMarkerPerimeterRate = 0.01
        parameters.maxMarkerPerimeterRate = 0.9
        # 印刷纸张的额外外框可能被旧版 OpenCV 与真正 Tag 合并，并优先保留
        # 无法解码的大框。缩小候选合并距离（相对周长），保留内部编码黑边；
        # 不放宽字典纠错、黑边校验或后续 PnP/多 Tag 质量门。
        parameters.minMarkerDistanceRate = 0.02
        # 解码后按每个 Tag 的像素格尺寸精修。固定7px半径对小Tag会跨入
        # 相邻编码格/印刷外框，可能吸附错角点或回退为整数角点。
        parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_NONE
        self._parameters = parameters
        detector_class = getattr(cv2.aruco, "ArucoDetector", None)
        self._detector = (
            detector_class(self._dictionary, parameters)
            if detector_class is not None
            else None
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
        if self._scale != 1.0:
            image = cv2.resize(
                image,
                None,
                fx=self._scale,
                fy=self._scale,
                interpolation=cv2.INTER_AREA,
            )
        if self._detector is not None:
            corners, ids, _ = self._detector.detectMarkers(image)
        else:
            corners, ids, _ = cv2.aruco.detectMarkers(
                image,
                self._dictionary,
                parameters=self._parameters,
            )

        marker_ids = (
            () if ids is None else tuple(int(value) for value in ids.reshape(-1))
        )
        detections: list[PoseDetection] = []
        for marker_id, marker_corners in zip(marker_ids, corners):
            tag = tags.get(marker_id)
            if tag is None:
                continue
            if self._corner_refinement:
                marker_corners = np.asarray(marker_corners, dtype=np.float32).copy()
                points = marker_corners.reshape(4, 2)
                shortest_edge = float(np.min(np.linalg.norm(
                    points - np.roll(points, 1, axis=0), axis=1
                )))
                # 36h11为6×6编码加两侧各1格黑边，共8格；半径最多半格，
                # 且保留历史7px上限。窗口使用检测缩放图对应的像素单位。
                radius = max(1, min(7, int(shortest_edge / 16)))
                cv2.cornerSubPix(
                    image, marker_corners, (radius, radius), (-1, -1),
                    (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_MAX_ITER, 40, 0.01),
                )
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
        half = size_m * 0.5
        object_points = np.array(
            (
                (-half, half, 0.0),
                (half, half, 0.0),
                (half, -half, 0.0),
                (-half, -half, 0.0),
            ),
            dtype=np.float64,
        )
        image_points = np.asarray(marker_corners, dtype=np.float64).reshape(4, 2)
        solved, rotation_vector, translation_vector = cv2.solvePnP(
            object_points,
            image_points,
            self._camera_matrix,
            self._distortion,
            flags=cv2.SOLVEPNP_IPPE_SQUARE,
        )
        if not solved:
            return None
        translation = np.asarray(translation_vector, dtype=np.float64).reshape(3)
        if not np.isfinite(translation).all() or translation[2] <= 0.0:
            return None
        rotation, _ = cv2.Rodrigues(rotation_vector)
        projected, _ = cv2.projectPoints(
            object_points,
            rotation_vector,
            translation_vector,
            self._camera_matrix,
            self._distortion,
        )
        residual = projected.reshape(4, 2) - image_points
        reprojection_rms = float(np.sqrt(np.mean(np.sum(residual * residual, axis=1))))
        if not np.isfinite(rotation).all() or not np.isfinite(reprojection_rms):
            return None
        return PoseDetection(
            tag_id=marker_id,
            camera_from_tag_standard=homogeneous(rotation, translation),
            reprojection_error_px=reprojection_rms,
            corners=image_points,
        )
