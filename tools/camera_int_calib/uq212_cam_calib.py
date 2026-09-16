#!/usr/bin/env python3
"""UQ212 内参标定，改编自 refresh 的 wainstek_cam_calib.py（2026-08-27）。

沿用同一块 6x6 AprilGrid 与原始角点次序；固定原生 MJPEG 1920x1080@120，
复用项目 UVC 采集和镜头控制。使用项目 .venv，详见同目录 README.md。
A 自动采样；空格强制采样；C 标定；R 新建采样批次；U 去畸变预览；S 保存；Q 退出。
所有输出保存在独立时间戳目录，不自动应用到生产配置。
"""

import argparse
import math
import logging
import subprocess
import sys
from datetime import datetime
from pathlib import Path
import time

import yaml

import cv2
import numpy as np


# 脚本可通过机载 camera_int_calib 目录的软链接运行；resolve 保持仓库根路径正确。
PROJECT_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(PROJECT_ROOT / "correction_service"))
from correction_service.config import load_config
from correction_service.camera_process import CameraProcess
from correction_service.uvc_capture import UvcCapture

# ============================== 用户配置 ==============================

# 请求相机输出的分辨率；相机必须原生支持该模式。
FRAME_WIDTH = 1920
FRAME_HEIGHT = 1080
CAMERA_FPS = 120

# 25～40 张通常足够。自动采样只收录清晰且与已有样本差异明显的画面。
TARGET_SAMPLES = 30
MIN_SAMPLES_TO_CALIBRATE = 15
AUTO_CAPTURE = True

# 固定五参数模型（k1,k2,p1,p2,k3），全部参与估计，不强制畸变为零。

# 自动样本质量阈值。高分辨率清晰图通常可把 MIN_SHARPNESS 提高到 120～250。
MIN_SHARPNESS = 80.0
MIN_BOARD_AREA_RATIO = 0.025
MAX_BOARD_AREA_RATIO = 0.72
MIN_TAGS_PER_VIEW = 10
MIN_CAPTURE_INTERVAL = 0.55
MIN_DIVERSITY_DISTANCE = 0.85
STABLE_DETECTIONS_REQUIRED = 2
MAX_CORNER_MOTION_PX = 1.8

DETECT_EVERY_N_FRAMES = 1  # 每个显示帧均检测，手动采样不得使用上帧角点。
DISPLAY_SCALE = 0.72
OUTPUT_ROOT = Path.home() / "camera_int_calib" / "uq212_runs"
OUTPUT_YAML = None  # 仅开始新一批采样后分配，避免覆盖旧相机结果。
CAPTURE_METADATA = {}
# ====================================================================

# 这部分严格对应用户提供的唯一标定板，不作为可选配置。
TAG_COLS = 6
TAG_ROWS = 6
TAG_SIZE = 0.055       # m
TAG_GAP = 0.0165       # m，两个 Tag 黑色外框之间的净间距
TAG_PITCH = TAG_SIZE + TAG_GAP
FIRST_TAG_ID = 0
LAST_TAG_ID = 35
BOARD_WIDTH = (TAG_COLS - 1) * TAG_PITCH + TAG_SIZE
BOARD_HEIGHT = (TAG_ROWS - 1) * TAG_PITCH + TAG_SIZE

WINDOW_NAME = "OpenCV camera calibration (Q quit, A auto, SPACE capture, C calibrate)"


TAG_DICTIONARY = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
# OpenCV 4.6 使用工厂函数；新版本兼容新构造器。
_parameter_factory = getattr(cv2.aruco, "DetectorParameters_create", None)
DETECTOR_PARAMETERS = (_parameter_factory() if _parameter_factory else cv2.aruco.DetectorParameters())
DETECTOR_PARAMETERS.markerBorderBits = 2
DETECTOR_PARAMETERS.minMarkerPerimeterRate = 0.01
DETECTOR_PARAMETERS.maxMarkerPerimeterRate = 0.8
DETECTOR_PARAMETERS.adaptiveThreshWinSizeMin = 3
DETECTOR_PARAMETERS.adaptiveThreshWinSizeMax = 73
DETECTOR_PARAMETERS.adaptiveThreshWinSizeStep = 10
DETECTOR_PARAMETERS.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
DETECTOR_PARAMETERS.cornerRefinementWinSize = 9
DETECTOR_PARAMETERS.cornerRefinementMaxIterations = 50
DETECTOR_PARAMETERS.cornerRefinementMinAccuracy = 0.005
TAG_DETECTOR = (cv2.aruco.ArucoDetector(TAG_DICTIONARY, DETECTOR_PARAMETERS)
                if hasattr(cv2.aruco, "ArucoDetector") else None)


def marker_object_corners(marker_id):
    """
    返回与 OpenCV detectMarkers 四角顺序对应的板面坐标。

    这块实物板的 ID 从左下角 0 开始，向右、再向上递增；所有 Tag 相对
    板面方向一致。下面的角点次序已用用户提供的整板照片逐角验证。
    """
    index = marker_id - FIRST_TAG_ID
    col = index % TAG_COLS
    row = index // TAG_COLS
    x = col * TAG_PITCH
    y = row * TAG_PITCH
    return np.array(
        [
            [x + TAG_SIZE, y, 0.0],
            [x, y, 0.0],
            [x, y + TAG_SIZE, 0.0],
            [x + TAG_SIZE, y + TAG_SIZE, 0.0],
        ],
        dtype=np.float32,
    )


def detect_board(gray):
    """兼容新旧 OpenCV，仅返回本板 ID 0~35 的原生分辨率角点。"""
    marker_corners, marker_ids, _ = (
        TAG_DETECTOR.detectMarkers(gray) if TAG_DETECTOR is not None else
        cv2.aruco.detectMarkers(gray, TAG_DICTIONARY, parameters=DETECTOR_PARAMETERS)
    )
    if marker_ids is None:
        return None

    valid = []
    for corners, marker_id in zip(marker_corners, marker_ids.reshape(-1)):
        marker_id = int(marker_id)
        if FIRST_TAG_ID <= marker_id <= LAST_TAG_ID:
            valid.append((marker_id, corners.astype(np.float32)))

    if not valid:
        return None

    valid.sort(key=lambda item: item[0])
    ids = np.array([[marker_id] for marker_id, _ in valid], dtype=np.int32)
    corners = [item[1] for item in valid]
    object_points = np.concatenate([marker_object_corners(marker_id) for marker_id, _ in valid], axis=0)
    image_points = np.concatenate([item[1].reshape(4, 2) for item in valid], axis=0).reshape(-1, 1, 2)
    return object_points, image_points, corners, ids


def board_descriptor(object_points, image_points, image_size):
    """用位置、大小、旋转和透视变化描述一张样本，供自动去重。"""
    width, height = image_size
    flat = image_points.reshape(-1, 2)
    center = flat.mean(axis=0)
    area_ratio = cv2.contourArea(cv2.convexHull(flat)) / float(width * height)

    homography, _ = cv2.findHomography(object_points[:, :2], flat)
    board_corners = np.array(
        [[0.0, 0.0], [BOARD_WIDTH, 0.0], [0.0, BOARD_HEIGHT], [BOARD_WIDTH, BOARD_HEIGHT]],
        dtype=np.float32,
    ).reshape(-1, 1, 2)
    projected = cv2.perspectiveTransform(board_corners, homography).reshape(4, 2)
    p00, p01, p10, p11 = projected
    top = np.linalg.norm(p01 - p00)
    bottom = np.linalg.norm(p11 - p10)
    left = np.linalg.norm(p10 - p00)
    right = np.linalg.norm(p11 - p01)
    rotation = math.atan2(float(p01[1] - p00[1]), float(p01[0] - p00[0]))
    perspective_x = (top - bottom) / (top + bottom + 1e-9)
    perspective_y = (left - right) / (left + right + 1e-9)

    return np.array(
        [
            center[0] / width,
            center[1] / height,
            math.sqrt(max(area_ratio, 1e-9)),
            rotation,
            perspective_x,
            perspective_y,
        ],
        dtype=np.float64,
    ), area_ratio


def descriptor_distance(a, b):
    """按位置、尺度、面内转角和透视变化计算姿态差异。"""
    angle = abs(a[3] - b[3])
    angle = min(angle, 2.0 * math.pi - angle)
    delta = np.array(
        [
            (a[0] - b[0]) / 0.16,
            (a[1] - b[1]) / 0.16,
            (a[2] - b[2]) / 0.10,
            angle / math.radians(24.0),
            (a[4] - b[4]) / 0.10,
            (a[5] - b[5]) / 0.10,
        ]
    )
    return float(np.linalg.norm(delta))


def sample_quality(gray, object_points, image_points, tag_count, descriptors, image_size):
    """沿用旧板清晰度、可见面积、Tag 数和姿态去重门限。"""
    x, y, w, h = cv2.boundingRect(image_points.reshape(-1, 2))
    board_roi = gray[y : y + h, x : x + w]
    sharpness = float(cv2.Laplacian(board_roi, cv2.CV_64F).var())
    descriptor, area_ratio = board_descriptor(object_points, image_points, image_size)

    if tag_count < MIN_TAGS_PER_VIEW:
        return False, descriptor, sharpness, area_ratio, "need more tags"
    if sharpness < MIN_SHARPNESS:
        return False, descriptor, sharpness, area_ratio, "too blurry"
    if area_ratio < MIN_BOARD_AREA_RATIO:
        return False, descriptor, sharpness, area_ratio, "board too small"
    if area_ratio > MAX_BOARD_AREA_RATIO:
        return False, descriptor, sharpness, area_ratio, "board too large"

    if descriptors:
        nearest = min(descriptor_distance(descriptor, old) for old in descriptors)
        if nearest < MIN_DIVERSITY_DISTANCE:
            return False, descriptor, sharpness, area_ratio, "change pose / position"

    return True, descriptor, sharpness, area_ratio, "good sample"


def calibrate(object_points, image_points, image_size):
    """使用当前全部视图执行一次标定，不筛选、不删帧。"""
    flags = 0  # 五参数模型；不固定焦距、主点或任意畸变系数。
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_COUNT, 150, 1e-12)
    result = cv2.calibrateCameraExtended(
        object_points,
        image_points,
        image_size,
        None,
        None,
        flags=flags,
        criteria=criteria,
    )

    rms, camera_matrix, dist_coeffs, rvecs, tvecs, std_intrinsics, _, per_view = result
    per_view = np.asarray(per_view, dtype=np.float64).reshape(-1)
    all_indices = list(range(len(image_points)))

    return {
        "rms": float(rms),
        "camera_matrix": camera_matrix,
        "dist_coeffs": dist_coeffs,
        "rvecs": rvecs,
        "tvecs": tvecs,
        "std_intrinsics": std_intrinsics,
        "per_view_errors": per_view,
        "kept_indices": all_indices,
    }


def save_calibration(path, image_size, calibration):
    """保存 OpenCV 原始结果和生产兼容 YAML，但绝不写活动 config/。"""
    path = Path(path)
    fs = cv2.FileStorage(str(path), cv2.FILE_STORAGE_WRITE)
    if not fs.isOpened():
        raise OSError(f"cannot write {path}")
    fs.write("image_width", image_size[0])
    fs.write("image_height", image_size[1])
    fs.write("camera_name", "UQ212")
    fs.write("distortion_model", "plumb_bob")
    fs.write("camera_matrix", calibration["camera_matrix"])
    fs.write("distortion_coefficients", calibration["dist_coeffs"])
    fs.write("rms_reprojection_error", calibration["rms"])
    fs.write("per_view_reprojection_errors", calibration["per_view_errors"])
    fs.write("intrinsic_parameter_stddev", calibration["std_intrinsics"])
    fs.write("target_type", "aprilgrid")
    fs.write("tag_family", "tag36h11")
    fs.write("tag_cols", TAG_COLS)
    fs.write("tag_rows", TAG_ROWS)
    fs.write("tag_size_m", TAG_SIZE)
    fs.write("tag_gap_m", TAG_GAP)
    fs.write("tag_spacing_ratio", TAG_GAP / TAG_SIZE)
    fs.write("first_tag_id", FIRST_TAG_ID)
    fs.write("last_tag_id", LAST_TAG_ID)
    fs.write("used_views", len(calibration["kept_indices"]))
    fs.release()
    k = np.asarray(calibration["camera_matrix"])
    d = np.asarray(calibration["dist_coeffs"]).reshape(-1)
    data = {
        "image_width": image_size[0], "image_height": image_size[1],
        "distortion_model": "plumb_bob",
        "camera_matrix": {"rows": 3, "cols": 3, "data": k.reshape(-1).tolist()},
        "distortion_coefficients": {"rows": 1, "cols": len(d), "data": d.tolist()},
        "rms_reprojection_error": calibration["rms"],
        "calibration_status": "calibrated_pending_independent_validation",
        "calibration_tag_family": "tag36h11",
        "used_views": len(calibration["kept_indices"]),
        "per_view_reprojection_errors": calibration["per_view_errors"].tolist(),
        "capture": CAPTURE_METADATA,
    }
    if len(d) != 5:
        raise ValueError("UQ212 production export requires five-coefficient plumb_bob model")
    with (path.parent / "intrinsics.yaml").open("w") as stream:
        stream.write("# UQ212 AprilGrid 标定结果；原始畸变图像对应的 K/D，尚待独立验证。\n")
        yaml.safe_dump(data, stream, sort_keys=False, allow_unicode=True)
    print(f"saved: {path} and {path.parent / 'intrinsics.yaml'}")


def load_calibration(path):
    """加载 OpenCV 结果或生产格式，拒绝分辨率错误的预览。"""
    text = Path(path).read_text()
    if not text.startswith("%YAML"):
        data = yaml.safe_load(text)
        return (int(data["image_width"]), int(data["image_height"])), {
            "camera_matrix": np.array(data["camera_matrix"]["data"]).reshape(3, 3),
            "dist_coeffs": np.array(data["distortion_coefficients"]["data"]),
            "rms": float(data["rms_reprojection_error"]),
        }
    fs = cv2.FileStorage(str(path), cv2.FILE_STORAGE_READ)
    if not fs.isOpened():
        raise ValueError(f"cannot load {path}")
    image_size = (
        int(fs.getNode("image_width").real()),
        int(fs.getNode("image_height").real()),
    )
    calibration = {
        "camera_matrix": fs.getNode("camera_matrix").mat(),
        "dist_coeffs": fs.getNode("distortion_coefficients").mat(),
        "rms": float(fs.getNode("rms_reprojection_error").real()),
    }
    fs.release()
    print(f"loaded calibration: {path}")
    print(f"resolution: {image_size[0]} x {image_size[1]}, RMS={calibration['rms']:.4f}px")
    return image_size, calibration


def print_calibration(calibration, image_size):
    """原样报告总残差和最差视图，不仅展示平均值。"""
    k = calibration["camera_matrix"]
    d = calibration["dist_coeffs"].reshape(-1)
    errors = calibration["per_view_errors"]
    print("\n========== CALIBRATION RESULT ==========")
    print(f"resolution: {image_size[0]} x {image_size[1]}")
    print(f"views used: {len(calibration['kept_indices'])}")
    print(f"global RMS: {calibration['rms']:.4f} px")
    print(f"view RMS: mean={errors.mean():.4f}, max={errors.max():.4f} px")
    print("camera matrix K:\n", np.array2string(k, precision=8, suppress_small=True))
    print("distortion D:\n", np.array2string(d, precision=8, suppress_small=True))
    print("========================================\n")


def evaluate_calibration(object_points, image_points, descriptors, image_size):
    """使用全部样本直接标定、保存并生成去畸变映射。"""
    if len(image_points) < MIN_SAMPLES_TO_CALIBRATE:
        raise ValueError("need at least 15 views")
    # 保留所有观测，后续可原样重算；不按残差删帧、美化结果。
    observations = {"image_size": np.array(image_size)}
    for index, (obj, img) in enumerate(zip(object_points, image_points)):
        observations[f"object_{index:03d}"] = obj
        observations[f"image_{index:03d}"] = img
    np.savez_compressed(Path(OUTPUT_YAML).parent / "observations.npz", **observations)
    calibration = calibrate(object_points, image_points, image_size)
    if (not np.isfinite(calibration["camera_matrix"]).all()
            or not np.isfinite(calibration["dist_coeffs"]).all()
            or not math.isfinite(calibration["rms"])
            or min(calibration["camera_matrix"][0, 0], calibration["camera_matrix"][1, 1]) <= 0):
        raise ValueError("calibration returned invalid parameters")

    kept = calibration["kept_indices"]
    object_points = [object_points[i] for i in kept]
    image_points = [image_points[i] for i in kept]
    descriptors = [descriptors[i] for i in kept]
    calibration["kept_indices"] = list(range(len(kept)))
    print_calibration(calibration, image_size)
    save_calibration(OUTPUT_YAML, image_size, calibration)

    new_k, _ = cv2.getOptimalNewCameraMatrix(
        calibration["camera_matrix"], calibration["dist_coeffs"], image_size, 0.0, image_size
    )
    map1, map2 = cv2.initUndistortRectifyMap(
        calibration["camera_matrix"],
        calibration["dist_coeffs"],
        None,
        new_k,
        image_size,
        cv2.CV_16SC2,
    )
    return object_points, image_points, descriptors, calibration, map1, map2


def corner_motion(previous, marker_corners, marker_ids):
    """比较共同 Tag 的角点位移，要求连续画面停稳后自动采样。"""
    current = {
        int(marker_id): corners.reshape(4, 2)
        for corners, marker_id in zip(marker_corners, marker_ids.reshape(-1))
    }
    common = previous.keys() & current.keys()
    if len(common) < 4:
        return float("inf"), current
    motions = [np.linalg.norm(current[marker_id] - previous[marker_id], axis=1) for marker_id in common]
    return float(np.median(np.concatenate(motions))), current


def draw_coverage(image, descriptors):
    """绘制九宫格和已采样中心，帮助用户覆盖画面边缘。"""
    height, width = image.shape[:2]
    for x in (width // 3, 2 * width // 3):
        cv2.line(image, (x, 0), (x, height), (70, 70, 70), 1)
    for y in (height // 3, 2 * height // 3):
        cv2.line(image, (0, y), (width, y), (70, 70, 70), 1)
    for descriptor in descriptors:
        p = (int(descriptor[0] * width), int(descriptor[1] * height))
        cv2.circle(image, p, 5, (0, 220, 255), -1, cv2.LINE_AA)


def put_text(image, text, line, color=(255, 255, 255)):
    """在预览画面叠加清晰可读的采样与求解状态。"""
    scale = max(0.55, image.shape[1] / 1900.0)
    y = int(30 + line * 31 * scale)
    cv2.putText(image, text, (16, y), cv2.FONT_HERSHEY_SIMPLEX, scale, (0, 0, 0), 4, cv2.LINE_AA)
    cv2.putText(image, text, (16, y), cv2.FONT_HERSHEY_SIMPLEX, scale, color, 1, cv2.LINE_AA)


def new_run():
    """每次启动/重置新建目录；PNG、角点和两种结果格式属于同一批次。"""
    global OUTPUT_YAML
    directory = OUTPUT_ROOT / datetime.now().strftime("%Y%m%d-%H%M%S-%f")
    directory.mkdir(parents=True, exist_ok=False)
    OUTPUT_YAML = directory / "camera_calibration.yaml"
    print(f"output directory: {directory}")


def save_sample(frame, index):
    """保留实际用于检测的原始图像，显示缩放和去畸变不参与标定。"""
    path = Path(OUTPUT_YAML).parent / f"view-{index:03d}.png"
    if not cv2.imwrite(str(path), frame):
        raise OSError(f"cannot write {path}")


class Uq212Camera:
    """单设备原生 UVC 采集，与生产链共用镜头设置，不依赖 ROS 节点。"""

    def __init__(self, config):
        self.capture = None
        busy = subprocess.run(["fuser", config.camera.device], capture_output=True)
        if busy.returncode == 0:
            raise RuntimeError("camera is busy; stop correction sampling before calibration")
        if busy.returncode != 1:
            raise RuntimeError("cannot determine whether camera is busy")
        try:
            self.capture = UvcCapture(config.camera.device, FRAME_WIDTH, FRAME_HEIGHT, CAMERA_FPS)
            num, den = self.capture.negotiated_fps
            if den <= 0 or abs(num / den - CAMERA_FPS) > 0.1:
                raise RuntimeError(f"camera negotiated unexpected FPS: {num}/{den}")
            self.read()  # 开流后再恢复并读回镜头参数，匹配生产时序。
            controls = CameraProcess(config.camera, config.lens_controls, Path("/dev/null"),
                                     logging.getLogger("uq212_calib")).apply_lens_controls()
            CAPTURE_METADATA.update(device=config.camera.device, width=FRAME_WIDTH,
                                    height=FRAME_HEIGHT, fps=num/den, pixel_format="MJPG",
                                    lens_controls=controls, opencv_version=cv2.__version__)
            # 自动曝光稳定前的帧不进入标定样本。
            settle_until = time.monotonic() + 1.0
            while time.monotonic() < settle_until:
                self.read()
        except BaseException:
            self.release()
            raise

    def read(self):
        """排空旧帧后只解码最新 MJPEG；连续超时/尺寸不符明确报错。"""
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline:
            item = self.capture.read_latest()
            if item is None:
                continue
            payload, _ = item
            frame = cv2.imdecode(np.frombuffer(payload, dtype=np.uint8), cv2.IMREAD_COLOR)
            if frame is None:
                continue
            if frame.shape[:2] != (FRAME_HEIGHT, FRAME_WIDTH):
                raise RuntimeError(f"unexpected image shape: {frame.shape}")
            return True, frame
        raise RuntimeError("no valid MJPEG image for 3 seconds")

    def release(self):
        """异常或正常退出均释放 mmap 与设备 fd。"""
        if self.capture is not None:
            self.capture.close()
            self.capture = None


def main():
    """解析参数；检查采集或启动原脚本的交互流程。"""
    global OUTPUT_ROOT
    parser = argparse.ArgumentParser(description="UQ212 same-board AprilGrid intrinsic calibration")
    parser.add_argument("--preview-yaml", metavar="PATH", help="preview OpenCV or production YAML")
    parser.add_argument("--output-dir", type=Path, default=OUTPUT_ROOT, help="parent of unique run directories")
    parser.add_argument("--check-camera", action="store_true", help="read 30 frames and lens controls without GUI/calibration")
    args = parser.parse_args()
    OUTPUT_ROOT = args.output_dir.expanduser().resolve()
    config = load_config(PROJECT_ROOT / "correction_service/config")
    if (config.camera.width, config.camera.height, config.camera.fps) != (FRAME_WIDTH, FRAME_HEIGHT, CAMERA_FPS):
        parser.error("production camera mode differs from UQ212 1920x1080@120")
    if "UQ212" not in config.camera.device:
        parser.error("active camera is not UQ212")
    preview_mode = args.preview_yaml is not None
    requested_size, loaded_calibration = ((FRAME_WIDTH, FRAME_HEIGHT), None)
    if preview_mode:
        requested_size, loaded_calibration = load_calibration(args.preview_yaml)
        if requested_size != (FRAME_WIDTH, FRAME_HEIGHT):
            parser.error("preview YAML must be 1920x1080; refusing silent rescaling")
    cap = Uq212Camera(config)
    try:
        if args.check_camera:
            for _ in range(30):
                cap.read()
            print(yaml.safe_dump(CAPTURE_METADATA, sort_keys=False))
            print("PASS: 30 frames at 1920x1080 decoded; controls verified; no calibration performed")
            return
        if not preview_mode:
            new_run()
        cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
        run_interactive(cap, preview_mode, requested_size, loaded_calibration)
    finally:
        cap.release()
        cv2.destroyAllWindows()


def run_interactive(cap, preview_mode, requested_size, loaded_calibration):
    """沿用旧脚本的覆盖提示、稳定性与姿态去重，保留全部被采纳的样本。"""
    object_points = []
    image_points = []
    descriptors = []
    auto_capture = AUTO_CAPTURE and not preview_mode
    last_capture_time = 0.0
    frame_number = 0
    current_object_points = None
    current_image_points = None
    current_marker_corners = []
    current_marker_ids = None
    current_tag_count = 0
    current_descriptor = None
    current_sharpness = 0.0
    current_area = 0.0
    previous_marker_map = {}
    stable_detections = 0
    current_motion = float("inf")
    quality_message = "YAML loaded; press U to compare" if preview_mode else "show the AprilGrid"
    calibration = loaded_calibration
    undistort = preview_mode
    map1 = map2 = None
    image_size = requested_size

    while True:
        ok, frame = cap.read()
        if not ok:
            break

        height, width = frame.shape[:2]
        image_size = (width, height)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        frame_number += 1

        if preview_mode and map1 is None:
            new_k, _ = cv2.getOptimalNewCameraMatrix(
                calibration["camera_matrix"], calibration["dist_coeffs"], image_size, 0.0, image_size
            )
            map1, map2 = cv2.initUndistortRectifyMap(
                calibration["camera_matrix"],
                calibration["dist_coeffs"],
                None,
                new_k,
                image_size,
                cv2.CV_16SC2,
            )

        if frame_number % DETECT_EVERY_N_FRAMES == 0 and not undistort and not preview_mode:
            detection = detect_board(gray)
            if detection is None:
                current_object_points = None
                current_image_points = None
                current_marker_corners = []
                current_marker_ids = None
                current_tag_count = 0
                previous_marker_map = {}
                stable_detections = 0
                current_motion = float("inf")
                quality_message = "AprilGrid not found"
            else:
                (
                    current_object_points,
                    current_image_points,
                    current_marker_corners,
                    current_marker_ids,
                ) = detection
                current_tag_count = len(current_marker_ids)
                current_motion, current_marker_map = corner_motion(
                    previous_marker_map, current_marker_corners, current_marker_ids
                )
                if not previous_marker_map:
                    stable_detections = 1
                elif current_motion <= MAX_CORNER_MOTION_PX:
                    stable_detections += 1
                else:
                    stable_detections = 1
                previous_marker_map = current_marker_map

                good, current_descriptor, current_sharpness, current_area, quality_message = sample_quality(
                    gray,
                    current_object_points,
                    current_image_points,
                    current_tag_count,
                    descriptors,
                    image_size,
                )
                if good and stable_detections < STABLE_DETECTIONS_REQUIRED:
                    good = False
                    quality_message = f"hold still {stable_detections}/{STABLE_DETECTIONS_REQUIRED}"
                now = time.monotonic()
                if auto_capture and good and now - last_capture_time >= MIN_CAPTURE_INTERVAL:
                    save_sample(frame, len(image_points))
                    object_points.append(current_object_points.copy())
                    image_points.append(current_image_points.copy())
                    descriptors.append(current_descriptor.copy())
                    last_capture_time = now
                    quality_message = "CAPTURED"

                    if len(image_points) >= TARGET_SAMPLES:
                        (
                            object_points,
                            image_points,
                            descriptors,
                            calibration,
                            map1,
                            map2,
                        ) = evaluate_calibration(object_points, image_points, descriptors, image_size)
                        auto_capture = False
                        undistort = True
                        quality_message = "CALIBRATION COMPLETE"

        if undistort and calibration is not None:
            view = cv2.remap(frame, map1, map2, cv2.INTER_LINEAR)
        else:
            view = frame.copy()
            draw_coverage(view, descriptors)
            if current_marker_ids is not None:
                cv2.aruco.drawDetectedMarkers(view, current_marker_corners, current_marker_ids)

        status_color = (0, 255, 0) if quality_message in ("good sample", "CAPTURED") else (0, 180, 255)
        if preview_mode:
            put_text(view, f"YAML PREVIEW: {'UNDISTORTED' if undistort else 'RAW'}", 0, (80, 255, 80))
            put_text(view, "U: toggle raw/undistorted   Q: quit", 1)
        else:
            put_text(view, f"samples: {len(image_points)}/{TARGET_SAMPLES}   auto: {'ON' if auto_capture else 'OFF'}", 0)
            put_text(
                view,
                f"{quality_message}   tags={current_tag_count}/36   motion={current_motion:.1f}px   sharpness={current_sharpness:.0f}",
                1,
                status_color,
            )
        put_text(view, f"actual resolution: {width}x{height}", 2)
        if calibration is not None:
            put_text(
                view,
                f"CALIBRATED  RMS={calibration['rms']:.3f}px   preview={'UNDISTORTED' if undistort else 'RAW'}",
                3,
                (80, 255, 80),
            )

        if DISPLAY_SCALE != 1.0:
            shown = cv2.resize(view, None, fx=DISPLAY_SCALE, fy=DISPLAY_SCALE, interpolation=cv2.INTER_AREA)
        else:
            shown = view
        cv2.imshow(WINDOW_NAME, shown)

        key = cv2.waitKey(1) & 0xFF
        if key in (ord("q"), 27):
            break
        if key == ord("a") and not preview_mode:
            auto_capture = not auto_capture
        elif key == ord(" ") and not preview_mode and not undistort and current_image_points is not None:
            descriptor, _ = board_descriptor(current_object_points, current_image_points, image_size)
            save_sample(frame, len(image_points))
            object_points.append(current_object_points.copy())
            image_points.append(current_image_points.copy())
            descriptors.append(descriptor)
            last_capture_time = time.monotonic()
            quality_message = "FORCE CAPTURED"
            if len(image_points) >= TARGET_SAMPLES:
                (
                    object_points,
                    image_points,
                    descriptors,
                    calibration,
                    map1,
                    map2,
                ) = evaluate_calibration(object_points, image_points, descriptors, image_size)
                auto_capture = False
                undistort = True
                quality_message = "CALIBRATION COMPLETE"
        elif key == ord("c") and len(image_points) >= MIN_SAMPLES_TO_CALIBRATE:
            (
                object_points,
                image_points,
                descriptors,
                calibration,
                map1,
                map2,
            ) = evaluate_calibration(object_points, image_points, descriptors, image_size)
            auto_capture = False
            undistort = True
            quality_message = "CALIBRATION COMPLETE"
        elif key == ord("u") and calibration is not None:
            undistort = not undistort
        elif key == ord("s") and calibration is not None and not preview_mode:
            save_calibration(OUTPUT_YAML, image_size, calibration)
        elif key == ord("r") and not preview_mode:
            new_run()
            object_points.clear()
            image_points.clear()
            descriptors.clear()
            calibration = None
            map1 = map2 = None
            undistort = False
            auto_capture = AUTO_CAPTURE
            previous_marker_map = {}
            stable_detections = 0
            current_motion = float("inf")
            quality_message = "reset"



if __name__ == "__main__":
    main()
