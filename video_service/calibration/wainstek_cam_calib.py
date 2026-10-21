#!/usr/bin/env python3
"""
使用指定 6x6 tag36h11 AprilGrid 的纯 OpenCV 实时相机内参标定。

依赖：pip install opencv-contrib-python numpy

使用：
1. 按需修改下方相机分辨率。
2. 使用固定标定板：6x6 tag36h11，ID 0~35，Tag=5.5cm，间隔=1.65cm。
3. 运行：python3 realtime_camera_calibration.py
4. 缓慢移动标定板，使其覆盖画面四角、中央，并改变距离、俯仰、偏航和旋转角。

直接加载已有 YAML 预览去畸变：
  python3 realtime_camera_calibration.py --preview-yaml camera_calibration.yaml
4. 收集完成后会自动标定并写出 camera_calibration.yaml。

按键：
  A      开关自动采样
  空格   强制加入当前检测结果
  C      立即标定
  R      清空并重新采样
  U      标定后切换原图/去畸变预览
  S      再次保存结果
  Q/ESC  退出
"""

import argparse
import math
import time

import cv2
import numpy as np


# ============================== 用户配置 ==============================
CAMERA_ID = 2

# 请求相机输出的分辨率；相机必须原生支持该模式。
FRAME_WIDTH = 1920
FRAME_HEIGHT = 1080
CAMERA_FPS = 30

# 25～40 张通常足够。自动采样只收录清晰且与已有样本差异明显的画面。
TARGET_SAMPLES = 30
MIN_SAMPLES_TO_CALIBRATE = 15
AUTO_CAPTURE = True

# 普通镜头建议 False（k1,k2,p1,p2,k3）；明显广角镜头可改为 True（增加 k4,k5,k6）。
USE_RATIONAL_MODEL = False

# 自动样本质量阈值。高分辨率清晰图通常可把 MIN_SHARPNESS 提高到 120～250。
MIN_SHARPNESS = 80.0
MIN_BOARD_AREA_RATIO = 0.025
MAX_BOARD_AREA_RATIO = 0.72
MIN_TAGS_PER_VIEW = 10
MIN_CAPTURE_INTERVAL = 0.55
MIN_DIVERSITY_DISTANCE = 0.85
STABLE_DETECTIONS_REQUIRED = 2
MAX_CORNER_MOTION_PX = 1.8

DETECT_EVERY_N_FRAMES = 2
DISPLAY_SCALE = 0.72
OUTPUT_YAML = "camera_calibration.yaml"
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
# OpenCV 4.6 requires its factory and module-level marker detection API.
DETECTOR_PARAMETERS = cv2.aruco.DetectorParameters_create()
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
    marker_corners, marker_ids, _ = cv2.aruco.detectMarkers(
        gray, TAG_DICTIONARY, parameters=DETECTOR_PARAMETERS
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
    """Evaluate the original quality criteria using an image-clipped board region."""
    x, y, w, h = cv2.boundingRect(image_points.reshape(-1, 2))
    # Refined edge corners can be negative; NumPy negative slices wrap from the end.
    height, width = gray.shape[:2]
    board_roi = gray[max(0, y):max(0, min(height, y + h)),
                     max(0, x):max(0, min(width, x + w))]
    if board_roi.size == 0:
        return False, None, 0.0, 0.0, "board outside image"
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
    flags = cv2.CALIB_RATIONAL_MODEL if USE_RATIONAL_MODEL else 0
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
    fs = cv2.FileStorage(path, cv2.FILE_STORAGE_WRITE)
    fs.write("image_width", image_size[0])
    fs.write("image_height", image_size[1])
    fs.write("camera_name", "camera")
    fs.write("distortion_model", "rational_polynomial" if USE_RATIONAL_MODEL else "plumb_bob")
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
    print(f"saved: {path}")


def load_calibration(path):
    fs = cv2.FileStorage(path, cv2.FILE_STORAGE_READ)
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
    calibration = calibrate(object_points, image_points, image_size)

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
    height, width = image.shape[:2]
    for x in (width // 3, 2 * width // 3):
        cv2.line(image, (x, 0), (x, height), (70, 70, 70), 1)
    for y in (height // 3, 2 * height // 3):
        cv2.line(image, (0, y), (width, y), (70, 70, 70), 1)
    for descriptor in descriptors:
        p = (int(descriptor[0] * width), int(descriptor[1] * height))
        cv2.circle(image, p, 5, (0, 220, 255), -1, cv2.LINE_AA)


def put_text(image, text, line, color=(255, 255, 255)):
    scale = max(0.55, image.shape[1] / 1900.0)
    y = int(30 + line * 31 * scale)
    cv2.putText(image, text, (16, y), cv2.FONT_HERSHEY_SIMPLEX, scale, (0, 0, 0), 4, cv2.LINE_AA)
    cv2.putText(image, text, (16, y), cv2.FONT_HERSHEY_SIMPLEX, scale, color, 1, cv2.LINE_AA)


def main():
    parser = argparse.ArgumentParser(description="Realtime AprilGrid camera calibration and YAML undistortion preview")
    parser.add_argument("--preview-yaml", metavar="PATH", help="load an existing calibration YAML and preview undistortion")
    args = parser.parse_args()

    preview_mode = args.preview_yaml is not None
    if preview_mode:
        requested_size, loaded_calibration = load_calibration(args.preview_yaml)
    else:
        requested_size = (FRAME_WIDTH, FRAME_HEIGHT)
        loaded_calibration = None

    cap = cv2.VideoCapture(CAMERA_ID)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, requested_size[0])
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, requested_size[1])
    cap.set(cv2.CAP_PROP_FPS, CAMERA_FPS)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)

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
                if current_descriptor is None:
                    current_image_points = None
                if good and stable_detections < STABLE_DETECTIONS_REQUIRED:
                    good = False
                    quality_message = f"hold still {stable_detections}/{STABLE_DETECTIONS_REQUIRED}"
                now = time.monotonic()
                if auto_capture and good and now - last_capture_time >= MIN_CAPTURE_INTERVAL:
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
        if key == ord("a"):
            auto_capture = not auto_capture
        elif key == ord(" ") and current_image_points is not None:
            descriptor, _ = board_descriptor(current_object_points, current_image_points, image_size)
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

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
