#!/usr/bin/env python3
"""Calibrate Wasintek intrinsics using the existing 6x6 tag36h11 AprilGrid.

Uses the project environment and production video settings; saves runs separately.
The original board geometry, sampling thresholds and five-coefficient model remain.
"""

import argparse
import json
import math
import os
from pathlib import Path
import re
import subprocess
import sys
import time
from datetime import datetime
from fractions import Fraction

# Re-enter the project environment when launched directly from the aircraft desktop.
if __name__ == "__main__":
    candidates = [Path(__file__).resolve().parents[2] / ".venv",
                  Path.home() / "ros2-ardupilot-mavros-control/.venv"]
    environment = next((p for p in candidates if (p / "bin/python").is_file()), None)
    if environment is None:
        raise SystemExit("Project .venv not found; use ros2-ardupilot-mavros-control/.venv")
    if Path(sys.prefix).resolve() != environment.resolve():
        os.execv(str(environment / "bin/python"),
                 [str(environment / "bin/python"), str(Path(__file__).resolve()), *sys.argv[1:]])

import cv2
import numpy as np

# Locate the production parsers for both the repository and aircraft standalone copy.
PROJECT_ROOT = next((root for root in (Path(__file__).resolve().parents[2],
                                     Path.home() / "ros2-ardupilot-mavros-control")
                     if (root / "video_service/camera_app/onboard_config.py").is_file()), None)
if PROJECT_ROOT is None:
    raise SystemExit("Cannot find project video_service/camera_app/onboard_config.py")
sys.path.insert(0, str(PROJECT_ROOT / "video_service"))
from camera_app.onboard_config import load_lens_controls, load_onboard_settings


# ============================== Camera configuration ==============================

# Collect distinct, sharp views; allow manual calibration after 15 samples.
TARGET_SAMPLES = 30
MIN_SAMPLES_TO_CALIBRATE = 15
AUTO_CAPTURE = True

# Keep the original five-coefficient model unless rational calibration is requested.
USE_RATIONAL_MODEL = False

# Original sharpness, board coverage and pose diversity thresholds.
MIN_SHARPNESS = 80.0
MIN_BOARD_AREA_RATIO = 0.025
MAX_BOARD_AREA_RATIO = 0.72
MIN_TAGS_PER_VIEW = 10
MIN_CAPTURE_INTERVAL = 0.55
MIN_DIVERSITY_DISTANCE = 0.85
STABLE_DETECTIONS_REQUIRED = 2
MAX_CORNER_MOTION_PX = 1.8

# Detection cadence and display size do not change the calibration coordinates.
DETECT_EVERY_N_FRAMES = 2
DISPLAY_SCALE = 0.72
# Each interactive run gets its own directory; old lens results remain intact.
OUTPUT_ROOT = Path.home() / "camera_int_calib/wainstek_runs"
OUTPUT_YAML = None
# Record the lens state alongside K/D; production configuration is never overwritten.
CAPTURE_METADATA = {}
# ====================================================================

# Fixed physical board: IDs increase rightwards from bottom-left, then upwards.
TAG_COLS = 6
TAG_ROWS = 6
TAG_SIZE = 0.055       # m
TAG_GAP = 0.0165       # m between the black outer borders
TAG_PITCH = TAG_SIZE + TAG_GAP
FIRST_TAG_ID = 0
LAST_TAG_ID = 35
BOARD_WIDTH = (TAG_COLS - 1) * TAG_PITCH + TAG_SIZE
BOARD_HEIGHT = (TAG_ROWS - 1) * TAG_PITCH + TAG_SIZE

# Shared HighGUI window title and the original AprilTag detector tuning.
WINDOW_NAME = "OpenCV camera calibration (Q quit, A auto, SPACE capture, C calibrate)"


TAG_DICTIONARY = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
# OpenCV 4.6 exposes a constructor that segfaults; use its supported factory.
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
    """Return the original physical board corners in OpenCV marker corner order."""
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
    """Detect the original board with either the OpenCV 4.6 or newer API."""
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
    """Describe position, size, rotation and perspective for sample deduplication."""
    width, height = image_size
    flat = image_points.reshape(-1, 2)
    center = flat.mean(axis=0)
    area_ratio = cv2.contourArea(cv2.convexHull(flat)) / float(width * height)

    homography, _ = cv2.findHomography(object_points[:, :2], flat)
    if homography is None or not np.isfinite(homography).all():
        raise ValueError("invalid board geometry")
    board_corners = np.array(
        [[0.0, 0.0], [BOARD_WIDTH, 0.0], [0.0, BOARD_HEIGHT], [BOARD_WIDTH, BOARD_HEIGHT]],
        dtype=np.float32,
    ).reshape(-1, 1, 2)
    projected = cv2.perspectiveTransform(board_corners, homography).reshape(4, 2)
    if not np.isfinite(projected).all():
        raise ValueError("invalid board projection")
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
    """Measure the original normalized pose diversity distance."""
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
    """Check sharpness and coverage without accepting invalid edge geometry."""
    flat = np.asarray(image_points).reshape(-1, 2)
    height, width = gray.shape[:2]
    if (gray.size == 0 or len(flat) < 4 or not np.isfinite(flat).all()
            or np.any(flat < 0) or np.any(flat[:, 0] >= width)
            or np.any(flat[:, 1] >= height)):
        return False, None, 0.0, 0.0, "invalid board corners"
    x, y, w, h = cv2.boundingRect(flat)
    board_roi = gray[max(0, y):min(height, y+h), max(0, x):min(width, x+w)]
    if board_roi.size == 0:
        return False, None, 0.0, 0.0, "empty board region"
    try:
        descriptor, area_ratio = board_descriptor(object_points, image_points, image_size)
    except (cv2.error, ValueError):
        return False, None, 0.0, 0.0, "invalid board geometry"
    sharpness = float(cv2.Laplacian(board_roi, cv2.CV_64F).var())

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
    """Calibrate using every accepted view without filtering or dropping samples."""
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
    """Save OpenCV K/D and the capture state in the current run directory."""
    fs = cv2.FileStorage(path, cv2.FILE_STORAGE_WRITE)
    if not fs.isOpened():
        raise RuntimeError(f"Cannot write calibration: {path}")
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
    fs.write("capture_metadata", json.dumps(CAPTURE_METADATA, sort_keys=True))
    fs.release()
    print(f"saved: {path}")


def load_calibration(path):
    """Load an OpenCV calibration without silently changing its resolution."""
    fs = cv2.FileStorage(path, cv2.FILE_STORAGE_READ)
    if not fs.isOpened():
        raise ValueError(f"Cannot read calibration: {path}")
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
    if min(image_size) <= 0 or calibration["camera_matrix"] is None or calibration["dist_coeffs"] is None:
        raise ValueError(f"Invalid OpenCV calibration YAML: {path}")
    print(f"loaded calibration: {path}")
    print(f"resolution: {image_size[0]} x {image_size[1]}, RMS={calibration['rms']:.4f}px")
    return image_size, calibration


def print_calibration(calibration, image_size):
    """Print K/D and global and per-view reprojection errors."""
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
    """Calibrate all samples, save the result and build undistortion maps."""
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
    """Measure median corner motion across tags visible in both detections."""
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
    """Draw the original sample coverage grid and accepted sample centers."""
    height, width = image.shape[:2]
    for x in (width // 3, 2 * width // 3):
        cv2.line(image, (x, 0), (x, height), (70, 70, 70), 1)
    for y in (height // 3, 2 * height // 3):
        cv2.line(image, (0, y), (width, y), (70, 70, 70), 1)
    for descriptor in descriptors:
        p = (int(descriptor[0] * width), int(descriptor[1] * height))
        cv2.circle(image, p, 5, (0, 220, 255), -1, cv2.LINE_AA)


def put_text(image, text, line, color=(255, 255, 255)):
    """Draw a readable status line over the camera image."""
    scale = max(0.55, image.shape[1] / 1900.0)
    y = int(30 + line * 31 * scale)
    cv2.putText(image, text, (16, y), cv2.FONT_HERSHEY_SIMPLEX, scale, (0, 0, 0), 4, cv2.LINE_AA)
    cv2.putText(image, text, (16, y), cv2.FONT_HERSHEY_SIMPLEX, scale, color, 1, cv2.LINE_AA)


def open_camera(device, image_size, fps, lens_config, codec):
    """Open the production camera mode and restore its lens control sequence."""
    if not Path(device).exists():
        raise RuntimeError(f"Camera not found: {device}; check /dev/v4l/by-id")
    busy = subprocess.run(["fuser", device], capture_output=True)
    if busy.returncode == 0:
        raise RuntimeError("Camera is busy; close video capture before calibration")
    cap = cv2.VideoCapture()
    try:
        fourcc = {"h264": "H264", "mjpeg": "MJPG"}[codec]
        if codec == "h264":
            # V4L2 VideoCapture can return H.264 bytes in an image-shaped buffer.
            # Decode explicitly; keep the production device, mode and lens settings.
            rate = Fraction(fps).limit_denominator(10000)
            pipeline = (
                f"v4l2src device={json.dumps(device)} ! video/x-h264,"
                f"width={image_size[0]},height={image_size[1]},"
                f"framerate={rate.numerator}/{rate.denominator} ! "
                "h264parse ! openh264dec ! videoconvert ! video/x-raw,format=BGR ! "
                "appsink max-buffers=1 drop=true sync=false"
            )
            if not cap.open(pipeline, cv2.CAP_GSTREAMER):
                raise RuntimeError("Cannot decode camera H.264; check OpenCV GStreamer support and "
                                   "gstreamer1.0-plugins-good / gstreamer1.0-plugins-bad")
        else:
            if not cap.open(device, cv2.CAP_V4L2):
                raise RuntimeError(f"Cannot open camera: {device}")
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*fourcc))
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, image_size[0])
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, image_size[1])
            cap.set(cv2.CAP_PROP_FPS, fps)
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        actual_size = (int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)), int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)))
        actual_fps = cap.get(cv2.CAP_PROP_FPS)
        if (actual_size != image_size or abs(actual_fps - fps) > 0.1
                or (codec == "mjpeg" and int(cap.get(cv2.CAP_PROP_FOURCC)) != cv2.VideoWriter_fourcc(*fourcc))):
            raise RuntimeError(f"Unexpected camera mode: {actual_size}@{actual_fps}; requested {fourcc} {image_size}@{fps}")
        # Wasintek resets some controls when the stream starts.
        settle_until = time.monotonic() + 1.0
        while time.monotonic() < settle_until:
            read_frame(cap, image_size)
        controls = load_lens_controls(lens_config)
        actual_controls = {}
        for name, value in controls:
            subprocess.run(["v4l2-ctl", "-d", device, "--set-ctrl", f"{name}={int(value)}"],
                           check=True, capture_output=True, text=True, timeout=3)
            # Match CameraController._apply_lens_controls on the aircraft.
            if name in {"auto_exposure", "exposure_time_absolute"}:
                time.sleep(0.2)
        for name, value in controls:
            result = subprocess.run(["v4l2-ctl", "-d", device, "--get-ctrl", name],
                                    check=True, capture_output=True, text=True, timeout=3)
            match = re.search(r":\s*(-?\d+)", result.stdout)
            if match is None or int(match.group(1)) != int(value):
                raise RuntimeError(f"Lens control readback differs: {name}: {result.stdout.strip()}")
            actual_controls[name] = int(match.group(1))
        CAPTURE_METADATA.update(device=device, width=actual_size[0], height=actual_size[1],
                                fps=actual_fps, pixel_format=fourcc, lens_controls=actual_controls,
                                capture_backend=cap.getBackendName(),
                                decoder="openh264dec" if codec == "h264" else "OpenCV MJPEG",
                                lens_config=str(lens_config), opencv_version=cv2.__version__,
                                python_environment=sys.prefix)
        for _ in range(5):
            read_frame(cap, image_size)
        print(f"Camera ready: {device}, {fourcc} {actual_size[0]}x{actual_size[1]}@{actual_fps}")
        return cap
    except BaseException:
        cap.release()
        raise


def read_frame(cap, image_size):
    """Reject read failures or frame sizes that would invalidate K/D."""
    ok, frame = cap.read()
    if not ok or frame is None:
        raise RuntimeError("Camera returned no image; check USB and close other camera clients")
    if frame.shape[1::-1] != image_size:
        raise RuntimeError(f"Unexpected image size: {frame.shape[1::-1]}, expected {image_size}")
    return frame


def main():
    """Run the camera check or the original interactive AprilGrid calibration."""
    global OUTPUT_YAML
    # Follow start_onboard_video.sh: explicit environment, then /etc, then repository.
    camera_config = Path(os.environ.get("VIDEO_SERVICE_ONBOARD_CONFIG") or
                         ("/etc/ros2-ardupilot/camera.conf" if Path("/etc/ros2-ardupilot/camera.conf").is_file()
                          else PROJECT_ROOT / "video_service/config/camera.conf"))
    lens_config = Path(os.environ.get("VIDEO_SERVICE_LENS_CONFIG") or
                       ("/etc/ros2-ardupilot/lens.conf" if Path("/etc/ros2-ardupilot/lens.conf").is_file()
                        else PROJECT_ROOT / "video_service/config/lens.conf"))
    parser = argparse.ArgumentParser(description="Realtime AprilGrid camera calibration and YAML undistortion preview")
    parser.add_argument("--preview-yaml", metavar="PATH", help="load an existing calibration YAML and preview undistortion")
    parser.add_argument("--camera-config", type=Path, default=camera_config, help="production camera.conf")
    parser.add_argument("--lens-config", type=Path, default=lens_config, help="production lens.conf")
    parser.add_argument("--output-dir", type=Path, default=OUTPUT_ROOT, help="parent of unique run directories")
    parser.add_argument("--check-camera", action="store_true", help="check 30 frames and board detection without a window or calibration")
    args = parser.parse_args()
    settings = load_onboard_settings(args.camera_config).camera
    requested_size = (settings.width, settings.height)

    preview_mode = args.preview_yaml is not None
    if preview_mode:
        yaml_size, loaded_calibration = load_calibration(args.preview_yaml)
        if yaml_size != requested_size:
            parser.error(f"Preview YAML size {yaml_size} differs from production size {requested_size}")
    else:
        loaded_calibration = None

    if not args.check_camera and not os.environ.get("DISPLAY") and not os.environ.get("WAYLAND_DISPLAY"):
        parser.error("No desktop display; run in an aircraft desktop terminal or use --check-camera over SSH")
    cap = open_camera(settings.device, requested_size, settings.fps, args.lens_config, settings.codec)
    CAPTURE_METADATA["camera_config"] = str(args.camera_config)
    try:
        if args.check_camera:
            max_tags = 0
            for _ in range(30):
                frame = read_frame(cap, requested_size)
                detection = detect_board(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY))
                if detection is not None:
                    max_tags = max(max_tags, len(detection[3]))
            print(json.dumps(CAPTURE_METADATA, indent=2))
            print(f"Image mean={frame.mean():.2f}, std={frame.std():.2f}; "
                  f"bottom-half mean={frame[requested_size[1] // 2:].mean():.2f}")
            print(f"PASS: 30 frames decoded and detector ran; max board tags={max_tags}/36; no calibration performed")
            return
        if not preview_mode:
            run_dir = args.output_dir.expanduser() / datetime.now().strftime("%Y%m%d-%H%M%S-%f")
            run_dir.mkdir(parents=True)
            OUTPUT_YAML = str(run_dir / "camera_calibration.yaml")
            print(f"Calibration output: {OUTPUT_YAML}")
        cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
        run_interactive(cap, preview_mode, requested_size, loaded_calibration)
    finally:
        cap.release()
        if not args.check_camera:
            cv2.destroyAllWindows()


def run_interactive(cap, preview_mode, requested_size, loaded_calibration):
    """Keep the original sampling thresholds, board geometry and calibration model."""
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
        frame = read_frame(cap, requested_size)

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
        if key == ord("a") and not preview_mode:
            auto_capture = not auto_capture
        elif key == ord(" ") and current_image_points is not None and not undistort:
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

if __name__ == "__main__":
    try:
        main()
    except (RuntimeError, ValueError, OSError, subprocess.SubprocessError, cv2.error) as exc:
        raise SystemExit(f"Calibration error: {exc}") from exc
