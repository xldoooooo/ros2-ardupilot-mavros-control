"""UQ212 标定板旧约定、畸变恢复、生产格式与独立批次回归。"""
import importlib.util
from pathlib import Path

import cv2
import numpy as np

from correction_service.config import _load_intrinsics

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location(
    "uq212_calib", ROOT / "tools/camera_int_calib/uq212_cam_calib.py"
)
calib = importlib.util.module_from_spec(spec)
spec.loader.exec_module(calib)


def test_same_board_layout_and_detector():
    """旧实物板 ID 0 左下起排、55mm 黑框、16.5mm 净距、两位黑边保持不变。"""
    assert np.allclose(calib.marker_object_corners(0),
                       [[.055, 0, 0], [0, 0, 0], [0, .055, 0], [.055, .055, 0]])
    assert np.allclose(calib.marker_object_corners(35),
                       [[.4125, .3575, 0], [.3575, .3575, 0],
                        [.3575, .4125, 0], [.4125, .4125, 0]])
    # 36 个独立图案，按实物排布和旋转摆放；同时验证 OpenCV 4.6/新版本兼容。
    board = np.full((900, 900), 255, dtype=np.uint8)
    for tag in range(36):
        if hasattr(cv2.aruco, "generateImageMarker"):
            marker = cv2.aruco.generateImageMarker(calib.TAG_DICTIONARY, tag, 100, borderBits=2)
        else:
            marker = cv2.aruco.drawMarker(calib.TAG_DICTIONARY, tag, 100, borderBits=2)
        col, row = tag % 6, tag // 6
        y, x = 50 + (5-row)*140, 50 + col*140
        board[y:y+100, x:x+100] = cv2.rotate(marker, cv2.ROTATE_180)
    objects, images, _, ids = calib.detect_board(board)
    assert ids.reshape(-1).tolist() == list(range(36))
    # ID0 四点应为图像右下、左下、左上、右上；独立于待测 object mapping。
    assert np.allclose(images[:4, 0], [[149,849], [50,849], [50,750], [149,750]], atol=1)
    assert objects.shape == (144, 3)


def test_calibration_recovers_distortion_and_exports_compatible_yaml(tmp_path, monkeypatch):
    """已知非零径向/切向畸变的多姿态数据须恢复，且结果能被生产加载器读取。"""
    rng = np.random.default_rng(77)
    k = np.array([[1125., 0, 951.], [0, 1116., 546.], [0, 0, 1.]])
    d = np.array([-.08, .015, .001, -.0007, -.003])
    objects = np.concatenate([calib.marker_object_corners(i) for i in range(36)])
    object_views, image_views = [], []
    for i in range(24):
        r = rng.uniform([-.6, -.5, -.5], [.6, .5, .5])
        t = rng.uniform([-.35, -.32, .6], [.05, .04, 1.1])
        pixels, _ = cv2.projectPoints(objects, r, t, k, d)
        pixels += rng.normal(0, .03, pixels.shape)
        object_views.append(objects.copy())
        image_views.append(pixels.astype(np.float32))
    result = calib.calibrate(object_views, image_views, (1920, 1080))
    assert result['rms'] < .06
    assert np.max(np.abs(result['camera_matrix']-k)) < .6
    assert np.allclose(result['dist_coeffs'].reshape(-1), d, atol=.005)
    assert result['kept_indices'] == list(range(24))
    calib.save_calibration(tmp_path/'camera_calibration.yaml', (1920,1080), result)
    parsed = _load_intrinsics(tmp_path)
    assert np.allclose(parsed.camera_matrix, result['camera_matrix'])
    assert np.allclose(parsed.distortion, result['dist_coeffs'].reshape(-1))
    for name in ('camera_calibration.yaml', 'intrinsics.yaml'):
        size, loaded = calib.load_calibration(tmp_path/name)
        assert size == (1920,1080)
        assert np.allclose(loaded['camera_matrix'], parsed.camera_matrix)
    monkeypatch.setattr(calib, 'OUTPUT_ROOT', tmp_path)
    calib.new_run()
    first = calib.OUTPUT_YAML
    calib.new_run()
    assert calib.OUTPUT_YAML != first and first.parent.is_dir()
