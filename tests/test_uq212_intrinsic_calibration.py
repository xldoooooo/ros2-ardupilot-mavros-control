"""UQ212 标定板旧约定、畸变恢复、生产格式与独立批次回归。"""
import importlib.util
from pathlib import Path

import cv2
import numpy as np
import pytest

from correction_service.config import _load_intrinsics, load_config

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


def test_standalone_camera_defaults_match_production():
    """单文件内置模式/镜头值必须与生产一致，避免日后仅一侧改动失配。"""
    config = load_config(ROOT / "correction_service/config")
    assert calib.CAMERA_DEVICE == config.camera.device
    assert (calib.FRAME_WIDTH, calib.FRAME_HEIGHT, calib.CAMERA_FPS) == (
        config.camera.width, config.camera.height, config.camera.fps)
    assert calib.LENS_CONTROLS == config.lens_controls


@pytest.mark.parametrize("points", [
    [[-.5, 10], [30, 10], [30, 40], [-.5, 40]],  # 原实现 x=-1，负切片导致空 ROI。
    [[-50, 10], [-20, 10], [-20, 40], [-50, 40]],
    [[90, 10], [101, 10], [101, 40], [90, 40]],
    [[10, 90], [30, 90], [30, 101], [10, 101]],
    [[float('nan'), 10], [30, 10], [30, 40], [10, 40]],
    [],
])
def test_invalid_corners_rejected_without_laplacian_crash(points):
    """板边移出图像、负细化坐标和空检测应拒绝本帧，不能崩溃。"""
    gray = np.zeros((100, 100), dtype=np.uint8)
    image = np.array(points, dtype=np.float32).reshape(-1, 1, 2)
    result = calib.sample_quality(gray, calib.marker_object_corners(0), image, 1, [], (100,100))
    assert result[0] is False and result[1] is None


def test_degenerate_board_geometry_rejected(monkeypatch):
    """求不出单应矩阵时拒绝本帧；恢复有效画面后仍能正常评估。"""
    gray = np.zeros((100, 100), dtype=np.uint8)
    image = np.array([[10,10], [40,10], [40,40], [10,40]], dtype=np.float32).reshape(-1,1,2)
    with monkeypatch.context() as patch:
        patch.setattr(cv2, 'findHomography', lambda *args: (None, None))
        result = calib.sample_quality(gray, calib.marker_object_corners(0), image, 1, [], (100,100))
        assert result[0] is False and result[1] is None
    result = calib.sample_quality(gray, calib.marker_object_corners(0), image, 1, [], (100,100))
    assert result[1] is not None


def test_interactive_rejects_invalid_force_capture_and_recovers(monkeypatch):
    """无效帧按空格不采纳，下一有效帧仍可采样，旧采样列表不被清空。"""
    class Camera:
        def read(self):
            return True, np.zeros((100,100,3), dtype=np.uint8)
    valid = np.array([[40,40],[10,40],[10,10],[40,10]], dtype=np.float32).reshape(1,4,2)
    invalid = valid.copy()
    invalid[0,1:3,0] = -.5
    frames = iter([valid, invalid, valid, valid])
    def detect(_):
        corners = next(frames)
        return calib.marker_object_corners(0), corners.reshape(-1,1,2), [corners], np.array([[0]])
    keys = iter([ord(' '), ord(' '), ord(' '), ord('q')])
    saved = []
    monkeypatch.setattr(calib, 'AUTO_CAPTURE', False)
    monkeypatch.setattr(calib, 'detect_board', detect)
    monkeypatch.setattr(calib, 'save_sample', lambda frame, index: saved.append(index))
    monkeypatch.setattr(cv2, 'imshow', lambda *args: None)
    monkeypatch.setattr(cv2, 'waitKey', lambda _: next(keys))
    calib.run_interactive(Camera(), False, (100,100), None)
    assert saved == [0,1]
