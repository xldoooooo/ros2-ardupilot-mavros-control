"""验证检测直接消费调度后的新帧，不引入第二次采样节拍。"""

import queue
import threading

import cv2
import numpy as np
import pytest
import _correction_native as native
from types import SimpleNamespace

from correction_service.node import CorrectionServiceNode


def test_detector_consumes_all_selected_frames_without_second_rate_gate():
    """调度后的新帧即使间隔略小于100ms，也逐帧处理且不重复采样。"""
    stamps = [1_000_000_000, 1_099_000_000, 1_099_000_000, 1_198_000_000]
    images = queue.Queue()
    for stamp in stamps:
        sec, nanosec = divmod(stamp, 1_000_000_000)
        images.put(SimpleNamespace(
            header=SimpleNamespace(stamp=SimpleNamespace(sec=sec, nanosec=nanosec))
        ))
    processed = []
    job = SimpleNamespace(
        started_monotonic=0.,
        last_image_stamp_ns=0.,
        snapshot=SimpleNamespace(diverged=False, converged=False),
    )
    closing = threading.Event()

    def process(_job, _image, stamp):
        """记录实际处理，并在最后一张唯一新帧后结束循环。"""
        processed.append(stamp)
        if stamp == stamps[-1]:
            closing.set()

    node = SimpleNamespace(
        _closing=closing,
        _image_queue=images,
        config=SimpleNamespace(timeouts=SimpleNamespace(ground_max_runtime_seconds=0.)),
        _raise_if_stopping=lambda _job: None,
        _check_first_tag_timeout=lambda _job: None,
        _process_image=process,
    )
    camera = SimpleNamespace(ensure_running=lambda: None)
    CorrectionServiceNode._sample_until_candidate(node, job, camera)
    assert processed == [stamps[0], stamps[1], stamps[3]]


@pytest.mark.parametrize("shape,scale", [((1200, 1920), .5), ((121, 193), .6), ((1200, 1920), 1.)])
def test_camera_resize_preserves_original_detection_pixels(shape, scale):
    """前移缩图逐像素等价，非整数尺寸按原fx/fy取整，原分辨率不改像素。"""
    image = np.random.default_rng(19).integers(0, 256, shape, dtype=np.uint8)
    expected = cv2.resize(image, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
    assert np.array_equal(native.resize_gray(image, scale), expected)
