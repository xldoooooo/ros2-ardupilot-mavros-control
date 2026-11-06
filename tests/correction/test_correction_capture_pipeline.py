"""验证采集与解码解耦、最新压缩帧覆盖和唯一采样节拍；不连接硬件。"""

import queue
import threading
from types import SimpleNamespace

import pytest

from correction_service.uvc_capture import LatestFrameCapture
from correction_service.node import CorrectionServiceNode


def test_capture_keeps_latest_compressed_frame_while_consumer_is_busy():
    """下游不取帧时继续采集，仅返回最新 JPEG 和原始时间戳对象。"""
    supplied = queue.Queue()
    samples = [(b"first", object()), (b"second", object()), (b"latest", object())]

    class Capture:
        negotiated_fps = (60, 1)
        closed = False

        def read_latest(self):
            """设备读取有界等待，让真实采集线程独立运行。"""
            try:
                return supplied.get(timeout=.02)
            except queue.Empty:
                return None

        def close(self):
            """记录设备释放，供验证线程先退出。"""
            self.closed = True

    device = Capture()
    reader = LatestFrameCapture(device)
    try:
        for index, sample in enumerate(samples, start=1):
            supplied.put(sample)
            with reader._condition:
                assert reader._condition.wait_for(lambda: reader.frames_received == index, 1.)
        assert reader.frames_received == 3
        assert reader.read_latest() is samples[-1]
        assert reader.read_latest(timeout=.03) is None
    finally:
        reader.close()
    assert device.closed
    assert not reader._thread.is_alive()


def test_capture_error_is_forwarded_and_device_is_released():
    """设备异常立即唤醒消费端，并在退出时释放采集设备。"""
    class Capture:
        negotiated_fps = (60, 1)
        closed = False

        def read_latest(self):
            """模拟读取失败，不伪造旧帧维持输出。"""
            raise OSError("disconnected")

        def close(self):
            """记录设备关闭。"""
            self.closed = True

    device = Capture()
    reader = LatestFrameCapture(device)
    try:
        with pytest.raises(RuntimeError, match="disconnected"):
            reader.read_latest()
    finally:
        reader.close()
    assert device.closed


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
