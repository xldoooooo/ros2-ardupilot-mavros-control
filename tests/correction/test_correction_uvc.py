"""自有采集 ABI、真实采集时间和设备资源释放的回归；不连接飞控。"""

import ctypes as ct
import subprocess

import pytest

from correction_service import uvc_capture as uvc
from correction_service.config import load_config
from test_correction_service import CONFIG_DIR


def test_v4l2_abi_matches_system_headers(tmp_path):
    """与系统 C ABI 独立核对，防止 amd64/ARM64 ioctl 静默写错布局。"""
    source = tmp_path / "abi.c"
    source.write_text('''/* Compare the ctypes ABI against the actual Linux headers. */
#include <linux/videodev2.h>
#include <stddef.h>
#include <stdio.h>
int main(void) {
  printf("%zu %zu %zu %zu %zu %zu %zu %zu %lu %lu %lu %lu %lu %lu %lu %lu\\n",
    sizeof(struct v4l2_format), sizeof(struct v4l2_streamparm),
    sizeof(struct v4l2_requestbuffers), sizeof(struct v4l2_buffer),
    offsetof(struct v4l2_format, fmt), offsetof(struct v4l2_buffer, timestamp),
    offsetof(struct v4l2_buffer, m), offsetof(struct v4l2_buffer, length),
    VIDIOC_S_FMT, VIDIOC_S_PARM, VIDIOC_REQBUFS, VIDIOC_QUERYBUF,
    VIDIOC_QBUF, VIDIOC_DQBUF, VIDIOC_STREAMON, VIDIOC_STREAMOFF);
}
''')
    executable = tmp_path / "abi"
    subprocess.run(["cc", str(source), "-o", str(executable)], check=True)
    actual = list(map(int, subprocess.check_output([str(executable)], text=True).split()))
    expected = [ct.sizeof(cls) for cls in (uvc.Format, uvc.StreamParm, uvc.RequestBuffers, uvc.Buffer)]
    expected += [uvc.Format.fmt.offset, uvc.Buffer.timestamp.offset,
                 uvc.Buffer.m.offset, uvc.Buffer.length.offset]
    expected += [uvc.ioctl_request(number, cls) for number, cls in
                 ((5, uvc.Format), (22, uvc.StreamParm), (8, uvc.RequestBuffers),
                  (9, uvc.Buffer), (15, uvc.Buffer), (17, uvc.Buffer))]
    expected += [uvc.ioctl_request(number, ct.c_int, 1) for number in (18, 19)]
    assert actual == expected


def test_capture_clock_does_not_drift_with_negotiated_fps():
    """实际 60 Hz 连续 20 秒，即使声明 120 Hz，帧龄仍为真实的 12 ms。"""
    stamps = []
    for index in range(1200):
        capture_ns = 100_000_000_000 + index * 16_667_000
        buffer = uvc.Buffer(flags=uvc.TIMESTAMP_MONOTONIC)
        buffer.timestamp.seconds, fraction = divmod(capture_ns, 1_000_000_000)
        buffer.timestamp.microseconds = fraction // 1000
        stamp, age = uvc.capture_stamp_ns(buffer, capture_ns + 12_000_000,
                                        capture_ns + 12_000_000 + 1_700_000_000_000_000_000)
        assert age == 12_000_000
        stamps.append(stamp)
    assert all(b > a for a, b in zip(stamps, stamps[1:]))
    # 真正积压的帧必须保持积压年龄，绝不重新标为“现在”。
    assert uvc.capture_stamp_ns(buffer, capture_ns + 500_000_000, 2_000_000_000)[1] == 500_000_000
    buffer.flags = 0
    with pytest.raises(RuntimeError, match="CLOCK_MONOTONIC"):
        uvc.capture_stamp_ns(buffer, capture_ns, capture_ns)
    buffer.flags = uvc.TIMESTAMP_MONOTONIC
    with pytest.raises(RuntimeError, match="future"):
        uvc.capture_stamp_ns(buffer, capture_ns - 1, capture_ns)


def test_partial_initialization_closes_device(monkeypatch):
    """设备模式不匹配时不能泄漏打开的 UVC fd。"""
    closed = []
    monkeypatch.setattr(uvc.os, "open", lambda *args: 42)
    monkeypatch.setattr(uvc.os, "close", closed.append)
    def change_format(fd, number, argument, direction=3):
        argument.fmt.pix[0] = 320
    monkeypatch.setattr(uvc, "_ioctl", change_format)
    with pytest.raises(RuntimeError, match="size/format"):
        uvc.UvcCapture("/dev/video0", 1920, 1080, 120)
    assert closed == [42]


def test_streamoff_failure_still_releases_resources(monkeypatch):
    """断开设备导致 STREAMOFF 失败时仍关闭 mmap/fd，重复清理可用。"""
    closed = []
    capture = uvc.UvcCapture.__new__(uvc.UvcCapture)
    capture.fd, capture.streaming = 42, True
    class Storage:
        def close(self):
            closed.append("mmap")
    capture.buffers = [Storage()]
    def fail(*args):
        raise OSError("disconnected")
    monkeypatch.setattr(uvc, "_ioctl", fail)
    monkeypatch.setattr(uvc.os, "close", closed.append)
    with pytest.raises(OSError, match="disconnected"):
        capture.close()
    capture.close()
    assert closed == ["mmap", 42]


def test_old_external_driver_is_rejected(tmp_path):
    """现场遗留配置不能重新引入独立联合标定工作区。"""
    import shutil
    shutil.copytree(CONFIG_DIR, tmp_path, dirs_exist_ok=True)
    path = tmp_path / "camera.conf"
    path.write_text(path.read_text().replace("package = correction_service", "package = wasintek_gst_camera"))
    with pytest.raises(ValueError, match="本包"):
        load_config(tmp_path)


def test_launch_and_install_have_no_external_camera_overlay():
    """入口链不 source 外部工作区，也不间接加载历史 underlay。"""
    from pathlib import Path
    root = Path(__file__).resolve().parents[2]
    launcher = (root / "start_onboard_correction.sh").read_text()
    installer = (root / "correction_service/deploy/install_correction_service.sh").read_text()
    for text in (launcher, installer):
        assert "vins_odin_calib" not in text
        assert "CAMERA_OVERLAY" not in text
        assert "install/local_setup.bash" in text


@pytest.mark.parametrize("source_fps", [29.97, 59.94, 60.1, 120.0])
def test_rate_limiter_keeps_average_rate_without_frame_rounding_loss(source_fps):
    """59.94→30 不能因每次重置截止时间退化为约 20 Hz；低帧率不补帧。"""
    limiter = uvc.CaptureRateLimiter(30.0)
    stamps = [1_000_000_000 + round(i * 1e9 / source_fps) for i in range(round(source_fps * 20))]
    kept = [stamp for stamp in stamps if limiter.accept(stamp)]
    assert abs(len(kept) - min(len(stamps), 600)) <= 1
    assert set(kept).issubset(stamps)


def test_rate_limiter_skips_stalls_and_zero_disables_limit():
    """断流跳过遗漏节拍，只处理恢复后的新帧；0 保留全部原始帧。"""
    limiter = uvc.CaptureRateLimiter(30.0)
    assert limiter.accept(1_000_000_000)
    assert not limiter.accept(1_010_000_000)
    assert limiter.accept(9_010_000_000)
    assert not limiter.accept(9_011_000_000)
    assert limiter.accept(9_040_000_000)
    unlimited = uvc.CaptureRateLimiter(0.0)
    assert all(unlimited.accept(stamp) for stamp in range(1000, 1100))


@pytest.mark.parametrize("value", [-1.0, float("nan"), float("inf")])
def test_rate_limiter_and_camera_config_reject_invalid_rates(tmp_path, value):
    """入口与文件配置均拒绝负数及非有限限频值，避免启动后失效。"""
    import shutil
    with pytest.raises(ValueError, match="publish_fps"):
        uvc.CaptureRateLimiter(value)
    shutil.copytree(CONFIG_DIR, tmp_path, dirs_exist_ok=True)
    path = tmp_path / "camera.conf"
    path.write_text(path.read_text().replace("publish_fps = 30.0", f"publish_fps = {value}"))
    with pytest.raises(ValueError, match="publish_fps"):
        load_config(tmp_path)


def test_camera_rate_default_and_unlimited_preserve_native_mode(tmp_path):
    """缺省使用 30；0 关闭软件限频，但不改变相机原生请求 120。"""
    import shutil
    shutil.copytree(CONFIG_DIR, tmp_path, dirs_exist_ok=True)
    path = tmp_path / "camera.conf"
    original = path.read_text()
    path.write_text(original.replace("publish_fps = 30.0\n", ""))
    assert load_config(tmp_path).camera.publish_fps == 30.0
    path.write_text(original.replace("publish_fps = 30.0", "publish_fps = 0"))
    camera = load_config(tmp_path).camera
    assert camera.publish_fps == 0.0
    assert camera.fps == 120


def test_node_limits_before_decode_and_preserves_capture_stamps(monkeypatch):
    """运行真实采集循环的虚拟设备输入，核对解码次数、原始时间戳及退出释放。"""
    from types import SimpleNamespace
    import numpy as np
    from correction_service import uvc_camera_node as camera
    decoded, published, capture_times = [], [], []
    now = [0]
    class Capture:
        negotiated_fps = (120, 1)
        closed = False
        def __init__(self, device, width, height, fps):
            assert fps == 120
            self.index = 0
        def read_latest(self):
            if self.index == 120:
                raise KeyboardInterrupt
            stamp = 1_000_000_000 + round(self.index * 1e6 / 59.94) * 1000
            capture_times.append(stamp)
            now[0] = stamp + 5_000_000
            buffer = uvc.Buffer(flags=uvc.TIMESTAMP_MONOTONIC)
            buffer.timestamp.seconds, fraction = divmod(stamp, 1_000_000_000)
            buffer.timestamp.microseconds = fraction // 1000
            self.index += 1
            return b"jpeg", buffer
        def close(self):
            Capture.closed = True
    class Node:
        def __init__(self, name):
            pass
        def declare_parameter(self, key, default):
            return SimpleNamespace(value={"image_width": 2, "image_height": 2,
                                          "framerate": 120}.get(key, default))
        def get_parameter(self, key):
            return SimpleNamespace(value=False)
        def get_clock(self):
            return SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=now[0]))
        def get_logger(self):
            return SimpleNamespace(info=lambda text: None, fatal=lambda text: None)
        def create_publisher(self, *args):
            return SimpleNamespace(publish=published.append)
        def destroy_node(self):
            pass
    def decode(*args):
        decoded.append(now[0])
        return np.zeros((2, 2), dtype=np.uint8)
    monkeypatch.setattr(camera, "Node", Node)
    monkeypatch.setattr(camera, "UvcCapture", Capture)
    monkeypatch.setattr(camera.cv2, "imdecode", decode)
    monkeypatch.setattr(camera.rclpy, "init", lambda **kwargs: None)
    monkeypatch.setattr(camera.rclpy, "ok", lambda: True)
    monkeypatch.setattr(camera.rclpy, "shutdown", lambda: None)
    monkeypatch.setattr(camera.signal, "signal", lambda *args: None)
    monkeypatch.setattr(camera.time, "monotonic_ns", lambda: now[0])
    camera.main()
    assert len(decoded) == len(published) == 60
    assert Capture.closed
    assert all(msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
               in capture_times for msg in published)
