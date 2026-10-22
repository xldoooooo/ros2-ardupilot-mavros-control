"""Check shared lens geometry and the software relay's recording timebase."""

from argparse import Namespace
from fractions import Fraction
import signal

import av
import cv2
import numpy as np
import pytest
import yaml

from video_service.relay_calibration import Calibration
from video_service.relay_cpu import run_cpu


def calibration_file(tmp_path):
    """Use a small camera with nonzero radial and tangential distortion."""
    path = tmp_path / 'camera.yaml'
    path.write_text(yaml.safe_dump({
        'image_width': 128, 'image_height': 72, 'distortion_model': 'plumb_bob',
        'camera_matrix': {'data': [100, 0, 63, 0, 101, 35, 0, 0, 1]},
        'distortion_coefficients': {'data': [-0.2, 0.03, 0.001, -0.002, 0.01]},
    }))
    return path


def test_calibration_matches_opencv_and_rejects_nan(tmp_path):
    """Verify the model independently and reject coefficients that produce invalid pixels."""
    path = calibration_file(tmp_path)
    camera = Calibration(path, 'left')
    reference = cv2.initUndistortRectifyMap(camera.matrix, camera.distortion, None,
                                          camera.target, (128, 72), cv2.CV_32FC1)
    actual = camera.maps()
    np.testing.assert_allclose(actual[0], np.clip(reference[0], 0, 127), atol=0.0001)
    np.testing.assert_allclose(actual[1], np.clip(reference[1], 0, 71), atol=0.0001)
    # Chroma positions must sample the camera model at their luma-space locations.
    xx, yy = 20, 10
    point = np.array([[(2*xx-camera.target[0, 2])/camera.target[0, 0],
                       (2*yy+.5-camera.target[1, 2])/camera.target[1, 1], 1.0]])
    projected, _ = cv2.projectPoints(point, np.zeros(3), np.zeros(3), camera.matrix, camera.distortion)
    cx, cy = camera.maps(True)
    np.testing.assert_allclose([cx[yy, xx], cy[yy, xx]],
                               [(projected[0, 0, 0])/2, (projected[0, 0, 1]-.5)/2], atol=0.0001)
    data = yaml.safe_load(path.read_text())
    data['camera_matrix']['data'][0] = float('nan')
    path.write_text(yaml.safe_dump(data))
    with pytest.raises(ValueError, match='finite'):
        Calibration(path, 'left')


@pytest.mark.parametrize('recording', (False, True))
def test_cpu_keeps_source_gaps_and_full_range(tmp_path, recording):
    """An input gap must survive encoding rather than becoming fabricated continuous video."""
    source, output = tmp_path / 'source.mp4', tmp_path / 'corrected.mp4'
    with av.open(str(source), 'w') as container:
        stream = container.add_stream('libx264', rate=60)
        stream.width, stream.height, stream.pix_fmt = 128, 72, 'yuv420p'
        stream.codec_context.time_base = Fraction(1, 90000)
        stream.codec_context.color_range = 2
        stream.codec_context.max_b_frames = 0
        for pts in (0, 1500, 3000, 48000, 49500):
            frame = av.VideoFrame.from_ndarray(np.full((108, 128), 128, np.uint8), format='yuv420p')
            frame.pts, frame.time_base, frame.color_range = pts, Fraction(1, 90000), 2
            container.mux(stream.encode(frame))
        container.mux(stream.encode())
    camera = Calibration(calibration_file(tmp_path), 'left')
    recorded = tmp_path / "录 制 | : ' [test].mkv"
    progress = tmp_path / 'progress.txt'
    args = Namespace(input=str(source), output=str(output), cpu_threads=2, bitrate_value=1000000,
                     gop=60, chroma_location='left', duration=None, frames=None, progress=progress,
                     recording=recorded if recording else None)
    handlers = {sig: signal.getsignal(sig) for sig in (signal.SIGINT, signal.SIGTERM)}
    try:
        run_cpu(args, camera)
    finally:
        for sig, handler in handlers.items():
            signal.signal(sig, handler)
    with av.open(str(source)) as container:
        expected = [frame.time for frame in container.decode(video=0)]
    with av.open(str(output)) as container:
        frames = list(container.decode(video=0))
    np.testing.assert_allclose([frame.time for frame in frames], expected, atol=1/90000)
    assert len(frames) == 5
    assert all(frame.color_range == 2 for frame in frames)
    assert abs(float(frames[0].to_ndarray()[:72].mean()) - 128) <= 1
    assert 'frame=5\nout_time_us=550000\nprogress=end' in progress.read_text()
    if recording:
        with av.open(str(recorded)) as container:
            saved_frames = list(container.decode(video=0))
        assert len(saved_frames) == len(frames)
        # Matroska uses milliseconds; both outputs retain the source's half-second gap.
        np.testing.assert_allclose([frame.time for frame in saved_frames], expected, atol=0.001)
        assert all(frame.color_range == 2 for frame in saved_frames)
        with av.open(str(recorded)) as container:
            saved_packets = [bytes(packet) for packet in container.demux(video=0) if packet.size]
        with av.open(str(output)) as container:
            main_packets = [bytes(packet) for packet in container.demux(video=0) if packet.size]
        assert saved_packets == main_packets
