"""Check shared lens geometry and the software relay's recording timebase."""

from argparse import Namespace
from fractions import Fraction
import json
import signal
import subprocess

import av
import cv2
import numpy as np
import pytest
import yaml

from video_service.relay_calibration import Calibration
from video_service.relay_cpu import run_cpu
from video_service.ground_relay import build_command, frame_step


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
    assert camera.target[0, 0] / camera.matrix[0, 0] == pytest.approx(
        camera.target[1, 1] / camera.matrix[1, 1])
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
@pytest.mark.parametrize('fps', (None, 30))
@pytest.mark.integration
def test_cpu_keeps_source_gaps_and_full_range(tmp_path, recording, fps):
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
                     recording=recorded if recording else None, fps=fps)
    handlers = {sig: signal.getsignal(sig) for sig in (signal.SIGINT, signal.SIGTERM)}
    try:
        run_cpu(args, camera)
    finally:
        for sig, handler in handlers.items():
            signal.signal(sig, handler)
    with av.open(str(source)) as container:
        expected = [frame.time for frame in container.decode(video=0)]
    if fps:
        expected = [expected[index] for index in (0, 2, 3)]
    with av.open(str(output)) as container:
        frames = list(container.decode(video=0))
    np.testing.assert_allclose([frame.time for frame in frames], expected, atol=1/90000)
    assert len(frames) == len(expected)
    assert all(frame.color_range == 2 for frame in frames)
    assert abs(float(frames[0].to_ndarray()[:72].mean()) - 128) <= 1
    assert f'frame={len(expected)}\nout_time_us={round(expected[-1] * 1000000)}\nprogress=end' in progress.read_text()
    if fps and not recording:
        # Exercise the exact GPU selector on CPU so timestamp parity does not require a GPU driver.
        gpu_args = Namespace(ffmpeg='ffmpeg', input=str(source), output='unused', device='auto',
                             bitrate='1M', gop=60, chroma_location='left', color_range='pc', fps=fps)
        gpu_command = build_command(gpu_args, 'intel', 'shader.cl', probe=True)
        selector = 'select=' + gpu_command[gpu_command.index('-vf') + 1].split(',select=', 1)[1]
        bitstream_filter = gpu_command[gpu_command.index('-bsf:v') + 1]
        sampled = tmp_path / 'sampled.mp4'
        subprocess.run(['ffmpeg', '-v', 'error', '-i', str(source), '-vf', selector,
                        '-c:v', 'libx264', '-enc_time_base', '1/90000', '-fps_mode', 'passthrough',
                        '-bsf:v', bitstream_filter, str(sampled)],
                       check=True, capture_output=True, timeout=10)
        with av.open(str(sampled)) as container:
            sampled_times = [frame.time for frame in container.decode(video=0)]
        np.testing.assert_allclose(sampled_times, expected, atol=1/90000)
        headers = subprocess.run(['ffmpeg', '-v', 'info', '-i', str(sampled), '-c:v', 'copy',
                                  '-bsf:v', 'trace_headers', '-frames:v', '1', '-f', 'null', '-'],
                                 check=True, capture_output=True, text=True, timeout=10).stderr
        assert any('num_units_in_tick' in line and line.endswith(' = 1') for line in headers.splitlines())
        assert any('time_scale' in line and line.endswith(' = 60') for line in headers.splitlines())
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


@pytest.mark.integration
def test_gpu_frame_step_updates_encoder_rate_and_preserves_gaps(tmp_path):
    """Exercise the GPU sampling filter with a real encoder, including rate control and sparse PTS."""
    args = Namespace(ffmpeg='ffmpeg', input='unused', output='unused', device='auto', bitrate='10M',
                     gop=60, chroma_location='left', color_range='pc', fps=30, source_fps=60)
    command = build_command(args, 'intel', 'shader.cl', probe=True)
    assert command[command.index('-vf') + 1].endswith(',framestep=2')
    sampling_filter = command[command.index('-vf') + 1].rsplit(',', 1)[1]
    assert '-bsf:v' not in command
    args.source_fps = 30
    assert frame_step(args) == 1
    args.source_fps = 50
    assert frame_step(args) is None  # A 50 fps input must not silently turn the requested 30 into 25.
    rates = {}
    for name, selector in (('old', "select='not(mod(n,2))'"), ('sampled', sampling_filter)):
        target = tmp_path / f'{name}.mp4'
        subprocess.run([
            'ffmpeg', '-v', 'error', '-f', 'lavfi', '-i', 'testsrc2=size=1920x1080:rate=60',
            '-t', '6', '-vf', selector, '-c:v', 'libx264', '-preset', 'ultrafast',
            '-tune', 'zerolatency', '-bf', '0', '-g', '60', '-b:v', '10M',
            '-enc_time_base', '1/90000', '-fps_mode', 'passthrough', str(target),
        ], check=True, capture_output=True, timeout=40)
        metadata = subprocess.run([
            'ffprobe', '-v', 'error', '-show_entries', 'stream=r_frame_rate,bit_rate,nb_frames',
            '-of', 'json', str(target),
        ], check=True, capture_output=True, text=True, timeout=10)
        stream = json.loads(metadata.stdout)['streams'][0]
        assert stream['r_frame_rate'] == '30/1' and stream['nb_frames'] == '180'
        rates[name] = int(stream['bit_rate'])
    assert rates['sampled'] / rates['old'] > 1.7, rates
    assert 8_000_000 <= rates['sampled'] <= 12_500_000, rates
    sparse = tmp_path / 'sparse.mp4'
    subprocess.run([
        'ffmpeg', '-v', 'error', '-f', 'lavfi', '-i', 'testsrc2=size=128x72:rate=60',
        '-frames:v', '4', '-vf', f"select='lt(n,3)+gte(n,30)',{sampling_filter}",
        '-c:v', 'libx264', '-enc_time_base', '1/90000', '-fps_mode', 'passthrough', str(sparse),
    ], check=True, capture_output=True, timeout=10)
    with av.open(str(sparse)) as container:
        times = [frame.time for frame in container.decode(video=0)]
    np.testing.assert_allclose(times, [0, 1/30, 31/60, 33/60], atol=1/90000)
