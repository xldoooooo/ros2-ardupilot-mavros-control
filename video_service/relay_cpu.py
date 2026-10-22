"""Software rectification with original timing and optional shared RTSP/recording packets."""

from contextlib import ExitStack
from fractions import Fraction
from pathlib import Path
import signal
import tempfile
import time

import cv2
import numpy as np


def run_cpu(args, calibration):
    """Process frames without GPU dependencies, preserving relative source timestamps."""
    try:
        import av  # Only the compatibility backend needs PyAV.
    except ImportError as exc:
        raise RuntimeError('CPU 兼容后端缺少 PyAV，请用项目 .venv 执行 pip install -r requirements-gui.txt') from exc

    stopping = False

    def stop(signum, frame):
        """Allow the encoder and container to finish when the relay is stopped."""
        nonlocal stopping
        stopping = True

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    cv2.setNumThreads(args.cpu_threads)
    maps = (calibration.maps(), calibration.maps(True))
    # Match the GPU path's short RTSP analysis without changing file inputs or source PTS.
    input_options = ({'rtsp_transport': 'tcp', 'analyzeduration': '200000', 'probesize': '5000000', 'fpsprobesize': '8'}
                     if args.input.startswith('rtsp://') else {})
    output_options = {'rtsp_transport': 'tcp'} if args.output.startswith('rtsp://') else {}
    output_format = 'rtsp' if output_options else None
    width, height = calibration.width, calibration.height
    count, first_time, last_pts, started, last_progress = 0, None, -1, time.monotonic(), 0.0
    with av.open(args.input, options=input_options, timeout=(15, 15)) as source:
        stream = source.streams.video[0]
        stream.codec_context.thread_count = args.cpu_threads
        rate = stream.average_rate or stream.guessed_rate or Fraction(30, 1)
        # A fine timebase preserves network gaps and VFR instead of manufacturing CFR frames.
        time_base = Fraction(1, 90000)
        with ExitStack() as outputs:
            destination = args.output
            if getattr(args, 'recording', None):
                # Reserve without overwriting, then keep tee away from arbitrary path characters.
                outputs.enter_context(args.recording.open('xb'))
                temporary = outputs.enter_context(tempfile.TemporaryDirectory(prefix='ground-relay-record-'))
                alias = Path(temporary) / 'recording.mkv'
                alias.symlink_to(args.recording.resolve())
                rtsp_options = '[f=rtsp:rtsp_transport=tcp]' if output_format == 'rtsp' else ''
                destination = f'[f=matroska]{alias}|{rtsp_options}{args.output}'
                output_format, output_options = 'tee', {}
            sink = outputs.enter_context(av.open(destination, 'w', format=output_format, options=output_options))
            encoded = sink.add_stream('libx264', rate=rate)
            encoded.width, encoded.height, encoded.pix_fmt = width, height, 'yuv420p'
            encoded.bit_rate = args.bitrate_value
            encoded.time_base = encoded.codec_context.time_base = time_base
            encoded.codec_context.thread_count = args.cpu_threads
            encoded.codec_context.gop_size = args.gop
            encoded.codec_context.max_b_frames = 0
            encoded.options = {'preset': 'veryfast', 'tune': 'zerolatency',
                               'x264-params': f'chromaloc={0 if args.chroma_location == "left" else 1}'}
            if getattr(args, 'recording', None):
                # Both tee slaves receive the same packets and complete H.264 headers.
                encoded.codec_context.flags |= av.codec.context.Flags.global_header
            for frame in source.decode(stream):
                if stopping:
                    break
                if (frame.width, frame.height) != (width, height):
                    raise ValueError(f'Input {frame.width}x{frame.height} differs from calibration {width}x{height}')
                if frame.pts is None:
                    raise ValueError('Source frame has no timestamp; cannot preserve recording timing')
                timestamp = frame.pts * frame.time_base
                if first_time is None:
                    first_time = timestamp
                relative = timestamp - first_time
                if args.duration and relative >= args.duration:
                    break
                pts = round(relative / time_base)
                if pts <= last_pts:
                    raise ValueError('Source timestamps moved backwards; output stopped without hiding the discontinuity')
                # Keep full-range camera YUV values; an implicit yuvj420p→yuv420p conversion rescales them.
                raw = (frame.to_ndarray() if frame.format.name in ('yuv420p', 'yuvj420p')
                       else frame.to_ndarray(format='yuvj420p' if frame.color_range == 2 else 'yuv420p')).reshape(-1)
                ysize, csize = width * height, width * height // 4
                planes = (raw[:ysize].reshape(height, width),
                          raw[ysize:ysize+csize].reshape(height//2, width//2),
                          raw[ysize+csize:].reshape(height//2, width//2))
                corrected = [cv2.remap(plane, *maps[0 if i == 0 else 1], cv2.INTER_LINEAR,
                                      borderMode=cv2.BORDER_REPLICATE) for i, plane in enumerate(planes)]
                packed = np.concatenate([plane.reshape(-1) for plane in corrected]).reshape(height*3//2, width)
                result = av.VideoFrame.from_ndarray(packed, format='yuv420p')
                result.pts, result.time_base = pts, time_base
                result.color_range, result.colorspace = frame.color_range, frame.colorspace
                if count == 0:
                    encoded.codec_context.color_range = frame.color_range
                    encoded.codec_context.colorspace = frame.colorspace
                    encoded.codec_context.color_primaries = stream.codec_context.color_primaries
                    encoded.codec_context.color_trc = stream.codec_context.color_trc
                for packet in encoded.encode(result):
                    sink.mux(packet)
                count, last_pts = count + 1, pts
                now = time.monotonic()
                if args.progress and now - last_progress >= 1:
                    args.progress.write_text(f'frame={count}\nfps={count / (now-started):.2f}\n'
                                             f'out_time_us={round(pts * time_base * 1000000)}\nprogress=continue\n')
                    last_progress = now
                if args.frames and count >= args.frames:
                    break
            for packet in encoded.encode():
                sink.mux(packet)
    if not count and not stopping:
        raise RuntimeError('No video frames were received')
    if args.progress:
        args.progress.write_text(f'frame={count}\nout_time_us={round(max(0, last_pts) * time_base * 1000000)}\nprogress=end\n')
