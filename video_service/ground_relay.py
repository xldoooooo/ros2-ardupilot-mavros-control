"""Select a ground video backend, report degradation and fall back to software processing."""

import argparse
from collections import deque
import json
import logging
import os
from pathlib import Path
import re
import signal
import subprocess
import sys
import tempfile
import time

from camera_app.relay_status import status_path, write_status
from relay_calibration import Calibration, DEFAULT_INTRINSICS

# Retain the Intel fast path, then probe NVIDIA and AMD before software fallback.
GPU_BACKENDS = ('intel', 'nvidia', 'amd')
# Finite probes never touch the user's output; slow driver initialization gets a generous timeout.
PROBE_TIMEOUT = 40
# A stable per-user installation keeps optional Intel drivers out of experiment directories.
DEFAULT_INTEL_RUNTIME = Path(os.environ.get('XDG_DATA_HOME', Path.home() / '.local/share')) / 'ros2-ardupilot-video/intel-opencl'


def bitrate_value(value):
    """Accept FFmpeg-style integer bitrates with an optional k/M suffix."""
    match = re.fullmatch(r'(\d+(?:\.\d+)?)([kKmM]?)', value)
    if not match:
        raise argparse.ArgumentTypeError('Use a positive bitrate such as 8M or 8000000')
    result = int(float(match[1]) * {'': 1, 'k': 1000, 'm': 1000000}[match[2].lower()])
    if result <= 0:
        raise argparse.ArgumentTypeError('Bitrate must be positive')
    return result


def build_command(args, backend, shader_path, input_url=None, output_url=None, probe=False):
    """Build one FFmpeg GPU pipeline, preserving source frame timing."""
    source, destination = input_url or args.input, output_url or args.output
    command = [args.ffmpeg, '-hide_banner', '-nostdin', '-loglevel', 'warning']
    if backend == 'intel':
        command += ['-init_hw_device', f'vaapi=va:{args.device}', '-init_hw_device', 'opencl=ocl@va',
                    '-filter_hw_device', 'ocl', '-hwaccel', 'vaapi', '-hwaccel_device', 'va',
                    '-hwaccel_output_format', 'vaapi']
        filters = ('hwmap=derive_device=opencl:mode=read+direct,'
                   f'program_opencl=source={shader_path}:kernel=rectify,'
                   'hwmap=derive_device=vaapi:reverse=1:mode=write+direct,format=vaapi')
        encoder = ['-c:v', 'h264_vaapi']
    elif backend == 'amd':
        command += ['-init_hw_device', f'vaapi=va:{args.device}', '-init_hw_device', 'vulkan=vk@va',
                    '-filter_hw_device', 'vk', '-hwaccel', 'vaapi', '-hwaccel_device', 'va',
                    '-hwaccel_output_format', 'vaapi']
        # AMD encoder surfaces are not Vulkan-renderable; transfer a Vulkan-owned output through RAM.
        filters = ('hwmap=derive_device=vulkan,'
                   f'libplacebo=custom_shader_path={shader_path}:format=yuv420p,'
                   'hwdownload,format=yuv420p,'
                   f'scale=in_range={args.color_range}:out_range={args.color_range},format=nv12,'
                   'hwupload=derive_device=vaapi')
        encoder = ['-c:v', 'h264_vaapi']
    else:
        command += ['-init_hw_device', f'opencl=ocl:{args.opencl_device}', '-filter_hw_device', 'ocl',
                    '-hwaccel', 'cuda', '-hwaccel_device', str(args.cuda_device),
                    '-hwaccel_output_format', 'cuda']
        # NVIDIA does not share VAAPI surfaces: explicit host transfers bridge CUDA and OpenCL.
        filters = (f'hwdownload,format=nv12,scale=in_range={args.color_range}:out_range={args.color_range},format=yuv420p,hwupload,'
                   f'program_opencl=source={shader_path}:kernel=rectify,'
                   'hwdownload,format=yuv420p')
        encoder = ['-c:v', 'h264_nvenc', '-gpu', str(args.cuda_device), '-preset', 'p4', '-tune', 'll']
    if source.startswith('rtsp://'):
        command += ['-rtsp_transport', 'tcp', '-timeout', '15000000']
    command += ['-threads', '2', '-i', source, '-map', '0:v:0', '-an', '-vf', filters,
                *encoder, '-b:v', args.bitrate, '-g', str(args.gop), '-bf', '0',
                '-fps_mode', 'passthrough', '-chroma_sample_location', args.chroma_location]
    if args.color_range != 'auto':
        command += ['-color_range', args.color_range]
    if probe:
        return command + ['-frames:v', '2', '-f', 'null', '-']
    if args.duration:
        command += ['-t', str(args.duration)]
    if args.frames:
        command += ['-frames:v', str(args.frames)]
    command += ['-progress', str(args.progress), '-stats_period', '1']
    if destination.startswith('rtsp://'):
        command += ['-f', 'rtsp', '-rtsp_transport', 'tcp']
    return command + ['-n', destination]


def run_probe(command, env=None):
    """Return a concise failure reason after an actual bounded hardware operation."""
    try:
        result = subprocess.run(command, capture_output=True, text=True, timeout=PROBE_TIMEOUT, env=env)
        if result.returncode:
            return '\n'.join(result.stderr.strip().splitlines()[-6:]) or f'Exit {result.returncode}'
    except (OSError, subprocess.TimeoutExpired) as exc:
        return str(exc)
    return ''


def intel_environment(runtime, temporary):
    """Discover an extracted Intel runtime and configure only its FFmpeg subprocesses."""
    libraries = list(runtime.expanduser().glob('usr/lib/*/intel-opencl/libigdrcl.so'))
    if not libraries:
        return None
    library = libraries[0].resolve()
    icd = temporary / 'intel-icd'
    icd.mkdir(exist_ok=True)
    (icd / 'intel.icd').write_text(str(library) + '\n')
    environment = os.environ.copy()
    environment['OCL_ICD_VENDORS'] = str(icd)
    paths = [str(library.parent.parent), str(library.parent)]
    if environment.get('LD_LIBRARY_PATH'):
        paths.append(environment['LD_LIBRARY_PATH'])
    environment['LD_LIBRARY_PATH'] = ':'.join(paths)
    return environment


def validate_source(args, calibration):
    """Check real input geometry before choosing an output backend."""
    command = [args.ffprobe, '-v', 'error']
    if args.input.startswith('rtsp://'):
        command += ['-rtsp_transport', 'tcp', '-timeout', '15000000']
    command += ['-select_streams', 'v:0', '-show_entries', 'stream=width,height,color_range', '-of', 'json', args.input]
    result = subprocess.run(command, capture_output=True, text=True, timeout=PROBE_TIMEOUT)
    if result.returncode:
        raise RuntimeError(f'无法读取输入视频：{result.stderr.strip()}')
    streams = json.loads(result.stdout).get('streams', [])
    if not streams:
        raise RuntimeError('输入没有视频轨道')
    args.color_range = streams[0].get('color_range', 'auto')
    actual = streams[0]['width'], streams[0]['height']
    if actual != (calibration.width, calibration.height):
        raise ValueError(f'输入尺寸 {actual} 与标定 {calibration.width}×{calibration.height} 不符，请选择对应标定')


def cpu_command(args):
    """Start software processing separately so signal and error handling remain uniform."""
    command = [sys.executable, str(Path(__file__).resolve()), '--worker-cpu',
               '--input', args.input, '--output', args.output, '--intrinsics', str(args.intrinsics),
               '--chroma-location', args.chroma_location, '--bitrate', args.bitrate,
               '--gop', str(args.gop), '--cpu-threads', str(args.cpu_threads), '--progress', str(args.progress)]
    if args.duration:
        command += ['--duration', str(args.duration)]
    if args.frames:
        command += ['--frames', str(args.frames)]
    return command


def supervise(args, calibration, state):
    """Probe GPUs before publishing, then allow one explicit software recovery."""
    stopping = False
    process = None

    def stop(signum, frame):
        """Forward a graceful stop and suppress fallback on deliberate shutdown."""
        nonlocal stopping
        stopping = True
        if process is None:
            raise KeyboardInterrupt
        if process is not None and process.poll() is None:
            process.send_signal(signal.SIGINT)

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)

    def report(stage, backend='', warning='', detail=''):
        """Publish state for terminal users and the independent camera panel."""
        state.update(state=stage, backend=backend, warning=warning, detail=detail)
        try:
            write_status(status_path(args.output), state)
        except OSError as exc:
            logging.warning('无法写入面板状态：%s', exc)

    with tempfile.TemporaryDirectory(prefix='ground-relay-') as temporary:
        root = Path(temporary)
        # Generated paths contain no filtergraph metacharacters, unlike arbitrary calibration paths.
        shader = root / 'rectify.cl'
        shader.write_text(calibration.shader())
        vulkan_shader = root / 'rectify.hook'
        vulkan_shader.write_text(calibration.shader(vulkan=True))
        shaders = {candidate: vulkan_shader if candidate == 'amd' else shader for candidate in GPU_BACKENDS}
        if args.progress is None:
            args.progress = root / 'progress.txt'
        backend, failures, worker_environment = 'cpu', [], None
        candidates = GPU_BACKENDS if args.backend == 'auto' else (() if args.backend == 'cpu' else (args.backend,))
        if candidates:
            report('starting')
            fixture = root / 'probe.mp4'
            reason = run_probe([args.ffmpeg, '-hide_banner', '-loglevel', 'error', '-f', 'lavfi', '-i',
                                f'color=size={calibration.width}x{calibration.height}:rate=30',
                                '-frames:v', '2', '-c:v', 'libx264', '-threads', '2', '-y', str(fixture)])
            if reason:
                failures.append(f'GPU探测样本生成失败：{reason}')
            else:
                for candidate in candidates:
                    if stopping:
                        break
                    command = build_command(args, candidate, shaders[candidate], str(fixture), probe=True)
                    reason = run_probe(command)
                    candidate_environment = None
                    if reason and candidate == 'intel':
                        candidate_environment = intel_environment(args.intel_opencl_runtime, root)
                        if candidate_environment is not None:
                            reason = run_probe(command, env=candidate_environment)
                            if not reason:
                                logging.info('已自动加载用户级 Intel OpenCL 运行时：%s', args.intel_opencl_runtime)
                    if not reason:
                        backend, worker_environment = candidate, candidate_environment
                        break
                    failures.append(f'{candidate}: {reason}')
        warning = ''
        if backend == 'cpu':
            warning = ('GPU 后端不可用，已回退 CPU 兼容模式；处理速度可能下降。' if candidates else
                       '已选择 CPU 兼容模式；处理速度可能下降。')
            logging.warning('%s\n%s', warning, '\n'.join(failures))
        elif backend == 'nvidia':
            logging.info('使用 NVIDIA：NVDEC → OpenCL 去畸变 → NVENC（含主存搬运）')
        elif backend == 'amd':
            logging.info('使用 AMD：VAAPI → Vulkan/libplacebo 去畸变 → VAAPI（含主存搬运）')
        else:
            logging.info('使用 Intel：VAAPI → OpenCL → VAAPI 共享表面')
        while not stopping:
            report('starting', backend, warning, '\n'.join(failures))
            args.progress.unlink(missing_ok=True)
            command = cpu_command(args) if backend == 'cpu' else build_command(args, backend, shaders[backend])
            error_log = root / 'worker.log'
            with error_log.open('w') as errors:
                process = subprocess.Popen(command, stdout=subprocess.DEVNULL, stderr=errors,
                                           env=worker_environment if backend == 'intel' else None)
                marked_running, stop_at = False, None
                try:
                    while process.poll() is None:
                        if stopping:
                            stop_at = stop_at or time.monotonic()
                            if time.monotonic() - stop_at > 20:
                                process.kill()
                        if not marked_running:
                            try:
                                progress = args.progress.read_text(errors='replace')
                            except OSError:
                                progress = ''
                            if re.search(r'frame=\s*[1-9]\d*', progress):
                                report('running', backend, warning, '\n'.join(failures))
                                marked_running = True
                        time.sleep(0.2)
                finally:
                    # A monitor exception must not orphan a publisher or an unfinished recorder.
                    if process.poll() is None:
                        process.send_signal(signal.SIGINT)
                        try:
                            process.wait(timeout=20)
                        except subprocess.TimeoutExpired:
                            process.kill()
                            process.wait()
            code = process.wait()
            with error_log.open() as errors:
                detail = ''.join(deque(errors, maxlen=12)).strip()
            if stopping:
                report('stopped', backend, warning)
                return 0
            if code == 0:
                try:
                    progress = args.progress.read_text(errors='replace')
                except OSError:
                    progress = ''
                if re.search(r'frame=\s*[1-9]\d*', progress):
                    report('completed', backend, warning)
                    return 0
                code, detail = 1, '处理进程没有报告任何已输出的视频帧'
            logging.error('%s 处理退出（%s）：%s', backend, code, detail)
            detail = f'{backend} 退出码 {code}：{detail}'
            failures.append(detail)
            if backend == 'cpu':
                report('error', backend, warning, detail)
                return 1
            # Preserve an interrupted local output before replaying the source from its beginning.
            if not args.output.startswith('rtsp://') and Path(args.output).exists():
                output = Path(args.output)
                saved = output.with_name(f'{output.stem}.gpu-failed-{time.time_ns()}{output.suffix}')
                output.rename(saved)
                logging.warning('GPU 失败输出已保留：%s', saved)
            backend = 'cpu'
            warning = 'GPU 处理已中断，正在切换 CPU 兼容模式；流存在中断，录像不能据此保证完整。'
            logging.warning(warning)
        report('stopped', backend, warning)
    return 0


def main():
    """Validate inputs and expose a standalone, observable compatibility relay."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input', required=True)
    parser.add_argument('--output', required=True)
    parser.add_argument('--backend', choices=('auto', *GPU_BACKENDS, 'cpu'), default='auto')
    parser.add_argument('--intrinsics', type=Path, default=DEFAULT_INTRINSICS)
    parser.add_argument('--device', default='/dev/dri/renderD128')
    parser.add_argument('--intel-opencl-runtime', type=Path, default=DEFAULT_INTEL_RUNTIME,
                        help='Extracted Intel runtime used if the normal environment fails')
    parser.add_argument('--cuda-device', type=int, default=0)
    parser.add_argument('--opencl-device', default=',device_type=gpu,platform_vendor=NVIDIA')
    parser.add_argument('--chroma-location', choices=('left', 'center'), default='left')
    parser.add_argument('--bitrate', default='8M')
    parser.add_argument('--gop', type=int, default=60)
    parser.add_argument('--cpu-threads', type=int, default=4)
    parser.add_argument('--ffmpeg', default='ffmpeg')
    parser.add_argument('--ffprobe', default='ffprobe')
    parser.add_argument('--duration', type=float)
    parser.add_argument('--frames', type=int)
    parser.add_argument('--progress', type=Path)
    parser.add_argument('--worker-cpu', action='store_true', help=argparse.SUPPRESS)
    args = parser.parse_args()

    def interrupt_startup(signum, frame):
        """Let subprocess.run reap a probe when startup is interrupted."""
        raise KeyboardInterrupt

    if not args.worker_cpu:
        signal.signal(signal.SIGTERM, interrupt_startup)
    logging.basicConfig(level=logging.INFO, format='[ground-relay] %(levelname)s %(message)s')
    state = dict(pid=os.getpid(), output=args.output, backend='', state='starting', warning='', detail='')
    try:
        args.bitrate_value = bitrate_value(args.bitrate)
        if args.cpu_threads <= 0 or args.gop <= 0 or (args.frames is not None and args.frames <= 0) or (args.duration is not None and args.duration <= 0):
            raise ValueError('Threads, GOP, frames and duration must be positive')
        calibration = Calibration(args.intrinsics, args.chroma_location)
        if args.worker_cpu:
            from relay_cpu import run_cpu
            run_cpu(args, calibration)
            return 0
        if not args.output.startswith('rtsp://') and Path(args.output).exists():
            raise FileExistsError(f'输出已存在，避免覆盖：{args.output}')
        try:
            write_status(status_path(args.output), state)
        except OSError as exc:
            logging.warning('无法写入面板状态：%s', exc)
        validate_source(args, calibration)
        return supervise(args, calibration, state)
    except (Exception, KeyboardInterrupt) as exc:
        interrupted = isinstance(exc, KeyboardInterrupt)
        if interrupted:
            logging.info('中继已停止')
        else:
            logging.error('%s', exc)
        if not args.worker_cpu:
            state.update(state='stopped' if interrupted else 'error', detail=str(exc))
            try:
                write_status(status_path(args.output), state)
            except OSError:
                pass
        return 0 if interrupted else 1


if __name__ == '__main__':
    sys.exit(main())
