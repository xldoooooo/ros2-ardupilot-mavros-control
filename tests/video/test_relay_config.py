"""Check optional GPU settings, device isolation and historical-media host selection."""

from argparse import Namespace

from video_service.ground_media import GroundMedia
from video_service.ground_relay import build_command
from video_service.relay_config import gpu_arguments


def test_optional_gpu_settings_and_invalid_category(caplog):
    """Old settings stay unchanged; one malformed category cannot disable the others."""
    assert gpu_arguments({}) == []
    assert gpu_arguments({'gpu': {'intel': {'opencl_runtime': ''}, 'amd': {}}}) == []
    flags = gpu_arguments({'gpu': {'intel': [1], 'nvidia': {'cuda_device': '--1'},
                                   'amd': {'device': '/dev/dri/renderD129'}}})
    assert flags == ['--amd-device=/dev/dri/renderD129']
    assert 'cuda_device' in caplog.text
    assert 'gpu.intel' in caplog.text


def test_rebuild_uses_current_host_devices():
    """Device locations follow this host while recorded image-processing options survive."""
    host = Namespace(config={'gpu': {
        'intel': {'device': '/dev/dri/renderD129', 'opencl_runtime': '~/Intel runtime'},
        'nvidia': {'cuda_device': 1, 'opencl_device': '0.1'},
        'amd': {'device': '/dev/dri/renderD130'},
    }})
    historical = {'intrinsics': 'saved.yaml', 'bitrate': '12M', 'gpu': {'nvidia': {'cuda_device': 9}}}
    command = GroundMedia.relay_command(host, 'source.mp4', 'out.mp4', 'progress', options=historical)
    assert command[command.index('--bitrate') + 1] == '12M'
    assert command[command.index('--intrinsics') + 1] == 'saved.yaml'
    assert command[-5:] == ['--intel-device=/dev/dri/renderD129', '--intel-opencl-runtime=~/Intel runtime',
                           '--cuda-device=1', '--opencl-device=0.1', '--amd-device=/dev/dri/renderD130']


def test_backend_devices_preserve_legacy_override():
    """Per-vendor overrides affect only that backend; the existing --device remains usable."""
    args = Namespace(ffmpeg='ffmpeg', input='source.mp4', output='out.mp4', device='/dev/dri/renderD131',
                     cuda_device=0, opencl_device='0.0', color_range='pc', bitrate='8M', gop=60,
                     chroma_location='left')
    legacy = {backend: build_command(args, backend, 'shader', probe=True) for backend in ('intel', 'amd', 'nvidia')}
    args.intel_device, args.amd_device = '/dev/dri/renderD129', '/dev/dri/renderD130'
    for backend, device in (('intel', args.intel_device), ('amd', args.amd_device)):
        expected = [arg.replace('vaapi=va:/dev/dri/renderD131', f'vaapi=va:{device}') for arg in legacy[backend]]
        assert build_command(args, backend, 'shader', probe=True) == expected
    assert build_command(args, 'nvidia', 'shader', probe=True) == legacy['nvidia']
    args.intel_device = args.amd_device = None
    for backend in legacy:
        assert build_command(args, backend, 'shader', probe=True) == legacy[backend]
