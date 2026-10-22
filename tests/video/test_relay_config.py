"""Check optional GPU settings, device isolation and historical-media host selection."""

from argparse import Namespace
import json
from pathlib import Path

from video_service.ground_media import GroundMedia, load_settings
from video_service.file_manager_app import configured_api_url
from video_service.ground_relay import build_command
from video_service import relay_config
from video_service.relay_config import gpu_arguments, render_devices


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
    assert command[-5:] == ['--intel-device=/dev/dri/renderD129', f'--intel-opencl-runtime={Path.home() / "Intel runtime"}',
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


def test_paths_follow_checkout_and_user_not_working_directory(tmp_path, monkeypatch):
    """Moving the checkout or changing cwd must not redirect configured files or tool discovery."""
    project = tmp_path / 'moved project'
    project.mkdir()
    monkeypatch.setattr(relay_config, 'PROJECT', project)
    monkeypatch.setenv('GPU_CACHE', str(tmp_path / 'gpu cache'))
    tool = tmp_path / 'mediamtx'
    tool.write_text('#!/bin/sh\nexit 0\n')
    tool.chmod(0o755)
    monkeypatch.setenv('PATH', str(tmp_path))
    settings = {'root': 'media', 'intrinsics': '$GPU_CACHE/lens.yaml', 'api_url': 'http://127.0.0.1:18090',
                'gpu': {'intel': {'opencl_runtime': 'drivers/intel'}}}
    (project / 'media.json').write_text(json.dumps(settings))
    monkeypatch.chdir(tmp_path)
    monkeypatch.setenv('GROUND_MEDIA_CONFIG', 'media.json')
    assert configured_api_url() == settings['api_url']
    value = load_settings('media.json')
    assert value['root'] == str(project / 'media')
    assert value['intrinsics'] == str(tmp_path / 'gpu cache/lens.yaml')
    assert value['mediamtx'] == str(tool)
    assert gpu_arguments(value) == [f'--intel-opencl-runtime={project / "drivers/intel"}']
    settings.update(root='~/Media/ardupilot', mediamtx='./tools/mediamtx', intrinsics=str(tmp_path / 'lens.yaml'))
    (project / 'media.json').write_text(json.dumps(settings))
    value = load_settings('media.json')
    assert value['root'] == str(Path.home() / 'Media/ardupilot')
    assert value['mediamtx'] == str(project / 'tools/mediamtx')
    assert value['intrinsics'] == str(tmp_path / 'lens.yaml')


def test_render_discovery_prioritizes_vendor_without_excluding_devices(tmp_path, monkeypatch):
    """Renumbered/mixed GPUs remain candidates; unknown IDs and explicit paths still work."""
    devices, sysfs = tmp_path / 'dri', tmp_path / 'sysfs'
    devices.mkdir()
    monkeypatch.setattr(relay_config, 'DRM_DIRECTORY', devices)
    monkeypatch.setattr(relay_config, 'DRM_SYSFS', sysfs)
    for number, vendor in ((130, '0x8086'), (129, '0x1002'), (128, '0x9999')):
        (devices / f'renderD{number}').touch()
        path = sysfs / f'renderD{number}' / 'device'
        path.mkdir(parents=True)
        (path / 'vendor').write_text(vendor)
    intel = render_devices('intel')
    assert intel[0] == str(devices / 'renderD130')
    assert set(intel) == {str(path) for path in devices.iterdir()}
    assert render_devices('amd')[0] == str(devices / 'renderD129')
    assert render_devices('intel', '/dev/dri/by-path/custom-render') == ['/dev/dri/by-path/custom-render']
    monkeypatch.setattr(relay_config, 'DRM_DIRECTORY', tmp_path / 'missing')
    assert render_devices('amd') == [str(tmp_path / 'missing/renderD128')]
