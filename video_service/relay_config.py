"""Translate optional host GPU settings into relay arguments without changing defaults."""

import logging
import os
from pathlib import Path
import re

# Anchor configured relative files to the checkout rather than the launch directory.
PROJECT = Path(__file__).resolve().parents[1]
# Linux exposes render nodes here; vendor IDs prioritize discovery, never exclude devices.
DRM_DIRECTORY = Path('/dev/dri')
DRM_SYSFS = Path('/sys/class/drm')
GPU_VENDORS = {'intel': '0x8086', 'amd': '0x1002'}

# Device selection is host-specific; processing algorithms remain in the relay backends.
GPU_ARGUMENTS = {
    'intel': {'device': '--intel-device', 'opencl_runtime': '--intel-opencl-runtime'},
    'nvidia': {'cuda_device': '--cuda-device', 'opencl_device': '--opencl-device'},
    'amd': {'device': '--amd-device'},
}


def project_path(value):
    """Expand user/environment paths and resolve relative files against this checkout."""
    path = Path(os.path.expandvars(str(value))).expanduser()
    return (path if path.is_absolute() else PROJECT / path).resolve()


def render_devices(backend, requested='auto'):
    """Honor explicit selectors; otherwise try matching vendors first, then all remaining nodes."""
    if requested and requested != 'auto':
        return [os.path.expandvars(str(Path(requested).expanduser()))]
    preferred, remaining = [], []
    try:
        nodes = sorted(DRM_DIRECTORY.glob('renderD*'))
    except OSError as exc:
        logging.warning('无法枚举 render 设备，将尝试兼容路径：%s', exc)
        nodes = []
    for node in nodes:
        try:
            vendor = (DRM_SYSFS / node.name / 'device/vendor').read_text().strip().lower()
        except OSError:
            vendor = ''
        target = preferred if vendor == GPU_VENDORS.get(backend) else remaining
        target.append(str(node))
    # Preserve the former selector when enumeration is unavailable (e.g. restricted sysfs/dev mounts).
    return preferred + remaining or [str(DRM_DIRECTORY / 'renderD128')]


def gpu_arguments(settings):
    """Ignore empty or malformed optional fields, retaining the relay's compatible defaults."""
    gpu = settings.get('gpu') or {}
    if not isinstance(gpu, dict):
        logging.warning('gpu 配置应为对象，已使用默认 GPU 参数')
        return []
    arguments = []
    for backend, fields in GPU_ARGUMENTS.items():
        values = gpu.get(backend) or {}
        if not isinstance(values, dict):
            logging.warning('gpu.%s 配置应为对象，已使用该后端默认参数', backend)
            continue
        for key, flag in fields.items():
            value = values.get(key)
            if value is None or value == '':
                continue
            if key == 'cuda_device':
                valid = type(value) is int or isinstance(value, str) and re.fullmatch(r'[+-]?[0-9]+', value.strip())
            else:
                valid = isinstance(value, str) and bool(value.strip())
            if not valid:
                logging.warning('gpu.%s.%s 参数类型无效，已使用默认值', backend, key)
                continue
            if key == 'opencl_runtime':
                value = project_path(value)
            arguments += [f'{flag}={value}']
    return arguments
