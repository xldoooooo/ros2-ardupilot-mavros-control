"""Translate optional host GPU settings into relay arguments without changing defaults."""

import logging
import re

# Device selection is host-specific; processing algorithms remain in the relay backends.
GPU_ARGUMENTS = {
    'intel': {'device': '--intel-device', 'opencl_runtime': '--intel-opencl-runtime'},
    'nvidia': {'cuda_device': '--cuda-device', 'opencl_device': '--opencl-device'},
    'amd': {'device': '--amd-device'},
}


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
            arguments += [f'{flag}={value}']
    return arguments
