#!/usr/bin/env bash
# 使用主项目自己的 Python 环境启动 UQ212 内参标定；不会管理机载服务。
set -euo pipefail
script_path="$(readlink -f -- "${BASH_SOURCE[0]}")"
script_dir="$(dirname -- "${script_path}")"
workspace_dir="$(cd -- "${script_dir}/../.." && pwd -P)"
exec "${workspace_dir}/.venv/bin/python" "${script_dir}/uq212_cam_calib.py" "$@"
