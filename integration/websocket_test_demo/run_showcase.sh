#!/usr/bin/env bash
# 一键准备隔离 Python 环境并运行甲方可见的逐帧通讯现场演示。
set -euo pipefail

demo_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
venv_dir="${demo_dir}/.venv"

if [[ ! -x "${venv_dir}/bin/python" ]]; then
  python3 -m venv "${venv_dir}"
fi

"${venv_dir}/bin/python" -m pip install --disable-pip-version-check -q \
  -r "${demo_dir}/requirements.txt"

exec "${venv_dir}/bin/python" "${demo_dir}/run_showcase.py" "$@"

