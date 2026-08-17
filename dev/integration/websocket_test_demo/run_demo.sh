#!/usr/bin/env bash
# 一键创建本目录隔离环境、执行协议单测并运行权威 JAR 端到端验收。
set -euo pipefail

demo_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
venv_dir="${demo_dir}/.venv"

if [[ ! -x "${venv_dir}/bin/python" ]]; then
  python3 -m venv "${venv_dir}"
fi

"${venv_dir}/bin/python" -m pip install --disable-pip-version-check -q \
  -r "${demo_dir}/requirements.txt"

PYTHONPATH="${demo_dir}" PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
  "${venv_dir}/bin/python" -m pytest -q "${demo_dir}/tests"
"${venv_dir}/bin/python" "${demo_dir}/run_acceptance.py" "$@"
