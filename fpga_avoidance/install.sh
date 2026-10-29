#!/usr/bin/env bash
# Install the optional Python environment and build; never alter the default CPU entrypoint.
set -Eeuo pipefail
module_root="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
project_root="${ONBOARD_WORKSPACE:-$(dirname "${module_root}")}"
fpga_workspace="${FPGA_AVOIDANCE_WORKSPACE:-${module_root}/workspace}"
[[ ! -d "${fpga_workspace}/ros2_ws/src" ]] || fpga_workspace="${fpga_workspace}/ros2_ws"
project_python="${PROJECT_PYTHON:-${project_root}/.venv/bin/python3}"
[[ -x "${project_python}" && -d "${fpga_workspace}/src/fpga_uav_planner_ros2" ]] || {
  echo 'Project Python or FPGA source missing; set PROJECT_PYTHON / FPGA_AVOIDANCE_WORKSPACE.' >&2
  exit 1
}
if [[ ! -x "${fpga_workspace}/.venv/bin/python3" ]]; then
  "${project_python}" -m venv --system-site-packages "${fpga_workspace}/.venv"
fi
if ! "${fpga_workspace}/.venv/bin/python3" -c 'import numpy, yaml' 2>/dev/null; then
  "${fpga_workspace}/.venv/bin/python3" -m pip install numpy PyYAML
fi
exec "${module_root}/build.sh" "$@"
