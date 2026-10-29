#!/usr/bin/env bash
# Load the optional overlay only on an explicit FPGA request; --check never probes the board.
set -Eeuo pipefail
module_root="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
project_root="${ONBOARD_WORKSPACE:-$(dirname "${module_root}")}"
fpga_workspace="${FPGA_AVOIDANCE_WORKSPACE:-${module_root}/workspace}"
[[ ! -d "${fpga_workspace}/ros2_ws/src" ]] || fpga_workspace="${fpga_workspace}/ros2_ws"
planner_root="${AVOIDANCE_WORKSPACE:-${project_root%/*}/dyn_small_obs_avoidance-ros2}"
config_file="${FPGA_AVOIDANCE_CONFIG:-${module_root}/config/backend.yaml}"
check_only=false
if [[ "${1:-}" == '--check' ]]; then
  check_only=true
  shift
fi
if [[ "${1:-}" == '--config' ]]; then
  config_file="${2:?--config requires a file}"
  shift 2
fi
source "${project_root}/scripts/lib/runtime_common.bash"
runtime_source_setup "$(runtime_detect_ros_setup "${ONBOARD_ROS_DISTRO:-}")"
runtime_source_setup "${planner_root}/install/setup.bash"
runtime_source_setup "${project_root}/install/setup.bash"
runtime_source_setup "${fpga_workspace}/install/local_setup.bash"
fpga_python="${fpga_workspace}/.venv/bin/python3"
# Imported modules and YAML are intentionally checked only inside this optional entrypoint.
"${fpga_python}" - "${config_file}" <<'PY'
"""Verify optional installation and YAML without constructing ROS or opening sockets."""
import importlib.util
import sys
import yaml
from path_planning.srv import PlanMotion, SetGoal
from path_planning.msg import PlanResult

if importlib.util.find_spec('fpga_uav_planner_ros2.flight_backend_node') is None:
    raise SystemExit('FPGA flight backend adapter is not built')
with open(sys.argv[1], encoding='utf-8') as stream:
    config = yaml.safe_load(stream)
if not isinstance(config, dict):
    raise SystemExit('FPGA backend config must be a mapping')
PY
if ${check_only}; then
  echo 'FPGA installation and configuration checked; no process or board probe started.'
  exit 0
fi
exec "${fpga_python}" -m fpga_uav_planner_ros2.flight_backend_node --config "${config_file}" "$@"
