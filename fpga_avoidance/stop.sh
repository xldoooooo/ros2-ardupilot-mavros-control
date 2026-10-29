#!/usr/bin/env bash
# Select CPU through the landed-state gate and stop only the manager-owned FPGA process.
set -Eeuo pipefail
module_root="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
project_root="${ONBOARD_WORKSPACE:-$(dirname "${module_root}")}"
source "${project_root}/scripts/lib/runtime_common.bash"
runtime_source_setup "$(runtime_detect_ros_setup "${ONBOARD_ROS_DISTRO:-}")"
runtime_source_setup "${project_root}/install/setup.bash"
result="$(timeout 25 ros2 service call /avoidance/backend/select std_srvs/srv/SetBool '{data: false}')"
printf '%s\n' "${result}"
[[ "${result}" == *'success=True'* ]] || {
  echo 'CPU selection was rejected; FPGA process left unchanged.' >&2; exit 1;
}
