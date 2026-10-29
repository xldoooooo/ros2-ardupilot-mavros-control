#!/usr/bin/env bash
# Run only local software/ROS contracts; these tests never address a flight controller or FPGA board.
set -Eeuo pipefail
module_root="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
project_root="${ONBOARD_WORKSPACE:-$(dirname "${module_root}")}"
planner_root="${AVOIDANCE_WORKSPACE:-${project_root%/*}/dyn_small_obs_avoidance-ros2}"
source "${project_root}/scripts/lib/runtime_common.bash"
runtime_source_setup "$(runtime_detect_ros_setup "${ONBOARD_ROS_DISTRO:-}")"
runtime_source_setup "${planner_root}/install/setup.bash"
runtime_source_setup "${project_root}/install/local_setup.bash"
runtime_source_setup "${module_root}/workspace/install/local_setup.bash"
export ROS_DOMAIN_ID=226 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
unset ROS_STATIC_PEERS
"${module_root}/workspace/.venv/bin/python3" "${module_root}/tests/test_executable_trajectory.py"
"${module_root}/workspace/.venv/bin/python3" "${module_root}/tests/test_flight_backend.py"
