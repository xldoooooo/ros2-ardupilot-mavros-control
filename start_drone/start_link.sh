#!/usr/bin/env bash
# Start the onboard control link from the deployed aircraft workspace.

set -Eeuo pipefail

readonly script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly project_root="$(cd "${script_dir}/.." && pwd)"
# shellcheck disable=SC1091
source "${script_dir}/runtime_common.bash"

ros_setup="$(runtime_detect_ros_setup "${ONBOARD_ROS_DISTRO:-}")" || exit 1
readonly ros_setup
readonly onboard_setup="${project_root}/install/setup.bash"
runtime_source_setup "${ros_setup}"
runtime_source_setup "${onboard_setup}"

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
[[ "${ROS_DISTRO:-}" == "humble" ]] && export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"

cd "${project_root}"
exec ros2 launch onboard_control control.launch.py
