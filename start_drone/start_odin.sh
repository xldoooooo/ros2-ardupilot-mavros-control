#!/usr/bin/env bash
# Start the Odin positioning driver from its uniquely discovered ROS overlay.

set -Eeuo pipefail

readonly script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${script_dir}/runtime_common.bash"

ros_setup="$(runtime_detect_ros_setup "${ONBOARD_ROS_DISTRO:-}")" || exit 1
readonly ros_setup
runtime_source_setup "${ros_setup}"
runtime_ensure_package odin_ros_driver "${ODIN_OVERLAY_SETUP:-}"

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
[[ "${ROS_DISTRO:-}" == "humble" ]] && export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"

exec ros2 launch odin_ros_driver odin1_ros2.launch.py
