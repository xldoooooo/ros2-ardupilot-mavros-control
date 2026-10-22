#!/usr/bin/env bash
# Start MAVROS on the aircraft flight-controller serial link.

set -Eeuo pipefail

readonly script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly project_root="${ONBOARD_WORKSPACE:-$(cd "${script_dir}/../../.." && pwd)}"
# shellcheck disable=SC1091
source "${script_dir}/../../lib/runtime_common.bash"
source "${project_root}/scripts/lib/project_config.bash"
project_config_load "${project_root}"

# Repository-owned launch keeps aircraft plugin selection independent of /opt/ros.
readonly mavros_launch="${project_root}/config/mavros/aircraft.launch.xml"
ros_setup="$(runtime_detect_ros_setup "${ONBOARD_ROS_DISTRO:-}")" || exit 1
readonly ros_setup
fcu_device="$(runtime_detect_fcu_device "${MAVROS_FCU_DEVICE:-}")" || exit 1
readonly fcu_device
readonly fcu_baud="${MAVROS_FCU_BAUD:-460800}"
runtime_source_setup "${ros_setup}"
if [[ -n "${MAVROS_OVERLAY_SETUP:-}" ]]; then
  runtime_source_setup "${MAVROS_OVERLAY_SETUP}"
fi

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
[[ "${ROS_DISTRO:-}" == "humble" ]] && export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"

echo "[mavros-startup] FCU=${fcu_device}:${fcu_baud}, ROS=${ROS_DISTRO:-unknown}"
echo "[mavros-startup] profile=${mavros_launch}"
echo "[mavros-startup] parameters=${MAVROS_CONFIG_FILE}"
ros2 launch "${mavros_launch}" fcu_url:="${fcu_device}:${fcu_baud}" "config_yaml:=${MAVROS_CONFIG_FILE}" &
mavros_pid=$!
# Keep the launch tree owned by this entry while checking the runtime clock configuration.
cleanup_mavros() {
  kill -TERM "${mavros_pid}" 2>/dev/null || true
  wait "${mavros_pid}" 2>/dev/null || true
}
trap cleanup_mavros EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
runtime_ensure_mavros_timesync
wait "${mavros_pid}"
