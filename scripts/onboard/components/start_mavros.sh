#!/usr/bin/env bash
# Start MAVROS on the aircraft flight-controller serial link.

set -Eeuo pipefail

readonly script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${script_dir}/../../lib/runtime_common.bash"
runtime_source_environment_file "${ONBOARD_ENV_FILE:-/etc/ros2-ardupilot/onboard.env}"

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
ros2 launch mavros apm.launch fcu_url:="${fcu_device}:${fcu_baud}" &
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
