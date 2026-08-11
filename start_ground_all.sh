#!/usr/bin/env bash
# Start the complete Humble/Jazzy ground station from this checkout.

set -Eeuo pipefail

readonly project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly runtime_helpers="${project_root}/start_drone/runtime_common.bash"
readonly workspace_setup="${project_root}/install/setup.bash"
[[ -r "${runtime_helpers}" ]] || {
  echo "[ground-startup] runtime discovery helper is missing: ${runtime_helpers}" >&2
  exit 1
}
# shellcheck disable=SC1090
source "${runtime_helpers}"

ros_setup="$(runtime_detect_ros_setup "${GROUND_STATION_ROS_DISTRO:-}")" || exit 1
readonly ros_setup
preferred_python="$(
  runtime_detect_python "${project_root}" "${GROUND_STATION_PYTHON:-}"
)" || exit 1
readonly preferred_python

for required_file in "${ros_setup}" "${workspace_setup}"; do
  if [[ ! -r "${required_file}" ]]; then
    echo "[ground-startup] required setup file is missing: ${required_file}" >&2
    exit 1
  fi
done

runtime_source_setup "${ros_setup}"
runtime_source_setup "${workspace_setup}"

# GUI 内部会为本地仿真重建 domain 231/localhost-only context；实机始终
# 回到机载服务既有的 domain 0，避免调用者环境把两套系统重新混在一起。
export ROS_DOMAIN_ID="0"
if [[ "${ROS_DISTRO:-}" == "humble" ]]; then
  export ROS_LOCALHOST_ONLY="0"
  unset ROS_AUTOMATIC_DISCOVERY_RANGE ROS_STATIC_PEERS
else
  unset ROS_LOCALHOST_ONLY
  export ROS_AUTOMATIC_DISCOVERY_RANGE="${ROS_AUTOMATIC_DISCOVERY_RANGE:-SUBNET}"
fi

echo "[ground-startup] ROS=${ROS_DISTRO:-unknown}, Python=${preferred_python}"
echo "[ground-startup] starting ground station on ROS domain ${ROS_DOMAIN_ID}"
exec "${preferred_python}" "${project_root}/ground_station.py" "$@"
