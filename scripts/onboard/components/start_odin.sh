#!/usr/bin/env bash
# Start the project-owned Odin driver with a named or absolute YAML configuration.

set -Eeuo pipefail

readonly script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly project_root="${ONBOARD_WORKSPACE:-$(cd "${script_dir}/../../.." && pwd)}"
# shellcheck disable=SC1091
source "${script_dir}/../../lib/runtime_common.bash"
# shellcheck disable=SC1091
source "${script_dir}/../../lib/odin_runtime.bash"

odin_parse_options "$@" || exit $?
if ((odin_help)); then
  echo "Usage: bash scripts/onboard/components/start_odin.sh [-c odom|slam|reloc|original|/path/config.yaml] [--check]"
  echo "Default: navigation.env mode (odom when absent). --check validates without starting Odin."
  exit 0
fi

source "${project_root}/scripts/lib/project_config.bash"
project_config_load "${project_root}"
odin_default_selection

ros_setup="$(runtime_detect_ros_setup "${ONBOARD_ROS_DISTRO:-}")" || exit 1
readonly ros_setup
runtime_source_setup "${ros_setup}"
odin_source_project_overlay "${project_root}"
odin_config_file="$(odin_resolve_config "${project_root}" "${odin_config_selection}")" || exit $?
readonly odin_config_file
odin_validate_map_config "${project_root}" "${odin_config_file}"

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
if [[ "${ROS_DISTRO:-}" == "humble" ]]; then
  export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"
  unset ROS_AUTOMATIC_DISCOVERY_RANGE ROS_STATIC_PEERS
else
  unset ROS_LOCALHOST_ONLY
  export ROS_AUTOMATIC_DISCOVERY_RANGE="${ROS_AUTOMATIC_DISCOVERY_RANGE:-SUBNET}"
fi

echo "[odin] package=$(runtime_package_prefix odin_ros_driver)"
echo "[odin] config=${odin_config_file}"
if ((odin_check)); then
  echo "[odin] discovery check passed; no component was started"
  exit 0
fi
exec ros2 launch odin_ros_driver odin1_ros2.launch.py "config_file:=${odin_config_file}"
