#!/usr/bin/env bash
# Start only the independent avoidance components; no hardware/control process is restarted.
set -Eeuo pipefail
project_root="${ONBOARD_WORKSPACE:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)}"
planner_root="${AVOIDANCE_WORKSPACE:-${project_root%/*}/dyn_small_obs_avoidance-ros2}"
source "${project_root}/scripts/lib/runtime_common.bash"
ros_setup="$(runtime_detect_ros_setup "${ONBOARD_ROS_DISTRO:-}")"
runtime_source_setup "${ros_setup}"
for path in "${planner_root}/install/setup.bash" "${project_root}/install/setup.bash"; do
  [[ -r "${path}" ]] || { echo "Missing overlay: ${path}" >&2; exit 1; }
  runtime_source_setup "${path}"
done
for package in guided_interfaces correction_interfaces avoidance_bridge; do
  runtime_verify_workspace_package_install "${project_root}" "${package}"
done
# The independent repository places packages at its root rather than under src.
planner_source_version="$(runtime_package_manifest_version "${planner_root}/path_planning/package.xml")"
planner_install_version="$(runtime_package_manifest_version "${planner_root}/install/path_planning/share/path_planning/package.xml")"
[[ "${planner_source_version}" == "${planner_install_version}" ]] || {
  echo 'Planner source/install version mismatch; rebuild the independent workspace.' >&2; exit 1;
}
if pgrep -f '(^|/)avoidance_bridge_node( |$)' >/dev/null; then
  echo 'Avoidance bridge is already running.' >&2; exit 1
fi
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
if [[ "${1:-}" == '--check' ]]; then
  echo 'Avoidance overlays and source/install manifests verified; no process started.'
  exit 0
fi
exec ros2 launch avoidance_bridge avoidance.launch.py "$@"
