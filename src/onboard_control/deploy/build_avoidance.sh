#!/usr/bin/env bash
# Build the native planner, adapter and consuming controller without starting hardware/services.
set -Eeuo pipefail
project_root="${ONBOARD_WORKSPACE:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)}"
planner_root="${AVOIDANCE_WORKSPACE:-${project_root%/*}/dyn_small_obs_avoidance-ros2}"
source "${project_root}/scripts/lib/runtime_common.bash"
runtime_source_setup "$(runtime_detect_ros_setup "${ONBOARD_ROS_DISTRO:-}")"
[[ -d "${planner_root}/path_searching" && -d "${planner_root}/path_planning" ]] || {
  echo "Missing standalone planner checkout: ${planner_root}" >&2; exit 1;
}
(
  cd "${planner_root}"
  colcon build --base-paths path_searching path_planning --packages-select path_searching path_planning \
    --parallel-workers 2 --cmake-force-configure \
    --cmake-args -DCMAKE_BUILD_TYPE=Release -DAMENT_CMAKE_SYMLINK_INSTALL=OFF
)
runtime_source_setup "${planner_root}/install/setup.bash"
(
  cd "${project_root}"
  colcon build --base-paths src --packages-select guided_interfaces correction_interfaces avoidance_bridge onboard_control guided_sim \
    --parallel-workers 2 --cmake-force-configure \
    --cmake-args -DCMAKE_BUILD_TYPE=Release -DAMENT_CMAKE_SYMLINK_INSTALL=OFF
)
echo 'Native avoidance and consuming controller build complete; no component was started.'
