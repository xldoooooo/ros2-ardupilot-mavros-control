#!/usr/bin/env bash
# Build only this standalone driver; never modify an external Odin workspace.
set -Eeuo pipefail

readonly workspace_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
readonly project_root="$(cd -- "${workspace_dir}/../.." && pwd -P)"
if [[ "${1:-}" == '-h' || "${1:-}" == '--help' ]]; then
  echo 'Usage: drivers/odin/build.sh [colcon build options]'
  echo 'Builds the project driver only. Optional: ODIN_CV_BRIDGE_SETUP and ODIN_BUILD_JOBS.'
  exit 0
fi
# Preserve caller choices while accepting the same optional environment as launchers.
cv_bridge_setup="${ODIN_CV_BRIDGE_SETUP:-}"
source "${project_root}/scripts/lib/runtime_common.bash"
source "${project_root}/scripts/lib/project_config.bash"
project_config_load "${project_root}"
set +u
cv_bridge_setup="${cv_bridge_setup:-${ODIN_CV_BRIDGE_SETUP:-}}"
source "/opt/ros/${ONBOARD_ROS_DISTRO:-${ROS_DISTRO:-jazzy}}/setup.bash"
if [[ -n "${cv_bridge_setup}" ]]; then
  source "${cv_bridge_setup}"
fi
set -u
# Resolve after sourcing so changing the selected ABI also updates an existing CMake cache.
cv_bridge_prefix="$(ros2 pkg prefix cv_bridge)"
# Two compiler processes keep the standalone SDK build modest on Jetson memory.
export CMAKE_BUILD_PARALLEL_LEVEL="${ODIN_BUILD_JOBS:-2}"
echo "[odin-build] workspace=${workspace_dir}; cv_bridge=${cv_bridge_setup:-ROS default}"
cd -- "${workspace_dir}"
exec colcon build --base-paths src --packages-select odin_ros_driver --symlink-install \
  "$@" --cmake-args -DBUILD_SYSTEM=ROS2 -DCMAKE_BUILD_TYPE=Release \
  "-Dcv_bridge_DIR=${cv_bridge_prefix}/share/cv_bridge/cmake"
