#!/usr/bin/env bash
# Build current local ROS consumers and the isolated extnav candidate; start no services or hardware.
set -eo pipefail
project_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
planner_root="${AVOIDANCE_WORKSPACE:-${project_root%/*}/dyn_small_obs_avoidance-ros2}"
source /opt/ros/jazzy/setup.bash
if [[ -f "${planner_root}/install/local_setup.bash" ]]; then
    source "${planner_root}/install/local_setup.bash"
else
    echo "Missing native planner overlay: ${planner_root}/install/local_setup.bash" >&2
    exit 1
fi
cd "${project_root}"
export CMAKE_BUILD_PARALLEL_LEVEL="$(nproc)"
export MAKEFLAGS="-j$(nproc)"
colcon build --base-paths src \
    --packages-select guided_interfaces correction_interfaces correction_native onboard_control avoidance_bridge guided_sim \
    --parallel-workers "$(nproc)" --cmake-force-configure \
    --cmake-args -DCMAKE_BUILD_TYPE=Release -DAMENT_CMAKE_SYMLINK_INSTALL=OFF \
    -DPython3_EXECUTABLE="${project_root}/.venv/bin/python"
source install/local_setup.bash
colcon --log-base log/test-extnav build --base-paths correction_service/extnav_patch \
    --build-base build/test-extnav --install-base build/test-extnav/install \
    --packages-select extnav_bridge --parallel-workers "$(nproc)" \
    --cmake-args -DCMAKE_BUILD_TYPE=Release -DEXTNAV_WITH_CORRECTION=ON \
    -DPython3_EXECUTABLE="${project_root}/.venv/bin/python"
