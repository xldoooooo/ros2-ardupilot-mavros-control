#!/usr/bin/env bash
# Build only the removable FPGA workspace against the existing CPU planner interfaces.
set -Eeuo pipefail
module_root="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
project_root="${ONBOARD_WORKSPACE:-$(dirname "${module_root}")}"
fpga_workspace="${FPGA_AVOIDANCE_WORKSPACE:-${module_root}/workspace}"
[[ ! -d "${fpga_workspace}/ros2_ws/src" ]] || fpga_workspace="${fpga_workspace}/ros2_ws"
planner_root="${AVOIDANCE_WORKSPACE:-${project_root%/*}/dyn_small_obs_avoidance-ros2}"
source "${project_root}/scripts/lib/runtime_common.bash"
runtime_source_setup "$(runtime_detect_ros_setup "${ONBOARD_ROS_DISTRO:-}")"
runtime_source_setup "${planner_root}/install/setup.bash"
runtime_source_setup "${project_root}/install/setup.bash"
[[ -x "${fpga_workspace}/.venv/bin/python3" ]] || {
  echo 'FPGA Python environment missing; run fpga_avoidance/install.sh first.' >&2; exit 1;
}
[[ -d "${fpga_workspace}/src/fpga_uav_native" &&
   -d "${fpga_workspace}/src/fpga_uav_planner_ros2" ]] || {
  echo "FPGA source packages missing in ${fpga_workspace}/src" >&2; exit 1;
}
ros2 interface show path_planning/srv/SetGoal >/dev/null
ros2 interface show path_planning/srv/PlanMotion >/dev/null
(
  cd "${fpga_workspace}"
  "${fpga_workspace}/.venv/bin/python3" -m colcon build --base-paths src/fpga_uav_native src/fpga_uav_planner_ros2 \
    --packages-select fpga_uav_native fpga_uav_planner_ros2 --parallel-workers 2 \
    --cmake-args -DCMAKE_BUILD_TYPE=Release -DAMENT_CMAKE_SYMLINK_INSTALL=OFF
)
# Follow the installed manifest so removing the optional workspace disables the selector.
ln -sfn "${fpga_workspace}/install/fpga_uav_planner_ros2/share/fpga_uav_planner_ros2/package.xml" \
  "${module_root}/.installed"
echo 'Independent FPGA build complete; no module or board probe was started.'
