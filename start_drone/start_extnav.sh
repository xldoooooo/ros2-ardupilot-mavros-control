#!/usr/bin/env bash
# Start the external-navigation bridge from uniquely discovered ROS overlays.

set -Eeuo pipefail

readonly script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly project_root="$(cd -- "${script_dir}/.." && pwd -P)"
# shellcheck disable=SC1091
source "${script_dir}/runtime_common.bash"

ros_setup="$(runtime_detect_ros_setup "${ONBOARD_ROS_DISTRO:-}")" || exit 1
readonly ros_setup
runtime_source_setup "${ros_setup}"
# correction_interfaces 位于项目 overlay；缺失时 extnav 源码仍会 identity 透传，
# 但正常任务 27 部署应在独立 extnav overlay 前先发现它。
if [[ -r "${project_root}/install/setup.bash" ]]; then
  runtime_source_setup "${project_root}/install/setup.bash"
fi
runtime_ensure_package odin_ros_driver "${ODIN_OVERLAY_SETUP:-}"
runtime_ensure_package extnav_bridge "${EXTNAV_OVERLAY_SETUP:-}"

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
[[ "${ROS_DISTRO:-}" == "humble" ]] && export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"

exec ros2 run extnav_bridge extnav_to_vision_pose --ros-args \
  -p vision_rate_hz:=40.0 \
  -p ctrl_rate_hz:=100.0 \
  -p odom_topic:=/odin1/odometry_highfreq \
  -p roll_cam:=0.0 \
  -p pitch_cam:=0.0 \
  -p yaw_cam:=0.0 \
  -p odin_x:=0.06 \
  -p odin_y:=-0.03 \
  -p odin_z:=0.05
