#!/usr/bin/env bash
# Start the independent onboard video node with project-owned configuration.

set -Eeuo pipefail

readonly WORKSPACE_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd -P)"
readonly VIDEO_DIR="${WORKSPACE_ROOT}/video_service"
source "${WORKSPACE_ROOT}/scripts/lib/runtime_common.bash"
source "${WORKSPACE_ROOT}/scripts/lib/project_config.bash"
project_config_load "${WORKSPACE_ROOT}"

# Locate the installed ROS distribution.
resolve_ros_setup() {
  local distro setup
  if [[ -n "${ROS_DISTRO:-}" && -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
    printf '%s\n' "/opt/ros/${ROS_DISTRO}/setup.bash"
    return
  fi
  for distro in humble jazzy; do
    setup="/opt/ros/${distro}/setup.bash"
    if [[ -f "${setup}" ]]; then
      printf '%s\n' "${setup}"
      return
    fi
  done
  printf '[video-service] ERROR: no ROS setup found under /opt/ros\n' >&2
  return 1
}

readonly ROS_SETUP="$(resolve_ros_setup)"
readonly PYTHON_EXECUTABLE="${VIDEO_SERVICE_PYTHON:-${WORKSPACE_ROOT}/.venv/bin/python}"
readonly CAMERA_CONFIG="${VIDEO_SERVICE_ONBOARD_CONFIG}"
readonly LENS_CONFIG="${VIDEO_SERVICE_LENS_CONFIG}"
for config_file in "${CAMERA_CONFIG}" "${LENS_CONFIG}"; do
  [[ -r "${config_file}" ]] || {
    printf '[video-service] ERROR: config is not readable: %s\n' "${config_file}" >&2
    exit 1
  }
done
printf '[video-service] camera=%s lens=%s\n' "${CAMERA_CONFIG}" "${LENS_CONFIG}"

[[ -f "${WORKSPACE_ROOT}/install/setup.bash" ]] || {
  printf '[video-service] ERROR: workspace overlay is missing: %s\n' \
    "${WORKSPACE_ROOT}/install/setup.bash" >&2
  exit 1
}
[[ -f "${VIDEO_DIR}/onboard_video_node.py" ]] || {
  printf '[video-service] ERROR: onboard video node is missing: %s\n' \
    "${VIDEO_DIR}/onboard_video_node.py" >&2
  exit 1
}

# ROS setup 脚本可能读取未设置变量，只在 source 期间临时关闭 nounset。
set +u
# shellcheck disable=SC1090
source "${ROS_SETUP}"
source "${WORKSPACE_ROOT}/install/setup.bash"
set -u

export VIDEO_SERVICE_ONBOARD_CONFIG="${CAMERA_CONFIG}"
export VIDEO_SERVICE_LENS_CONFIG="${LENS_CONFIG}"

# Fast DDS 在 participant 创建时枚举网卡；必须先让主路由源 IPv4 就绪，
# 否则 Wi-Fi 稍后取得地址也不会可靠恢复跨机发现。
runtime_wait_for_lan_ipv4
exec "${PYTHON_EXECUTABLE}" "${VIDEO_DIR}/onboard_video_node.py"
