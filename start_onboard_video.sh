#!/usr/bin/env bash
# 独立启动机载视频 ROS 节点；不加入飞控四进程的共同生命周期。

set -Eeuo pipefail

readonly WORKSPACE_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
readonly VIDEO_DIR="${WORKSPACE_ROOT}/video_service"

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
readonly PYTHON_EXECUTABLE="${VIDEO_SERVICE_PYTHON:-python3}"

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

export VIDEO_SERVICE_ONBOARD_CONFIG="${VIDEO_SERVICE_ONBOARD_CONFIG:-${VIDEO_DIR}/config/camera.conf}"
export VIDEO_SERVICE_LENS_CONFIG="${VIDEO_SERVICE_LENS_CONFIG:-${VIDEO_DIR}/config/lens.conf}"

exec "${PYTHON_EXECUTABLE}" "${VIDEO_DIR}/onboard_video_node.py"
