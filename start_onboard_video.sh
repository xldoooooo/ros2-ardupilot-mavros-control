#!/usr/bin/env bash
# 独立启动机载视频 ROS 节点；不加入飞控四进程的共同生命周期。

set -Eeuo pipefail

readonly WORKSPACE_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
readonly VIDEO_DIR="${WORKSPACE_ROOT}/video_service"
readonly SYSTEM_CAMERA_CONFIG="/etc/ros2-ardupilot/camera.conf"
readonly SYSTEM_LENS_CONFIG="/etc/ros2-ardupilot/lens.conf"
# 冷启动时有限等待主路由获得源 IPv4；不探测或等待外网连通性。
readonly LAN_ROUTE_TIMEOUT_SECONDS=30

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

resolve_video_config() {
  local requested="$1"
  local system_config="$2"
  local project_config="$3"
  if [[ -n "${requested}" ]]; then
    [[ -r "${requested}" ]] || {
      printf '[video-service] ERROR: configured file is not readable: %s\n' \
        "${requested}" >&2
      return 1
    }
    printf '%s\n' "${requested}"
  elif [[ -r "${system_config}" ]]; then
    printf '%s\n' "${system_config}"
  else
    printf '%s\n' "${project_config}"
  fi
}

wait_for_lan_route() {
  local attempt=0 source_ip
  command -v ip >/dev/null 2>&1 || {
    printf '[video-service] ERROR: ip command is required for LAN readiness\n' >&2
    return 1
  }
  while (( attempt < LAN_ROUTE_TIMEOUT_SECONDS * 4 )); do
    source_ip="$(
      ip -4 route show default 2>/dev/null | awk '
        !/linkdown/ {
          for (field = 1; field < NF; field++) {
            if ($field == "src" && $(field + 1) != "127.0.0.1") {
              print $(field + 1)
              exit
            }
          }
        }
      '
    )"
    if [[ -n "${source_ip}" ]]; then
      printf '[video-service] LAN route ready with source IPv4: %s\n' "${source_ip}"
      return 0
    fi
    sleep 0.25
    attempt=$((attempt + 1))
  done
  printf '[video-service] ERROR: no usable LAN default-route IPv4 after %ss\n' \
    "${LAN_ROUTE_TIMEOUT_SECONDS}" >&2
  return 1
}

readonly ROS_SETUP="$(resolve_ros_setup)"
readonly PYTHON_EXECUTABLE="${VIDEO_SERVICE_PYTHON:-python3}"
readonly CAMERA_CONFIG="$(resolve_video_config \
  "${VIDEO_SERVICE_ONBOARD_CONFIG:-}" \
  "${SYSTEM_CAMERA_CONFIG}" \
  "${VIDEO_DIR}/config/camera.conf")"
readonly LENS_CONFIG="$(resolve_video_config \
  "${VIDEO_SERVICE_LENS_CONFIG:-}" \
  "${SYSTEM_LENS_CONFIG}" \
  "${VIDEO_DIR}/config/lens.conf")"

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
wait_for_lan_route
exec "${PYTHON_EXECUTABLE}" "${VIDEO_DIR}/onboard_video_node.py"
