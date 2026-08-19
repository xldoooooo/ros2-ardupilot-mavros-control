#!/usr/bin/env bash
# 彻底停止独立机载视频服务、残留媒体进程和真实摄像头占用者。

set -Eeuo pipefail

readonly WORKSPACE_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
readonly SERVICE_NAME="${VIDEO_SERVICE_SYSTEMD_UNIT:-video-service.service}"
readonly DEFAULT_CONFIG="/etc/ros2-ardupilot/camera.conf"
readonly PROJECT_CONFIG="${WORKSPACE_ROOT}/video_service/config/camera.conf"
readonly PROCESS_PATTERN='(^|[ /])(start_onboard_video\.sh|onboard_video_node\.py)([[:space:]]|$)'

restart_requested=false
declare -A target_pids=()
declare -A camera_devices=()

usage() {
  cat <<'EOF'
Usage: ./stop_onboard_video.sh [--restart]

Without options, stop video-service.service and remove every residual onboard
video process, configured RTSP-port owner, and real camera-device owner.

  --restart  Perform the same complete cleanup, then start video-service.service.
  -h, --help Show this help without changing the host.

This script never stops or restarts ros2-ardupilot-onboard.service.
EOF
}

while (($# > 0)); do
  case "$1" in
    --restart)
      restart_requested=true
      ;;
    -h | --help)
      usage
      exit 0
      ;;
    *)
      printf '[video-stop] ERROR: unknown option: %s\n' "$1" >&2
      usage >&2
      exit 2
      ;;
  esac
  shift
done

run_privileged() {
  if ((EUID == 0)); then
    "$@"
  else
    sudo "$@"
  fi
}

unit_exists() {
  command -v systemctl >/dev/null 2>&1 &&
    systemctl cat "${SERVICE_NAME}" >/dev/null 2>&1
}

read_ini_value() {
  local config_file="$1"
  local section="$2"
  local key="$3"
  awk -v wanted_section="${section}" -v wanted_key="${key}" '
    /^[[:space:]]*\[/ {
      current = $0
      gsub(/^[[:space:]]*\[|\][[:space:]]*$/, "", current)
      next
    }
    current == wanted_section {
      split($0, pieces, "=")
      name = pieces[1]
      gsub(/^[[:space:]]+|[[:space:]]+$/, "", name)
      if (name == wanted_key) {
        sub(/^[^=]*=/, "", $0)
        sub(/[[:space:]]*[#;].*$/, "", $0)
        gsub(/^[[:space:]]+|[[:space:]]+$/, "", $0)
        print $0
        exit
      }
    }
  ' "${config_file}"
}

resolve_config() {
  local requested="${VIDEO_SERVICE_ONBOARD_CONFIG:-}"
  if [[ -n "${requested}" && -r "${requested}" ]]; then
    printf '%s\n' "${requested}"
  elif [[ -r "${DEFAULT_CONFIG}" ]]; then
    printf '%s\n' "${DEFAULT_CONFIG}"
  else
    printf '%s\n' "${PROJECT_CONFIG}"
  fi
}

collect_camera_devices() {
  local config_file="$1"
  local configured_device="${VIDEO_SERVICE_CAMERA_DEVICE:-}"
  local candidate

  camera_devices=()
  if [[ -z "${configured_device}" && -r "${config_file}" ]]; then
    configured_device="$(read_ini_value "${config_file}" camera device)"
  fi
  if [[ -n "${configured_device}" && "${configured_device,,}" != "auto" ]]; then
    camera_devices["${configured_device}"]=1
  fi

  while IFS= read -r candidate; do
    [[ -n "${candidate}" ]] && camera_devices["${candidate}"]=1
  done < <(compgen -G '/dev/v4l/by-id/*-video-index*' || true)

  # 没有稳定 by-id 时才退回 video 节点，避免无差别扩大设备范围。
  if ((${#camera_devices[@]} == 0)); then
    while IFS= read -r candidate; do
      [[ -c "${candidate}" ]] && camera_devices["${candidate}"]=1
    done < <(compgen -G '/dev/video*' || true)
  fi
}

add_pid() {
  local pid="$1"
  [[ "${pid}" =~ ^[0-9]+$ ]] || return 0
  ((pid > 1)) || return 0
  [[ "${pid}" != "$$" && "${pid}" != "${PPID}" ]] || return 0
  [[ -d "/proc/${pid}" ]] || return 0
  target_pids["${pid}"]=1
}

collect_fuser_pids() {
  local resource="$1"
  local pid
  local output=""

  if ((EUID == 0)); then
    output="$(fuser "${resource}" 2>/dev/null || true)"
  else
    output="$(sudo fuser "${resource}" 2>/dev/null || true)"
  fi
  for pid in ${output}; do
    add_pid "${pid}"
  done
}

collect_targets() {
  local rtsp_port="$1"
  local pid parent_pid
  local changed=1
  local device

  target_pids=()
  while IFS= read -r pid; do
    add_pid "${pid}"
  done < <(pgrep -f -- "${PROCESS_PATTERN}" || true)

  for device in "${!camera_devices[@]}"; do
    [[ -e "${device}" ]] && collect_fuser_pids "${device}"
  done
  collect_fuser_pids "${rtsp_port}/tcp"

  # 把命中的启动器/节点/媒体父进程的全部后代纳入清理。
  while ((changed)); do
    changed=0
    while read -r pid parent_pid; do
      [[ -n "${pid}" && -n "${parent_pid}" ]] || continue
      if [[ -n "${target_pids[${parent_pid}]:-}" && -z "${target_pids[${pid}]:-}" ]]; then
        add_pid "${pid}"
        changed=1
      fi
    done < <(ps -eo pid=,ppid=)
  done
}

show_targets() {
  ((${#target_pids[@]} > 0)) || return 0
  run_privileged ps -o pid=,ppid=,pgid=,stat=,user=,args= \
    -p "$(IFS=,; echo "${!target_pids[*]}")" || true
}

signal_targets() {
  local signal_name="$1"
  local pid
  for pid in "${!target_pids[@]}"; do
    run_privileged kill "-${signal_name}" "${pid}" 2>/dev/null || true
  done
}

wait_for_exit() {
  local rtsp_port="$1"
  local attempts="$2"
  local attempt
  for ((attempt = 0; attempt < attempts; attempt++)); do
    collect_targets "${rtsp_port}"
    ((${#target_pids[@]} == 0)) && return 0
    sleep 0.1
  done
  return 1
}

readonly CONFIG_FILE="$(resolve_config)"
rtsp_port="$(read_ini_value "${CONFIG_FILE}" rtsp port 2>/dev/null || true)"
[[ "${rtsp_port}" =~ ^[0-9]+$ ]] || rtsp_port=8554
readonly RTSP_PORT="${rtsp_port}"
collect_camera_devices "${CONFIG_FILE}"

if unit_exists; then
  printf '[video-stop] stopping %s...\n' "${SERVICE_NAME}"
  run_privileged systemctl stop "${SERVICE_NAME}"
else
  printf '[video-stop] %s is not installed; cleaning manual processes only\n' \
    "${SERVICE_NAME}"
fi

collect_targets "${RTSP_PORT}"
if ((${#target_pids[@]} > 0)); then
  printf '[video-stop] stopping onboard video or camera/RTSP owners with SIGINT:\n'
  show_targets
  signal_targets INT
  wait_for_exit "${RTSP_PORT}" 50 || true
fi

collect_targets "${RTSP_PORT}"
if ((${#target_pids[@]} > 0)); then
  printf '[video-stop] escalating residual processes to SIGTERM:\n'
  show_targets
  signal_targets TERM
  wait_for_exit "${RTSP_PORT}" 50 || true
fi

collect_targets "${RTSP_PORT}"
if ((${#target_pids[@]} > 0)); then
  printf '[video-stop] force-killing processes that ignored graceful shutdown:\n'
  show_targets
  signal_targets KILL
  wait_for_exit "${RTSP_PORT}" 20 || true
fi

collect_targets "${RTSP_PORT}"
if ((${#target_pids[@]} > 0)); then
  printf '[video-stop] ERROR: residual video/camera processes remain:\n' >&2
  show_targets >&2
  exit 1
fi
if unit_exists && systemctl is-active --quiet "${SERVICE_NAME}"; then
  printf '[video-stop] ERROR: %s is still active\n' "${SERVICE_NAME}" >&2
  exit 1
fi

printf '[video-stop] complete: service inactive, RTSP port %s and camera devices are free\n' \
  "${RTSP_PORT}"

if "${restart_requested}"; then
  unit_exists || {
    printf '[video-stop] ERROR: cannot restart missing unit %s\n' \
      "${SERVICE_NAME}" >&2
    exit 1
  }
  printf '[video-stop] starting %s...\n' "${SERVICE_NAME}"
  run_privileged systemctl start "${SERVICE_NAME}"
  if ! systemctl is-active --quiet "${SERVICE_NAME}"; then
    printf '[video-stop] ERROR: %s did not become active\n' "${SERVICE_NAME}" >&2
    exit 1
  fi
  printf '[video-stop] restart complete: %s is active\n' "${SERVICE_NAME}"
fi
