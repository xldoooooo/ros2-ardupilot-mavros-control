#!/usr/bin/env bash
# 停止独立修正 systemd unit 并清理仅属于 correction_service 的手工残留进程。

set -Eeuo pipefail

readonly SERVICE_NAME="${CORRECTION_SERVICE_SYSTEMD_UNIT:-odin-correction.service}"
readonly PROCESS_PATTERN='(^|[ /])(start_onboard_correction\.sh|ros2 run correction_service correction_node|correction_service/correction_node)([[:space:]]|$)'

declare -A target_pids=()

usage() {
  cat <<'EOF'
Usage: ./scripts/onboard/stop_onboard_correction.sh

Stop odin-correction.service and any residual manually launched correction
node. This does not stop Odin, extnav, MAVROS, onboard_control, or video.
Stopping correction_service does not clear an active correction in extnav.
EOF
}

case "${1:-}" in
  "") ;;
  -h | --help) usage; exit 0 ;;
  *) printf '[correction-stop] ERROR: unknown option: %s\n' "$1" >&2; usage >&2; exit 2 ;;
esac
(( $# <= 1 )) || {
  printf '[correction-stop] ERROR: unexpected extra arguments\n' >&2
  exit 2
}

run_privileged() {
  if (( EUID == 0 )); then
    "$@"
  else
    sudo "$@"
  fi
}

unit_exists() {
  command -v systemctl >/dev/null 2>&1 &&
    systemctl cat "${SERVICE_NAME}" >/dev/null 2>&1
}

add_pid() {
  local pid="$1"
  [[ "${pid}" =~ ^[0-9]+$ ]] || return 0
  (( pid > 1 )) || return 0
  [[ "${pid}" != "$$" && "${pid}" != "${PPID}" ]] || return 0
  [[ -d "/proc/${pid}" ]] || return 0
  target_pids["${pid}"]=1
}

collect_targets() {
  local pid parent_pid
  local changed=1

  target_pids=()
  while IFS= read -r pid; do
    add_pid "${pid}"
  done < <(pgrep -f -- "${PROCESS_PATTERN}" || true)

  # Include camera-driver children while they are still attributable to the node.
  while (( changed )); do
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
  local attempts="$1"
  local attempt
  for ((attempt = 0; attempt < attempts; attempt++)); do
    collect_targets
    ((${#target_pids[@]} == 0)) && return 0
    sleep 0.1
  done
  return 1
}

if unit_exists; then
  printf '[correction-stop] stopping %s...\n' "${SERVICE_NAME}"
  run_privileged systemctl stop "${SERVICE_NAME}"
else
  printf '[correction-stop] %s is not installed; cleaning manual processes only\n' \
    "${SERVICE_NAME}"
fi

collect_targets
if ((${#target_pids[@]} > 0)); then
  printf '[correction-stop] stopping residual correction processes with SIGINT:\n'
  show_targets
  signal_targets INT
  wait_for_exit 50 || true
fi

collect_targets
if ((${#target_pids[@]} > 0)); then
  printf '[correction-stop] escalating residual correction processes to SIGTERM:\n'
  show_targets
  signal_targets TERM
  wait_for_exit 50 || true
fi

collect_targets
if ((${#target_pids[@]} > 0)); then
  printf '[correction-stop] force-killing correction processes that ignored shutdown:\n'
  show_targets
  signal_targets KILL
  wait_for_exit 20 || true
fi

collect_targets
if ((${#target_pids[@]} > 0)); then
  printf '[correction-stop] ERROR: correction processes are still running:\n' >&2
  show_targets >&2
  exit 1
fi
if unit_exists && systemctl is-active --quiet "${SERVICE_NAME}"; then
  printf '[correction-stop] ERROR: %s is still active\n' "${SERVICE_NAME}" >&2
  exit 1
fi

printf '[correction-stop] complete: service inactive and no correction process remains\n'
printf '[correction-stop] note: any active extnav correction is unchanged\n'
