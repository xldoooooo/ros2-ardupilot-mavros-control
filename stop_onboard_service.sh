#!/usr/bin/env bash
# Stop the managed onboard service and any manually started flight-stack processes.

set -Eeuo pipefail

readonly service_name="ros2-ardupilot-onboard.service"
# Match the aircraft ROS stack and Odin/RViz processes that can conflict with a restart.
readonly process_pattern='start_drone_all\.sh|ros2 launch mavros|/mavros_node([[:space:]]|$)|odin1_ros2\.launch\.py|/odin_ros_driver/|host_sdk_sample|image_overlay_node|extnav_to_vision_pose|ros2 launch onboard_control|/onboard_control_node([[:space:]]|$)|/rviz2([[:space:]]|$)'

declare -A target_pids=()

collect_targets() {
  local pid
  local parent_pid
  local changed=1

  target_pids=()
  while read -r pid; do
    [[ -n "${pid}" && "${pid}" != "$$" && "${pid}" != "${PPID}" ]] || continue
    target_pids["${pid}"]=1
  done < <(pgrep -f -- "${process_pattern}" || true)

  # Include every descendant of matched launchers, even when a child has a generic name.
  while ((changed)); do
    changed=0
    while read -r pid parent_pid; do
      [[ -n "${pid}" && -n "${parent_pid}" ]] || continue
      if [[ -n "${target_pids[${parent_pid}]:-}" && -z "${target_pids[${pid}]:-}" ]]; then
        target_pids["${pid}"]=1
        changed=1
      fi
    done < <(ps -eo pid=,ppid=)
  done
}

show_targets() {
  ((${#target_pids[@]} > 0)) || return 0
  ps -o pid=,ppid=,pgid=,stat=,args= -p "$(IFS=,; echo "${!target_pids[*]}")" || true
}

signal_targets() {
  local signal_name="$1"
  local pid

  for pid in "${!target_pids[@]}"; do
    kill "-${signal_name}" "${pid}" 2>/dev/null || true
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

echo "[stop] stopping ${service_name}..."
sudo systemctl stop "${service_name}"

collect_targets
if ((${#target_pids[@]} > 0)); then
  echo "[stop] stopping manually started or residual flight-stack processes:"
  show_targets
  signal_targets INT
  wait_for_exit 50 || true
fi

collect_targets
if ((${#target_pids[@]} > 0)); then
  echo "[stop] escalating residual processes to SIGTERM:"
  show_targets
  signal_targets TERM
  wait_for_exit 50 || true
fi

collect_targets
if ((${#target_pids[@]} > 0)); then
  echo "[stop] force-killing processes that ignored graceful shutdown:"
  show_targets
  signal_targets KILL
  wait_for_exit 20 || true
fi

collect_targets
if ((${#target_pids[@]} > 0)); then
  echo "[stop] ERROR: flight-stack processes are still running:" >&2
  show_targets >&2
  exit 1
fi

if systemctl is-active --quiet "${service_name}"; then
  echo "[stop] ERROR: ${service_name} is still active" >&2
  exit 1
fi

echo "[stop] complete: managed service is inactive and no flight-stack process remains"
