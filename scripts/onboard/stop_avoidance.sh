#!/usr/bin/env bash
# Stop avoidance launchers, nodes and residual descendants without stopping the flight stack.
set -Eeuo pipefail

# Stop the supervisor before collecting processes, so it cannot restart them.
if systemctl is-active --quiet ros2-ardupilot-avoidance.service; then
  sudo systemctl stop ros2-ardupilot-avoidance.service
fi

# Cover direct nodes, the operator entry, package launches and launch-file paths.
readonly process_pattern='(^|[[:space:]])([^[:space:]]*/)?(start_avoidance\.sh|avoidance_bridge_node|path_planning_node|avoidance_backend_manager\.py|fpga_uav_planner_ros2\.flight_backend_node)([[:space:]]|$)|(^|/)ros2[[:space:]]+launch[[:space:]]+((avoidance_bridge[[:space:]]+|[^[:space:]]*/)?avoidance\.launch\.py|(path_planning[[:space:]]+|[^[:space:]]*/)(odin|planner)\.launch\.py|(guided_sim[[:space:]]+|[^[:space:]]*/)?avoidance_demo\.launch\.py)([[:space:]]|$)'
# Match the graceful/forced shutdown windows used by stop_onboard_control.sh.
readonly graceful_attempts=50
readonly forced_attempts=20
readonly poll_interval=0.1

declare -A target_pids=()
declare -A excluded_pids=()

# Protect this script and its callers, including shells containing matching command text.
ancestor_pid=$$
while [[ "${ancestor_pid}" =~ ^[0-9]+$ ]] && ((ancestor_pid > 0)); do
  excluded_pids["${ancestor_pid}"]=1
  ancestor_pid="$(ps -o ppid= -p "${ancestor_pid}" | tr -d ' ')"
done

# Return the process start tick; zombies have exited and cannot be killed again.
process_identity() {
  local stat_line
  local -a fields
  [[ -r "/proc/$1/stat" ]] || return 1
  IFS= read -r stat_line < "/proc/$1/stat" 2>/dev/null || return 1
  read -r -a fields <<< "${stat_line##*) }"
  [[ "${fields[0]}" != Z && "${fields[0]}" != X ]] || return 1
  printf '%s\n' "${fields[19]}"
}

# Retain process identity so PID reuse cannot redirect a later shutdown signal.
add_target() {
  local pid="$1"
  local identity
  [[ -z "${excluded_pids[${pid}]:-}" ]] || return 1
  identity="$(process_identity "${pid}")" || return 1
  target_pids["${pid}"]="${identity}"
}

# Keep captured descendants after reparenting and discover newly spawned children.
collect_targets() {
  local pid parent_pid identity
  local changed=1
  local process_tree

  for pid in "${!target_pids[@]}"; do
    identity="$(process_identity "${pid}")" || identity=''
    if [[ "${identity}" != "${target_pids[${pid}]}" ]]; then
      unset 'target_pids['"${pid}"']'
    fi
  done
  while read -r pid; do
    add_target "${pid}" || true
  done < <(pgrep -f -- "${process_pattern}" || true)

  process_tree="$(ps -eo pid=,ppid=)"
  while ((changed)); do
    changed=0
    while read -r pid parent_pid; do
      if [[ -n "${target_pids[${parent_pid}]:-}" && -z "${target_pids[${pid}]:-}" ]]; then
        if add_target "${pid}"; then
          changed=1
        fi
      fi
    done <<< "${process_tree}"
  done
}

# Display the remaining processes for operator diagnosis.
show_targets() {
  ((${#target_pids[@]} > 0)) || return 0
  ps -o pid=,ppid=,pgid=,stat=,args= -p "$(IFS=,; echo "${!target_pids[*]}")" || true
}

# Signal individual processes; their process group may contain unrelated flight services.
signal_targets() {
  local signal_name="$1"
  local pid identity
  for pid in "${!target_pids[@]}"; do
    identity="$(process_identity "${pid}")" || continue
    [[ "${identity}" == "${target_pids[${pid}]}" ]] || continue
    kill "-${signal_name}" "${pid}" 2>/dev/null || true
  done
}

# Wait for real exit instead of treating successful signal delivery as completion.
wait_for_exit() {
  local attempts="$1"
  local attempt
  for ((attempt = 0; attempt < attempts; attempt++)); do
    collect_targets
    ((${#target_pids[@]} == 0)) && return 0
    sleep "${poll_interval}"
  done
  return 1
}

collect_targets
if ((${#target_pids[@]} > 0)); then
  echo '[stop] stopping avoidance launchers, nodes and descendants:'
  show_targets
  signal_targets INT
  wait_for_exit "${graceful_attempts}" || true
fi

collect_targets
if ((${#target_pids[@]} > 0)); then
  echo '[stop] escalating residual avoidance processes to SIGTERM:'
  show_targets
  signal_targets TERM
  wait_for_exit "${graceful_attempts}" || true
fi

collect_targets
if ((${#target_pids[@]} > 0)); then
  echo '[stop] force-killing avoidance processes that ignored graceful shutdown:'
  show_targets
  signal_targets KILL
  wait_for_exit "${forced_attempts}" || true
fi

collect_targets
if ((${#target_pids[@]} > 0)); then
  echo '[stop] ERROR: avoidance processes are still running:' >&2
  show_targets >&2
  exit 1
fi

echo '[stop] complete: no active avoidance process remains'
