#!/usr/bin/env bash
# Start the complete Humble/Jazzy ground station from this checkout.

set -Eeuo pipefail

readonly project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
readonly runtime_helpers="${project_root}/scripts/lib/runtime_common.bash"
readonly workspace_setup="${project_root}/install/setup.bash"
[[ -r "${runtime_helpers}" ]] || {
  echo "[ground-startup] runtime discovery helper is missing: ${runtime_helpers}" >&2
  exit 1
}
# shellcheck disable=SC1090
source "${runtime_helpers}"
# Host dependency paths are local project settings, independent of aircraft profiles.
runtime_source_environment_file "${project_root}/config/host.local.env"

ros_setup="$(runtime_detect_ros_setup "${GROUND_STATION_ROS_DISTRO:-}")" || exit 1
readonly ros_setup
preferred_python="$(
  runtime_detect_python "${project_root}" "${GROUND_STATION_PYTHON:-}"
)" || exit 1
readonly preferred_python

for required_file in "${ros_setup}" "${workspace_setup}"; do
  if [[ ! -r "${required_file}" ]]; then
    echo "[ground-startup] required setup file is missing: ${required_file}" >&2
    exit 1
  fi
done

runtime_source_setup "${ros_setup}"
runtime_source_setup "${workspace_setup}"

# 主机配置明确选择实机 domain；GUI 仿真仍独立使用 231/LOCALHOST。
hardware_domain="${GROUND_STATION_HARDWARE_DOMAIN_ID:-0}"
if [[ ! "${hardware_domain}" =~ ^[0-9]+$ ]] ||
   (( 10#${hardware_domain} > 232 || 10#${hardware_domain} == 231 )); then
  echo "[ground-startup] invalid GROUND_STATION_HARDWARE_DOMAIN_ID: ${hardware_domain}" >&2
  exit 1
fi
export ROS_DOMAIN_ID="$((10#${hardware_domain}))"
if [[ "${ROS_DISTRO:-}" == "humble" ]]; then
  export ROS_LOCALHOST_ONLY="0"
  unset ROS_AUTOMATIC_DISCOVERY_RANGE ROS_STATIC_PEERS
else
  unset ROS_LOCALHOST_ONLY
  export ROS_AUTOMATIC_DISCOVERY_RANGE="${ROS_AUTOMATIC_DISCOVERY_RANGE:-SUBNET}"
fi

echo "[ground-startup] ROS=${ROS_DISTRO:-unknown}, Python=${preferred_python}"
echo "[ground-startup] starting ground station on ROS domain ${ROS_DOMAIN_ID}"

# 保留此壳进程作为最后一道生命周期兜底。正常退出时 GUI 已完成清理；若 GUI
# 被强制结束，EXIT trap 仍会按项目专属 argv 规则清除本机 SITL/RViz 等残留。
cleanup_local_processes() {
  local gui_status=$?
  local cleanup_status=0
  trap - EXIT HUP INT QUIT TERM

  # 若只有监督壳收到终止信号，先给 GUI 最多 30 秒完成自身安全清理；
  # 超时后结束 GUI，再由下方项目专属扫描收尾。
  if [[ -n "${gui_pid:-}" ]] && kill -0 "${gui_pid}" 2>/dev/null; then
    kill -TERM "${gui_pid}" 2>/dev/null || true
    for _attempt in $(seq 1 300); do
      if ! kill -0 "${gui_pid}" 2>/dev/null; then
        break
      fi
      sleep 0.1
    done
    if kill -0 "${gui_pid}" 2>/dev/null; then
      kill -KILL "${gui_pid}" 2>/dev/null || true
    fi
    wait "${gui_pid}" 2>/dev/null || true
  fi

  "${preferred_python}" \
    "${project_root}/ground_station.py" --cleanup-local-processes \
    || cleanup_status=$?
  if ((gui_status == 0 && cleanup_status != 0)); then
    gui_status=${cleanup_status}
  fi
  exit "${gui_status}"
}

trap cleanup_local_processes EXIT
trap 'exit 129' HUP
trap 'exit 130' INT
trap 'exit 131' QUIT
trap 'exit 143' TERM

gui_pid=""
"${preferred_python}" "${project_root}/ground_station.py" "$@" &
gui_pid=$!
if wait "${gui_pid}"; then
  gui_status=0
else
  gui_status=$?
fi
exit "${gui_status}"
