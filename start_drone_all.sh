#!/usr/bin/env bash
# Auto-discover, launch, and supervise the four onboard ROS components.

set -Eeuo pipefail

readonly project_root="${ONBOARD_WORKSPACE:-$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)}"
readonly runtime_helpers="${project_root}/start_drone/runtime_common.bash"
readonly onboard_setup="${project_root}/install/setup.bash"
[[ -r "${runtime_helpers}" ]] || {
  echo "[startup] runtime discovery helper is missing: ${runtime_helpers}" >&2
  exit 1
}
# shellcheck disable=SC1090
source "${runtime_helpers}"

# Per-aircraft hardware choices stay outside Git and are shared by systemd/manual starts.
readonly onboard_environment_file="${ONBOARD_ENV_FILE:-/etc/ros2-ardupilot/onboard.env}"
if [[ -e "${onboard_environment_file}" && ! -r "${onboard_environment_file}" ]]; then
  echo "[startup] onboard environment is not readable: ${onboard_environment_file}" >&2
  exit 1
fi
if [[ -r "${onboard_environment_file}" ]]; then
  runtime_source_setup "${onboard_environment_file}"
fi

ros_setup="$(runtime_detect_ros_setup "${ONBOARD_ROS_DISTRO:-}")" || exit 1
readonly ros_setup
fcu_device="$(runtime_detect_fcu_device "${MAVROS_FCU_DEVICE:-}")" || exit 1
readonly fcu_device
readonly fcu_baud="${MAVROS_FCU_BAUD:-460800}"
readonly log_root="${ONBOARD_LOG_ROOT:-/tmp/ros2_ardupilot_onboard}"

# All four participants inherit one domain. Domain 0 matches the current manual setup.
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
if [[ "${ros_setup}" == */humble/setup.bash ]]; then
  export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"
  unset ROS_AUTOMATIC_DISCOVERY_RANGE ROS_STATIC_PEERS
else
  unset ROS_LOCALHOST_ONLY
  export ROS_AUTOMATIC_DISCOVERY_RANGE="${ROS_AUTOMATIC_DISCOVERY_RANGE:-SUBNET}"
fi

for required_path in \
  "${ros_setup}" \
  "${onboard_setup}" \
  "${fcu_device}"
do
  if [[ ! -e "${required_path}" ]]; then
    echo "[startup] required path is missing: ${required_path}" >&2
    exit 1
  fi
done

runtime_source_setup "${ros_setup}"
runtime_source_setup "${onboard_setup}"

for required_command in ros2 setsid stdbuf sed tee timeout grep pgrep; do
  if ! command -v "${required_command}" >/dev/null 2>&1; then
    echo "[startup] required command is missing: ${required_command}" >&2
    exit 1
  fi
done

runtime_ensure_package mavros
runtime_ensure_package odin_ros_driver "${ODIN_OVERLAY_SETUP:-}"
runtime_ensure_package extnav_bridge "${EXTNAV_OVERLAY_SETUP:-}"

# Duplicate MAVROS or control nodes can contend for the serial port or setpoint topic.
readonly process_pattern='mavros_node|odin1_ros2.launch.py|extnav_to_vision_pose|onboard_control_node'
existing_processes="$(pgrep -af "${process_pattern}" || true)"
if [[ -n "${existing_processes}" ]]; then
  echo "[startup] refusing to create duplicate flight-stack processes:" >&2
  echo "${existing_processes}" >&2
  exit 1
fi

echo "[startup] auto-discovered ROS=${ROS_DISTRO:-unknown}"
echo "[startup] workspace=${project_root}"
echo "[startup] FCU=${fcu_device}:${fcu_baud}"
echo "[startup] odin=$(ros2 pkg prefix odin_ros_driver)"
echo "[startup] extnav=$(ros2 pkg prefix extnav_bridge)"

if [[ "${1:-}" == "--check" ]]; then
  echo "[startup] discovery check passed; no component was started"
  exit 0
elif [[ -n "${1:-}" ]]; then
  echo "Usage: bash start_drone_all.sh [--check]" >&2
  exit 2
fi

readonly run_stamp="$(date '+%Y%m%d-%H%M%S')"
readonly run_directory="${log_root}/${run_stamp}"
mkdir -p "${run_directory}"

declare -a component_names=()
declare -a component_pids=()
cleanup_started=0

launch_component() {
  local name="$1"
  local command="$2"
  local log_file="${run_directory}/${name}.log"

  # A separate session lets Ctrl+C stop each ros2 launch tree, including child nodes.
  setsid bash --noprofile --norc -c "${command}" \
    > >(stdbuf -oL sed -u "s/^/[${name}] /" | tee -a "${log_file}") 2>&1 &
  local pid=$!
  component_names+=("${name}")
  component_pids+=("${pid}")
  echo "[startup] ${name} started: pid=${pid}, log=${log_file}"
}

children_alive() {
  local pid
  for pid in "${component_pids[@]}"; do
    if ! kill -0 "${pid}" 2>/dev/null; then
      return 1
    fi
  done
  return 0
}

any_child_alive() {
  local pid
  for pid in "${component_pids[@]}"; do
    if kill -0 "${pid}" 2>/dev/null; then
      return 0
    fi
  done
  return 1
}

cleanup() {
  local exit_code=$?
  local pid
  local attempt

  if ((cleanup_started)); then
    return "${exit_code}"
  fi
  cleanup_started=1
  trap - EXIT INT TERM

  if ((${#component_pids[@]} > 0)); then
    echo "[startup] stopping all four components..."
  fi
  for pid in "${component_pids[@]}"; do
    if kill -0 "${pid}" 2>/dev/null; then
      kill -INT -- "-${pid}" 2>/dev/null || true
    fi
  done

  for attempt in {1..50}; do
    any_child_alive || break
    sleep 0.1
  done
  for pid in "${component_pids[@]}"; do
    if kill -0 "${pid}" 2>/dev/null; then
      kill -TERM -- "-${pid}" 2>/dev/null || true
    fi
  done
  for pid in "${component_pids[@]}"; do
    wait "${pid}" 2>/dev/null || true
  done
  echo "[startup] components stopped; logs retained at ${run_directory}"
  return "${exit_code}"
}

trap cleanup EXIT
trap 'exit 130' INT TERM

launch_component "mavros" \
  "exec ros2 launch mavros apm.launch fcu_url:='${fcu_device}:${fcu_baud}'"
sleep 1
launch_component "odin" \
  "exec ros2 launch odin_ros_driver odin1_ros2.launch.py"
sleep 1
launch_component "extnav" \
  "exec ros2 run extnav_bridge extnav_to_vision_pose --ros-args -p vision_rate_hz:=40.0 -p ctrl_rate_hz:=100.0 -p odom_topic:=/odin1/odometry_highfreq -p roll_cam:=0.0 -p pitch_cam:=0.0 -p yaw_cam:=0.0 -p odin_x:=0.06 -p odin_y:=-0.03 -p odin_z:=0.05"
sleep 1
launch_component "onboard" \
  "cd '${project_root}'; exec ros2 launch onboard_control control.launch.py"

echo "[startup] all components launched on ROS domain ${ROS_DOMAIN_ID}"
echo "[startup] waiting for a safe, unarmed readiness snapshot (no control commands are sent)..."

ready=0
readonly readiness_deadline=$((SECONDS + 120))
while ((SECONDS < readiness_deadline)); do
  if ! children_alive; then
    echo "[startup] a component exited during readiness checks" >&2
    exit 1
  fi

  status_sample="$(timeout 3 ros2 topic echo --once /onboard_control/status 2>&1 || true)"
  if grep -q '^armed: true$' <<<"${status_sample}"; then
    echo "[startup] WARNING: FCU reports armed=true; no command was sent by this script" >&2
  fi
  if grep -q '^armed: false$' <<<"${status_sample}" && \
    grep -q '^fcu_connected: true$' <<<"${status_sample}" && \
    grep -q '^message_rates_configured: true$' <<<"${status_sample}" && \
    grep -q '^thrust_mode_verified: true$' <<<"${status_sample}" && \
    grep -q '^local_position_valid: true$' <<<"${status_sample}"
  then
    ready=1
    break
  fi
  sleep 1
done

if ((ready)); then
  echo "[startup] READY: FCU connected, unarmed, rates/parameters/local position verified"
else
  echo "[startup] WARNING: processes remain running, but full readiness was not reached in time" >&2
  echo "[startup] inspect ${run_directory} and /onboard_control/status before any manual flight" >&2
fi

echo "[startup] press Ctrl+C once to stop all four components"
exited_pid=""
set +e
wait -n -p exited_pid "${component_pids[@]}"
component_exit_code=$?
set -e

exited_name="unknown"
for index in "${!component_pids[@]}"; do
  if [[ "${component_pids[${index}]}" == "${exited_pid}" ]]; then
    exited_name="${component_names[${index}]}"
    break
  fi
done
echo "[startup] component ${exited_name} exited unexpectedly with code ${component_exit_code}" >&2
if ((component_exit_code == 0)); then
  component_exit_code=1
fi
exit "${component_exit_code}"
