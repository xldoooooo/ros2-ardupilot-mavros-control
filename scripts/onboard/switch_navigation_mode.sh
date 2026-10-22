#!/usr/bin/env bash
# Restart the previously running flight/avoidance stacks after a verified ground-only mode switch.
set -Eeuo pipefail

# Keep the worker outside the flight unit so stopping that unit cannot kill the restart.
readonly script_path="$(realpath -- "${BASH_SOURCE[0]}")"
readonly project_root="${ONBOARD_WORKSPACE:-$(cd "$(dirname "${script_path}")/../.." && pwd)}"
readonly flight_service="ros2-ardupilot-onboard.service"
worker=0
if [[ "${1:-}" == --worker ]]; then worker=1; shift; fi
[[ $# == 1 && ( "$1" == odom || "$1" == map ) ]] || {
  echo 'Usage: switch_navigation_mode.sh odom|map' >&2; exit 2;
}
readonly requested_mode="$1"
readonly result_file="${NAVIGATION_SWITCH_RESULT_FILE:-/tmp/ros2_ardupilot_navigation_switch.result}"
switch_step="starting the restart worker"
switch_error=""

# The old control node can clear its pending flag even when ROS or sudo failed.
navigation_switch_exit() {
  local result=$?
  local marker
  if ((result)); then
    set +e
    local reason="Navigation switch failed at ${switch_step} (exit ${result}): ${switch_error:-inspect ${log_file:-the onboard service journal}}"
    reason="${reason//$'\n'/ }"
    echo "[navigation-switch] ${reason}" >&2
    marker="$(mktemp "${result_file}.XXXXXX")" || return 0
    printf '%s\n' "${reason}" > "${marker}"
    mv -f -- "${marker}" "${result_file}"
  fi
}
trap navigation_switch_exit EXIT

# Never prompt for credentials from a detached control process.
run_root() {
  if ((EUID == 0)); then "$@"; else sudo -n "$@"; fi
}

if ((!worker)) && grep -q "/${flight_service}" /proc/self/cgroup; then
  transient_args=(--unit="ros2-ardupilot-navigation-switch-$$" --collect
    --uid="$(id -u)" --gid="$(id -g)" --property="WorkingDirectory=${project_root}"
    --property=KillMode=process
    --setenv="ONBOARD_WORKSPACE=${project_root}")
  for variable in ONBOARD_ENV_FILE NAVIGATION_ENV_FILE NAVIGATION_SWITCH_RESULT_FILE \
      AVOIDANCE_SYSTEMD_SERVICE ONBOARD_LOG_ROOT ONBOARD_PYTHON \
      ROS_DOMAIN_ID ROS_AUTOMATIC_DISCOVERY_RANGE ROS_STATIC_PEERS ROS_LOCALHOST_ONLY RMW_IMPLEMENTATION; do
    [[ -v "${variable}" ]] && transient_args+=(--setenv="${variable}=${!variable}")
  done
  if dispatch_output="$(run_root systemd-run "${transient_args[@]}" /bin/bash "${script_path}" --worker "${requested_mode}" 2>&1)"; then
    echo "${dispatch_output}"
    exit 0
  fi
  switch_error="${dispatch_output}"
  exit 1
fi

# One transaction owns the restart; its log survives either service going down.
readonly log_root="${ONBOARD_LOG_ROOT:-/tmp/ros2_ardupilot_onboard}"
mkdir -p "${log_root}/navigation-switch"
readonly log_file="${log_root}/navigation-switch/$(date +%Y%m%d-%H%M%S)-$$.log"
exec > >(tee -a "${log_file}") 2>&1
exec 9>"${log_root}/navigation-switch/transaction.lock"
flock -n 9 || { echo '[navigation-switch] another switch is in progress'; exit 1; }
echo "[navigation-switch] requested=${requested_mode}, log=${log_file}"

source "${project_root}/scripts/lib/runtime_common.bash"
source "${project_root}/scripts/lib/odin_runtime.bash"
switch_step="loading the aircraft environment"
source "${project_root}/scripts/lib/project_config.bash"
project_config_load "${project_root}"
readonly avoidance_service="${AVOIDANCE_SYSTEMD_SERVICE:-ros2-ardupilot-avoidance.service}"
readonly navigation_env="${NAVIGATION_ENV_FILE}"
readonly project_python="${ONBOARD_PYTHON:-${project_root}/.venv/bin/python}"
[[ -x "${project_python}" ]] || { echo "Missing project Python: ${project_python}"; exit 1; }
runtime_source_setup "$(runtime_detect_ros_setup "${ONBOARD_ROS_DISTRO:-}")"
runtime_source_setup "${project_root}/install/setup.bash"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
if [[ "${ROS_DISTRO:-}" == humble ]]; then
  export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"
else
  unset ROS_LOCALHOST_ONLY
  export ROS_AUTOMATIC_DISCOVERY_RANGE="${ROS_AUTOMATIC_DISCOVERY_RANGE:-SUBNET}"
fi

selection=odom
[[ "${requested_mode}" == map ]] && selection=reloc
switch_step="validating the navigation configuration and sudo permissions"
selected_config="$(odin_resolve_config "${project_root}" "${selection}")"
if validation_output="$(odin_validate_map_config "${project_root}" "${selected_config}" 2>&1)"; then
  [[ -z "${validation_output}" ]] || echo "${validation_output}"
else
  switch_error="${validation_output}"
  exit 1
fi
run_root true

# Read newly received ROS evidence, never a cached CLI or GUI state.
switch_step="validating fresh ground and idle status"
if ground_output="$("${project_python}" - 2>&1 <<'PY'
"""Require an idle aircraft on the ground and no active Tag operation."""
import time
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from guided_interfaces.msg import ControlStatus
from correction_interfaces.msg import CorrectionStatus

rclpy.init()
node = rclpy.create_node("navigation_switch_ground_check")
received = {}
qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                 durability=DurabilityPolicy.VOLATILE)

def remember(name, message):
    """Keep receipt time with each status to reject stopped publishers."""
    received[name] = (time.monotonic(), message)

control = node.create_subscription(ControlStatus, "/onboard_control/status",
                                   lambda message: remember("control", message), qos)
tag = node.create_subscription(CorrectionStatus, "/correction_service/status",
                               lambda message: remember("tag", message), qos)
deadline = time.monotonic() + 4.0
while time.monotonic() < deadline:
    rclpy.spin_once(node, timeout_sec=0.1)
status_time, status = received.get("control", (0, None))
if status is None or time.monotonic() - status_time > 2.0:
    raise SystemExit("No fresh onboard status; switch refused")
if (not status.fcu_connected or not status.local_position_valid or status.armed or
        not status.on_ground or status.reboot_in_progress or status.controller_active or
        status.control_mode not in (status.MODE_IDLE, status.MODE_HOVER) or status.waypoint_count):
    raise SystemExit("Aircraft must be connected, unarmed, landed and idle without an active task")
tag_publishers = node.count_publishers("/correction_service/status")
if tag_publishers and ("tag" not in received or
                       time.monotonic() - received["tag"][0] > 2.0):
    raise SystemExit("Tag publisher has no fresh state; switch refused")
if "tag" in received and received["tag"][1].active:
    # TODO(Task6): Odin-Tag corrections are not supported during map operation.
    raise SystemExit("Stop the active Odin-Tag task before changing navigation mode")
node.destroy_node()
rclpy.shutdown()
print("[navigation-switch] fresh unarmed/on-ground/idle evidence verified")
PY
)"; then
  echo "${ground_output}"
else
  switch_error="${ground_output}"
  exit 1
fi

switch_step="snapshotting the running components"
flight_was_service=0
flight_was_running=0
avoidance_was_service=0
avoidance_was_running=0
manual_avoidance_pid=""
systemctl is-active --quiet "${flight_service}" && flight_was_service=1
if ((flight_was_service)) || pgrep -f '(^|/)onboard_control_node([[:space:]]|$)|ros2 launch onboard_control|start_onboard_control\.sh' >/dev/null; then
  flight_was_running=1
fi
((flight_was_running)) || { echo '[navigation-switch] no local flight stack is running; switch refused'; exit 1; }
systemctl is-active --quiet "${avoidance_service}" && avoidance_was_service=1
if ((avoidance_was_service)) || pgrep -f '(^|/)(avoidance_bridge_node|path_planning_node)([[:space:]]|$)|ros2 launch avoidance_bridge|start_avoidance\.sh' >/dev/null; then
  avoidance_was_running=1
fi
echo "[navigation-switch] snapshot: flight_systemd=${flight_was_service}, avoidance=${avoidance_was_running}, avoidance_systemd=${avoidance_was_service}"

switch_step="stopping the previous flight and avoidance stacks"
if ((avoidance_was_service)); then run_root systemctl stop "${avoidance_service}"; fi
if ((avoidance_was_running)); then bash "${project_root}/scripts/onboard/stop_avoidance.sh"; fi
run_root systemctl stop "${flight_service}"
bash "${project_root}/scripts/onboard/stop_onboard_control.sh" --skip-service

# Preserve unrelated per-aircraft assignments and replace only NAVIGATION_MODE.
switch_step="saving the navigation selection"
stage="$(mktemp)"
if [[ -e "${navigation_env}" ]]; then
  sed '/^[[:space:]]*\(export[[:space:]]\+\)\?NAVIGATION_MODE[[:space:]]*=/d' "${navigation_env}" > "${stage}"
fi
printf 'NAVIGATION_MODE=%s\n' "${requested_mode}" >> "${stage}"
# Keep the project parameter file editable by the service user after a GUI switch.
install -d -m 0755 "$(dirname "${navigation_env}")"
install -m 0644 "${stage}" "${navigation_env}"
rm -f -- "${stage}"

switch_step="restarting the previously running components"
if ((flight_was_service)); then
  run_root systemctl start "${flight_service}"
else
  setsid bash "${project_root}/scripts/onboard/start_onboard_control.sh" \
    > "${log_file%.log}-flight.log" 2>&1 < /dev/null 9>&- &
  echo "[navigation-switch] manual flight launcher pid=$!"
fi
if ((avoidance_was_running)); then
  if ((avoidance_was_service)); then
    run_root systemctl start "${avoidance_service}"
  else
    setsid bash "${project_root}/scripts/onboard/start_avoidance.sh" \
      > "${log_file%.log}-avoidance.log" 2>&1 < /dev/null 9>&- &
    manual_avoidance_pid=$!
    echo "[navigation-switch] manual avoidance launcher pid=${manual_avoidance_pid}"
  fi
fi

# A saved preference is not success: wait for evidence from the restarted driver.
switch_step="waiting for the restarted Odin driver"
"${project_python}" - "${requested_mode}" <<'PY'
"""Wait for the newly started driver to report the requested runtime mode."""
import json
import sys
import time
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import String

rclpy.init()
node = rclpy.create_node("navigation_switch_result")
actual = {}

def update(message):
    """Decode only the authoritative Odin status payload."""
    try:
        actual.update(json.loads(message.data))
    except (ValueError, TypeError):
        pass

qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                 durability=DurabilityPolicy.TRANSIENT_LOCAL)
subscription = node.create_subscription(String, "/odin1/navigation_status", update, qos)
deadline = time.monotonic() + 90.0
while time.monotonic() < deadline:
    rclpy.spin_once(node, timeout_sec=0.2)
    if actual.get("mode") == sys.argv[1] and actual.get("connected"):
        print(f"[navigation-switch] driver confirmed {actual['mode']}; ground station may reconnect")
        break
else:
    raise SystemExit(f"Restarted driver did not confirm {sys.argv[1]}: {actual}")
node.destroy_node()
rclpy.shutdown()
PY
switch_step="verifying the restored avoidance launcher"
if [[ -n "${manual_avoidance_pid}" ]] && ! kill -0 "${manual_avoidance_pid}" 2>/dev/null; then
  switch_error="Avoidance exited during restart; inspect ${log_file%.log}-avoidance.log"
  exit 1
fi
if ((avoidance_was_service)) && ! systemctl is-active --quiet "${avoidance_service}"; then
  switch_error="The previous avoidance service did not remain active: ${avoidance_service}"
  exit 1
fi
