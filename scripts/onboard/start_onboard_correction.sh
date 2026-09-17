#!/usr/bin/env bash
# 独立启动 AprilTag-Odin 修正节点；不启动飞控、Odin、extnav 或视频服务。

set -Eeuo pipefail

readonly WORKSPACE_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd -P)"
readonly ENV_FILE="${CORRECTION_ENV_FILE:-/etc/ros2-ardupilot/correction.env}"

usage() {
  cat <<'EOF'
Usage: ./scripts/onboard/start_onboard_correction.sh

Start the independent correction_service node in the foreground. The node
remains idle with the camera closed until an explicit calibration request.
This launcher does not start Odin, extnav, MAVROS, onboard_control, or video.
EOF
}

case "${1:-}" in
  "") ;;
  -h | --help) usage; exit 0 ;;
  *) printf '[correction-start] ERROR: unknown option: %s\n' "$1" >&2; usage >&2; exit 2 ;;
esac
(( $# <= 1 )) || {
  printf '[correction-start] ERROR: unexpected extra arguments\n' >&2
  exit 2
}

source_setup() {
  local setup_file="$1"
  [[ -r "${setup_file}" ]] || {
    printf '[correction-start] ERROR: setup is not readable: %s\n' \
      "${setup_file}" >&2
    return 1
  }
  # ROS setup scripts may inspect optional variables that are unset.
  set +u
  # shellcheck disable=SC1090
  source "${setup_file}"
  set -u
}

# Match systemd EnvironmentFile behavior for a direct foreground launch.
if [[ -e "${ENV_FILE}" ]]; then
  [[ -r "${ENV_FILE}" ]] || {
    printf '[correction-start] ERROR: environment file is not readable: %s\n' \
      "${ENV_FILE}" >&2
    exit 1
  }
  set -a
  # shellcheck disable=SC1090
  source "${ENV_FILE}"
  set +a
fi

readonly ROS_SETUP="${CORRECTION_ROS_SETUP:-/opt/ros/jazzy/setup.bash}"
readonly WORKSPACE_SETUP="${WORKSPACE_ROOT}/install/local_setup.bash"
readonly CONFIG_DIR="${CORRECTION_CONFIG_DIR:-${WORKSPACE_ROOT}/correction_service/config}"

[[ -r "${CONFIG_DIR}/general_settings.yaml" ]] || {
  printf '[correction-start] ERROR: correction config is missing: %s\n' \
    "${CONFIG_DIR}/general_settings.yaml" >&2
  exit 1
}
source_setup "${ROS_SETUP}"
source_setup "${WORKSPACE_SETUP}"

for command_name in ros2 pgrep; do
  command -v "${command_name}" >/dev/null 2>&1 || {
    printf '[correction-start] ERROR: required command is missing: %s\n' \
      "${command_name}" >&2
    exit 1
  }
done
ros2 pkg prefix correction_service >/dev/null 2>&1 || {
  printf '[correction-start] ERROR: correction_service is not installed in the workspace overlay\n' >&2
  exit 1
}

# A second node would contend for the camera and expose duplicate service endpoints.
existing_processes="$(pgrep -af '[c]orrection_node' || true)"
if [[ -n "${existing_processes}" ]]; then
  printf '[correction-start] ERROR: correction_service is already running:\n%s\n' \
    "${existing_processes}" >&2
  exit 1
fi

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
unset ROS_LOCALHOST_ONLY
export ROS_AUTOMATIC_DISCOVERY_RANGE="${ROS_AUTOMATIC_DISCOVERY_RANGE:-SUBNET}"
export CORRECTION_CONFIG_DIR="${CONFIG_DIR}"

printf '[correction-start] workspace=%s domain=%s config=%s\n' \
  "${WORKSPACE_ROOT}" "${ROS_DOMAIN_ID}" "${CONFIG_DIR}"
printf '[correction-start] starting idle correction node; no flight command is sent\n'
exec ros2 run correction_service correction_node --ros-args \
  -p "config_dir:=${CONFIG_DIR}"
