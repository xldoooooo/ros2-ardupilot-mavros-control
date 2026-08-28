#!/usr/bin/env bash
# 构建并安装独立 correction_service；启动后 idle，绝不自动打开相机或控制飞行。

set -Eeuo pipefail

readonly SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
readonly WORKSPACE_ROOT="$(cd -- "${SCRIPT_DIR}/../.." && pwd -P)"
readonly SERVICE_NAME="odin-correction.service"
readonly SERVICE_TEMPLATE="${SCRIPT_DIR}/correction-service.service.example"
readonly ENV_TEMPLATE="${SCRIPT_DIR}/correction.env.example"
readonly FLIGHT_SERVICE="ros2-ardupilot-onboard.service"

die() {
  printf '[correction-install] ERROR: %s\n' "$*" >&2
  exit 1
}

run_root() {
  if (( EUID == 0 )); then
    "$@"
  else
    command sudo "$@"
  fi
}

source_setup() {
  # ROS setup files legitimately inspect optional variables that may be unset.
  local setup_file="$1"
  set +u
  # shellcheck disable=SC1090
  source "${setup_file}"
  set -u
}

usage() {
  cat <<'EOF'
Usage: ./correction_service/deploy/install_correction_service.sh

Build correction_interfaces/correction_service, verify the calibrated camera
overlay and configuration, install odin-correction.service, then enable/start
the independent node. The node remains idle with the camera closed until a
ground start request. The installer refuses an active flight service because it
updates the shared project overlay. It sends no arm/takeoff/flight command.
EOF
}

if [[ "${1:-}" == "--help" || "${1:-}" == "-h" ]]; then
  usage
  exit 0
fi
(( $# == 0 )) || die "unknown argument: $1"
[[ -r /opt/ros/jazzy/setup.bash ]] || die "ROS 2 Jazzy setup is missing"
[[ -f "${SERVICE_TEMPLATE}" && -f "${ENV_TEMPLATE}" ]] ||
  die "service deployment templates are incomplete"
[[ -f "${WORKSPACE_ROOT}/correction_service/config/general_settings.yaml" ]] ||
  die "correction configuration is missing"
if systemctl is-active --quiet "${FLIGHT_SERVICE}" 2>/dev/null; then
  die "${FLIGHT_SERVICE} is active; stop it only in a confirmed safe maintenance window"
fi
if systemctl is-active --quiet "${SERVICE_NAME}" 2>/dev/null; then
  die "${SERVICE_NAME} is active; stop it before updating its shared overlay"
fi

correction_user="${SUDO_USER:-$(id -un)}"
[[ "${correction_user}" != "root" ]] ||
  die "run as the normal onboard user, not a root login shell"
correction_home="$(getent passwd "${correction_user}" | cut -d: -f6)"
[[ -n "${correction_home}" ]] || die "cannot resolve home for ${correction_user}"
readonly CAMERA_OVERLAY="${CORRECTION_CAMERA_OVERLAY_SETUP:-${correction_home}/vins_odin_calib/camera_ws/install/setup.bash}"
[[ -r "${CAMERA_OVERLAY}" ]] || die "calibrated camera overlay is missing: ${CAMERA_OVERLAY}"
source_setup /opt/ros/jazzy/setup.bash
source_setup "${CAMERA_OVERLAY}"
for command_name in colcon ros2 v4l2-ctl python3; do
  command -v "${command_name}" >/dev/null 2>&1 ||
    die "required command is missing: ${command_name}"
done
ros2 pkg prefix wasintek_gst_camera >/dev/null 2>&1 ||
  die "wasintek_gst_camera is not discoverable from ${CAMERA_OVERLAY}"
python3 - <<'PY'
import cv2
import yaml

assert hasattr(cv2, "aruco"), "OpenCV was built without aruco"
assert hasattr(cv2.aruco, "DICT_APRILTAG_36h11"), "OpenCV lacks tag36h11"
assert yaml.safe_load("ok: true")["ok"] is True
PY
(
  cd -- "${WORKSPACE_ROOT}"
  colcon build \
    --packages-select correction_interfaces correction_service \
    --symlink-install
)
source_setup "${WORKSPACE_ROOT}/install/setup.bash"
ros2 interface show correction_interfaces/srv/SetCorrection >/dev/null
ros2 pkg prefix correction_service >/dev/null

run_root install -d -m 0755 /etc/ros2-ardupilot
if [[ ! -e /etc/ros2-ardupilot/correction.env ]]; then
  env_stage="$(mktemp)"
  sed \
    -e "s|/home/nvidia/ros2-ardupilot-sitl-hardware|${WORKSPACE_ROOT}|g" \
    -e "s|/home/nvidia/vins_odin_calib|${correction_home}/vins_odin_calib|g" \
    "${ENV_TEMPLATE}" > "${env_stage}"
  run_root install -m 0644 "${env_stage}" /etc/ros2-ardupilot/correction.env
else
  printf '[correction-install] keeping /etc/ros2-ardupilot/correction.env\n'
fi

unit_stage="$(mktemp)"
trap 'rm -f -- "${env_stage:-}" "${unit_stage}"' EXIT
sed \
  -e "s|ONBOARD_USER|${correction_user}|g" \
  -e "s|ONBOARD_WORKSPACE_PATH|${WORKSPACE_ROOT}|g" \
  -e "s|ONBOARD_HOME_PATH|${correction_home}|g" \
  -e "s|CAMERA_OVERLAY_PATH|${CAMERA_OVERLAY}|g" \
  "${SERVICE_TEMPLATE}" > "${unit_stage}"
run_root install -m 0644 "${unit_stage}" "/etc/systemd/system/${SERVICE_NAME}"
run_root systemd-analyze verify "/etc/systemd/system/${SERVICE_NAME}"
run_root systemctl daemon-reload
run_root systemctl enable --now "${SERVICE_NAME}"

printf '[correction-install] installed and started %s for %s\n' \
  "${SERVICE_NAME}" "${correction_user}"
printf '[correction-install] node is idle; camera remains closed until explicit start\n'
printf '[correction-install] no arm/takeoff/flight command was sent\n'
