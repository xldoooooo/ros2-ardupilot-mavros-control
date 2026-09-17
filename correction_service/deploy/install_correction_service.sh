#!/usr/bin/env bash
# 构建并安装独立 correction_service；启动后 idle，绝不自动打开相机或控制飞行。

set -Eeuo pipefail

readonly SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
readonly WORKSPACE_ROOT="$(cd -- "${SCRIPT_DIR}/../.." && pwd -P)"
readonly SERVICE_NAME="odin-correction.service"
readonly SERVICE_TEMPLATE="${SCRIPT_DIR}/correction-service.service.example"
readonly ENV_TEMPLATE="${SCRIPT_DIR}/correction.env.example"
readonly FLIGHT_SERVICE="ros2-ardupilot-onboard.service"
readonly CORRECTION_PACKAGE_VERSION="2.0.0"

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

verify_manifest_version() {
  # 同时检查 source/install 清单，防止旧 overlay 静默提供 1.x 接口。
  local manifest="$1"
  [[ -r "${manifest}" ]] || die "package manifest is missing: ${manifest}"
  grep -Eq "<version>[[:space:]]*${CORRECTION_PACKAGE_VERSION//./\\.}[[:space:]]*</version>" \
    "${manifest}" || die "package manifest is not ${CORRECTION_PACKAGE_VERSION}: ${manifest}"
}

usage() {
  cat <<'EOF'
Usage: ./correction_service/deploy/install_correction_service.sh [--install-only]

Build correction_interfaces/correction_service, verify the in-package UVC driver
and configuration, install odin-correction.service, then enable/start
the independent node. The node remains idle with the camera closed until a
ground start request. The installer refuses an active flight service because it
updates the shared project overlay. It sends no arm/takeoff/flight command.

--install-only performs the same validation, build and unit/configuration
installation, but leaves odin-correction.service disabled and stopped.
EOF
}

install_only=false
case "${1:-}" in
  "") ;;
  --install-only) install_only=true ;;
  --help|-h) usage; exit 0 ;;
  *) die "unknown argument: $1" ;;
esac
(( $# <= 1 )) || die "unexpected extra argument: $2"
[[ -r /opt/ros/jazzy/setup.bash ]] || die "ROS 2 Jazzy setup is missing"
[[ -f "${SERVICE_TEMPLATE}" && -f "${ENV_TEMPLATE}" ]] ||
  die "service deployment templates are incomplete"
[[ -x "${WORKSPACE_ROOT}/scripts/onboard/start_onboard_correction.sh" ]] ||
  die "independent correction launcher is missing or not executable"
[[ -x "${WORKSPACE_ROOT}/scripts/onboard/stop_onboard_correction.sh" ]] ||
  die "independent correction stop helper is missing or not executable"
[[ -f "${WORKSPACE_ROOT}/correction_service/config/general_settings.yaml" ]] ||
  die "correction configuration is missing"
verify_manifest_version "${WORKSPACE_ROOT}/src/correction_interfaces/package.xml"
verify_manifest_version "${WORKSPACE_ROOT}/correction_service/package.xml"
grep -q 'expected_service_instance_id' \
  "${WORKSPACE_ROOT}/src/correction_interfaces/srv/StartCorrection.srv" ||
  die "source StartCorrection is not interface 2.0"
[[ -f "${WORKSPACE_ROOT}/src/correction_interfaces/srv/ClearWindow.srv" ]] ||
  die "source ClearWindow interface is missing"
[[ -f "${WORKSPACE_ROOT}/src/correction_interfaces/srv/ApplySavedCorrection.srv" ]] ||
  die "source ApplySavedCorrection interface is missing"
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
source_setup /opt/ros/jazzy/setup.bash
for command_name in colcon ros2 v4l2-ctl python3; do
  command -v "${command_name}" >/dev/null 2>&1 ||
    die "required command is missing: ${command_name}"
done
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
    --cmake-args -DAMENT_CMAKE_SYMLINK_INSTALL=OFF
)
source_setup "${WORKSPACE_ROOT}/install/local_setup.bash"
interfaces_prefix="$(ros2 pkg prefix correction_interfaces)"
service_prefix="$(ros2 pkg prefix correction_service)"
[[ -x "${service_prefix}/lib/correction_service/uvc_camera_node" ]] ||
  die "in-package UVC camera executable is missing"
verify_manifest_version "${interfaces_prefix}/share/correction_interfaces/package.xml"
verify_manifest_version "${service_prefix}/share/correction_service/package.xml"
ros2 interface show correction_interfaces/msg/CalibrationKeyframe >/dev/null
installed_status="$(ros2 interface show correction_interfaces/msg/CorrectionStatus)"
grep -q 'service_instance_id' <<< "${installed_status}" ||
  die "installed CorrectionStatus is not 2.0"
installed_start="$(ros2 interface show correction_interfaces/srv/StartCorrection)"
grep -q 'expected_window_revision' <<< "${installed_start}" ||
  die "installed StartCorrection is not 2.0"
ros2 interface show correction_interfaces/srv/ClearWindow >/dev/null
ros2 interface show correction_interfaces/srv/ApplySavedCorrection >/dev/null
ros2 interface show correction_interfaces/srv/SetCorrection >/dev/null

run_root install -d -m 0755 /etc/ros2-ardupilot
if [[ ! -e /etc/ros2-ardupilot/correction.env ]]; then
  env_stage="$(mktemp)"
  sed \
    -e "s|/home/nvidia/ros2-ardupilot-sitl-hardware|${WORKSPACE_ROOT}|g" \
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
  "${SERVICE_TEMPLATE}" > "${unit_stage}"
run_root install -m 0644 "${unit_stage}" "/etc/systemd/system/${SERVICE_NAME}"
run_root systemd-analyze verify "/etc/systemd/system/${SERVICE_NAME}"
run_root systemctl daemon-reload

if ${install_only}; then
  printf '[correction-install] installed %s for %s; it was not enabled or started\n' \
    "${SERVICE_NAME}" "${correction_user}"
  printf '[correction-install] verify this aircraft camera path and calibration, then rerun without --install-only\n'
  exit 0
fi

run_root systemctl enable --now "${SERVICE_NAME}"

printf '[correction-install] installed and started %s for %s\n' \
  "${SERVICE_NAME}" "${correction_user}"
printf '[correction-install] node is idle; camera remains closed until explicit start\n'
printf '[correction-install] no arm/takeoff/flight command was sent\n'
