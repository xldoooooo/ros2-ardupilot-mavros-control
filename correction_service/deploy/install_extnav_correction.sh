#!/usr/bin/env bash
# 备份并部署任务 27 extnav 源码；只构建，不启动、重启或控制真实飞机。

set -Eeuo pipefail

readonly SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
readonly WORKSPACE_ROOT="$(cd -- "${SCRIPT_DIR}/../.." && pwd -P)"
readonly EXTNAV_ROOT="${EXTNAV_WORKSPACE:-/home/nvidia/vrpn_mavros}"
readonly EXTNAV_PACKAGE_ROOT="${EXTNAV_ROOT}/src/extnav_bridge"
readonly EXTNAV_SOURCE="${EXTNAV_PACKAGE_ROOT}/extnav_bridge/extnav_to_vision_pose.py"
readonly PATCH_SOURCE="${WORKSPACE_ROOT}/correction_service/extnav_patch/extnav_to_vision_pose.py"
readonly PATCH_PACKAGE="${WORKSPACE_ROOT}/correction_service/extnav_patch/package.xml"
readonly FLIGHT_SERVICE="ros2-ardupilot-onboard.service"

die() {
  printf '[extnav-correction-install] ERROR: %s\n' "$*" >&2
  exit 1
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
Usage: ./correction_service/deploy/install_extnav_correction.sh

Run as the normal onboard user in a confirmed unarmed maintenance window.
The script refuses an active flight service or extnav process, makes a targeted
backup with SHA-256, deploys the controlled extnav source, and rebuilds only
extnav_bridge. It never starts/restarts services and sends no flight command.

Override EXTNAV_WORKSPACE only when the production workspace is not
/home/nvidia/vrpn_mavros.
EOF
}

if [[ "${1:-}" == "--help" || "${1:-}" == "-h" ]]; then
  usage
  exit 0
fi
(( $# == 0 )) || die "unknown argument: $1"
[[ "$(id -u)" -ne 0 ]] || die "run as the normal onboard user, not root"
[[ -r /opt/ros/jazzy/setup.bash ]] || die "ROS 2 Jazzy setup is missing"
[[ -f "${WORKSPACE_ROOT}/src/correction_interfaces/package.xml" ]] ||
  die "correction_interfaces source is missing"
[[ -f "${PATCH_SOURCE}" && -f "${PATCH_PACKAGE}" ]] ||
  die "controlled extnav patch is incomplete"
[[ -f "${EXTNAV_SOURCE}" && -f "${EXTNAV_PACKAGE_ROOT}/package.xml" ]] ||
  die "production extnav source is missing under ${EXTNAV_PACKAGE_ROOT}"

if systemctl is-active --quiet "${FLIGHT_SERVICE}" 2>/dev/null; then
  die "${FLIGHT_SERVICE} is active; stop it only in a confirmed safe maintenance window"
fi
if pgrep -af '[e]xtnav_to_vision_pose' >/dev/null; then
  die "an extnav_to_vision_pose process is still running"
fi

source_setup /opt/ros/jazzy/setup.bash
(
  cd -- "${WORKSPACE_ROOT}"
  colcon build --packages-select correction_interfaces --symlink-install
)
source_setup "${WORKSPACE_ROOT}/install/setup.bash"

readonly BACKUP_ROOT="${EXTNAV_BACKUP_ROOT:-/home/nvidia/backups}"
readonly BACKUP_DIR="${BACKUP_ROOT}/extnav-task27-$(date '+%Y%m%d-%H%M%S')"
mkdir -p -- "${BACKUP_DIR}/extnav_bridge/extnav"
cp -a -- "${EXTNAV_SOURCE}" "${BACKUP_DIR}/extnav_bridge/extnav/"
cp -a -- "${EXTNAV_PACKAGE_ROOT}/package.xml" "${BACKUP_DIR}/extnav_bridge/"
(
  cd -- "${BACKUP_DIR}"
  sha256sum \
    extnav_bridge/extnav/extnav_to_vision_pose.py \
    extnav_bridge/package.xml > SHA256SUMS
  sha256sum -c SHA256SUMS
)

install -m 0644 "${PATCH_SOURCE}" "${EXTNAV_SOURCE}"
install -m 0644 "${PATCH_PACKAGE}" "${EXTNAV_PACKAGE_ROOT}/package.xml"
python3 -m py_compile "${EXTNAV_SOURCE}"
(
  cd -- "${EXTNAV_ROOT}"
  colcon build --packages-select extnav_bridge --symlink-install
)

printf '[extnav-correction-install] deployed and built extnav_bridge\n'
printf '[extnav-correction-install] targeted backup=%s\n' "${BACKUP_DIR}"
printf '[extnav-correction-install] flight service was not started or restarted\n'
printf '[extnav-correction-install] no arm/takeoff/flight command was sent\n'
