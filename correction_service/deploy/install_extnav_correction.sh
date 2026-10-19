#!/usr/bin/env bash
# Back up and natively build the C++ bridge; never start services or send flight commands.

set -Eeuo pipefail

readonly SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
readonly WORKSPACE_ROOT="$(cd -- "${SCRIPT_DIR}/../.." && pwd -P)"
readonly EXTNAV_ROOT="${EXTNAV_WORKSPACE:-/home/nvidia/vrpn_mavros}"
readonly EXTNAV_PACKAGE_ROOT="${EXTNAV_ROOT}/src/extnav_bridge"
readonly PATCH_ROOT="${WORKSPACE_ROOT}/correction_service/extnav_patch"
readonly FLIGHT_SERVICE="ros2-ardupilot-onboard.service"
readonly CORRECTION_PACKAGE_VERSION="2.0.0"
readonly INSTALLER_PID="${BASHPID}"

die() {
  # Stop before touching the installed bridge when prerequisites are missing.
  printf '[extnav-correction-install] ERROR: %s\n' "$*" >&2
  exit 1
}

source_setup() {
  # ROS setup files legitimately read optional environment variables.
  set +u
  # shellcheck disable=SC1090
  source "$1"
  set -u
}

verify_manifest_version() {
  # Keep the correction wire contract at 2.0 during the implementation migration.
  [[ -r "$1" ]] || die "package manifest is missing: $1"
  grep -Eq "<version>[[:space:]]*${CORRECTION_PACKAGE_VERSION//./\\.}[[:space:]]*</version>" "$1" ||
    die "package manifest is not ${CORRECTION_PACKAGE_VERSION}: $1"
}

usage() {
  # Installation requires an already stopped, unarmed maintenance window.
  cat <<'HELP'
Usage: ./correction_service/deploy/install_extnav_correction.sh

Back up and replace extnav_bridge with its native C++ implementation, then build
in Release mode. An active flight service or extnav process is refused.
The old source/build/install are restored if migration or build fails.
No service is started/restarted and no flight command is sent.

Override EXTNAV_WORKSPACE when it is not /home/nvidia/vrpn_mavros.
HELP
}

if [[ "${1:-}" == --help || "${1:-}" == -h ]]; then usage; exit 0; fi
(( $# == 0 )) || die "unknown argument: $1"
[[ "$(id -u)" -ne 0 ]] || die "run as the normal onboard user, not root"
[[ -r /opt/ros/jazzy/setup.bash ]] || die "ROS 2 Jazzy setup is missing"
for required_file in CMakeLists.txt package.xml include/extnav_bridge/math.hpp \
  src/extnav_to_vision_pose.cpp scripts/extnav_to_vision_pose; do
  [[ -f "${PATCH_ROOT}/${required_file}" ]] || die "native bridge file missing: ${required_file}"
done
verify_manifest_version "${WORKSPACE_ROOT}/src/correction_interfaces/package.xml"
verify_manifest_version "${PATCH_ROOT}/package.xml"
[[ -f "${EXTNAV_PACKAGE_ROOT}/package.xml" ]] || die "extnav_bridge is missing under ${EXTNAV_ROOT}"
if systemctl is-active --quiet "${FLIGHT_SERVICE}" 2>/dev/null; then
  die "${FLIGHT_SERVICE} is active; stop it only in an unarmed maintenance window"
fi
if pgrep -af '[e]xtnav_to_vision_pose' >/dev/null; then die "an extnav process is still running"; fi

source_setup /opt/ros/jazzy/setup.bash
(
  cd -- "${WORKSPACE_ROOT}"
  colcon build --packages-select correction_interfaces --cmake-args -DAMENT_CMAKE_SYMLINK_INSTALL=OFF
)
source_setup "${WORKSPACE_ROOT}/install/setup.bash"
interfaces_prefix="$(ros2 pkg prefix correction_interfaces)"
verify_manifest_version "${interfaces_prefix}/share/correction_interfaces/package.xml"
installed_status="$(ros2 interface show correction_interfaces/msg/ExtnavCorrectionStatus)"
grep -q final_sample_revision <<< "${installed_status}" ||
  die "installed ExtnavCorrectionStatus is not interface 2.0"
ros2 interface show correction_interfaces/srv/SetCorrection >/dev/null

readonly BACKUP_ROOT="${EXTNAV_BACKUP_ROOT:-/home/nvidia/backups}"
readonly BACKUP_DIR="${BACKUP_ROOT}/extnav-cpp-$(date '+%Y%m%d-%H%M%S')"
mkdir -p -- "${BACKUP_DIR}/previous" "${BACKUP_DIR}/failed" "${BACKUP_DIR}/staging"
cp -a -- "${PATCH_ROOT}/." "${BACKUP_DIR}/staging/"
(
  cd -- "${BACKUP_DIR}/staging"
  sha256sum CMakeLists.txt package.xml include/extnav_bridge/math.hpp \
    src/extnav_to_vision_pose.cpp scripts/extnav_to_vision_pose > SHA256SUMS
  sha256sum -c SHA256SUMS
)

# ament_python and ament_cmake cannot reuse the same package build/install cache.
declare -a swapped_parts=()
rollback() {
  # Restore only the package directories moved by this installation attempt.
  local exit_code=$? part
  (( BASHPID == INSTALLER_PID )) || exit "${exit_code}"
  trap - ERR
  for part in "${swapped_parts[@]}"; do
    if [[ -e "${EXTNAV_ROOT}/${part}/extnav_bridge" || -L "${EXTNAV_ROOT}/${part}/extnav_bridge" ]]; then
      mkdir -p "${BACKUP_DIR}/failed/${part}"
      mv -- "${EXTNAV_ROOT}/${part}/extnav_bridge" "${BACKUP_DIR}/failed/${part}/"
    fi
    if [[ -e "${BACKUP_DIR}/previous/${part}/extnav_bridge" ]]; then
      mv -- "${BACKUP_DIR}/previous/${part}/extnav_bridge" "${EXTNAV_ROOT}/${part}/"
    fi
  done
  printf '[extnav-correction-install] build failed; previous bridge restored; backup=%s\n' \
    "${BACKUP_DIR}" >&2
  exit "${exit_code}"
}
trap rollback ERR
for part in src build install; do
  mkdir -p -- "${EXTNAV_ROOT}/${part}" "${BACKUP_DIR}/previous/${part}"
  if [[ -e "${EXTNAV_ROOT}/${part}/extnav_bridge" || -L "${EXTNAV_ROOT}/${part}/extnav_bridge" ]]; then
    mv -- "${EXTNAV_ROOT}/${part}/extnav_bridge" "${BACKUP_DIR}/previous/${part}/"
  fi
  swapped_parts+=("${part}")
done
mv -- "${BACKUP_DIR}/staging" "${EXTNAV_PACKAGE_ROOT}"
(
  cd -- "${EXTNAV_ROOT}"
  colcon build --base-paths "${EXTNAV_PACKAGE_ROOT}" --packages-select extnav_bridge \
    --cmake-args -DCMAKE_BUILD_TYPE=Release -DAMENT_CMAKE_SYMLINK_INSTALL=OFF
)
[[ -x "${EXTNAV_ROOT}/install/extnav_bridge/lib/extnav_bridge/extnav_to_vision_pose_native" ]]
trap - ERR
printf '[extnav-correction-install] native extnav_bridge built; backup=%s\n' "${BACKUP_DIR}"
printf '[extnav-correction-install] flight service was not started or restarted\n'
