#!/usr/bin/env bash
# 一键构建并安装四组件机载飞控 systemd 服务；绝不发送解锁或起飞命令。

set -Eeuo pipefail

readonly SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
readonly WORKSPACE_ROOT="$(cd -- "${SCRIPT_DIR}/../../.." && pwd -P)"
readonly SERVICE_NAME="ros2-ardupilot-onboard.service"
readonly SERVICE_TEMPLATE="${WORKSPACE_ROOT}/system/systemd/ros2-ardupilot-onboard.service"
# Canonical USB access policy installed alongside the service.
readonly UDEV_RULE_TEMPLATE="${WORKSPACE_ROOT}/system/udev/99-odin-usb.rules"

usage() {
  cat <<'EOF'
Usage: ./src/onboard_control/deploy/install_onboard_service.sh [--install-only]

On Ubuntu 24.04/Jazzy, verify and build the onboard ROS packages, use the selected
project aircraft configuration, install the four-component systemd unit, then
enable and start it. Existing aircraft parameters are kept.

--install-only still builds, tests, runs the hardware-isolated smoke test and
installs the unit, but skips the hardware check and does not enable
or start the unit. Use it while extnav or aircraft hardware is incomplete.

Project Odin is built automatically. Run as the normal onboard user after
ROS 2 Jazzy, MAVROS and extnav are installed. The script may ask for sudo once. It sends no arm/takeoff command.
For safety it refuses to update an already active flight service.
EOF
}

die() {
  printf '[onboard-install] ERROR: %s\n' "$*" >&2
  exit 1
}

run_root() {
  if (( EUID == 0 )); then
    "$@"
  else
    command sudo "$@"
  fi
}

install_only=false
case "${1:-}" in
  "") ;;
  --install-only) install_only=true ;;
  --help|-h) usage; exit 0 ;;
  *) die "unknown argument: $1" ;;
esac
(( $# <= 1 )) || die "unexpected extra argument: $2"

[[ -r /opt/ros/jazzy/setup.bash ]] ||
  die "ROS 2 Jazzy is missing: /opt/ros/jazzy/setup.bash"
[[ -f "${SERVICE_TEMPLATE}" ]] || die "missing unit template: ${SERVICE_TEMPLATE}"
[[ -f "${UDEV_RULE_TEMPLATE}" ]] || die "missing udev rule: ${UDEV_RULE_TEMPLATE}"
source "${WORKSPACE_ROOT}/scripts/lib/runtime_common.bash"
source "${WORKSPACE_ROOT}/scripts/lib/project_config.bash"
project_config_load "${WORKSPACE_ROOT}"
[[ -r "${ONBOARD_ENV_FILE}" ]] || die "missing selected aircraft environment: ${ONBOARD_ENV_FILE}"
[[ -x "${WORKSPACE_ROOT}/src/onboard_control/deploy/build_onboard_control.sh" ]] ||
  die "missing build entry: ${WORKSPACE_ROOT}/src/onboard_control/deploy/build_onboard_control.sh"
[[ -x "${WORKSPACE_ROOT}/scripts/onboard/start_onboard_control.sh" ]] ||
  die "missing integrated launcher: ${WORKSPACE_ROOT}/scripts/onboard/start_onboard_control.sh"

if command systemctl is-active --quiet "${SERVICE_NAME}" 2>/dev/null; then
  die "${SERVICE_NAME} is active; stop it only in a confirmed safe maintenance window"
fi

onboard_user="${SUDO_USER:-$(id -un)}"
[[ "${onboard_user}" != "root" ]] ||
  die "run as the normal onboard user, not a root login shell"
onboard_home="$(getent passwd "${onboard_user}" | cut -d: -f6)"
[[ -n "${onboard_home}" ]] ||
  die "cannot resolve home directory for ${onboard_user}"

# 构建、单测和 localhost 隔离 smoke 不连接真实 MAVROS，也不发送飞行命令。
"${WORKSPACE_ROOT}/src/onboard_control/deploy/build_onboard_control.sh" --verify

unit_stage="$(mktemp)"
trap 'rm -f -- "${unit_stage}"' EXIT
sed \
  -e "s|ONBOARD_USER|${onboard_user}|g" \
  -e "s|ONBOARD_WORKSPACE_PATH|${WORKSPACE_ROOT}|g" \
  -e "s|ONBOARD_HOME_PATH|${onboard_home}|g" \
  "${SERVICE_TEMPLATE}" > "${unit_stage}"
run_root install -m 0644 "${unit_stage}" "/etc/systemd/system/${SERVICE_NAME}"
run_root systemd-analyze verify "/etc/systemd/system/${SERVICE_NAME}"
run_root systemctl daemon-reload
run_root install -d -m 0755 /etc/udev/rules.d
run_root install -m 0644 "${UDEV_RULE_TEMPLATE}" /etc/udev/rules.d/99-odin-usb.rules
run_root udevadm control --reload-rules

if ${install_only}; then
  printf '[onboard-install] Installed %s for %s; it was not enabled or started.\n' \
    "${SERVICE_NAME}" "${onboard_user}"
  printf '[onboard-install] Install extnav, connect and verify the aircraft hardware, then rerun this installer without --install-only.\n'
  printf '[onboard-install] No hardware check, arm or takeoff command was sent.\n'
  exit 0
fi

# --check 只做硬件路径和 ROS 包发现；任何歧义都在启动 systemd 前明确失败。
"${WORKSPACE_ROOT}/scripts/onboard/start_onboard_control.sh" --check
run_root systemctl enable --now "${SERVICE_NAME}"

printf '[onboard-install] Installed and started %s for %s\n' \
  "${SERVICE_NAME}" "${onboard_user}"
printf '[onboard-install] No arm or takeoff command was sent.\n'
