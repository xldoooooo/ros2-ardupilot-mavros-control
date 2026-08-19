#!/usr/bin/env bash
# 一键构建并安装四组件机载飞控 systemd 服务；绝不发送解锁或起飞命令。

set -Eeuo pipefail

readonly SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
readonly WORKSPACE_ROOT="$(cd -- "${SCRIPT_DIR}/../../.." && pwd -P)"
readonly SERVICE_NAME="ros2-ardupilot-onboard.service"
readonly SERVICE_TEMPLATE="${SCRIPT_DIR}/onboard-control.service.example"
readonly ENV_TEMPLATE="${SCRIPT_DIR}/onboard.env.example"

usage() {
  cat <<'EOF'
Usage: ./src/onboard_control/deploy/install_onboard_service.sh

On Ubuntu 24.04/Jazzy, verify and build the onboard ROS packages, create the
per-aircraft environment file when absent, install the four-component systemd
unit, then enable and start it. Existing /etc onboard configuration is kept.

Run as the normal onboard user after ROS 2 Jazzy, MAVROS, Odin and extnav are
installed. The script may ask for sudo once. It sends no arm/takeoff command.
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

if [[ "${1:-}" == "--help" || "${1:-}" == "-h" ]]; then
  usage
  exit 0
fi
(( $# == 0 )) || die "unknown argument: $1"

[[ -r /opt/ros/jazzy/setup.bash ]] ||
  die "ROS 2 Jazzy is missing: /opt/ros/jazzy/setup.bash"
[[ -f "${SERVICE_TEMPLATE}" ]] || die "missing unit template: ${SERVICE_TEMPLATE}"
[[ -f "${ENV_TEMPLATE}" ]] || die "missing environment template: ${ENV_TEMPLATE}"
[[ -x "${WORKSPACE_ROOT}/build_onboard_control.sh" ]] ||
  die "missing build entry: ${WORKSPACE_ROOT}/build_onboard_control.sh"
[[ -x "${WORKSPACE_ROOT}/start_onboard_control.sh" ]] ||
  die "missing integrated launcher: ${WORKSPACE_ROOT}/start_onboard_control.sh"

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
"${WORKSPACE_ROOT}/build_onboard_control.sh" --verify

run_root install -d -m 0755 /etc/ros2-ardupilot
if [[ ! -e /etc/ros2-ardupilot/onboard.env ]]; then
  env_stage="$(mktemp)"
  sed \
    -e 's|ONBOARD_ROS_DISTRO_VALUE|jazzy|g' \
    -e "s|ONBOARD_WORKSPACE_PATH|${WORKSPACE_ROOT}|g" \
    "${ENV_TEMPLATE}" > "${env_stage}"
  run_root install -m 0644 "${env_stage}" /etc/ros2-ardupilot/onboard.env
else
  printf '[onboard-install] Keeping existing /etc/ros2-ardupilot/onboard.env\n'
fi

unit_stage="$(mktemp)"
trap 'rm -f -- "${env_stage:-}" "${unit_stage}"' EXIT
sed \
  -e "s|ONBOARD_USER|${onboard_user}|g" \
  -e "s|ONBOARD_WORKSPACE_PATH|${WORKSPACE_ROOT}|g" \
  -e "s|ONBOARD_HOME_PATH|${onboard_home}|g" \
  "${SERVICE_TEMPLATE}" > "${unit_stage}"
run_root install -m 0644 "${unit_stage}" "/etc/systemd/system/${SERVICE_NAME}"
run_root systemd-analyze verify "/etc/systemd/system/${SERVICE_NAME}"
run_root systemctl daemon-reload

# --check 只做硬件路径和 ROS 包发现；任何歧义都在启动 systemd 前明确失败。
ONBOARD_ENV_FILE=/etc/ros2-ardupilot/onboard.env \
  "${WORKSPACE_ROOT}/start_onboard_control.sh" --check
run_root systemctl enable --now "${SERVICE_NAME}"

printf '[onboard-install] Installed and started %s for %s\n' \
  "${SERVICE_NAME}" "${onboard_user}"
printf '[onboard-install] No arm or takeoff command was sent.\n'
