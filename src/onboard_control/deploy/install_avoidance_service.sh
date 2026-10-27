#!/usr/bin/env bash
# Install the independent avoidance unit using already built native workspaces.
set -Eeuo pipefail

# Resolve the current checkout instead of assuming an onboard home directory.
readonly project_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../../.." && pwd -P)"
# This unit owns only the native planner and avoidance bridge.
readonly service_name=ros2-ardupilot-avoidance.service

install_only=false
case "${1:-}" in
  '') ;;
  --install-only) install_only=true ;;
  -h|--help)
    echo 'Usage: install_avoidance_service.sh [--install-only]'
    echo 'Build avoidance first. Stop existing avoidance before installation.'
    echo '--install-only installs without enabling or starting the service.'
    exit 0 ;;
  *) echo "Unknown option: $1" >&2; exit 2 ;;
esac
(( $# <= 1 )) || exit 2

if systemctl is-active --quiet "${service_name}"; then
  echo "Stop ${service_name} before updating its unit." >&2
  exit 1
fi
onboard_user="${SUDO_USER:-$(id -un)}"
onboard_home="$(getent passwd "${onboard_user}" | cut -d: -f6)"
[[ -n "${onboard_home}" && "${onboard_user}" != root ]] || {
  echo 'Run this installer as the normal onboard user.' >&2; exit 1;
}
"${project_root}/scripts/onboard/start_avoidance.sh" --check
unit_stage="$(mktemp)"
trap 'rm -f -- "${unit_stage}"' EXIT
sed -e "s|ONBOARD_USER|${onboard_user}|g" \
  -e "s|ONBOARD_WORKSPACE_PATH|${project_root}|g" \
  -e "s|ONBOARD_HOME_PATH|${onboard_home}|g" \
  "${project_root}/system/systemd/${service_name}" > "${unit_stage}"
sudo install -m 0644 "${unit_stage}" "/etc/systemd/system/${service_name}"
sudo systemd-analyze verify "/etc/systemd/system/${service_name}"
sudo systemctl daemon-reload
if ! ${install_only}; then
  sudo systemctl enable --now "${service_name}"
fi
echo "Installed ${service_name}; no flight command was sent."
