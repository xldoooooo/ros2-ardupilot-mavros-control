#!/usr/bin/env bash
# 一键配置独立机载视频 systemd 服务；MediaMTX 必须已安装到系统。

set -Eeuo pipefail

readonly SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
readonly WORKSPACE_ROOT="$(cd -- "${SCRIPT_DIR}/../.." && pwd -P)"
readonly SERVICE_NAME="video-service.service"
readonly SERVICE_TEMPLATE="${WORKSPACE_ROOT}/system/systemd/video-service.service"
readonly MEDIAMTX_VERSION="1.20.0"

usage() {
  cat <<'EOF'
Usage: ./video_service/deploy/install_onboard_video_service.sh [--install-only]

On Ubuntu 24.04/Jazzy ARM64, verify system FFmpeg, v4l2-ctl and MediaMTX,
build the ROS interface when needed, use project configuration, install writable
media directories and video-service.service, then enable and start the service.
Selected aircraft camera configuration is kept.

--install-only performs the same validation, build and unit/configuration
installation, but leaves video-service.service disabled and stopped.

Install the documented OS dependencies first, then run this as the normal
onboard user. The script may ask for sudo once. It never manages the
flight-control service.
EOF
}

die() {
  printf '[video-install] ERROR: %s\n' "$*" >&2
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

[[ -f "${SERVICE_TEMPLATE}" ]] || die "missing unit template: ${SERVICE_TEMPLATE}"
source "${WORKSPACE_ROOT}/scripts/lib/runtime_common.bash"
source "${WORKSPACE_ROOT}/scripts/lib/project_config.bash"
project_config_load "${WORKSPACE_ROOT}"
[[ -f "${AIRCRAFT_CONFIG_DIR}/video/camera.conf" ]] || die "missing selected camera config"
[[ -f "${AIRCRAFT_CONFIG_DIR}/video/lens.conf" ]] || die "missing selected lens config"
[[ "$(uname -m)" == "aarch64" ]] ||
  die "this onboard installer currently supports Ubuntu ARM64 only"

# sudo 启动时仍以原登录用户运行服务；普通启动时直接使用当前用户。
video_user="${SUDO_USER:-$(id -un)}"
[[ "${video_user}" != "root" ]] ||
  die "run as the normal onboard user, not a root login shell"
video_home="$(getent passwd "${video_user}" | cut -d: -f6)"
[[ -n "${video_home}" ]] || die "cannot resolve home directory for ${video_user}"

for command_name in ffmpeg v4l2-ctl rsync ssh; do
  command -v "${command_name}" >/dev/null 2>&1 ||
    die "missing ${command_name}; install the packages documented in video_service/README.md"
done
[[ -x /usr/local/bin/mediamtx ]] ||
  die "missing /usr/local/bin/mediamtx; install MediaMTX as documented in video_service/README.md"
[[ "$(/usr/local/bin/mediamtx --version 2>/dev/null || true)" == "v${MEDIAMTX_VERSION}" ]] ||
  die "MediaMTX v${MEDIAMTX_VERSION} is required at /usr/local/bin/mediamtx"

# 新 sparse checkout 不要求用户先记住另一条构建命令。
if [[ ! -f "${WORKSPACE_ROOT}/install/setup.bash" ]]; then
  "${WORKSPACE_ROOT}/src/onboard_control/deploy/build_onboard_control.sh" --verify
fi

# Media now stays in a private user cache, not the former aircraft FTP root.
run_root install -d -o "${video_user}" -g "${video_user}" -m 0750 \
  "${video_home}/.local/share/ardupilot-media/cache/videos" \
  "${video_home}/.local/share/ardupilot-media/cache/images"

unit_stage="$(mktemp)"
trap 'rm -f -- "${unit_stage}"' EXIT
sed \
  -e "s|ONBOARD_USER|${video_user}|g" \
  -e "s|ONBOARD_WORKSPACE_PATH|${WORKSPACE_ROOT}|g" \
  -e "s|ONBOARD_HOME_PATH|${video_home}|g" \
  "${SERVICE_TEMPLATE}" > "${unit_stage}"
run_root install -m 0644 "${unit_stage}" "/etc/systemd/system/${SERVICE_NAME}"
run_root systemd-analyze verify "/etc/systemd/system/${SERVICE_NAME}"
run_root systemctl daemon-reload

if ${install_only}; then
  printf '[video-install] Installed %s for %s; it was not enabled or started.\n' \
    "${SERVICE_NAME}" "${video_user}"
  printf '[video-install] Verify the camera path and calibration on this aircraft, then rerun without --install-only.\n'
  exit 0
fi

run_root systemctl enable --now "${SERVICE_NAME}"

printf '[video-install] Installed and started %s for %s\n' \
  "${SERVICE_NAME}" "${video_user}"
printf '[video-install] Camera remains closed until ROS requests enabled=true.\n'
