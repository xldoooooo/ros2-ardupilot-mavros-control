#!/usr/bin/env bash
# 为 Ubuntu 24.04 amd64/arm64 安装机载工作区依赖；不启动服务或访问飞控硬件。

set -Eeuo pipefail

readonly ROS_DISTRO="jazzy"
readonly ROS_KEY_URL="https://raw.githubusercontent.com/ros/rosdistro/master/ros.key"
readonly ROS_KEY_FINGERPRINT="C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654"
readonly ROS_KEYRING="/usr/share/keyrings/ros-archive-keyring.gpg"
readonly ROS_SOURCE="/etc/apt/sources.list.d/ros2.list"
readonly MEDIAMTX_VERSION="1.20.0"
readonly ROS_APT_MIRROR="${ROS_APT_MIRROR:-http://packages.ros.org/ros2/ubuntu}"

skip_mediamtx=false
skip_geographiclib=false
with_odin_deps=false
with_camera_deps=false

usage() {
  cat <<'EOF'
Usage: setup_onboard_dependencies.sh [--skip-mediamtx] [--skip-geographiclib]
       [--with-odin-deps] [--with-camera-deps]

Install the ROS 2 Jazzy build/runtime, MAVROS, correction/video dependencies,
GeographicLib datasets and architecture-matched MediaMTX v1.20.0 on Ubuntu
24.04 amd64 or arm64. The script does not enable or start any service and does
not install Odin or extnav, which are aircraft-specific external drivers.
--with-odin-deps adds the public build/runtime dependencies for the authorized
Odin source package; it does not download or build that external source.
--with-camera-deps adds the public GStreamer build/runtime dependencies for the
Jetson camera source; NVIDIA hardware plugins must already match the platform.
EOF
}

die() {
  printf '[onboard-deps] ERROR: %s\n' "$*" >&2
  exit 1
}

run_root() {
  if (( EUID == 0 )); then
    "$@"
  else
    command sudo "$@"
  fi
}

while (( $# )); do
  case "$1" in
    --skip-mediamtx) skip_mediamtx=true ;;
    --skip-geographiclib) skip_geographiclib=true ;;
    --with-odin-deps) with_odin_deps=true ;;
    --with-camera-deps) with_camera_deps=true ;;
    -h|--help) usage; exit 0 ;;
    *) die "unknown argument: $1" ;;
  esac
  shift
done

[[ -r /etc/os-release ]] || die "cannot read /etc/os-release"
# shellcheck disable=SC1091
source /etc/os-release
[[ "${ID:-}" == "ubuntu" && "${VERSION_ID:-}" == "24.04" ]] ||
  die "only Ubuntu 24.04 is supported (found ${PRETTY_NAME:-unknown})"

architecture="$(dpkg --print-architecture)"
case "${architecture}" in
  amd64)
    mediamtx_arch="amd64"
    mediamtx_sha256="952d5f7d31d1b448ab4da4509550594c511d42636db9d7bb175d377f4ede81df"
    ;;
  arm64)
    mediamtx_arch="arm64"
    mediamtx_sha256="6aa3c03da7b6477f1e110c8e18e819cf9ef121e8981b52b8f8219982dae35f2f"
    ;;
  *) die "unsupported architecture: ${architecture} (expected amd64 or arm64)" ;;
esac

[[ "${ROS_APT_MIRROR}" =~ ^https?://[^[:space:]]+/ros2/ubuntu/?$ ]] ||
  die "ROS_APT_MIRROR must be an http(s) URL ending in /ros2/ubuntu"

printf '[onboard-deps] Configuring Ubuntu universe and the ROS 2 apt source\n'
run_root apt-get update
run_root apt-get install -y ca-certificates curl gnupg software-properties-common
run_root add-apt-repository -y universe

stage_dir="$(mktemp -d)"
trap 'rm -rf -- "${stage_dir}"' EXIT
mkdir -m 0700 "${stage_dir}/gnupg"
curl -fsSL --connect-timeout 15 --max-time 120 --retry 2 \
  --proto '=https' --tlsv1.2 "${ROS_KEY_URL}" -o "${stage_dir}/ros.key"
downloaded_fingerprint="$(gpg --homedir "${stage_dir}/gnupg" --show-keys --with-colons "${stage_dir}/ros.key" |
  awk -F: '$1 == "fpr" { print $10; exit }')"
[[ "${downloaded_fingerprint}" == "${ROS_KEY_FINGERPRINT}" ]] ||
  die "unexpected ROS apt key fingerprint: ${downloaded_fingerprint:-missing}"
run_root install -m 0644 "${stage_dir}/ros.key" "${ROS_KEYRING}"

# 保留旧文件备份并只注释其中 ROS 2 行，防止同一仓库的重复定义或 Signed-By 冲突。
for old_source in /etc/apt/sources.list /etc/apt/sources.list.d/*.list; do
  [[ -f "${old_source}" && "${old_source}" != "${ROS_SOURCE}" ]] || continue
  grep -Eq '^[[:space:]]*deb(-src)?[[:space:]].*/ros2/ubuntu([[:space:]/]|$)' \
    "${old_source}" || continue
  backup="${old_source}.before-onboard-deps"
  [[ -e "${backup}" ]] || run_root cp -a -- "${old_source}" "${backup}"
  awk '
    /^[[:space:]]*deb(-src)?[[:space:]].*\/ros2\/ubuntu([[:space:]\/]|$)/ {
      print "# disabled by setup_onboard_dependencies.sh: " $0; next
    }
    { print }
  ' "${old_source}" > "${stage_dir}/source.list"
  run_root install -m 0644 "${stage_dir}/source.list" "${old_source}"
  printf '[onboard-deps] Disabled duplicate ROS 2 entries in %s (backup: %s)\n' \
    "${old_source}" "${backup}"
done

printf 'deb [arch=%s signed-by=%s] %s noble main\n' \
  "${architecture}" "${ROS_KEYRING}" "${ROS_APT_MIRROR%/}" > "${stage_dir}/ros2.list"
run_root install -m 0644 "${stage_dir}/ros2.list" "${ROS_SOURCE}"

run_root apt-get update
run_root apt-get install -y \
  build-essential cmake git libeigen3-dev \
  python3-colcon-common-extensions python3-numpy python3-opencv python3-pytest \
  python3-rosdep python3-vcstool python3-yaml \
  python3-venv \
  ffmpeg geographiclib-tools v4l-utils \
  "ros-${ROS_DISTRO}-ros-base" \
  "ros-${ROS_DISTRO}-ament-cmake-gtest" \
  "ros-${ROS_DISTRO}-eigen3-cmake-module" \
  "ros-${ROS_DISTRO}-geographic-msgs" \
  "ros-${ROS_DISTRO}-mavros" \
  "ros-${ROS_DISTRO}-mavros-extras" \
  "ros-${ROS_DISTRO}-rosidl-default-generators" \
  "ros-${ROS_DISTRO}-tf2-geometry-msgs"

# 新安装用户通常尚无串口权限；组变更需重新登录后进入交互会话。
onboard_user="${SUDO_USER:-$(id -un)}"
if [[ "${onboard_user}" != root ]]; then
  run_root usermod -aG dialout,video,plugdev "${onboard_user}"
fi

if ${with_odin_deps}; then
  run_root apt-get install -y \
    libopencv-dev libpcl-dev libssl-dev libusb-1.0-0-dev libyaml-cpp-dev pkg-config \
    "ros-${ROS_DISTRO}-ament-index-cpp" \
    "ros-${ROS_DISTRO}-cv-bridge" \
    "ros-${ROS_DISTRO}-image-transport" \
    "ros-${ROS_DISTRO}-message-filters" \
    "ros-${ROS_DISTRO}-pcl-conversions" \
    "ros-${ROS_DISTRO}-rviz2" \
    "ros-${ROS_DISTRO}-tf2-ros" \
    "ros-${ROS_DISTRO}-visualization-msgs"
fi

if ${with_camera_deps}; then
  run_root apt-get install -y \
    pkg-config libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
    gstreamer1.0-tools gstreamer1.0-plugins-base gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-bad
fi

if ! ${skip_geographiclib}; then
  geographic_installer="/opt/ros/${ROS_DISTRO}/lib/mavros/install_geographiclib_datasets.sh"
  [[ -x "${geographic_installer}" ]] ||
    die "MAVROS GeographicLib installer is missing: ${geographic_installer}"
  # SourceForge 不可达时明确失败，避免上游 wget 无限重试卡住整次部署。
  run_root timeout 300 "${geographic_installer}"
  [[ -s /usr/share/GeographicLib/geoids/egm96-5.pgm ||
     -s /usr/local/share/GeographicLib/geoids/egm96-5.pgm ]] ||
    die "MAVROS installer returned without the required egm96-5.pgm dataset"
fi

if ! ${skip_mediamtx}; then
  archive="mediamtx_v${MEDIAMTX_VERSION}_linux_${mediamtx_arch}.tar.gz"
  curl -fL --connect-timeout 15 --max-time 300 --retry 2 --proto '=https' --tlsv1.2 \
    "https://github.com/bluenviron/mediamtx/releases/download/v${MEDIAMTX_VERSION}/${archive}" \
    -o "${stage_dir}/${archive}"
  printf '%s  %s\n' "${mediamtx_sha256}" "${archive}" |
    (cd "${stage_dir}" && sha256sum -c -)
  tar -xzf "${stage_dir}/${archive}" -C "${stage_dir}" mediamtx
  run_root install -m 0755 "${stage_dir}/mediamtx" /usr/local/bin/mediamtx
fi

[[ -r "/opt/ros/${ROS_DISTRO}/setup.bash" ]] || die "ROS setup file is missing"
command -v ffmpeg >/dev/null || die "ffmpeg is missing after installation"
command -v v4l2-ctl >/dev/null || die "v4l2-ctl is missing after installation"
if ! ${skip_mediamtx}; then
  /usr/local/bin/mediamtx --version | grep -Fq "v${MEDIAMTX_VERSION}" ||
    die "installed MediaMTX did not report v${MEDIAMTX_VERSION}"
fi

printf '[onboard-deps] Dependencies installed for Ubuntu 24.04/%s.\n' "${architecture}"
printf '[onboard-deps] No service was enabled or started; no arm/takeoff command was sent.\n'
printf '[onboard-deps] Odin and extnav remain external, aircraft-specific prerequisites.\n'
