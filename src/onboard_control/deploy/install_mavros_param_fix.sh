#!/usr/bin/env bash
# Build the pinned MAVROS parameter retry fix in a separate, reversible overlay.
set -Eeuo pipefail
readonly deploy_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly workspace="${MAVROS_FIX_WORKSPACE:-${HOME}/mavros_param_fix_ws}"
readonly archive="${MAVROS_SOURCE_ARCHIVE:-${workspace}/mavros-2.15.1.tar.gz}"
readonly source_sha=af9c8faa91f51b3c335263094689a329fa79ab252d1ef6962c149f113aa810ab
readonly fix_patch="${deploy_dir}/patches/mavros-2.15.1-param-retry.patch"
if ! dpkg-query -W -f='${Version}' ros-jazzy-mavros | grep -q '^2\.15\.1-'; then
  echo 'This patch requires the installed Jazzy MAVROS 2.15.1 ABI.' >&2
  exit 1
fi
mkdir -p "${workspace}/src"
if [[ ! -f "${archive}" ]]; then
  curl -fL --connect-timeout 15 --max-time 180 \
    https://codeload.github.com/mavlink/mavros/tar.gz/refs/tags/2.15.1 -o "${archive}"
fi
printf '%s  %s\n' "${source_sha}" "${archive}" | sha256sum --check
if [[ ! -d "${workspace}/src/mavros-2.15.1" ]]; then
  tar -xzf "${archive}" -C "${workspace}/src"
fi
cd "${workspace}/src/mavros-2.15.1"
if patch --dry-run --forward -p1 < "${fix_patch}" >/dev/null 2>&1; then
  patch --forward -p1 < "${fix_patch}"
else
  # A second install is allowed only when the exact patch is already applied.
  patch --dry-run --reverse -p1 < "${fix_patch}" >/dev/null
fi
set +u
source /opt/ros/jazzy/setup.bash
set -u
cd "${workspace}"
MAKEFLAGS="-j${MAVROS_BUILD_JOBS:-2}" \
  CMAKE_BUILD_PARALLEL_LEVEL="${MAVROS_BUILD_JOBS:-2}" colcon build \
  --packages-select mavros --allow-overriding mavros \
  --packages-ignore libmavconn mavros_msgs \
  --cmake-args -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release
printf 'Built overlay. Set MAVROS_OVERLAY_SETUP=%s/install/local_setup.bash in onboard.env.\n' "${workspace}"
echo 'No running service has been restarted.'
