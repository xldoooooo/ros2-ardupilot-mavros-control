#!/usr/bin/env bash
# Build the pinned MAVROS retry and upstream plugin-parameter fixes in one overlay.
set -Eeuo pipefail
readonly deploy_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# Match the installed ABI; a newer source must not share the old build cache.
package_version="$(dpkg-query -W -f='${Version}' ros-jazzy-mavros)"
readonly source_version="${package_version%%-*}"
fix_patches=("${deploy_dir}/patches/mavros-2.15.1-param-retry.patch")
case "${source_version}" in
  2.15.1)
    source_sha=af9c8faa91f51b3c335263094689a329fa79ab252d1ef6962c149f113aa810ab
    default_workspace="${HOME}/mavros_param_fix_ws"
    fix_patches+=("${deploy_dir}/patches/mavros-2.15.1-plugin-parameters.patch")
    ;;
  2.16.0)
    source_sha=47d1f26b484a78b670744734e5c5bb310f29043899ba3ebf9694adb1c8c7bc9b
    default_workspace="${HOME}/mavros_param_fix_2.16.0_ws"
    ;;
  *)
    echo "Unsupported MAVROS ABI: ${source_version}; expected 2.15.1 or 2.16.0." >&2
    exit 1
    ;;
esac
readonly source_sha default_workspace fix_patches
readonly workspace="${MAVROS_FIX_WORKSPACE:-${default_workspace}}"
readonly archive="${MAVROS_SOURCE_ARCHIVE:-${workspace}/mavros-${source_version}.tar.gz}"
mkdir -p "${workspace}/src"
if [[ ! -f "${archive}" ]]; then
  curl -fL --connect-timeout 15 --max-time 180 \
    "https://codeload.github.com/mavlink/mavros/tar.gz/refs/tags/${source_version}" -o "${archive}"
fi
printf '%s  %s\n' "${source_sha}" "${archive}" | sha256sum --check
if [[ ! -d "${workspace}/src/mavros-${source_version}" ]]; then
  tar -xzf "${archive}" -C "${workspace}/src"
fi
cd "${workspace}/src/mavros-${source_version}"
for fix_patch in "${fix_patches[@]}"; do
  if patch --dry-run --forward -p1 < "${fix_patch}" >/dev/null 2>&1; then
    patch --forward -p1 < "${fix_patch}"
  else
    # A second install is allowed only when the exact patch is already applied.
    patch --dry-run --reverse -p1 < "${fix_patch}" >/dev/null
  fi
done
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
