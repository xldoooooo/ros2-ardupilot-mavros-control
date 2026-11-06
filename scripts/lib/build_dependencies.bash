#!/usr/bin/env bash
# Shared Ubuntu development dependencies for all controller-consuming builds.

# Native controller libraries declared by src/onboard_control/package.xml.
readonly -a CONTROL_BUILD_APT_PACKAGES=(libeigen3-dev nlohmann-json3-dev)
# Native Odin ingress, OpenCV AprilTag and Python bindings for correction_service.
readonly -a CORRECTION_BUILD_APT_PACKAGES=(pybind11-dev "ros-${ROS_DISTRO:-jazzy}-pybind11-vendor")

# Install only missing development packages; leave existing versions and services alone.
build_ensure_apt_packages() {
  local package status
  local -a missing_packages=() root_command=()
  for package in "$@"; do
    status="$(dpkg-query -W -f='${Status}' "${package}" 2>/dev/null)" || status=""
    [[ "${status}" == "install ok installed" ]] || missing_packages+=("${package}")
  done
  ((${#missing_packages[@]})) || return 0

  printf '[build-deps] Installing missing packages: %s\n' "${missing_packages[*]}"
  if ((EUID != 0)); then
    root_command=(sudo)
  fi
  "${root_command[@]}" apt-get update || return 1
  "${root_command[@]}" apt-get install -y --no-upgrade "${missing_packages[@]}"
}

# Install the shared controller build dependencies.
build_ensure_control_dependencies() {
  build_ensure_apt_packages "${CONTROL_BUILD_APT_PACKAGES[@]}"
}

# Keep native correction dependencies available at every correction build entry.
build_ensure_correction_dependencies() {
  build_ensure_apt_packages "${CORRECTION_BUILD_APT_PACKAGES[@]}"
}
