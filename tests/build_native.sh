#!/usr/bin/env bash
# Build selected local test dependencies; start no services or hardware.
set -eo pipefail
project_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
planner_root="${AVOIDANCE_WORKSPACE:-${project_root%/*}/dyn_small_obs_avoidance-ros2}"
# Keep the complete build available; explicit packages build their local dependencies too.
default_packages=(guided_interfaces correction_interfaces correction_native onboard_control avoidance_bridge guided_sim extnav_bridge)
packages=()
list_only=false
while (($#)); do
    case "$1" in
        --packages)
            shift
            while (($#)) && [[ "$1" != --* ]]; do
                packages+=("$1")
                shift
            done
            if ((${#packages[@]} == 0)); then
                echo "--packages requires at least one package" >&2
                exit 2
            fi
            ;;
        --list) list_only=true; shift ;;
        --help|-h)
            echo "Usage: bash tests/build_native.sh [--packages PACKAGE ...] [--list]"
            echo "Packages: ${default_packages[*]} (default: all; dependencies included)"
            exit 0 ;;
        *) echo "Unknown option: $1" >&2; exit 2 ;;
    esac
done
((${#packages[@]})) || packages=("${default_packages[@]}")
main_packages=()
build_extnav=false
needs_planner=false
for package in "${packages[@]}"; do
    case "$package" in
        extnav_bridge)
            build_extnav=true ;;
        avoidance_bridge)
            needs_planner=true
            main_packages+=("$package") ;;
        guided_interfaces|correction_interfaces|correction_native|onboard_control|guided_sim)
            main_packages+=("$package") ;;
        *) echo "Unknown test package: $package" >&2; exit 2 ;;
    esac
done
if "$build_extnav" && [[ " ${main_packages[*]} " != *" correction_interfaces "* ]]; then
    main_packages+=(correction_interfaces)
fi
cd "$project_root"
# Use all CPUs, including builds requested through a narrower package selection.
export CMAKE_BUILD_PARALLEL_LEVEL="$(nproc)"
export MAKEFLAGS="-j$(nproc)"
main_command=(colcon build --base-paths src --packages-up-to "${main_packages[@]}"
    --parallel-workers "$(nproc)" --cmake-force-configure
    --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
    -DAMENT_CMAKE_SYMLINK_INSTALL=OFF -DPython3_EXECUTABLE="${project_root}/.venv/bin/python")
extnav_command=(colcon --log-base log/test-extnav build --base-paths correction_service/extnav_patch
    --build-base build/test-extnav --install-base build/test-extnav/install
    --packages-select extnav_bridge --parallel-workers "$(nproc)"
    --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON -DEXTNAV_WITH_CORRECTION=ON
    -DPython3_EXECUTABLE="${project_root}/.venv/bin/python")
if "$list_only"; then
    printf '%q ' "${main_command[@]}"; printf '\n'
    if "$build_extnav"; then
        printf '%q ' "${extnav_command[@]}"; printf '\n'
    fi
    exit 0
fi
source /opt/ros/jazzy/setup.bash
if [[ -f "${planner_root}/install/local_setup.bash" ]]; then
    source "${planner_root}/install/local_setup.bash"
elif "$needs_planner"; then
    echo "Missing native planner overlay: ${planner_root}/install/local_setup.bash" >&2
    exit 1
fi
"${main_command[@]}"
if "$build_extnav"; then
    source install/local_setup.bash
    "${extnav_command[@]}"
fi
