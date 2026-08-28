#!/usr/bin/env bash
# Build and verify the complete ground station on Ubuntu 22.04/Humble or 24.04/Jazzy.

set -Eeuo pipefail

readonly project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly runtime_helpers="${project_root}/start_drone/runtime_common.bash"
[[ -r "${runtime_helpers}" ]] || {
  echo "[ground-setup] runtime discovery helper is missing" >&2
  exit 1
}
# shellcheck disable=SC1090
source "${runtime_helpers}"

ros_setup="$(runtime_detect_ros_setup "${PROJECT_ROS_DISTRO:-}")" || exit 1
readonly ros_setup
runtime_source_setup "${ros_setup}"

for command in colcon cmake g++ python3; do
  command -v "${command}" >/dev/null 2>&1 || {
    echo "[ground-setup] required command is missing: ${command}" >&2
    exit 1
  }
done

# 本机摄像头面板依赖系统媒体工具；MediaMTX 是架构相关文件，不随 Git 分发。
for command in ffmpeg ffprobe v4l2-ctl; do
  command -v "${command}" >/dev/null 2>&1 || {
    echo "[ground-setup] required media command is missing: ${command}" >&2
    echo "[ground-setup] install ffmpeg and v4l-utils, then rerun" >&2
    exit 1
  }
done
if [[ ! -x /usr/local/bin/mediamtx ]]; then
  echo "[ground-setup] required MediaMTX is missing: /usr/local/bin/mediamtx" >&2
  echo "[ground-setup] install the matching architecture from video_service/README.md" >&2
  exit 1
fi
if ! mediamtx_version="$(/usr/local/bin/mediamtx --version 2>/dev/null)"; then
  echo "[ground-setup] MediaMTX cannot run on this host; verify its CPU architecture" >&2
  exit 1
fi
echo "[ground-setup] MediaMTX ${mediamtx_version}"

cd "${project_root}"
./src/onboard_control/deploy/onboard_workspace.sh deps-check

missing_packages=()
for package in robot_state_publisher rviz2 tf2_ros; do
  ros2 pkg prefix "${package}" >/dev/null 2>&1 || missing_packages+=("${package}")
done
if ((${#missing_packages[@]} > 0)); then
  echo "[ground-setup] missing ROS packages: ${missing_packages[*]}" >&2
  echo "[ground-setup] install their ${ROS_DISTRO} packages after review, then rerun" >&2
  exit 1
fi

if [[ ! -x "${project_root}/.venv/bin/python3" ]]; then
  echo "[ground-setup] creating project Python environment: .venv"
  python3 -m venv --system-site-packages "${project_root}/.venv"
fi
"${project_root}/.venv/bin/python3" -m pip install -r requirements-gui.txt

colcon build \
  --packages-select \
    guided_interfaces onboard_control guided_sim correction_interfaces correction_service \
  --cmake-args -DCMAKE_BUILD_TYPE=Release

runtime_source_setup "${project_root}/install/setup.bash"
colcon test \
  --packages-select \
    guided_interfaces onboard_control guided_sim correction_interfaces correction_service \
  --event-handlers console_direct+
colcon test-result --verbose

"${project_root}/.venv/bin/python3" ground_station.py --check-environment
echo "[ground-setup] setup passed; run: bash start_ground_all.sh"
