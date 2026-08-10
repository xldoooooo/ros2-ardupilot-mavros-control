#!/usr/bin/env bash
# Start the complete Jazzy ground-station application from the local workspace.

set -Eeuo pipefail

readonly project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly jazzy_setup="/opt/ros/jazzy/setup.bash"
readonly workspace_setup="${project_root}/install/setup.bash"
readonly preferred_python="${GROUND_STATION_PYTHON:-/home/nvidia/venv-ardupilot/bin/python3}"

# ROS-generated setup files may inspect optional environment variables.
source_setup() {
  local setup_file="$1"
  set +u
  # shellcheck disable=SC1090
  source "${setup_file}"
  set -u
}

for required_file in "${jazzy_setup}" "${workspace_setup}"; do
  if [[ ! -r "${required_file}" ]]; then
    echo "[ground-startup] required setup file is missing: ${required_file}" >&2
    exit 1
  fi
done

if [[ ! -x "${preferred_python}" ]]; then
  echo "[ground-startup] Python is not executable: ${preferred_python}" >&2
  exit 1
fi

source_setup "${jazzy_setup}"
source_setup "${workspace_setup}"

# GUI 内部会为本地仿真重建 domain 231/localhost-only context；实机始终
# 回到机载服务既有的 domain 0，避免调用者环境把两套系统重新混在一起。
export ROS_DOMAIN_ID="0"
unset ROS_LOCALHOST_ONLY
export ROS_AUTOMATIC_DISCOVERY_RANGE="${ROS_AUTOMATIC_DISCOVERY_RANGE:-SUBNET}"

echo "[ground-startup] starting ground station on ROS domain ${ROS_DOMAIN_ID}"
exec "${preferred_python}" "${project_root}/ground_station.py" "$@"
