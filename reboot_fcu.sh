#!/usr/bin/env bash
# Thin onboard-only entry; all reboot and recovery decisions live in onboard_control.
set -euo pipefail
project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [[ ! -r /etc/ros2-ardupilot/onboard.env ]] ||
   ! pgrep -f "^${project_root}/install/onboard_control/lib/onboard_control/onboard_control_node( |$)" >/dev/null; then
  echo '拒绝执行：仅允许在运行本项目机载控制服务的飞机上使用；地面站请使用 GUI。' >&2
  exit 1
fi
source "${project_root}/start_drone/runtime_common.bash"
runtime_source_setup /etc/ros2-ardupilot/onboard.env
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
runtime_source_setup /opt/ros/jazzy/setup.bash
runtime_source_setup "${project_root}/install/setup.bash"
if [[ ! -x "${project_root}/.venv/bin/python" ]]; then
  echo '缺少项目 .venv/bin/python，请按部署文档创建项目环境。' >&2
  exit 1
fi
exec "${project_root}/.venv/bin/python" "${project_root}/src/onboard_control/scripts/reboot_fcu_client.py"
