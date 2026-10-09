#!/usr/bin/env bash
# Stop only the avoidance launch tree; do not touch MAVROS, Odin or onboard control.
set -Eeuo pipefail
mapfile -t launch_pids < <(pgrep -f '(^|/)ros2 launch avoidance_bridge avoidance.launch.py( |$)' || true)
if ((${#launch_pids[@]}==0)); then
  echo 'No avoidance launch process found.'
  exit 0
fi
kill -INT "${launch_pids[@]}"
echo 'Sent SIGINT to the independent avoidance launcher.'
