#!/usr/bin/env bash
# Load one aircraft profile for both foreground and systemd service entry points.

# Export only the selected source paths; installed parameter copies are never selected.
project_config_load() {
  local config_project_root="$1"
  local selector="${config_project_root}/config/aircraft.local.env"
  local selected_id=simulation
  local key value
  if [[ -r "${selector}" ]]; then
    while IFS='=' read -r key value; do
      if [[ "${key}" == AIRCRAFT_ID ]]; then
        selected_id="${value%%#*}"
        selected_id="${selected_id//\"/}"
        selected_id="${selected_id//\'/}"
        selected_id="${selected_id//[[:space:]]/}"
      fi
    done < "${selector}"
  fi
  [[ "${selected_id}" =~ ^[A-Za-z0-9_-]+$ ]] || {
    echo "[config] Invalid AIRCRAFT_ID in ${selector}" >&2; return 1;
  }
  export AIRCRAFT_ID="${selected_id}"
  export AIRCRAFT_CONFIG_DIR="${config_project_root}/config/aircraft/${selected_id}"
  [[ -d "${AIRCRAFT_CONFIG_DIR}" ]] || {
    echo "[config] Missing aircraft profile: ${AIRCRAFT_CONFIG_DIR}" >&2; return 1;
  }
  runtime_source_environment_file "${AIRCRAFT_CONFIG_DIR}/onboard.env" || return 1
  runtime_source_environment_file "${config_project_root}/config/host.local.env" || return 1
  export ONBOARD_WORKSPACE="${config_project_root}"
  export ONBOARD_ENV_FILE="${AIRCRAFT_CONFIG_DIR}/onboard.env"
  export NAVIGATION_ENV_FILE="${AIRCRAFT_CONFIG_DIR}/navigation.env"
  export ODIN_EXTRINSICS_FILE="${AIRCRAFT_CONFIG_DIR}/odin_extrinsics.yaml"
  export ONBOARD_CONTROL_CONFIG="${AIRCRAFT_CONFIG_DIR}/control.yaml"
  export AVOIDANCE_CONFIG_FILE="${AIRCRAFT_CONFIG_DIR}/avoidance/planner.yaml"
  export MAVROS_CONFIG_FILE="${AIRCRAFT_CONFIG_DIR}/mavros/apm_config.yaml"
  export ODIN_CALIB_DIR="${AIRCRAFT_CONFIG_DIR}/odin/calibration"
  export VIDEO_SERVICE_ONBOARD_CONFIG="${AIRCRAFT_CONFIG_DIR}/video/camera.conf"
  export VIDEO_SERVICE_LENS_CONFIG="${AIRCRAFT_CONFIG_DIR}/video/lens.conf"
  export CORRECTION_ENV_FILE="${AIRCRAFT_CONFIG_DIR}/correction.env"
  export CORRECTION_CONFIG_DIR="${AIRCRAFT_CONFIG_DIR}/correction"
  export PYTHONPATH="${config_project_root}${PYTHONPATH:+:${PYTHONPATH}}"
  printf '[config] aircraft=%s directory=%s\n' "${AIRCRAFT_ID}" "${AIRCRAFT_CONFIG_DIR}" >&2
}
