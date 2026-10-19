#!/usr/bin/env bash
# Shared configuration selection and project-local Odin overlay loading.

# Parse the options shared by standalone Odin and the complete onboard launcher.
odin_parse_options() {
  odin_config_selection=odom
  odin_check=0
  odin_help=0
  while (($#)); do
    case "$1" in
      -c|--config)
        if (($# < 2)) || [[ -z "$2" ]]; then
          runtime_die "${1} requires odom, slam, reloc, original, or an absolute YAML path"
          return 2
        fi
        odin_config_selection="$2"
        shift 2
        ;;
      --check) odin_check=1; shift ;;
      -h|--help) odin_help=1; shift ;;
      *) runtime_die "unknown option: $1 (use --help)"; return 2 ;;
    esac
  done
}

# Resolve a named project configuration or a readable operator-supplied YAML file.
odin_resolve_config() {
  local odin_project_root="$1"
  local selection="$2"
  local config_file
  case "${selection}" in
    odom|slam|reloc|original)
      config_file="${odin_project_root}/drivers/odin/src/odin_ros_driver/config/${selection}.yaml"
      ;;
    /*) config_file="${selection}" ;;
    *) runtime_die "unknown Odin configuration: ${selection}; use a preset or absolute YAML path"; return 2 ;;
  esac
  if [[ ! -f "${config_file}" || ! -r "${config_file}" ]]; then
    runtime_die "Odin configuration is not a readable file: ${config_file}"
    return 1
  fi
  realpath -- "${config_file}"
}

# Reload an overlay at the front even when colcon would skip existing path entries.
odin_source_overlay_first() {
  local setup_file="$1"
  local overlay_root="$2"
  local variable entry retained
  local -a entries=()
  [[ -r "${setup_file}" ]] || { runtime_die "setup file is not readable: ${setup_file}"; return 1; }
  overlay_root="$(realpath -- "${overlay_root}")" || return 1
  for variable in AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH LD_LIBRARY_PATH PATH PYTHONPATH PKG_CONFIG_PATH; do
    [[ -v "${variable}" ]] || continue
    retained=""
    IFS=: read -r -a entries <<<"${!variable}"
    for entry in "${entries[@]}"; do
      case "${entry}" in
        "${overlay_root}"|"${overlay_root}"/*) continue ;;
      esac
      retained+="${retained:+:}${entry}"
    done
    export "${variable}=${retained}"
  done
  runtime_source_setup "${setup_file}"
}

# Explicitly select this project's driver even when an older Odin overlay is visible.
odin_source_project_overlay() {
  local odin_project_root="$1"
  local odin_install="${odin_project_root}/drivers/odin/install"
  local prefix cv_bridge_root
  local cv_bridge_package_setup=0
  if [[ -n "${ODIN_CV_BRIDGE_SETUP:-}" ]]; then
    cv_bridge_root="${ODIN_CV_BRIDGE_SETUP%/share/cv_bridge/local_setup.bash}"
    if [[ "${cv_bridge_root}" == "${ODIN_CV_BRIDGE_SETUP}" ]]; then
      cv_bridge_root="$(dirname -- "${ODIN_CV_BRIDGE_SETUP}")"
    else
      cv_bridge_package_setup=1
    fi
    odin_source_overlay_first "${ODIN_CV_BRIDGE_SETUP}" "${cv_bridge_root}" || return 1
    if ((cv_bridge_package_setup)); then
      # Package-level ament setup omits colcon's shared-library path hook.
      export LD_LIBRARY_PATH="${cv_bridge_root}/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
    fi
  fi
  odin_source_overlay_first "${odin_install}/local_setup.bash" "${odin_install}" || return 1
  prefix="$(runtime_package_prefix odin_ros_driver)" || {
    runtime_die "project Odin package is unavailable; build drivers/odin first"
    return 1
  }
  case "$(realpath -- "${prefix}")" in
    "$(realpath -- "${odin_install}")"|"$(realpath -- "${odin_install}")"/*) ;;
    *) runtime_die "project Odin overlay was shadowed by ${prefix}"; return 1 ;;
  esac
}
