#!/usr/bin/env bash
# Portable runtime discovery shared by the ground and onboard launchers.

# Print an error and stop the current launcher without mutating the host.
runtime_die() {
  printf '[runtime-discovery] ERROR: %s\n' "$*" >&2
  return 1
}

# ROS-generated setup files may inspect optional unset variables.
runtime_source_setup() {
  local setup_file="$1"
  if [[ ! -r "${setup_file}" ]]; then
    runtime_die "setup file is not readable: ${setup_file}"
    return 1
  fi
  set +u
  # shellcheck disable=SC1090
  source "${setup_file}"
  set -u
}

# Resolve Humble/Jazzy from an explicit choice, the active shell, or the host OS.
runtime_detect_ros_setup() {
  local requested_distro="${1:-}"
  local ros_root="${ROS_INSTALL_ROOT:-/opt/ros}"
  local os_version=""
  local -a candidates=()
  local -a available=()
  local candidate setup_file

  local os_release_file="${RUNTIME_OS_RELEASE_FILE:-/etc/os-release}"
  if [[ -r "${os_release_file}" ]]; then
    os_version="$(
      sed -n 's/^VERSION_ID="\{0,1\}\([^" ]*\)"\{0,1\}$/\1/p' \
        "${os_release_file}" | head -n 1
    )"
  fi

  [[ -n "${requested_distro}" ]] && candidates+=("${requested_distro}")
  case "${os_version}" in
    22.04) candidates+=(humble) ;;
    24.04) candidates+=(jazzy) ;;
  esac
  [[ -n "${ROS_DISTRO:-}" ]] && candidates+=("${ROS_DISTRO}")
  candidates+=(humble jazzy)

  for candidate in "${candidates[@]}"; do
    setup_file="${ros_root}/${candidate}/setup.bash"
    if [[ -r "${setup_file}" && ! " ${available[*]} " =~ " ${candidate} " ]]; then
      available+=("${candidate}")
    fi
  done
  ((${#available[@]} > 0)) || {
    runtime_die "no supported ROS setup found below ${ros_root} (humble or jazzy)"
    return 1
  }
  printf '%s/%s/setup.bash\n' "${ros_root}" "${available[0]}"
}

# Return the first project-local Python, falling back to the current PATH.
runtime_detect_python() {
  local runtime_project_root="$1"
  local requested_python="${2:-}"
  local candidate
  local -a candidates=()

  [[ -n "${requested_python}" ]] && candidates+=("${requested_python}")
  candidates+=(
    "${runtime_project_root}/.venv/bin/python3"
    "${runtime_project_root}/venv/bin/python3"
  )
  if command -v python3 >/dev/null 2>&1; then
    candidates+=("$(command -v python3)")
  fi

  for candidate in "${candidates[@]}"; do
    if [[ -x "${candidate}" ]]; then
      printf '%s\n' "${candidate}"
      return 0
    fi
  done
  runtime_die "no executable Python 3 found; create ${runtime_project_root}/.venv"
}

# Locate the install prefix that owns a ROS package without sourcing unrelated overlays.
runtime_find_package_setup() {
  local package_name="$1"
  local requested_setup="${2:-}"
  local search_roots="${RUNTIME_OVERLAY_SEARCH_ROOTS:-${HOME:-}:/home:/opt}"
  local marker prefix setup_file search_root
  local -a matches=()
  local -a roots=()

  if ros2 pkg prefix "${package_name}" >/dev/null 2>&1; then
    return 0
  fi
  if [[ -n "${requested_setup}" ]]; then
    [[ -r "${requested_setup}" ]] || {
      runtime_die "configured setup for ${package_name} is unreadable: ${requested_setup}"
      return 1
    }
    printf '%s\n' "${requested_setup}"
    return 0
  fi

  IFS=: read -r -a roots <<<"${search_roots}"
  for search_root in "${roots[@]}"; do
    [[ -n "${search_root}" && -d "${search_root}" ]] || continue
    while IFS= read -r marker; do
      prefix="${marker%/share/ament_index/resource_index/packages/${package_name}}"
      setup_file="${prefix}/setup.bash"
      if [[ -r "${setup_file}" && ! " ${matches[*]} " =~ " ${setup_file} " ]]; then
        matches+=("${setup_file}")
      fi
    done < <(
      find "${search_root}" -maxdepth 8 -type f \
        -path "*/share/ament_index/resource_index/packages/${package_name}" \
        -print 2>/dev/null
    )
  done

  case "${#matches[@]}" in
    0)
      runtime_die \
        "ROS package ${package_name} is not visible and no owning overlay was found"
      return 1
      ;;
    1)
      printf '%s\n' "${matches[0]}"
      ;;
    *)
      printf '[runtime-discovery] ERROR: multiple overlays provide %s:\n' \
        "${package_name}" >&2
      printf '  %s\n' "${matches[@]}" >&2
      runtime_die "remove stale overlays or select one with the documented environment override"
      return 1
      ;;
  esac
}

# Source a package's uniquely discovered overlay only when the package is not already visible.
runtime_ensure_package() {
  local package_name="$1"
  local requested_setup="${2:-}"
  local setup_file

  if ros2 pkg prefix "${package_name}" >/dev/null 2>&1; then
    return 0
  fi
  setup_file="$(
    runtime_find_package_setup "${package_name}" "${requested_setup}"
  )" || return 1
  [[ -n "${setup_file}" ]] && runtime_source_setup "${setup_file}"
  ros2 pkg prefix "${package_name}" >/dev/null 2>&1 || {
    runtime_die "${package_name} is still unavailable after sourcing ${setup_file}"
    return 1
  }
}

# Print a serial candidate only when the current user can open it read/write.
runtime_print_fcu_device() {
  local candidate="$1"
  if [[ ! -r "${candidate}" || ! -w "${candidate}" ]]; then
    runtime_die \
      "FCU device is not readable and writable by the current user: ${candidate}"
    return 1
  fi
  printf '%s\n' "${candidate}"
}

# Resolve a stable FCU serial path; ambiguous hardware is rejected rather than guessed.
runtime_detect_fcu_device() {
  local requested_device="${1:-}"
  local dev_root="${RUNTIME_DEV_ROOT:-/dev}"
  local candidate canonical
  local -a stable_candidates=()
  local -a raw_candidates=()
  local -a canonical_seen=()

  if [[ -n "${requested_device}" ]]; then
    [[ -e "${requested_device}" ]] || {
      runtime_die "configured FCU device does not exist: ${requested_device}"
      return 1
    }
    runtime_print_fcu_device "${requested_device}" || return 1
    return 0
  fi

  for candidate in "${dev_root}"/serial/by-id/*; do
    [[ -e "${candidate}" ]] || continue
    stable_candidates+=("${candidate}")
  done
  if ((${#stable_candidates[@]} == 1)); then
    runtime_print_fcu_device "${stable_candidates[0]}" || return 1
    return 0
  fi
  if ((${#stable_candidates[@]} > 1)); then
    printf '[runtime-discovery] ERROR: multiple stable serial devices found:\n' >&2
    printf '  %s\n' "${stable_candidates[@]}" >&2
    runtime_die "refusing to guess which serial device is the flight controller"
    return 1
  fi

  for candidate in \
    "${dev_root}"/ttyTHS* "${dev_root}"/ttyACM* "${dev_root}"/ttyUSB*; do
    [[ -e "${candidate}" ]] || continue
    canonical="$(readlink -f "${candidate}" 2>/dev/null || printf '%s' "${candidate}")"
    if [[ ! " ${canonical_seen[*]} " =~ " ${canonical} " ]]; then
      canonical_seen+=("${canonical}")
      raw_candidates+=("${candidate}")
    fi
  done
  case "${#raw_candidates[@]}" in
    0)
      runtime_die "no FCU serial device found below ${dev_root}"
      return 1
      ;;
    1)
      runtime_print_fcu_device "${raw_candidates[0]}"
      ;;
    *)
      printf '[runtime-discovery] ERROR: multiple serial candidates found:\n' >&2
      printf '  %s\n' "${raw_candidates[@]}" >&2
      runtime_die "refusing to guess which serial device is the flight controller"
      return 1
      ;;
  esac
}
