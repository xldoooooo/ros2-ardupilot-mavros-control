#!/usr/bin/env bash
# Manage the minimal onboard checkout, native ROS build, tests, and an isolated smoke run.

set -Eeuo pipefail

readonly SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
readonly RUNTIME_HELPERS="$(cd -- "${SCRIPT_DIR}/../../.." && pwd -P)/start_drone/runtime_common.bash"
[[ -r "${RUNTIME_HELPERS}" ]] || {
  printf '[onboard-workspace] ERROR: runtime helper is missing: %s\n' \
    "${RUNTIME_HELPERS}" >&2
  exit 1
}
# shellcheck disable=SC1090
source "${RUNTIME_HELPERS}"
readonly GUIDED_SPARSE_PATH="/src/guided_interfaces/"
readonly ONBOARD_SPARSE_PATH="/src/onboard_control/"
readonly VIDEO_SPARSE_PATH="/video_service/"
readonly VIDEO_START_SPARSE_PATH="/start_onboard_video.sh"
readonly VIDEO_STOP_SPARSE_PATH="/stop_onboard_video.sh"
readonly DRONE_START_SPARSE_PATH="/start_drone/"
readonly ONBOARD_CONTROL_START_SPARSE_PATH="/start_onboard_control.sh"
readonly ONBOARD_CONTROL_STOP_SPARSE_PATH="/stop_onboard_control.sh"
readonly ONBOARD_BUILD_SPARSE_PATH="/build_onboard_control.sh"
readonly SMOKE_MAVROS_PREFIX="/_task08_smoke_mavros"
readonly SMOKE_INTERFACE_PREFIX="/_task08_smoke_onboard"
readonly DEFAULT_SMOKE_DOMAIN_ID="231"

log() {
  printf '[onboard-workspace] %s\n' "$*"
}

die() {
  printf '[onboard-workspace] ERROR: %s\n' "$*" >&2
  exit 1
}

usage() {
  cat <<'EOF'
Usage: onboard_workspace.sh COMMAND

Commands:
  show-config  Show the resolved workspace, ROS distribution, and Git revision.
  update       Fast-forward the onboard packages and launchers sparse checkout.
  deps-check   Check ROS packages and toolchain without changing the OS.
  build        Build guided_interfaces and onboard_control in Release mode.
  test         Run package tests and report all colcon test results.
  smoke        Run the node in a localhost-only, nonzero ROS domain without MAVROS.
  verify       Run deps-check, build, test, and the isolated smoke test.

Environment overrides:
  ONBOARD_WORKSPACE         Repository/workspace root. Defaults to this Git checkout.
  ONBOARD_ROS_DISTRO        ROS distribution. Auto-detects humble before jazzy.
  ONBOARD_GIT_BRANCH        Branch used by update. Defaults to main.
  ONBOARD_SMOKE_DOMAIN_ID   Isolated test domain in [1, 232]. Defaults to 231.
EOF
}

resolve_workspace() {
  if [[ -n "${ONBOARD_WORKSPACE:-}" ]]; then
    cd -- "${ONBOARD_WORKSPACE}" 2>/dev/null ||
      die "ONBOARD_WORKSPACE does not exist: ${ONBOARD_WORKSPACE}"
    pwd -P
    return
  fi

  git -C "${SCRIPT_DIR}" rev-parse --show-toplevel 2>/dev/null ||
    die "cannot resolve the Git workspace; set ONBOARD_WORKSPACE"
}

source_setup() {
  local setup_file="$1"
  [[ -f "${setup_file}" ]] || die "setup file not found: ${setup_file}"
  # ROS setup files may read optional variables, so nounset is disabled only while sourcing.
  set +u
  # shellcheck disable=SC1090
  source "${setup_file}"
  set -u
}

source_underlay() {
  source_setup "${ROS_SETUP_RESOLVED}"
}

source_overlay() {
  source_underlay
  source_setup "${WORKSPACE_ROOT}/install/setup.bash"
}

require_command() {
  command -v "$1" >/dev/null 2>&1 || die "required command is missing: $1"
}

validate_workspace_layout() {
  [[ -f "${WORKSPACE_ROOT}/src/guided_interfaces/package.xml" ]] ||
    die "guided_interfaces is missing from ${WORKSPACE_ROOT}/src"
  [[ -f "${WORKSPACE_ROOT}/src/onboard_control/package.xml" ]] ||
    die "onboard_control is missing from ${WORKSPACE_ROOT}/src"
  [[ -x "${WORKSPACE_ROOT}/video_service/deploy/install_onboard_video_service.sh" ]] ||
    die "independent video service installer is missing or not executable"
  [[ -x "${WORKSPACE_ROOT}/src/onboard_control/deploy/install_onboard_service.sh" ]] ||
    die "onboard flight service installer is missing or not executable"
  [[ -x "${WORKSPACE_ROOT}/start_onboard_video.sh" ]] ||
    die "independent video service launcher is missing or not executable"
  [[ -x "${WORKSPACE_ROOT}/stop_onboard_video.sh" ]] ||
    die "independent video service stop helper is missing or not executable"
  [[ -d "${WORKSPACE_ROOT}/start_drone" ]] ||
    die "start_drone is missing from ${WORKSPACE_ROOT}"
  [[ -f "${WORKSPACE_ROOT}/start_onboard_control.sh" ]] ||
    die "start_onboard_control.sh is missing from ${WORKSPACE_ROOT}"
  [[ -x "${WORKSPACE_ROOT}/stop_onboard_control.sh" ]] ||
    die "stop_onboard_control.sh is missing or not executable in ${WORKSPACE_ROOT}"
  [[ -x "${WORKSPACE_ROOT}/build_onboard_control.sh" ]] ||
    die "build_onboard_control.sh is missing or not executable in ${WORKSPACE_ROOT}"
}

show_config() {
  local revision="unavailable"
  revision="$(git -C "${WORKSPACE_ROOT}" rev-parse --short HEAD 2>/dev/null || true)"
  log "workspace=${WORKSPACE_ROOT}"
  log "ros_distro=${ROS_DISTRO_RESOLVED}"
  log "architecture=$(uname -m)"
  log "revision=${revision:-unavailable}"
  log "smoke isolation: domain=${SMOKE_DOMAIN_ID}, localhost-only, mavros=${SMOKE_MAVROS_PREFIX}"
}

update_checkout() {
  require_command git
  git -C "${WORKSPACE_ROOT}" rev-parse --is-inside-work-tree >/dev/null 2>&1 ||
    die "workspace is not a Git checkout: ${WORKSPACE_ROOT}"
  [[ "$(git -C "${WORKSPACE_ROOT}" config --bool core.sparseCheckout || true)" == "true" ]] ||
    die "refusing update: checkout is not sparse; follow ONBOARD_DEPLOYMENT.md"
  [[ -z "$(git -C "${WORKSPACE_ROOT}" status --porcelain)" ]] ||
    die "refusing update: Git worktree has local changes"

  # The checkout was initialized in non-cone mode; older Git treats --no-cone here as a path.
  git -C "${WORKSPACE_ROOT}" sparse-checkout set \
    "${GUIDED_SPARSE_PATH}" "${ONBOARD_SPARSE_PATH}" \
    "${VIDEO_SPARSE_PATH}" \
    "${VIDEO_START_SPARSE_PATH}" "${VIDEO_STOP_SPARSE_PATH}" \
    "${DRONE_START_SPARSE_PATH}" "${ONBOARD_CONTROL_START_SPARSE_PATH}" \
    "${ONBOARD_CONTROL_STOP_SPARSE_PATH}" \
    "${ONBOARD_BUILD_SPARSE_PATH}"
  git -C "${WORKSPACE_ROOT}" pull --ff-only origin "${ONBOARD_GIT_BRANCH:-main}"
  validate_workspace_layout
}

check_dependencies() {
  local package
  local -a ros_packages=(
    ament_cmake
    ament_cmake_gtest
    ament_index_python
    builtin_interfaces
    eigen3_cmake_module
    geographic_msgs
    geometry_msgs
    launch
    launch_ros
    mavros_msgs
    rclcpp
    rosidl_default_generators
    rosidl_default_runtime
    std_msgs
    tf2
    tf2_geometry_msgs
  )
  source_underlay
  require_command colcon
  require_command cmake
  require_command g++
  require_command ros2
  require_command python3
  require_command ffmpeg
  require_command ffprobe
  require_command v4l2-ctl
  [[ -f /usr/include/eigen3/Eigen/Core ]] ||
    die "Eigen3 headers are missing: /usr/include/eigen3/Eigen/Core"
  for package in "${ros_packages[@]}"; do
    ros2 pkg prefix "${package}" >/dev/null 2>&1 ||
      die "required ROS package is missing: ${package}"
  done
  log "dependency check passed (${ROS_DISTRO_RESOLVED}, ${#ros_packages[@]} ROS packages, video tools present)"
}

build_workspace() {
  source_underlay
  require_command colcon
  validate_workspace_layout
  (
    cd -- "${WORKSPACE_ROOT}"
    colcon build \
      --packages-up-to onboard_control \
      --cmake-args -DCMAKE_BUILD_TYPE=Release
  )
}

test_workspace() {
  source_overlay
  require_command colcon
  (
    cd -- "${WORKSPACE_ROOT}"
    colcon test \
      --packages-select guided_interfaces onboard_control \
      --event-handlers console_direct+
    colcon test-result --verbose
  )
}

validate_smoke_domain() {
  [[ "${SMOKE_DOMAIN_ID}" =~ ^[0-9]+$ ]] ||
    die "ONBOARD_SMOKE_DOMAIN_ID must be an integer"
  ((SMOKE_DOMAIN_ID >= 1 && SMOKE_DOMAIN_ID <= 232)) ||
    die "ONBOARD_SMOKE_DOMAIN_ID must be in [1, 232], never domain 0"
}

configure_smoke_discovery() {
  export ROS_DOMAIN_ID="${SMOKE_DOMAIN_ID}"
  unset ROS_STATIC_PEERS
  if [[ "${ROS_DISTRO_RESOLVED}" == "humble" ]]; then
    unset ROS_AUTOMATIC_DISCOVERY_RANGE
    export ROS_LOCALHOST_ONLY=1
  else
    unset ROS_LOCALHOST_ONLY
    export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
  fi
}

smoke_test() {
  local smoke_dir node_log status_log attitude_log onboard_prefix node_executable
  local node_pid="" echo_status
  source_overlay
  require_command timeout
  validate_smoke_domain
  configure_smoke_discovery

  smoke_dir="$(mktemp -d)"
  node_log="${smoke_dir}/node.log"
  status_log="${smoke_dir}/status.yaml"
  attitude_log="${smoke_dir}/attitude.yaml"

  cleanup_smoke() {
    if [[ -n "${node_pid}" ]] && kill -0 "${node_pid}" 2>/dev/null; then
      kill -INT "${node_pid}" 2>/dev/null || true
      for _ in {1..20}; do
        if ! kill -0 "${node_pid}" 2>/dev/null ||
          [[ "$(awk '{print $3}' "/proc/${node_pid}/stat" 2>/dev/null || true)" == "Z" ]]
        then
          break
        fi
        sleep 0.1
      done
      if kill -0 "${node_pid}" 2>/dev/null; then
        kill -TERM "${node_pid}" 2>/dev/null || true
        sleep 0.2
      fi
      if kill -0 "${node_pid}" 2>/dev/null; then
        kill -KILL "${node_pid}" 2>/dev/null || true
      fi
      wait "${node_pid}" 2>/dev/null || true
    fi
    find "${smoke_dir}" -depth -delete 2>/dev/null || true
  }
  trap cleanup_smoke EXIT
  trap 'exit 130' INT
  trap 'exit 143' TERM

  log "starting isolated smoke node; no MAVROS or flight command will be contacted"
  onboard_prefix="$(ros2 pkg prefix onboard_control)"
  node_executable="${onboard_prefix}/lib/onboard_control/onboard_control_node"
  [[ -x "${node_executable}" ]] || die "onboard executable is missing: ${node_executable}"
  "${node_executable}" --ros-args \
    --params-file "${WORKSPACE_ROOT}/src/onboard_control/config/control.yaml" \
    -r __node:=task08_smoke_onboard_control \
    -p mavros_prefix:="${SMOKE_MAVROS_PREFIX}" \
    -p interface_prefix:="${SMOKE_INTERFACE_PREFIX}" \
    >"${node_log}" 2>&1 &
  node_pid=$!

  set +e
  timeout 12s ros2 topic echo --no-daemon --once \
    "${SMOKE_INTERFACE_PREFIX}/status" \
    guided_interfaces/msg/ControlStatus >"${status_log}" 2>&1
  echo_status=$?
  set -e
  [[ ${echo_status} -eq 0 ]] || {
    sed -n '1,160p' "${node_log}" >&2
    sed -n '1,160p' "${status_log}" >&2
    die "isolated status topic was not received"
  }

  grep -Eq "^interface_version: ['\"]?3\\.2['\"]?$" "${status_log}" ||
    die "smoke status did not report interface version 3.2"
  grep -q '^fcu_connected: false$' "${status_log}" ||
    die "smoke node unexpectedly reported an FCU connection"
  grep -q '^armed: false$' "${status_log}" ||
    die "smoke node unexpectedly reported armed=true"

  # A publisher exists by design, but an unarmed isolated node must emit no setpoint messages.
  set +e
  timeout 2s ros2 topic echo --no-daemon --once \
    "${SMOKE_MAVROS_PREFIX}/setpoint_raw/attitude" \
    mavros_msgs/msg/AttitudeTarget >"${attitude_log}" 2>&1
  echo_status=$?
  set -e
  [[ ${echo_status} -eq 124 ]] ||
    die "smoke node emitted an attitude setpoint or the no-output check failed (${echo_status})"

  log "smoke passed: interface=3.2, fcu_connected=false, armed=false, setpoint_messages=0"
  cleanup_smoke
  trap - EXIT INT TERM
}

readonly WORKSPACE_ROOT="$(resolve_workspace)"
ROS_SETUP_RESOLVED="$(runtime_detect_ros_setup "${ONBOARD_ROS_DISTRO:-}")" || exit 1
readonly ROS_SETUP_RESOLVED
readonly ROS_DISTRO_RESOLVED="$(basename "$(dirname "${ROS_SETUP_RESOLVED}")")"
readonly SMOKE_DOMAIN_ID="${ONBOARD_SMOKE_DOMAIN_ID:-${DEFAULT_SMOKE_DOMAIN_ID}}"

case "${1:-}" in
  show-config)
    validate_smoke_domain
    show_config
    ;;
  update)
    update_checkout
    ;;
  deps-check)
    check_dependencies
    ;;
  build)
    build_workspace
    ;;
  test)
    test_workspace
    ;;
  smoke)
    smoke_test
    ;;
  verify)
    check_dependencies
    build_workspace
    test_workspace
    smoke_test
    ;;
  -h|--help|help)
    usage
    ;;
  *)
    usage >&2
    exit 2
    ;;
esac
