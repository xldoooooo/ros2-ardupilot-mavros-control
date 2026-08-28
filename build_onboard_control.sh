#!/usr/bin/env bash
# Rebuild the shared interfaces and onboard controller on either workstation or aircraft.

set -Eeuo pipefail

readonly project_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
readonly workspace_helper="${project_root}/src/onboard_control/deploy/onboard_workspace.sh"

usage() {
  cat <<'EOF'
Usage: ./build_onboard_control.sh [--verify]

Without arguments, rebuild flight packages plus the independent correction
interfaces/service in Release mode.

Options:
  --verify  Check dependencies, rebuild, run unit tests, and run the isolated smoke test.
  -h, --help  Show this help text.

The script auto-detects Jazzy on the ground workstation and Humble on the aircraft.
It does not start, stop, or restart the onboard service and sends no flight command.
EOF
}

[[ -x "${workspace_helper}" ]] || {
  printf '[build-onboard] ERROR: helper is missing or not executable: %s\n' \
    "${workspace_helper}" >&2
  exit 1
}

case "${1:-}" in
  "")
    command=build
    ;;
  --verify)
    command=verify
    ;;
  -h|--help)
    usage
    exit 0
    ;;
  *)
    usage >&2
    exit 2
    ;;
esac

printf '[build-onboard] workspace=%s\n' "${project_root}"
printf '[build-onboard] mode=%s; running services will not be restarted\n' "${command}"
export ONBOARD_WORKSPACE="${project_root}"
exec "${workspace_helper}" "${command}"
