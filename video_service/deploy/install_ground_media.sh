#!/usr/bin/env bash
# Install the independent media receiver with host-local storage and public addresses.
set -Eeuo pipefail
# Resolve source-owned runtime and service templates without installed file copies.
readonly project_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd -P)"
readonly media_user="${SUDO_USER:-$(id -un)}"
readonly media_home="$(getent passwd "${media_user}" | cut -d: -f6)"
readonly media_python="${project_root}/.venv/bin/python"
[[ "${media_user}" != root && -x "${media_python}" ]] || {
  echo 'Run as the ground user after setup_ground_station.sh creates .venv' >&2
  exit 1
}
command sudo apt-get install -y rsync openssh-server ffmpeg python3-opencv python3-yaml python3-numpy
"${media_python}" -m pip install -r "${project_root}/video_service/requirements-media.txt"
# Only Intel hosts need the VAAPI/OpenCL media sharing runtime.
if lspci -nn | grep -Ei '(VGA|Display).*Intel' >/dev/null; then
  command sudo apt-get install -y intel-opencl-icd intel-media-va-driver
fi
[[ -x /usr/local/bin/mediamtx ]] || { echo 'Install MediaMTX v1.20.0 as documented in video_service/README.md' >&2; exit 1; }
if [[ ! -f "${project_root}/config/ground/media.json" ]]; then
  "${media_python}" - "${project_root}" <<'PY'
"""Seed host settings once; preserve subsequent user edits."""
import json
from pathlib import Path
import sys
sys.path.insert(0, sys.argv[1])
from video_service.camera_app.config import detect_lan_ipv4
root = Path(sys.argv[1])
config = json.loads((root / 'config/ground/media.example.conf').read_text())
config['public_host'] = detect_lan_ipv4()
(root / 'config/ground/media.json').write_text(json.dumps(config, indent=2) + '\n')
PY
fi
unit_stage="$(mktemp)"
trap 'rm -f -- "${unit_stage}"' EXIT
sed -e "s|MEDIA_USER|${media_user}|g" -e "s|MEDIA_HOME|${media_home}|g" \
  -e "s|MEDIA_WORKSPACE|${project_root}|g" "${project_root}/system/systemd/ground-media.service" > "${unit_stage}"
command sudo install -m 0644 "${unit_stage}" /etc/systemd/system/ground-media.service
command sudo systemctl daemon-reload
command sudo systemctl enable --now ground-media.service
