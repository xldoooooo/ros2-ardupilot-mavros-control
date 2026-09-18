"""Verify runtime clock setup without touching a serial port or sending flight commands."""

import os
from pathlib import Path
import subprocess

import pytest

ROOT = Path(__file__).resolve().parents[2]
HELPER = ROOT / "scripts/lib/runtime_common.bash"


@pytest.mark.parametrize(
    "initial,mode,readback,stream,success,writes",
    [
        ("0.0", "MAVLINK", "10.0", "yes", True, 1),
        ("5.0", "MAVLINK", "10.0", "yes", True, 0),
        ("0.0", "MAVLINK", "0.0", "yes", False, 1),
        ("0.0", "MAVLINK", "10.0", "no", False, 1),
        ("0.0", "NONE", "10.0", "yes", False, 0),
        ("invalid", "MAVLINK", "10.0", "yes", False, 0),
    ],
)
def test_timesync_requires_readback_and_actual_boot_clock(
    tmp_path, initial, mode, readback, stream, success, writes
):
    """Restore only zero rates; parameter ACK cannot replace readback or clock evidence."""
    fake = tmp_path / "ros2"
    rate_file = tmp_path / "rate"
    rate_file.write_text(initial)
    calls = tmp_path / "calls"
    fake.write_text('''#!/usr/bin/env bash
set -eu
printf '%s\n' "$*" >> "$CALLS"
case "$1 $2" in
  'param get')
    if [[ "${@: -1}" == timesync_mode ]]; then echo "$MODE"; else cat "$RATE_FILE"; fi ;;
  'param set') printf '%s' "$READBACK" > "$RATE_FILE"; echo 'Set parameter successful' ;;
  'topic echo') [[ "$STREAM" == yes ]] ;;
  *) exit 99 ;;
esac
''')
    fake.chmod(0o755)
    env = dict(os.environ, PATH=f"{tmp_path}:{os.environ['PATH']}", MODE=mode,
               READBACK=readback, STREAM=stream, CALLS=str(calls), RATE_FILE=str(rate_file))
    result = subprocess.run(
        ["bash", "-c", 'source "$1"; runtime_ensure_mavros_timesync', "bash", str(HELPER)],
        env=env, text=True, capture_output=True, timeout=10,
    )
    assert (result.returncode == 0) == success, result.stdout + result.stderr
    recorded = calls.read_text()
    assert recorded.count("param set") == writes
    if success:
        assert "m.remote_timestamp_ns > 0" in recorded
        assert "FCU TIMESYNC verified" in result.stdout
    assert "/cmd/" not in recorded
