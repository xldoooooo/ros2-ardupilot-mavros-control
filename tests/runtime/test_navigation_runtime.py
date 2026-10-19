"""Verify persisted mode selection and map prerequisites without any aircraft processes."""
from __future__ import annotations

import os
from pathlib import Path
import shlex
import subprocess

import pytest
import yaml

# Resolve helpers from source so these tests do not need a ROS installation.
ROOT = Path(__file__).resolve().parents[2]


def run_helpers(command: str, environment: dict[str, str]) -> subprocess.CompletedProcess:
    """Run only the configuration helpers in an isolated shell."""
    setup = "; ".join(f"source {shlex.quote(str(ROOT / path))}" for path in (
        "scripts/lib/runtime_common.bash", "scripts/lib/odin_runtime.bash"))
    return subprocess.run(["bash", "-eu", "-c", f"{setup}; {command}"],
                          env=environment, text=True, capture_output=True, check=False)


@pytest.mark.parametrize("saved,options,expected", [
    (None, "", "odom"), ("map", "", "reloc"), ("odom", "", "odom"),
    ("map", "-c odom", "odom"),
])
def test_default_mode_uses_saved_selection_with_explicit_override(
    tmp_path: Path, saved: str | None, options: str, expected: str,
) -> None:
    """A ground switch survives restart, while operator -c remains authoritative."""
    environment_file = tmp_path / "navigation.env"
    if saved is not None:
        environment_file.write_text(f"NAVIGATION_MODE={saved}\n")
    environment = dict(os.environ, NAVIGATION_ENV_FILE=str(environment_file))
    environment.pop("NAVIGATION_MODE", None)
    result = run_helpers(f'odin_parse_options {options}; odin_default_selection; '
                         'printf "%s" "$odin_config_selection"', environment)
    assert result.returncode == 0, result.stderr
    assert result.stdout == expected


def test_relocation_map_is_checked_before_starting_components(tmp_path: Path) -> None:
    """Reject a missing map, then accept the same configuration once the map exists."""
    map_path = tmp_path / 'map with "quoted" path.bin'
    config = tmp_path / "reloc.yaml"
    config.write_text(yaml.safe_dump({"register_keys": {
        "custom_map_mode": 2, "relocalization_map_abs_path": str(map_path),
    }}))
    environment = dict(os.environ, ONBOARD_PYTHON=str(ROOT / ".venv/bin/python"))
    command = f"odin_validate_map_config {shlex.quote(str(ROOT))} {shlex.quote(str(config))}"
    missing = run_helpers(command, environment)
    assert missing.returncode != 0
    assert "existing nonempty map" in missing.stderr
    map_path.write_bytes(b"test map data")
    existing = run_helpers(command, environment)
    assert existing.returncode == 0, existing.stderr
