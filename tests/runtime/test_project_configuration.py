"""Verify one profile is selected consistently by Python and shell service entry points."""

import os
import subprocess
import sys
from pathlib import Path

import pytest

from project_configuration import config_dir, config_file


# These configuration tests also run on the aircraft's sparse checkout.
PROJECT_ROOT = Path(__file__).resolve().parents[2]


@pytest.mark.parametrize('aircraft', ['simulation', 'uav001', 'uav002'])
def test_shell_and_python_select_same_profile_and_preserve_identity_on_rename(tmp_path, aircraft):
    """Display-name edits and obsolete environment overrides must not change the selected files."""
    profile = tmp_path / 'config/aircraft' / aircraft
    profile.mkdir(parents=True)
    (profile / 'onboard.env').write_text('ONBOARD_WORKSPACE=/obsolete/checkout\n')
    metadata = profile / 'aircraft.yaml'
    metadata.write_text('name: Original aircraft\n')
    if aircraft != 'simulation':
        (tmp_path / 'config/aircraft.local.env').write_text(f'AIRCRAFT_ID="{aircraft}" # Fixed identity\n')
    command = '''set -eu
source "$1/scripts/lib/runtime_common.bash"
source "$1/scripts/lib/project_config.bash"
project_config_load "$2"
printf '%s\\n' "$AIRCRAFT_CONFIG_DIR" "$ONBOARD_CONTROL_CONFIG" "$ONBOARD_WORKSPACE"
'''

    def selected_paths():
        """Exercise the real shell loader without sourcing ROS or touching hardware."""
        result = subprocess.run(
            ['bash', '--noprofile', '--norc', '-c', command, 'bash', str(PROJECT_ROOT), str(tmp_path)],
            check=True, capture_output=True, text=True,
        )
        return result.stdout.splitlines()

    expected = [str(config_dir(tmp_path)), str(config_file('control.yaml', tmp_path)), str(tmp_path)]
    assert selected_paths() == expected
    metadata.write_text('name: Renamed aircraft\n')
    assert selected_paths() == expected
    assert config_dir(tmp_path) == profile


def test_unknown_profile_is_reported_instead_of_loading_another_aircraft(tmp_path):
    """A misspelled host selector must remain visible rather than silently borrowing tuning."""
    (tmp_path / 'config/aircraft/simulation').mkdir(parents=True)
    (tmp_path / 'config/aircraft.local.env').write_text('AIRCRAFT_ID=missing\n')
    with pytest.raises(FileNotFoundError, match='missing'):
        config_dir(tmp_path)


@pytest.mark.parametrize(
    ('domain', 'origin', 'valid'),
    [
        ('42', '31.2,121.5,15', True),
        ('231', '31.2,121.5,15', False),
        ('bad', '31.2,121.5,15', False),
        ('42', 'nan,121.5,15', False),
    ],
)
def test_ground_host_settings_select_domain_and_default_origin(domain, origin, valid):
    """Host overrides must load together and reject a colliding domain or invalid GPS origin."""
    environment = os.environ.copy()
    environment.update(
        GROUND_STATION_HARDWARE_DOMAIN_ID=domain,
        GROUND_STATION_GPS_ORIGIN=origin,
    )
    result = subprocess.run(
        [sys.executable, '-c',
         'from ground_station_core.config import HARDWARE_DOMAIN_ID, DEFAULT_GPS_ORIGIN; '
         'print(HARDWARE_DOMAIN_ID, DEFAULT_GPS_ORIGIN)'],
        cwd=PROJECT_ROOT, env=environment, capture_output=True, text=True,
    )
    if valid:
        assert result.returncode == 0, result.stderr
        assert result.stdout.strip() == '42 (31.2, 121.5, 15.0)'
    else:
        assert result.returncode != 0
        assert 'GROUND_STATION_' in result.stderr
