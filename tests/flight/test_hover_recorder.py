"""Exercise opt-in capture, disconnect/reconnect and both manual CLIs on an isolated node."""

import csv
import json
import os
from pathlib import Path
import subprocess
import sys

import pytest

from tests.support.avoidance_node import avoidance_node


@pytest.mark.integration
def test_manual_recorder_controls_diagnostic_lifetime(avoidance_node, tmp_path):
    """Capture starts only on subscription and sequence stops while ordinary hover continues."""
    from guided_interfaces.msg import TrackingSample
    from guided_interfaces.srv import FlightCommand
    from rclpy.qos import qos_profile_sensor_data

    h = avoidance_node
    assert h.command(FlightCommand.Request.COMMAND_HOVER).accepted
    h.until(lambda: h.status[-1].controller_active)
    h.spin(.6)
    root = Path(__file__).resolve().parents[2]
    output = tmp_path / "hover.csv"
    env = dict(os.environ, ROS_DOMAIN_ID=str(h.node.context.get_domain_id()),
               ROS_LOG_DIR=str(tmp_path / "recorder-ros"))
    command = [sys.executable, str(root / "scripts/diagnostics/record_hover.py"),
               "--prefix", h.mavros.replace("_mavros", "_onboard"),
               "--output", str(output), "--duration", "4"]
    with (tmp_path / "recorder.log").open("w") as log:
        process = subprocess.Popen(command, env=env, stdout=log, stderr=log)
        try:
            h.until(lambda: process.poll() is not None, 12)
        finally:
            if process.poll() is None:
                process.terminate()
            process.wait(timeout=5)
    assert process.returncode == 0, (tmp_path / "recorder.log").read_text()
    with output.open() as stream:
        rows = list(csv.DictReader(stream))
    assert len(rows) > 40
    assert int(rows[0]["sequence"]) <= 20  # No sampling during the preceding ordinary hover.
    assert float(rows[0]["position_error_z"]) == pytest.approx(0.)
    assert float(rows[0]["wn_xy"]) == pytest.approx(2.236)
    assert all(float(row["control_mode"]) == 3 for row in rows)
    metadata = json.loads(output.with_suffix(".csv.json").read_text())
    assert metadata["samples"] == len(rows) and len(metadata["sessions"]) == 1
    h.spin(1.)  # Let discovery observe the last subscription disappearing.
    last_sequence = int(rows[-1]["sequence"])
    h.spin(1.)
    resumed = []
    subscription = h.node.create_subscription(
        TrackingSample, h.mavros.replace("_mavros", "_onboard") + "/tracking_sample",
        resumed.append, qos_profile_sensor_data)
    try:
        h.until(lambda: len(resumed) >= 5)
        assert 0 < resumed[0].sequence - last_sequence < 50
    finally:
        h.node.destroy_subscription(subscription)
    assert h.status[-1].controller_active and h.status[-1].control_mode == 3
    result = subprocess.run([sys.executable, str(root / "scripts/diagnostics/analyze_hover.py"),
                             str(output), "--output", str(tmp_path / "analysis"),
                             "--settle", "0", "--min-seconds", ".5", "--no-plots"],
                            capture_output=True, text=True, timeout=15)
    assert result.returncode == 0, result.stderr
    assert json.loads((tmp_path / "analysis/summary.json").read_text())["analyzed_windows"] >= 1
