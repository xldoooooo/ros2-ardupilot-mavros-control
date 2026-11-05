"""Verify deferred telemetry evidence against the unchanged native LAND protection."""
from types import SimpleNamespace

import pytest

from test_avoidance_executor import avoidance_node
from test_waypoint_calibration import calibration_node, execute, point


@pytest.fixture(autouse=True)
def diagnostic_directory(monkeypatch, tmp_path):
    """Keep the diagnostic file in this isolated test's retained artifacts."""
    monkeypatch.setenv("ROS_LOG_DIR", str(tmp_path / "ros"))


@pytest.mark.parametrize("failure", ["velocity_timeout", "pose_nonfinite"])
def test_telemetry_fault_is_frozen_and_logged_once(calibration_node, tmp_path, failure):
    """Separate invalid values from missing samples; retain evidence after telemetry recovers."""
    h = calibration_node
    assert execute(h, [point()]).accepted
    h.until(lambda: len(h.starts) == 1)
    log, = (tmp_path / "ros").glob("onboard-control-diagnostics-*.log")
    h.spin(1.1)
    assert "[TELEMETRY_FAULT]" not in log.read_text()
    original_velocity = h.pubs["local_position/velocity_local"]
    if failure == "velocity_timeout":
        h.pubs["local_position/velocity_local"] = SimpleNamespace(publish=lambda _: None)
    else:
        h.position_x = float("nan")
    h.until(lambda: h.land_calls == ["LAND"])
    # Recovery must not overwrite the fault snapshot or resume the interrupted waypoint.
    h.pubs["local_position/velocity_local"] = original_velocity
    h.position_x = 0.
    h.until(lambda: "[TELEMETRY_FAULT]" in log.read_text(), 3.)
    h.spin(1.2)
    text = log.read_text()
    lines = [line for line in text.splitlines() if "[TELEMETRY_FAULT]" in line]
    assert len(lines) == 1
    fields = dict(token.split("=", 1) for token in lines[0].split() if "=" in token)
    assert fields["event"] == "telemetry_fault"
    assert fields["velocity_valid"] == "1"
    assert float(fields["pose_timeout_ms"]) == 300.
    if failure == "velocity_timeout":
        assert fields["pose_valid"] == "1"
        assert float(fields["velocity_age_ms"]) > 300.
    else:
        assert fields["pose_valid"] == "0"
    assert "control_period_ms=" in lines[0] and "control_lock_wait_ms=" in lines[0]
    assert "[TELEMETRY_RX]" in text and "phase=before stream=pose" in text
    assert "dds_receive_ns=" in text and "callback_wall_ns=" in text
    assert h.land_calls == ["LAND"]
    assert h.status[-1].failsafe_reason == "本地位置或速度遥测无效/超时"
