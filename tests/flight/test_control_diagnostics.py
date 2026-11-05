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


def test_fusion_metrics_are_deferred_and_rate_limited(calibration_node, tmp_path):
    """Pending fusion gets bounded evidence without changing references or completing the task."""
    from guided_interfaces.srv import FlightCommand

    h = calibration_node
    assert execute(h, [point()]).accepted
    h.until(lambda: bool(h.starts))
    h.external_x = .7
    h.finish_calibration()
    h.spin(2.3)
    log, = (tmp_path / "ros").glob("onboard-control-diagnostics-*.log")
    lines = [line for line in log.read_text().splitlines() if line.startswith("[CALIBRATION_FUSION]")]
    assert 1 <= len(lines) <= 4
    assert all("xy_error_m=0.700000" in line and "yaw_error_rad=0.000000" in line for line in lines)
    assert all("ages_ms=" in line and "post_apply_ms=" in line and "target=" in line for line in lines)
    assert h.status[-1].calibration_active and not h.land_calls
    assert h.status[-1].target_position.x == pytest.approx(0.)
    assert h.command(FlightCommand.Request.COMMAND_HOVER).accepted
    h.spin(1.2)
    assert not h.status[-1].calibration_active
    assert len([line for line in log.read_text().splitlines() if line.startswith("[CALIBRATION_FUSION]")]) == len(lines)
