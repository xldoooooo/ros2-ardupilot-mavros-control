"""Verify deferred telemetry evidence and the configured native LAND protection."""
from types import SimpleNamespace

import pytest

pytestmark = pytest.mark.integration

from tests.support.avoidance_node import avoidance_node
from tests.support.calibration_node import calibration_node, execute, point


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
    assert float(fields["pose_timeout_ms"]) == 500.
    if failure == "velocity_timeout":
        assert fields["pose_valid"] == "1"
        assert float(fields["velocity_age_ms"]) > 500.
    else:
        assert fields["pose_valid"] == "0"
    assert "control_period_ms=" in lines[0] and "control_lock_wait_ms=" in lines[0]
    assert "[TELEMETRY_RX]" in text and "phase=before stream=pose" in text
    assert "dds_receive_ns=" in text and "callback_wall_ns=" in text
    assert h.land_calls == ["LAND"]
    assert h.status[-1].failsafe_reason == "本地位置或速度遥测无效/超时"


@pytest.mark.parametrize("avoidance_node", [{"pose_timeout_seconds": 1.5}], indirect=True)
def test_telemetry_gap_recovers_before_configured_protection(calibration_node, tmp_path):
    """A 350 ms FCU gap recovers with scheduling margin; a sustained outage still lands."""
    h = calibration_node
    assert execute(h, [point()]).accepted
    h.until(lambda: bool(h.starts))
    names = ("local_position/pose", "local_position/velocity_local")
    publishers = {name: h.pubs[name] for name in names}
    for name in names:
        h.pubs[name] = SimpleNamespace(publish=lambda _: None)
    h.spin(.35)
    assert not h.land_calls
    h.pubs.update(publishers)
    h.tick()
    h.spin(.15)
    assert not h.land_calls and h.status[-1].local_position_valid
    # Keep raw odometry and heartbeats alive so only FCU telemetry causes protection.
    for name in names:
        h.pubs[name] = SimpleNamespace(publish=lambda _: None)
    h.until(lambda: h.land_calls == ["LAND"])
    log, = (tmp_path / "ros").glob("onboard-control-diagnostics-*.log")
    h.until(lambda: "[TELEMETRY_FAULT]" in log.read_text(), 3.)
    line, = [line for line in log.read_text().splitlines() if line.startswith("[TELEMETRY_FAULT]")]
    fields = dict(token.split("=", 1) for token in line.split() if "=" in token)
    assert fields["event"] == "telemetry_fault"
    assert float(fields["pose_timeout_ms"]) == 1500.
    assert max(float(fields["pose_age_ms"]), float(fields["velocity_age_ms"])) > 1500.
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


@pytest.mark.parametrize("failure", ["stale", "frame", "xy_nonfinite", "velocity_nonfinite", "quaternion_invalid"])
def test_sampling_raw_fault_keeps_payload_and_late_recovery(calibration_node, tmp_path, monkeypatch, failure):
    """Capture pre-apply faults before cancellation; raw recovery may arrive after the 1 Hz flush."""
    h = calibration_node
    assert execute(h, [point()]).accepted
    h.until(lambda: bool(h.status[-1].calibration_job_id))
    job = h.status[-1].calibration_job_id
    log, = (tmp_path / "ros").glob("onboard-control-diagnostics-*.log")
    h.spin(1.1)
    assert "[TELEMETRY_FAULT]" not in log.read_text()
    raw_pub = next(p for p in h.node.publishers if p.topic_name == "/odin1/odometry_highfreq")
    original_publish = raw_pub.publish

    def faulty_publish(message):
        """Change exactly one original acceptance condition, not timestamps or FCU state."""
        if failure == "stale":
            return
        if failure == "frame":
            message.child_frame_id = "wrong_body"
        elif failure == "xy_nonfinite":
            message.pose.pose.position.x = float("nan")
        elif failure == "velocity_nonfinite":
            message.twist.twist.linear.z = float("nan")
        else:
            message.pose.pose.orientation.w = 0.
        original_publish(message)

    monkeypatch.setattr(raw_pub, "publish", faulty_publish)
    h.until(lambda: h.land_calls == ["LAND"])
    h.until(lambda: "[TELEMETRY_FAULT]" in log.read_text(), 3.)
    text = log.read_text()
    line, = [line for line in text.splitlines() if line.startswith("[TELEMETRY_FAULT]")]
    fields = dict(token.split("=", 1) for token in line.split() if "=" in token)
    assert fields["event"] == "calibration_raw_" + failure
    assert fields["pose_valid"] == fields["velocity_valid"] == "1"
    raw_line, = [line for line in text.splitlines() if line.startswith("[RAW_ODOMETRY]")
                 and f"event_ns={fields['event_ns']} " in line]
    assert f"job={job} " in raw_line and "revision=0 follow_fusion=0" in raw_line
    assert "present=1" in raw_line and "timeout_ms=500.000" in raw_line
    assert "phase=before stream=raw_odin" in text and "dds_source_ns=" in text
    if failure == "stale":
        raw_fields = dict(token.split("=", 1) for token in raw_line.split() if "=" in token)
        assert float(raw_fields["age_ms"]) > 500.
        assert "phase=first_after stream=raw_odin" not in text
    elif failure == "frame":
        assert "child=wrong_body" in raw_line
    elif failure == "quaternion_invalid":
        assert "quaternion_norm_squared=0.000000000" in raw_line
    else:
        assert "nan" in raw_line
    monkeypatch.setattr(raw_pub, "publish", original_publish)
    h.until(lambda: "phase=first_after stream=raw_odin" in log.read_text(), 3.)
    h.spin(1.1)
    assert log.read_text().count("[TELEMETRY_FAULT]") == 1
    assert log.read_text().count("phase=first_after stream=raw_odin") == 1
    assert h.land_calls == ["LAND"] and not h.status[-1].calibration_active


@pytest.mark.parametrize("avoidance_node", [{"calibration_odometry_topic": "/diagnostic_test_missing_raw"}], indirect=True)
def test_initial_raw_wait_logs_only_terminal_failure(calibration_node, tmp_path):
    """Initial missing-data retries stay silent until the existing start timeout expires."""
    h = calibration_node
    assert execute(h, [point()]).accepted
    h.until(lambda: h.status[-1].calibration_active)
    log, = (tmp_path / "ros").glob("onboard-control-diagnostics-*.log")
    h.spin(.5)
    assert "[TELEMETRY_FAULT]" not in log.read_text()
    assert not h.starts and not h.land_calls
    h.until(lambda: h.land_calls == ["LAND"], 5.)
    h.until(lambda: "[TELEMETRY_FAULT]" in log.read_text(), 3.)
    text = log.read_text()
    assert text.count("[TELEMETRY_FAULT]") == 1
    assert "event=calibration_raw_missing" in text and "present=0 age_ms=-1.000" in text
    assert "phase=before stream=raw_odin count=0" in text
