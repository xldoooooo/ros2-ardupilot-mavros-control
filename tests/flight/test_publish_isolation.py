"""Exercise slow DDS calls, fresh inputs, and LAND ordering in the unchanged flight state machine."""
import os
from pathlib import Path
import re
import subprocess
import time

import pytest

from test_avoidance_executor import avoidance_node
from test_waypoint_calibration import calibration_node, execute, point


@pytest.fixture(scope="session")
def publish_delay_library(tmp_path_factory):
    """Compile the isolated publish hook using the already-sourced ROS installation."""
    output = tmp_path_factory.mktemp("publish-hook") / "delay.so"
    includes = set()
    for prefix in os.environ.get("AMENT_PREFIX_PATH", "").split(os.pathsep):
        base = Path(prefix) / "include"
        if base.is_dir():
            includes.add(str(base))
            includes.update(str(p) for p in base.iterdir() if p.is_dir())
    subprocess.run(["g++", "-std=c++17", "-O2", "-shared", "-fPIC",
                    *["-I" + p for p in sorted(includes)],
                    str(Path(__file__).with_name("publish_delay.cpp")),
                    "-o", str(output), "-ldl", "-pthread"], check=True)
    return output


@pytest.fixture
def output_node(publish_delay_library, tmp_path, monkeypatch, request):
    """Set child-only injection parameters before constructing the existing ROS fixtures."""
    monkeypatch.setenv("LD_PRELOAD", str(publish_delay_library))
    monkeypatch.setenv("ROS_LOG_DIR", str(tmp_path / "ros"))
    monkeypatch.setenv("ONBOARD_TEST_TRIGGER", str(tmp_path / "trigger"))
    monkeypatch.setenv("ONBOARD_TEST_TOPIC", "/avoidance_executor_double_mavros/setpoint_raw/attitude")
    monkeypatch.setenv("ONBOARD_TEST_DELAY_MS", "450")
    # getfixturevalue ensures the child inherits the hook before its launch.
    return request.getfixturevalue("calibration_node")


def log_path(tmp_path):
    """Locate the single native node's independent diagnostic sink."""
    path, = (tmp_path / "ros").glob("onboard-control-diagnostics-*.log")
    return path


def stage_times(text, field):
    """Read the last completed output's documented four-stage timing tuple."""
    line = [line for line in text.splitlines() if line.startswith("[CONTROL_OUTPUT]")][-1]
    return [float(v) for v in re.search(field + r"=\(([^)]+)\)", line)[1].split(",")]


@pytest.mark.parametrize("delay_mode", ["sleep", "busy"])
def test_attitude_delay_keeps_inputs_fresh(publish_delay_library, tmp_path, monkeypatch, request, delay_mode):
    """A stalled real publish must not fabricate telemetry loss; CPU evidence separates wait/work."""
    monkeypatch.setenv("ONBOARD_TEST_DELAY_MODE", delay_mode)
    h = request.getfixturevalue("output_node")
    assert execute(h, [point()]).accepted
    h.until(lambda: bool(h.starts))
    (tmp_path / "trigger").touch()
    h.until(lambda: (tmp_path / "trigger.returned").exists())
    log = log_path(tmp_path)
    h.until(lambda: "[CONTROL_OUTPUT]" in log.read_text(), 3.)
    h.spin(.2)
    text = log.read_text()
    assert not h.land_calls and "[TELEMETRY_FAULT]" not in text
    assert h.status[-1].local_position_valid
    assert stage_times(text, "wall_ms")[2] >= 440.
    cpu = stage_times(text, "cpu_ms")[2]
    if delay_mode == "sleep":
        assert cpu < 80.
    else:
        assert cpu > 100.
    assert "voluntary_switches=" in text and "involuntary_switches=" in text
    timing = [line for line in text.splitlines() if line.startswith("[CONTROL_TIMING]")][-1]
    assert 0. <= float(re.search(r"pose_age_ms=([\d.]+)", timing)[1]) < 100.
    assert 0. <= float(re.search(r"velocity_age_ms=([\d.]+)", timing)[1]) < 100.


def test_land_waits_for_inflight_output_without_blocking_inputs(publish_delay_library, tmp_path, monkeypatch, request):
    """LAND follows the started publish, while the independent input group keeps receiving."""
    from guided_interfaces.srv import FlightCommand

    # Override the child launch environment after output_node's defaults are prepared.
    original = subprocess.Popen

    def launch(args, **kwargs):
        """Set a long but bounded injection for the native controller only."""
        kwargs["env"]["ONBOARD_TEST_DELAY_MS"] = "1400"
        return original(args, **kwargs)

    monkeypatch.setattr(subprocess, "Popen", launch)
    h = request.getfixturevalue("output_node")
    assert execute(h, [point()]).accepted
    h.until(lambda: bool(h.starts))
    (tmp_path / "trigger").touch()
    h.until(lambda: (tmp_path / "trigger.entered").exists())
    command = h.request(FlightCommand)
    command.command, command.ttl_ms = command.COMMAND_LAND, 3000
    future = h.flight.call_async(command)
    h.spin(.15)
    assert not future.done() and not h.land_calls
    log = log_path(tmp_path)
    h.until(lambda: "[PUBLISH_INFLIGHT]" in log.read_text(), 2.)
    line = next(line for line in log.read_text().splitlines() if line.startswith("[PUBLISH_INFLIGHT]"))
    ages = [float(v) for v in re.search(r"input_age_ms=\(([^)]+)\)", line)[1].split(",")]
    assert all(0. <= age < 100. for age in ages)
    h.until(future.done)
    assert future.result().accepted and (tmp_path / "trigger.returned").exists()
    h.until(lambda: h.land_calls == ["LAND"])
    assert int((tmp_path / "trigger.returned").read_text()) <= int((tmp_path / "trigger.land_sent").read_text())
    assert not (tmp_path / "trigger.late_attitude").exists()
    h.spin(.2)
    count = len(h.attitudes)
    h.spin(.3)
    assert len(h.attitudes) == count
    assert not (tmp_path / "trigger.late_attitude").exists()
    assert "[TELEMETRY_FAULT]" not in log.read_text()


def test_status_delay_leaves_control_running(publish_delay_library, tmp_path, monkeypatch, request):
    """The ten-Hz status publisher must hold neither flight lock during DDS work."""
    original = subprocess.Popen

    def launch(args, **kwargs):
        """Select only the fictitious status topic for this child."""
        kwargs["env"]["ONBOARD_TEST_TOPIC"] = "/avoidance_executor_double_onboard/status"
        return original(args, **kwargs)

    monkeypatch.setattr(subprocess, "Popen", launch)
    h = request.getfixturevalue("output_node")
    assert execute(h, [point()]).accepted
    h.until(lambda: bool(h.starts))
    times = []
    from mavros_msgs.msg import AttitudeTarget
    from rclpy.qos import qos_profile_sensor_data
    subscription = h.node.create_subscription(AttitudeTarget, h.mavros + "/setpoint_raw/attitude",
                                               lambda _: times.append(time.monotonic()), qos_profile_sensor_data)
    h.spin(.2)
    (tmp_path / "trigger").touch()
    h.until(lambda: (tmp_path / "trigger.returned").exists())
    log = log_path(tmp_path)
    h.until(lambda: "slow_operation=status_publish" in log.read_text(), 3.)
    assert max(b-a for a, b in zip(times, times[1:])) < .1
    assert not h.land_calls and "[TELEMETRY_FAULT]" not in log.read_text()
    h.node.destroy_subscription(subscription)
