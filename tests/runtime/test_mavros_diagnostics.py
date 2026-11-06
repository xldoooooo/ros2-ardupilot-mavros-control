"""Verify diagnostic timing and slow-sink isolation without ROS or flight hardware."""

import json
import os
from pathlib import Path
import shutil
import subprocess

import pytest

# Compile the exact deployable recorder, keeping all generated files in pytest's work directory.
ROOT = Path(__file__).resolve().parents[2]
SOURCE = ROOT / "src/onboard_control/deploy/mavros_diagnostics"
FIXTURE = Path(__file__).with_name("mavros_diagnostics_fixture.cpp")


@pytest.fixture(scope="module")
def recorder_binary(tmp_path_factory):
    """Build a standalone executable and an isolated slow-flush preload library."""
    build = tmp_path_factory.mktemp("mavros-recorder-build")
    include = build / "include/mavros"
    include.mkdir(parents=True)
    shutil.copy2(SOURCE / "telemetry_diagnostics.hpp", include)
    binary = build / "recorder"
    subprocess.run(
        ["g++", "-std=c++17", "-O2", "-pthread", "-I", str(include.parent),
         str(FIXTURE), str(SOURCE / "telemetry_diagnostics.cpp"), "-o", str(binary)],
        check=True, capture_output=True, text=True,
    )
    preload = build / "slow_sink.so"
    subprocess.run(
        ["g++", "-std=c++17", "-O2", "-pthread", "-shared", "-fPIC", "-DSLOW_SINK",
         str(FIXTURE), "-ldl", "-o", str(preload)],
        check=True, capture_output=True, text=True,
    )
    return binary, preload


def run_recorder(recorder_binary, tmp_path, mode, slow_sink=False):
    """Run one process and parse its bounded plain-text diagnostic records."""
    binary, preload = recorder_binary
    env = dict(os.environ, ROS_LOG_DIR=str(tmp_path))
    if slow_sink:
        env.update(LD_PRELOAD=str(preload), SLOW_SINK_MARKER=str(tmp_path / "slow-flush"))
    result = subprocess.run([str(binary), mode], env=env, text=True,
                            capture_output=True, timeout=12, check=True)
    lines = next(tmp_path.glob("mavros-telemetry-*.log")).read_text().splitlines()
    records = [dict(item.split("=", 1) for item in line.split()[1:]) | {"kind": line.split()[0]}
               for line in lines if not line.startswith("[MAVROS_DIAGNOSTICS]")]
    return result, records


def test_gap_and_sleep_vs_busy_cpu(recorder_binary, tmp_path):
    """Preserve wire identity, arrival gaps and distinguish blocking from CPU-heavy work."""
    _, records = run_recorder(recorder_binary, tmp_path, "timing")
    events = {row["stage"]: row for row in records if row["kind"] == "[MAVROS_EVENT]"}
    gap = events["position_receive"]
    assert 330 < float(gap["gap_ms"]) < 800
    assert gap["previous_boot_ms"] == "1000" and gap["boot_ms"] == "1350"
    sleeping, busy = events["pose_publish"], events["velocity_publish"]
    assert float(sleeping["duration_ms"]) >= 430
    assert float(busy["duration_ms"]) >= 430
    assert float(sleeping["cpu_ms"]) < 0.2 * float(sleeping["duration_ms"])
    assert float(busy["cpu_ms"]) > 0.5 * float(busy["duration_ms"])


def test_writer_observes_unfinished_publication(recorder_binary, tmp_path):
    """An independent writer can report a stage before its callback returns."""
    _, records = run_recorder(recorder_binary, tmp_path, "inflight")
    inflight = [row for row in records if row["kind"] == "[MAVROS_INFLIGHT]"]
    assert len(inflight) == 1
    assert inflight[0]["stage"] == "pose_publish"
    assert float(inflight[0]["duration_ms"]) >= 900
    assert float(inflight[0]["cpu_ms"]) == -1


def test_slow_sink_does_not_stall_or_queue_probes(recorder_binary, tmp_path):
    """A 2.5-second writer stall cannot propagate to probes or build a log backlog."""
    result, records = run_recorder(recorder_binary, tmp_path, "slow_sink", slow_sink=True)
    metrics = json.loads(result.stdout)
    assert (tmp_path / "slow-flush").exists()
    assert metrics["probes"] == 20000
    assert metrics["probe_ms"] < 1000, metrics
    assert 1 <= len(records) <= 4, records
    print(metrics)
