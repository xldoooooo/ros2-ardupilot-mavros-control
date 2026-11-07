"""Known-signal checks for hover statistics, window boundaries and diagnostic directions."""

from argparse import Namespace
import csv
from pathlib import Path
import runpy

import numpy as np
import pytest

# Load the independent CLI without making it part of the control or ground-station package.
ANALYSIS = runpy.run_path(str(Path(__file__).resolve().parents[2] / "scripts/diagnostics/analyze_hover.py"))


def settings(**changes):
    """Use short explicit windows in synthetic tests; retain production quality thresholds."""
    return Namespace(**(dict(settle=0., min_seconds=5., max_gap=.1, target_tolerance=.01,
                             position_tolerance=.1, start=0., end=None,
                             include_waypoint_holds=False) | changes))


def signal_rows(seconds=24):
    """Generate a known 0.5 Hz X oscillation, Y bias and exact unclipped PD+DOB command."""
    rows = []
    for n, t in enumerate(np.arange(0., seconds, .01)):
        row = dict(schema_version=1., monotonic_s=100. + t, dt_s=.01, pose_age_s=.005,
                   velocity_age_s=.005, control_mode=3., calibration_active=0.,
                   tracking_controller=0., output_jerk=0., acceleration_feedforward=0.,
                   wn_xy=2., wn_z=2., zeta_xy=.8, zeta_z=.6, observer_xy=1.5, observer_z=.6,
                   max_acceleration_xy=4., max_acceleration_z=4., max_disturbance=3., max_tilt_rad=.4,
                   gravity=9.8, hover_throttle=.3, thrust=.3, reference_yaw=0.,
                   session_id="1820000000000000001", sequence=str(n + 1),
                   publisher_gid="test-source", header_frame_id="fcu_local_enu")
        for name in ANALYSIS["VECTORS"]:
            row.update({f"{name}_{a}": 0. for a in "xyz"})
        for name in ("attitude", "attitude_setpoint"):
            row.update({f"{name}_{a}": float(a == "w") for a in "xyzw"})
        row["position_x"] = -.15 * np.sin(np.pi * t)
        row["velocity_x"] = -.15 * np.pi * np.cos(np.pi * t)
        row["position_y"] = -.08
        row["disturbance_x"] = .2
        for axis in "xyz":
            u = -4 * row["position_" + axis] - 3.2 * row["velocity_" + axis] - row["disturbance_" + axis]
            row["controller_acceleration_" + axis] = row["mapped_acceleration_" + axis] = u
        rows.append(row)
    return rows


def test_known_bias_rms_frequency_and_limits():
    """Compare against analytic values rather than duplicating the FFT/statistical implementation."""
    result = ANALYSIS["summarize"](signal_rows(), settings())
    x, y = result["axes"]["x"], result["axes"]["y"]
    assert x["rms_m"] == pytest.approx(.15 / np.sqrt(2), abs=.001)
    assert x["dominant_hz"] == pytest.approx(.5, abs=.05)
    assert x["output_modified_fraction"] == 0
    assert y["bias_m"] == pytest.approx(.08)
    assert any("周期摆动" in d for d in result["directions"])
    assert any("稳态偏差" in d for d in result["directions"])
    limited = signal_rows()
    for row in limited:
        row["mapped_acceleration_x"] = 0.
    assert "限幅" in ANALYSIS["summarize"](limited, settings())["directions"][0]


@pytest.mark.parametrize("boundary", ["invalid", "target", "profile", "calibration", "restart", "gap"])
def test_boundaries_never_merge_hover_windows(boundary):
    """Split transitions and corruption, but keep large actual errors in valid windows."""
    rows = signal_rows()
    if boundary == "invalid":
        rows[1200] = None
    elif boundary == "calibration":
        rows[1200]["calibration_active"] = 1.
    else:
        for row in rows[1200:]:
            if boundary == "target":
                row["reference_position_x"] = .2
            elif boundary == "profile":
                row["wn_xy"] = 1.5
            elif boundary == "restart":
                row["session_id"] = "1820000000000000002"
            else:
                row["monotonic_s"] += 1.
    windows, _ = ANALYSIS["hover_windows"](rows, settings())
    assert len(windows) == 2
    assert sum(map(len, windows)) >= len(rows) - 1


def test_csv_nonfinite_and_exact_identifiers(tmp_path):
    """A corrupt sample breaks continuity; large session IDs remain distinguishable."""
    rows = signal_rows()
    rows[1200]["position_x"] = float("nan")
    path = tmp_path / "hover.csv"
    with path.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    loaded, invalid = ANALYSIS["load_rows"](path)
    assert invalid == 1 and loaded[1200] is None
    assert loaded[0]["session_id"] == "1820000000000000001"
    assert len(ANALYSIS["hover_windows"](loaded, settings())[0]) == 2


def test_bad_quality_suppresses_gain_changes():
    """Stale inputs must direct attention to telemetry before numerical gain trials."""
    rows = signal_rows()
    for row in rows:
        row["velocity_age_s"] = .3
    result = ANALYSIS["summarize"](rows, settings())
    assert all("先改善" in d for d in result["directions"] if d.startswith(("X", "Y", "Z")))
