#!/usr/bin/env python3
"""Exercise native execution boundaries, analytic dynamics and full-curve voxels."""

from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np

# Resolve the independent workspace, never the main flight project's Python.
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "workspace/src/fpga_uav_planner_ros2/fpga_uav_planner_ros2/fpga_solver"))

from dyn_ising_adapter import GridFrame3D, xyz_to_cell3d
from executable_trajectory import executable_segment_limits, executable_segments_clear, sample_executable_segment
from uav_kino_planner import FpgaKinoPathPlanner


class NoBoard:
    """Fail if an offline direct-geometry scenario unexpectedly asks for a board."""

    def solve_runs(self, *args, **kwargs):
        """Assert that the direct path does not issue hardware transactions."""
        raise AssertionError("offline direct case must not contact hardware")


def main() -> None:
    """Check the execution contract independently of ROS and physical hardware."""
    frame = GridFrame3D("map", 0.2, 0.2, -2.0, -2.0, 0.0, 20, 40, 40)
    grid = np.zeros((20, 40, 40), dtype=np.uint8)
    planner = FpgaKinoPathPlanner(NoBoard())
    start, goal = (0.03, -0.07, 1.03), (2.04, 0.11, 1.07)
    measured_v = (0.45, -0.1, 0.0)
    start_cell, goal_cell = xyz_to_cell3d(*start, frame), xyz_to_cell3d(*goal, frame)
    options = dict(exact_start_xyz=start, exact_goal_xyz=goal, start_vel_xyz=measured_v,
                   max_vel=1.0, max_acc=1.5, max_vel_z=0.5, max_acc_z=1.0)
    result = planner.plan_executable(grid, start_cell, goal_cell, frame, **options)
    for caps, expected in (({"max_duration": .001}, "duration"), ({"max_samples": 2}, "sample")):
        try:
            planner.plan_executable(grid, start_cell, goal_cell, frame, **options, **caps)
        except RuntimeError as error:
            assert expected in str(error)
        else:
            raise AssertionError("unbounded execution output was accepted")
    assert result.source == "direct" and result.details["hardware_transactions"] == 0
    assert result.details["interpolation"] == "cubic-hermite"
    assert np.allclose(result.path_xyz[0], start, atol=1.0e-12)
    assert np.allclose(result.path_xyz[-1], goal, atol=1.0e-12)
    assert np.allclose(result.velocities_xyz[0], measured_v, atol=1.0e-12)
    assert np.linalg.norm(result.velocities_xyz[-1]) < 1.0e-12
    previous = None
    sampled_speed, sampled_acc = 0.0, 0.0
    for coefficients, duration in zip(result.polynomial_coefficients_xyz, result.segment_durations_s):
        p, v, a = sample_executable_segment(np.asarray(coefficients), np.linspace(0.0, duration, 10001))
        sampled_speed = max(sampled_speed, float(np.linalg.norm(v, axis=1).max()))
        sampled_acc = max(sampled_acc, float(np.linalg.norm(a, axis=1).max()))
        if previous is not None:
            assert np.allclose(p[0], previous[0], atol=1.0e-12)
            assert np.allclose(v[0], previous[1], atol=1.0e-12)
        previous = p[-1], v[-1]
    assert sampled_speed <= result.max_vel_mps + 1.0e-10 <= 1.0 + 1.0e-7
    assert sampled_acc <= result.max_acc_mps2 + 1.0e-10 <= 1.5 + 1.0e-7

    # Interior velocity maximum is invisible to an endpoint-only speed check.
    peak_curve = np.array([[[0.0, 0.0, 3.0, -2.0], [0.0] * 4, [0.0] * 4]])
    peak = executable_segment_limits(peak_curve, [1.0])
    assert abs(peak["speed"] - 1.5) < 1.0e-12
    assert abs(peak["acceleration"] - 6.0) < 1.0e-12

    # The chord is free while the real cubic bows through an occupied voxel.
    collision_frame = GridFrame3D("offset", 0.2, 0.1, 0.0, 0.0, 0.0, 10, 20, 20)
    collision_grid = np.zeros((10, 20, 20), dtype=np.uint8)
    collision_grid[5, 7, 7] = 1
    curved = np.array([[[0.5, 2.0, 0.0, 0.0], [0.5, 4.0, -4.0, 0.0], [0.55, 0.0, 0.0, 0.0]]])
    chord = curved.copy()
    chord[0, 1, 1:] = 0.0
    assert executable_segments_clear(chord, [1.0], collision_grid, collision_frame)
    assert not executable_segments_clear(curved, [1.0], collision_grid, collision_frame)
    # A root hull overlap alone must not reject a separated actual curve.
    collision_grid[5, 7, 7] = 0
    collision_grid[5, 3, 7] = 1
    assert executable_segments_clear(curved, [1.0], collision_grid, collision_frame)
    collision_grid[5, 3, 7] = 0
    collision_grid[5, 2, 9] = 1
    assert not executable_segments_clear(chord, [1.0], collision_grid, collision_frame)

    # An occupied measured start cannot be hidden by the legacy raw/grace path.
    grid[start_cell] = 1
    try:
        planner.plan_executable(grid, start_cell, goal_cell, frame, raw_grid=np.zeros_like(grid), **options)
    except ValueError as error:
        assert "occupied" in str(error)
    else:
        raise AssertionError("occupied execution start was accepted")
    grid[start_cell] = 0
    try:
        planner.plan_executable(grid, start_cell, goal_cell, frame, **{**options, "start_vel_xyz": (2.0, 0.0, 0.0)})
    except RuntimeError as error:
        assert "start velocity" in str(error)
    else:
        raise AssertionError("initial velocity was silently clipped")

    close_goal = (0.05, -0.06, 1.05)
    close_result = planner.plan_executable(grid, start_cell, start_cell, frame, exact_start_xyz=start,
                                         exact_goal_xyz=close_goal, reaches_goal=False)
    assert not close_result.reaches_goal
    assert np.allclose(close_result.path_xyz[-1], close_goal, atol=1.0e-12)
    assert np.linalg.norm(close_result.velocities_xyz[-1]) < 1.0e-12

    # A reversed measured velocity must brake continuously, rather than being
    # rejected by the legacy direct-line forward-speed shortcut.
    reversed_result = planner.plan_executable(
        grid, start_cell, start_cell, frame, exact_start_xyz=start, exact_goal_xyz=close_goal,
        start_vel_xyz=(-0.35, 0.0, 0.0), max_vel=1.0, max_acc=1.5)
    assert np.allclose(reversed_result.velocities_xyz[0], (-0.35, 0.0, 0.0), atol=1.0e-12)
    assert np.allclose(reversed_result.path_xyz[-1], close_goal, atol=1.0e-12)
    assert np.linalg.norm(reversed_result.velocities_xyz[-1]) < 1.0e-12
    assert reversed_result.max_acc_mps2 <= 1.5 + 1.0e-7

    # Zero-distance with nonzero speed is an actual brake-and-return curve;
    # claiming immediate arrival or replacing the measured speed would be false.
    same = (0.1, -0.1, 1.1)
    same_cell = xyz_to_cell3d(*same, frame)
    returning = planner.plan_executable(
        grid, same_cell, same_cell, frame, exact_start_xyz=same, exact_goal_xyz=same,
        start_vel_xyz=(0.4, 0.0, 0.0), max_vel=1.0, max_acc=1.5)
    assert np.allclose(returning.velocities_xyz[0], (0.4, 0.0, 0.0), atol=1.0e-12)
    assert np.max(np.linalg.norm(np.asarray(returning.path_xyz) - same, axis=1)) > 0.01
    assert np.allclose(returning.path_xyz[-1], same, atol=1.0e-12)
    assert np.linalg.norm(returning.velocities_xyz[-1]) < 1.0e-12
    assert returning.max_acc_mps2 <= 1.5 + 1.0e-7

    # If the necessary braking excursion exits the only free cell, reject it.
    constrained_start = (0.195, -0.1, 1.1)
    constrained_cell = xyz_to_cell3d(*constrained_start, frame)
    constrained_grid = np.ones_like(grid)
    constrained_grid[constrained_cell] = 0
    try:
        planner.plan_executable(
            constrained_grid, constrained_cell, constrained_cell, frame,
            exact_start_xyz=constrained_start, exact_goal_xyz=constrained_start,
            start_vel_xyz=(0.4, 0.0, 0.0), max_vel=1.0, max_acc=1.5)
    except RuntimeError as error:
        assert "collides" in str(error)
    else:
        raise AssertionError("same-cell braking escaped the only free voxel")
    print(json.dumps({"status": "PASS", "source": result.source,
                      "segments": len(result.segment_durations_s),
                      "duration_s": sum(result.segment_durations_s),
                      "analytic_maxima": result.details["execution_maxima"],
                      "sampled_max_speed": sampled_speed, "sampled_max_acceleration": sampled_acc,
                      "planning_elapsed_ms": result.planning_elapsed_ms,
                      "same_cell_reversed_duration_s": sum(reversed_result.segment_durations_s),
                      "same_cell_return_duration_s": sum(returning.segment_durations_s),
                      "same_cell_return_excursion_m": float(np.max(np.linalg.norm(np.asarray(returning.path_xyz) - same, axis=1))),
                      "hardware_tested": False}, indent=2))


if __name__ == "__main__":
    main()
