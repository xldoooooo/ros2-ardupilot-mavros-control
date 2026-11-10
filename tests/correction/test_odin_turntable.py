"""Verify physical pivot recovery and independent replay with unchanged mounting conventions."""

import argparse
import importlib.util
import json
from pathlib import Path

import numpy as np
import pytest
import yaml

# Load the standalone tool without importing ROS or creating discovery participants.
TOOL = Path(__file__).resolve().parents[2] / 'scripts/diagnostics/calibrate_odin_turntable.py'
SPEC = importlib.util.spec_from_file_location('odin_turntable', TOOL)
CALIBRATE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(CALIBRATE)


def physical_turn(seed, yaw_start=.7):
    """Generate IMU observations from a fixed known FCU center, noise and world drift."""
    rng = np.random.default_rng(seed)
    times = np.linspace(0., 60., 601)
    yaw = yaw_start + 2*np.pi*times/60.
    rotation = np.zeros((len(times), 3, 3))
    rotation[:, 0, 0] = rotation[:, 1, 1] = np.cos(yaw)
    rotation[:, 1, 0] = np.sin(yaw)
    rotation[:, 0, 1] = -np.sin(yaw)
    rotation[:, 2, 2] = 1.
    lever = np.array([.112, -.013, .060])
    drift = np.array([.00015, -.0001, 0.])
    center = np.array([2., -3., .8]) + times[:, None]*drift
    position = center + rotation @ lever + rng.normal(0, .0003, center.shape)
    return times, yaw, rotation, position, lever, drift


def write_recording(path, seed, tag_yaw):
    """Create independent raw/FCU/vision observations with exact common source stamps."""
    times, yaw, rotation, position, lever, drift = physical_turn(seed, yaw_start=seed*.2)
    pitch = -np.deg2rad(25.)
    baseline = np.array([.100, -.029, .060])
    config = {'/**': {'ros__parameters': dict(zip(CALIBRATE.MOUNT_KEYS, [float(v) for v in (*baseline, pitch)]))},
              '/extnav_to_vision_pose': {'ros__parameters': {'vision_rate_hz': 40.}}}
    half_yaw = yaw/2
    # Independently form Hamilton q_yaw*q_pitch for the tilted IMU.
    imu_q = np.column_stack((-np.sin(half_yaw)*np.sin(pitch/2),
                              np.cos(half_yaw)*np.sin(pitch/2),
                              np.sin(half_yaw)*np.cos(pitch/2),
                              np.cos(half_yaw)*np.cos(pitch/2)))
    fcu_q = np.column_stack((np.zeros((len(times), 2)), np.sin(half_yaw), np.cos(half_yaw)))
    vision_q = np.column_stack((np.zeros((len(times), 2)), np.sin((yaw+tag_yaw)/2), np.cos((yaw+tag_yaw)/2)))
    c, s = np.cos(tag_yaw), np.sin(tag_yaw)
    world_rotation = np.array([[c, -s, 0.], [s, c, 0.], [0., 0., 1.]])
    fcu = position - rotation @ baseline
    vision = fcu @ world_rotation.T + np.array([1.5, -.7, .06])
    with path.open('w') as output:
        output.write(json.dumps({'kind': 'metadata', 'config_yaml': yaml.safe_dump(config)})+'\n')
        for i, value in enumerate(times):
            for name, p, q in [('imu', position, imu_q), ('standard', fcu, fcu_q),
                               ('highfreq', fcu, fcu_q), ('vision', vision, vision_q)]:
                output.write(json.dumps({'kind': 'pose', 'topic': name, 'stamp_ns': int((10.+value)*1e9),
                                         'frame': 'odom', 'child': 'imu' if name == 'imu' else 'fcu',
                                         'p': p[i].tolist(), 'q': q[i].tolist()})+'\n')
    return lever, drift


def test_fit_recovers_absolute_lever_with_drift_and_outliers():
    """A complete turn separates a physical lever from origin, slow drift and tracking glitches."""
    times, _, rotation, position, expected, drift = physical_turn(41)
    position[120:124, :2] += .06
    actual, detail = CALIBRATE.fit_xy(times, position, rotation, expected[2])
    np.testing.assert_allclose(actual, expected[:2], atol=.0005)
    np.testing.assert_allclose(detail['linear_drift_xy_m_per_s'], drift[:2], atol=.00002)


@pytest.mark.parametrize('tag_yaw', (0., .4))
def test_candidate_replay_on_independent_recording_preserves_drift(tmp_path, tag_yaw):
    """One candidate fixes both streams on another turn without fitting away their true drift."""
    first, second = tmp_path/'first.jsonl', tmp_path/'second.jsonl'
    lever, _ = write_recording(first, 10, tag_yaw)
    write_recording(second, 31, tag_yaw)
    output = tmp_path/'fit'
    CALIBRATE.analyze(argparse.Namespace(input=first, config=None, candidate=None, output=output))
    fit = json.loads((output/'result.json').read_text())
    np.testing.assert_allclose([fit['candidate']['odin_x'], fit['candidate']['odin_y']], lever[:2], atol=.0002)
    assert fit['candidate']['odin_z'] == .06
    assert fit['candidate']['pitch_cam'] == fit['baseline']['pitch_cam']
    assert fit['baseline_runtime_check']['max_position_error_m'] < 1e-12
    validation = tmp_path/'validation'
    CALIBRATE.analyze(argparse.Namespace(input=second, config=None, candidate=output/'candidate.yaml', output=validation))
    result = json.loads((validation/'result.json').read_text())
    assert result['mode'] == 'fixed_candidate_replay'
    assert result['candidate'] == fit['candidate']
    assert result['fit'] == {}
    for name in ('imu', 'highfreq', 'vision'):
        metrics = result['streams'][name]
        assert metrics['before']['radial_rms_m'] > .018
        # Nonzero world drift survives: reporting only the fit residual would falsely give ~0.4 mm.
        assert .002 < metrics['predicted_after']['radial_rms_m'] < .005
    expected_rotation_independent = result['streams']['highfreq']['predicted_after']['radial_rms_m']
    assert result['streams']['vision']['predicted_after']['radial_rms_m'] == pytest.approx(expected_rotation_independent)


def test_stationary_recording_cannot_determine_lever():
    """A static position is not sufficient evidence to generate a mounting calibration."""
    with pytest.raises(ValueError, match='unobservable'):
        CALIBRATE.fit_xy(np.arange(20.), np.tile([1., 2., 3.], (20, 1)),
                         np.tile(np.eye(3), (20, 1, 1)), .06)
