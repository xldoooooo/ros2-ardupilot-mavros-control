#!/usr/bin/env python3
"""Record fixed-FCU rotations and estimate Odin XY or XYZ mounting using the current driver geometry."""

import argparse
import csv
import hashlib
import json
import math
import socket
import sys
import time
from pathlib import Path

import numpy as np
import yaml

# Raw IMU identifies the mounting; the other streams independently expose live output behavior.
TOPICS = {
    'imu': '/odin1/odometry_imu', 'standard': '/odin1/odometry',
    'highfreq': '/odin1/odometry_highfreq', 'vision': '/mavros/vision_pose/pose',
    'status': '/extnav/correction_status',
}
# Pitch remains measured; horizontal yaw fits XY, while tilted motion also constrains Z.
MOUNT_KEYS = ('odin_x', 'odin_y', 'odin_z', 'pitch_cam')
# Five-millimetre robust loss limits isolated tracking glitches without clipping reported errors.
ROBUST_SCALE_M = .005
# Small tilt variations leave height strongly coupled to the unknown world origin.
MIN_TILT_SPAN_DEG = 10.


def mounting(config):
    """Read the driver's authoritative shared parameters, with finite metre/radian values."""
    params = yaml.safe_load(config)['/**']['ros__parameters']
    values = np.array([float(params[k]) for k in MOUNT_KEYS])
    if not np.all(np.isfinite(values)):
        raise ValueError('Nonfinite mounting parameters')
    return values


def rotation_matrices(quaternions):
    """Convert xyzw quaternions to body-to-world matrices without a SciPy dependency."""
    q = np.asarray(quaternions, dtype=float)
    norm = np.linalg.norm(q, axis=1)
    if not np.all(np.isfinite(q)) or np.any(norm < 1e-9):
        raise ValueError('Invalid orientation in recording')
    x, y, z, w = (q / norm[:, None]).T
    return np.stack((1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w),
                     2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w),
                     2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)), axis=1).reshape(-1, 3, 3)


def fcu_rotations(imu_rotations, pitch):
    """Use the same R_OF=R_OI*Ry(-pitch_cam) convention as FcuInstallation."""
    c, s = math.cos(-pitch), math.sin(-pitch)
    return imu_rotations @ np.array(((c, 0., s), (0., 1., 0.), (-s, 0., c)))


def fit_translation(times, positions, rotations, fixed_z=None):
    """Fit XY with supplied Z, or all XYZ, plus nuisance world origin and linear drift."""
    if len(times) < 12 or np.ptp(times) <= 0:
        raise ValueError('Need at least 12 IMU poses covering a rotation')
    dimensions = 3 if fixed_z is None else 2
    label = 'xyz' if dimensions == 3 else 'xy'
    duration = np.ptp(times)
    tau = (times - np.mean(times)) / duration
    identity = np.tile(np.eye(dimensions), (len(times), 1))
    design = np.column_stack((identity, (tau[:, None, None] * np.eye(dimensions)).reshape(-1, dimensions),
                              rotations[:, :dimensions, :dimensions].reshape(-1, dimensions)))
    target = positions[:, :dimensions].copy()
    if fixed_z is not None:
        target -= rotations[:, :dimensions, 2] * fixed_z
    target = target.ravel()
    # Equal total weight per heading prevents long start/end pauses dominating the calibration.
    yaw = np.arctan2(rotations[:, 1, 0], rotations[:, 0, 0])
    bins = np.floor((yaw + np.pi) / (2*np.pi) * 72).astype(int) % 72
    counts = np.bincount(bins, minlength=72)
    angular_weight = 1. / counts[bins]
    robust_weight = np.ones(len(times))
    coefficients = None
    for _ in range(6):
        weight = np.repeat(np.sqrt(angular_weight * robust_weight), dimensions)
        weighted = design * weight[:, None]
        coefficients, _, rank, singular = np.linalg.lstsq(weighted, target * weight, rcond=None)
        if rank < 3 * dimensions or singular[-1] / singular[0] < 1e-8:
            raise ValueError(f'{label.upper()} mounting is unobservable: recording has insufficient rotation')
        residual = (target - design @ coefficients).reshape(-1, dimensions)
        robust_weight = 1. / np.sqrt(1. + np.sum(residual**2, axis=1) / ROBUST_SCALE_M**2)
    return coefficients[2*dimensions:], {
        f'linear_drift_{label}_m_per_s': (coefficients[dimensions:2*dimensions] / duration).tolist(),
        f'fit_residual_{label}_rms_m': float(np.sqrt(np.mean(np.sum(residual**2, axis=1)))),
        'design_condition': float(singular[0] / singular[-1]),
    }


def fit_xy(times, positions, rotations, fixed_z):
    """Keep the original XY-only numerical interface for existing diagnostics."""
    return fit_translation(times, positions, rotations, fixed_z)


def xy_metrics(position):
    """Measure actual XY spread about a constant center; never remove a fitted drift."""
    error = position[:, :2] - np.mean(position[:, :2], axis=0)
    return {'peak_to_peak_xy_m': np.ptp(position[:, :2], axis=0).tolist(),
            'radial_rms_m': float(np.sqrt(np.mean(np.sum(error**2, axis=1)))),
            'radial_p95_m': float(np.percentile(np.linalg.norm(error, axis=1), 95))}


def z_metrics(position):
    """Report actual vertical spread without subtracting drift or fitting a trajectory."""
    z = position[:, 2]
    return {'peak_to_peak_z_m': float(np.ptp(z)),
            'z_rms_m': float(np.sqrt(np.mean((z - np.mean(z))**2)))}


def compact_pose(message):
    """Keep exact source stamps, frames and geometry while avoiding unused covariance traffic."""
    pose = message.pose.pose if hasattr(message.pose, 'pose') else message.pose
    return {'stamp_ns': message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec,
            'frame': message.header.frame_id, 'child': getattr(message, 'child_frame_id', ''),
            'p': [getattr(pose.position, a) for a in 'xyz'],
            'q': [getattr(pose.orientation, a) for a in 'xyzw']}


def record(args):
    """Subscribe only; snapshot selected configuration and preserve every received pose."""
    import rclpy
    from correction_interfaces.msg import ExtnavCorrectionStatus
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import Odometry
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from rosidl_runtime_py.convert import message_to_ordereddict

    config = args.config.read_text()
    mounting(config)
    stream = sys.stdout if args.output == '-' else Path(args.output).open('x')
    rclpy.init()
    node = rclpy.create_node('odin_turntable_calibration_recorder')
    counts = {name: 0 for name in TOPICS}

    def emit(row):
        """Write newline-delimited evidence, retaining an interrupted capture as readable JSONL."""
        stream.write(json.dumps(row, ensure_ascii=False) + '\n')

    def callback(message, name):
        """Record geometry or correction state with separate source and host clocks."""
        row = {'kind': 'status' if name == 'status' else 'pose', 'topic': name,
               'recv_wall_ns': time.time_ns(), 'recv_monotonic': time.monotonic()}
        row.update({'msg': message_to_ordereddict(message)} if name == 'status' else compact_pose(message))
        emit(row)
        counts[name] += 1

    emit({'kind': 'metadata', 'schema': 1, 'hostname': socket.gethostname(),
          'config_path': str(args.config.resolve()), 'config_yaml': config,
          'config_sha256': hashlib.sha256(config.encode()).hexdigest(), 'started_wall_ns': time.time_ns()})
    subscriptions = []
    for name, topic in TOPICS.items():
        is_status = name == 'status'
        qos = QoSProfile(depth=2000, reliability=ReliabilityPolicy.RELIABLE if is_status else ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL if is_status else DurabilityPolicy.VOLATILE)
        message_type = ExtnavCorrectionStatus if is_status else PoseStamped if name == 'vision' else Odometry
        subscriptions.append(node.create_subscription(message_type, topic, lambda msg, key=name: callback(msg, key), qos))
    print('READY: recording; perform the planned rotations about the fixed FCU center.', file=sys.stderr, flush=True)
    started = time.monotonic()
    next_progress = started + 5.
    interrupted = False
    try:
        while time.monotonic() - started < args.seconds:
            rclpy.spin_once(node, timeout_sec=.1)
            if time.monotonic() >= next_progress:
                stream.flush()
                print(f'{time.monotonic()-started:.0f}s {counts}', file=sys.stderr, flush=True)
                next_progress += 5.
    except KeyboardInterrupt:
        interrupted = True
    finally:
        for name, topic in TOPICS.items():
            emit({'kind': 'publishers', 'topic': name, 'nodes': [
                {'name': item.node_name, 'namespace': item.node_namespace, 'gid': list(item.endpoint_gid)}
                for item in node.get_publishers_info_by_topic(topic)]})
        emit({'kind': 'complete', 'interrupted': interrupted, 'counts': counts})
        stream.flush()
        node.destroy_node()
        rclpy.shutdown()
        if stream is not sys.stdout:
            stream.close()


def load_recording(path):
    """Read compact captures and the earlier full-message diagnostic recordings."""
    rows = {name: [] for name in TOPICS}
    metadata, statuses, warnings = {}, [], []
    reverse = {topic: name for name, topic in TOPICS.items()}
    with path.open() as stream:
        for line in stream:
            row = json.loads(line)
            if row['kind'] == 'metadata':
                metadata = row
            elif row['kind'] == 'status' or (row['kind'] == 'sample' and row['topic'] == TOPICS['status']):
                statuses.append(row['msg'])
            elif row['kind'] in ('pose', 'sample'):
                name = reverse.get(row['topic'], row['topic'])
                if name not in rows or name == 'status':
                    continue
                if row['kind'] == 'sample':
                    msg = row['msg']
                    pose = msg['pose'].get('pose', msg['pose'])
                    row = {'stamp_ns': msg['header']['stamp']['sec']*1_000_000_000+msg['header']['stamp']['nanosec'],
                           'frame': msg['header']['frame_id'], 'child': msg.get('child_frame_id', ''),
                           'p': [pose['position'][a] for a in 'xyz'], 'q': [pose['orientation'][a] for a in 'xyzw']}
                if rows[name] and row['stamp_ns'] <= rows[name][-1]['stamp_ns']:
                    if row['stamp_ns'] < rows[name][-1]['stamp_ns']:
                        raise ValueError(f'{name}: source clock reversed; record one Odin session at a time')
                    continue
                rows[name].append(row)
            elif row['kind'] == 'publishers':
                nodes = row.get('nodes', row.get('endpoints', []))
                if len(nodes) != 1:
                    warnings.append(f"{row['topic']}: {len(nodes)} publishers observed")
    sessions = {s['odin_session_id'] for s in statuses if s.get('odin_session_id')}
    if len(sessions) > 1:
        raise ValueError('Recording contains multiple Odin sessions; record a single uninterrupted turn')
    states = {(s.get('revision'), s.get('correction_valid')) for s in statuses}
    if len(states) > 1:
        warnings.append('Tag state changed; vision variation includes that change')
    return rows, metadata, warnings


def pose_arrays(rows):
    """Build pose arrays after validating geometry and preserving exact integer timestamps."""
    stamps = np.array([r['stamp_ns'] for r in rows], dtype=np.int64)
    position = np.array([r['p'] for r in rows], dtype=float)
    if not np.all(np.isfinite(position)):
        raise ValueError('Nonfinite position in recording')
    return stamps, position, rotation_matrices([r['q'] for r in rows])


def analyze(args):
    """Write a candidate and replay metrics; a supplied candidate is evaluated without refitting."""
    rows, metadata, warnings = load_recording(args.input)
    config = metadata.get('config_yaml') or (args.config.read_text() if args.config else None)
    if config is None:
        raise ValueError('Recording has no configuration snapshot; supply --config from its acquisition')
    baseline = mounting(config)
    if not rows['imu']:
        raise ValueError('No raw /odin1/odometry_imu samples; keep the project driver running during capture')
    if any(r['frame'] != 'odom' or r['child'] != 'imu' for r in rows['imu']):
        raise ValueError('Expected raw diagnostic odometry in odom/imu')
    stamps, position, imu_rotation = pose_arrays(rows['imu'])
    times = (stamps - stamps[0]) / 1e9
    rotation = fcu_rotations(imu_rotation, baseline[3])
    yaw = np.unwrap(np.arctan2(rotation[:, 1, 0], rotation[:, 0, 0]))
    yaw_span = float(np.degrees(np.ptp(yaw)))
    roll = np.arctan2(rotation[:, 2, 1], rotation[:, 2, 2])
    pitch = np.arctan2(-rotation[:, 2, 0], np.hypot(rotation[:, 2, 1], rotation[:, 2, 2]))
    tilt_spans = np.degrees([np.ptp(roll), np.ptp(pitch)]).tolist()
    if yaw_span < 330.:
        warnings.append(f'Only {yaw_span:.1f} degrees covered; complete a full turn before applying')
    candidate, details = baseline.copy(), {}
    fit_z = getattr(args, 'fit_z', False)
    if args.candidate:
        candidate = mounting(args.candidate.read_text())
        if not np.isclose(candidate[3], baseline[3], atol=1e-12, rtol=0):
            raise ValueError('Candidate replay keeps installation pitch fixed')
    elif fit_z:
        candidate[:3], details = fit_translation(times, position, rotation)
        if max(tilt_spans) < MIN_TILT_SPAN_DEG:
            warnings.append(f'Less than {MIN_TILT_SPAN_DEG:g} degrees of tilt variation: Z is weakly constrained; collect fixed-FCU tilted motion')
    else:
        candidate[:2], details = fit_xy(times, position, rotation, baseline[2])
    delta = candidate[:3] - baseline[:3]
    result = {'input': str(args.input.resolve()), 'mode': 'fixed_candidate_replay' if args.candidate else 'fit',
              'baseline': dict(zip(MOUNT_KEYS, baseline.tolist())),
              'candidate': dict(zip(MOUNT_KEYS, candidate.tolist())), 'delta_xy_m': delta[:2].tolist(),
              'delta_xyz_m': delta.tolist(), 'fit_axes': None if args.candidate else 'xyz' if fit_z else 'xy',
              'yaw_span_deg': yaw_span, 'roll_pitch_span_deg': tilt_spans,
              'duration_s': float(times[-1]), 'fit': details,
              'warnings': warnings, 'streams': {},
              'scope': 'Translation mounting only, fixed pitch; predictions are offline, up to constant origins; drift is NOT removed from metrics'}
    args.output.mkdir(parents=True, exist_ok=True)
    imu_by_stamp = {r['stamp_ns']: r for r in rows['imu']}
    runtime_errors = []
    for row in rows['standard']:
        if row['stamp_ns'] in imu_by_stamp:
            raw = imu_by_stamp[row['stamp_ns']]
            expected_r = fcu_rotations(rotation_matrices([raw['q']]), baseline[3])[0]
            runtime_errors.append(np.linalg.norm(np.array(row['p']) - (np.array(raw['p']) - expected_r @ baseline[:3])))
    result['baseline_runtime_check'] = {'exact_pairs': len(runtime_errors),
        'max_position_error_m': float(max(runtime_errors)) if runtime_errors else None}
    if not runtime_errors or max(runtime_errors) > 1e-5:
        warnings.append('Recorded config not confirmed against live IMU/FCU pairs; output replay assumes it was loaded')
    for name in ('imu', 'highfreq', 'vision'):
        if not rows[name]:
            warnings.append(f'No {name} poses: cannot evaluate this output')
            continue
        source_stamps, before, source_rotation = pose_arrays(rows[name])
        if name == 'imu':
            source_rotation = fcu_rotations(source_rotation, baseline[3])
            before = before - source_rotation @ baseline[:3]
        after = before - source_rotation @ delta
        result['streams'][name] = {'samples': len(before), 'before': {**xy_metrics(before), **z_metrics(before)},
                                   'predicted_after': {**xy_metrics(after), **z_metrics(after)}}
        with (args.output / f'{name}.csv').open('w', newline='') as stream:
            writer = csv.writer(stream)
            writer.writerow(('stamp_s', 'before_x_m', 'before_y_m', 'predicted_x_m', 'predicted_y_m', 'yaw_deg',
                             'before_z_m', 'predicted_z_m'))
            angles = np.degrees(np.arctan2(source_rotation[:, 1, 0], source_rotation[:, 0, 0]))
            writer.writerows((s/1e9, *b[:2], *a[:2], angle, b[2], a[2])
                             for s, b, a, angle in zip(source_stamps, before, after, angles))
    document = yaml.safe_load(config)
    document['/**']['ros__parameters'].update(result['candidate'])
    (args.output / 'candidate.yaml').write_text('# Candidate only; apply mounting to the aircraft profile and restart Odin/extnav for validation.\n' +
                                               yaml.safe_dump(document, sort_keys=False, allow_unicode=True))
    (args.output / 'result.json').write_text(json.dumps(result, ensure_ascii=False, indent=2) + '\n')
    print(json.dumps(result, ensure_ascii=False, indent=2))


def main():
    """Keep capture and offline fitting explicit; neither path edits a live configuration."""
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest='command', required=True)
    capture = commands.add_parser('record', help='Read-only ROS capture on the aircraft')
    capture.add_argument('--config', type=Path, required=True)
    capture.add_argument('--seconds', type=float, default=90.)
    capture.add_argument('--output', required=True, help='New JSONL path, or - for stdout')
    capture.set_defaults(function=record)
    fit = commands.add_parser('fit', help='Fit XY/XYZ or evaluate an existing candidate on another recording')
    fit.add_argument('--input', type=Path, required=True)
    fit.add_argument('--config', type=Path, help='Acquisition config for older recordings without metadata')
    fit.add_argument('--candidate', type=Path, help='Evaluate fixed candidate, without refitting')
    fit.add_argument('--fit-z', action='store_true', help='Fit all XYZ using fixed-FCU motion with pitch/roll variation')
    fit.add_argument('--output', type=Path, required=True)
    fit.set_defaults(function=analyze)
    args = parser.parse_args()
    if args.command == 'record' and (not math.isfinite(args.seconds) or args.seconds <= 0):
        parser.error('--seconds must be positive and finite')
    try:
        args.function(args)
    except (ValueError, KeyError, OSError) as error:
        parser.exit(1, f'Error: {error}\n')


if __name__ == '__main__':
    main()
