"""Verify shared native planner parameters through all three isolated ROS launch entrances."""
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import unittest

import yaml
import rclpy
from rclpy.parameter import parameter_value_to_python
from rclpy.parameter_client import AsyncParameterClient


class LaunchParameterTest(unittest.TestCase):
    """Read actual node parameters; no sensor, FCU or flight-command processes are launched."""

    def read_parameters(self, reader, process, node, names, log):
        """Wait for DDS discovery and fetch each expected native parameter with its ROS type."""
        client = AsyncParameterClient(reader, node)
        deadline = time.monotonic() + 15.
        while not client.wait_for_services(timeout_sec=.2):
            self.assertIsNone(process.poll(), log.read_text(encoding='utf-8'))
            self.assertLess(time.monotonic(), deadline, log.read_text(encoding='utf-8'))
        future = client.get_parameters(names)
        rclpy.spin_until_future_complete(reader, future, timeout_sec=5.)
        self.assertTrue(future.done(), log.read_text(encoding='utf-8'))
        return [parameter_value_to_python(value) for value in future.result().values]

    def test_shared_defaults_and_cli_precedence(self):
        """Check native defaults, YAML, scalar/directional CLI precedence and bad-file fallback."""
        project = Path(__file__).resolve().parents[3]
        planner = Path(os.environ.get('AVOIDANCE_WORKSPACE', project.parent / 'dyn_small_obs_avoidance-ros2'))
        artifacts = project / 'agent/codex/native-avoidance-refine/launch-parameters'
        artifacts.mkdir(parents=True, exist_ok=True)
        self.assertNotEqual(os.environ.get('ROS_DOMAIN_ID', '0'), '0')
        self.assertEqual(os.environ.get('ROS_AUTOMATIC_DISCOVERY_RANGE'), 'LOCALHOST')
        original = yaml.safe_load((planner / 'path_planning/config/planner.yaml').read_text())
        defaults = original['path_planning']['ros__parameters']
        custom = dict(defaults)
        custom.update({
            'search.safe_distance': .37, 'search.max_tau': .7, 'search.horizon': 75.,
            'search.resolution': .15, 'search.voxel_size': .12, 'search.max_vel': 1.7,
            'search.max_acc': 1.6, 'search.max_vel_xy': 1.3, 'search.max_vel_z': .4,
            'search.max_acc_xy': .8, 'search.max_acc_z': .3,
            'search.vertical_distance_weight': 17.0,
            'search.lower': [-42., -41., -3.], 'cloud.blind_radius': .17,
            'planning_frame': 'yaml_frame', 'stamp_clock': 'ros', 'planning_rate': 7.,
        })
        fixture = artifacts / 'custom.yaml'
        fixture.write_text('# Isolated shared parameter fixture.\n' +
                           yaml.safe_dump({'path_planning': {'ros__parameters': custom}}), encoding='utf-8')
        malformed = artifacts / 'malformed.yaml'
        malformed.write_text('path_planning: [invalid\n', encoding='utf-8')
        names = [name for name in defaults if name.startswith('search.')] + [
            'cloud.map_mode', 'cloud.blind_radius', 'planning_frame', 'stamp_clock', 'planning_rate']
        scalar = dict(custom, **{'search.max_vel': 2.1, 'search.max_acc': 1.4,
                                 'search.max_vel_xy': 2.1, 'search.max_vel_z': 2.1,
                                 'search.max_acc_xy': 1.4, 'search.max_acc_z': 1.4})
        specific = dict(scalar, **{'search.max_vel_z': .31, 'search.max_acc_xy': .62,
                                   'search.safe_distance': .41, 'search.lower': [-31., -32., -4.],
                                   'cloud.blind_radius': 0., 'planning_frame': 'cli_frame',
                                   'stamp_clock': 'receive'})
        fallback = dict(defaults)
        fallback['search.vertical_distance_weight'] = 0.0  # Bare core keeps legacy behavior.
        scalar_args = [f'config:={fixture}', 'search.max_vel:=2.1', 'search.max_acc:=1.4']
        cases = [
            ('shared_yaml', [], defaults),
            ('custom_yaml', [f'config:={fixture}'], custom),
            ('scalar_cli', scalar_args, scalar),
            ('direction_cli', [*scalar_args, 'search.max_vel_z:=0.31', 'search.max_acc_xy:=0.62',
                               'search.safe_distance:=0.41', 'search.lower:=[-31,-32,-4]',
                               'blind_radius:=0.0', 'map_mode:=dual_tree',
                               'planning_frame:=cli_frame', 'stamp_clock:=receive'], specific),
            ('unreadable', [f'config:={artifacts / "nonexistent.yaml"}'], fallback),
            ('malformed', [f'config:={malformed}'], fallback),
        ]
        launches = [
            ('odin', planner / 'path_planning/launch/odin.launch.py', '/path_planning'),
            ('generic', planner / 'path_planning/launch/planner.launch.py', '/path_planning'),
            ('bridge', project / 'src/avoidance_bridge/launch/avoidance.launch.py', '/avoidance/path_planning'),
        ]
        rclpy.init()
        results = []
        try:
            for label, launch, node in launches:
                for case, arguments, expected in cases:
                    with self.subTest(launch=label, case=case):
                        reader = rclpy.create_node('launch_parameter_reader')
                        log = artifacts / f'{label}-{case}.log'
                        env = dict(os.environ, ROS_LOG_DIR=str(artifacts / 'ros-log'))
                        with log.open('w', encoding='utf-8') as stream:
                            process = subprocess.Popen(
                                [sys.executable, '/opt/ros/jazzy/bin/ros2', 'launch', str(launch), *arguments],
                                stdout=stream, stderr=subprocess.STDOUT, env=env, start_new_session=True)
                            try:
                                actual = self.read_parameters(reader, process, node, names, log)
                                self.assertEqual(dict(zip(names, actual)), {name: expected[name] for name in names})
                                if label == 'bridge' and case == 'shared_yaml':
                                    bridge = self.read_parameters(reader, process, '/avoidance_bridge',
                                                                  ['coordinate_mode', 'odometry_topic'], log)
                                    self.assertEqual(bridge, ['extnav', '/mavros/local_position/odom'])
                                results.append(dict(launch=label, case=case, passed=True,
                                                    parameters=dict(zip(names, actual))))
                            finally:
                                if process.poll() is None:
                                    os.killpg(process.pid, signal.SIGINT)
                                    try:
                                        process.wait(timeout=8.)
                                    except subprocess.TimeoutExpired:
                                        os.killpg(process.pid, signal.SIGKILL)
                                        process.wait(timeout=5.)
                                reader.destroy_node()
        finally:
            rclpy.shutdown()
            (artifacts / 'results.json').write_text(json.dumps(results, indent=2) + '\n', encoding='utf-8')


if __name__ == '__main__':
    unittest.main()
