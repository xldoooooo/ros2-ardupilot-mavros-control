"""Exercise real Odin planner parameter precedence on an isolated ROS domain, without scans."""
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import unittest

import rclpy
from rclpy.parameter_client import AsyncParameterClient


class LaunchParameterTest(unittest.TestCase):
    """No sensors or flight services are started; fixtures/logs stay in agent/codex."""

    def test_radius_precedence(self):
        project = Path(__file__).resolve().parents[3]
        planner = Path(os.environ.get('AVOIDANCE_WORKSPACE',
                                     project.parent / 'dyn_small_obs_avoidance-ros2'))
        artifacts = project / 'agent/codex/new-1-path-delay/blind-radius'
        artifacts.mkdir(parents=True, exist_ok=True)
        # Reject accidental hardware-domain execution even though these launches have no flight output.
        self.assertNotEqual(os.environ.get('ROS_DOMAIN_ID', '0'), '0')
        self.assertEqual(os.environ.get('ROS_AUTOMATIC_DISCOVERY_RANGE'), 'LOCALHOST')
        launches = [
            ('odin', planner / 'path_planning/launch/odin.launch.py', '/path_planning'),
        ]
        configured = artifacts / 'radius.yaml'
        configured.write_text('# Parameter precedence fixture, not a hardware configuration.\n'
                              'path_planning:\n  ros__parameters:\n'
                              '    cloud.blind_radius: 0.63\n    planning_rate: 7.0\n',
                              encoding='utf-8')
        missing = artifacts / 'missing-radius.yaml'
        missing.write_text('# Missing radius must retain the C++ node default.\n'
                           'path_planning:\n  ros__parameters:\n    planning_rate: 7.0\n',
                           encoding='utf-8')
        nested = artifacts / 'nested-radius.yaml'
        nested.write_text('# ROS also accepts nested parameter namespaces.\n'
                          'path_planning:\n  ros__parameters:\n'
                          '    cloud:\n      blind_radius: 0.63\n    planning_rate: 7.0\n',
                          encoding='utf-8')
        cases = [
            ('default_yaml', [], 0.0, 10.0),
            ('yaml_radius', [f'config:={configured}'], 0.63, 7.0),
            ('missing_radius', [f'config:={missing}'], 0.0, 7.0),
            ('explicit_radius', [f'config:={configured}', 'blind_radius:=0.25'], 0.25, 7.0),
            ('explicit_zero', [f'config:={configured}', 'blind_radius:=0.0'], 0.0, 7.0),
            ('nested_yaml', [f'config:={nested}'], 0.63, 7.0),
        ]
        rclpy.init()
        results = []
        try:
            for label, launch, node in launches:
                for case, arguments, radius, rate in cases:
                    with self.subTest(launch=label, case=case):
                        reader = rclpy.create_node('launch_parameter_reader')
                        env = dict(os.environ, ROS_LOG_DIR=str(artifacts / 'ros-log'))
                        log = artifacts / f'{label}-{case}.log'
                        with log.open('w', encoding='utf-8') as stream:
                            process = subprocess.Popen(
                                [sys.executable, '/opt/ros/jazzy/bin/ros2', 'launch', str(launch),
                                 *arguments], stdout=stream, stderr=subprocess.STDOUT,
                                env=env, start_new_session=True)
                            try:
                                client = AsyncParameterClient(reader, node)
                                deadline = time.monotonic() + 10.0
                                while not client.wait_for_services(timeout_sec=0.2):
                                    self.assertIsNone(process.poll(), log.read_text(encoding='utf-8'))
                                    self.assertLess(time.monotonic(), deadline,
                                                    log.read_text(encoding='utf-8'))
                                future = client.get_parameters(
                                    ['cloud.blind_radius', 'planning_rate', 'service_only'])
                                rclpy.spin_until_future_complete(reader, future, timeout_sec=5.0)
                                self.assertTrue(future.done(), log.read_text(encoding='utf-8'))
                                values = future.result().values
                                self.assertAlmostEqual(values[0].double_value, radius)
                                self.assertAlmostEqual(values[1].double_value, rate)
                                self.assertFalse(values[2].bool_value)
                                results.append(dict(launch=label, case=case, radius=radius,
                                                    planning_rate=rate, passed=True))
                            finally:
                                if process.poll() is None:
                                    os.killpg(process.pid, signal.SIGINT)
                                    try:
                                        process.wait(timeout=5.0)
                                    except subprocess.TimeoutExpired:
                                        os.killpg(process.pid, signal.SIGKILL)
                                        process.wait(timeout=5.0)
                                reader.destroy_node()
        finally:
            rclpy.shutdown()
            (artifacts / 'results.json').write_text(json.dumps(results, indent=2) + '\n',
                                                  encoding='utf-8')


if __name__ == '__main__':
    unittest.main()
