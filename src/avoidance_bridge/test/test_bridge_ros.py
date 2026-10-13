"""Exercise native planner streaming through the bridge in an isolated local ROS domain."""
import copy
import os
from pathlib import Path
import signal
import struct
import subprocess
import sys
import time
import unittest

import rclpy
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, PointField
from guided_interfaces.msg import AvoidanceRequest, AvoidanceResult
from path_planning.msg import PlanResult
from correction_interfaces.msg import ExtnavCorrectionStatus
from geometry_msgs.msg import PoseStamped


class BridgeContract(unittest.TestCase):
    """Use the real native planner, with synthetic FCU odometry and extnav status only."""

    @classmethod
    def setUpClass(cls):
        """Launch local planning without MAVROS, hardware, or a flight command publisher."""
        if os.environ.get('ROS_DOMAIN_ID', '0') == '0':
            raise RuntimeError('Run regression in a nonzero isolated ROS domain')
        if os.environ.get('ROS_AUTOMATIC_DISCOVERY_RANGE') != 'LOCALHOST':
            raise RuntimeError('Run regression with LOCALHOST discovery')
        rclpy.init()
        cls.node = rclpy.create_node('bridge_contract_test')
        cls.results, cls.native_results, cls.odometry = [], [], []
        cls.node.create_subscription(AvoidanceResult, '/onboard_control/avoidance_result',
                                     cls.results.append, 10)
        cls.node.create_subscription(PlanResult, '/avoidance/plan_result', cls.native_results.append, 10)
        cls.node.create_subscription(Odometry, '/avoidance/odom', cls.odometry.append,
                                     qos_profile_sensor_data)
        cls.raw = cls.node.create_publisher(Odometry, '/test/raw', qos_profile_sensor_data)
        cls.cloud = cls.node.create_publisher(PointCloud2, '/test/cloud', qos_profile_sensor_data)
        cls.req = cls.node.create_publisher(AvoidanceRequest, '/onboard_control/avoidance_request', 10)
        cls.native = cls.node.create_publisher(PlanResult, '/avoidance/plan_result', 10)
        cls.status = cls.node.create_publisher(ExtnavCorrectionStatus, '/extnav/correction_status', 10)
        cls.extnav = os.environ.get('AVOIDANCE_COORD_MODE') == 'extnav'
        cls.shift = (.06, -.03, .05) if cls.extnav else (0., 0., 0.)
        cls.frame = 'odom' if cls.extnav else 'map'
        cls.tag_corrected = False
        cls.status_enabled = True
        cls.status_revision = 7
        cls.artifacts = Path(__file__).resolve().parents[3] / 'agent/codex/native-avoidance-refine'
        cls.artifacts.mkdir(parents=True, exist_ok=True)
        cls.processes, cls.logs = {}, {}
        cls.start_process('native')
        cls.start_process('bridge')
        until = time.monotonic() + 15.
        while cls.req.get_subscription_count() == 0 and time.monotonic() < until:
            cls.spin(.05)
            if any(process.poll() is not None for process in cls.processes.values()):
                raise RuntimeError('Planner launch exited; inspect regression log')
        if cls.req.get_subscription_count() == 0:
            raise RuntimeError('Avoidance request subscriber was not discovered')
        cls.spin(.2)

    @classmethod
    def spin(cls, duration):
        """Dispatch subscriptions and synthetic input timers for a bounded duration."""
        until = time.monotonic() + duration
        while time.monotonic() < until:
            rclpy.spin_once(cls.node, timeout_sec=.02)

    @classmethod
    def start_process(cls, component):
        """Start native and adapter separately so either lifecycle can be exercised independently."""
        if component == 'native':
            arguments = ['run', 'path_planning', 'path_planning_node', '--ros-args',
                         '-r', '__ns:=/avoidance', '-r', 'cloud:=/test/cloud',
                         '-r', 'odom:=/avoidance/odom', '-r', 'goal:=/avoidance/goal',
                         '-p', 'planning_frame:=' + cls.frame,
                         '-p', 'search.max_vel:=2.0', '-p', 'search.max_acc:=2.0']
        else:
            arguments = ['run', 'avoidance_bridge', 'avoidance_bridge_node', '--ros-args',
                         '-p', 'coordinate_mode:=' + ('extnav' if cls.extnav else 'identity'),
                         '-p', 'planning_frame:=' + cls.frame, '-p', 'odometry_topic:=/test/raw']
        cls.logs[component] = (cls.artifacts / f'{component}-{cls.frame}.log').open('a', encoding='utf-8')
        cls.processes[component] = subprocess.Popen(
            [sys.executable, '/opt/ros/jazzy/bin/ros2', *arguments], stdout=cls.logs[component],
            stderr=subprocess.STDOUT, start_new_session=True)

    @classmethod
    def stop_process(cls, component):
        """Stop only one process group created by this regression."""
        process = cls.processes[component]
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=8.)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=5.)
        cls.logs[component].close()

    @classmethod
    def tearDownClass(cls):
        """Clean up both local processes and ROS subscriptions."""
        for component in cls.processes:
            cls.stop_process(component)
        cls.node.destroy_node()
        rclpy.shutdown()

    @classmethod
    def publish_inputs(cls):
        """Keep real native map inputs fresh; one distant obstacle leaves the goal unobserved."""
        now = cls.node.get_clock().now().to_msg()
        if cls.extnav and cls.status_enabled:
            status = ExtnavCorrectionStatus()
            status.header.stamp = now
            status.interface_version = '2.0'
            status.service_available = status.odin_available = True
            status.odin_session_id = status.final_sample_odin_session_id = 'odin-test'
            status.revision = status.final_sample_revision = cls.status_revision
            status.final_sample_available = True
            status.final_sample_reference_mode = 'local_identity_origin'
            status.lever_arm_x_m, status.lever_arm_y_m, status.lever_arm_z_m = cls.shift
            status.correction_valid = status.final_sample_correction_valid = cls.tag_corrected
            cls.status.publish(status)
        odom = Odometry()
        odom.header.frame_id = 'map'
        odom.header.stamp = now
        odom.child_frame_id = 'base_link'
        odom.pose.pose.orientation.w = 1.
        odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z = cls.shift
        cls.raw.publish(odom)
        cloud = PointCloud2()
        cloud.header.frame_id = cls.frame
        cloud.header.stamp = now
        cloud.height = cloud.width = 1
        cloud.point_step = cloud.row_step = 12
        cloud.fields = [PointField(name=name, offset=i * 4, datatype=PointField.FLOAT32, count=1)
                        for i, name in enumerate(('x', 'y', 'z'))]
        cloud.data = struct.pack('<fff', 4., 0., 0.)
        cls.cloud.publish(cloud)

    def request(self, request_id, mode, goal=(1., .6, 0.)):
        """Publish a complete business target in the FCU map frame."""
        request = AvoidanceRequest()
        request.header.frame_id = 'map'
        request.header.stamp = self.node.get_clock().now().to_msg()
        request.controller_session = 'controller-test'
        request.task_revision = 42
        request.waypoint_index = 3
        request.request_id = request_id
        request.mode = mode
        request.start.x, request.start.y, request.start.z = self.shift
        request.goal.x, request.goal.y, request.goal.z = tuple(goal[i] + self.shift[i] for i in range(3))
        self.req.publish(request)
        return request

    def wait_results(self, request_id, count=1):
        """Wait for a bounded number of replies to this exact business request."""
        until = time.monotonic() + 6.
        matches = []
        while time.monotonic() < until:
            self.spin(.05)
            matches = [r for r in self.results if r.request_id == request_id]
            if len(matches) >= count:
                return matches
        self.fail(f'Request {request_id}: expected {count} results, got {len(matches)}')

    def wait_condition(self, predicate, description):
        """Wait for real DDS lifecycle evidence with a bounded discovery allowance."""
        until = time.monotonic() + 10.
        while not predicate() and time.monotonic() < until:
            self.spin(.05)
        self.assertTrue(predicate(), description)

    def test_native_stream_coordinates_and_cancel(self):
        """Verify repeated native plans, exact forwarding, target replacement and cancellation."""
        self.request(1, AvoidanceRequest.CHECK_LINE)
        unavailable = self.wait_results(1)[-1]
        self.assertEqual(unavailable.status, AvoidanceResult.WAITING_FOR_INPUT)
        self.assertEqual(unavailable.plan_sequence, 0)
        timer = self.node.create_timer(.04, self.publish_inputs)
        until = time.monotonic() + 10.
        while self.cloud.get_subscription_count() == 0 and time.monotonic() < until:
            self.spin(.05)
        self.assertGreater(self.cloud.get_subscription_count(), 0)
        self.spin(.5)
        self.wait_condition(lambda: any(r.request_id == 0 and r.ready and r.planner_session
                                       for r in self.results), 'Native identity heartbeat must make bridge ready')
        self.assertTrue(any(not r.target_id and r.planner_session for r in self.native_results))
        self.assertTrue(self.odometry)
        adapted = self.odometry[-1]
        self.assertEqual((adapted.header.frame_id, adapted.child_frame_id), (self.frame, 'base_link'))
        for value in (adapted.pose.pose.position.x, adapted.pose.pose.position.y, adapted.pose.pose.position.z):
            self.assertAlmostEqual(value, 0.)

        # The native obstacle map has no observed-free-space requirement for this off-ray line.
        self.request(2, AvoidanceRequest.CHECK_LINE)
        line = self.wait_results(2)[-1]
        self.assertEqual(line.status, AvoidanceResult.REACH_END, line.detail)
        self.assertEqual((line.max_velocity_xy, line.max_velocity_z,
                          line.max_acceleration_xy, line.max_acceleration_z), (2., 2., 2., 2.))
        self.assertFalse(line.segments)
        self.request(10, AvoidanceRequest.PLAN)
        stream = self.wait_results(10, 3)
        self.assertEqual(len({r.plan_sequence for r in stream}), len(stream))
        self.assertEqual([r.plan_sequence for r in stream], sorted(r.plan_sequence for r in stream))
        for result in stream:
            self.assertGreater(result.plan_sequence, 0)
            self.assertEqual(result.status, AvoidanceResult.REACH_END, result.detail)
            self.assertEqual((result.controller_session, result.task_revision, result.waypoint_index, result.mode),
                             ('controller-test', 42, 3, AvoidanceRequest.PLAN))
            self.assertTrue(result.bridge_session and result.planner_session and result.segments)
            native = next(r for r in self.native_results if r.target_id == 'controller-test:42:3:10'
                          and r.header.stamp == result.header.stamp)
            self.assertEqual(native.planner_session, result.planner_session)
            for field in ('max_velocity_xy', 'max_velocity_z',
                          'max_acceleration_xy', 'max_acceleration_z'):
                self.assertEqual(getattr(native, field), getattr(result, field))
            self.assertEqual(len(native.segments), len(result.segments))
            for source, forwarded in zip(native.segments, result.segments):
                self.assertEqual(source.duration, forwarded.duration)
                for axis, shift in zip(('x', 'y', 'z'), self.shift):
                    self.assertAlmostEqual(getattr(forwarded, axis)[0], getattr(source, axis)[0] + shift)
                    self.assertEqual(list(getattr(forwarded, axis)[1:]), list(getattr(source, axis)[1:]))

        if self.extnav:
            # A 2 Hz coordinate heartbeat may jitter beyond one period without losing FCU inputs.
            type(self).status_enabled = False
            before_gap = len(self.results)
            self.spin(.8)
            during_gap = [r for r in self.results[before_gap:] if r.request_id == 10]
            self.assertTrue(during_gap)
            self.assertTrue(all(r.ready and r.segments for r in during_gap))
            type(self).status_enabled = True
            self.spin(.1)

        old_native = copy.deepcopy(native)
        self.request(11, AvoidanceRequest.PLAN, (1., -.6, 0.))
        self.wait_results(11, 2)
        old_count = sum(r.request_id == 10 for r in self.results)
        old_native.sequence += 10000
        self.native.publish(old_native)
        self.spin(.4)
        self.assertEqual(sum(r.request_id == 10 for r in self.results), old_count)
        self.assertFalse(any(r.request_id == 11 and r.header.stamp == old_native.header.stamp
                             for r in self.results))
        self.wait_results(11, 3)
        if self.extnav:
            # Reusing a business ID after a coordinate change must not relabel an old native result.
            old_coordinates = copy.deepcopy(next(r for r in reversed(self.native_results)
                                                if r.target_id == 'controller-test:42:3:11'))
            old_revision = self.wait_results(11)[-1].coordinate_revision
            type(self).status_revision += 1
            self.spin(.2)
            previous = len(self.wait_results(11))
            self.request(11, AvoidanceRequest.PLAN, (1., -.6, 0.))
            updated = self.wait_results(11, previous + 2)[-1]
            self.assertGreater(updated.coordinate_revision, old_revision)
            old_coordinates.sequence += 10000
            self.native.publish(old_coordinates)
            self.spin(.3)
            self.assertFalse(any(r.request_id == 11 and r.coordinate_revision == updated.coordinate_revision
                                 and r.header.stamp == old_coordinates.header.stamp for r in self.results))
        self.request(11, AvoidanceRequest.CANCEL)
        self.spin(.2)
        count = sum(r.request_id == 11 for r in self.results)
        native_count = sum(r.target_id == 'controller-test:42:3:11' for r in self.native_results)
        self.spin(.6)
        self.assertEqual(sum(r.request_id == 11 for r in self.results), count)
        self.assertEqual(sum(r.target_id == 'controller-test:42:3:11' for r in self.native_results), native_count)

        self.request(20, AvoidanceRequest.PLAN)
        initial = self.wait_results(20, 2)
        old_session = initial[-1].planner_session
        type(self).stop_process('native')
        type(self).start_process('native')
        self.wait_condition(lambda: any(r.request_id == 0 and r.ready and r.planner_session
                                       and r.planner_session != old_session for r in self.results),
                            'Native-only restart must become visible in the bridge identity heartbeat')
        new_session = next(r.planner_session for r in reversed(self.results)
                           if r.request_id == 0 and r.ready and r.planner_session != old_session)
        self.assertTrue(any(not r.target_id and r.planner_session == new_session for r in self.native_results))
        previous = len(self.wait_results(20))
        self.request(20, AvoidanceRequest.PLAN)
        restarted = self.wait_results(20, previous + 2)[previous:]
        self.assertTrue(all(r.planner_session == new_session and r.plan_sequence > 0 for r in restarted))
        self.assertTrue(all(r.status == AvoidanceResult.REACH_END for r in restarted))

        old_bridge = restarted[-1].bridge_session
        type(self).stop_process('bridge')
        type(self).start_process('bridge')
        self.wait_condition(lambda: any(r.request_id == 0 and r.ready and r.bridge_session != old_bridge
                                       for r in self.results), 'Restarted adapter must discover native identity')
        # The new adapter has never received PLAN 20, but complete CANCEL still reaches native.
        self.request(20, AvoidanceRequest.CANCEL)
        self.spin(.3)
        native_count = sum(r.target_id == 'controller-test:42:3:20' for r in self.native_results)
        self.spin(.8)
        self.assertEqual(sum(r.target_id == 'controller-test:42:3:20' for r in self.native_results), native_count)
        self.request(30, AvoidanceRequest.PLAN)
        self.wait_results(30, 2)
        manual = self.node.create_publisher(PoseStamped, '/avoidance/goal', 10)
        self.wait_condition(lambda: manual.get_subscription_count() > 0, 'Manual native goal subscriber')
        target = PoseStamped()
        target.header.frame_id = self.frame
        target.header.stamp = self.node.get_clock().now().to_msg()
        target.pose.position.x, target.pose.position.y = .5, -.5
        target.pose.orientation.w = 1.
        manual.publish(target)
        self.wait_condition(lambda: any(r.request_id == 30 and r.status == AvoidanceResult.NO_PATH
                                       and 'replaced' in r.detail for r in self.results),
                            'A manual replacement must revoke the old business reference')
        self.request(30, AvoidanceRequest.CANCEL)
        self.spin(.2)
        manual_count = sum(r.target_id.startswith('manual:') for r in self.native_results)
        self.spin(.5)
        self.assertGreater(sum(r.target_id.startswith('manual:') for r in self.native_results), manual_count)
        self.assertEqual(self.node.get_publishers_info_by_topic('/mavros/setpoint_raw/attitude'), [])
        for topic, _ in self.node.get_topic_names_and_types():
            if topic.startswith('/mavros/setpoint'):
                self.assertEqual(self.node.get_publishers_info_by_topic(topic), [])
        if self.extnav:
            type(self).tag_corrected = True
            self.spin(.1)
            self.request(12, AvoidanceRequest.CHECK_LINE)
            unavailable = self.wait_results(12)[-1]
            self.assertEqual(unavailable.status, AvoidanceResult.WAITING_FOR_INPUT)
            self.assertEqual(unavailable.plan_sequence, 0)
        timer.cancel()


if __name__ == '__main__':
    unittest.main()
