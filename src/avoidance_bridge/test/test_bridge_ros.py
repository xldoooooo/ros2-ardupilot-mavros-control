"""Isolated live ROS contract regression; no hardware, FCU commands or real planner search."""
import os
import struct
import subprocess
import time
import unittest

import rclpy
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, PointField
from guided_interfaces.msg import AvoidanceRequest, AvoidanceResult
from path_planning.srv import PlanMotion
from path_planning.msg import PolynomialSegment
from correction_interfaces.msg import ExtnavCorrectionStatus
from geometry_msgs.msg import PoseStamped


class BridgeContract(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('bridge_contract_test')
        cls.results = []
        cls.maps = []
        cls.node.create_subscription(AvoidanceResult, '/onboard_control/avoidance_result', cls.results.append, 10)
        cls.node.create_subscription(PointCloud2, '/avoidance/map', cls.maps.append, qos_profile_sensor_data)
        cls.raw = cls.node.create_publisher(Odometry, '/test/raw', qos_profile_sensor_data)
        cls.cloud = cls.node.create_publisher(PointCloud2, '/test/cloud', qos_profile_sensor_data)
        cls.req = cls.node.create_publisher(AvoidanceRequest, '/onboard_control/avoidance_request', 10)
        cls.pose = cls.node.create_publisher(PoseStamped, '/mavros/local_position/pose', qos_profile_sensor_data)
        cls.status = cls.node.create_publisher(ExtnavCorrectionStatus, '/extnav/correction_status', 10)
        cls.extnav = os.environ.get('AVOIDANCE_COORD_MODE') == 'extnav'
        cls.shift = (.06, -.03, .05) if cls.extnav else (0., 0., 0.)
        cls.frame = 'odom' if cls.extnav else 'map'
        cls.lag_remaining = 0
        cls.always_lag = False
        cls.lag_stamp = None
        cls.validation_status = 2
        cls.service_calls = []

        def service(request, response):
            cls.service_calls.append(request.mode)
            response.planner_session = 'mock-planner-session'
            response.result.header.frame_id = cls.frame
            if cls.maps:
                response.result.cloud_stamp = cls.maps[-1].header.stamp
            if cls.always_lag or cls.lag_remaining > 0:
                response.result.cloud_stamp = cls.lag_stamp
                cls.lag_remaining = max(0, cls.lag_remaining - 1)
            response.result.status = 2
            if request.mode == 3:
                response.result.status = cls.validation_status
            if request.mode == 1:
                p = PolynomialSegment()
                p.duration = 1.
                p.x[1] = 1.
                response.result.segments = [p]
            return response

        cls.real = os.environ.get('AVOIDANCE_REAL_PLANNER') == '1'
        if cls.real:
            command = ['ros2', 'launch', 'avoidance_bridge', 'avoidance.launch.py',
                       'coordinate_mode:=identity', 'cloud:=/test/cloud', 'odom:=/test/raw']
        else:
            cls.node.create_service(PlanMotion, '/avoidance/plan_motion', service)
            command = ['ros2', 'run', 'avoidance_bridge', 'avoidance_bridge_node', '--ros-args',
                       '-p', 'coordinate_mode:=' + ('extnav' if cls.extnav else 'identity'), '-p', 'cloud_topic:=/test/cloud',
                       '-p', 'odometry_topic:=/test/raw', '-p', 'input_timeout:=1.0']
        cls.process = subprocess.Popen(command, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE,
                                       start_new_session=True)
        cls.spin(2)

    @classmethod
    def spin(cls, duration):
        until = time.monotonic() + duration
        while time.monotonic() < until:
            rclpy.spin_once(cls.node, timeout_sec=.02)

    @classmethod
    def tearDownClass(cls):
        import signal
        os.killpg(cls.process.pid, signal.SIGINT)
        cls.process.wait(timeout=5)
        cls.node.destroy_node()
        rclpy.shutdown()

    def send_request(self, request_id, end=(1., 0., 0.), mode=2):
        q = AvoidanceRequest()
        q.header.frame_id = 'map'
        q.header.stamp = self.node.get_clock().now().to_msg()
        q.controller_session = 'controller-test'
        q.task_revision = 42
        q.waypoint_index = 3
        q.request_id = request_id
        q.mode = mode
        q.start.x, q.start.y, q.start.z = self.shift
        q.goal.x, q.goal.y, q.goal.z = tuple(end[i] + self.shift[i] for i in range(3))
        self.req.publish(q)
        self.spin(.4 if self.real else .15)
        until = time.monotonic() + .9
        while not any(r.request_id == request_id for r in self.results) and time.monotonic() < until:
            self.spin(.02)
        return next(r for r in reversed(self.results) if r.request_id == request_id)

    def scan(self, points):
        now = self.node.get_clock().now().to_msg()
        odom = Odometry()
        odom.header.frame_id = self.frame
        odom.header.stamp = now
        odom.pose.pose.orientation.w = 1.
        if self.extnav:
            status = ExtnavCorrectionStatus()
            status.header.stamp = now
            status.interface_version = '2.0'
            status.service_available = status.odin_available = True
            status.odin_session_id = status.final_sample_odin_session_id = 'odin-test'
            status.revision = status.final_sample_revision = 7
            status.final_sample_available = True
            status.final_sample_reference_mode = 'local_identity_origin'
            status.lever_arm_x_m, status.lever_arm_y_m, status.lever_arm_z_m = self.shift
            self.status.publish(status)
            pose = PoseStamped()
            pose.header.stamp = now
            pose.pose.orientation.w = 1.
            self.pose.publish(pose)
        self.raw.publish(odom)
        self.spin(.03)
        cloud = PointCloud2()
        cloud.header.frame_id = self.frame
        cloud.header.stamp = now
        cloud.height = 1
        cloud.width = len(points)
        cloud.point_step = 12
        cloud.row_step = len(points) * 12
        cloud.fields = [PointField(name=name, offset=i * 4, datatype=7, count=1)
                        for i, name in enumerate(('x', 'y', 'z'))]
        cloud.data = b''.join(struct.pack('<fff', *p) for p in points)
        self.cloud.publish(cloud)
        self.spin(.12)
        return odom, cloud

    def test_contract_and_observation(self):
        self.assertEqual(self.send_request(1).status, 6)
        self.scan([(4., 0., 0.)])
        self.assertTrue(self.maps)
        r = self.send_request(2)
        self.assertEqual(r.status, 2)
        self.assertEqual((r.controller_session, r.task_revision, r.waypoint_index, r.mode),
                         ('controller-test', 42, 3, 2))
        self.assertTrue(r.bridge_session)
        self.assertTrue(r.planner_session)
        if not self.real:
            self.assertEqual(r.planner_session, 'mock-planner-session')
        if self.real or self.extnav:
            self.scan([(4., 0., 0.)])
            p = self.send_request(20, mode=1)
            self.assertEqual(p.status, 2, p.detail)
            self.assertTrue(p.segments)
            self.assertAlmostEqual(p.segments[0].x[0], self.shift[0])
            self.assertAlmostEqual(p.segments[0].y[0], self.shift[1])
            self.assertAlmostEqual(p.segments[0].z[0], self.shift[2])
        self.assertEqual(self.send_request(3, (1., 1., 0.)).status, 6)
        self.scan([])
        self.assertEqual(self.send_request(4).status, 6)
        if self.extnav:
            self.scan([(4., 0., 0.)])
            far = PoseStamped()
            far.pose.position.x = 10.
            self.pose.publish(far)
            self.spin(.05)
            self.assertEqual(self.send_request(5).status, 6)

    def test_schema_and_clock_rejection(self):
        # Exercise the subscriber path immediately after a good map, while the 2Hz throttle is active.
        import copy
        request_id = 100
        for kind in ('duplicate_x', 'overlap', 'extra_data', 'invalid_nsec', 'negative_sec', 'duplicate', 'rewind'):
            _, good = self.scan([(4., 0., 0.)])
            before = len(self.maps)
            bad = copy.deepcopy(good)
            if kind == 'duplicate_x':
                bad.fields[2].name = 'x'
            elif kind == 'overlap':
                bad.fields[2].offset = 0
            elif kind == 'extra_data':
                bad.data.append(0)
            elif kind == 'invalid_nsec':
                bad.header.stamp.nanosec = 1000000000
            elif kind == 'negative_sec':
                bad.header.stamp.sec = -1
            elif kind == 'rewind':
                bad.header.stamp.sec -= 1
            self.cloud.publish(bad)
            self.spin(.04)
            self.assertEqual(len(self.maps), before, kind)
            self.assertEqual(self.send_request(request_id).status, 6, kind)
            request_id += 1
        raw, _ = self.scan([(4., 0., 0.)])
        self.raw.publish(raw)
        self.spin(.04)
        self.assertEqual(self.send_request(request_id).status, 6, 'duplicate raw')
        raw, _ = self.scan([(4., 0., 0.)])
        raw.header.stamp.sec -= 1
        self.raw.publish(raw)
        self.spin(.04)
        self.assertEqual(self.send_request(request_id + 1).status, 6, 'rewound raw')

    def test_map_service_consumption_race(self):
        if self.real:
            self.skipTest('Deterministic delayed consumption requires the mock planner.')
        cls = type(self)
        self.scan([(4., 0., 0.)])
        old = self.maps[-1].header.stamp
        self.spin(.55)
        self.scan([(4., 0., 0.)])
        self.assertNotEqual(old, self.maps[-1].header.stamp)
        cls.lag_stamp = old
        cls.lag_remaining = 2
        cls.service_calls.clear()
        result = self.send_request(200, mode=1)
        self.assertEqual(result.status, 2, result.detail)
        self.assertTrue(result.segments)
        self.assertEqual(cls.service_calls, [1, 3, 3])
        self.assertEqual(sum(r.request_id == 200 for r in self.results), 1)
        self.assertEqual(result.mode, 1)

        # A candidate successful on the old map must not escape a latest-map geometry rejection.
        cls.lag_remaining = 1
        cls.validation_status = 3
        result = self.send_request(201, mode=1)
        self.assertEqual(result.status, 3)
        self.assertFalse(result.segments)
        cls.validation_status = 2

        # Persistent DDS lag is bounded by the same initial 800ms active deadline, never renewed.
        self.spin(.55)
        self.scan([(4., 0., 0.)])
        cls.always_lag = True
        started = time.monotonic()
        result = self.send_request(202)
        duration = time.monotonic() - started
        cls.always_lag = False
        self.assertEqual(result.status, 7, result.detail)
        self.assertFalse(result.segments)
        self.assertLess(duration, .95)
        self.assertGreater(duration, .7)


if __name__ == '__main__':
    unittest.main()
