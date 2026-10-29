"""Verify the bridge's single source switch with two native CPU protocol peers, no FPGA."""
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import rclpy
from rclpy.parameter import Parameter
from rclpy.parameter_client import AsyncParameterClient
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Point
from guided_interfaces.msg import AvoidanceRequest, AvoidanceResult, ControlStatus
from nav_msgs.msg import Odometry
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from std_srvs.srv import SetBool


def test_selected_native_source():
    """Check startup CPU truth, airborne refusal, switch identity and landed CPU recovery."""
    if os.environ.get('ROS_DOMAIN_ID', '0') == '0' or os.environ.get('ROS_AUTOMATIC_DISCOVERY_RANGE') != 'LOCALHOST':
        raise RuntimeError('requires isolated nonzero domain and LOCALHOST')
    rclpy.init()
    node = rclpy.create_node('backend_selection_contract')
    processes, streams, results = [], [], []
    root = Path(__file__).resolve().parents[3]
    output = root / 'agent/codex/task10-integration/selection'
    output.mkdir(parents=True, exist_ok=True)
    odom_pub = node.create_publisher(Odometry, '/selection/raw', qos_profile_sensor_data)
    cloud_pub = node.create_publisher(point_cloud2.PointCloud2, '/selection/cloud', qos_profile_sensor_data)
    control_pub = node.create_publisher(ControlStatus, '/onboard_control/status', qos_profile_sensor_data)
    request_pub = node.create_publisher(AvoidanceRequest, '/onboard_control/avoidance_request', 10)
    node.create_subscription(AvoidanceResult, '/onboard_control/avoidance_result', results.append, 20)
    control = ControlStatus(on_ground=True, control_mode=ControlStatus.MODE_IDLE, avoidance_ready=True)

    def publish():
        """Keep world data and authority state fresh without a flight command publisher."""
        stamp = node.get_clock().now().to_msg()
        odom = Odometry(header=Header(frame_id='map', stamp=stamp), child_frame_id='base_link')
        odom.pose.pose.position.z = 1.
        odom.pose.pose.orientation.w = 1.
        odom_pub.publish(odom)
        cloud_pub.publish(point_cloud2.create_cloud_xyz32(odom.header, [(5., 5., 1.)]))
        control_pub.publish(control)

    timer = node.create_timer(.04, publish)

    def wait(predicate, label, timeout=12.):
        """Advance actual DDS while checking a bounded observable condition."""
        deadline = time.monotonic()+timeout
        while not predicate() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.02)
        assert predicate(), label

    def rpc(future):
        """Await one management response without stopping synthetic sensor traffic."""
        wait(future.done, 'management response')
        return future.result()

    def select(namespace):
        """Exercise the installed bridge's real on-set callback."""
        return rpc(client.set_parameters([Parameter('native_namespace', value=namespace)])).results[0]

    def plan(request_id):
        """Send the original business identity to the currently selected result source."""
        request = AvoidanceRequest(header=Header(frame_id='map'), controller_session='selection',
                                   task_revision=1, waypoint_index=1, request_id=request_id,
                                   mode=AvoidanceRequest.PLAN, start=Point(z=1.), goal=Point(x=1., z=1.))
        request_pub.publish(request)
        wait(lambda: any(r.request_id == request_id and r.segments for r in results), 'executable result')
        return next(r for r in reversed(results) if r.request_id == request_id and r.segments)

    try:
        for namespace in ('avoidance', 'fpga_avoidance'):
            stream = (output / (namespace+'.log')).open('w')
            streams.append(stream)
            processes.append(subprocess.Popen([sys.executable, '/opt/ros/jazzy/bin/ros2', 'run',
                'path_planning', 'path_planning_node', '--ros-args', '-r', '__ns:=/'+namespace,
                '-r', 'cloud:=/selection/cloud', '-r', 'odom:=/avoidance/odom',
                '-p', 'planning_frame:=map'], stdout=stream, stderr=subprocess.STDOUT, start_new_session=True))
        stream = (output / 'bridge.log').open('w')
        streams.append(stream)
        processes.append(subprocess.Popen([sys.executable, '/opt/ros/jazzy/bin/ros2', 'run',
            'avoidance_bridge', 'avoidance_bridge_node', '--ros-args', '-p', 'coordinate_mode:=identity',
            '-p', 'planning_frame:=map', '-p', 'odometry_topic:=/selection/raw',
            '-p', 'native_namespace:=/fpga_avoidance'], stdout=stream, stderr=subprocess.STDOUT, start_new_session=True))
        stream = (output / 'manager.log').open('w')
        streams.append(stream)
        processes.append(subprocess.Popen([sys.executable,
            str(root / 'src/avoidance_bridge/scripts/avoidance_backend_manager.py')],
            env=dict(os.environ, FPGA_AVOIDANCE_CONFIG='/nonexistent/task10-broken.yaml'),
            stdout=stream, stderr=subprocess.STDOUT, start_new_session=True))
        client = AsyncParameterClient(node, '/avoidance_bridge')
        wait(client.services_are_ready, 'bridge parameter services')
        assert rpc(client.get_parameters(['native_namespace'])).values[0].string_value == '/avoidance'
        wait(lambda: any(r.ready and not r.request_id for r in results), 'CPU readiness')
        manager = node.create_client(SetBool, '/avoidance/backend/select')
        wait(manager.service_is_ready, 'backend manager')
        failed = rpc(manager.call_async(SetBool.Request(data=True)))
        assert not failed.success
        assert rpc(client.get_parameters(['native_namespace'])).values[0].string_value == '/avoidance'
        assert all(process.poll() is None for process in processes)
        cpu = plan(1)
        control.armed = True
        for _ in range(10):
            rclpy.spin_once(node, timeout_sec=.03)
        assert not select('/fpga_avoidance').successful
        control.armed = False
        for _ in range(10):
            rclpy.spin_once(node, timeout_sec=.03)
        assert select('/fpga_avoidance').successful
        wait(lambda: any(r.ready and r.planner_session != cpu.planner_session for r in results), 'new source heartbeat')
        fpga_peer = plan(2)
        assert fpga_peer.planner_session != cpu.planner_session
        assert fpga_peer.coordinate_revision > cpu.coordinate_revision
        boundary = len(results)
        for _ in range(20):
            rclpy.spin_once(node, timeout_sec=.03)
        assert not any(r.request_id == 1 and r.segments for r in results[boundary:])
        assert select('/avoidance').successful
        wait(lambda: any(r.ready and not r.request_id and r.planner_session == cpu.planner_session and
                         r.coordinate_revision > fpga_peer.coordinate_revision for r in results),
             'recovered CPU heartbeat')
        recovered = plan(3)
        assert recovered.planner_session == cpu.planner_session
        assert recovered.coordinate_revision > fpga_peer.coordinate_revision
    finally:
        node.destroy_timer(timer)
        for process in processes:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=5.)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait(timeout=3.)
        for stream in streams:
            stream.close()
        node.destroy_node()
        rclpy.shutdown()
