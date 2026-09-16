"""隔离 DDS 验证正式机载优先参数请求，不连接实机、不发送飞行命令。"""
import os
from pathlib import Path
import signal
import subprocess
import time

import pytest

ROOT = Path(__file__).resolve().parents[1]


@pytest.mark.parametrize('enabled', [True, False])
def test_priority_parameter_requests_are_bounded_and_reset_on_disconnect(tmp_path, enabled):
    """校验真实 MAVLink 编码、上限、开关、重连、收齐停止与未武装门。"""
    rclpy = pytest.importorskip('rclpy')
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.parameter import Parameter
    from rclpy.qos import qos_profile_sensor_data as qos
    from mavros_msgs.msg import Mavlink, ParamEvent, State
    from guided_interfaces.msg import ControlStatus
    from guided_interfaces.srv import AcquireControl
    from mavros.mavlink import convert_to_bytes
    from pymavlink.dialects.v20 import ardupilotmega

    domain = 220 + os.getpid() % 8
    context = Context()
    rclpy.init(context=context, domain_id=domain)
    node = rclpy.create_node('priority_read_fixture', context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    state_pub = node.create_publisher(State, '/priority_fixture/state', 10)
    event_pub = node.create_publisher(ParamEvent, '/priority_fixture/param/event', 20)
    requests, statuses = [], []
    decoder = ardupilotmega.MAVLink(None)
    node.create_subscription(Mavlink, '/priority_fixture/sink',
                             lambda m: requests.append(decoder.decode(convert_to_bytes(m))), qos)
    node.create_subscription(ControlStatus, '/priority_fixture/control/status', statuses.append, qos)
    lease_client = node.create_client(AcquireControl, "/priority_fixture/control/acquire_control")
    connected, armed = True, True
    node.create_timer(.05, lambda: state_pub.publish(State(connected=connected, armed=armed)))

    def until(predicate, seconds=3):
        deadline = time.monotonic()+seconds
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.02)
            if predicate():
                return True
        return False

    def event(name, value):
        event_pub.publish(ParamEvent(param_id=name, value=Parameter(name, value=value).get_parameter_value()))

    executable = ROOT / 'install/onboard_control/lib/onboard_control/onboard_control_node'
    with (tmp_path / 'node.log').open('w') as output:
        process = subprocess.Popen([str(executable), '--ros-args',
            '-p', 'mavros_prefix:=/priority_fixture', '-p', 'interface_prefix:=/priority_fixture/control',
            '-p', 'parameter_request_topic:=/priority_fixture/sink',
            '-p', 'parameter_target_system:=42', '-p', 'parameter_target_component:=7',
            '-p', f'priority_parameter_reads:={str(enabled).lower()}'],
            env=dict(os.environ, ROS_DOMAIN_ID=str(domain), ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST'),
            stdout=output, stderr=subprocess.STDOUT)
        try:
            assert until(lambda: statuses and statuses[-1].armed)
            assert not until(lambda: requests, .5), '武装时不得开始启动参数请求'
            if enabled:
                assert until(lease_client.service_is_ready)
                future = lease_client.call_async(AcquireControl.Request(
                    stamp=node.get_clock().now().to_msg(), source_id='fixture',
                    sequence=1, lease_duration_ms=5000))
                assert until(future.done) and future.result().granted
                assert until(lambda: statuses[-1].lease_active)
            armed = False
            if not enabled:
                assert not until(lambda: requests, 4)
                return
            assert until(lambda: len(requests) >= 2, 1.5), '仅持有租约不得阻塞只读同步'
            assert statuses[-1].lease_active
            assert until(lambda: len(requests) == 8, 5), (len(requests), (tmp_path/'node.log').read_text())
            assert not until(lambda: len(requests) > 8, 1.2), '无回复时必须停止重试'
            assert not statuses[-1].thrust_mode_verified, '发送请求不能替代参数值校验'
            for message in requests:
                assert message.get_msgId() == 20
                assert (message.target_system, message.target_component) == (42, 7)
                assert message.param_index == -1
                assert message.param_id in ('GUID_OPTIONS', 'MOT_THST_HOVER')
            assert len({m.get_seq() for m in requests}) == 8
            connected = False
            assert until(lambda: not statuses[-1].fcu_connected)
            before = len(requests)
            connected = True
            assert until(lambda: len(requests) >= before+2), '重连必须重新尝试'
            assert until(lambda: event_pub.get_subscription_count() > 0)
            event('GUID_OPTIONS', 8)
            assert not until(lambda: statuses[-1].thrust_mode_verified, .15)
            event('MOT_THST_HOVER', .22)
            assert until(lambda: statuses[-1].thrust_mode_verified)
            before = len(requests)
            assert not until(lambda: len(requests) > before, 1.2), '收齐后不得继续读取'
            connected = False
            assert until(lambda: not statuses[-1].fcu_connected)
            connected = True
            assert until(lambda: len(requests) >= before+2)
            event('GUID_OPTIONS', 8)
            assert not until(lambda: statuses[-1].thrust_mode_verified, .25), '不得拼接上个连接的悬停值'
            event('MOT_THST_HOVER', .22)
            assert until(lambda: statuses[-1].thrust_mode_verified)
        finally:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
            executor.shutdown()
            node.destroy_node()
            context.shutdown()
