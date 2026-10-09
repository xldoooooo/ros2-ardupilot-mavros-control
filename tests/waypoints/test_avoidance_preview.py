"""规划预览生命周期回归；隔离 ROS 域，不接入机载控制或发送飞行命令。"""

import importlib.util
import time
from types import SimpleNamespace

import pytest
import rclpy
from geometry_msgs.msg import PoseStamped
from guided_interfaces.msg import AvoidanceResult, ControlStatus
from nav_msgs.msg import Path
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from ground_station_core.config import PROJECT_ROOT


_spec = importlib.util.spec_from_file_location(
    "avoidance_preview", PROJECT_ROOT / "src/guided_sim/scripts/avoidance_preview.py"
)
preview_module = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(preview_module)


@pytest.fixture
def preview(monkeypatch):
    """使用真实消息和节点，但捕获输出以精确覆盖超时、会话和状态边界。"""
    context = Context()
    rclpy.init(context=context, domain_id=229)
    node = preview_module.AvoidancePreview(context=context)
    node.timer.cancel()
    output = []
    node.publisher = SimpleNamespace(publish=output.append)
    clock = [100.0]
    monkeypatch.setattr(preview_module.time, "monotonic", lambda: clock[0])
    try:
        yield node, output, clock
    finally:
        node.destroy_node()
        context.shutdown()


def trajectory():
    """构造一个保留源端时间戳的 map 预览；控制器不消费此话题。"""
    path = Path()
    path.header.frame_id = 'map'
    path.header.stamp.sec = 123
    pose = PoseStamped()
    pose.pose.position.x = 4.0
    path.poses = [pose]
    return path


def executing_status():
    """只包含预览显示门控需要的机载执行状态。"""
    status = ControlStatus()
    status.avoidance_state = ControlStatus.AVOIDANCE_EXECUTING
    return status


def test_preview_requires_fresh_execution_status_and_preserves_source_path(preview):
    node, output, clock = preview
    path = trajectory()
    node.path_callback(path)
    assert not output[-1].poses
    node.status_callback(executing_status())
    node.path_callback(path)
    assert output[-1] is path
    assert output[-1].header.stamp.sec == 123
    assert node.path_subscription.qos_profile.reliability == ReliabilityPolicy.RELIABLE
    assert node.path_subscription.qos_profile.durability == DurabilityPolicy.VOLATILE
    assert node.status_subscription.qos_profile.reliability == ReliabilityPolicy.BEST_EFFORT
    assert node.status_subscription.qos_profile.durability == DurabilityPolicy.VOLATILE
    assert node.status_subscription.qos_profile.depth == 1
    clock[0] += 0.801
    # 新路径也不能让已超时的状态“复活”；需重新收到 EXECUTING 状态。
    node.path_callback(path)
    assert not output[-1].poses
    node.status_callback(executing_status())
    node.path_callback(path)
    assert output[-1].poses


def test_preview_path_timeout_clears_even_while_status_remains_fresh(preview):
    node, output, clock = preview
    node.status_callback(executing_status())
    node.path_callback(trajectory())
    clock[0] += 0.8
    node.status_callback(executing_status())
    node.expire()
    assert output[-1].poses
    clock[0] += 0.001
    node.expire()
    assert output[-1].header.frame_id == 'map'
    assert not output[-1].poses


@pytest.mark.parametrize('state', [0, 1, 3, 4])
def test_preview_revokes_trajectory_for_non_execution_states(preview, state):
    node, output, _ = preview
    node.status_callback(executing_status())
    node.path_callback(trajectory())
    status = ControlStatus()
    status.avoidance_state = state
    node.status_callback(status)
    assert not output[-1].poses


@pytest.mark.parametrize('field,value', [
    ('bridge_session', 'bridge-new'), ('planner_session', 'planner-new'),
    ('coordinate_revision', 2),
])
def test_preview_clears_on_new_sessions_and_ignores_empty_controller_heartbeat(preview, field, value):
    node, output, _ = preview
    result = AvoidanceResult()
    result.bridge_session = 'bridge'
    result.planner_session = 'planner'
    result.coordinate_revision = 1
    result.controller_session = 'controller'
    node.session_callback(result)
    node.status_callback(executing_status())
    node.path_callback(trajectory())
    result.controller_session = ''
    result.request_id = 0
    node.session_callback(result)
    assert output[-1].poses
    setattr(result, field, value)
    node.session_callback(result)
    assert not output[-1].poses


def test_preview_dds_timer_clears_last_path_after_source_stops():
    """通过真实 DDS 转发轨迹，再停止输入，确认定时器主动发布空 Path。"""
    context = Context()
    rclpy.init(context=context, domain_id=228)
    preview_node = preview_module.AvoidancePreview(context=context)
    probe = rclpy.create_node('avoidance_preview_probe', context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(preview_node)
    executor.add_node(probe)
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
    # 必须模拟真实机载状态 QoS；可靠测试发布者会掩盖生产订阅不匹配。
    statuses = probe.create_publisher(
        ControlStatus, '/onboard_control/status',
        QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                   durability=DurabilityPolicy.VOLATILE),
    )
    paths = probe.create_publisher(Path, '/onboard_control/avoidance_path', qos)
    output = []
    subscription = probe.create_subscription(Path, '/ground_station/avoidance_path', output.append, qos)
    try:
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            statuses.publish(executing_status())
            paths.publish(trajectory())
            executor.spin_once(timeout_sec=0.01)
            if any(path.poses for path in output):
                break
        assert any(path.poses for path in output)
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline and output[-1].poses:
            executor.spin_once(timeout_sec=0.05)
        assert not output[-1].poses
        assert output[-1].header.frame_id == 'map'
    finally:
        probe.destroy_subscription(subscription)
        executor.remove_node(probe)
        executor.remove_node(preview_node)
        probe.destroy_node()
        preview_node.destroy_node()
        executor.shutdown()
        context.shutdown()
