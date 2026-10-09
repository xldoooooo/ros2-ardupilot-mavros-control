#!/usr/bin/env python3
"""地面规划轨迹预览转发；撤销失效轨迹只影响 RViz，不发布控制请求。"""

import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from guided_interfaces.msg import AvoidanceResult, ControlStatus
from nav_msgs.msg import Path


class AvoidancePreview(Node):
    """只显示新鲜执行中的轨迹，并在通信超时或规划会话变化时清屏。"""

    TIMEOUT_SECONDS = 0.8  # 本地接收时钟门控，避免要求机载/地面绝对时钟同步。

    def __init__(self, **kwargs):
        """建立小队列订阅和 10 Hz 清屏定时器；不申请租约。"""
        super().__init__('ground_station_avoidance_preview', **kwargs)
        qos = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.publisher = self.create_publisher(Path, '/ground_station/avoidance_path', qos)
        self.path_subscription = self.create_subscription(
            Path, '/onboard_control/avoidance_path', self.path_callback, qos,
        )
        self.status_subscription = self.create_subscription(
            # 机载状态使用 SensorDataQoS；可靠订阅无法匹配 best-effort 发布者。
            ControlStatus, '/onboard_control/status', self.status_callback,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                       durability=DurabilityPolicy.VOLATILE),
        )
        self.session_subscription = self.create_subscription(
            AvoidanceResult, '/onboard_control/avoidance_result', self.session_callback, qos,
        )
        self.last_path_at = None
        self.last_status_at = None
        self.executing = False
        self.visible = False
        self.session = None
        self.timer = self.create_timer(0.1, self.expire)
        self.clear(force=True)

    def clear(self, *, force=False):
        """用 map 空 Path 覆盖 RViz 缓存；不重复发送已撤销的预览。"""
        self.last_path_at = None
        if self.visible or force:
            empty = Path()
            empty.header.frame_id = 'map'
            empty.header.stamp = self.get_clock().now().to_msg()
            self.publisher.publish(empty)
        self.visible = False

    def path_callback(self, message):
        """原样转发 map 轨迹，状态未就绪或源端清空时立即撤销。"""
        now = time.monotonic()
        if (not message.poses or message.header.frame_id != 'map'
                or not self.executing or self.last_status_at is None
                or now - self.last_status_at > self.TIMEOUT_SECONDS):
            self.clear(force=True)
            return
        self.last_path_at = now
        self.visible = True
        self.publisher.publish(message)

    def status_callback(self, message):
        """只有机载明确 EXECUTING 才允许显示轨迹；制动/等待/降落清屏。"""
        self.last_status_at = time.monotonic()
        self.executing = message.avoidance_state == ControlStatus.AVOIDANCE_EXECUTING
        if not self.executing:
            self.clear()

    def session_callback(self, message):
        """能力心跳不携带控制器身份；桥/规划器/坐标版本变化撤销旧图。"""
        session = (message.bridge_session, message.planner_session, message.coordinate_revision)
        if self.session is not None and session != self.session:
            self.clear(force=True)
        self.session = session

    def expire(self):
        """即使机载节点或网络停止发送，也能按接收超时撤销遗留轨迹。"""
        now = time.monotonic()
        if (self.last_path_at is None or self.last_status_at is None
                or now - self.last_path_at > self.TIMEOUT_SECONDS
                or now - self.last_status_at > self.TIMEOUT_SECONDS):
            self.clear()


def main(args=None):
    """作为 guided_sim ros2 可执行脚本运行，遵循 launch 的 SIGINT 清理。"""
    rclpy.init(args=args)
    node = AvoidancePreview()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
