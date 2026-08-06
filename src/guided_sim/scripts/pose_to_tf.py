#!/usr/bin/env python3
"""
pose_to_tf.py — Bridge MAVROS local_position/pose to TF2 transforms.

Subscribes to /mavros/local_position/pose (PoseStamped) and publishes the
transform map -> base_link so that RViz2 can display the quadcopter URDF
model at the correct 3D pose in real time.
"""

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import TransformBroadcaster


class PoseToTF(Node):
    """把 MAVROS 本地位姿转换为 RViz 使用的 map → base_link TF。"""

    def __init__(self):
        """创建 TF broadcaster 与 MAVROS 位姿订阅。"""
        super().__init__('pose_to_tf')

        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Subscribe to MAVROS local position
        self.sub = self.create_subscription(
            PoseStamped,
            '/mavros/local_position/pose',
            self.pose_callback,
            rclpy.qos.QoSProfile(
                depth=10,
                durability=rclpy.qos.DurabilityPolicy.VOLATILE,
                reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            ),
        )

        self.get_logger().info(
            'pose_to_tf node started — bridging /mavros/local_position/pose to TF'
        )

    def pose_callback(self, msg: PoseStamped):
        """逐帧转发位移和姿态，保留 MAVROS 原始时间戳。"""
        t = TransformStamped()

        t.header.stamp = msg.header.stamp
        t.header.frame_id = 'map'
        t.child_frame_id = 'base_link'

        t.transform.translation.x = msg.pose.position.x
        t.transform.translation.y = msg.pose.position.y
        t.transform.translation.z = msg.pose.position.z
        t.transform.rotation = msg.pose.orientation

        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    """运行桥接节点，并在 ros2 launch 发送 SIGINT 时安静退出。"""
    rclpy.init(args=args)
    node = PoseToTF()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # launch 可能连续转发 SIGINT；销毁阶段同样捕获，避免正常清理打印堆栈。
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except KeyboardInterrupt:
                pass


if __name__ == '__main__':
    main()
