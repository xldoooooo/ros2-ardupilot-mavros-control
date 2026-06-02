#!/usr/bin/env python3
"""
pose_to_tf.py — Bridge MAVROS local_position/pose to TF2 transforms.

Subscribes to /mavros/local_position/pose (PoseStamped) and publishes the
transform map -> base_link so that RViz2 can display the quadcopter URDF
model at the correct 3D pose in real time.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import TransformBroadcaster


class PoseToTF(Node):

    def __init__(self):
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

        self.get_logger().info('pose_to_tf node started — bridging /mavros/local_position/pose to TF')

    def pose_callback(self, msg: PoseStamped):
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
    rclpy.init(args=args)
    node = PoseToTF()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
