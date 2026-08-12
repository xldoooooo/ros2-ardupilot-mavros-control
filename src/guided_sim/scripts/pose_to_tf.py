#!/usr/bin/env python3
"""把当前会话选择的位姿转换为隔离命名的 RViz TF2 机体变换。"""

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import TransformBroadcaster


class PoseToTF(Node):
    """把本地预览位姿转换为 map → ground_station_preview/base_link TF。"""

    def __init__(self):
        """创建可配置的位姿订阅与 TF broadcaster。"""
        super().__init__('pose_to_tf')

        self.declare_parameter('pose_topic', '/mavros/local_position/pose')
        self.declare_parameter('parent_frame', 'map')
        self.declare_parameter(
            'child_frame', 'ground_station_preview/base_link'
        )
        self.pose_topic = str(self.get_parameter('pose_topic').value)
        self.parent_frame = str(self.get_parameter('parent_frame').value)
        self.child_frame = str(self.get_parameter('child_frame').value)

        # TF 仅服务地面预览命名空间，不覆盖远端飞控或 Odin 的 frame。
        self.tf_broadcaster = TransformBroadcaster(self)

        # 仿真选本地域 MAVROS；实机启动器改选地面聚合位姿，二者共用桥接逻辑。
        self.sub = self.create_subscription(
            PoseStamped,
            self.pose_topic,
            self.pose_callback,
            rclpy.qos.QoSProfile(
                # 预览只需要最新位姿；负载高时不得追赶过期姿态形成回调突发。
                depth=1,
                durability=rclpy.qos.DurabilityPolicy.VOLATILE,
                reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            ),
        )

        self.get_logger().info(
            f'pose_to_tf started — bridging {self.pose_topic} to '
            f'{self.parent_frame} -> {self.child_frame}'
        )

    def pose_callback(self, msg: PoseStamped):
        """逐帧转发位移和姿态，保留上游 PoseStamped 时间戳。"""
        t = TransformStamped()

        t.header.stamp = msg.header.stamp
        t.header.frame_id = self.parent_frame
        t.child_frame_id = self.child_frame

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
