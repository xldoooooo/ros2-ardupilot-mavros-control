#!/usr/bin/env python3
"""把当前会话选择的位姿转换为隔离命名的 RViz TF2 机体变换。"""

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_geometry_msgs import do_transform_pose
from tf2_ros import Buffer, TransformBroadcaster, TransformException, TransformListener


# 缺少地图 TF 时最多每五秒告警一次，避免启动发现阶段刷屏。
_TF_WARNING_INTERVAL_SECONDS = 5.0


class PoseToTF(Node):
    """Bridge preview poses into one display parent without mixing TF clock domains."""

    def __init__(self, **kwargs):
        """创建可配置的位姿订阅与 TF broadcaster。"""
        super().__init__('pose_to_tf', **kwargs)

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
        self.tf_buffer = Buffer(node=self)
        self.tf_listener = TransformListener(self.tf_buffer, self)

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
        """MAP 位姿按最新 TF 转入显示系；模型始终保留源时刻和同一父 frame。"""
        pose = msg.pose
        if msg.header.frame_id == 'fcu_local':
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.parent_frame, 'fcu_local', Time(),
                )
            except TransformException as exc:
                self.get_logger().warning(
                    f'MAP 模型等待 {self.parent_frame} ← fcu_local TF：{exc}',
                    throttle_duration_sec=_TF_WARNING_INTERVAL_SECONDS,
                )
                return
            # 先求显示坐标，避免设备秒地图 TF 与墙钟机体 TF 混在同一动态链中。
            pose = do_transform_pose(pose, transform)

        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = self.parent_frame
        t.child_frame_id = self.child_frame

        t.transform.translation.x = pose.position.x
        t.transform.translation.y = pose.position.y
        t.transform.translation.z = pose.position.z
        t.transform.rotation = pose.orientation

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
