#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
订阅 /odin1/cloud_slam_down (PointCloud2)，用 tf2 把点云从原始 frame
(通常是 odom) 变换到 map 系，然后每帧随机抽 10 个点打印坐标。

直接执行：
    source /opt/ros/humble/setup.bash
    python3 cloud_sample_in_map.py
可选：
    --topic /odin1/cloud_slam_down
    --target-frame map
    --num 10
    --min-interval 0.0        # 两次打印最小间隔(秒)，防刷屏
"""

import argparse
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2
import tf2_ros
from tf2_ros import TransformException

try:
    from sensor_msgs_py import point_cloud2 as pc2
    _HAS_PC2_NUMPY = True
except ImportError:
    pc2 = None
    _HAS_PC2_NUMPY = False


def cloud2_xyz(msg: PointCloud2) -> np.ndarray:
    """PointCloud2 -> (N,3) float32 ndarray，只取 x/y/z，剔除 NaN/Inf。"""
    if _HAS_PC2_NUMPY:
        try:
            pts = pc2.read_points_numpy(
                msg, field_names=('x', 'y', 'z'), skip_nans=True)
            return np.asarray(pts, dtype=np.float32).reshape(-1, 3)
        except Exception:
            pass  # 回退到手动解析

    fields = {f.name: f for f in msg.fields}
    if not {'x', 'y', 'z'} <= fields.keys():
        return np.empty((0, 3), dtype=np.float32)

    dt = np.dtype({
        'names': ['x', 'y', 'z'],
        'formats': ['<f4', '<f4', '<f4'],
        'offsets': [fields['x'].offset, fields['y'].offset, fields['z'].offset],
        'itemsize': msg.point_step,
    })
    n = msg.width * msg.height
    arr = np.frombuffer(msg.data, dtype=dt, count=n)
    pts = np.stack([arr['x'], arr['y'], arr['z']], axis=1).astype(np.float32)
    mask = np.isfinite(pts).all(axis=1)
    return pts[mask]


def quat_to_mat(q) -> np.ndarray:
    """四元数 (x,y,z,w) -> 3x3 旋转矩阵。"""
    x, y, z, w = q.x, q.y, q.z, q.w
    xx, yy, zz = x * x, y * y, z * z
    xy, xz, yz = x * y, x * z, y * z
    wx, wy, wz = w * x, w * y, w * z
    return np.array([
        [1 - 2 * (yy + zz), 2 * (xy - wz),     2 * (xz + wy)],
        [2 * (xy + wz),     1 - 2 * (xx + zz), 2 * (yz - wx)],
        [2 * (xz - wy),     2 * (yz + wx),     1 - 2 * (xx + yy)],
    ], dtype=np.float32)


def ts_to_Rt(ts):
    """TransformStamped -> (R, t)，使 p_target = R @ p_source + t。"""
    R = quat_to_mat(ts.transform.rotation)
    t = np.array([ts.transform.translation.x,
                  ts.transform.translation.y,
                  ts.transform.translation.z], dtype=np.float32)
    return R, t


class CloudSampler(Node):
    def __init__(self, topic, target_frame, n_sample, min_interval):
        super().__init__('cloud_sampler_map')
        self.target_frame = target_frame.lstrip('/')
        self.n_sample = n_sample
        self.min_interval = min_interval
        self.last_print = 0.0
        self.frame_id = 0

        # TF 监听
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # 传感器点云常用 BEST_EFFORT；兼容 reliable 发布者
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.sub = self.create_subscription(
            PointCloud2, topic, self.cb, qos)
        self.get_logger().info(
            f'已订阅 {topic}，目标坐标系 {self.target_frame}，'
            f'每帧随机打印 {n_sample} 个点')

    def lookup(self, src_frame, stamp):
        """优先按点云时间戳查，失败则回退到最新可用变换。"""
        try:
            return self.tf_buffer.lookup_transform(
                self.target_frame, src_frame, stamp)
        except TransformException:
            pass
        try:
            return self.tf_buffer.lookup_transform(
                self.target_frame, src_frame, Time())
        except TransformException as e:
            self.get_logger().warn(
                f'无法获取 {self.target_frame} <- {src_frame} 的变换: {e}',
                throttle_duration_sec=2.0)
            return None

    def cb(self, msg: PointCloud2):
        self.frame_id += 1
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self.last_print < self.min_interval:
            return
        self.last_print = now

        src_frame = msg.header.frame_id.lstrip('/')
        if not src_frame:
            self.get_logger().warn('点云 header.frame_id 为空，跳过该帧')
            return

        ts = self.lookup(src_frame, msg.header.stamp)
        if ts is None:
            return

        pts = cloud2_xyz(msg)
        total = pts.shape[0]
        if total == 0:
            self.get_logger().warn(f'第 {self.frame_id} 帧无有效点')
            return

        # odom -> map：p_map = R @ p_odom + t
        R, t = ts_to_Rt(ts)
        pts_map = pts @ R.T + t          # (N,3)

        k = min(self.n_sample, total)
        idx = np.random.choice(total, size=k, replace=False)
        sampled = pts_map[idx]

        stamp = f'{msg.header.stamp.sec}.{msg.header.stamp.nanosec:09d}'
        print(f'\n=== 帧 {self.frame_id} | src={src_frame} -> '
              f'dst={self.target_frame} | stamp={stamp} | '
              f'总点数={total} | 采样={k} ===')
        print(f'    (TF 平移 = [{t[0]:+.4f}, {t[1]:+.4f}, {t[2]:+.4f}])')
        for i, (x, y, z) in enumerate(sampled, 1):
            print(f'  [{i:2d}] x={x:+.4f}  y={y:+.4f}  z={z:+.4f}')


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--topic', default='/odin1/cloud_slam_down')
    p.add_argument('--target-frame', default='map')
    p.add_argument('--num', type=int, default=10, help='每帧采样点数')
    p.add_argument('--min-interval', type=float, default=0.0,
                   help='两次打印之间的最小间隔(秒)，防止刷屏')
    args = p.parse_args()

    rclpy.init()
    node = CloudSampler(args.topic, args.target_frame,
                        args.num, args.min_interval)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
