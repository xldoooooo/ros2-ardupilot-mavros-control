#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
订阅 /odin1/cloud_slam_down，每帧随机采样 10 个点并打印坐标。
直接执行：
    source /opt/ros/humble/setup.bash    # 换成你的发行版
    python3 cloud_sample.py
可选参数：
    --topic 话题名  --num 每帧采样数  --min-interval 打印最小间隔(秒)
"""

import argparse
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2

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
    pts = np.stack([arr['x'], arr['y'], arr['z']], axis=1)
    mask = np.isfinite(pts).all(axis=1)
    return pts[mask]


class CloudSampler(Node):
    def __init__(self, topic, n_sample, min_interval):
        super().__init__('cloud_sampler')
        self.n_sample = n_sample
        self.min_interval = min_interval
        self.last_print = 0.0
        self.frame_id = 0

        # 传感器点云常用 BEST_EFFORT；兼容 reliable 发布者
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.sub = self.create_subscription(
            PointCloud2, topic, self.cb, qos)
        self.get_logger().info(
            f'已订阅 {topic}，每帧随机打印 {n_sample} 个点')

    def cb(self, msg: PointCloud2):
        self.frame_id += 1
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self.last_print < self.min_interval:
            return
        self.last_print = now

        pts = cloud2_xyz(msg)
        total = pts.shape[0]
        if total == 0:
            self.get_logger().warn(f'第 {self.frame_id} 帧无有效点')
            return

        k = min(self.n_sample, total)
        idx = np.random.choice(total, size=k, replace=False)
        sampled = pts[idx]

        stamp = f'{msg.header.stamp.sec}.{msg.header.stamp.nanosec:09d}'
        print(f'\n=== 帧 {self.frame_id} | frame={msg.header.frame_id} '
              f'| stamp={stamp} | 总点数={total} | 采样={k} ===')
        for i, (x, y, z) in enumerate(sampled, 1):
            print(f'  [{i:2d}] x={x:+.4f}  y={y:+.4f}  z={z:+.4f}')


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--topic', default='/odin1/cloud_slam_down')
    p.add_argument('--num', type=int, default=10, help='每帧采样点数')
    p.add_argument('--min-interval', type=float, default=0.0,
                   help='两次打印之间的最小间隔(秒)，防止刷屏')
    args = p.parse_args()

    rclpy.init()
    node = CloudSampler(args.topic, args.num, args.min_interval)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
