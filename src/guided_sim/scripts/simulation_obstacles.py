#!/usr/bin/env python3
"""SITL-only geometric scan: first-surface occlusion, delayed cylinder and RViz marker; no flight commands."""
import os
import signal
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Bool
from visualization_msgs.msg import Marker, MarkerArray


# Finite demonstration room fits the production 300000-cell map; scan faces map +X.
ROOM_LOWER = np.array([-2.5, -2.5, -.25])
ROOM_UPPER = np.array([6., 2.5, 3.5])
POLE_RADIUS = .18  # Geometric cylinder surface radius in metres; RViz uses the same geometry.


def raycast_scene(origin, directions, anchor, pole_center, pole):
    """Return only current first hits in map, never points behind a foreground surface."""
    lower, upper = anchor + ROOM_LOWER, anchor + ROOM_UPPER
    if not np.isfinite(origin).all() or np.any(origin <= lower) or np.any(origin >= upper):
        return np.empty((0, 3), dtype='<f4')
    with np.errstate(divide='ignore', invalid='ignore'):
        hits = np.where(directions > 0, (upper-origin)/directions, (lower-origin)/directions)
        hits[hits <= 0] = np.inf
        distance = np.min(hits, axis=1)
        if pole:
            offset = origin[:2] - pole_center
            aa = np.sum(directions[:, :2]**2, axis=1)
            bb = 2*(directions[:, :2] @ offset)
            cc = offset @ offset - POLE_RADIUS**2
            disc = bb**2 - 4*aa*cc
            valid = (disc >= 0) & (aa > 0)
            t = np.full(len(directions), np.inf)
            t[valid] = (-bb[valid]-np.sqrt(disc[valid]))/(2*aa[valid])
            hit_z = origin[2] + t*directions[:, 2]
            valid &= (t > 0) & (hit_z >= lower[2]) & (hit_z <= upper[2])
            distance[valid] = np.minimum(distance[valid], t[valid])
        points = origin + distance[:, None]*directions
    return points[np.isfinite(points).all(axis=1)].astype('<f4')


class SimulationObstacles(Node):
    """Registered geometric scans follow actual simulated FCU pose, inserting an occupied cylinder after takeoff."""
    def __init__(self):
        super().__init__('simulation_obstacles')
        self.pose = None
        self.anchor = None
        self.last_pose_received = 0.
        self.settled_since = None
        self.previous_origin = None
        self.previous_time = None
        self.last_pose_stamp = None
        self.pole = False
        self.manual_obstacle = None
        a, e = np.meshgrid(np.radians(np.arange(-55., 55.1, 1.)),
                          np.radians(np.arange(-35., 35.1, 1.)))
        self.directions = np.column_stack((np.cos(e).ravel()*np.cos(a).ravel(),
                                          np.cos(e).ravel()*np.sin(a).ravel(), np.sin(e).ravel()))
        self.cloud_pub = self.create_publisher(PointCloud2, '/simulation/scan', qos_profile_sensor_data)
        self.odom_pub = self.create_publisher(Odometry, '/simulation/scan_odom', qos_profile_sensor_data)
        self.marker_pub = self.create_publisher(MarkerArray, '/simulation/obstacles', 1)
        self.create_subscription(PoseStamped, '/mavros/local_position/pose', self.on_pose, qos_profile_sensor_data)
        self.create_subscription(Bool, '/simulation/obstacle_enabled', self.on_obstacle, 1)
        self.create_timer(.1, self.publish_scan)
        self.get_logger().info('SITL scene: cylinder at initial (x+1.5,y); waypoint example (x+3,y,1.5)')

    def on_pose(self, pose):
        """Anchor once; stopping FCU input stops scan freshness rather than replaying old poses."""
        stamp = pose.header.stamp.sec*10**9+pose.header.stamp.nanosec
        p = pose.pose.position
        if pose.header.frame_id != 'map' or not np.isfinite([p.x, p.y, p.z]).all():
            return
        if self.last_pose_stamp is not None and stamp <= self.last_pose_stamp:
            return
        self.last_pose_stamp = stamp
        self.pose = pose
        self.last_pose_received = time.monotonic()
        if self.anchor is None:
            self.anchor = np.array([p.x, p.y, p.z])

    def on_obstacle(self, message):
        """Optional manual show/hide changes geometry, not planner readiness or flight state."""
        self.manual_obstacle = message.data
        self.pole = message.data

    def publish_scan(self):
        """Publish synchronized current scan/odom and a marker matching the occupied geometry."""
        if self.pose is None or time.monotonic()-self.last_pose_received > .5:
            return
        p = self.pose.pose.position
        origin = np.array([p.x, p.y, p.z])
        current_time = time.monotonic()
        speed = (np.linalg.norm(origin-self.previous_origin)/(current_time-self.previous_time)
                 if self.previous_origin is not None else np.inf)
        self.previous_origin, self.previous_time = origin.copy(), current_time
        # Observe airspace before inserting a moving obstacle; its shadow is never marked free.
        if self.manual_obstacle is None and not self.pole:
            if origin[2]-self.anchor[2] > .2 and speed < .1:
                if self.settled_since is None:
                    self.settled_since = current_time
                if current_time-self.settled_since >= 2.:
                    self.pole = True
                    self.get_logger().info('Cylinder inserted after two seconds of settled airborne scans')
            else:
                self.settled_since = None
        center = self.anchor[:2] + np.array([1.5, 0.])
        points = raycast_scene(origin, self.directions, self.anchor, center, self.pole)
        odom = Odometry()
        odom.header.frame_id = 'map'
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.child_frame_id = 'simulated_lidar'
        odom.pose.pose = self.pose.pose
        self.odom_pub.publish(odom)
        cloud = PointCloud2()
        cloud.header = odom.header
        cloud.height, cloud.width = 1, len(points)
        cloud.point_step, cloud.row_step = 12, 12*len(points)
        cloud.is_dense = True
        cloud.fields = [PointField(name=n, offset=i*4, datatype=PointField.FLOAT32, count=1)
                        for i, n in enumerate(('x', 'y', 'z'))]
        cloud.data = points.tobytes()
        self.cloud_pub.publish(cloud)
        marker = Marker()
        marker.header = odom.header
        marker.ns, marker.id = 'simulation_cylinder', 0
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD if self.pole else Marker.DELETE
        marker.pose.position.x, marker.pose.position.y = float(center[0]), float(center[1])
        marker.pose.position.z = float(self.anchor[2]+(ROOM_UPPER[2]+ROOM_LOWER[2])/2)
        marker.pose.orientation.w = 1.
        marker.scale.x = marker.scale.y = 2*POLE_RADIUS
        marker.scale.z = float(ROOM_UPPER[2]-ROOM_LOWER[2])
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = 1., .15, .1, .8
        self.marker_pub.publish(MarkerArray(markers=[marker]))


def main():
    # Demo inputs must never appear in the real aircraft's domain 0.
    if os.environ.get('ROS_DOMAIN_ID') != '231' or os.environ.get('ROS_AUTOMATIC_DISCOVERY_RANGE') != 'LOCALHOST':
        raise RuntimeError('Simulation scene requires ROS_DOMAIN_ID=231 and LOCALHOST discovery')
    rclpy.init()
    node = SimulationObstacles()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # Supervisor and launch can both send SIGINT; do not interrupt DDS entity destruction twice.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
