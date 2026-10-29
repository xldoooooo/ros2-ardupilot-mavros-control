#!/usr/bin/env python3
"""Publish the original fixed demo goal through ROS 2."""
import time

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

from . import rospy_compat as rospy


def _run() -> None:
    rospy.init_node("publish_demo_goal")
    pub = rospy.Publisher("/goal", PoseStamped, queue_size=1, latch=True)
    deadline = time.monotonic() + 10.0
    while pub.get_num_connections() < 3 and time.monotonic() < deadline:
        rospy.sleep(0.1)
    connections = pub.get_num_connections()
    if connections < 3:
        rospy.logwarn(
            "Publishing demo goal with only %d/3 expected subscriptions",
            connections,
        )

    # The frozen V24 reference node intentionally ignores a goal received
    # before its first odometry callback.  Observe the bag itself instead of
    # relying on a scheduler-sensitive fixed delay, then allow the other nodes
    # to consume the same stream before publishing the test goal.
    odometry_seen = False

    def mark_odometry_seen(_message: Odometry) -> None:
        nonlocal odometry_seen
        odometry_seen = True

    odometry_subscription = rospy.Subscriber(
        "/mavros/odometry/out",
        Odometry,
        mark_odometry_seen,
        queue_size=1,
    )
    odometry_deadline = time.monotonic() + 10.0
    while not odometry_seen and time.monotonic() < odometry_deadline:
        rospy.spin_once(0.1)
    if not odometry_seen:
        rospy.logwarn("No odometry observed before demo goal timeout")
    else:
        rospy.sleep(0.5)

    msg = PoseStamped()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = "camera_init"
    msg.pose.position.x = 5.0
    msg.pose.position.y = 0.0
    msg.pose.position.z = 1.0
    msg.pose.orientation.w = 1.0

    pub.publish(msg)
    publish_count = 1

    def republish_until_reference_is_ready(_event: object) -> None:
        nonlocal publish_count
        if publish_count >= 6:
            return
        msg.header.stamp = rospy.Time.now()
        pub.publish(msg)
        publish_count += 1

    retry_timer = rospy.Timer(
        rospy.Duration(1.0),
        republish_until_reference_is_ready,
    )
    rospy.loginfo(
        "Published demo goal: x=5.0 y=0.0 z=1.0 frame=camera_init subscribers=%d",
        connections,
    )
    # Retain the subscription for the lifetime of this short-lived helper.
    _ = (odometry_subscription, retry_timer)
    rospy.spin()


def main() -> None:
    try:
        _run()
    except KeyboardInterrupt:
        # ros2 launch can interrupt this helper while it is still waiting for
        # subscribers/odometry; a normal shutdown must not print a traceback.
        pass


if __name__ == "__main__":
    main()
