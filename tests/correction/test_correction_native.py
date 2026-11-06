"""Native ingress equivalence, source-clock scheduling and full ROS subscription lifecycle."""

import time

import numpy as np
import pytest
import rclpy
from nav_msgs.msg import Odometry
from rclpy.serialization import serialize_message, deserialize_message
import _correction_native as native

from correction_service.geometry import transform_from_pose
from correction_service.synchronizer import is_apply_time_source_safe


def raw(stamp, x=0.):
    """Build a complete FCU odometry record with distinguishable pose and covariance."""
    message = Odometry()
    message.header.frame_id, message.child_frame_id = 'odom', 'fcu'
    message.header.stamp.sec, message.header.stamp.nanosec = divmod(stamp, 1_000_000_000)
    message.pose.pose.position.x = x
    message.pose.pose.orientation.z, message.pose.pose.orientation.w = .2, .9
    message.twist.twist.linear.x = .3
    message.pose.covariance[0] = .007
    return message


def capture():
    """Create a replay receiver without subscribing to hardware topics."""
    return native.OdometryCapture(4., 20., 30., 5., 'test', lambda: None)


@pytest.mark.parametrize('epoch', [0, 1_700_000_000_000_000_000])
def test_native_history_matches_original_time_axis_and_complete_message(epoch):
    """Keep strict same-epoch matching, arrival fallback, tie order and all ROS fields."""
    receiver = capture()
    for index in range(1800):
        stamp = 100_000_000_000 + index*2_500_000
        message = raw(stamp, index*.001)
        arrival = stamp+epoch+1_000_000
        receiver.add(serialize_message(message), arrival, 1.+index*.0025)
    # The first record has expired; equal header distances keep the earlier record.
    cases = [(100_000_000_000, None, None),
             (102_503_750_000, 1001, .25 if epoch else 1.25),
             (104_499_000_000, 1799, .5 if epoch else 1.5),
             (104_600_000_000, None, None)]
    for target, index, expected_delta in cases:
        result = receiver.match(target+epoch)
        if index is None:
            assert result is None
        else:
            data, delta, source = result
            expected = raw(100_000_000_000+index*2_500_000, index*.001)
            assert source == ("arrival_history" if epoch else "header")
            assert delta == expected_delta
            assert serialize_message(expected) == data
            assert deserialize_message(data, Odometry).pose.covariance[0] == .007


def test_native_same_epoch_rejection_and_explicit_epoch_fallback():
    """A bad same-epoch header cannot hide behind a close arrival timestamp."""
    receiver = capture()
    image = 100_000_000_000
    receiver.add(serialize_message(raw(image+100_000_000)), image, 1.)
    assert receiver.match(image) is None
    receiver.clear()
    receiver.add(serialize_message(raw(8_000_000_000_000)), image+4_000_000, 1.1)
    data, delta, source = receiver.match(image)
    assert deserialize_message(data, Odometry).header.stamp.sec == 8000
    assert source == "arrival_history" and delta == 4.
    assert is_apply_time_source_safe(source) and is_apply_time_source_safe("header")
    assert not is_apply_time_source_safe("mixed")


def test_native_faults_preserve_last_valid_pose_and_clear_discontinuous_history():
    """Bad frame/pose and time discontinuities must survive the low-rate Python boundary."""
    receiver = capture()
    message = raw(10_000_000_000)
    receiver.add(serialize_message(message), 10_000_000_000, 1.)
    expected = transform_from_pose(np.array((0., 0., 0.)), np.array((0., 0., .2, .9)))
    assert np.allclose(np.array(receiver.snapshot().transform).reshape(4,4), expected, atol=1e-15)
    message.child_frame_id = 'odin_imu'
    receiver.add(serialize_message(message), 10_000_000_001, 1.1)
    assert receiver.faults()[0][1] is True
    assert receiver.snapshot().received == 1.
    message.child_frame_id = 'fcu'
    message.pose.pose.orientation.w = float('nan')
    receiver.add(serialize_message(message), 10_000_000_002, 1.2)
    assert receiver.faults() == [('原始 Odin 位姿无效：四元数无效', False)]
    message = raw(9_000_000_000)
    old_session = receiver.snapshot().session
    receiver.add(serialize_message(message), 11_000_000_000, 1.3)
    assert receiver.faults() == [('任务 Odin header 时间戳回退', True)]
    assert receiver.match(10_000_000_000) is None
    assert receiver.snapshot().session != old_session
    receiver.add(serialize_message(message), 8_000_000_000, 1.4)
    assert receiver.faults() == [('主机 ROS 时钟回退', True)]
    receiver.add(serialize_message(message), 12_000_000_000, 4.)
    assert receiver.faults() == [('任务 Odin 断流 2.600s', True)]


def test_native_capture_clock_and_scheduler_preserve_real_frames():
    """Negotiated-fps mismatch never retimes frames or introduces a second sampling gate."""
    limiter = native.CaptureRateLimiter(10.)
    accepted = []
    for index in range(1200):
        stamp = 100_000_000_000+round(index*1e6/59.94)*1000
        sec, fraction = divmod(stamp, 1_000_000_000)
        ros, age = native.capture_stamp(0x2000, sec, fraction//1000,
                                        stamp+12_000_000, stamp+12_000_000+1_700_000_000_000_000_000)
        assert age == 12_000_000 and ros == stamp+1_700_000_000_000_000_000
        if limiter.accept(stamp):
            accepted.append(stamp)
    assert len(accepted) == 201
    with pytest.raises(RuntimeError, match='CLOCK_MONOTONIC'):
        native.capture_stamp(0, 100, 0, 101_000_000_000, 101_000_000_000)


@pytest.mark.integration
def test_native_subscription_receives_real_dds_and_releases_context():
    """Exercise native ROS reception and on-demand conversion in an isolated domain."""
    from rclpy.context import Context
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data

    context = Context()
    context.init(args=[])
    node = Node('native_capture_test', context=context)
    publisher = node.create_publisher(Odometry, '/native_capture_test/raw', qos_profile_sensor_data)
    receiver = capture()
    try:
        receiver.start('/native_capture_test/raw', '/', context.get_domain_id(), False)
        deadline = time.monotonic()+5.
        while time.monotonic() < deadline and not receiver.snapshot().valid:
            publisher.publish(raw(10_000_000_000, .42))
            time.sleep(.02)
        assert receiver.snapshot().valid
        assert receiver.match(10_000_000_000) is not None
        assert np.array(receiver.snapshot().transform).reshape(4,4)[0,3] == .42
        assert receiver.running
        receiver.close()
        deadline = time.monotonic()+2.
        while publisher.get_subscription_count() and time.monotonic() < deadline:
            time.sleep(.02)
        assert publisher.get_subscription_count() == 0
    finally:
        receiver.close()
        receiver.shutdown()
        node.destroy_node()
        context.shutdown()
    assert not receiver.running and not receiver.snapshot().valid
