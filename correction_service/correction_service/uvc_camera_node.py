"""按需启动的自有 UVC ROS 节点：系统设备 → MJPEG → 原始 mono8 图像。"""

from __future__ import annotations

import signal
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from sensor_msgs.msg import Image

from .uvc_capture import UvcCapture, capture_stamp_ns


def main(args: list[str] | None = None) -> None:
    """保留采集时间与灰度像素；异常退出由 CameraProcess 转为任务失败。"""
    # 后台 shell 可能继承 SIG_IGN；显式恢复 Python 中断处理，保证按需停止可用。
    signal.signal(signal.SIGINT, signal.default_int_handler)
    # 采集循环由 Python SIGINT 打断；不让 ROS 异步关闭 context 与 publish/清理竞争。
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = Node("correction_uvc_camera")
    capture = None
    try:
        defaults = {
            "video_device": "/dev/video0", "image_width": 1920,
            "image_height": 1080, "framerate": 30,
            "frame_id": "correction_camera_optical_frame",
            "image_topic": "/correction_service/image_raw",
            "max_capture_age_ms": 200.0,
        }
        values = {key: node.declare_parameter(key, value).value for key, value in defaults.items()}
        width, height = values["image_width"], values["image_height"]
        max_age = float(values["max_capture_age_ms"]) * 1_000_000
        if min(width, height, values["framerate"], max_age) <= 0:
            raise ValueError("invalid UVC dimensions, fps or capture-age limit")
        if node.get_parameter("use_sim_time").value:
            raise ValueError("UVC hardware capture requires the system ROS clock")
        publisher = node.create_publisher(
            Image, values["image_topic"],
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT),
        )
        capture = UvcCapture(values["video_device"], width, height, values["framerate"])
        node.get_logger().info(
            f"UVC mmap started: {values['video_device']} {width}x{height} MJPEG "
            f"requested_fps={values['framerate']} negotiated_fps={capture.negotiated_fps} "
            "timestamp=V4L2_CLOCK_MONOTONIC decoder=OpenCV"
        )
        count = stale = nonmonotonic = 0
        age_sum = age_max = 0.0
        last_stamp = 0
        report_time = last_frame = time.monotonic()
        while rclpy.ok():
            sample = capture.read_latest()
            current = time.monotonic()
            if sample is None:
                if current - last_frame > 3.0:
                    raise RuntimeError("UVC stream produced no complete frame for 3 seconds")
                continue
            last_frame = current
            payload, buffer = sample
            mono_now = time.monotonic_ns()
            stamp, age = capture_stamp_ns(buffer, mono_now, node.get_clock().now().nanoseconds)
            if age > max_age:
                stale += 1
            elif stamp <= last_stamp:
                nonmonotonic += 1
            else:
                frame = cv2.imdecode(np.frombuffer(payload, dtype=np.uint8), cv2.IMREAD_GRAYSCALE)
                if frame is None or frame.shape != (height, width):
                    raise RuntimeError("UVC MJPEG decoder returned an invalid image/size")
                # 解码开销也计入帧龄，不允许积压图像越过原有 200 ms 门。
                age += time.monotonic_ns() - mono_now
                if age > max_age:
                    stale += 1
                else:
                    message = Image()
                    message.header.stamp = Time(nanoseconds=stamp).to_msg()
                    message.header.frame_id = values["frame_id"]
                    message.height, message.width = height, width
                    message.encoding, message.step = "mono8", width
                    message.data = frame.tobytes()
                    publisher.publish(message)
                    last_stamp = stamp
                    count += 1
                    age_sum += age / 1_000_000
                    age_max = max(age_max, age / 1_000_000)
            if current - report_time >= 5.0:
                node.get_logger().info(
                    f"CAMERA_HEALTH rate={count / (current - report_time):.2f}Hz "
                    f"capture_age_mean={age_sum / max(count, 1):.2f}ms "
                    f"capture_age_max={age_max:.2f}ms stale_dropped={stale} "
                    f"nonmonotonic_dropped={nonmonotonic}"
                )
                count, age_sum, age_max, report_time = 0, 0.0, 0.0, current
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        node.get_logger().fatal(str(exc))
        raise
    finally:
        # ros2 run 和进程组可能重复转发 SIGINT，清理期间避免再次打断 STREAMOFF。
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        try:
            if capture is not None:
                capture.close()
        finally:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
