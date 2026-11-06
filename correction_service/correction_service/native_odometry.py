"""原生400Hz Odin历史适配；仅在需要匹配观测或检查跳变时进入Python。"""

from __future__ import annotations

import uuid
import weakref

from nav_msgs.msg import Odometry
from rclpy.serialization import deserialize_message
from _correction_native import OdometryCapture

from .synchronizer import OdomMatch


class NativeOdometrySynchronizer:
    """保留完整历史、原始接收时刻和异常通知，不产生高频Python回调。"""

    def __init__(self, settings, notify) -> None:
        """复用原同步参数，异常通过现有executor的guard condition处理。"""
        callback_ref = weakref.WeakMethod(notify)

        def notify_fault():
            """避免原生回调持有guard再反向保活整个Python节点。"""
            callback = callback_ref()
            if callback is not None:
                callback()

        self._capture = OdometryCapture(
            settings.odometry_history_seconds,
            settings.max_header_delta_ms,
            settings.max_arrival_delta_ms,
            settings.header_epoch_tolerance_seconds,
            uuid.uuid4().hex[:8], notify_fault,
        )

    def start(self, topic, namespace, domain, use_sim_time) -> None:
        """仅在采样或apply_saved期间启动本任务的原生订阅。"""
        self._capture.start(topic, namespace, domain, use_sim_time)

    def prepare(self, topic, namespace, domain, use_sim_time) -> None:
        """服务启动时预热DDS发现；idle仍不订阅或处理400Hz里程计。"""
        self._capture.prepare(topic, namespace, domain, use_sim_time)

    def close(self) -> None:
        """等待原生回调退出并释放任务订阅和历史，仅保留DDS发现。"""
        self._capture.close()

    def shutdown(self) -> None:
        """服务退出时一并释放原生DDS上下文。"""
        self._capture.shutdown()

    @property
    def running(self) -> bool:
        """返回实际原生接收器是否运行。"""
        return self._capture.running

    def snapshot(self):
        """读取最新有效raw及真实到达时间，不将读取时间当成接收时间。"""
        return self._capture.snapshot()

    def faults(self):
        """取走尚未交给校准状态机的异常。"""
        return self._capture.faults()

    def clear(self) -> None:
        """清除不能跨session复用的匹配历史。"""
        self._capture.clear()

    def match(self, image_stamp_ns):
        """只反序列化最终选中的一条完整里程计，保持原有OdomMatch接口。"""
        result = self._capture.match(image_stamp_ns)
        if result is None:
            return None
        data, delta_ms, time_source = result
        return OdomMatch(deserialize_message(data, Odometry), delta_ms, time_source)
