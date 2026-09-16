"""本服务自有的 Linux UVC/MJPEG mmap 采集；直接保留 V4L2 单调时钟时间戳。"""

from __future__ import annotations

import ctypes as ct
import fcntl
import math
import mmap
import os
import select

# Linux videodev2.h 单平面采集 ABI；使用原生对齐，兼容 amd64/aarch64。
CAPTURE = 1
MMAP = 1
MJPEG = int.from_bytes(b"MJPG", "little")
TIMESTAMP_MASK = 0xE000
TIMESTAMP_MONOTONIC = 0x2000
BUFFER_ERROR = 0x0040


class FormatData(ct.Union):
    """v4l2_format 的 200 字节 union，指针成员决定原生对齐。"""

    _fields_ = [("raw", ct.c_uint8 * 200), ("pix", ct.c_uint32 * 12),
                ("alignment", ct.c_void_p)]


class Format(ct.Structure):
    """v4l2_format。"""

    _fields_ = [("type", ct.c_uint32), ("fmt", FormatData)]


class StreamParm(ct.Structure):
    """v4l2_streamparm，capture.timeperframe 位于 parm[2:4]。"""

    _fields_ = [("type", ct.c_uint32), ("parm", ct.c_uint32 * 50)]


class RequestBuffers(ct.Structure):
    """v4l2_requestbuffers，末四字节为 flags/reserved。"""

    _fields_ = [(name, ct.c_uint32) for name in
                ("count", "type", "memory", "capabilities", "flags_reserved")]


class Timeval(ct.Structure):
    """内核 timeval，不能用 ROS 接收时间冒充采集时间。"""

    _fields_ = [("seconds", ct.c_long), ("microseconds", ct.c_long)]


class BufferMemory(ct.Union):
    """v4l2_buffer.m，mmap 使用 offset。"""

    _fields_ = [("offset", ct.c_uint32), ("userptr", ct.c_ulong),
                ("planes", ct.c_void_p), ("fd", ct.c_int32)]


class Buffer(ct.Structure):
    """v4l2_buffer，保留 timecode 所需的四字节对齐。"""

    _fields_ = [(name, ct.c_uint32) for name in
                ("index", "type", "bytesused", "flags", "field")] + [
        ("timestamp", Timeval), ("timecode", ct.c_uint32 * 4),
        ("sequence", ct.c_uint32), ("memory", ct.c_uint32),
        ("m", BufferMemory), ("length", ct.c_uint32),
        ("reserved2", ct.c_uint32), ("request_fd", ct.c_int32),
    ]


def ioctl_request(number: int, argument: object, direction: int = 3) -> int:
    """Linux _IOWR('V', number, type)；STREAMON/OFF 使用 _IOW。"""
    return (direction << 30) | (ct.sizeof(argument) << 16) | (ord("V") << 8) | number


def _ioctl(fd: int, number: int, argument: object, direction: int = 3) -> None:
    """传递可写 ABI 缓冲区，保留 errno 给调用方处理 EAGAIN/设备错误。"""
    fcntl.ioctl(fd, ioctl_request(number, argument, direction), argument)


def capture_stamp_ns(buffer: Buffer, monotonic_ns: int, ros_ns: int) -> tuple[int, int]:
    """按真实采集帧龄映射到 ROS 时钟，不用声明 fps 推算，也不静默回退。"""
    if buffer.flags & TIMESTAMP_MASK != TIMESTAMP_MONOTONIC:
        raise RuntimeError("UVC driver did not provide a CLOCK_MONOTONIC timestamp")
    stamp = buffer.timestamp.seconds * 1_000_000_000 + buffer.timestamp.microseconds * 1000
    age = monotonic_ns - stamp
    if stamp <= 0 or age < 0:
        raise RuntimeError("UVC driver returned an invalid/future capture timestamp")
    return ros_ns - age, age


class CaptureRateLimiter:
    """按采集单调时钟筛帧；持续排空设备，不休眠、不积压、不改图像时间戳。"""

    def __init__(self, fps: float) -> None:
        if not math.isfinite(fps) or fps < 0:
            raise ValueError("publish_fps must be finite and nonnegative (0 = unlimited)")
        self.period_ns = max(1, round(1_000_000_000 / fps)) if fps else 0
        self.next_stamp_ns: int | None = None

    def accept(self, capture_ns: int) -> bool:
        """固定采集时间节拍消除逐帧取整降频；断流后跳过空档，不补发历史帧。"""
        if not self.period_ns:
            return True
        if self.next_stamp_ns is None:
            self.next_stamp_ns = capture_ns + self.period_ns
            return True
        if capture_ns < self.next_stamp_ns:
            return False
        elapsed_periods = (capture_ns - self.next_stamp_ns) // self.period_ns + 1
        self.next_stamp_ns += elapsed_periods * self.period_ns
        return True


class UvcCapture:
    """拥有单个设备 fd/mmap；初始化失败和正常退出均释放全部采集资源。"""

    def __init__(self, device: str, width: int, height: int, fps: int) -> None:
        self.fd = -1
        self.buffers: list[mmap.mmap] = []
        self.streaming = False
        try:
            self.fd = os.open(device, os.O_RDWR | os.O_NONBLOCK | os.O_CLOEXEC)
            fmt = Format(type=CAPTURE)
            fmt.fmt.pix[:4] = (width, height, MJPEG, 0)
            _ioctl(self.fd, 5, fmt)  # VIDIOC_S_FMT
            if tuple(fmt.fmt.pix[:3]) != (width, height, MJPEG):
                raise RuntimeError("UVC device changed the requested MJPEG size/format")
            parm = StreamParm(type=CAPTURE)
            parm.parm[2:4] = (1, fps)
            _ioctl(self.fd, 22, parm)  # VIDIOC_S_PARM
            self.negotiated_fps = (int(parm.parm[3]), int(parm.parm[2]))
            request = RequestBuffers(count=4, type=CAPTURE, memory=MMAP)
            _ioctl(self.fd, 8, request)  # VIDIOC_REQBUFS
            if request.count < 2:
                raise RuntimeError("UVC device did not allocate enough mmap buffers")
            for index in range(request.count):
                buffer = Buffer(index=index, type=CAPTURE, memory=MMAP)
                _ioctl(self.fd, 9, buffer)  # VIDIOC_QUERYBUF
                self.buffers.append(mmap.mmap(
                    self.fd, buffer.length, flags=mmap.MAP_SHARED,
                    prot=mmap.PROT_READ | mmap.PROT_WRITE, offset=buffer.m.offset,
                ))
                _ioctl(self.fd, 15, buffer)  # VIDIOC_QBUF
            _ioctl(self.fd, 18, ct.c_int(CAPTURE), 1)  # VIDIOC_STREAMON
            self.streaming = True
        except BaseException:
            self.close()
            raise

    def read_latest(self, timeout: float = 0.2) -> tuple[bytes, Buffer] | None:
        """有限排空当前队列，只解码最新完整帧；复制后立即归还内核缓冲区。"""
        if not select.select([self.fd], [], [], timeout)[0]:
            return None
        latest = None
        for _ in self.buffers:
            buffer = Buffer(type=CAPTURE, memory=MMAP)
            try:
                _ioctl(self.fd, 17, buffer)  # VIDIOC_DQBUF
            except BlockingIOError:
                break
            try:
                if buffer.index >= len(self.buffers):
                    raise RuntimeError("UVC driver returned an invalid buffer index")
                storage = self.buffers[buffer.index]
                if buffer.bytesused > len(storage):
                    raise RuntimeError("UVC payload exceeds mmap buffer")
                if buffer.bytesused and not buffer.flags & BUFFER_ERROR:
                    # QBUF may overwrite metadata, so keep an independent copy.
                    latest = (storage[:buffer.bytesused], Buffer.from_buffer_copy(buffer))
            finally:
                _ioctl(self.fd, 15, buffer)
        return latest

    def close(self) -> None:
        """关闭 fd 也会释放内核队列；STREAMOFF 失败时仍须完成其余清理。"""
        try:
            if self.streaming:
                _ioctl(self.fd, 19, ct.c_int(CAPTURE), 1)
        finally:
            self.streaming = False
            for storage in self.buffers:
                storage.close()
            self.buffers.clear()
            if self.fd >= 0:
                os.close(self.fd)
                self.fd = -1
