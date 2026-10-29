from __future__ import annotations

from dataclasses import dataclass
from collections import deque
import binascii
import ipaddress
import math
import os
import socket
import subprocess
import struct
import threading
import time
from typing import Iterable, List, Optional

try:
    from . import native_fastpath as _native_fastpath
except Exception:
    try:
        import native_fastpath as _native_fastpath
    except Exception:
        _native_fastpath = None

SOF = b"\x55\xAA"
VERSION = 4
MAX_PAYLOAD = 140000

CMD_PING = 0x01
CMD_LOAD_ISING = 0x10
CMD_LOAD_H_ONLY = 0x11
CMD_LOAD_START = 0x13
CMD_LOAD_H_START = 0x14
CMD_START = 0x20
CMD_STATUS = 0x21
CMD_ABORT = 0x22
CMD_GET_RESULT = 0x23
CMD_SOFT_RESET = 0x24
# UAV compact protocol: active-N coefficient payload + solve in one request.
CMD_UAV_LOAD_START = 0x30
CMD_UAV_H_START = 0x31
CMD_ERROR = 0xFF

FORMAT_Q9_22 = 1
FORMAT_BFP16_J = 2
FORMAT_BFP8_J = 3
# Compact upper-triangle formats carry only active-N h/J coefficients.
FORMAT_UAV_BFP8_J = 4
FORMAT_UAV_BFP16_J = 5
FORMAT_UAV_Q9_22 = 6
FORMAT_UAV_SPARSE_BFP16_J = 7
FLAG_UPPER_TRIANGLE = 1

ERROR_TEXT = {
    1: "协议版本不匹配",
    2: "帧长度错误",
    3: "CRC16校验错误",
    4: "命令不支持",
    5: "问题格式错误",
    6: "问题尚未配置",
    7: "求解器忙",
    8: "求解超时",
    9: "FPGA系数配置失败",
    10: "缓冲区不足",
    11: "PL/AXI求解器不可访问，请检查bitstream与XSA",
}


class FpgaApplicationError(RuntimeError):
    """A valid FPGA response reporting a protocol/service error.

    This is not a transport failure. Retrying the same semantic request over
    broadcast/UART only adds latency and can hide the real board-side error.
    """

    def __init__(self, code: int, command: int = 0, reported_length: int | None = None):
        self.code = int(code) & 0xFF
        self.command = int(command) & 0xFF
        self.reported_length = reported_length
        text = ERROR_TEXT.get(self.code, f"FPGA错误{self.code}")
        detail = f"FPGA返回错误: {text} (code={self.code}, cmd=0x{self.command:02X}"
        if reported_length is not None:
            detail += f", payload={reported_length} B"
        detail += ")"
        super().__init__(detail)


def _raise_fpga_error(payload: bytes) -> None:
    command = payload[0] if len(payload) > 0 else 0
    code = payload[1] if len(payload) > 1 else 0
    reported_length = struct.unpack_from("<I", payload, 2)[0] if len(payload) >= 6 else None
    raise FpgaApplicationError(code, command, reported_length)


def crc16_ccitt(data: bytes, initial: int = 0xFFFF) -> int:
    """CRC-16/CCITT-FALSE using CPython's C implementation."""
    return binascii.crc_hqx(data, initial) & 0xFFFF


def build_frame(
    cmd: int,
    seq: int,
    payload: bytes = b"",
    *,
    trusted_udp: bool = False,
) -> bytes:
    """Build an inner Q128 frame.

    FAST_REQUEST packets are handled by ``q128_handle_frame_trusted_udp`` on
    the board, where the inner CRC16 is intentionally not rescanned.  Writing a
    zero placeholder avoids an otherwise redundant full-payload CRC pass on the
    host for latency-critical LOAD/START requests.  UART and normal UDP retain
    the original CRC16 behavior.
    """
    if len(payload) > MAX_PAYLOAD:
        raise ValueError(f"payload过大: {len(payload)}")
    header = SOF + bytes((VERSION, cmd & 0xFF)) + struct.pack(
        "<HI", seq & 0xFFFF, len(payload)
    )
    body = header + payload
    crc = 0 if trusted_udp else crc16_ccitt(body)
    return body + struct.pack("<H", crc)


@dataclass(frozen=True)
class Frame:
    version: int
    cmd: int
    seq: int
    payload: bytes
    raw: bytes


def parse_frame(raw: bytes) -> Frame:
    if len(raw) < 12 or raw[:2] != SOF:
        raise ValueError("非法帧头/长度")
    version, cmd = raw[2], raw[3]
    seq, length = struct.unpack_from("<HI", raw, 4)
    if len(raw) != 12 + length:
        raise ValueError("帧总长度不匹配")
    expected_crc = struct.unpack_from("<H", raw, len(raw) - 2)[0]
    if crc16_ccitt(raw[:-2]) != expected_crc:
        raise ValueError("CRC16错误")
    return Frame(version, cmd, seq, raw[10:-2], raw)


class FrameParser:
    def __init__(self) -> None:
        self.buffer = bytearray()

    def feed(self, data: bytes) -> List[Frame]:
        self.buffer.extend(data)
        frames: List[Frame] = []
        while True:
            start = self.buffer.find(SOF)
            if start < 0:
                if len(self.buffer) > 1:
                    del self.buffer[:-1]
                break
            if start:
                del self.buffer[:start]
            if len(self.buffer) < 12:
                break
            length = struct.unpack_from("<I", self.buffer, 6)[0]
            total = 12 + length
            if length > MAX_PAYLOAD:
                del self.buffer[0]
                continue
            if len(self.buffer) < total:
                break
            raw = bytes(self.buffer[:total])
            del self.buffer[:total]
            try:
                frames.append(parse_frame(raw))
            except ValueError:
                continue
        return frames


def hex_bytes(data: Iterable[int] | bytes, limit: int = 128) -> str:
    raw = bytes(data)
    text = " ".join(f"{value:02X}" for value in raw[:limit])
    return text + (f" ... ({len(raw)} bytes)" if len(raw) > limit else "")


UDP_MAGIC = b"Q128"
UDP_VERSION = 1
UDP_REQUEST = 1
UDP_RESPONSE = 2
UDP_FAST_REQUEST = 3
# WSL2/NAT-safe application fragment payload.
# Wire size at the IPv4 layer is:
#   IPv4(20) + UDP(8) + Q128 UDP header(24) + UDP_DATA(1280) = 1332 bytes.
# Keeping this comfortably below the observed WSL path MTU avoids a second
# layer of IP fragmentation on top of the Q128 application fragmentation.
UDP_DATA = 1280
UDP_HEADER = struct.Struct("<4sBBIHHHII")

DEFAULT_FPGA_IP = os.environ.get("FPGA_IP", "192.168.2.10")
DEFAULT_HOST_IP = os.environ.get("FPGA_BIND_IP", "192.168.2.100")
DEFAULT_UDP_PORT = int(os.environ.get("FPGA_PORT", "4000"))


def _udp_packet(
    flag: int,
    session: int,
    index: int,
    count: int,
    total: int,
    payload: bytes,
) -> bytes:
    crc = 0 if flag == UDP_FAST_REQUEST else (binascii.crc32(payload) & 0xFFFFFFFF)
    return (
        UDP_HEADER.pack(
            UDP_MAGIC,
            UDP_VERSION,
            flag,
            session,
            index,
            count,
            len(payload),
            total,
            crc,
        )
        + payload
    )


def _udp_unpack(data: bytes):
    if len(data) < UDP_HEADER.size:
        raise ValueError("UDP短包")
    magic, version, flag, session, index, count, dlen, total, crc = UDP_HEADER.unpack_from(data)
    payload = data[UDP_HEADER.size :]
    if (
        magic != UDP_MAGIC
        or version != UDP_VERSION
        or flag not in (UDP_REQUEST, UDP_RESPONSE, UDP_FAST_REQUEST)
        or count == 0
        or count > 128
        or index >= count
        or dlen > UDP_DATA
        or total > MAX_PAYLOAD + 12
        or index * UDP_DATA + dlen > total
        or len(payload) != dlen
        or (
            flag != UDP_FAST_REQUEST
            and (binascii.crc32(payload) & 0xFFFFFFFF) != crc
        )
    ):
        raise ValueError("UDP包头/CRC错误")
    return flag, session, index, count, total, payload


def _broadcast_address(bind_ip: str) -> Optional[str]:
    try:
        ip = ipaddress.IPv4Address(bind_ip)
    except ipaddress.AddressValueError:
        return None
    if ip.is_loopback or ip.is_unspecified:
        return None
    return str(ipaddress.IPv4Network(f"{bind_ip}/24", strict=False).broadcast_address)


def _windows_arp_hint(fpga_ip: str) -> str:
    """Return a concise ARP hint without changing network state."""
    if os.name != "nt":
        return ""
    try:
        completed = subprocess.run(
            ["arp", "-a"],
            capture_output=True,
            text=True,
            errors="replace",
            timeout=1.5,
            creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0),
        )
        table = completed.stdout or ""
    except Exception:
        return ""

    if fpga_ip in table:
        return f"；ARP表中存在 {fpga_ip}，应继续检查板端UDP 4000和协议响应"
    return (
        f"；ARP表中不存在 {fpga_ip}，板端尚未响应ARP，"
        "优先检查双通道ELF是否已Resume、Platform/XSA和GEM0初始化"
    )


class UdpSession:
    """Q128 UDP session with unicast first and /24 broadcast fallback."""

    last_transport = "UDP"

    def __init__(
        self,
        fpga_ip: str = DEFAULT_FPGA_IP,
        port: int = DEFAULT_UDP_PORT,
        bind_ip: str = DEFAULT_HOST_IP,
        broadcast_fallback: bool = True,
        spin_wait_us: int = 1500,
        adaptive_spin: bool = False,
        spin_wait_min_us: int = 1500,
        blocking_poll_us: int = 1000,
        incoming_cpu: int = -1,
        kernel_busy_poll_us: int = 0,
        native_receive: bool = False,
    ):
        self.addr = (fpga_ip, int(port))
        self.bind_ip = bind_ip
        self.broadcast_fallback = bool(broadcast_fallback)
        self.seq = 0
        self.lock = threading.Lock()
        self.last_attempts: list[str] = []
        self._sock: Optional[socket.socket] = None
        self._connected_udp = False
        self._io_timeout_s = 0.005
        self.blocking_poll_us = max(250, min(5000, int(blocking_poll_us)))
        self._connected_blocking_poll_s = self.blocking_poll_us * 1.0e-6
        self.incoming_cpu = int(incoming_cpu)
        self.kernel_busy_poll_us = max(0, min(500, int(kernel_busy_poll_us)))
        self.incoming_cpu_applied = False
        self.kernel_busy_poll_applied = False
        self.native_receive_requested = bool(native_receive)
        self.native_receive_applied = False
        self.native_receive_version = 0
        self.native_receive_fallbacks = 0
        self.spin_wait_us = max(0, min(5000, int(spin_wait_us)))
        self.adaptive_spin = bool(adaptive_spin)
        self.spin_wait_min_us = max(0, min(self.spin_wait_us, int(spin_wait_min_us)))
        self._rtt_history_us: deque[float] = deque(maxlen=64)
        self.last_spin_hit = False
        self.last_spin_elapsed_us = 0.0
        self.last_spin_budget_us = float(self.spin_wait_us)
        self.last_receive_phase = "none"

    def _effective_spin_wait_us(self) -> int:
        """Return a bounded receive-spin budget from recent transaction RTTs."""
        cap = int(self.spin_wait_us)
        if cap <= 0:
            return 0
        if not self.adaptive_spin or len(self._rtt_history_us) < 8:
            return cap
        ordered = sorted(self._rtt_history_us)
        index = int(math.ceil(0.95 * (len(ordered) - 1)))
        target = int(math.ceil(ordered[index] + 150.0))
        return max(int(self.spin_wait_min_us), min(cap, target))

    def _record_rtt_us(self, elapsed_us: float) -> None:
        if math.isfinite(float(elapsed_us)) and elapsed_us > 0.0:
            self._rtt_history_us.append(float(elapsed_us))

    @property
    def is_open(self) -> bool:
        return True

    def close(self) -> None:
        sock, self._sock = self._sock, None
        self._connected_udp = False
        if sock is not None:
            try:
                sock.close()
            except OSError:
                pass

    def _socket(self) -> socket.socket:
        if self._sock is not None:
            return self._sock
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1 << 20)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 1 << 20)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        try:
            sock.setsockopt(socket.IPPROTO_IP, socket.IP_TOS, 0x10)
        except OSError:
            pass
        if self.incoming_cpu >= 0:
            try:
                so_incoming_cpu = getattr(socket, "SO_INCOMING_CPU", 49)
                sock.setsockopt(socket.SOL_SOCKET, so_incoming_cpu, int(self.incoming_cpu))
                self.incoming_cpu_applied = True
            except OSError:
                self.incoming_cpu_applied = False
        if self.kernel_busy_poll_us > 0:
            try:
                so_busy_poll = getattr(socket, "SO_BUSY_POLL", 46)
                sock.setsockopt(socket.SOL_SOCKET, so_busy_poll, int(self.kernel_busy_poll_us))
                self.kernel_busy_poll_applied = True
            except OSError:
                self.kernel_busy_poll_applied = False
        try:
            sock.bind((self.bind_ip, 0))
        except OSError as exc:
            sock.close()
            raise OSError(
                f"上位机无法绑定本地IP {self.bind_ip}；"
                "请确认有线网卡已配置该地址且处于启用状态"
            ) from exc

        broadcast = _broadcast_address(self.bind_ip) if self.broadcast_fallback else None
        self._connected_udp = False
        if not broadcast:
            try:
                sock.connect(self.addr)
                self._connected_udp = True
            except OSError:
                self._connected_udp = False
        sock.settimeout(self._io_timeout_s)
        self.native_receive_applied = False
        self.native_receive_version = 0
        if self.native_receive_requested and self._connected_udp and _native_fastpath is not None:
            try:
                st = _native_fastpath.status()
                self.native_receive_version = int(st.get("version", 0) or 0)
                self.native_receive_applied = bool(
                    st.get("available", False) and self.native_receive_version >= 2701
                )
            except Exception:
                self.native_receive_applied = False
        self._sock = sock
        return sock

    def _destinations(self, retries: int) -> list[tuple[str, int, str]]:
        count = max(1, int(retries))
        destinations: list[tuple[str, int, str]] = [
            (self.addr[0], self.addr[1], f"unicast#{attempt + 1}")
            for attempt in range(count)
        ]
        if self.broadcast_fallback:
            broadcast = _broadcast_address(self.bind_ip)
            if broadcast and broadcast != self.addr[0]:
                destinations.append((broadcast, self.addr[1], "broadcast"))
        return destinations

    def transact(
        self,
        cmd: int,
        payload: bytes = b"",
        timeout: float = 2.0,
        retries: int = 1,
    ) -> tuple[bytes, float]:
        with self.lock:
            self.seq = (self.seq + 1) & 0xFFFF
            seq = self.seq
            request_flag = (
                UDP_FAST_REQUEST
                if cmd in (
                    CMD_LOAD_ISING,
                    CMD_LOAD_H_ONLY,
                    CMD_LOAD_START,
                    CMD_LOAD_H_START,
                    CMD_UAV_LOAD_START,
                    CMD_UAV_H_START,
                )
                else UDP_REQUEST
            )
            frame = build_frame(
                cmd,
                seq,
                payload,
                trusted_udp=(request_flag == UDP_FAST_REQUEST),
            )
            session = (int(time.time_ns()) ^ seq) & 0xFFFFFFFF
            count = math.ceil(len(frame) / UDP_DATA)

            errors: list[str] = []
            self.last_attempts = []
            self.last_spin_hit = False
            self.last_spin_elapsed_us = 0.0
            self.last_spin_budget_us = float(self._effective_spin_wait_us())
            self.last_receive_phase = "none"
            for dest_ip, dest_port, label in self._destinations(retries):
                self.last_attempts.append(label)
                sock = self._socket()
                try:
                    started = time.perf_counter()
                    for index in range(count):
                        begin = index * UDP_DATA
                        part = frame[begin : begin + UDP_DATA]
                        packet = _udp_packet(
                            request_flag,
                            session,
                            index,
                            count,
                            len(frame),
                            part,
                        )
                        if self._connected_udp and (dest_ip, dest_port) == self.addr:
                            sock.send(packet)
                        else:
                            sock.sendto(packet, (dest_ip, dest_port))

                    chunks: dict[int, bytes] = {}
                    total: Optional[int] = None
                    expected: Optional[int] = None
                    deadline = started + timeout

                    def consume_datagram(data: bytes, source: tuple[str, int]) -> Optional[bytes]:
                        nonlocal total, expected
                        if source[0] != self.addr[0]:
                            return None
                        try:
                            flag, response_session, index, response_count, response_total, part = _udp_unpack(data)
                        except ValueError:
                            return None
                        if flag != UDP_RESPONSE or response_session != session:
                            return None

                        if response_count == 1 and index == 0:
                            raw = part[:response_total]
                        else:
                            chunks[index] = part
                            total = response_total
                            expected = response_count
                            if not expected or len(chunks) != expected:
                                return None
                            try:
                                raw = b"".join(chunks[i] for i in range(expected))[:total]
                            except KeyError:
                                return None

                        response = parse_frame(raw)
                        if response.seq != seq:
                            return None
                        if response.cmd not in (CMD_ERROR, cmd | 0x80):
                            return None
                        if response.cmd == CMD_ERROR:
                            _raise_fpga_error(response.payload)
                        return response.payload

                    use_native_receive = bool(
                        self.native_receive_applied
                        and self._connected_udp
                        and (dest_ip, dest_port) == self.addr
                        and _native_fastpath is not None
                    )
                    if use_native_receive:
                        native_first = True
                        native_failed = False
                        while time.perf_counter() < deadline:
                            remaining_us = max(1, int((deadline - time.perf_counter()) * 1.0e6))
                            try:
                                native_result = _native_fastpath.udp_recv_native(
                                    sock.fileno(),
                                    2048,
                                    int(self.last_spin_budget_us) if native_first else 0,
                                    int(self.blocking_poll_us),
                                    remaining_us,
                                )
                            except Exception:
                                self.native_receive_fallbacks += 1
                                native_failed = True
                                break
                            if native_result is None:
                                errors.append(f"{label}: timeout")
                                native_failed = False
                                break
                            data, native_phase, native_elapsed_us, native_spin_elapsed_us = native_result
                            if native_first:
                                self.last_spin_elapsed_us = float(native_spin_elapsed_us)
                            native_first = False
                            response_payload = consume_datagram(data, self.addr)
                            if response_payload is None:
                                continue
                            self.last_spin_hit = native_phase == "spin"
                            self.last_receive_phase = native_phase
                            self.last_transport = "UDP"
                            elapsed_ms = (time.perf_counter() - started) * 1000.0
                            self._record_rtt_us(elapsed_ms * 1000.0)
                            return response_payload, elapsed_ms
                        if not native_failed:
                            # Native helper consumed the transaction deadline.
                            continue

                    use_spin = (
                        self._connected_udp
                        and (dest_ip, dest_port) == self.addr
                        and self.last_spin_budget_us > 0
                    )
                    if use_spin:
                        spin_started = time.perf_counter()
                        spin_deadline = min(
                            deadline,
                            spin_started + self.last_spin_budget_us * 1.0e-6,
                        )
                        try:
                            sock.setblocking(False)
                            while time.perf_counter() < spin_deadline:
                                try:
                                    data = sock.recv(2048)
                                except (BlockingIOError, InterruptedError):
                                    continue
                                response_payload = consume_datagram(data, self.addr)
                                if response_payload is None:
                                    continue
                                self.last_spin_hit = True
                                self.last_receive_phase = "spin"
                                self.last_spin_elapsed_us = (
                                    time.perf_counter() - spin_started
                                ) * 1.0e6
                                self.last_transport = "UDP"
                                elapsed_ms = (time.perf_counter() - started) * 1000.0
                                self._record_rtt_us(elapsed_ms * 1000.0)
                                return response_payload, elapsed_ms
                        finally:
                            # Restore the normal bounded blocking mode even when
                            # consume_datagram raises a valid FPGA error.
                            sock.settimeout(self._io_timeout_s)
                        self.last_spin_elapsed_us = (
                            time.perf_counter() - spin_started
                        ) * 1.0e6
                        sock.settimeout(self._connected_blocking_poll_s)

                    while time.perf_counter() < deadline:
                        try:
                            if self._connected_udp:
                                data = sock.recv(2048)
                                source = self.addr
                            else:
                                data, source = sock.recvfrom(2048)
                        except socket.timeout:
                            continue

                        response_payload = consume_datagram(data, source)
                        if response_payload is None:
                            continue
                        self.last_receive_phase = "blocking"
                        self.last_transport = "UDP"
                        elapsed_ms = (time.perf_counter() - started) * 1000.0
                        self._record_rtt_us(elapsed_ms * 1000.0)
                        return response_payload, elapsed_ms

                    errors.append(f"{label}: timeout")
                except FpgaApplicationError:
                    raise
                except Exception as exc:
                    errors.append(f"{label}: {type(exc).__name__}: {exc}")

            details = "; ".join(errors) if errors else "timeout"
            arp_hint = _windows_arp_hint(self.addr[0])
            raise TimeoutError(
                f"UDP通信失败 {self.addr}，本地绑定 {self.bind_ip}："
                f"{details}{arp_hint}"
            )


class SerialSession:
    last_transport = "UART"

    def __init__(self, port: str, baud: int = 921600):
        self.port = port
        self.baud = baud
        self.ser = None
        self.seq = 0
        self.lock = threading.Lock()

    @property
    def is_open(self) -> bool:
        return bool(self.ser and self.ser.is_open)

    def open(self) -> None:
        import serial

        self.ser = serial.Serial(
            self.port,
            self.baud,
            timeout=0.02,
            write_timeout=10,
            inter_byte_timeout=0.10,
        )
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()

    def close(self) -> None:
        if self.ser:
            try:
                self.ser.close()
            except Exception:
                pass
        self.ser = None

    def transact(
        self,
        cmd: int,
        payload: bytes = b"",
        timeout: float = 5.0,
    ) -> tuple[bytes, float]:
        if not self.is_open:
            raise RuntimeError("串口尚未连接")

        with self.lock:
            # Keep a stable object reference for the whole transaction. UI-side
            # reconnect/close must not turn self.ser into None between the
            # is_open check and read().
            ser = self.ser
            if ser is None or not ser.is_open:
                raise RuntimeError("串口尚未连接")
            self.seq = (self.seq + 1) & 0xFFFF
            seq = self.seq
            frame = build_frame(cmd, seq, payload)
            parser = FrameParser()
            ser.reset_input_buffer()
            started = time.perf_counter()

            if ser.write(frame) != len(frame):
                raise IOError("串口写入不完整")
            ser.flush()

            deadline = started + timeout
            while time.perf_counter() < deadline:
                data = ser.read(max(1, min(4096, ser.in_waiting or 1)))
                for response in parser.feed(data):
                    if response.seq != seq:
                        continue
                    if response.cmd not in (CMD_ERROR, cmd | 0x80):
                        continue
                    if response.cmd == CMD_ERROR:
                        _raise_fpga_error(response.payload)
                    self.last_transport = "UART"
                    return response.payload, (time.perf_counter() - started) * 1000.0

            raise TimeoutError(f"串口等待响应超时 {self.port}@{self.baud}")


class DualSession:
    """UDP primary with UART fallback, both served by one PS application."""

    def __init__(
        self,
        udp: UdpSession,
        serial: Optional[SerialSession],
        prefer: str = "udp",
    ):
        self.udp = udp
        self.serial = serial
        self.prefer = prefer.lower()
        self.last_transport = ""
        self.last_errors: dict[str, str] = {}

    @property
    def is_open(self) -> bool:
        return self.udp.is_open or bool(self.serial and self.serial.is_open)

    def close(self) -> None:
        return None

    def _ordered_sessions(self):
        candidates = [("UDP", self.udp), ("UART", self.serial)]
        if self.prefer == "uart":
            candidates.reverse()
        return candidates

    def transact(
        self,
        cmd: int,
        payload: bytes = b"",
        timeout: float = 3.0,
    ) -> tuple[bytes, float]:
        self.last_errors = {}
        for name, session in self._ordered_sessions():
            if session is None:
                self.last_errors[name] = "未配置"
                continue
            if name == "UART" and not session.is_open:
                self.last_errors[name] = "串口未连接"
                continue
            try:
                response, milliseconds = session.transact(
                    cmd,
                    payload,
                    timeout=timeout,
                )
                self.last_transport = name
                return response, milliseconds
            except FpgaApplicationError:
                # UDP/UART terminate in the same firmware service. A semantic
                # FPGA error will be identical on the fallback channel.
                raise
            except Exception as exc:
                self.last_errors[name] = f"{type(exc).__name__}: {exc}"

        details = "; ".join(f"{name}={error}" for name, error in self.last_errors.items())
        raise TimeoutError(f"双通道通信均失败：{details}")
