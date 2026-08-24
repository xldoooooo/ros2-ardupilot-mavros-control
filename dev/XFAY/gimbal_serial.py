import serial
import threading
import time
from typing import Optional
from protocol_defs import SYNC_RECV, RECV_PACKET_LEN
from crc16 import calculate_crc16


class GimbalSerial:
    def __init__(self, port: str, baudrate: int = 115200):
        self.port = port
        self.baudrate = baudrate
        self._ser: Optional[serial.Serial] = None

        # 接收状态机
        self._recv_buffer = bytearray()
        self._recv_state = 0
        self._latest_packet: Optional[bytes] = None

        # 线程控制
        self._send_running = False
        self._recv_running = False
        self._send_thread: Optional[threading.Thread] = None
        self._recv_thread: Optional[threading.Thread] = None

        # 外部注入的发送数据生成函数
        self._send_data_generator = None

    def set_send_generator(self, gen_func):
        """设置发送数据生成函数，每次发送前调用获取字节流"""
        self._send_data_generator = gen_func

    def connect(self) -> bool:
        """打开串口并启动收发线程"""
        try:
            self._ser = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                stopbits=serial.STOPBITS_ONE,
                parity=serial.PARITY_NONE,
                timeout=0.1
            )
        except Exception as e:
            print(f"串口打开失败：{e}")
            return False

        # 启动接收线程
        self._recv_running = True
        self._recv_thread = threading.Thread(target=self._recv_loop, daemon=True)
        self._recv_thread.start()

        # 启动发送线程
        self._send_running = True
        self._send_thread = threading.Thread(target=self._send_loop, daemon=True)
        self._send_thread.start()

        print(f"串口 {self.port} 连接成功")
        return True

    def disconnect(self):
        """关闭串口与线程"""
        self._send_running = False
        self._recv_running = False
        if self._send_thread:
            self._send_thread.join(timeout=1)
        if self._recv_thread:
            self._recv_thread.join(timeout=1)
        if self._ser and self._ser.is_open:
            self._ser.close()
        print("串口已关闭")

    def _send_loop(self):
        """50Hz定时发送循环"""
        while self._send_running:
            try:
                if self._send_data_generator:
                    data = self._send_data_generator()
                    self._ser.write(data)
            except Exception as e:
                print(f"发送异常：{e}")
            time.sleep(0.02)

    def _recv_loop(self):
        """接收状态机，处理粘包断包"""
        while self._recv_running:
            try:
                if not self._ser or not self._ser.is_open:
                    break
                byte = self._ser.read(1)
                if not byte:
                    continue

                if self._recv_state == 0:
                    if byte == SYNC_RECV[0:1]:
                        self._recv_buffer.clear()
                        self._recv_buffer.append(byte[0])
                        self._recv_state = 1
                elif self._recv_state == 1:
                    if byte == SYNC_RECV[1:2]:
                        self._recv_buffer.append(byte[0])
                        self._recv_state = 2
                    else:
                        self._recv_state = 0
                elif self._recv_state == 2:
                    self._recv_buffer.append(byte[0])
                    if len(self._recv_buffer) == RECV_PACKET_LEN:
                        # CRC校验
                        crc_calc = calculate_crc16(self._recv_buffer[:-2])
                        crc_recv = (self._recv_buffer[-2] << 8) | self._recv_buffer[-1]
                        if crc_calc == crc_recv:
                            self._latest_packet = bytes(self._recv_buffer)
                        self._recv_state = 0
            except Exception as e:
                print(f"接收异常：{e}")
                self._recv_state = 0

    def get_latest_packet(self) -> Optional[bytes]:
        """获取最新一帧有效数据包"""
        return self._latest_packet