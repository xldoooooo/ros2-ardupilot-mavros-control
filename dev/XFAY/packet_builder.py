import struct
from protocol_defs import *
from crc16 import calculate_crc16


def build_send_packet(
    cmd_value: int,
    trig_cnt: int,
    fl_sens: int,
    axis_go_zero: list,
    axis_wk_mode: list,
    axis_op_type: list,
    axis_op_value: list,
    uav_valid: int,
    uav_angle: list,
    uav_accel: list,
    cam_vert_fov1x: int,
    cam_zoom_value: int,
    target_angle: list
) -> bytes:
    """
    组装上位机发送数据包
    :return: 完整字节流（含CRC）
    """
    packet = bytearray(SEND_PACKET_LEN)
    offset = 0

    # 1. 帧头
    packet[offset:offset+2] = SYNC_SEND
    offset += 2

    # 2. cmd字节：低3位trig，高5位命令码
    cmd_byte = ((cmd_value & 0x1F) << 3) | (trig_cnt & 0x07)
    packet[offset] = cmd_byte
    offset += 1

    # 3. aux字节：高3位保留0，低5位FPV灵敏度
    aux_byte = fl_sens & 0x1F
    packet[offset] = aux_byte
    offset += 1

    # 4. 三轴控制组
    for i in range(3):
        ctrl_byte = ((axis_op_type[i] & 0x03) << 6) | \
                    ((axis_wk_mode[i] & 0x03) << 4) | \
                    ((axis_go_zero[i] & 0x01) << 3)
        packet[offset] = ctrl_byte
        offset += 1
        # op_value：int16 小端序
        packet[offset:offset+2] = struct.pack('<h', axis_op_value[i])
        offset += 2

    # 5. 载机数据
    uav_ctrl_byte = (uav_valid & 0x01) << 7
    packet[offset] = uav_ctrl_byte
    offset += 1
    # 姿态角：int16 小端序，单位0.01度
    for angle in uav_angle:
        packet[offset:offset+2] = struct.pack('<h', int(angle * 100))
        offset += 2
    # 加速度：int16 小端序，单位0.01m/s²
    for accel in uav_accel:
        packet[offset:offset+2] = struct.pack('<h', int(accel * 100))
        offset += 2

    # 6. 相机参数（32位位域，小端序）
    cam_val = (cam_vert_fov1x & 0x7F) | \
              ((cam_zoom_value & 0xFFFFFF) << 7)
    packet[offset:offset+4] = struct.pack('<I', cam_val)
    offset += 4

    # 7. 指点平移目标角度：float 小端序
    packet[offset:offset+4] = struct.pack('<f', target_angle[0])
    offset += 4
    packet[offset:offset+4] = struct.pack('<f', target_angle[1])
    offset += 4

    # 8. CRC16校验（大端序）
    crc_val = calculate_crc16(packet[:offset])
    packet[offset] = (crc_val >> 8) & 0xFF
    packet[offset+1] = crc_val & 0xFF

    return bytes(packet)


def parse_response_packet(data: bytes) -> dict:
    """解析云台返回数据包，返回状态字典"""
    status = {}
    offset = 2  # 跳过帧头

    # 基础信息
    status['fw_ver'] = data[offset]
    offset += 1
    status['hw_err'] = data[offset]
    offset += 1

    # 状态字节
    stat_byte = data[offset]
    offset += 1
    status['inv_flag'] = (stat_byte >> 7) & 0x01
    status['gbc_stat'] = (stat_byte >> 4) & 0x07
    status['tca_flag'] = (stat_byte >> 3) & 0x01

    # 命令应答字节
    cmd_byte = data[offset]
    offset += 1
    status['cmd_stat'] = cmd_byte & 0x07
    status['cmd_value'] = (cmd_byte >> 3) & 0x1F

    # 相机本体角速度
    cam_rate = list(struct.unpack('<hhh', data[offset:offset+6]))
    status['cam_rate'] = [r * 0.1 for r in cam_rate]
    offset += 6

    # 相机姿态角
    cam_angle = list(struct.unpack('<hhh', data[offset:offset+6]))
    status['cam_angle'] = [a * 0.01 for a in cam_angle]
    offset += 6

    # 电机相对角
    mtr_angle = list(struct.unpack('<hhh', data[offset:offset+6]))
    status['mtr_angle'] = [a * 0.01 for a in mtr_angle]

    return status