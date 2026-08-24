def calculate_crc16(data: bytes) -> int:
    """
    云台协议专用CRC16校验（半字节查表法）
    :param data: 待校验字节流
    :return: 16位CRC校验值
    """
    crc = 0x0000
    crc_table = [
        0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50a5, 0x60c6, 0x70e7,
        0x8108, 0x9129, 0xa14a, 0xb16b, 0xc18c, 0xd1ad, 0xe1ce, 0xf1ef
    ]
    for byte in data:
        da = (crc >> 12) & 0x0F
        crc = ((crc << 4) & 0xFFFF) ^ crc_table[da ^ (byte >> 4)]
        da = (crc >> 12) & 0x0F
        crc = ((crc << 4) & 0xFFFF) ^ crc_table[da ^ (byte & 0x0F)]
    return crc & 0xFFFF