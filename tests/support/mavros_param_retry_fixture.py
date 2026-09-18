"""UDP-only MAVROS 2.15.1 regression fixture; never opens aircraft serial hardware.

Run a separate MAVROS in domain 229 with LOCALHOST discovery and FCU URL
udp://127.0.0.1:15962@127.0.0.1:15963, then run this using the project Python.
--case burst checks duplicate amplification and recovery; stall checks that a
continuous duplicate stream cannot postpone retry exhaustion. Check MAVROS logs
for list completion (burst) or the missing-parameter error (stall).
"""
import argparse
import json
import socket
import struct
import time

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--case', choices=('burst', 'stall'), default='burst')
parser.add_argument('--baseline', action='store_true', help='Expect the unfixed defect')
args = parser.parse_args()
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(('127.0.0.1', 15963))
sock.settimeout(.02)
sequence = 0


def send(message, payload, extra):
    """Encode MAVLink v1 with its X25 CRC for the isolated virtual FCU only."""
    global sequence
    header = bytes((len(payload), sequence, 1, 1, message))
    sequence = (sequence + 1) % 256
    crc = 65535
    for byte in header + payload + bytes((extra,)):
        temp = byte ^ (crc & 255)
        temp = (temp ^ (temp << 4)) & 255
        crc = ((crc >> 8) ^ (temp << 8) ^ (temp << 3) ^ (temp >> 4)) & 65535
    sock.sendto(b'\xfe' + header + payload + struct.pack('<H', crc),
                ('127.0.0.1', 15962))


def param(index):
    """A three-parameter table deliberately omits index 1 until recovery."""
    send(22, struct.pack('<fHH16sB', float(index), 3, index,
                         ('TEST_%d' % index).encode(), 9), 220)


start = time.monotonic()
heartbeat = duplicate = 0
burst_at = None
reads = lists = 0
measurement = None
while time.monotonic() - start < 35:
    now = time.monotonic()
    if now - heartbeat >= .5:
        send(0, struct.pack('<IBBBBB', 0, 2, 3, 0, 3, 3), 50)
        heartbeat = now
    if burst_at and args.case == 'stall' and now - duplicate > .05:
        param(0)
        duplicate = now
    if burst_at and measurement is None and now - burst_at > .5:
        measurement = reads
        if args.case == 'burst':
            param(1)
    if burst_at and now - burst_at > (3 if args.case == 'burst' else 7):
        break
    try:
        packet, _ = sock.recvfrom(65535)
    except socket.timeout:
        continue
    while packet:
        if packet[0] == 253:
            length = 12 + packet[1] + (13 if packet[2] & 1 else 0)
            msg = int.from_bytes(packet[7:10], 'little')
            payload = packet[10:10 + packet[1]]
        elif packet[0] == 254:
            length = 8 + packet[1]
            msg = packet[5]
            payload = packet[6:6 + packet[1]]
        else:
            raise RuntimeError('Invalid MAVLink packet')
        packet = packet[length:]
        if msg == 21:
            lists += 1
            param(0)
            param(2)
        elif msg == 20 and struct.unpack('<h', payload[:2])[0] == 1:
            reads += 1
            if burst_at is None:
                burst_at = time.monotonic()
                for _ in range(30):
                    param(0)
print(json.dumps(dict(case=args.case, lists=lists,
                      reads_during_duplicate_burst=measurement,
                      total_reads=reads)), flush=True)
assert burst_at is not None, 'No parameter retry observed'
assert lists == 1, 'Unexpected parameter list restart'
if args.baseline:
    assert measurement > 1, 'Expected the original retry amplification'
elif args.case == 'burst':
    assert measurement == reads == 1, 'Duplicate replies amplified requests'
else:
    assert reads == 3, 'Duplicate replies postponed or amplified finite retries'
