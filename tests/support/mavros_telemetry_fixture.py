"""Exercise MAVROS diagnostics with an unarmed UDP FCU in a localhost-only ROS domain."""
import argparse
import json
import os
from pathlib import Path
import signal
import socket
import statistics
import struct
import subprocess
import time


def packet(message, payload, extra, sequence):
    """Encode the small MAVLink v1 input set without any vehicle command messages."""
    header = bytes((len(payload), sequence % 256, 1, 1, message))
    crc = 65535
    for byte in header + payload + bytes((extra,)):
        temp = byte ^ (crc & 255)
        temp = (temp ^ (temp << 4)) & 255
        crc = ((crc >> 8) ^ (temp << 8) ^ (temp << 3) ^ (temp >> 4)) & 65535
    return b'\xfe' + header + payload + struct.pack('<H', crc)


def cpu_seconds(pid):
    """Read process CPU time without a sampling thread or profiler."""
    fields = Path(f'/proc/{pid}/stat').read_text().split()
    return (int(fields[13]) + int(fields[14])) / os.sysconf('SC_CLK_TCK')


def main():
    """Run one normal/arrival-gap/publish-delay case against an explicit native installation."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--prefix', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--case', choices=('normal', 'gap', 'sleep', 'busy'), default='normal')
    parser.add_argument('--hook', type=Path)
    args = parser.parse_args()
    assert os.environ.get('ROS_AUTOMATIC_DISCOVERY_RANGE') == 'LOCALHOST'
    assert int(os.environ.get('ROS_DOMAIN_ID', '0')) >= 150
    import rclpy
    from geometry_msgs.msg import PoseStamped
    from rclpy.qos import qos_profile_sensor_data

    args.output.mkdir(parents=True, exist_ok=True)
    config = args.output / 'plugins.yaml'
    config.write_text('/**:\n  ros__parameters:\n    plugin_allowlist: [imu, local_position, sys_status]\n')
    env = dict(os.environ, ROS_LOG_DIR=str(args.output / 'ros'))
    prefix = str(args.prefix.resolve())
    env['AMENT_PREFIX_PATH'] = prefix + ':' + env.get('AMENT_PREFIX_PATH', '')
    env['LD_LIBRARY_PATH'] = prefix + '/lib:' + env.get('LD_LIBRARY_PATH', '')
    trigger = args.output / 'trigger'
    if args.case in ('sleep', 'busy'):
        assert args.hook
        env.update(LD_PRELOAD=str(args.hook.resolve()), ONBOARD_TEST_TRIGGER=str(trigger),
                   ONBOARD_TEST_TOPIC='/mavros/local_position/pose', ONBOARD_TEST_DELAY_MS='450',
                   ONBOARD_TEST_DELAY_MODE=args.case)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(('127.0.0.1', 15973))
    process = subprocess.Popen([prefix + '/lib/mavros/mavros_node', '--ros-args',
                                '--params-file', str(config), '-p',
                                'fcu_url:=udp://127.0.0.1:15972@127.0.0.1:15973'],
                               env=env, stdout=(args.output / 'node.log').open('w'),
                               stderr=subprocess.STDOUT)
    rclpy.init()
    node = rclpy.create_node('mavros_diagnostic_udp_fixture')
    received = []
    sub = node.create_subscription(PoseStamped, '/mavros/local_position/pose',
                                   lambda msg: received.append((time.monotonic(), msg.pose.position.z)),
                                   qos_profile_sensor_data)
    sequence = tick = 0
    begin = next_tick = time.monotonic()
    measure_at = cpu_start = None
    try:
        while time.monotonic() - begin < 10:
            now = time.monotonic()
            elapsed = now - begin
            assert process.poll() is None, (args.output / 'node.log').read_text()
            if measure_at is None and elapsed >= 2:
                measure_at, cpu_start = now, cpu_seconds(process.pid)
                received.clear()
            if elapsed >= 4 and args.case in ('sleep', 'busy') and not trigger.exists():
                trigger.touch()
            if now >= next_tick:
                boot = int(elapsed * 1000)
                data = [(30, struct.pack('<I6f', boot, 0, 0, 0, 0, 0, 0), 39)]
                if tick % 4 == 0 and not (args.case == 'gap' and 4 <= elapsed < 4.35):
                    data.append((32, struct.pack('<I6f', boot, 0, 0, -1, 0, 0, 0), 185))
                if tick % 400 == 0:
                    data.append((0, struct.pack('<IBBBBB', 0, 2, 3, 0, 3, 3), 50))
                for msg, payload, extra in data:
                    sock.sendto(packet(msg, payload, extra, sequence), ('127.0.0.1', 15972))
                    sequence += 1
                tick += 1
                next_tick += .0025
                if now - next_tick > .01:
                    next_tick = now + .0025
            rclpy.spin_once(node, timeout_sec=max(0, min(.001, next_tick-time.monotonic())))
        duration = time.monotonic() - measure_at
        cpu = cpu_seconds(process.pid) - cpu_start
        gaps = sorted((b[0] - a[0]) * 1000 for a, b in zip(received, received[1:]))
        assert len(received) > 650, len(received)
        assert all(abs(z - 1) < 1e-6 for _, z in received)
        diagnostics = list((args.output / 'ros').glob('mavros-telemetry-*.log'))
        text = diagnostics[0].read_text() if diagnostics else ''
        if args.case == 'gap':
            assert '[MAVROS_EVENT] stage=position_receive' in text, text
        elif args.case in ('sleep', 'busy'):
            records = [dict(x.split('=', 1) for x in line.split() if '=' in x)
                       for line in text.splitlines() if '[MAVROS_EVENT] stage=pose_publish' in line]
            assert records and max(float(r['duration_ms']) for r in records) >= 440, text
            slow = max(records, key=lambda r: float(r['duration_ms']))
            assert float(slow['cpu_ms']) > 300 if args.case == 'busy' else float(slow['cpu_ms']) < 80
        result = dict(case=args.case, cpu_percent_one_core=cpu/duration*100,
                      samples=len(received), p99_ms=gaps[int(.99*(len(gaps)-1))],
                      max_ms=max(gaps), median_ms=statistics.median(gaps), diagnostic_bytes=len(text))
        (args.output / 'result.json').write_text(json.dumps(result, indent=2))
        print(json.dumps(result), flush=True)
    finally:
        process.send_signal(signal.SIGINT)
        try:
            process.wait(timeout=8)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait()
        node.destroy_node()
        rclpy.shutdown()
        sock.close()


if __name__ == '__main__':
    main()
