"""Verify public forwarding ownership, retry behavior and real RTSP packet copying."""

import json
import shutil
import socket
import subprocess
import time

import pytest

from video_service import public_stream as module
from video_service.public_stream import PublicStreamRelay


def test_ready_sources_retry_without_forwarding_loop(monkeypatch):
    """Unavailable WAN does not block local monitoring and never includes the 24-hour loop."""
    processes = []
    now = [10.0]
    monkeypatch.setattr(module.time, 'monotonic', lambda: now[0])

    class Process:
        """Minimal owned process used to drive failure and source-loss transitions."""

        def __init__(self, command, **kwargs):
            """Capture commands without opening any network connection."""
            self.command, self.code = command, None
            processes.append(self)

        def poll(self):
            """Return the simulated child state."""
            return self.code

        def terminate(self):
            """Complete shutdown immediately for this lifecycle check."""
            self.code = -15

        def wait(self, timeout=None):
            """Report the already collected process state."""
            return self.code

    monkeypatch.setattr(module.subprocess, 'Popen', Process)
    relay = PublicStreamRelay({'rtsp_port': 9854, 'public_stream': {'enabled': True, 'streams': {
        'uav002': 'rtsp://localhost/live/test?secret=private',
        'loop': 'rtsp://localhost/live/loop',
    }}})
    try:
        relay.sync({'loop/rectified': {}})
        assert not processes
        relay.sync({'uav002/rectified': {}})
        assert len(processes) == 1
        command = processes[0].command
        assert command[command.index('-c:v') + 1] == 'copy'
        assert command[command.index('-i') + 1] == 'rtsp://127.0.0.1:9854/uav002/rectified'
        assert 'private' not in str(relay.status())
        processes[0].code = 1
        relay.sync({'uav002/rectified': {}})
        assert relay.status()['uav002']['state'] == 'retrying'
        now[0] += module.RETRY_SECONDS
        relay.sync({'uav002/rectified': {}})
        assert len(processes) == 2
        relay.sync({})
        assert processes[1].code == -15
        relay.sync({})
        assert relay.status()['uav002']['state'] == 'idle'
    finally:
        relay.close()


def _port():
    """Reserve an available loopback port for the isolated media server."""
    with socket.socket() as sock:
        sock.bind(('127.0.0.1', 0))
        return sock.getsockname()[1]


@pytest.mark.integration
@pytest.mark.skipif(not shutil.which('mediamtx') or not shutil.which('ffmpeg'), reason='Requires MediaMTX and FFmpeg')
def test_real_public_copy_and_source_stop(tmp_path):
    """Decode forwarded H.264 through isolated RTSP paths, then stop only the relay child."""
    port = _port()
    config = tmp_path / 'mediamtx.json'
    config.write_text(json.dumps(dict(logLevel='error', rtspAddress=f'127.0.0.1:{port}',
                                     rtspTransports=['tcp'], rtmp=False, hls=False,
                                     webrtc=False, srt=False, api=False,
                                     paths={'all_others': {}})))
    server = subprocess.Popen([shutil.which('mediamtx'), str(config)], stdout=subprocess.DEVNULL,
                              stderr=subprocess.PIPE)
    publisher = None
    relay = PublicStreamRelay({'rtsp_port': port, 'public_stream': {'enabled': True, 'streams': {
        'uav002': f'rtsp://127.0.0.1:{port}/live/test',
    }}})
    try:
        deadline = time.monotonic() + 5
        while time.monotonic() < deadline:
            try:
                with socket.create_connection(('127.0.0.1', port), timeout=0.1):
                    break
            except OSError:
                assert server.poll() is None, server.stderr.read().decode(errors='replace')
                time.sleep(0.05)
        publisher = subprocess.Popen([
            'ffmpeg', '-hide_banner', '-loglevel', 'error', '-nostdin', '-re', '-f', 'lavfi',
            '-i', 'testsrc2=size=128x72:rate=30', '-t', '20', '-an', '-c:v', 'libx264',
            '-preset', 'ultrafast', '-tune', 'zerolatency', '-threads', '1', '-g', '30',
            '-f', 'rtsp', '-rtsp_transport', 'tcp', f'rtsp://127.0.0.1:{port}/uav002/rectified',
        ], stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)
        deadline = time.monotonic() + 10
        while time.monotonic() < deadline:
            relay.sync({'uav002/rectified': {}})
            if relay.status()['uav002']['state'] == 'streaming':
                break
            assert publisher.poll() is None
            time.sleep(0.1)
        assert relay.status()['uav002']['state'] == 'streaming', relay._jobs['uav002']['progress'].read_text()
        result = subprocess.run([
            'ffmpeg', '-hide_banner', '-loglevel', 'error', '-nostdin',
            '-rtsp_transport', 'tcp', '-timeout', '5000000', '-i', f'rtsp://127.0.0.1:{port}/live/test',
            '-frames:v', '30', '-f', 'null', '-',
        ], capture_output=True, timeout=10)
        assert result.returncode == 0, result.stderr.decode(errors='replace')
        relay.sync({})
        deadline = time.monotonic() + 5
        while relay.status()['uav002']['state'] != 'idle' and time.monotonic() < deadline:
            relay.sync({})
            time.sleep(0.05)
        assert relay.status()['uav002']['state'] == 'idle'
        assert publisher.poll() is None
    finally:
        relay.close()
        for process in (publisher, server):
            if process is not None:
                if process.poll() is None:
                    process.terminate()
                try:
                    process.communicate(timeout=3)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.communicate()
