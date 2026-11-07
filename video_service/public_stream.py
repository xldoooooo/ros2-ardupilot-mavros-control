"""Forward explicitly configured corrected streams without coupling public network and recording."""

import logging
from pathlib import Path
import re
import subprocess
import tempfile
import time
from urllib.parse import urlsplit

# Public forwarding is opt-in for aircraft and simulation; the 24-hour loop stays local.
AIRCRAFT_IDS = frozenset(('uav001', 'uav002', 'simulation'))
# A failed public link retries independently, allowing slow connections before restart.
RETRY_SECONDS = 3.0
STALL_SECONDS = 30.0
# Forwarded packets have no local recording to finalize, so stop stays short.
STOP_SECONDS = 3.0


class PublicStreamRelay:
    """Own one copy-only FFmpeg process per ready, explicitly mapped local RTSP stream."""

    def __init__(self, settings):
        """Read optional public_stream settings without opening any network connection."""
        config = settings.get('public_stream') or {}
        if not isinstance(config, dict):
            logging.warning('Ignoring invalid public_stream configuration')
            config = {}
        self._targets = {}
        self._jobs = {}
        self._closed = False
        self._temporary = tempfile.TemporaryDirectory(prefix='ground-public-stream-')
        self._port = int(settings.get('rtsp_port', 8554))
        self._ffmpeg = settings.get('ffmpeg', 'ffmpeg')
        if config.get('enabled', False):
            streams = config.get('streams') or {}
            if not isinstance(streams, dict):
                logging.warning('Ignoring invalid public_stream.streams configuration')
                streams = {}
            for aircraft, target in streams.items():
                try:
                    parsed = urlsplit(target)
                    valid = aircraft in AIRCRAFT_IDS and parsed.scheme == 'rtsp' and parsed.hostname
                except (TypeError, ValueError):
                    valid = False
                if not valid:
                    logging.warning('Ignoring invalid public stream mapping for %s', aircraft)
                    continue
                self._targets[aircraft] = target

    def _command(self, aircraft, progress):
        """Copy existing H.264 packets; keep authentication solely in private configuration."""
        source = f'rtsp://127.0.0.1:{self._port}/{aircraft}/rectified'
        return [self._ffmpeg, '-hide_banner', '-nostdin', '-loglevel', 'error',
                '-rtsp_transport', 'tcp', '-timeout', '15000000',
                '-analyzeduration', '200000', '-probesize', '5000000', '-i', source,
                '-map', '0:v:0', '-an', '-c:v', 'copy', '-progress', str(progress),
                '-stats_period', '1', '-f', 'rtsp', '-rtsp_transport', 'tcp',
                self._targets[aircraft]]

    def _stop(self, job, now):
        """Begin a nonblocking shutdown; the local stream monitor must never wait for WAN I/O."""
        if job.get('stopping') is None:
            job['stopping'] = now
            job['state'] = 'stopping'
            try:
                job['process'].terminate()
            except ProcessLookupError:
                pass
        elif now - job['stopping'] >= STOP_SECONDS:
            try:
                job['process'].kill()
            except ProcessLookupError:
                pass

    def sync(self, paths):
        """Start ready sources, retry failed public links and stop sources that disappeared."""
        if self._closed:
            return
        now = time.monotonic()
        for aircraft in self._targets:
            ready = f'{aircraft}/rectified' in paths
            job = self._jobs.setdefault(aircraft, dict(process=None, next_retry=0, state='idle'))
            process = job['process']
            if process is not None:
                code = process.poll()
                if code is not None:
                    job.update(process=None, next_retry=now + RETRY_SECONDS,
                               state='retrying' if ready else 'idle')
                    if job.get('stopping') is None:
                        logging.warning('Public video %s exited (%s); retrying independently', aircraft, code)
                elif not ready or job.get('stopping') is not None:
                    self._stop(job, now)
                else:
                    try:
                        modified = job['progress'].stat().st_mtime_ns
                    except OSError:
                        modified = None
                    if modified is not None and modified != job.get('modified'):
                        job.update(modified=modified, last_progress=now)
                        try:
                            progress = job['progress'].read_text()
                        except OSError:
                            progress = ''
                        if re.search(r'(?:frame|out_time_us)=\s*[1-9]\d*', progress):
                            job['state'] = 'streaming'
                    if now - job['last_progress'] > STALL_SECONDS:
                        logging.warning('Public video %s stalled; restarting independently', aircraft)
                        self._stop(job, now)
            if job['process'] is None and ready and now >= job['next_retry']:
                progress = Path(self._temporary.name) / f'{aircraft}.progress'
                progress.unlink(missing_ok=True)
                try:
                    # FFmpeg may echo credential-bearing URLs; expose only exit state in public logs.
                    process = subprocess.Popen(self._command(aircraft, progress), stdin=subprocess.DEVNULL,
                                               stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                except OSError:
                    logging.warning('Unable to start public video %s; check FFmpeg installation', aircraft)
                    job.update(state='retrying', next_retry=now + RETRY_SECONDS)
                    continue
                job.update(process=process, progress=progress, modified=None, stopping=None,
                           last_progress=now, state='connecting')
            elif job['process'] is None and not ready:
                job['state'] = 'idle'

    def status(self):
        """Expose forwarding state without returning secret-bearing destination addresses."""
        return {aircraft: {'state': self._jobs.get(aircraft, {}).get('state', 'idle')}
                for aircraft in self._targets}

    def close(self):
        """Reap only owned forwarding processes when the ground media service exits."""
        self._closed = True
        processes = [job['process'] for job in self._jobs.values() if job.get('process') is not None]
        for job in self._jobs.values():
            if job.get('process') is not None and job['process'].poll() is None:
                self._stop(job, time.monotonic())
        deadline = time.monotonic() + STOP_SECONDS
        for process in processes:
            try:
                process.wait(timeout=max(0, deadline - time.monotonic()))
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
        self._temporary.cleanup()
