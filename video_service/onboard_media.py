"""Forward onboard video and persist uploads without changing the flight lifecycle."""

from datetime import datetime, timedelta
import json
import logging
from pathlib import Path
import signal
import subprocess
import threading
import time
from urllib.request import Request, urlopen

from video_service.media_transfer import UploadQueue


class OnboardMedia:
    """Own a single stream publisher and a durable aircraft upload queue."""

    def __init__(self, controller, settings, aircraft_id):
        """Read aircraft destinations and resume transfers independently of the camera."""
        self.controller = controller
        self.settings = settings
        self.aircraft_id = aircraft_id
        self.cache = Path(settings['cache_directory']).expanduser()
        self.cache.mkdir(parents=True, exist_ok=True)
        self.queue = UploadQueue(self.cache, aircraft_id, settings['upload_target'],
                                 settings['upload_root'], float(settings.get('retain_seconds', 86400)),
                                 int(settings.get('bwlimit_kib', 2048)))
        self.queue.start()
        self._session_ids = {record['session_id'] for record in self.queue.records()}
        self.session = None
        self._last_session = None
        self.error = ''
        self._upload_error = ''
        self._forward = None
        self._lock = threading.RLock()
        self._shutdown = threading.Event()
        self._thread = threading.Thread(target=self._loop, daemon=True, name='onboard-media')
        self._thread.start()

    def begin(self, source_id):
        """Persist session identity before capture, so late reception remains visible."""
        with self._lock:
            if self.session:
                return
            stamp = datetime.now().replace(microsecond=0)
            session_id = stamp.strftime('mission-%Y%m%d-%H%M%S')
            # Keep same-second restarts readable without merging their session identities.
            while session_id in self._session_ids:
                stamp += timedelta(seconds=1)
                session_id = stamp.strftime('mission-%Y%m%d-%H%M%S')
            self._session_ids.add(session_id)
            self.session = {'session_id': session_id,
                            'source_id': source_id, 'started_at': time.time(), 'mission_sequence': 0}
            self._previous_recording = self.controller.status().get('last_recording_file', '')
            self._last_session = dict(self.session)
            self.queue.enqueue_event('session_start', self.session['session_id'], dict(self.session))
            logging.info('Media session started: session=%s source=%s', session_id, source_id)
        self._notify('start')

    def update_mission(self, source_id, sequence):
        """Associate passive flight status with media without blocking a ROS callback."""
        with self._lock:
            if self.session and sequence:
                self.session.update(source_id=str(source_id), mission_sequence=int(sequence))

    def capture(self, path, metadata):
        """Queue only the completed original and keep photo number separate from waypoint."""
        with self._lock:
            if not self.session:
                self.begin(metadata.get('source_id', 'manual'))
            if metadata.get('mission_sequence') and metadata.get('source_id'):
                self.session.update(source_id=metadata['source_id'], mission_sequence=metadata['mission_sequence'])
            identity = dict(self.session)
            identity.update(metadata)
            media_id = self.queue.enqueue(Path(path), 'image', self.session['session_id'], identity)
            logging.info('Photo queued: id=%s session=%s source=%s mission=%s waypoint=%s file=%s',
                         media_id, self.session['session_id'], identity.get('source_id'),
                         identity.get('mission_sequence'), identity.get('waypoint_index'), Path(path).name)
        self._notify('update')
        return media_id

    def finish(self, error=''):
        """Upload a finalized original even when its real-time processed version failed."""
        with self._lock:
            if not self.session:
                return
            status = self.controller.status()
            metadata = {**self.session, 'ended_at': time.time(), 'error': error}
            path = status.get('last_recording_file', '')
            if path and path != self._previous_recording and Path(path).is_file():
                self.queue.enqueue(Path(path), 'video', self.session['session_id'], metadata)
            self.queue.enqueue_event('session_end', self.session['session_id'], metadata)
            logging.info('Media session ended: session=%s source=%s mission=%s error=%s',
                         self.session['session_id'], metadata.get('source_id'),
                         metadata.get('mission_sequence'), error)
            self._notify('end', metadata)
            self.session = None
        self._stop_forward()

    def _notify(self, action, metadata=None):
        """Best-effort low-latency session notice; the queue provides durable delivery."""
        payload = {'action': action, 'aircraft_id': self.aircraft_id,
                   **(metadata or self.session or self._last_session or {}),
                   'upload_records': self.queue.summary()}
        if not payload.get('session_id'):
            return
        try:
            request = Request(self.settings['ground_api'].rstrip('/') + '/api/session',
                              json.dumps(payload).encode(), {'Content-Type': 'application/json'})
            with urlopen(request, timeout=2) as response:
                response.read()
        except Exception as exc:
            self.error = f'地面媒体通知失败，将由上传队列重试：{exc}'

    def _stop_forward(self):
        """Stop only this publisher, never the camera or flight processes."""
        process, self._forward = self._forward, None
        if process and process.poll() is None:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()

    def _loop(self):
        """Reconnect the one raw publisher and periodically report cache state."""
        notified = 0.0
        while not self._shutdown.wait(1):
            try:
                status = self.controller.status()
                if self.session and status['state'] in ('starting', 'running'):
                    if self._forward is None or self._forward.poll() is not None:
                        if self._forward is not None:
                            self.error = '原始流推送中断，正在重连；本地录像继续'
                        log = self.cache / 'forward.log'
                        with log.open('ab') as output:
                            self._forward = subprocess.Popen([
                                'ffmpeg', '-hide_banner', '-nostdin', '-loglevel', 'warning',
                                '-rtsp_transport', 'tcp', '-timeout', '5000000', '-i',
                                status['local_rtsp_url'], '-map', '0:v:0', '-an', '-c:v', 'copy',
                                '-f', 'rtsp', '-rtsp_transport', 'tcp', '-timeout', '5000000',
                                self.settings['push_url']], stdout=subprocess.DEVNULL, stderr=output)
                elif self.session and status['state'] == 'error':
                    self.finish(status.get('last_error', '机载录像中断'))
                if time.monotonic() - notified > 5:
                    self._notify('update')
                    notified = time.monotonic()
                upload_error = next((str(record['error']) for record in self.queue.records()
                                     if record.get('error')), '')
                if upload_error:
                    self.error = upload_error
                elif self.error == self._upload_error:
                    # Clear only the recovered queue warning, preserving unrelated failures.
                    self.error = ''
                self._upload_error = upload_error
            except Exception as exc:
                self.error = str(exc)
                logging.exception('Onboard media background error')

    def close(self):
        """Keep finalized media queued when the independent node is stopped."""
        self.finish()
        self._shutdown.set()
        self._thread.join(timeout=8)
        self._stop_forward()
        self.queue.close()
