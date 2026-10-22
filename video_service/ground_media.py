"""Independent ground media API, corrected-stream recording and read-only FTP service."""

import argparse
from concurrent.futures import ThreadPoolExecutor
import fcntl
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import logging
import os
from pathlib import Path
import re
import shutil
import signal
import subprocess
import sys
import threading
import time
from urllib.parse import parse_qs, quote, urlparse
from urllib.request import urlopen

# Resolve the source checkout even when launched directly by systemd.
PROJECT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT))
from video_service.media_store import MediaStore, atomic_json, safe_component
from video_service.camera_app.relay_status import status_path


def load_settings(path=None):
    """Load editable host settings while keeping calibration relative to the checkout."""
    path = Path(path or os.environ.get('GROUND_MEDIA_CONFIG', PROJECT / 'config/ground/media.json'))
    value = json.loads(path.read_text())
    value['root'] = str(Path(value['root']).expanduser().resolve())
    intrinsics = Path(value.get('intrinsics', 'config/video/Wasintek/intrinsics.yaml'))
    value['intrinsics'] = str(intrinsics if intrinsics.is_absolute() else PROJECT / intrinsics)
    return value


def stop_process(process, timeout=12):
    """Let a media process finalize before enforcing a finite shutdown."""
    if process and process.poll() is None:
        process.send_signal(signal.SIGINT)
        try:
            process.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait()


class GroundMedia:
    """Serve a persistent catalog and one processing pipeline per active aircraft."""

    def __init__(self, settings):
        """Open the store and recover interrupted recordings as explicitly abnormal."""
        self.config = settings
        self.root = Path(settings['root'])
        self.store = MediaStore(self.root, settings.get('max_bytes', 100 * 1024**3), Path(settings['intrinsics']))
        self.lock = threading.RLock()
        self.sessions = {}
        self.error = ''
        self.stop = threading.Event()
        self.workers = ThreadPoolExecutor(max_workers=3, thread_name_prefix='media-finalize')
        self.mtx = None
        self.ftp = None
        self._seen_events = set()
        self._processes = set()
        self._catalog = self.root / 'sessions.json'
        self._ended = set()
        self._uploads = {}
        if self._catalog.exists():
            for session in json.loads(self._catalog.read_text()).values():
                key = (session['aircraft_id'], session['session_id'])
                self._ended.add(key)
        for record in self.store.records():
            if record['kind'] == 'video' and record['state'] in ('recording', 'processing'):
                self.store.update(record['id'], state='error', error='地面媒体服务曾中断；可从已上传原片重建')

    def start(self):
        """Start network services and workers independently of preview windows."""
        self._start_mediamtx()
        self._start_ftp()
        for target in (self._ingest_loop, self._photo_loop, self._stream_loop):
            threading.Thread(target=target, daemon=True).start()

    def _start_mediamtx(self):
        """Expose corrected streams only; raw paths are readable on loopback only."""
        port = int(self.config.get('rtsp_port', 8554))
        api_port = int(self.config.get('mediamtx_api_port', 9997))
        users = [dict(user='any', ips=['127.0.0.1', '::1'], permissions=[
            dict(action='read', path=''), dict(action='publish', path=''), dict(action='api')]),
            dict(user='any', ips=[], permissions=[
                dict(action='publish', path='~^[A-Za-z0-9_-]+/raw$'),
                dict(action='read', path='~^[A-Za-z0-9_-]+/rectified$')])]
        value = dict(logLevel='warn', rtsp=True, rtspAddress=f':{port}', rtspTransports=['tcp'],
                     rtmp=False, hls=False, webrtc=False, srt=False, moq=False, playback=False,
                     api=True, apiAddress=f'127.0.0.1:{api_port}', authMethod='internal',
                     authInternalUsers=users, writeQueueSize=1024,
                     pathDefaults=dict(source='publisher', overridePublisher=False),
                     paths={'~^[A-Za-z0-9_-]+/(raw|rectified)$': {}})
        path = self.root / 'mediamtx.json'
        atomic_json(path, value)
        # JSON is valid YAML and avoids a second generated-configuration implementation.
        self.mtx = self._spawn([self.config.get('mediamtx', '/usr/local/bin/mediamtx'), str(path)], 'mediamtx')

    def _start_ftp(self):
        """Publish only completed corrected files using a read-only anonymous FTP root."""
        from pyftpdlib.authorizers import DummyAuthorizer
        from pyftpdlib.handlers import FTPHandler
        from pyftpdlib.servers import FTPServer
        authorizer = DummyAuthorizer()
        authorizer.add_anonymous(str(self.root / 'public'), perm='elr')
        handler = type('MediaFTPHandler', (FTPHandler,), {})
        handler.authorizer = authorizer
        handler.passive_ports = range(*self.config.get('ftp_passive_ports', [30000, 30020]))
        self.ftp = FTPServer((self.config.get('listen', '0.0.0.0'), int(self.config.get('ftp_port', 2121))), handler)
        threading.Thread(target=self.ftp.serve_forever, kwargs={'timeout': 1}, daemon=True).start()

    def _spawn(self, command, name):
        """Keep logs and process ownership local to this media service."""
        with (self.root / f'{name}.log').open('ab') as output:
            process = subprocess.Popen(command, stdout=subprocess.DEVNULL, stderr=output)
        with self.lock:
            self._processes.add(process)
        return process

    def relay_command(self, source, target, progress, intrinsics=None, options=None):
        """Use the existing Intel/NVIDIA/CPU implementation for live and rebuilt media."""
        options = options or self.config
        return [sys.executable, str(PROJECT / 'video_service/ground_relay.py'),
                '--input', source, '--output', str(target), '--backend', options.get('backend', 'auto'),
                '--intrinsics', str(intrinsics or options['intrinsics']),
                '--bitrate', str(options.get('bitrate', '8M')), '--gop', str(options.get('gop', 60)),
                '--chroma-location', options.get('chroma_location', 'left'),
                '--cpu-threads', str(options.get('cpu_threads', 4)), '--progress', str(progress)]

    def session(self, payload):
        """Merge idempotent aircraft notices without restarting a closed session."""
        aircraft = safe_component(payload['aircraft_id'])
        session_id = safe_component(payload['session_id'])
        action = payload.get('action', 'update')
        key = aircraft, session_id
        with self.lock:
            self._uploads[aircraft] = payload.get('upload_records', self._uploads.get(aircraft, []))
            if payload.get('mission_sequence') and payload.get('source_id'):
                for record in self.store.records():
                    if (record['aircraft_id'], record['session_id']) == key and record['kind'] == 'video':
                        metadata = dict(record['metadata'])
                        metadata.update(source_id=payload['source_id'], mission_sequence=payload['mission_sequence'])
                        self.store.update(record['id'], metadata=metadata)
            if key in self._ended:
                return
            session = self.sessions.get(aircraft)
            if session and session['session_id'] != session_id:
                if payload.get('started_at', 0) < session['metadata'].get('started_at', 0):
                    self._ended.add(key)
                    self._save_sessions()
                    return
                self._finish(session, '收到下一次会话，上一会话结束通知缺失')
                session = None
            if session is None:
                session = dict(aircraft_id=aircraft, session_id=session_id, metadata=dict(payload),
                               received_at=time.time(), relay=None, recorder=None, record_id=None,
                               next_retry=0, interruptions=[], last_seen=time.time())
                self.sessions[aircraft] = session
                # Freeze the intrinsics and processing settings for this recording and rebuild.
                calibration = self.store.session_intrinsics(aircraft, session_id)
                session['intrinsics'] = str(calibration)
                session['options'] = {key: self.config.get(key, default) for key, default in
                                      [('backend', 'auto'), ('bitrate', '8M'), ('gop', 60),
                                       ('chroma_location', 'left'), ('cpu_threads', 4)]}
                session['options']['intrinsics'] = str(calibration)
            session['metadata'].update(payload)
            session['last_seen'] = time.time()
            if action == 'end':
                self._finish(session, payload.get('error', ''))
                self.sessions.pop(aircraft, None)
            self._save_sessions()

    def _save_sessions(self):
        """Remember closed identities across restarts to avoid replaying durable events."""
        atomic_json(self._catalog, {f'{aircraft}/{session}': dict(aircraft_id=aircraft, session_id=session)
                                   for aircraft, session in self._ended})

    def _start_stream(self, session):
        """Start one correction pipeline and stream-copy its output into a private file."""
        aircraft = session['aircraft_id']
        base = f"rtsp://127.0.0.1:{self.config.get('rtsp_port', 8554)}/{aircraft}"
        session['output'] = base + '/rectified'
        session['relay'] = self._spawn(self.relay_command(base + '/raw', session['output'],
            self.root / f'{aircraft}-relay.progress', session['intrinsics'], session['options']), aircraft + '-relay')
        metadata = {**session['metadata'], 'processing': session['options']}
        record_id = self.store.create_record(aircraft, session['session_id'], metadata=metadata)
        session['record_id'] = record_id
        session['partial'] = str(self.root / 'staging' / f'{record_id}.mkv')
        session['record_started'] = None
        session['recorder'] = None

    def _start_recorder(self, session):
        """Mux the already encoded stream without decoding or encoding a second time."""
        progress = self.root / f"{session['record_id']}-record.progress"
        session['record_progress'] = progress
        session['recorder'] = self._spawn([
            'ffmpeg', '-hide_banner', '-nostdin', '-loglevel', 'warning', '-rtsp_transport', 'tcp',
            '-timeout', '15000000', '-i', session['output'], '-map', '0:v:0', '-an', '-c:v', 'copy',
            '-progress', str(progress), '-stats_period', '1', '-n', session['partial']],
            session['record_id'] + '-record')

    def _paths(self):
        """Read ready MediaMTX paths from the loopback control API."""
        with urlopen(f"http://127.0.0.1:{self.config.get('mediamtx_api_port', 9997)}/v3/paths/list", timeout=2) as response:
            return {item['name']: item for item in json.load(response)['items'] if item.get('ready')}

    def _stream_loop(self):
        """Reconnect interrupted publishers and preserve failure markers on each fragment."""
        while not self.stop.wait(0.5):
            try:
                paths = self._paths()
                with self.lock:
                    for aircraft, session in list(self.sessions.items()):
                        if time.time() < session['next_retry']:
                            continue
                        if not session['relay']:
                            if f'{aircraft}/raw' in paths:
                                self._start_stream(session)
                            continue
                        if session['relay'].poll() is not None:
                            self._finish_fragment(session, '去畸变进程退出，实时录像中断')
                            session['next_retry'] = time.time() + 3
                            continue
                        if not session['recorder'] and f'{aircraft}/rectified' in paths:
                            self._start_recorder(session)
                        recorder = session['recorder']
                        if recorder:
                            if recorder.poll() is not None:
                                self._finish_fragment(session, '录像封装进程退出，实时录像中断')
                                session['next_retry'] = time.time() + 3
                                continue
                            if not session['record_started'] and session['record_progress'].exists():
                                progress = session['record_progress'].read_text()
                                if re.search(r'out_time_us=[1-9]', progress):
                                    session['record_started'] = time.time()
                                    delay = session['record_started'] - session['received_at']
                                    if delay > 3:
                                        session['interruptions'].append(f'地面首帧比会话通知晚约{delay:.1f}秒，开头覆盖未保证')
                        if f'{aircraft}/raw' not in paths and time.time() - session['last_seen'] > 30:
                            self._finish(session, '飞机原始流和会话通知中断，结束状态未知')
                            self.sessions.pop(aircraft, None)
                        try:
                            notice = json.loads(status_path(session['output']).read_text())
                            if notice.get('warning') and notice['warning'] not in session['interruptions']:
                                session['interruptions'].append(notice['warning'])
                        except (OSError, ValueError):
                            pass
            except Exception as exc:
                self.error = str(exc)
                logging.warning('Media stream monitor: %s', exc)

    def _finish_fragment(self, session, error=''):
        """Finalize one interrupted fragment; never splice it into an apparently complete file."""
        if error:
            session['interruptions'].append(error)
        stop_process(session.get('recorder'))
        stop_process(session.get('relay'))
        if session.get('record_id'):
            self.store.update(session['record_id'], state='processing')
            snapshot = {**session, 'metadata': dict(session['metadata']),
                        'interruptions': list(session['interruptions']), 'finished_at': time.time()}
            self.workers.submit(self._validate_recording, snapshot)
        session.update(relay=None, recorder=None, record_id=None, record_started=None)

    def _finish(self, session, error=''):
        """Finish a session even if no corrected frame ever reached the recorder."""
        self._finish_fragment(session, error)
        if not session.get('partial'):
            record = self.store.create_record(session['aircraft_id'], session['session_id'], metadata=session['metadata'])
            self.store.update(record, state='error', error=error or '未收到可录制的去畸变视频')
        self._ended.add((session['aircraft_id'], session['session_id']))

    def _validate_recording(self, session):
        """Decode the file and compare approximate duration, retaining all known interruptions."""
        record_id = session['record_id']
        try:
            path = Path(session['partial'])
            probe = subprocess.run(['ffprobe', '-v', 'error', '-show_entries', 'format=duration',
                                    '-of', 'json', str(path)], capture_output=True, text=True, timeout=30)
            duration = float(json.loads(probe.stdout)['format']['duration'])
            check = subprocess.run(['ffmpeg', '-v', 'error', '-nostdin', '-i', str(path), '-map', '0:v:0',
                                    '-f', 'null', '-'], capture_output=True, text=True, timeout=max(60, duration * 3))
            errors = list(session['interruptions'])
            expected = max(0, session['metadata'].get('ended_at', session['finished_at']) -
                           session['metadata'].get('started_at', session['received_at']))
            if abs(duration - expected) > max(3, expected * 0.1):
                errors.append(f'时长与会话不符：视频{duration:.1f}秒 / 会话约{expected:.1f}秒')
            if check.returncode or check.stderr.strip():
                errors.append('完整解码检查异常：' + check.stderr[-500:])
            metadata = {**session['metadata'], 'duration': duration, 'expected_duration': expected,
                        'processing': session['options'], 'state': 'incomplete' if errors else 'ready',
                        'error': '；'.join(dict.fromkeys(errors)), 'variant': 'experiment'}
            self.store.register_processed(path, session['aircraft_id'], session['session_id'], metadata,
                                          record_id=record_id)
        except Exception as exc:
            self.store.update(record_id, state='error', error=f'录像保存检查失败：{exc}')

    def _ingest_loop(self):
        """Accept verified uploads and replay durable start/end events independently of previews."""
        while not self.stop.wait(1):
            try:
                results = self.store.ingest_pending()
                for result in results:
                    if result['state'] == 'error':
                        self.error = result['error']
                for event in self.store.events():
                    if event['id'] in self._seen_events:
                        continue
                    if event['event'] in ('session_start', 'session_end'):
                        self.session({**event['metadata'], 'aircraft_id': event['aircraft_id'],
                                      'session_id': event['session_id'],
                                      'action': 'end' if event['event'] == 'session_end' else 'start'})
                    self._seen_events.add(event['id'])
                self.store.cleanup()
            except Exception as exc:
                self.error = str(exc)
                logging.exception('Media ingestion failed')

    def _photo_loop(self):
        """Process images on a separate worker so acknowledgements remain responsive."""
        while not self.stop.wait(1):
            try:
                self.store.process_pending()
            except Exception as exc:
                self.error = str(exc)

    def inventory(self):
        """Expose only processed download URLs while retaining private-file state in the catalog."""
        records = self.store.records()
        public = self.config['public_host']
        known = {record['id']: record for record in records}
        for uploads in list(self._uploads.values()):
            for upload in uploads:
                if upload['id'] not in known:
                    records.append({**upload, 'variant': 'original', 'path': '', 'processed_path': '',
                                    'original_path': '', 'processed_size': 0})
                else:
                    known[upload['id']]['cache_state'] = upload['cache_state']
                    known[upload['id']]['upload_state'] = upload['state']
        originals = {(row['aircraft_id'], row['session_id']): row for row in records
                     if row['kind'] == 'video' and row['variant'] == 'original'}
        for record in records:
            if record['kind'] == 'video' and record['variant'] != 'original':
                original = originals.get((record['aircraft_id'], record['session_id']))
                if original:
                    record['original_id'] = original['id']
                    record['cache_state'] = original.get('cache_state', 'unknown')
                    record['missing_reason'] = f"机载原片状态：{original['state']}"
            record.update({key: record['metadata'].get(key, '') for key in
                           ('mission_sequence', 'waypoint_index', 'photo_no', 'source_id')})
            relative = self.store.public_url(record)
            if record.get('path') and not relative and record['state'] in {'ready', 'incomplete'}:
                record.update(state='missing', error='地面处理文件缺失，下载暂不可用')
            record['download_url'] = f"http://{public}:{self.config.get('api_port', 8090)}/api/file?id={record['id']}" if relative else ''
            record['download_directory'] = f"ftp://{public}:{self.config.get('ftp_port', 2121)}/{quote(str(Path(relative).parent))}/" if relative else ''
        with self.lock:
            streams = [dict(aircraft_id=aircraft, session_id=item['session_id'],
                            state='recording' if item.get('record_started') else 'starting',
                            url=f"rtsp://{public}:{self.config.get('rtsp_port', 8554)}/{aircraft}/rectified",
                            error='；'.join(item['interruptions'])) for aircraft, item in self.sessions.items()]
        storage = self.store.storage()
        return dict(records=records, storage=storage, streams=streams,
                    error='；'.join(filter(None, [self.error, storage.get('error', '')])))

    def rebuild(self, record_id):
        """Reprocess a complete original with the session's frozen calibration and settings."""
        record = self.store.get(record_id)
        if not record or record['kind'] != 'video':
            raise ValueError('请选择原始录像记录')
        if not record.get('original_path'):
            candidates = [row for row in self.store.records() if row['aircraft_id'] == record['aircraft_id']
                          and row['session_id'] == record['session_id'] and row.get('original_path') and row['kind'] == 'video']
            if not candidates:
                raise ValueError('完整原片尚未上传，暂不能重建')
            record = candidates[0]
        if not Path(record['original_path']).is_file():
            raise ValueError('原片已被容量策略清理，暂不能重建')
        metadata = dict(record['metadata'])
        metadata['variant'] = 'rebuilt'
        new_id = self.store.create_record(record['aircraft_id'], record['session_id'], metadata=metadata, variant='rebuilt')
        self.store.update(new_id, state='processing')
        original_state = record['state']
        self.store.update(record['id'], state='processing')
        self.workers.submit(self._rebuild, record, new_id, metadata, original_state)
        return new_id

    def _rebuild(self, record, record_id, metadata, original_state):
        """Keep rebuilt output private until full decoding succeeds."""
        try:
            target = self.root / 'staging' / (record_id + '.mp4')
            calibration = self.root / 'internal' / record['aircraft_id'] / record['session_id'] / 'intrinsics.yaml'
            previous = next((r for r in self.store.records() if r['session_id'] == record['session_id']
                             and r['aircraft_id'] == record['aircraft_id'] and r['metadata'].get('processing')), None)
            options = previous['metadata']['processing'] if previous else self.config
            if not calibration.is_file():
                raise ValueError('会话固定内参缺失，不能保证重建与实时版一致')
            process = self._spawn(self.relay_command(record['original_path'], target,
                self.root / (record_id + '.progress'), calibration, options), record_id + '-rebuild')
            if process.wait() != 0:
                raise RuntimeError('原片去畸变进程失败，详见后台日志')
            check = subprocess.run(['ffmpeg', '-v', 'error', '-nostdin', '-i', str(target),
                                    '-f', 'null', '-'], capture_output=True, text=True)
            if check.returncode or check.stderr.strip():
                raise RuntimeError('重建文件不能完整解码：' + check.stderr[-500:])
            metadata.update(state='ready', variant='rebuilt', rebuilt_from=record['id'])
            self.store.register_processed(target, record['aircraft_id'], record['session_id'], metadata,
                                          record_id=record_id)
        except Exception as exc:
            self.store.update(record_id, state='error', error=str(exc))
        finally:
            self.store.update(record['id'], state=original_state)

    def close(self):
        """Finalize owned media without touching the ROS flight-control stack."""
        self.stop.set()
        with self.lock:
            for session in self.sessions.values():
                self._finish(session, '地面媒体服务关闭，实时会话中断')
            self._save_sessions()
        for process in list(self._processes):
            stop_process(process)
        self.workers.shutdown(wait=True)
        if self.ftp:
            self.ftp.close_all()


class MediaHandler(BaseHTTPRequestHandler):
    """Small JSON control API plus finalized-file downloads."""

    def _json(self, value, status=200):
        """Write a complete JSON response with explicit size."""
        data = json.dumps(value, ensure_ascii=False).encode()
        self.send_response(status)
        self.send_header('Content-Type', 'application/json; charset=utf-8')
        self.send_header('Content-Length', str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def do_GET(self):
        """List media, or serve a validated processed path only."""
        parsed = urlparse(self.path)
        if parsed.path == '/api/media':
            self._json(self.server.media.inventory())
        elif parsed.path == '/api/file':
            record = self.server.media.store.get(parse_qs(parsed.query).get('id', [''])[0])
            if not record or not self.server.media.store.public_url(record):
                self._json({'error': '文件未就绪或已清理'}, 404)
                return
            try:
                with Path(record['path']).open('rb') as source:
                    size = os.fstat(source.fileno()).st_size
                    self.send_response(200)
                    self.send_header('Content-Type', 'application/octet-stream')
                    self.send_header('Content-Length', str(size))
                    self.send_header('Content-Disposition', f'attachment; filename="{Path(record["path"]).name}"')
                    self.end_headers()
                    shutil.copyfileobj(source, self.wfile)
            except (OSError, BrokenPipeError):
                return
        else:
            self._json({'error': 'Not found'}, 404)

    def do_POST(self):
        """Accept aircraft lifecycle notices and local file-manager operations."""
        try:
            size = int(self.headers.get('Content-Length', 0))
            if not 0 < size <= 1024 * 1024:
                raise ValueError('Invalid request size')
            payload = json.loads(self.rfile.read(size))
            if self.path == '/api/session':
                self.server.media.session(payload)
                result = {'ok': True}
            else:
                if self.client_address[0] not in ('127.0.0.1', '::1'):
                    self._json({'error': '文件管理操作仅供地面本机使用'}, 403)
                    return
                if self.path == '/api/delete':
                    self.server.media.store.delete(payload['id'])
                    result = {'ok': True}
                elif self.path == '/api/rebuild':
                    result = {'id': self.server.media.rebuild(payload['id'])}
                else:
                    self._json({'error': 'Not found'}, 404)
                    return
            self._json(result)
        except (ValueError, KeyError, OSError) as exc:
            self._json({'error': str(exc)}, 400)

    def log_message(self, format, *args):
        """Keep per-second polling out of the operator's log."""
        if args and str(args[1]) not in ('200',):
            logging.info(format, *args)


def main():
    """Run a single host media instance with graceful process finalization."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path)
    args = parser.parse_args()
    logging.basicConfig(level=logging.INFO, format='[ground-media] %(message)s')
    settings = load_settings(args.config)
    root = Path(settings['root'])
    root.mkdir(parents=True, exist_ok=True)
    with (root / 'service.lock').open('w') as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            return 0
        media = GroundMedia(settings)
        server = ThreadingHTTPServer((settings.get('listen', '0.0.0.0'), int(settings.get('api_port', 8090))), MediaHandler)
        server.media = media
        server.timeout = 0.5
        stopping = threading.Event()
        def request_stop(signum, frame):
            """Defer cleanup until the current HTTP request has completed."""
            stopping.set()
        signal.signal(signal.SIGINT, request_stop)
        signal.signal(signal.SIGTERM, request_stop)
        try:
            media.start()
            while not stopping.is_set():
                server.handle_request()
        finally:
            server.server_close()
            media.close()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
