"""Persistent aircraft uploads with resumable SSH transfer and ground-confirmed retention."""

import errno
import json
import logging
import os
from pathlib import Path, PurePosixPath
import shlex
import signal
import sqlite3
import subprocess
import threading
import time
import uuid

from .media_store import atomic_json, file_hash, safe_component, safe_media_filename


class UploadQueue:
    """Run media transfer independently of capture and flight-control lifecycles."""

    def __init__(self, cache_root: Path, aircraft_id: str, ssh_target: str, remote_root: str,
                 retain_seconds: float = 86400, bwlimit_kib: int = 2048):
        """Recover persistent queue entries; callers explicitly start the background worker."""
        self.root = Path(cache_root).resolve()
        self.root.mkdir(parents=True, exist_ok=True)
        self.aircraft_id = safe_component(aircraft_id)
        if not ssh_target or ssh_target.startswith('-') or any(c.isspace() for c in ssh_target):
            raise ValueError('SSH upload target must be a host alias or user@host')
        if not PurePosixPath(remote_root).is_absolute():
            raise ValueError('Ground media root must be an absolute path')
        self.ssh_target = ssh_target
        self.remote_root = str(PurePosixPath(remote_root))
        self.retain_seconds = max(0, float(retain_seconds))
        self.bwlimit_kib = max(0, int(bwlimit_kib))
        self.lock = threading.RLock()
        self.tick_lock = threading.Lock()
        self.stop = threading.Event()
        self.wake = threading.Event()
        self.worker = None
        self.current_process = None
        self._active_record_id = None
        self.db = sqlite3.connect(self.root / 'uploads.sqlite3', check_same_thread=False)
        self.db.execute('PRAGMA journal_mode=WAL')
        self.db.execute('CREATE TABLE IF NOT EXISTS uploads (id TEXT PRIMARY KEY, data TEXT NOT NULL)')
        self.db.execute('CREATE TABLE IF NOT EXISTS delivery (session_id TEXT PRIMARY KEY, state TEXT, error TEXT)')
        self.db.commit()
        for name in ('manifests', 'acks'):
            (self.root / name).mkdir(exist_ok=True)
        for record in self.records():
            if record['state'] == 'uploading':
                record.update(state='retry', retry_at=0)
                self._save(record)

    def _save(self, record):
        """Persist a whole queue record in one SQLite transaction."""
        with self.lock, self.db:
            delivery = self.db.execute('SELECT state, error FROM delivery WHERE session_id=?',
                                       (record['session_id'],)).fetchone()
            if delivery and record['kind'] != 'event' and record['state'] != 'cache_deleted':
                if delivery[0] == 'failed':
                    record.update(state='needs_attention', error=delivery[1], retry_at=0)
                elif delivery[0] == 'ready' and record['kind'] == 'video':
                    record.update(state='retained', error='', retry_at=0,
                                  acknowledged_at=record.get('acknowledged_at') or time.time())
            self.db.execute('INSERT OR REPLACE INTO uploads VALUES (?, ?)',
                            (record['id'], json.dumps(record, ensure_ascii=False)))

    def records(self):
        """Return upload states without blocking on hashing or network activity."""
        with self.lock:
            rows = self.db.execute('SELECT data FROM uploads').fetchall()
        return sorted((json.loads(row[0]) for row in rows), key=lambda row: row['created_at'])

    def summary(self, session_id=None):
        """Expose small upload snapshots for ground session notices without local paths."""
        rows = []
        for record in self.records():
            if record['kind'] == 'event' or session_id is not None and record['session_id'] != session_id:
                continue
            row = {key: record.get(key) for key in ('id', 'aircraft_id', 'session_id', 'kind',
                   'state', 'size', 'sha256', 'metadata', 'error', 'created_at', 'acknowledged_at')}
            row['cache_state'] = ('deleted' if record['state'] == 'cache_deleted' else
                                  'retained' if Path(record['path']).is_file() else 'missing')
            rows.append(row)
        return rows

    def enqueue(self, path, kind, session_id, metadata=None, held=False):
        """Persist a finalized file reference quickly; hashing happens on the worker."""
        kind = 'image' if kind == 'photo' else kind
        if kind not in {'image', 'video'}:
            raise ValueError('Upload kind must be image or video')
        path = Path(path).resolve()
        if not path.is_file():
            raise FileNotFoundError(path)
        session_id = safe_component(session_id)
        with self.lock:
            # Repeated enqueue of the same finalized pathname is one business file.
            for record in self.records():
                if (record.get('path') == str(path) and record['session_id'] == session_id
                        and record['state'] != 'cache_deleted'):
                    return record['id']
            record_id = uuid.uuid4().hex
            self._save(dict(id=record_id, aircraft_id=self.aircraft_id, session_id=session_id,
                            kind=kind, path=str(path), metadata=metadata or {}, state='held' if held else 'queued',
                            created_at=time.time(), attempts=0, retry_at=0, error='',
                            size=0, sha256='', acknowledged_at=None))
        self.wake.set()
        return record_id

    def hold_pending_videos(self):
        """Require a fresh ground decision for originals recovered by the onboard service."""
        for record in self.records():
            if record['kind'] == 'video' and record['state'] in {'queued', 'retry', 'uploading', 'awaiting_ack'}:
                record.update(state='held', retry_at=0)
                self._save(record)

    def unavailable_video(self, session_id, metadata=None):
        """Persist a missing-source placeholder until ground decides whether fallback is needed."""
        self._save(dict(id=uuid.uuid4().hex, aircraft_id=self.aircraft_id,
                        session_id=safe_component(session_id), kind='video', path='',
                        metadata=metadata or {}, state='held', created_at=time.time(),
                        attempts=0, retry_at=0, error='', size=0, sha256='', acknowledged_at=None))

    def delivery_sessions(self):
        """Return unfinished video sessions, newest first, without polling historical deliveries."""
        with self.lock:
            decisions = dict(self.db.execute('SELECT session_id, state FROM delivery').fetchall())
            pending = {}
            for record in self.records():
                if record['kind'] != 'video' or decisions.get(record['session_id']) == 'ready':
                    continue
                unfinished = record['state'] in {'held', 'queued', 'retry', 'uploading',
                                                  'awaiting_ack', 'needs_attention'}
                awaiting_decision = (record['state'] == 'acknowledged'
                                     and decisions.get(record['session_id']) in {'waiting', 'fallback'})
                if unfinished or awaiting_decision:
                    pending[record['session_id']] = record['created_at']
        return sorted(pending, key=pending.get, reverse=True)

    def set_delivery(self, session_id, state, reason=''):
        """Persist delivery decisions and stop only the affected session's active transfer."""
        if state not in {'waiting', 'fallback', 'ready', 'failed'}:
            raise ValueError(f'Unknown video delivery state: {state}')
        with self.lock:
            previous = self.db.execute('SELECT state FROM delivery WHERE session_id=?', (session_id,)).fetchone()
            # Local and ground terminal decisions never resume work after a restart.
            if previous and previous[0] in {'ready', 'failed'}:
                return
            with self.db:
                self.db.execute('INSERT OR REPLACE INTO delivery VALUES (?, ?, ?)',
                                (session_id, state, str(reason)))
            if state == 'failed':
                logging.error('Media delivery stopped: session=%s reason=%s', session_id, reason)
            for record in self.records():
                if record['session_id'] != session_id or record['kind'] == 'event':
                    continue
                if state == 'fallback' and record['kind'] == 'video' and record['state'] == 'held':
                    record.update(state='queued', retry_at=0)
                self._save(record)
            active = next((record for record in self.records() if record['id'] == self._active_record_id), None)
            should_stop = (active and active['session_id'] == session_id and
                           (state == 'failed' or state == 'ready' and active['kind'] == 'video'))
            if should_stop and self.current_process and self.current_process.poll() is None:
                self._stop_process(self.current_process)
        self.wake.set()

    def _check_active(self):
        """Observe persisted cancellation before launching commands or publishing a manifest."""
        if self.stop.is_set():
            raise RuntimeError('Upload queue is stopping')
        if self._active_record_id:
            active = next((record for record in self.records() if record['id'] == self._active_record_id), None)
            if active and active['state'] in {'needs_attention', 'retained', 'cache_deleted'}:
                raise RuntimeError('Session media upload stopped by delivery decision')

    @staticmethod
    def _needs_attention(error):
        """Separate actionable configuration/source faults from recoverable network failures."""
        if isinstance(error, OSError) and error.errno in {errno.ENOENT, errno.EACCES, errno.EPERM,
                                                       errno.ENOSPC, errno.EROFS, errno.ENOTDIR}:
            return True
        message = str(error).lower()
        return any(text in message for text in (
            'permission denied', 'authentication failed', 'host key verification failed',
            'no space left on device', 'disk quota exceeded', 'read-only file system',
            'no such file or directory', 'not a directory', 'bad configuration option',
            'unknown option', 'identity file', 'source changed', 'media changed'))

    def enqueue_event(self, event, session_id, metadata=None):
        """Persist a session notification through the same atomic manifest protocol."""
        record_id = uuid.uuid4().hex
        self._save(dict(id=record_id, aircraft_id=self.aircraft_id,
                        session_id=safe_component(session_id), kind='event', event=str(event),
                        path='', metadata=metadata or {}, state='queued', created_at=time.time(),
                        attempts=0, retry_at=0, error='', size=0, sha256='', acknowledged_at=None))
        self.wake.set()
        return record_id

    def _run(self, args, timeout=900):
        """Run a bounded transfer command, allowing close to terminate its child."""
        with self.lock:
            self._check_active()
            process = subprocess.Popen(args, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                                       text=True, start_new_session=True)
            self.current_process = process
        try:
            _, error = process.communicate(timeout=timeout)
            if process.returncode:
                raise OSError(error.strip() or f'{args[0]} exited {process.returncode}')
        except subprocess.TimeoutExpired:
            self._stop_process(process, force=True)
            process.communicate()
            raise OSError(f'{args[0]} timed out') from None
        finally:
            self.current_process = None

    @staticmethod
    def _stop_process(process, force=False):
        """Stop the transfer group so rsync cannot leave an SSH child uploading."""
        try:
            os.killpg(process.pid, signal.SIGKILL if force else signal.SIGTERM)
        except ProcessLookupError:
            pass

    def _ssh(self, command):
        """Execute an SSH command without interactive password prompts."""
        self._run(['ssh', '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=10',
                   '-o', 'ServerAliveInterval=15', '-o', 'ServerAliveCountMax=3',
                   self.ssh_target, command], timeout=60)

    def _rsync(self, source, destination):
        """Resume partial copies while limiting the aircraft's upload bandwidth."""
        # Checksum comparison also repairs a same-size corrupt receiver file on retry.
        args = ['rsync', '-rs', '--partial', '--checksum', '--timeout=60',
                '-e', 'ssh -o BatchMode=yes -o ConnectTimeout=10 -o ServerAliveInterval=15 -o ServerAliveCountMax=3']
        if self.bwlimit_kib:
            args.append(f'--bwlimit={self.bwlimit_kib}')
        self._run(args + ['--', str(source), str(destination)])

    def _remote(self, *parts):
        """Build a configured remote pathname from validated identity components."""
        return str(PurePosixPath(self.remote_root).joinpath(*parts))

    def _upload(self, record):
        """Copy the complete payload first and publish its manifest with remote rename."""
        record.update(state='uploading', attempts=record['attempts'] + 1, error='')
        self._save(record)
        self._check_active()
        if record['kind'] != 'event' and (not record['path'] or not Path(record['path']).is_file()):
            raise FileNotFoundError(errno.ENOENT, 'Aircraft original/source is unavailable', record['path'])
        inbox = self._remote('inbox', self.aircraft_id)
        ack = self._remote('ack', self.aircraft_id)
        self._ssh(f'mkdir -p -- {shlex.quote(inbox)} {shlex.quote(ack)}')
        manifest = {key: record[key] for key in ('id', 'aircraft_id', 'session_id', 'kind',
                                                'metadata', 'created_at')}
        if record['kind'] == 'event':
            manifest['event'] = record['event']
        else:
            source = Path(record['path'])
            before = source.stat()
            sha256 = record['sha256'] or file_hash(source)
            after = source.stat()
            if before.st_size != after.st_size or before.st_mtime_ns != after.st_mtime_ns:
                raise OSError('Media changed while hashing; enqueue only finalized files')
            if record['sha256'] and (after.st_size != record['size'] or
                                    after.st_mtime_ns != record.get('source_mtime_ns')):
                raise OSError('Finalized upload source changed after its first attempt')
            filename = record['id'] + source.suffix.lower()
            safe_component(filename)
            record.update(size=after.st_size, sha256=sha256, source_mtime_ns=after.st_mtime_ns)
            self._save(record)
            manifest.update(size=record['size'], sha256=sha256, filename=filename,
                            original_filename=safe_media_filename(source.name))
            self._rsync(source, f'{self.ssh_target}:{inbox}/{filename}')
            final = source.stat()
            if (final.st_size, final.st_mtime_ns) != (after.st_size, after.st_mtime_ns):
                raise OSError('Media changed during upload; manifest withheld')
        local = self.root / 'manifests' / (record['id'] + '.json')
        self._check_active()
        atomic_json(local, manifest)
        remote = f'{inbox}/{record["id"]}.json'
        self._rsync(local, f'{self.ssh_target}:{remote}.tmp')
        self._ssh(f'mv -f -- {shlex.quote(remote + ".tmp")} {shlex.quote(remote)}')
        record.update(state='awaiting_ack', error='', retry_at=time.time() + 30)
        self._save(record)

    def _read_acks(self):
        """Pull small ground receipts and start retention only after a matching ACK."""
        waiting = [row for row in self.records() if row['state'] == 'awaiting_ack']
        if not waiting:
            return
        remote = self._remote('ack', self.aircraft_id) + '/'
        try:
            # Do not interpret a receipt cached before ground capacity eviction as a fresh ACK.
            for record in waiting:
                (self.root / 'acks' / (record['id'] + '.json')).unlink(missing_ok=True)
            self._rsync(f'{self.ssh_target}:{remote}', str(self.root / 'acks') + '/')
        except (OSError, RuntimeError) as error:
            for record in waiting:
                record['error'] = f'Ground acknowledgement unavailable: {error}'
                self._save(record)
            return
        for record in waiting:
            path = self.root / 'acks' / (record['id'] + '.json')
            try:
                ack = json.loads(path.read_text())
                if (ack['id'] != record['id'] or ack.get('sha256', '') != record['sha256']
                        or int(ack.get('size', 0)) != record['size']):
                    raise ValueError('Ground acknowledgement does not match original')
                record.update(state='acknowledged', acknowledged_at=time.time(), error='')
                self._save(record)
            except FileNotFoundError:
                continue
            except (ValueError, KeyError, TypeError, OSError) as error:
                record['error'] = str(error)
                self._save(record)

    def _cleanup(self):
        """Delete originals only after ground delivery or upload ACK and configured retention."""
        for record in self.records():
            if record['state'] not in {'acknowledged', 'retained'} or time.time() - record['acknowledged_at'] < self.retain_seconds:
                continue
            try:
                if record['path']:
                    Path(record['path']).unlink(missing_ok=True)
                record.update(state='cache_deleted', error='')
                self._save(record)
                if record['kind'] != 'event':
                    self.enqueue_event('cache_deleted', record['session_id'], {'record_id': record['id']})
            except OSError as error:
                record['error'] = f'Cannot clean acknowledged cache: {error}'
                self._save(record)

    def tick(self):
        """Perform one transfer iteration; run this on a worker, never a capture callback."""
        if not self.tick_lock.acquire(blocking=False):
            return
        try:
            self._read_acks()
            self._cleanup()
            for record in self.records():
                if record['state'] == 'awaiting_ack' and record['retry_at'] <= time.time():
                    record.update(state='retry', retry_at=0)
                    self._save(record)
            ready = [row for row in self.records() if row['state'] in {'queued', 'retry'}
                     and row['retry_at'] <= time.time()]
            ready.sort(key=lambda row: (0 if row['kind'] == 'event' else 1 if row['kind'] in {'image', 'photo'} else 2,
                                        row['created_at']))
            if ready and not self.stop.is_set():
                record = ready[0]
                self._active_record_id = record['id']
                try:
                    self._upload(record)
                except (OSError, ValueError, RuntimeError) as error:
                    if record['kind'] != 'event' and self._needs_attention(error):
                        self.set_delivery(record['session_id'], 'failed', str(error))
                    else:
                        record.update(state='retry', error=str(error),
                                      retry_at=time.time() + min(300, 2 ** min(record['attempts'], 8)))
                        self._save(record)
                finally:
                    self._active_record_id = None
        finally:
            self.tick_lock.release()

    def _loop(self):
        """Keep asynchronous work and network errors away from flight callbacks."""
        while not self.stop.is_set():
            try:
                self.tick()
            except Exception as error:
                # A visible worker error must not terminate the independent media worker.
                print(f'Media upload worker: {error}', flush=True)
            self.wake.wait(2)
            self.wake.clear()

    def start(self):
        """Start the single background transfer worker idempotently."""
        if self.worker and self.worker.is_alive():
            return
        self.stop.clear()
        self.worker = threading.Thread(target=self._loop, name='media-upload', daemon=True)
        self.worker.start()

    def close(self):
        """Stop pending network activity and close SQLite after the worker exits."""
        self.stop.set()
        self.wake.set()
        process = self.current_process
        if process and process.poll() is None:
            self._stop_process(process)
        if self.worker:
            self.worker.join(timeout=5)
        if not self.worker or not self.worker.is_alive():
            with self.lock:
                self.db.close()
