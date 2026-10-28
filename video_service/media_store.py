"""Persistent ground media inventory, verified inbox ingestion and photo correction."""

import hashlib
from datetime import datetime
import json
import logging
import os
from pathlib import Path
import re
import shutil
import sqlite3
import threading
import time
import uuid

# Only completed records may be removed by the automatic capacity policy.
ACTIVE_STATES = {'recording', 'uploading', 'processing', 'pending'}
# Give the aircraft time to retrieve its receipt before evicting a newly received original.
INGEST_GRACE_SECONDS = 60


def safe_component(value):
    """Reject path syntax while retaining ordinary aircraft and session identifiers."""
    value = str(value)
    if value in {'.', '..'} or not re.fullmatch(r'[A-Za-z0-9_.-]{1,160}', value):
        raise ValueError(f'Invalid media identifier: {value!r}')
    return value


def file_hash(path):
    """Hash a finalized file without loading it entirely into memory."""
    digest = hashlib.sha256()
    with Path(path).open('rb') as source:
        for block in iter(lambda: source.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def safe_media_filename(value):
    """Preserve camera photoNo percent encoding while excluding path syntax."""
    value = str(value)
    if (not value or value in {'.', '..'} or len(value.encode('utf-8')) > 255
            or re.search(r'[/\\\x00-\x1f\x7f]', value)):
        raise ValueError(f'Invalid media filename: {value!r}')
    return value


def atomic_json(path, value):
    """Publish a complete JSON document using a same-directory rename."""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name + '.tmp')
    with temporary.open('w') as output:
        json.dump(value, output, ensure_ascii=False)
        output.flush()
        os.fsync(output.fileno())
    temporary.replace(path)


class MediaStore:
    """Keep original files private and publish only finalized corrected media."""

    def __init__(self, root: Path, max_bytes: int, intrinsics: Path):
        """Open the persistent index; processing does not require a GUI session."""
        self.root = Path(root).resolve()
        self.max_bytes = int(max_bytes)
        self.intrinsics = Path(intrinsics)
        self.lock = threading.RLock()
        self.photo_lock = threading.Lock()
        self.calibration = None
        self.photo_maps = None
        self.photo_session = None
        self.stopped_sessions = set()
        for name in ('inbox', 'internal', 'public', 'ack', 'staging'):
            (self.root / name).mkdir(parents=True, exist_ok=True)
        self.db = sqlite3.connect(self.root / 'media.sqlite3', check_same_thread=False)
        self.db.execute('PRAGMA journal_mode=WAL')
        self.db.execute('CREATE TABLE IF NOT EXISTS records (id TEXT PRIMARY KEY, data TEXT NOT NULL)')
        self.db.execute('CREATE TABLE IF NOT EXISTS events (id TEXT PRIMARY KEY, data TEXT NOT NULL)')
        self.db.commit()
        # Interrupted photo work is repeatable from its already verified original.
        for record in self.records():
            if record['kind'] in {'image', 'photo'} and record['state'] == 'processing':
                self.update(record['id'], state='pending')

    def _save(self, record):
        """Commit one complete record while serializing access to SQLite."""
        with self.lock, self.db:
            self.db.execute('INSERT OR REPLACE INTO records VALUES (?, ?)',
                            (record['id'], json.dumps(record, ensure_ascii=False)))

    def records(self):
        """Return snapshots ordered oldest first for GUI and cleanup consumers."""
        with self.lock:
            rows = self.db.execute('SELECT data FROM records').fetchall()
        return sorted((json.loads(row[0]) for row in rows), key=lambda row: row['created_at'])

    def get(self, record_id):
        """Return one inventory record or None."""
        with self.lock:
            row = self.db.execute('SELECT data FROM records WHERE id=?', (record_id,)).fetchone()
        return json.loads(row[0]) if row else None

    def update(self, record_id, **values):
        """Update explicit inventory fields and retain metadata as a separate mapping."""
        with self.lock:
            record = self.get(record_id)
            if record is None:
                raise KeyError(record_id)
            allowed = {'state', 'variant', 'path', 'processed_path', 'original_path', 'error',
                       'metadata', 'cache_state', 'size', 'sha256', 'processed_size', 'ingested_at'}
            if set(values) - allowed:
                raise ValueError(f'Unknown record fields: {set(values) - allowed}')
            for field, prefix in [('path', 'public'), ('processed_path', 'public'),
                                  ('original_path', 'internal')]:
                if values.get(field):
                    self._inside(values[field], prefix)
            record.update(values)
            if 'path' in values:
                record['processed_path'] = values['path']
            elif 'processed_path' in values:
                record['path'] = values['processed_path']
            record['updated_at'] = time.time()
            self._save(record)
            return record

    def _inside(self, path, prefix):
        """Resolve every managed path beneath its expected private/public directory."""
        path = Path(path).resolve()
        path.relative_to((self.root / prefix).resolve())
        return path

    def session_intrinsics(self, aircraft_id, session_id):
        """Freeze the first session calibration for live, still-image and rebuilt outputs."""
        path = self.root / 'internal' / safe_component(aircraft_id) / safe_component(session_id) / 'intrinsics.yaml'
        with self.lock:
            if not path.exists():
                path.parent.mkdir(parents=True, exist_ok=True)
                temporary = path.with_name(path.name + '.tmp')
                shutil.copyfile(self.intrinsics, temporary)
                temporary.replace(path)
        return path

    def create_record(self, aircraft_id, session_id, kind='video', metadata=None,
                      variant='experiment'):
        """Reserve an active record before a ground recording starts."""
        record_id = uuid.uuid4().hex
        now = time.time()
        self._save(dict(id=record_id, aircraft_id=safe_component(aircraft_id),
                        session_id=safe_component(session_id), kind=kind, variant=variant,
                        state='recording', metadata=metadata or {}, original_path='', path='',
                        processed_path='', created_at=now, updated_at=now, size=0,
                        processed_size=0, cache_state='unknown', error=''))
        return record_id

    def _media_path(self, prefix, record, suffix):
        """Use the original camera name, or the same timestamp format for live fragments."""
        filename = record.get('filename')
        if not filename:
            stamp = datetime.fromtimestamp(record['created_at']).strftime('%Y%m%d-%H%M%S-%f')
            filename = ('snapshot-' if record['kind'] in {'image', 'photo'} else 'recording-') + stamp + suffix
        filename = safe_media_filename(str(Path(filename).with_suffix(suffix)))
        directory = self.root / prefix / record['aircraft_id'] / record['session_id']
        destination = directory / filename
        # Distinct records may be rebuilt or uploaded with the same human filename.
        occupied = {row.get('original_path' if prefix == 'internal' else 'path')
                    for row in self.records() if row['id'] != record['id']}
        number = 2
        while str(destination) in occupied:
            destination = directory / f'{Path(filename).stem}-{number}{suffix}'
            number += 1
        return destination

    def register_processed(self, path, aircraft_id, session_id, metadata=None, kind='video',
                           record_id=None):
        """Move a finalized corrected recording into public storage and publish its record."""
        with self.lock:
            if record_id is None:
                record_id = self.create_record(aircraft_id, session_id, kind, metadata)
            record = self.get(record_id)
            if record is None or (record['aircraft_id'], record['session_id']) != (
                    safe_component(aircraft_id), safe_component(session_id)):
                raise ValueError('Processed recording ownership does not match its record')
            source = Path(path).resolve()
            suffix = source.suffix.lower()
            if suffix not in {'.mp4', '.mkv', '.avi', '.mov', '.ts', '.jpg', '.jpeg', '.png'}:
                raise ValueError('Finalized media needs its real media extension')
            destination = self._media_path('public', record, suffix)
            destination.parent.mkdir(parents=True, exist_ok=True)
            if source != destination:
                staged = self.root / 'staging' / (record_id + suffix)
                if source != staged:
                    shutil.move(str(source), str(staged))
                staged.replace(destination)
            info = dict(record['metadata'], **(metadata or {}))
            return self.update(record_id, path=str(destination), processed_size=destination.stat().st_size,
                               state=info.get('state', 'ready'), variant=info.get('variant', record['variant']),
                               metadata=info, error=info.get('error', ''))['id']

    def public_url(self, record):
        """Return a public relative path only for an existing finalized processed file."""
        if not record.get('path') or record['state'] in ACTIVE_STATES:
            return ''
        path = self._inside(record['path'], 'public')
        return path.relative_to(self.root / 'public').as_posix() if path.is_file() else ''

    def events(self):
        """Return persistent aircraft session events in chronological order."""
        with self.lock:
            rows = self.db.execute('SELECT data FROM events').fetchall()
        return sorted((json.loads(row[0]) for row in rows), key=lambda event: event['time'])

    def stop_session(self, aircraft, session):
        """Stop automatic ingestion/photo work while retaining every existing source file."""
        with self.lock:
            self.stopped_sessions.add((aircraft, session))
            for record in self.records():
                if ((record['aircraft_id'], record['session_id']) == (aircraft, session)
                        and record['state'] in ACTIVE_STATES):
                    self.update(record['id'], state='error', error='交付已终止，不再自动处理')

    def _event(self, manifest):
        """Persist a session notification and apply cache-deletion state changes."""
        event = dict(id=manifest['id'], time=manifest['created_at'],
                     aircraft_id=manifest['aircraft_id'], session_id=manifest['session_id'],
                     event=manifest['event'], metadata=manifest.get('metadata', {}))
        with self.lock, self.db:
            self.db.execute('INSERT OR IGNORE INTO events VALUES (?, ?)',
                            (event['id'], json.dumps(event, ensure_ascii=False)))
        if event['event'] == 'cache_deleted':
            record = self.get(event['metadata'].get('record_id'))
            if record and record['aircraft_id'] == event['aircraft_id']:
                self.update(record['id'], cache_state='deleted')

    def ingest_pending(self):
        """Verify inbox payloads and ACK original persistence before photo processing."""
        results = []
        with self.lock:
            for manifest_path in sorted((self.root / 'inbox').glob('*/*.json')):
                try:
                    self._inside(manifest_path, 'inbox')
                    manifest = json.loads(manifest_path.read_text())
                    aircraft = safe_component(manifest['aircraft_id'])
                    session = safe_component(manifest['session_id'])
                    record_id = safe_component(manifest['id'])
                    if (aircraft, session) in self.stopped_sessions and manifest.get('kind') != 'event':
                        continue
                    if manifest_path.parent.name != aircraft or manifest_path.stem != record_id:
                        raise ValueError('Manifest location does not match its identity')
                    if manifest.get('kind') == 'event':
                        self._event(manifest)
                    else:
                        self._ingest_file(manifest, manifest_path, aircraft, session, record_id)
                    atomic_json(self.root / 'ack' / aircraft / (record_id + '.json'),
                                {'id': record_id, 'sha256': manifest.get('sha256', ''),
                                 'size': manifest.get('size', 0), 'ingested_at': time.time()})
                    manifest_path.unlink()
                    results.append({'id': record_id, 'state': 'ingested'})
                except (ValueError, KeyError, TypeError, OSError) as error:
                    # Retain bad/incomplete inputs so the operator can inspect or retry them.
                    results.append({'manifest': str(manifest_path), 'state': 'error', 'error': str(error)})
        return results

    def _ingest_file(self, manifest, manifest_path, aircraft, session, record_id):
        """Hash a payload and atomically retain its original under its stable business ID."""
        if manifest['kind'] not in {'image', 'photo', 'video'}:
            raise ValueError('Unsupported media kind')
        filename = safe_component(manifest['filename'])
        source = self._inside(manifest_path.parent / filename, 'inbox')
        if source.parent != manifest_path.parent.resolve():
            raise ValueError('Payload must be beside its manifest')
        existing = self.get(record_id)
        if existing and (existing['aircraft_id'] != aircraft or existing['session_id'] != session
                         or existing.get('sha256') != manifest['sha256']):
            raise ValueError('Existing media identity differs from incoming manifest')
        # A retry after the original move may find the payload only at its final path.
        retained = Path(existing['original_path']) if existing and existing.get('original_path') else None
        candidate = source if source.is_file() else retained
        if candidate is None or not candidate.is_file():
            raise FileNotFoundError('Original is absent; awaiting payload retransmission')
        if candidate.stat().st_size != int(manifest['size']) or file_hash(candidate) != manifest['sha256']:
            raise ValueError('Payload size or SHA-256 mismatch')
        record_name = safe_media_filename(manifest.get('original_filename', filename))
        descriptor = dict(id=record_id, aircraft_id=aircraft, session_id=session,
                          filename=record_name, kind=manifest['kind'])
        destination = self._media_path('internal', descriptor, source.suffix.lower())
        destination.parent.mkdir(parents=True, exist_ok=True)
        if not existing:
            now = time.time()
            self._save(dict(id=record_id, aircraft_id=aircraft, session_id=session,
                            filename=record_name,
                            kind='image' if manifest['kind'] == 'photo' else manifest['kind'], variant='original',
                            state='pending' if manifest['kind'] in {'image', 'photo'} else 'original_ready',
                            original_path=str(destination), path='', processed_path='',
                            created_at=manifest.get('created_at', now), updated_at=now,
                            ingested_at=now,
                            size=int(manifest['size']), sha256=manifest['sha256'], processed_size=0,
                            metadata=manifest.get('metadata', {}), cache_state='retained', error=''))
        elif not retained or not retained.is_file():
            # Capacity deletion is a tombstone, not a reason to lose an incoming replacement.
            self.update(record_id, original_path=str(destination), size=int(manifest['size']),
                        state='pending' if manifest['kind'] in {'image', 'photo'} else 'original_ready',
                        cache_state='retained', error='', ingested_at=time.time())
        if source.is_file():
            source.replace(destination)
        with Path(self.get(record_id)['original_path']).open('rb') as original:
            os.fsync(original.fileno())
        if not existing:
            logging.info('Media original received: id=%s aircraft=%s session=%s file=%s',
                         record_id, aircraft, session, destination.name)

    def process_pending(self):
        """Correct queued still images using the relay's five coefficients and alpha zero."""
        import cv2
        from .relay_calibration import Calibration
        results = []
        if not self.photo_lock.acquire(blocking=False):
            return results
        try:
            for record in self.records():
                if record['kind'] not in {'image', 'photo'} or record['state'] != 'pending':
                    continue
                if (record['aircraft_id'], record['session_id']) in self.stopped_sessions:
                    continue
                try:
                    self.update(record['id'], state='processing')
                    calibration_path = self.session_intrinsics(record['aircraft_id'], record['session_id'])
                    key = record['aircraft_id'], record['session_id']
                    if key != self.photo_session:
                        # Keep only the latest session's maps in this long-running service.
                        self.calibration = Calibration(calibration_path, 'left')
                        self.photo_maps = self.calibration.maps()
                        self.photo_session = key
                    image = cv2.imread(record['original_path'])
                    if image is None:
                        raise ValueError('Cannot decode original photo')
                    if image.shape[:2] != (self.calibration.height, self.calibration.width):
                        raise ValueError('Photo resolution differs from camera intrinsics')
                    corrected = cv2.remap(image, *self.photo_maps, cv2.INTER_LINEAR,
                                          borderMode=cv2.BORDER_REPLICATE)
                    destination = self._media_path('public', record, '.jpg')
                    destination.parent.mkdir(parents=True, exist_ok=True)
                    temporary = self.root / 'staging' / (record['id'] + '.jpg')
                    if not cv2.imwrite(str(temporary), corrected, [cv2.IMWRITE_JPEG_QUALITY, 95]):
                        raise OSError('Cannot write corrected photo')
                    with self.lock:
                        if (record['aircraft_id'], record['session_id']) in self.stopped_sessions:
                            continue
                        temporary.replace(destination)
                        results.append(self.update(record['id'], path=str(destination),
                                                   processed_size=destination.stat().st_size,
                                                   state='ready', variant='experiment', error=''))
                    logging.info('Photo ready: id=%s session=%s mission=%s waypoint=%s file=%s',
                                 record['id'], record['session_id'], record['metadata'].get('mission_sequence'),
                                 record['metadata'].get('waypoint_index'), destination.name)
                except (ValueError, OSError, cv2.error) as error:
                    results.append(self.update(record['id'], state='processing_error', error=str(error)))
                    logging.error('Photo processing failed: id=%s session=%s error=%s',
                                  record['id'], record['session_id'], error)
        finally:
            self.photo_lock.release()
        return results

    def storage(self):
        """Count managed media including inbox and active staging, excluding SQLite/logs."""
        sizes = {}
        for name in ('internal', 'public', 'inbox', 'staging'):
            sizes[name] = 0
            for path in (self.root / name).rglob('*'):
                try:
                    if path.is_file() and not path.is_symlink():
                        sizes[name] += path.stat().st_size
                except FileNotFoundError:
                    # Workers publish and remove files concurrently with this inventory.
                    continue
        used = sum(sizes.values())
        over_limit = self.max_bytes > 0 and used > self.max_bytes
        return dict(bytes=used, max_bytes=self.max_bytes, directories=sizes, over_limit=over_limit,
                    error='地面媒体存储超过容量上限；活动文件或新入库原片暂受保护' if over_limit else '',
                    scope='internal/public/inbox/staging (includes active media; excludes indexes and logs)')

    def delete(self, record_id):
        """Remove a completed record's media and retain its deletion history."""
        with self.lock:
            record = self.get(record_id)
            if not record:
                return False
            if record['state'] in ACTIVE_STATES:
                raise ValueError('Cannot delete active media')
            # Revoke the receipt before deleting its original, including across a restart.
            (self.root / 'ack' / record['aircraft_id'] / (record_id + '.json')).unlink(missing_ok=True)
            for field, prefix in [('original_path', 'internal'), ('path', 'public')]:
                if record.get(field):
                    self._inside(record[field], prefix).unlink(missing_ok=True)
            for staged in (self.root / 'staging').glob(record_id + '.*'):
                if staged.is_file():
                    self._inside(staged, 'staging').unlink(missing_ok=True)
            self.update(record_id, original_path='', path='', size=0, processed_size=0,
                        state='deleted')
            return True

    def cleanup(self):
        """Delete oldest inactive media until the configured total capacity is met."""
        removed = []
        with self.lock:
            used = self.storage()['bytes']
            if self.max_bytes <= 0:
                return removed
            for record in self.records():
                if used <= self.max_bytes:
                    break
                if record['state'] in ACTIVE_STATES or record['state'] == 'deleted':
                    continue
                if (record.get('original_path') and
                        time.time() - record.get('ingested_at', 0) < INGEST_GRACE_SECONDS):
                    continue
                self.delete(record['id'])
                removed.append(record['id'])
                used = self.storage()['bytes']
        return removed

    def close(self):
        """Close the persistent index after background workers have stopped."""
        with self.lock:
            self.db.close()
