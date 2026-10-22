"""Exercise durable media transfer, verified ingestion and public-file boundaries locally."""

import json
from pathlib import Path
import shlex
import time

import cv2
import numpy as np
import pytest
import yaml

from video_service.media_store import MediaStore, atomic_json, file_hash
from video_service.media_transfer import UploadQueue


class LocalQueue(UploadQueue):
    """Use actual rsync and rename commands against local directories without SSH."""

    def _run(self, args, timeout=900):
        """Translate the SSH command and remote rsync operands into local equivalents."""
        if args[0] == 'ssh':
            args = shlex.split(args[-1])
        else:
            args = [arg.removeprefix('local:') for arg in args]
        return super()._run(args, timeout)


def calibration_file(tmp_path):
    """Create a tiny five-coefficient calibration with visible nonzero distortion."""
    path = tmp_path / 'intrinsics.yaml'
    path.write_text(yaml.safe_dump({
        'image_width': 128, 'image_height': 72, 'distortion_model': 'plumb_bob',
        'camera_matrix': {'data': [100, 0, 63, 0, 101, 35, 0, 0, 1]},
        'distortion_coefficients': {'data': [-0.2, 0.03, 0.001, -0.002, 0.01]},
    }))
    return path


def test_photo_priority_ack_correction_and_cache_retention(tmp_path, monkeypatch):
    """Transfer with real rsync and confirm cleanup follows the ground hash receipt."""
    ground = tmp_path / 'ground with spaces'
    cache = tmp_path / 'cache'
    store = MediaStore(ground, 10**7, calibration_file(tmp_path))
    queue = LocalQueue(cache, 'drone-refresh', 'local', str(ground), 3600, 1024)
    video = cache / 'finished.mp4'
    video.write_bytes(b'original-video-container')
    image = cache / 'photo.jpg'
    pixels = np.random.default_rng(3).integers(0, 256, (72, 128, 3), dtype=np.uint8)
    assert cv2.imwrite(str(image), pixels)
    video_id = queue.enqueue(video, 'video', 'session-a', {'mission_sequence': 3})
    photo_id = queue.enqueue(image, 'image', 'session-a', {
        'source_id': 'refresh', 'mission_sequence': 3, 'waypoint_index': 7, 'photo_no': 21})
    assert all(record['sha256'] == '' for record in queue.records())
    queue.tick()
    assert {row['id']: row['state'] for row in queue.records()} == {
        video_id: 'queued', photo_id: 'awaiting_ack'}
    assert store.ingest_pending()[0]['id'] == photo_id
    assert (ground / 'ack' / 'drone-refresh' / (photo_id + '.json')).exists()
    assert store.get(photo_id)['state'] == 'pending'
    assert store.public_url(store.get(photo_id)) == ''
    queue.tick()
    assert next(row for row in queue.records() if row['id'] == photo_id)['state'] == 'acknowledged'
    assert image.exists()
    write_image = cv2.imwrite

    def write_private(path, *args):
        """Check incomplete corrected pixels never enter the FTP root."""
        assert Path(path).parent == ground / 'staging'
        assert not list((ground / 'public').rglob('*.jpg'))
        return write_image(path, *args)

    with monkeypatch.context() as patch:
        patch.setattr(cv2, 'imwrite', write_private)
        store.process_pending()
    record = store.get(photo_id)
    assert record['state'] == 'ready' and record['variant'] == 'experiment'
    assert record['metadata']['photo_no'] == 21 and record['metadata']['waypoint_index'] == 7
    decoded = cv2.imread(str(image))
    expected = cv2.remap(decoded, *store.photo_maps, cv2.INTER_LINEAR, borderMode=cv2.BORDER_REPLICATE)
    _, encoded = cv2.imencode('.jpg', expected, [cv2.IMWRITE_JPEG_QUALITY, 95])
    np.testing.assert_array_equal(cv2.imread(record['path']), cv2.imdecode(encoded, cv2.IMREAD_COLOR))
    assert store.public_url(record).startswith('drone-refresh/session-a/')
    assert store.ingest_pending()[0]['id'] == video_id
    assert store.get(video_id)['state'] == 'original_ready'
    assert not store.get(video_id)['path']
    queue.retain_seconds = 0
    queue.tick()
    assert not image.exists() and not video.exists()
    assert Path(record['original_path']).exists()
    store.ingest_pending()
    queue.tick()
    store.ingest_pending()
    assert store.get(photo_id)['cache_state'] == 'deleted'
    assert store.events()[0]['event'] == 'cache_deleted'
    queue.close()
    store.close()


def test_bad_hash_and_path_traversal_never_ack(tmp_path):
    """Reject corrupt originals, nested filenames and escaped session identifiers."""
    store = MediaStore(tmp_path / 'ground', 10000, calibration_file(tmp_path))
    inbox = store.root / 'inbox' / 'aircraft'
    inbox.mkdir()
    (inbox / 'media.jpg').write_bytes(b'wrong-content')
    manifest = dict(id='media', aircraft_id='aircraft', session_id='session', kind='image',
                    filename='media.jpg', size=13, sha256='0' * 64, created_at=time.time())
    atomic_json(inbox / 'media.json', manifest)
    assert store.ingest_pending()[0]['state'] == 'error'
    assert not store.records() and not list((store.root / 'ack').rglob('*.json'))
    for field, value in [('filename', '../escape.jpg'), ('session_id', '../../outside')]:
        atomic_json(inbox / 'media.json', dict(manifest, **{field: value}))
        assert store.ingest_pending()[0]['state'] == 'error'
    outside = tmp_path / 'outside.json'
    outside.write_text(json.dumps(dict(manifest, session_id='valid')))
    (inbox / 'media.json').unlink()
    (inbox / 'media.json').symlink_to(outside)
    assert store.ingest_pending()[0]['state'] == 'error'
    assert not store.records()
    store.close()


def test_retry_restart_and_duplicate_ingestion(tmp_path, monkeypatch):
    """Recover interrupted upload state and ingest the same ID only once."""
    ground, cache = tmp_path / 'ground', tmp_path / 'cache'
    store = MediaStore(ground, 100000, calibration_file(tmp_path))
    queue = LocalQueue(cache, 'new', 'local', str(ground), 3600, 0)
    source = cache / 'clip.mp4'
    source.write_bytes(b'video')
    record_id = queue.enqueue(source, 'video', 'session')
    with monkeypatch.context() as patch:
        patch.setattr(queue, '_ssh', lambda *args: (_ for _ in ()).throw(OSError('offline')))
        queue.tick()
    record = queue.records()[0]
    assert record['state'] == 'retry' and 'offline' in record['error']
    assert source.exists()
    record.update(state='uploading', retry_at=0)
    queue._save(record)
    queue.close()
    queue = LocalQueue(cache, 'new', 'local', str(ground), 3600, 0)
    assert queue.records()[0]['state'] == 'retry'
    queue.tick()
    manifest_path = ground / 'inbox' / 'new' / (record_id + '.json')
    manifest = json.loads(manifest_path.read_text())
    store.ingest_pending()
    # A worker crash after the move but before ACK simply republishes the manifest.
    atomic_json(manifest_path, manifest)
    assert store.ingest_pending()[0]['state'] == 'ingested'
    assert len(store.records()) == 1
    queue.tick()
    assert queue.records()[0]['state'] == 'acknowledged'
    queue.close()
    store.close()


def test_capacity_keeps_active_and_aircraft_ownership(tmp_path):
    """Evict oldest completed media without exposing originals or removing active work."""
    store = MediaStore(tmp_path / 'ground', 16, calibration_file(tmp_path))
    active = store.create_record('refresh', 'same-session')
    staging = store.root / 'staging' / 'active.mp4.tmp'
    staging.write_bytes(b'active')
    first = tmp_path / 'one.mp4'
    first.write_bytes(b'a' * 10)
    first_id = store.register_processed(first, 'refresh', 'same-session')
    second = tmp_path / 'two.mp4'
    second.write_bytes(b'b' * 10)
    second_id = store.register_processed(second, 'new', 'same-session', {'variant': 'rebuilt'})
    assert store.cleanup() == [first_id]
    assert store.get(first_id)['state'] == 'deleted'
    assert not store.public_url(store.get(first_id))
    assert store.get(active)['state'] == 'recording' and staging.exists()
    assert Path(store.get(second_id)['path']).is_file()
    with pytest.raises(ValueError, match='active'):
        store.delete(active)
    with pytest.raises(ValueError):
        store.update(second_id, path=str(tmp_path / 'outside.mp4'))
    store.close()


def test_photo_processing_failure_retains_acknowledged_original(tmp_path):
    """A photo decoding fault leaves an explicit state and never revokes original ACK."""
    ground = tmp_path / 'ground'
    store = MediaStore(ground, 100000, calibration_file(tmp_path))
    queue = LocalQueue(tmp_path / 'cache', 'new', 'local', str(ground), 3600, 0)
    image = tmp_path / 'invalid.jpg'
    image.write_bytes(b'invalid image')
    record_id = queue.enqueue(image, 'image', 'session')
    queue.tick()
    store.ingest_pending()
    assert (ground / 'ack' / 'new' / (record_id + '.json')).exists()
    store.process_pending()
    record = store.get(record_id)
    assert record['state'] == 'processing_error' and 'decode' in record['error']
    assert Path(record['original_path']).is_file()
    assert store.public_url(record) == ''
    queue.close()
    store.close()


def test_ack_loss_republishes_manifest(tmp_path):
    """A missing receipt retries its existing ID rather than waiting forever."""
    ground = tmp_path / 'ground'
    queue = LocalQueue(tmp_path / 'cache', 'refresh', 'local', str(ground), 0, 0)
    source = tmp_path / 'clip.mp4'
    source.write_bytes(b'original')
    record_id = queue.enqueue(source, 'video', 's')
    queue.tick()
    manifest = ground / 'inbox' / 'refresh' / (record_id + '.json')
    manifest.unlink()
    record = queue.records()[0]
    record['retry_at'] = 0
    queue._save(record)
    queue.tick()
    assert manifest.exists() and source.exists()
    assert file_hash(source) == queue.records()[0]['sha256']
    queue.close()


def test_retry_repairs_same_size_corrupt_payload(tmp_path):
    """Rsync must repair content corruption even when destination length already matches."""
    ground = tmp_path / 'ground'
    store = MediaStore(ground, 100000, calibration_file(tmp_path))
    queue = LocalQueue(tmp_path / 'cache', 'refresh', 'local', str(ground), 3600, 0)
    source = tmp_path / 'clip.mp4'
    source.write_bytes(b'original-data')
    record_id = queue.enqueue(source, 'video', 's')
    queue.tick()
    payload = ground / 'inbox' / 'refresh' / (record_id + '.mp4')
    payload.write_bytes(b'x' * source.stat().st_size)
    assert store.ingest_pending()[0]['state'] == 'error'
    record = queue.records()[0]
    record['retry_at'] = 0
    queue._save(record)
    queue.tick()
    assert store.ingest_pending()[0]['state'] == 'ingested'
    assert file_hash(Path(store.get(record_id)['original_path'])) == file_hash(source)
    queue.close()
    store.close()


def test_eviction_before_ack_restores_original_without_stale_receipt(tmp_path):
    """Eviction must not leave a retry unindexed or allow an obsolete cached ACK."""
    ground = tmp_path / 'ground'
    store = MediaStore(ground, 100000, calibration_file(tmp_path))
    queue = LocalQueue(tmp_path / 'cache', 'refresh', 'local', str(ground), 0, 0)
    source = tmp_path / 'clip.mp4'
    source.write_bytes(b'original')
    record_id = queue.enqueue(source, 'video', 's')
    queue.tick()
    manifest_path = ground / 'inbox' / 'refresh' / (record_id + '.json')
    manifest = json.loads(manifest_path.read_text())
    store.ingest_pending()
    receipt = json.loads((ground / 'ack' / 'refresh' / (record_id + '.json')).read_text())
    atomic_json(queue.root / 'acks' / (record_id + '.json'), receipt)
    store.delete(record_id)
    # The payload-free retry cannot ACK an already evicted original.
    atomic_json(manifest_path, manifest)
    assert store.ingest_pending()[0]['state'] == 'error'
    queue.tick()
    assert source.exists() and queue.records()[0]['state'] == 'awaiting_ack'
    record = queue.records()[0]
    record['retry_at'] = 0
    queue._save(record)
    queue.tick()
    assert store.ingest_pending()[0]['state'] == 'ingested'
    restored = store.get(record_id)
    assert restored['state'] == 'original_ready' and Path(restored['original_path']).read_bytes() == b'original'
    queue.tick()
    assert not source.exists() and Path(restored['original_path']).exists()
    queue.close()
    store.close()


def test_session_calibration_is_frozen_across_restart(tmp_path):
    """Configuration edits apply to the next session while old photos retain their lens model."""
    calibration = calibration_file(tmp_path)
    store = MediaStore(tmp_path / 'ground', 10**7, calibration)
    first = store.session_intrinsics('refresh', 'first')
    frozen = first.read_bytes()
    data = yaml.safe_load(calibration.read_text())
    data['distortion_coefficients']['data'][0] = -0.1
    calibration.write_text(yaml.safe_dump(data))
    assert store.session_intrinsics('refresh', 'first').read_bytes() == frozen
    assert store.session_intrinsics('refresh', 'second').read_bytes() != frozen
    store.close()
    store = MediaStore(tmp_path / 'ground', 10**7, calibration)
    assert store.session_intrinsics('refresh', 'first').read_bytes() == frozen
    store.close()


def test_upload_summary_has_pending_and_cleanup_states_without_paths(tmp_path):
    """Ground notices can distinguish pending uploads without exposing aircraft paths."""
    queue = LocalQueue(tmp_path / 'cache', 'refresh', 'local', str(tmp_path / 'ground'), 0, 0)
    source = tmp_path / 'photo.jpg'
    source.write_bytes(b'photo')
    record_id = queue.enqueue(source, 'image', 's', {'photo_no': 12, 'waypoint_index': 3})
    queue.enqueue_event('start', 's')
    summary = queue.summary('s')
    assert len(summary) == 1 and summary[0]['id'] == record_id
    assert summary[0]['state'] == 'queued' and summary[0]['cache_state'] == 'retained'
    assert 'path' not in summary[0] and summary[0]['metadata']['photo_no'] == 12
    assert queue.summary('other-session') == []
    queue.close()


def test_capacity_grace_preserves_receipt_then_evicts_original(tmp_path, monkeypatch):
    """An over-capacity arrival gets a short ACK window and a visible storage warning."""
    ground = tmp_path / 'ground'
    store = MediaStore(ground, 4, calibration_file(tmp_path))
    queue = LocalQueue(tmp_path / 'cache', 'refresh', 'local', str(ground), 3600, 0)
    source = tmp_path / 'clip.mp4'
    source.write_bytes(b'original')
    record_id = queue.enqueue(source, 'video', 's')
    queue.tick()
    store.ingest_pending()
    assert store.cleanup() == []
    assert store.storage()['over_limit'] and store.storage()['error']
    assert (ground / 'ack' / 'refresh' / (record_id + '.json')).exists()
    future = time.time() + 61
    monkeypatch.setattr('video_service.media_store.time.time', lambda: future)
    assert store.cleanup() == [record_id]
    assert not store.storage()['over_limit']
    queue.close()
    store.close()


def test_failed_recording_staging_is_managed_but_active_is_preserved(tmp_path):
    """Failed recordings are reclaimable by their ID without deleting another active file."""
    store = MediaStore(tmp_path / 'ground', 10, calibration_file(tmp_path))
    failed = store.create_record('refresh', 'failed')
    active = store.create_record('refresh', 'active')
    failed_path = store.root / 'staging' / (failed + '.mkv')
    active_path = store.root / 'staging' / (active + '.mkv')
    failed_path.write_bytes(b'a' * 10)
    active_path.write_bytes(b'b' * 5)
    store.update(failed, state='error', error='Recording interrupted')
    assert store.cleanup() == [failed]
    assert not failed_path.exists() and active_path.exists()
    store.close()
