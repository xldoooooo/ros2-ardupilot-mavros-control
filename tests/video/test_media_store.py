"""Exercise durable media transfer, verified ingestion and public-file boundaries locally."""

import json
import io
from ftplib import FTP
from pathlib import Path
import subprocess
import time

import cv2
import numpy as np
import pytest
import yaml

from video_service.media_store import MediaStore, atomic_json, file_hash
from video_service.media_transfer import UploadQueue
from video_service.ground_media import GroundMedia
from ground_station_core.event_log import EventLog
from ground_station_core.models import CommandResult, VideoCaptureEvent
from ground_station_core.upstream.status_projector import UpstreamStatusProjector
from tests.video.support import LocalQueue, calibration_file


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


def test_serial_inspections_upload_finalize_and_download_their_single_08_video(tmp_path):
    """Deliver both rounds' photos and video over real rsync/FTP despite reused capture counters."""
    ground = GroundMedia({
        'root': str(tmp_path / 'ground'), 'intrinsics': str(calibration_file(tmp_path)),
        'public_host': '127.0.0.1', 'listen': '127.0.0.1', 'ftp_port': 0,
        'ftp_passive_ports': [0, 0], 'backend': 'cpu', 'cpu_threads': 2,
    })
    queue = LocalQueue(tmp_path / 'cache', 'uav002', 'local', str(ground.root), 3600, 0)
    sent = []
    projector = UpstreamStatusProjector(
        lambda: 'UAV01001', lambda payload: not sent.append(dict(payload)), EventLog()
    )
    paths = []
    try:
        ground._start_ftp()
        for ticket, color in ((8, 'red'), (18, 'blue')):
            session = f'mission-20261027-1633{ticket:02}'
            metadata = {'source_id': 'serial-gui', 'mission_sequence': ticket}
            notice = dict(metadata, aircraft_id='uav002', session_id=session, started_at=time.time())
            ground.session(dict(notice, action='start'))
            ground.session(dict(notice, action='end', ended_at=time.time()))
            source = tmp_path / f'recording-20261027-1633{ticket:02}-000000.mp4'
            subprocess.run([
                'ffmpeg', '-v', 'error', '-f', 'lavfi', '-i', f'color={color}:s=128x72:r=10',
                '-t', '1.2', '-c:v', 'libx264', '-threads', '1', str(source),
            ], check=True, capture_output=True)
            photo = tmp_path / f'snapshot-20261027-1633{ticket:02}-000001-photoNo-%E6%B5%8B%E8%AF%95.jpg'
            assert cv2.imwrite(str(photo), np.full((72, 128, 3), 120, dtype=np.uint8))
            queue.enqueue(source, 'video', session, metadata)
            queue.enqueue(photo, 'image', session, dict(
                metadata, sequence=42, waypoint_index=1, photo_no='测试'))
            for _ in range(2):
                queue.tick()
                assert all(row['state'] == 'ingested' for row in ground.store.ingest_pending())
            ground.store.process_pending()
            inventory = ground.inventory()['records']
            images = [row for row in inventory if row['session_id'] == session and row['kind'] == 'image']
            assert len(images) == 1 and images[0]['state'] == 'ready'
            assert images[0]['mission_sequence'] == ticket
            assert images[0]['source_id'] == 'serial-gui' and images[0]['metadata']['sequence'] == 42
            assert Path(images[0]['path']).name == photo.name
            assert Path(images[0]['original_path']).name == photo.name

            projector.begin_mission(ticket, 'inspection', (1,), source_id='serial-gui')
            projector.observe_video_capture(VideoCaptureEvent(1, 'serial-gui', 42, True, 2, ticket, 1, '测试', '', ''))
            projector.observe_result(CommandResult(1, ticket, 'waypoints', True, '完成'))
            projector.begin_landing(ticket + 1)
            projector.observe_result(CommandResult(2, ticket + 1, 'land', True, '落地'))
            before = sum(row['uavStatus'] == '08' for row in sent)
            photos_before = sum(row['uavStatus'] == '09' for row in sent)
            projector.observe_ground_media(tuple(ground.inventory()['records']))
            assert sum(row['uavStatus'] == '08' for row in sent) == before
            assert sum(row['uavStatus'] == '09' for row in sent) == photos_before + 1
            assert sent[-1]['data']['pointPic'] == images[0]['ftp_path']
            photo_bytes = io.BytesIO()
            with FTP() as ftp:
                ftp.connect('127.0.0.1', ground.ftp.socket.getsockname()[1], timeout=5)
                ftp.login()
                ftp.retrbinary('RETR ' + images[0]['ftp_path'], photo_bytes.write)
            decoded = cv2.imdecode(np.frombuffer(photo_bytes.getvalue(), dtype=np.uint8), cv2.IMREAD_COLOR)
            assert decoded is not None and decoded.shape == (72, 128, 3)

            ground._finalize_uploaded_videos()
            ground._finalize_uploaded_videos()
            deadline = time.monotonic() + 30
            while time.monotonic() < deadline:
                videos = [row for row in ground.store.records()
                          if row['session_id'] == session and row['variant'] == 'rebuilt']
                if videos and videos[0]['state'] in ('ready', 'error'):
                    break
                time.sleep(.05)
            assert len(videos) == 1 and videos[0]['state'] == 'ready', videos
            inventory = tuple(ground.inventory()['records'])
            projector.observe_ground_media(inventory)
            projector.observe_ground_media(inventory)
            assert sum(row['uavStatus'] == '08' for row in sent) == before + 1
            message = sent[-1]
            assert message['uavStatus'] == '08' and set(message['data']) == {'videoPath', 'JPGPath'}
            path = message['data']['videoPath']
            assert path.startswith(f'/uav002/{session}/') and path.endswith('.mp4')
            paths.append(path)
            public = next(row for row in inventory if row['ftp_path'] == path)
            assert Path(public['path']).name.startswith('recording-')
            assert public['download_url'].startswith('http://')
            downloaded = io.BytesIO()
            with FTP() as ftp:
                ftp.connect('127.0.0.1', ground.ftp.socket.getsockname()[1], timeout=5)
                ftp.login()
                ftp.retrbinary('RETR ' + path, downloaded.write)
            assert downloaded.getvalue() == Path(public['path']).read_bytes()
            received = tmp_path / f'downloaded-{ticket}.mp4'
            received.write_bytes(downloaded.getvalue())
            probe = subprocess.run([
                'ffprobe', '-v', 'error', '-count_frames', '-select_streams', 'v:0',
                '-show_entries', 'stream=nb_read_frames', '-of', 'json', str(received),
            ], check=True, capture_output=True, text=True)
            assert int(json.loads(probe.stdout)['streams'][0]['nb_read_frames']) == 12
            capture = cv2.VideoCapture(str(received))
            ok, frame = capture.read()
            capture.release()
            assert ok
            blue, _, red = frame.mean(axis=(0, 1))
            assert red > blue if color == 'red' else blue > red
        assert len(set(paths)) == 2
    finally:
        queue.close()
        ground.close()
        ground.store.close()
