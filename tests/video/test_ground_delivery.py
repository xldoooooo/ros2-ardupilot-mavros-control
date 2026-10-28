"""Verify live-first video delivery, original fallback and persistent terminal failures."""

import json
from pathlib import Path
import subprocess
import time

import pytest

from video_service.ground_media import GroundMedia
from tests.video.support import LocalQueue, calibration_file


def ground(tmp_path, timeout=600):
    """Create an isolated receiver with CPU encoding and no listeners or aircraft."""
    return GroundMedia({'root': str(tmp_path / 'ground'), 'intrinsics': str(calibration_file(tmp_path)),
                        'public_host': '127.0.0.1', 'backend': 'cpu', 'cpu_threads': 1,
                        'delivery_timeout_seconds': timeout})


def notice(session='session-one', mission=8, action='end'):
    """Identify one ended inspection without any ROS flight command."""
    return dict(aircraft_id='uav002', session_id=session, source_id='delivery-gui',
                mission_sequence=mission, started_at=time.time() - 5, ended_at=time.time(), action=action)


def video(path, color='red'):
    """Generate twelve real H.264 frames for decoding and packet-copy merge checks."""
    subprocess.run(['ffmpeg', '-v', 'error', '-f', 'lavfi', '-i', f'color={color}:s=128x72:r=10',
                    '-t', '1.2', '-c:v', 'libx264', '-threads', '1', str(path)], check=True, capture_output=True)
    return path


def live_record(media, tmp_path, payload, number=1, corrupt=False):
    """Finalize a local recording through the actual decoder validation path."""
    record_id = media.store.create_record(payload['aircraft_id'], payload['session_id'], metadata=payload)
    partial = tmp_path / f'live-{number}.mkv'
    if corrupt:
        partial.write_bytes(b'not-a-video')
    else:
        video(partial, 'red' if number == 1 else 'blue')
    snapshot = dict(aircraft_id=payload['aircraft_id'], session_id=payload['session_id'],
                    record_id=record_id, partial=str(partial), metadata=payload,
                    interruptions=['网络中断，有时间缺口'], options={}, received_at=time.time() - 5,
                    finished_at=time.time(), record_started=None)
    media._validate_recording(snapshot)
    return record_id


def wait_delivery(media, state, timeout=15):
    """Advance the real delivery scheduler while asynchronous encoding finishes."""
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        media._finalize_uploaded_videos()
        result = media.delivery_status('uav002', 'session-one')
        if result['state'] == state:
            return result
        time.sleep(.03)
    raise AssertionError(result)


def close(media):
    """Wait for workers and close the isolated SQLite catalog."""
    media.close()
    media.store.close()


def test_incomplete_live_video_finishes_without_original_upload(tmp_path):
    """A playable local file with missing time is immediately selected and survives restart."""
    media = ground(tmp_path)
    payload = notice()
    try:
        media.session(payload)
        selected = live_record(media, tmp_path, payload)
        result = wait_delivery(media, 'ready')
        assert result['record_id'] == selected
        assert media.store.get(selected)['state'] == 'incomplete'
        assert not any(r['variant'] == 'rebuilt' for r in media.store.records())
    finally:
        close(media)
    media = ground(tmp_path)
    try:
        assert media.delivery_status('uav002', 'session-one')['state'] == 'ready'
    finally:
        close(media)


def test_multiple_live_fragments_are_joined_without_original(tmp_path):
    """Keep every playable fragment in one public file without hiding the interruption."""
    media = ground(tmp_path)
    payload = notice()
    try:
        media.session(payload)
        live_record(media, tmp_path, payload, 1)
        live_record(media, tmp_path, payload, 2)
        result = wait_delivery(media, 'ready')
        row = media.store.get(result['record_id'])
        assert row['variant'] == 'experiment' and row['state'] == 'incomplete'
        assert row['metadata']['assembled']
        probe = subprocess.run(['ffprobe', '-v', 'error', '-count_frames', '-select_streams', 'v:0',
                                '-show_entries', 'stream=nb_read_frames', '-of', 'json', row['path']],
                               check=True, capture_output=True, text=True)
        assert int(json.loads(probe.stdout)['streams'][0]['nb_read_frames']) == 24
    finally:
        close(media)


def test_corrupt_live_requests_original_and_builds_real_playable_fallback(tmp_path):
    """Only a failed local decode opens the held original upload, then correct it once."""
    media = ground(tmp_path)
    queue = LocalQueue(tmp_path / 'cache', 'uav002', 'local', str(media.root), 3600, 0)
    payload = notice()
    try:
        media.session(payload)
        bad = live_record(media, tmp_path, payload, corrupt=True)
        assert media.store.get(bad)['state'] == 'error'
        source = video(tmp_path / 'original.mp4')
        record_id = queue.enqueue(source, 'video', 'session-one', payload, held=True)
        queue.tick()
        assert queue.records()[0]['state'] == 'held'
        wait_delivery(media, 'fallback')
        queue.set_delivery('session-one', 'fallback')
        queue.tick()
        media.store.ingest_pending()
        result = wait_delivery(media, 'ready')
        delivered = media.store.get(result['record_id'])
        assert delivered['variant'] == 'rebuilt' and Path(delivered['path']).is_file()
        queue.set_delivery('session-one', result['state'])
        assert next(r for r in queue.records() if r['id'] == record_id)['state'] == 'retained'
    finally:
        queue.close()
        close(media)


@pytest.mark.parametrize('cause', ['manual', 'timeout'])
def test_failure_stops_retry_and_processing_across_restart(tmp_path, cause):
    """Manual intervention or elapsed landing deadline freezes this round, not the next."""
    media = ground(tmp_path, timeout=1)
    payload = notice()
    try:
        media.session(payload)
        if cause == 'timeout':
            media._deliveries['uav002/session-one']['ended_at'] -= 2
        else:
            media._uploads['uav002'] = [dict(session_id='session-one', kind='video',
                                           state='needs_attention', error='Permission denied')]
        wait_delivery(media, 'failed')
        assert ('uav002', 'session-one') in media.store.stopped_sessions
        restored = media.store.create_record('uav002', 'session-one', variant='original', metadata=payload)
        source = media.root / 'internal' / 'restored.mp4'
        source.write_bytes(b'late restored original')
        media.store.update(restored, state='original_ready', original_path=str(source))
        with pytest.raises(ValueError, match='终止'):
            media.rebuild(restored)
        media.session(notice(session='session-two', mission=18))
        assert media.delivery_status('uav002', 'session-two')['state'] == 'waiting'
    finally:
        close(media)
    media = ground(tmp_path)
    try:
        assert media.delivery_status('uav002', 'session-one')['state'] == 'failed'
        media.session(payload)
        assert not media.sessions
        assert ('uav002', 'session-one') in media.store.stopped_sessions
    finally:
        close(media)


def test_failure_before_any_notice_stops_late_session(tmp_path):
    """A GUI timeout without a media directory remains terminal when the aircraft reconnects."""
    media = ground(tmp_path)
    try:
        media.fail_delivery(dict(source_id='delivery-gui', mission_sequence=8, error='落地超时'))
        media.session(notice())
        assert media.delivery_status('uav002', 'session-one')['state'] == 'failed'
        assert ('uav002', 'session-one') in media.store.stopped_sessions
        assert not media.sessions
        assert not media.store.records()
    finally:
        close(media)


def test_failed_latest_notice_still_updates_other_session_availability(tmp_path):
    """A closed latest session must still relay loss of an older original to the receiver."""
    media = ground(tmp_path)
    try:
        media.session(notice())
        media.session(notice(session='session-two', mission=18))
        media.fail_delivery(dict(source_id='delivery-gui', mission_sequence=18, error='人工结束'))
        media.session(dict(notice(session='session-two', mission=18), upload_records=[
            dict(session_id='session-one', kind='video', state='cache_deleted', cache_state='deleted')]))
        assert wait_delivery(media, 'failed')['error'] == '两端均无可用原片，也无可播放地面录像'
    finally:
        close(media)
