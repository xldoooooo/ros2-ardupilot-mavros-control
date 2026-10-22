"""Exercise media session identity without cameras, network calls or flight commands."""

import threading
from unittest.mock import Mock

from video_service.ground_media import GroundMedia
from video_service.onboard_media import OnboardMedia


def _onboard(last_recording=''):
    """Use real session methods with a fake controller, queue and notification sink."""
    media = OnboardMedia.__new__(OnboardMedia)
    media.controller = Mock()
    media.controller.status.return_value = {'last_recording_file': str(last_recording)}
    media.queue = Mock()
    media.queue.enqueue.return_value = 'queued-file'
    media.session = None
    media._last_session = None
    media._lock = threading.RLock()
    media._notify = Mock()
    media._stop_forward = Mock()
    return media


def _ground(tmp_path):
    """Construct the catalog only; no background worker or network service is started."""
    calibration = tmp_path / 'intrinsics.yaml'
    calibration.write_text('session-calibration-placeholder\n')
    return GroundMedia({
        'root': str(tmp_path / 'ground'), 'intrinsics': str(calibration),
        'public_host': 'ground.example',
    })


def test_failed_new_session_does_not_upload_previous_recording(tmp_path):
    """A camera startup failure must not relabel the previous task's finalized original."""
    previous = tmp_path / 'previous.mp4'
    previous.write_bytes(b'previous-recording')
    media = _onboard(previous)
    media.begin('onboard-flight')
    media.update_mission('gui-one', 8)
    media.finish('camera failed to start')
    media.queue.enqueue.assert_not_called()
    assert media.session is None
    assert media.queue.enqueue_event.call_args.args[0] == 'session_end'

    # A genuinely new finalized pathname remains eligible for asynchronous upload.
    current = tmp_path / 'current.mp4'
    current.write_bytes(b'current-recording')
    media.begin('onboard-flight')
    media.update_mission('gui-one', 9)
    media.controller.status.return_value = {'last_recording_file': str(current)}
    media.finish()
    args = media.queue.enqueue.call_args.args
    assert args[0] == current and args[1] == 'video'
    assert args[3]['mission_sequence'] == 9


def test_manual_photo_preserves_video_mission_and_its_own_manual_identity(tmp_path):
    """Manual photo metadata stays manual while the session retains its accepted mission."""
    media = _onboard()
    media.begin('onboard-flight')
    media.update_mission('mission-gui', 8)
    photo = tmp_path / 'manual.jpg'
    photo.write_bytes(b'manual-photo')
    media.capture(photo, {
        'source_id': 'manual-gui', 'mission_sequence': 0,
        'sequence': 100, 'waypoint_index': 0, 'photo_no': '',
    })
    assert media.session['source_id'] == 'mission-gui'
    assert media.session['mission_sequence'] == 8
    queued_metadata = media.queue.enqueue.call_args.args[3]
    assert queued_metadata['source_id'] == 'manual-gui'
    assert queued_metadata['mission_sequence'] == 0
    media.finish()
    end_metadata = media.queue.enqueue_event.call_args.args[2]
    assert end_metadata['source_id'] == 'mission-gui'
    assert end_metadata['mission_sequence'] == 8


def test_replayed_older_session_cannot_stop_current_ground_recording(tmp_path):
    """Delayed durable start/end events must not replace a newer live session."""
    media = _ground(tmp_path)
    try:
        media.session({
            'action': 'start', 'aircraft_id': 'refresh', 'session_id': 'new-session',
            'source_id': 'gui-one', 'mission_sequence': 9, 'started_at': 200,
        })
        current = media.sessions['refresh']
        for action in ('start', 'end'):
            media.session({
                'action': action, 'aircraft_id': 'refresh', 'session_id': 'old-session',
                'source_id': 'gui-one', 'mission_sequence': 8, 'started_at': 100,
            })
        assert media.sessions['refresh'] is current
        assert ('refresh', 'new-session') not in media._ended
        assert not any(row['session_id'] == 'new-session' for row in media.store.records())
    finally:
        media.close()
        media.store.close()


def test_late_api_identity_updates_closed_video_fragments_without_changing_processing(tmp_path):
    """Late task identity reaches completed fragments and API fields without rewriting calibration."""
    media = _ground(tmp_path)
    try:
        frozen = {'intrinsics': 'frozen.yaml', 'bitrate': '8M'}
        record_id = media.store.create_record('refresh', 'closed-session', metadata={
            'source_id': 'onboard-flight', 'mission_sequence': 0, 'processing': frozen,
        })
        media.store.update(record_id, state='incomplete', error='stream interrupted')
        media._ended.add(('refresh', 'closed-session'))
        media.session({
            'action': 'update', 'aircraft_id': 'refresh', 'session_id': 'closed-session',
            'source_id': 'mission-gui', 'mission_sequence': 8,
        })
        record = media.store.get(record_id)
        assert record['metadata']['source_id'] == 'mission-gui'
        assert record['metadata']['mission_sequence'] == 8
        assert record['metadata']['processing'] == frozen
        assert record['state'] == 'incomplete' and record['error'] == 'stream interrupted'
        published = next(row for row in media.inventory()['records'] if row['id'] == record_id)
        assert published['source_id'] == 'mission-gui'
        assert published['mission_sequence'] == 8
        assert not media.sessions
    finally:
        media.close()
        media.store.close()
