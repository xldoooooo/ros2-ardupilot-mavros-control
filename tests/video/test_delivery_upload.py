"""Verify conditional original uploads, durable failures and active-transfer cancellation."""

import errno
import json
import sys
import threading
import time
from unittest.mock import Mock

import pytest

from video_service.media_transfer import UploadQueue
from video_service.onboard_media import OnboardMedia
import video_service.onboard_media as onboard_module


def _queue(tmp_path):
    """Create a stopped queue with local files and no network dependencies."""
    return UploadQueue(tmp_path / 'cache', 'refresh', 'unused', str(tmp_path / 'ground'))


def _source(tmp_path, name='original.mp4'):
    """Create a finalized source without requiring a camera or video encoder."""
    path = tmp_path / name
    path.write_bytes(b'original-payload')
    return path


def test_live_delivery_holds_original_but_not_photos(tmp_path):
    """A waiting ground decision does not delay photos or permit video transfer."""
    queue = _queue(tmp_path)
    try:
        video = queue.enqueue(_source(tmp_path), 'video', 's', held=True)
        photo = queue.enqueue(_source(tmp_path, 'photo.jpg'), 'image', 's')
        queue._upload = Mock()
        queue.tick()
        assert queue._upload.call_args.args[0]['id'] == photo
        assert next(row for row in queue.records() if row['id'] == video)['state'] == 'held'
        queue.set_delivery('s', 'ready')
        video_row = next(row for row in queue.records() if row['id'] == video)
        assert video_row['state'] == 'retained' and video_row['attempts'] == 0
        assert queue.summary('s')[0]['cache_state'] == 'retained'
    finally:
        queue.close()


def test_fallback_releases_original_and_network_fault_keeps_retry(tmp_path):
    """Only an explicit fallback permits upload, with recoverable network backoff."""
    queue = _queue(tmp_path)
    try:
        queue.enqueue(_source(tmp_path), 'video', 's', held=True)
        queue._ssh = Mock(side_effect=OSError('Connection timed out'))
        queue.tick()
        queue._ssh.assert_not_called()
        queue.set_delivery('s', 'fallback')
        queue.tick()
        row = queue.records()[0]
        assert row['state'] == 'retry' and row['retry_at'] > time.time()
        assert row['attempts'] == 1
    finally:
        queue.close()


def test_recovered_ack_wait_needs_fresh_fallback_permission(tmp_path):
    """A pre-restart ACK timeout cannot silently resend the original during an HTTP outage."""
    queue = _queue(tmp_path)
    try:
        queue.enqueue(_source(tmp_path), 'video', 's')
        record = queue.records()[0]
        record.update(state='awaiting_ack', retry_at=0)
        queue._save(record)
        queue.hold_pending_videos()
        queue._upload = Mock()
        queue.tick()
        queue._upload.assert_not_called()
        assert queue.records()[0]['state'] == 'held'
    finally:
        queue.close()


def test_delivery_poll_excludes_history_but_keeps_unresolved_failures(tmp_path):
    """Only unfinished transfers or explicit pending decisions need ground polling."""
    queue = _queue(tmp_path)
    try:
        source = _source(tmp_path)
        for session, state in [('legacy-ack', 'acknowledged'), ('legacy-deleted', 'cache_deleted'),
                               ('pending-ack', 'acknowledged'), ('live-ready', 'held'),
                               ('local-failed', 'held'), ('pending', 'held')]:
            record_id = queue.enqueue(source, 'video', session, held=True)
            record = next(row for row in queue.records() if row['id'] == record_id)
            record['state'] = state
            queue._save(record)
        queue.set_delivery('pending-ack', 'waiting')
        queue.set_delivery('live-ready', 'ready')
        queue.set_delivery('local-failed', 'failed', 'Permission denied')
        assert set(queue.delivery_sessions()) == {'pending-ack', 'local-failed', 'pending'}
    finally:
        queue.close()


@pytest.mark.parametrize('fault', [
    PermissionError(errno.EACCES, 'Permission denied'),
    OSError(errno.ENOSPC, 'No space left on device'),
    OSError('Host key verification failed'),
])
def test_actionable_fault_stops_session_durably(tmp_path, fault):
    """Configuration/storage faults stop retries and remain terminal after reopening."""
    queue = _queue(tmp_path)
    source = _source(tmp_path)
    queue.enqueue(source, 'video', 'bad', held=True)
    queue.enqueue(_source(tmp_path, 'photo.jpg'), 'image', 'bad')
    queue.set_delivery('bad', 'fallback')
    queue._ssh = Mock(side_effect=fault)
    queue.tick()
    assert {row['state'] for row in queue.records()} == {'needs_attention'}
    assert source.exists()
    queue.close()
    queue = _queue(tmp_path)
    try:
        queue.set_delivery('bad', 'fallback')
        queue._upload = Mock()
        queue.tick()
        queue._upload.assert_not_called()
        assert {row['state'] for row in queue.summary('bad')} == {'needs_attention'}
        assert all(row['error'] == str(fault) for row in queue.summary('bad'))
    finally:
        queue.close()


def test_absent_original_is_harmless_until_fallback(tmp_path):
    """Missing aircraft footage fails promptly only when ground actually requires it."""
    queue = _queue(tmp_path)
    try:
        queue.unavailable_video('missing')
        queue._ssh = Mock()
        queue.tick()
        assert queue.records()[0]['state'] == 'held'
        queue.set_delivery('missing', 'fallback')
        queue.tick()
        queue._ssh.assert_not_called()
        assert queue.records()[0]['state'] == 'needs_attention'
        assert queue.summary()[0]['cache_state'] == 'missing'
    finally:
        queue.close()


@pytest.mark.parametrize('decision, expected', [('failed', 'needs_attention'), ('ready', 'retained')])
def test_terminal_decision_interrupts_only_its_session(tmp_path, decision, expected):
    """Cancel a real child process without a late retry overwriting the terminal state."""
    queue = _queue(tmp_path)
    source = _source(tmp_path)
    queue.enqueue(source, 'video', 'active', held=True)
    queue.enqueue(_source(tmp_path, 'other.mp4'), 'video', 'other', held=True)
    queue.set_delivery('active', 'fallback')
    queue._ssh = Mock()

    def blocking_copy(*_args):
        """Keep parent and SSH-like child alive until cancellation stops their group."""
        queue._run([sys.executable, '-c',
                    'import subprocess,sys,time; '
                    'subprocess.Popen([sys.executable,"-c","import time; time.sleep(30)"]); '
                    'time.sleep(30)'])

    queue._rsync = blocking_copy
    worker = threading.Thread(target=queue.tick)
    worker.start()
    deadline = time.monotonic() + 3
    while queue.current_process is None and time.monotonic() < deadline:
        time.sleep(0.01)
    try:
        assert queue.current_process is not None
        queue.set_delivery('active', decision, 'delivery timeout')
        worker.join(timeout=3)
        assert not worker.is_alive()
        states = {row['session_id']: row['state'] for row in queue.records()}
        assert states == {'active': expected, 'other': 'held'}
        assert source.exists() and not list((queue.root / 'manifests').iterdir())
    finally:
        queue.close()


def test_poll_recovers_all_rounds_and_never_grants_fallback_on_http_failure(tmp_path, monkeypatch):
    """Recovered rounds receive their own decisions while HTTP outages leave originals held."""
    queue = _queue(tmp_path)
    queue.enqueue(_source(tmp_path), 'video', 'older', held=True)
    queue.enqueue(_source(tmp_path, 'new.mp4'), 'video', 'newer', held=True)
    media = OnboardMedia.__new__(OnboardMedia)
    media.queue = queue
    media.aircraft_id = 'refresh'
    media.settings = {'ground_api': 'http://ground.invalid'}
    media._delivery_terminal = set()
    media._delivery_poll_offset = 0
    media.error = ''
    response = Mock()
    response.__enter__ = Mock(return_value=response)
    response.__exit__ = Mock(return_value=False)
    response.read.return_value = json.dumps({'state': 'fallback'}).encode()
    request = Mock(side_effect=[OSError('offline'), response])
    monkeypatch.setattr(onboard_module, 'urlopen', request)
    try:
        media._poll_delivery()
        states = {row['session_id']: row['state'] for row in queue.records()}
        assert states == {'older': 'queued', 'newer': 'held'}
        assert 'session_id=newer' in request.call_args_list[0].args[0]
        assert 'session_id=older' in request.call_args_list[1].args[0]
    finally:
        queue.close()


def test_offline_poll_is_bounded_and_rotates_older_rounds(monkeypatch):
    """A large offline backlog still gives the newest round priority and visits older rounds."""
    media = OnboardMedia.__new__(OnboardMedia)
    media.queue = Mock()
    media.queue.delivery_sessions.return_value = ['newest', *[f'old-{index}' for index in range(8)]]
    media.aircraft_id = 'refresh'
    media.settings = {'ground_api': 'http://ground.invalid'}
    media._delivery_terminal = set()
    media._delivery_poll_offset = 0
    request = Mock(side_effect=OSError('offline'))
    monkeypatch.setattr(onboard_module, 'urlopen', request)
    media._poll_delivery()
    media._poll_delivery()
    sessions = [call.args[0].split('session_id=')[1] for call in request.call_args_list]
    assert sessions == ['newest', 'old-0', 'old-1', 'old-2', 'old-3',
                        'newest', 'old-4', 'old-5', 'old-6', 'old-7']
    media.queue.set_delivery.assert_not_called()
