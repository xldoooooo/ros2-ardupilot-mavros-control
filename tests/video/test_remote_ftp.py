"""Exercise remote media publication and retries against a real local FTP server."""

from ftplib import error_perm
import threading
import time

import pytest
from pyftpdlib.authorizers import DummyAuthorizer
from pyftpdlib.handlers import FTPHandler
from pyftpdlib.servers import FTPServer

from video_service.remote_ftp import RemoteFTP
from video_service.ground_media import GroundMedia
from tests.video.support import calibration_file


@pytest.mark.integration
def test_ground_delivery_waits_for_remote_publication_and_recovers_receipts(tmp_path, ftp_server):
    """Gate real delivery paths on FTP success while retaining local files and restart receipts."""
    remote, port, handler = ftp_server
    settings = dict(root=str(tmp_path / 'ground'), intrinsics=str(calibration_file(tmp_path)),
                    public_host='127.0.0.1', remote_ftp=dict(enabled=True, host='127.0.0.1',
                    port=port, user='media-user', password='local-test-password', timeout=3))
    media = GroundMedia(settings)
    try:
        records = []
        for kind, suffix in [('video', '.mp4'), ('image', '.jpg')]:
            source = tmp_path / (kind + suffix)
            source.write_bytes((kind + '-finalized').encode() * 100)
            records.append(media.store.register_processed(source, 'uav002', 'mission-one',
                                                          kind=kind))
        video_id, photo_id = records
        delivery = dict(aircraft_id='uav002', session_id='mission-one', source_id='test-gui',
                        mission_sequence=1, state='waiting', ended_at=time.time(),
                        landed_at=time.time(), error='', record_id='')
        media._deliveries['uav002/mission-one'] = delivery
        assert all(not row['ftp_path'] for row in media.inventory()['records'])
        media._finalize_uploaded_videos()
        assert delivery['state'] == 'uploading'
        media._publish_remote()
        photo = next(row for row in media.inventory()['records'] if row['id'] == photo_id)
        assert photo['ftp_path'].endswith('.jpg')
        assert (remote / photo['ftp_path'].lstrip('/')).is_file()
        handler.reject_rename = True
        media._publish_remote()
        assert not media._remote_path(media.store.get(video_id))
        assert media.store.get(video_id)['state'] == 'ready'
        assert '公网 FTP' in media.inventory()['error']
        media._remote_ftp_retry.clear()
        media._publish_remote()
        media._finalize_uploaded_videos()
        assert delivery['state'] == 'ready' and delivery['record_id'] == video_id
        assert media._remote_path(media.store.get(video_id)).endswith('.mp4')
    finally:
        media.close()
        media.store.close()
    recovered = GroundMedia(settings)
    try:
        assert recovered.delivery_status('uav002', 'mission-one')['state'] == 'ready'
        assert recovered._remote_path(recovered.store.get(video_id)).endswith('.mp4')
    finally:
        recovered.close()
        recovered.store.close()


@pytest.fixture
def ftp_server(tmp_path):
    """Serve an isolated writable FTP root, including a bad private PASV address."""
    root = tmp_path / 'remote'
    root.mkdir()
    authorizer = DummyAuthorizer()
    authorizer.add_user('media-user', 'local-test-password', str(root), perm='elradfmwMT')

    class Handler(FTPHandler):
        """Allow one real protocol failure to verify retained files and retry behavior."""

        reject_rename = False
        wrong_size = False
        masquerade_address = '10.123.45.67'

        def ftp_RNTO(self, path):
            """Reject an explicitly requested rename without replacing the existing file."""
            if type(self).reject_rename:
                type(self).reject_rename = False
                self.respond('550 Temporary publication failure')
                return
            return super().ftp_RNTO(path)

        def ftp_SIZE(self, path):
            """Report an incomplete upload once to exercise the publication size check."""
            if type(self).wrong_size:
                type(self).wrong_size = False
                self.respond('213 0')
                return
            return super().ftp_SIZE(path)

    Handler.authorizer = authorizer
    server = FTPServer(('127.0.0.1', 0), Handler)
    port = server.socket.getsockname()[1]
    worker = threading.Thread(target=server.serve_forever,
                              kwargs={'timeout': .05, 'handle_exit': False}, daemon=True)
    worker.start()
    try:
        yield root, port, Handler
    finally:
        server.close_all()
        worker.join(timeout=3)


@pytest.mark.integration
@pytest.mark.parametrize('use_url', [False, True])
def test_upload_preserves_public_path_and_bytes(tmp_path, ftp_server, use_url):
    """Publish a nested media file with URL or explicit credentials through passive FTP."""
    remote, port, _ = ftp_server
    public = tmp_path / 'public'
    source = public / 'uav002' / 'session-one' / 'photo%20one.jpg'
    source.parent.mkdir(parents=True)
    source.write_bytes(b'finalized-media' * 10000)
    config = {'url': f'ftp://media-user:local-test-password@127.0.0.1:{port}/'} if use_url else {
        'host': '127.0.0.1', 'port': port, 'user': 'media-user',
        'password': 'local-test-password'}
    config['timeout'] = 3
    published = RemoteFTP(config).upload(public, source)
    assert published == '/uav002/session-one/photo%20one.jpg'
    assert (remote / published.lstrip('/')).read_bytes() == source.read_bytes()
    assert list((remote / 'uav002' / 'session-one').iterdir()) == [
        remote / 'uav002' / 'session-one' / source.name]


@pytest.mark.integration
def test_receipt_prefix_does_not_repeat_upload_directory(tmp_path, ftp_server):
    """Use a receipt path prefix without appending it to the FTP working directory."""
    remote, port, _ = ftp_server
    (remote / 'tenant').mkdir()
    public = tmp_path / 'public'
    source = public / 'uav002' / 'session-one' / 'recording.mp4'
    source.parent.mkdir(parents=True)
    source.write_bytes(b'finalized-video')
    uploader = RemoteFTP({'host': '127.0.0.1', 'port': port, 'user': 'media-user',
                          'password': 'local-test-password', 'timeout': 3,
                          'directory': '/tenant', 'path_prefix': '/tenant'})
    published = uploader.upload(public, source)
    assert published == '/tenant/uav002/session-one/recording.mp4'
    assert (remote / published.lstrip('/')).read_bytes() == source.read_bytes()
    assert not (remote / 'tenant' / 'tenant').exists()


@pytest.mark.integration
def test_mismatched_receipt_root_does_not_repeat_large_uploads(tmp_path, ftp_server):
    """Reject an incorrect account root before sending bytes that cannot have usable receipts."""
    remote, port, _ = ftp_server
    (remote / 'tenant').mkdir()
    public = tmp_path / 'public'
    public.mkdir()
    source = public / 'recording.mp4'
    source.write_bytes(b'finalized-video')
    uploader = RemoteFTP({'host':'127.0.0.1', 'port':port, 'user':'media-user',
                          'password':'local-test-password', 'directory':'/tenant', 'timeout':3})
    with pytest.raises(OSError, match='receipt root'):
        uploader.upload(public, source)
    assert not list((remote / 'tenant').iterdir())


@pytest.mark.integration
@pytest.mark.parametrize('fault, error, message', [
    ('reject_rename', error_perm, 'publication failure'),
    ('wrong_size', OSError, 'size does not match'),
])
def test_failed_publication_is_visible_and_retry_replaces_file(
        tmp_path, ftp_server, fault, error, message):
    """An incomplete upload or failed rename retains old/local media and retries cleanly."""
    remote, port, handler = ftp_server
    public = tmp_path / 'public'
    source = public / 'uav002' / 'session-one' / 'recording.mp4'
    source.parent.mkdir(parents=True)
    source.write_bytes(b'old-finalized-video')
    uploader = RemoteFTP({'host': '127.0.0.1', 'port': port, 'user': 'media-user',
                          'password': 'local-test-password', 'timeout': 3})
    uploader.upload(public, source)
    destination = remote / 'uav002' / 'session-one' / source.name
    source.write_bytes(b'new-finalized-video' * 100)
    setattr(handler, fault, True)
    with pytest.raises(error, match=message):
        uploader.upload(public, source)
    assert destination.read_bytes() == b'old-finalized-video'
    assert source.read_bytes() == b'new-finalized-video' * 100
    assert len(list(destination.parent.iterdir())) == 1
    uploader.upload(public, source)
    assert destination.read_bytes() == source.read_bytes()


def test_upload_rejects_files_outside_public_root(tmp_path):
    """Private originals cannot acquire public relative paths or start a connection."""
    source = tmp_path / 'internal' / 'original.mp4'
    source.parent.mkdir()
    source.write_bytes(b'private-original')
    with pytest.raises(ValueError):
        RemoteFTP({'host': 'unused.invalid'}).upload(tmp_path / 'public', source)


def test_receipt_prefix_rejects_parent_traversal():
    """Receipt paths cannot escape their configured public directory prefix."""
    with pytest.raises(ValueError, match='path prefix'):
        RemoteFTP({'host': 'unused.invalid', 'path_prefix': '/tenant/../other'})
