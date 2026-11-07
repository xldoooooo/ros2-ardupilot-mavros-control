"""Publish finalized public media to a remote FTP server without changing local files."""

from ftplib import FTP, all_errors, error_perm
from pathlib import Path, PurePosixPath
from urllib.parse import unquote, urlsplit
import uuid


class RemoteFTP:
    """Upload one media file per connection; the caller owns scheduling and retries."""

    def __init__(self, config):
        """Read an FTP URL or explicit connection settings without exposing credentials."""
        address = urlsplit(str(config.get('url', '')))
        if address.scheme and address.scheme != 'ftp':
            raise ValueError('Remote media upload requires an ftp URL')
        self.host = str(config.get('host') or address.hostname or '')
        if not self.host:
            raise ValueError('Remote FTP host is required')
        self.port = int(config.get('port') or address.port or 21)
        self.user = str(config.get('user') or config.get('username')
                        or unquote(address.username or '') or 'anonymous')
        self.password = str(config.get('password', unquote(address.password or '')))
        self.directory = str(config.get('directory', unquote(address.path) or '/'))
        self.path_prefix = PurePosixPath('/' + str(config.get('path_prefix', '')).lstrip('/'))
        self.timeout = float(config.get('timeout', 60))
        self.passive = bool(config.get('passive', True))
        if any(character in self.directory for character in ('\r', '\n', '\x00')):
            raise ValueError('Invalid remote FTP directory')
        if ('..' in self.path_prefix.parts
                or any(character in str(self.path_prefix) for character in ('\r', '\n', '\x00'))):
            raise ValueError('Invalid remote FTP path prefix')

    def upload(self, public_root, source_path):
        """Publish under the public relative path, returning it only after SIZE verification."""
        source = Path(source_path).resolve()
        relative = source.relative_to(Path(public_root).resolve())
        if any(character in relative.as_posix() for character in ('\r', '\n', '\x00')):
            raise ValueError('Invalid remote media path')
        size = source.stat().st_size
        temporary = f'.{relative.name}.{uuid.uuid4().hex}.uploading'
        with FTP() as ftp:
            ftp.connect(self.host, self.port, timeout=self.timeout)
            ftp.login(self.user, self.password)
            ftp.set_pasv(self.passive)
            ftp.cwd(self.directory)
            if PurePosixPath(ftp.pwd()) != self.path_prefix:
                raise OSError('FTP working directory does not match receipt root; check server root or directory/path_prefix')
            for directory in relative.parts[:-1]:
                try:
                    ftp.mkd(directory)
                except error_perm:
                    # CWD confirms an existing directory and still exposes permission failures.
                    pass
                ftp.cwd(directory)
            try:
                with source.open('rb') as content:
                    ftp.storbinary(f'STOR {temporary}', content)
                if ftp.size(temporary) != size:
                    raise OSError('Remote FTP temporary file size does not match local media')
                ftp.rename(temporary, relative.name)
                # Verify the exact receipt path, including servers whose login directory is not chrooted.
                if ftp.size(str(self.path_prefix / relative.as_posix())) != size:
                    raise OSError('Remote FTP published file size does not match local media')
            finally:
                try:
                    ftp.delete(temporary)
                except all_errors:
                    # A completed rename removes this name; disconnected uploads retry later.
                    pass
        # The receipt prefix is independent of the FTP working directory.
        return str(self.path_prefix / relative.as_posix())
