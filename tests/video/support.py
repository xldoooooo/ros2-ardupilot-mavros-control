"""Shared local-rsync and camera-calibration helpers for the video test group."""

import shlex

import yaml

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
