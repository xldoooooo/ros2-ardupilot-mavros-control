"""Shared five-coefficient lens geometry for CPU and OpenCL relay backends."""

from pathlib import Path

import cv2
import numpy as np
import yaml

# Camera calibration belongs to the camera model, independently of the aircraft.
DEFAULT_INTRINSICS = Path(__file__).resolve().parents[1] / 'config/video/Wasintek/intrinsics.yaml'


class Calibration:
    """Keep the same projection, edge clamping and chroma siting on every backend."""

    def __init__(self, path: Path, chroma_location: str):
        """Read the complete Brown model and choose the existing alpha-zero projection."""
        data = yaml.safe_load(path.read_text())
        self.width, self.height = int(data['image_width']), int(data['image_height'])
        self.matrix = np.asarray(data['camera_matrix']['data'], dtype=float).reshape(3, 3)
        self.distortion = np.asarray(data['distortion_coefficients']['data'], dtype=float)
        if data.get('distortion_model') != 'plumb_bob' or len(self.distortion) != 5:
            raise ValueError('Requires the five-coefficient plumb_bob calibration')
        if not np.isfinite(self.matrix).all() or not np.isfinite(self.distortion).all() or self.matrix[0, 0] <= 0 or self.matrix[1, 1] <= 0:
            raise ValueError('Calibration must contain finite values and positive focal lengths')
        if self.width <= 0 or self.height <= 0 or self.width % 2 or self.height % 2:
            raise ValueError('Calibration dimensions must be positive and even for YUV420')
        self.target, _ = cv2.getOptimalNewCameraMatrix(
            self.matrix, self.distortion, (self.width, self.height), 0, (self.width, self.height))
        if not np.isfinite(self.target).all() or self.target[0, 0] <= 0 or self.target[1, 1] <= 0:
            raise ValueError('Calibration produced an invalid output projection')
        self.chroma_x = 0.0 if chroma_location == 'left' else 0.5

    def shader(self) -> str:
        """Specialize the shared planar/NV12 OpenCL kernel with this calibration."""
        matrix, target = self.matrix, self.target
        k1, k2, p1, p2, k3 = self.distortion
        values = dict(fx=matrix[0, 0], fy=matrix[1, 1], cx=matrix[0, 2], cy=matrix[1, 2],
                      nfx=target[0, 0], nfy=target[1, 1], ncx=target[0, 2], ncy=target[1, 2],
                      k1=k1, k2=k2, p1=p1, p2=p2, k3=k3, image_width=self.width,
                      image_height=self.height, chroma_x=self.chroma_x)
        definitions = '\n'.join(f'#define {key.upper()} {value:.12f}f' for key, value in values.items())
        path = Path(__file__).resolve().parent / 'shaders/undistort_nv12.cl'
        return path.read_text().replace('// CALIBRATION_CONSTANTS', definitions)

    def maps(self, chroma: bool = False):
        """Build bilinear CPU maps at luma or correctly sited chroma sample positions."""
        scale = 2 if chroma else 1
        ox, oy = (self.chroma_x, 0.5) if chroma else (0.0, 0.0)
        yy, xx = np.indices((self.height // scale, self.width // scale), dtype=np.float32)
        x = (xx * scale + ox - self.target[0, 2]) / self.target[0, 0]
        y = (yy * scale + oy - self.target[1, 2]) / self.target[1, 1]
        k1, k2, p1, p2, k3 = self.distortion
        radius = x*x + y*y
        radial = 1 + radius*(k1 + radius*(k2 + radius*k3))
        mx = (self.matrix[0, 0]*(x*radial + 2*p1*x*y + p2*(radius + 2*x*x)) + self.matrix[0, 2] - ox)/scale
        my = (self.matrix[1, 1]*(y*radial + p1*(radius + 2*y*y) + 2*p2*x*y) + self.matrix[1, 2] - oy)/scale
        return (np.clip(mx, 0, self.width // scale - 1).astype(np.float32),
                np.clip(my, 0, self.height // scale - 1).astype(np.float32))
