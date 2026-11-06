"""Select Python's actual OpenCV libraries and matching headers without replacing JetPack."""

import argparse
from pathlib import Path
import re
import subprocess

import cv2


# OpenCV 4.7 moved the ArUco implementation and dictionary headers into objdetect.
MODERN_ARUCO = tuple(map(int, cv2.__version__.split('.')[:2])) >= (4, 7)
# Only these existing shared libraries are used by the native correction data plane.
MODULES = ('core', 'imgproc', 'imgcodecs') + (('objdetect',) if MODERN_ARUCO else ('aruco',)) + ('calib3d',)
# Ubuntu splits the needed headers across these development packages.
HEADER_PACKAGES = ('core', 'flann', 'features2d', 'imgproc', 'imgcodecs', 'calib3d') + (('objdetect',) if MODERN_ARUCO else ('contrib',))


def header_version(directory):
    """Read public header version macros instead of trusting pkg-config's active prefix."""
    source = directory/'opencv2/core/version.hpp'
    aruco = 'opencv2/objdetect/aruco_detector.hpp' if MODERN_ARUCO else 'opencv2/aruco.hpp'
    if not source.is_file() or not (directory/aruco).is_file():
        return ''
    text = source.read_text()
    return '.'.join(re.search(rf'#define\s+CV_VERSION_{part}\s+(\d+)', text)[1]
                    for part in ('MAJOR', 'MINOR', 'REVISION'))


def private_headers(cache):
    """Extract matching Ubuntu headers into the build tree; never install or downgrade packages."""
    root = cache/cv2.__version__/'root'
    include = root/'usr/include/opencv4'
    if header_version(include) == cv2.__version__:
        return include
    packages = cache/cv2.__version__/'packages'
    packages.mkdir(parents=True, exist_ok=True)
    root.mkdir(parents=True, exist_ok=True)
    for module in HEADER_PACKAGES:
        package = f'libopencv-{module}-dev'
        metadata = subprocess.check_output(['apt-cache', 'show', package], text=True)
        version = next((value for value in re.findall(r'^Version: (.+)$', metadata, re.M)
                        if value.startswith(cv2.__version__+'+')), None)
        if version is None:
            raise RuntimeError(f'No {package} headers matching Python OpenCV {cv2.__version__}; provide a matching development prefix')
        subprocess.run(['apt-get', 'download', f'{package}={version}'], cwd=packages, check=True)
    for package in packages.glob('*.deb'):
        subprocess.run(['dpkg-deb', '-x', str(package), str(root)], check=True)
    if header_version(include) != cv2.__version__:
        raise RuntimeError('Extracted OpenCV headers do not match the active Python binding')
    return include


def main():
    """Write exact include/library paths for CMake, preserving every system OpenCV installation."""
    parser = argparse.ArgumentParser()
    parser.add_argument('--cache', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    candidates = (Path('/usr/include/opencv4'), Path('/usr/local/include/opencv4'))
    include = next((p for p in candidates if header_version(p) == cv2.__version__), None)
    if include is None:
        include = private_headers(args.cache.resolve())
    dependencies = subprocess.check_output(['ldd', cv2.__file__], text=True)
    libraries = dict(re.findall(r'libopencv_(\w+)\.so\.[\d.]+\s+=>\s+(\S+)', dependencies))
    missing = set(MODULES)-libraries.keys()
    if missing:
        raise RuntimeError(f'Python OpenCV has no separately linkable libraries: {sorted(missing)}')
    selected = [str(Path(libraries[module]).resolve()) for module in MODULES]
    args.output.write_text(f'set(CORRECTION_OPENCV_INCLUDE "{include}")\n'
                           f'set(CORRECTION_OPENCV_LIBRARIES "{";".join(selected)}")\n'
                           f'set(CORRECTION_OPENCV_VERSION "{cv2.__version__}")\n')


if __name__ == '__main__':
    main()
