"""Run one Intel GPU decode/undistort/encode pipeline for a ground RTSP relay."""

import argparse
import os
from pathlib import Path
import tempfile

import cv2
import numpy as np
import yaml

# The current video calibration describes the Wasintek camera independently of aircraft identity.
SERVICE_ROOT = Path(__file__).resolve().parent
DEFAULT_INTRINSICS = SERVICE_ROOT.parent / "config/video/Wasintek/intrinsics.yaml"


def make_shader(calibration_path: Path, chroma_location: str) -> str:
    """Bake the calibrated camera model and chroma siting into an OpenCL shader."""
    calibration = yaml.safe_load(calibration_path.read_text())
    width, height = int(calibration["image_width"]), int(calibration["image_height"])
    matrix = np.asarray(calibration["camera_matrix"]["data"], dtype=float).reshape(3, 3)
    distortion = np.asarray(calibration["distortion_coefficients"]["data"], dtype=float)
    if calibration.get("distortion_model") != "plumb_bob" or len(distortion) != 5:
        raise ValueError("This relay requires the project's five-coefficient plumb_bob calibration")
    target, _ = cv2.getOptimalNewCameraMatrix(matrix, distortion, (width, height), 0, (width, height))
    values = dict(fx=matrix[0, 0], fy=matrix[1, 1], cx=matrix[0, 2], cy=matrix[1, 2],
                  nfx=target[0, 0], nfy=target[1, 1], ncx=target[0, 2], ncy=target[1, 2],
                  k1=distortion[0], k2=distortion[1], p1=distortion[2], p2=distortion[3], k3=distortion[4],
                  image_width=width, image_height=height,
                  chroma_x=0.0 if chroma_location == "left" else 0.5)
    definitions = "\n".join(f"#define {key.upper()} {value:.12f}f" for key, value in values.items())
    return (SERVICE_ROOT / "shaders/undistort_nv12.cl").read_text().replace("// CALIBRATION_CONSTANTS", definitions)


def build_command(args, shader_path: Path) -> list[str]:
    """Keep decoded NV12 surfaces on the Intel GPU through remapping and encoding."""
    command = [args.ffmpeg, "-hide_banner", "-loglevel", "warning",
               "-init_hw_device", f"vaapi=va:{args.device}", "-init_hw_device", "opencl=ocl@va",
               "-filter_hw_device", "ocl", "-hwaccel", "vaapi", "-hwaccel_device", "va",
               "-hwaccel_output_format", "vaapi"]
    if args.input.startswith("rtsp://"):
        command += ["-rtsp_transport", "tcp"]
    # Hardware decoding does not need the default CPU-sized frame-thread pool.
    command += ["-threads", "2", "-i", args.input, "-map", "0:v:0", "-an", "-vf",
                "hwmap=derive_device=opencl:mode=read+direct,"
                f"program_opencl=source={shader_path}:kernel=rectify,"
                # Reverse mapping gives the shader encoder-owned VAAPI surfaces.
                "hwmap=derive_device=vaapi:reverse=1:mode=write+direct,format=vaapi",
                "-c:v", "h264_vaapi", "-b:v", args.bitrate, "-g", str(args.gop), "-bf", "0"]
    if args.duration:
        command += ["-t", str(args.duration)]
    if args.frames:
        command += ["-frames:v", str(args.frames)]
    if args.progress:
        command += ["-stats_period", "1", "-progress", args.progress]
    if args.output.startswith("rtsp://"):
        command += ["-f", "rtsp", "-rtsp_transport", "tcp"]
    return command + ["-y", args.output]


def main():
    """Launch the standalone relay without changing camera or GUI lifecycles."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--intrinsics", type=Path, default=DEFAULT_INTRINSICS)
    parser.add_argument("--device", default="/dev/dri/renderD128")
    parser.add_argument("--chroma-location", choices=["left", "center"], default="left")
    parser.add_argument("--bitrate", default="8M")
    parser.add_argument("--gop", type=int, default=60)
    parser.add_argument("--ffmpeg", default="ffmpeg")
    parser.add_argument("--duration", type=float)
    parser.add_argument("--frames", type=int)
    parser.add_argument("--progress")
    args = parser.parse_args()
    with tempfile.TemporaryFile(mode="w+b") as shader:
        shader.write(make_shader(args.intrinsics, args.chroma_location).encode())
        shader.flush()
        # An inherited anonymous file keeps the shader alive without a resident Python wrapper.
        os.set_inheritable(shader.fileno(), True)
        command = build_command(args, Path(f"/proc/self/fd/{shader.fileno()}"))
        os.execvp(args.ffmpeg, command)


if __name__ == "__main__":
    main()
