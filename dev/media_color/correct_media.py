#!/usr/bin/env python3
"""Grade existing example media into a separate directory using FFmpeg.

Preview mode extracts short clips; batch mode processes complete originals.
Neither mode changes input files, configuration, services, or active playlists.
"""

import argparse
import fnmatch
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys

# Only these existing sample containers are supported; unsupported files are reported.
VIDEO_SUFFIXES = {".mp4", ".mkv", ".mov"}
PICTURE_SUFFIXES = {".jpg", ".jpeg", ".png"}
# H.264 settings are fixed across the playlist to retain concat compatibility.
VIDEO_OPTIONS = [
    "-c:v", "libx264", "-preset", "medium", "-crf", "18", "-profile:v", "high",
    "-pix_fmt", "yuv420p", "-g", "120", "-keyint_min", "120", "-sc_threshold", "0",
    "-color_range", "tv", "-colorspace", "bt709", "-color_trc", "bt709",
    "-color_primaries", "bt709", "-video_track_timescale", "90000", "-movflags", "+faststart",
]


def run(command):
    """Run argument lists directly and retain diagnostics on failure."""
    result = subprocess.run(command, capture_output=True, text=True)
    if result.returncode:
        raise RuntimeError(f"Command failed: {command}\n{result.stderr[-6000:]}")
    return result.stdout


def probe(path):
    """Read stream parameters without decoding the complete input."""
    data = json.loads(run([
        "ffprobe", "-v", "error", "-select_streams", "v:0", "-show_streams",
        "-show_format", "-of", "json", str(path),
    ]))
    stream = data["streams"][0]
    stream["duration"] = float(data.get("format", {}).get("duration", 0))
    return stream


def sha256(path):
    """Identify the exact source and result without loading a video into memory."""
    with path.open("rb") as source:
        return hashlib.file_digest(source, "sha256").hexdigest()


def select_profile(profiles, name, source):
    """Resolve explicit filename rules without guessing camera settings from dates."""
    settings = profiles[name]
    for rule in settings.get("rules", []):
        if any(fnmatch.fnmatchcase(source.name, pattern) for pattern in rule["patterns"]):
            selected = rule["profile"]
            return selected, profiles[selected]
    selected = settings.get("default", name)
    return selected, profiles[selected]


def grade_filter(stream, profile, *, picture=False):
    """Normalize the actual range before grading, with full-range JPEG output."""
    source_range = {"pc": "full", "tv": "limited"}.get(stream.get("color_range"), "auto")
    matrix = {"bt470bg": "bt601"}.get(stream.get("color_space"), stream.get("color_space"))
    matrix_option = f":in_color_matrix={matrix}" if matrix in {"bt709", "bt601", "smpte170m"} else ""
    # Range conversion changes pixels, rather than merely relabelling full-range originals.
    filters = (
        f"scale=in_range={source_range}:out_range=limited{matrix_option}:out_color_matrix=bt709,"
        "format=yuv420p,"
        f"curves=master='{profile['curve']}',"
        f"eq=saturation={profile['saturation']}"
    )
    if picture:
        filters += ",scale=in_range=limited:out_range=full:in_color_matrix=bt709:out_color_matrix=bt601,format=yuvj420p"
    else:
        filters += ",format=yuv420p"
    return filters


def convert(source, destination, profile, args):
    """Write one independently graded result and verify it before publishing the file."""
    stream = probe(source)
    picture = source.suffix.lower() in PICTURE_SUFFIXES
    temporary = destination.with_name(destination.stem + ".partial" + destination.suffix)
    start = 0.0 if picture or args.mode == "batch" else max(
        0.0, min(stream["duration"] * args.position, stream["duration"] - args.seconds),
    )
    command = ["ffmpeg", "-hide_banner", "-loglevel", "error", "-nostdin", "-n",
               "-threads", str(args.threads), "-filter_threads", str(args.threads)]
    if start:
        command += ["-ss", str(start)]
    command += ["-i", str(source), "-map", "0:v:0"]
    if not picture:
        command += ["-map", "0:a?"]
    command += ["-sn", "-dn", "-map_metadata", "-1", "-vf", grade_filter(stream, profile, picture=picture)]
    if picture:
        if destination.suffix.lower() in {".jpg", ".jpeg"}:
            command += ["-frames:v", "1", "-c:v", "mjpeg", "-q:v", "2", "-color_range", "pc"]
        else:
            command += ["-frames:v", "1", "-c:v", "png", "-pix_fmt", "rgb24"]
    else:
        if args.mode == "preview":
            command += ["-t", str(args.seconds)]
        command += VIDEO_OPTIONS + ["-c:a", "copy", "-fps_mode", "passthrough"]
    command += ["-threads", str(args.threads), str(temporary)]
    destination.parent.mkdir(parents=True, exist_ok=True)
    try:
        run(command)
        run(["ffmpeg", "-v", "error", "-xerror", "-threads", str(args.threads),
             "-i", str(temporary), "-map", "0:v:0", "-f", "null", "-"])
        output = probe(temporary)
        if (output["width"], output["height"]) != (stream["width"], stream["height"]):
            raise RuntimeError(f"Unexpected resolution change: {source}")
        if not picture:
            expected_duration = min(args.seconds, stream["duration"] - start) if args.mode == "preview" else stream["duration"]
            if abs(output["duration"] - expected_duration) > 0.25:
                raise RuntimeError(f"Unexpected duration change: {source}")
            if output["r_frame_rate"] != stream["r_frame_rate"]:
                raise RuntimeError(f"Unexpected frame-rate change: {source}")
        temporary.rename(destination)
    finally:
        if temporary.exists():
            temporary.unlink()
    return {"source": str(source), "output": str(destination), "source_sha256": sha256(source),
            "output_sha256": sha256(destination), "start_seconds": start,
            "input": stream, "result": output}


def main():
    """Process a preview or a complete batch without activating any result."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("mode", choices=("preview", "batch"))
    parser.add_argument("--input", type=Path, required=True, help="Existing examples directory")
    parser.add_argument("--output", type=Path, required=True, help="New directory outside the input tree")
    parser.add_argument("--profile", choices=("natural", "enhanced", "selective"), default="natural")
    parser.add_argument("--profiles", type=Path, default=Path(__file__).with_name("profiles.json"))
    parser.add_argument("--seconds", type=float, default=6.0, help="Preview clip length")
    parser.add_argument("--position", type=float, default=0.5, help="Preview position as a fraction of duration")
    parser.add_argument("--threads", type=int, default=os.cpu_count() or 1)
    parser.add_argument("--picture-count", type=int, default=8, help="Pictures sampled in preview mode")
    args = parser.parse_args()
    source_root, output_root = args.input.expanduser().resolve(), args.output.expanduser().resolve()
    if args.seconds <= 0 or args.threads < 1 or args.picture_count < 0 or not 0 <= args.position <= 1:
        parser.error("Invalid preview duration, position, thread count, or picture count")
    if source_root == output_root or source_root in output_root.parents or output_root in source_root.parents:
        parser.error("Input and output directory trees must be separate")
    if output_root.exists() and any(output_root.iterdir()):
        parser.error("Output directory must be new or empty; originals are never overwritten")
    profiles = json.loads(args.profiles.read_text())
    profile = profiles[args.profile]
    videos = sorted(p for p in (source_root / "videos").rglob("*") if p.suffix.lower() in VIDEO_SUFFIXES)
    pictures = sorted(p for p in (source_root / "pictures").rglob("*") if p.suffix.lower() in PICTURE_SUFFIXES)
    if not videos and not pictures:
        parser.error("No supported source media found")
    total_pictures = len(pictures)
    if args.mode == "preview" and len(pictures) > args.picture_count:
        pictures = [pictures[i * len(pictures) // args.picture_count] for i in range(args.picture_count)]
    output_root.mkdir(parents=True, exist_ok=True)
    manifest = {"mode": args.mode, "profile": args.profile, "parameters": profile,
                "input": str(source_root), "total_source_pictures": total_pictures,
                "threads": args.threads, "files": [], "errors": [], "complete": False}
    manifest_path = output_root / "manifest.json"
    for source in videos + pictures:
        destination = output_root / source.relative_to(source_root)
        if source.suffix.lower() in VIDEO_SUFFIXES:
            destination = destination.with_suffix(".mp4")
        print(f"[{len(manifest['files']) + len(manifest['errors']) + 1}/{len(videos) + len(pictures)}] {source.name}", flush=True)
        try:
            selected, settings = select_profile(profiles, args.profile, source)
            result = convert(source, destination, settings, args)
            result.update(profile=selected, parameters=settings)
            manifest["files"].append(result)
        except (OSError, RuntimeError, ValueError, KeyError) as exc:
            manifest["errors"].append({"source": str(source), "error": str(exc)})
            print(str(exc), file=sys.stderr, flush=True)
        manifest_path.write_text(json.dumps(manifest, indent=2) + "\n")
    manifest["complete"] = not manifest["errors"]
    manifest_path.write_text(json.dumps(manifest, indent=2) + "\n")
    print(f"Completed: {len(manifest['files'])}; failed: {len(manifest['errors'])}; {manifest_path}")
    return int(bool(manifest["errors"]))


if __name__ == "__main__":
    sys.exit(main())
