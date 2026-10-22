#!/usr/bin/env python3
"""Measure RTSP publish/read capacity across a router using FFmpeg and MediaMTX.

Linux, Python standard library only; run with the project's .venv Python.
"""

import argparse
import csv
import itertools
import json
import math
import os
import re
import shutil
import signal
import socket
import statistics
import subprocess
import sys
import tempfile
import threading
import time
from datetime import datetime
from pathlib import Path
from urllib.parse import urlsplit

# A short encoded clip keeps concurrent sessions free of real-time encoder cost.
CLIP_SECONDS = 8
# One second matches FFmpeg progress reporting and keeps monitoring overhead small.
SAMPLE_SECONDS = 1.0
# Only processes launched by this invocation are registered for cleanup.
CHILDREN = []


def capture(command):
    """Collect optional diagnostics without making unavailable tools fatal."""
    try:
        result = subprocess.run(command, capture_output=True, text=True, timeout=8, check=False)
        return (result.stdout + result.stderr).strip()
    except (OSError, subprocess.TimeoutExpired) as exc:
        return str(exc)


def stop_children(children):
    """Stop owned process groups together, then reap them with a shared deadline."""
    for process in children:
        if process.poll() is None:
            try:
                os.killpg(process.pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
    deadline = time.monotonic() + 4
    for process in children:
        try:
            process.wait(timeout=max(0.01, deadline - time.monotonic()))
        except subprocess.TimeoutExpired:
            try:
                os.killpg(process.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            process.wait()


def launch(command, **kwargs):
    """Launch an owned process without invoking a shell."""
    process = subprocess.Popen(command, start_new_session=True, **kwargs)
    CHILDREN.append(process)
    return process


def interrupt(signum, frame):
    """Route terminal and SSH termination through normal cleanup."""
    raise KeyboardInterrupt


def network_snapshot(interface):
    """Read Linux counters; absent interfaces produce an empty optional metric."""
    result = {}
    for key in ("rx_bytes", "tx_bytes", "rx_dropped", "tx_dropped", "rx_errors", "tx_errors"):
        try:
            result[key] = int(Path(f"/sys/class/net/{interface}/statistics/{key}").read_text())
        except OSError:
            pass
    return result


def cpu_snapshot():
    """Read total and idle host CPU ticks for interval utilization."""
    values = [int(value) for value in Path("/proc/stat").read_text().splitlines()[0].split()[1:9]]
    return sum(values), values[3] + values[4]


def process_snapshot(processes):
    """Read owned child CPU ticks and resident memory on Linux."""
    ticks, rss = 0, 0
    for process in processes:
        try:
            fields = Path(f"/proc/{process.pid}/stat").read_text().rsplit(")", 1)[1].split()
            ticks += int(fields[11]) + int(fields[12])
            rss += int(fields[21]) * os.sysconf("SC_PAGE_SIZE")
        except (OSError, ProcessLookupError):
            pass
    return ticks, rss / 1024**2


def environment(args):
    """Record endpoint, routing and Wi-Fi context so later router runs can compare."""
    return {
        "started": datetime.now().astimezone().isoformat(),
        "hostname": socket.gethostname(), "router_label": args.label,
        "arguments": {key: str(value) if isinstance(value, Path) else value
                      for key, value in vars(args).items()},
        "ffmpeg": capture([args.ffmpeg, "-version"]).splitlines()[0],
        "addresses": capture(["ip", "-br", "address"]),
        "routes": capture(["ip", "route"]),
        "wifi": capture(["nmcli", "-f", "ACTIVE,SSID,BSSID,CHAN,RATE,SIGNAL", "device", "wifi", "list", "--rescan", "no"]),
        "link": capture(["iw", "dev", args.interface, "link"]) if args.interface else "",
    }


def prepare_clip(args, bitrate, directory):
    """Encode H.264 CBR once and measure video packet bytes with ffprobe."""
    target = directory / f"source-{bitrate:g}M.mp4"
    command = [args.ffmpeg, "-hide_banner", "-loglevel", "error", "-nostdin"]
    if args.video:
        command += ["-stream_loop", "-1", "-i", str(args.video), "-vf", f"scale={args.size.replace('x', ':')},fps={args.fps}"]
    else:
        command += ["-f", "lavfi", "-i", f"testsrc2=size={args.size}:rate={args.fps}"]
    command += ["-t", str(args.clip_seconds), "-an", "-c:v", "libx264", "-threads", "2",
               "-preset", "ultrafast", "-tune", "zerolatency", "-pix_fmt", "yuv420p",
               "-b:v", f"{bitrate:g}M", "-minrate", f"{bitrate:g}M", "-maxrate", f"{bitrate:g}M",
               "-bufsize", f"{bitrate * 2:g}M", "-g", str(args.fps), "-keyint_min", str(args.fps),
               "-sc_threshold", "0", "-x264-params", "nal-hrd=cbr:force-cfr=1", "-y", str(target)]
    process = launch(command)
    if process.wait() != 0:
        raise RuntimeError("Synthetic clip encoding failed (FFmpeg requires libx264)")
    probe = subprocess.run([args.ffprobe, "-v", "error", "-select_streams", "v:0",
                            "-show_packets", "-show_entries", "packet=size:format=duration", "-of", "json", str(target)],
                           capture_output=True, text=True, check=True, timeout=30)
    data = json.loads(probe.stdout)
    return target, sum(int(packet["size"]) for packet in data["packets"]) * 8 / float(data["format"]["duration"]) / 1e6


class Session:
    """One FFmpeg process and its continuously drained progress stream."""

    def __init__(self, name, role, command, directory):
        """Persist stderr and collect only complete FFmpeg progress updates."""
        self.name, self.role = name, role
        self.started_at = time.monotonic()
        self.frames = 0
        self.video_bytes = 0
        self.packet_counted = False
        self.last_frame_at = None
        self.first_frame_at = None
        self.lock = threading.Lock()
        self.log_path = directory / f"{name}.log"
        self.log = self.log_path.open("w")
        self.process = launch(command, stdout=subprocess.PIPE, stderr=self.log, text=True)
        self.thread = threading.Thread(target=self.consume, daemon=True)
        self.thread.start()

    def consume(self):
        """Drain stdout promptly so the observer cannot block media forwarding."""
        update = {}
        with self.process.stdout:
            for line in self.process.stdout:
                # FFmpeg 6 omits frame progress during stream copy. Count framecrc
                # packet records instead; the second copy output never re-encodes.
                fields = line.split(",")
                if len(fields) >= 6 and fields[0].strip().isdigit():
                    with self.lock:
                        self.frames += 1
                        self.video_bytes += int(fields[4])
                        self.packet_counted = True
                        self.last_frame_at = time.monotonic()
                        if self.first_frame_at is None:
                            self.first_frame_at = self.last_frame_at
                    continue
                key, _, value = line.strip().partition("=")
                update[key] = value
                if key == "progress":
                    frames = int(update.get("frame", "0"))
                    with self.lock:
                        if "frame" in update and frames > self.frames:
                            self.last_frame_at = time.monotonic()
                            if self.first_frame_at is None:
                                self.first_frame_at = self.last_frame_at
                            self.frames = frames
                    update = {}

    def snapshot(self):
        """Return a consistent progress view including unexpected exits."""
        with self.lock:
            return {"name": self.name, "role": self.role, "frames": self.frames,
                    "video_bytes": self.video_bytes if self.packet_counted else None,
                    "last_frame_at": self.last_frame_at, "first_frame_at": self.first_frame_at,
                    "exit_code": self.process.poll()}

    def close(self):
        """Join the progress reader after process cleanup and close its log."""
        self.thread.join(timeout=2)
        self.log.close()


def ffmpeg_command(args, transport, url, clip=None, source_url=None):
    """Build a real-time copy publisher or an unpaced RTSP receiver."""
    command = [args.ffmpeg, "-hide_banner", "-nostdin", "-nostats", "-loglevel", "warning",
               "-stats_period", str(SAMPLE_SECONDS), "-progress", "pipe:1"]
    if clip or source_url:
        if source_url:
            command += ["-rtsp_transport", args.source_transport, "-timeout", str(int(args.timeout * 1e6)), "-i", source_url]
        else:
            command += ["-re", "-stream_loop", "-1", "-i", str(clip)]
        return command + ["-map", "0:v:0", "-an", "-c:v", "copy", "-f", "rtsp", "-rtsp_transport", transport, url,
                          "-map", "0:v:0", "-an", "-c:v", "copy", "-f", "framecrc", "pipe:1"]
    command += ["-rtsp_transport", transport, "-timeout", str(int(args.timeout * 1e6)),
                "-probesize", "32768", "-analyzeduration", "1000000", "-threads", "2",
                "-i", url, "-map", "0:v:0", "-an"]
    if args.decode:
        command += ["-threads", "2"]
    else:
        command += ["-c:v", "copy"]
    return command + ["-fps_mode", "passthrough", "-f", "null" if args.decode else "framecrc", "-"]


class PingProbe:
    """Probe once per second with bounded pings that need no privileged signals."""

    def __init__(self, target, path):
        """Resolve the target once and start an independent network observer."""
        self.target = socket.gethostbyname(target)
        self.path, self.responses = path, []
        self.stopped = threading.Event()
        self.thread = threading.Thread(target=self.consume, daemon=True)
        self.thread.start()

    def consume(self):
        """Keep each ping finite; some Linux capability profiles forbid killing ping."""
        with self.path.open("w") as output:
            while not self.stopped.is_set():
                started = time.monotonic()
                result = subprocess.run(["ping", "-n", "-c", "1", "-W", "1", self.target],
                                        capture_output=True, text=True, check=False, env={**os.environ, "LC_ALL": "C"})
                output.write(result.stdout + result.stderr)
                output.flush()
                match = re.search(r"time[=<]([\d.]+) ms", result.stdout)
                self.responses.append(float(match[1]) if result.returncode == 0 and match else None)
                self.stopped.wait(max(0, SAMPLE_SECONDS - (time.monotonic() - started)))

    def close(self):
        """Wait for the current finite probe and return observed RTT/loss."""
        self.stopped.set()
        self.thread.join()
        valid = [value for value in self.responses if value is not None]
        return {"sent": len(self.responses), "received": len(valid),
                "loss_percent": 100 * (1 - len(valid) / len(self.responses)) if self.responses else None,
                "rtt_min_avg_max_ms": [min(valid), statistics.mean(valid), max(valid)] if valid else None}


def run_stage(args, name, transport, streams, readers, bitrate, clip, actual_mbps, directory):
    """Run one bounded load level and preserve measurements even if interrupted."""
    directory.mkdir()
    sessions = []
    result = {"stage": name, "transport": transport, "mode": args.mode, "streams": streams,
              "readers_per_stream": readers, "target_stream_mbps": bitrate,
              "encoded_stream_mbps": actual_mbps, "duration_s": 0, "interrupted": False}
    urls = [f"{args.url.rstrip('/')}/{directory.parent.name}-{name}/s{index + 1}"
            if args.mode != "read" and not args.fixed_paths else args.url.format(stream=index + 1)
            for index in range(streams)]
    result["urls"] = urls
    (directory / "plan.json").write_text(json.dumps(result, indent=2) + "\n")
    if args.mode != "read":
        print("Publishing: " + ", ".join(urls), flush=True)
    ping_path = directory / "ping.log"
    ping = None
    samples = []
    start = None
    baselines, max_stalls = {}, {}
    try:
        if args.mode != "read":
            for index, url in enumerate(urls):
                sessions.append(Session(f"publish-{index + 1}", "publish",
                                        ffmpeg_command(args, transport, url, clip,
                                                       args.source_url.format(stream=index + 1) if args.source_url else None), directory))
            deadline = time.monotonic() + args.timeout
            while time.monotonic() < deadline:
                states = [session.snapshot() for session in sessions]
                if all(state["frames"] > 0 or state["exit_code"] is not None for state in states):
                    break
                time.sleep(0.1)
        if args.mode != "publish":
            for index, url in enumerate(urls):
                for reader in range(readers):
                    sessions.append(Session(f"read-{index + 1}-{reader + 1}", "read",
                                            ffmpeg_command(args, transport, url), directory))
        time.sleep(args.warmup)
        if args.ping and shutil.which("ping"):
            ping = PingProbe(args.ping, ping_path)
        start = previous_time = time.monotonic()
        result["measurement_started_at"] = datetime.now().astimezone().isoformat()
        previous_net, previous_cpu = network_snapshot(args.interface), cpu_snapshot()
        processes = [session.process for session in sessions]
        previous_ticks, _ = process_snapshot(processes)
        baselines = {session.name: session.snapshot() for session in sessions}
        max_stalls = {session.name: 0.0 for session in sessions}
        with (directory / "samples.jsonl").open("w") as output:
            while True:
                time.sleep(min(SAMPLE_SECONDS, max(0, args.duration - (time.monotonic() - start))))
                now = time.monotonic()
                elapsed = now - previous_time
                net, cpu = network_snapshot(args.interface), cpu_snapshot()
                ticks, rss = process_snapshot(processes)
                states = [session.snapshot() for session in sessions]
                for state in states:
                    # Exclude warmup; a session that never starts still accumulates a stall.
                    last = max(start, state["last_frame_at"] or start)
                    max_stalls[state["name"]] = max(max_stalls[state["name"]], now - last)
                sample = {"timestamp": datetime.now().astimezone().isoformat(), "elapsed_s": now - start, "host_cpu_percent":
                          100 * (1 - (cpu[1] - previous_cpu[1]) / max(1, cpu[0] - previous_cpu[0])),
                          "ffmpeg_cpu_one_core_percent": max(0, ticks - previous_ticks) /
                          os.sysconf("SC_CLK_TCK") / elapsed * 100, "ffmpeg_rss_mib": rss,
                          "network": {key: max(0, value - previous_net.get(key, value))
                                      for key, value in net.items()}, "sessions": states}
                for direction in ("rx", "tx"):
                    sample[f"{direction}_mbps"] = sample["network"].get(f"{direction}_bytes", 0) * 8 / elapsed / 1e6
                output.write(json.dumps(sample) + "\n")
                output.flush()
                samples.append(sample)
                previous_time, previous_net, previous_cpu, previous_ticks = now, net, cpu, ticks
                if now - start >= args.duration or all(state["exit_code"] is not None for state in states):
                    break
    except KeyboardInterrupt:
        result["interrupted"] = True
        raise
    finally:
        duration = time.monotonic() - start if start else 0
        result["duration_s"] = duration
        result["measurement_ended_at"] = datetime.now().astimezone().isoformat()
        result["sessions"] = []
        for session in sessions:
            state = session.snapshot()
            baseline = baselines.get(session.name, state)
            frames = max(0, state["frames"] - baseline["frames"])
            state.update({"frames_measured": frames, "fps_measured": frames / duration if duration else 0,
                          "video_mbps": (state["video_bytes"] - (baseline["video_bytes"] or 0)) * 8 / duration / 1e6
                          if state["video_bytes"] is not None and duration else None,
                          "frame_ratio": frames / (args.fps * duration) if duration else 0,
                          "max_progress_stall_s": max_stalls.get(session.name, 0),
                          "startup_s": (state["first_frame_at"] - session.started_at)
                          if state["first_frame_at"] else None})
            result["sessions"].append(state)
        # Snapshot exit codes before intentional shutdown, including a legitimate early code 0.
        result["healthy"] = bool(duration and not result["interrupted"] and result["sessions"] and all(
            state["exit_code"] is None and state["frame_ratio"] >= args.min_frame_ratio and
            state["max_progress_stall_s"] <= args.max_stall for state in result["sessions"]))
        stop_children([session.process for session in sessions])
        for session in sessions:
            session.close()
            content = session.log_path.read_text(errors="replace")
            matching = next(state for state in result["sessions"] if state["name"] == session.name)
            matching["warning_lines"] = len(content.splitlines())
            matching["rtp_missed_packets"] = sum(int(value) for value in re.findall(r"RTP: missed (\d+) packets", content))
        if ping:
            result["ping"] = ping.close()
        for key in ("rx_mbps", "tx_mbps", "host_cpu_percent", "ffmpeg_cpu_one_core_percent", "ffmpeg_rss_mib"):
            result[f"mean_{key}"] = statistics.mean(sample[key] for sample in samples) if samples else None
        result["interface_counter_deltas"] = {key: sum(sample["network"].get(key, 0) for sample in samples)
                                               for key in network_snapshot(args.interface)}
        (directory / "result.json").write_text(json.dumps(result, indent=2) + "\n")
    return result


def write_summary(args, directory, results):
    """Write machine-readable stage comparisons and a compact human summary."""
    (directory / "summary.json").write_text(json.dumps(results, indent=2) + "\n")
    keys = ["stage", "transport", "mode", "streams", "readers_per_stream", "target_stream_mbps",
            "encoded_stream_mbps", "duration_s", "healthy", "mean_tx_mbps", "mean_rx_mbps",
            "mean_host_cpu_percent", "mean_ffmpeg_cpu_one_core_percent"]
    with (directory / "summary.csv").open("w") as output:
        writer = csv.DictWriter(output, keys, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(results)
    lines = ["# RTSP 压测结果", "", f"路由器/场景：{args.label}", "",
             "healthy 仅按接收/发送帧进度、停顿和进程存活判定，不代表画面正确或端到端延迟达标。",
             "网卡吞吐包含该网卡所有业务；同 AP 两个无线端之间的传输会占用两段无线空口。", "",
             "| 阶段 | 传输 | 流数 | 每路读者 | 每流目标 Mbps | 最低帧比例 | 最长停顿 s | 健康 |",
             "| --- | --- | --- | --- | --- | --- | --- | --- |"]
    for result in results:
        states = result["sessions"]
        lines.append(f"| {result['stage']} | {result['transport']} | {result['streams']} | "
                     f"{result['readers_per_stream']} | {result['target_stream_mbps']} | "
                     f"{min((state['frame_ratio'] for state in states), default=0):.3f} | "
                     f"{max((state['max_progress_stall_s'] for state in states), default=0):.2f} | {result['healthy']} |")
    lines += ["", "每路详见各阶段 result.json；UDP 丢包日志是 FFmpeg 的局部观测，不是精确链路丢包率。",
              "本工具不测玻璃到玻璃延迟，也不读取路由器 CPU、内存或交换芯片统计。", ""]
    (directory / "summary.md").write_text("\n".join(lines))


def run(args):
    """Sweep a reproducible workload matrix, persisting each completed stage."""
    parsed = urlsplit(args.url)
    if parsed.scheme != "rtsp" or not parsed.hostname or parsed.query or parsed.fragment:
        raise ValueError("--url must be rtsp://host:port/path without query or fragment")
    if args.mode != "read" and "{" in args.url and not args.fixed_paths:
        raise ValueError("Publish URL placeholders require --fixed-paths")
    if args.fixed_paths and args.mode != "read" and max(args.streams) > 1 and "{stream}" not in args.url:
        raise ValueError("Multiple fixed publish paths require a {stream} URL placeholder")
    if args.mode == "read" and (len(args.bitrates) != 1):
        raise ValueError("Read mode takes one --bitrates value as expected source metadata")
    if args.source_url and (args.mode == "read" or len(args.bitrates) != 1 or args.video):
        raise ValueError("--source-url requires publish/both mode, one expected bitrate and no --video")
    if not args.interface:
        route = capture(["ip", "route", "get", socket.gethostbyname(parsed.hostname)])
        match = re.search(r"\bdev (\S+)", route)
        args.interface = match[1] if match else ""
    if args.interface == "lo":
        print("WARNING: loopback traffic does not test the router", flush=True)
    args.output.mkdir(parents=True, exist_ok=True)
    directory = Path(tempfile.mkdtemp(prefix=datetime.now().astimezone().strftime("%Y%m%d-%H%M%S-"), dir=args.output))
    (directory / "environment.json").write_text(json.dumps(environment(args), indent=2) + "\n")
    print(f"Results: {directory.resolve()}", flush=True)
    results = []
    try:
        clips = {bitrate: prepare_clip(args, bitrate, directory) for bitrate in args.bitrates} if args.mode != "read" and not args.source_url else {}
        levels = itertools.product(args.transports, args.bitrates, args.streams,
                                   [0] if args.mode == "publish" else args.readers)
        for index, (transport, bitrate, streams, readers) in enumerate(levels, 1):
            name = f"{index:03d}-{transport}-{bitrate:g}M-{streams}s-{readers}r"
            print(f"Starting {name}: {args.warmup:g}s warmup + {args.duration:g}s measurement", flush=True)
            clip, actual = clips.get(bitrate, (None, None))
            result = run_stage(args, name, transport, streams, readers, bitrate, clip, actual, directory / name)
            results.append(result)
            write_summary(args, directory, results)
            print(f"{name}: healthy={result['healthy']}, tx={result['mean_tx_mbps']:.2f}, rx={result['mean_rx_mbps']:.2f} Mbps", flush=True)
            if args.stop_on_failure and not result["healthy"]:
                break
            if args.cooldown:
                time.sleep(args.cooldown)
    finally:
        # Include a stage interrupted before run_stage returned its result.
        known = {result["stage"] for result in results}
        for path in sorted(directory.glob("*/result.json")):
            result = json.loads(path.read_text())
            if result["stage"] not in known:
                results.append(result)
        write_summary(args, directory, results)
    return 0 if results and all(result["healthy"] for result in results) else 1


def serve(args):
    """Run an isolated foreground MediaMTX and sample its host, without systemd changes."""
    args.output.mkdir(parents=True, exist_ok=True)
    directory = Path(tempfile.mkdtemp(prefix="server-", dir=args.output))
    config = directory / "mediamtx.yml"
    config.write_text(f"logLevel: info\nrtspAddress: :{args.port}\nrtspTransports: [tcp, udp]\n"
                      f"rtpAddress: :{args.rtp_port}\nrtcpAddress: :{args.rtp_port + 1}\n"
                      "rtmp: no\nhls: no\nwebrtc: no\nsrt: no\nmoq: no\n"
                      "paths:\n  all_others:\n    source: publisher\n")
    print(f"MediaMTX RTSP :{args.port}, UDP :{args.rtp_port}/:{args.rtp_port + 1}; logs: {directory.resolve()}", flush=True)
    (directory / "environment.json").write_text(json.dumps(environment(args), indent=2) + "\n")
    with (directory / "mediamtx.log").open("w") as log, (directory / "samples.jsonl").open("w") as output:
        process = launch([args.mediamtx, str(config.resolve())], stdout=log, stderr=subprocess.STDOUT)
        start = previous_time = time.monotonic()
        previous_net, previous_cpu = network_snapshot(args.interface), cpu_snapshot()
        previous_ticks, _ = process_snapshot([process])
        try:
            while not args.duration or time.monotonic() - start < args.duration:
                time.sleep(SAMPLE_SECONDS)
                if process.poll() is not None:
                    raise RuntimeError(f"MediaMTX exited with {process.returncode}: {log.name}")
                now = time.monotonic()
                net, cpu = network_snapshot(args.interface), cpu_snapshot()
                ticks, rss = process_snapshot([process])
                output.write(json.dumps({"timestamp": datetime.now().astimezone().isoformat(), "elapsed_s": now - start,
                    "host_cpu_percent": 100 * (1 - (cpu[1] - previous_cpu[1]) / max(1, cpu[0] - previous_cpu[0])),
                    "mediamtx_cpu_one_core_percent": (ticks - previous_ticks) / os.sysconf("SC_CLK_TCK") / (now - previous_time) * 100,
                    "mediamtx_rss_mib": rss,
                    "network": {key: value - previous_net.get(key, value) for key, value in net.items()}}) + "\n")
                output.flush()
                previous_time, previous_net, previous_cpu, previous_ticks = now, net, cpu, ticks
        finally:
            stop_children([process])
    return 0


def numbers(value, cast=float):
    """Parse positive comma-separated workload levels."""
    try:
        values = [cast(part) for part in value.split(",")]
        if not values or any(number <= 0 or not math.isfinite(number) for number in values):
            raise ValueError
        return values
    except ValueError as exc:
        raise argparse.ArgumentTypeError("Use positive comma-separated numbers") from exc


def main():
    """Parse the standalone server and load generator commands."""
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="command", required=True)
    common = argparse.ArgumentParser(add_help=False)
    common.add_argument("--label", default="unnamed-router", help="Router model/firmware and topology label")
    common.add_argument("--interface", default="", help="Linux interface (run detects route if omitted)")
    common.add_argument("--output", type=Path, default=Path(__file__).resolve().parent / "results")
    common.add_argument("--ffmpeg", default="ffmpeg")
    load = subparsers.add_parser("run", parents=[common], help="Run publish/read load levels")
    load.add_argument("--url", required=True, help="Publish base URL; exact read URL or template with {stream}")
    load.add_argument("--mode", choices=["both", "publish", "read"], default="both")
    load.add_argument("--fixed-paths", action="store_true", help="Publish to exact URL/template for a coordinated relay test")
    load.add_argument("--transports", default="tcp", help="tcp,udp (applies to both publish and read)")
    load.add_argument("--streams", type=lambda value: numbers(value, int), default=[1, 2, 4])
    load.add_argument("--readers", type=lambda value: numbers(value, int), default=[1], help="Readers per stream")
    load.add_argument("--bitrates", type=numbers, default=[8], help="Per-stream Mbps; expected value only in read mode")
    load.add_argument("--size", default="1920x1080")
    load.add_argument("--video", type=Path, help="Real video to pre-encode instead of synthetic pattern")
    load.add_argument("--clip-seconds", type=float, default=CLIP_SECONDS)
    load.add_argument("--source-url", help="Relay an existing RTSP source; optional {stream} placeholder")
    load.add_argument("--source-transport", choices=["tcp", "udp"], default="tcp")
    load.add_argument("--fps", type=int, default=30, help="Generated/expected source frame rate")
    load.add_argument("--duration", type=float, default=60, help="Measurement seconds per level")
    load.add_argument("--warmup", type=float, default=5)
    load.add_argument("--cooldown", type=float, default=2)
    load.add_argument("--timeout", type=float, default=10, help="Publisher startup and reader I/O timeout seconds")
    load.add_argument("--ping", default="", help="Gateway or peer IP to probe during measurement")
    load.add_argument("--ffprobe", default="ffprobe")
    load.add_argument("--decode", action="store_true", help="Decode received video instead of packet copy")
    load.add_argument("--min-frame-ratio", type=float, default=0.95)
    load.add_argument("--max-stall", type=float, default=3, help="Max seconds without frame progress")
    load.add_argument("--stop-on-failure", action="store_true")
    server = subparsers.add_parser("serve", parents=[common], help="Start an isolated MediaMTX instance")
    server.add_argument("--port", type=int, default=18554)
    server.add_argument("--rtp-port", type=int, default=18000)
    server.add_argument("--mediamtx", default="mediamtx")
    server.add_argument("--duration", type=float, default=0, help="0 runs until Ctrl+C")
    args = parser.parse_args()
    if not math.isfinite(args.duration) or args.duration < 0 or (args.command == "run" and (
            args.duration <= 0 or args.warmup < 0 or args.cooldown < 0 or args.timeout <= 0 or
            args.fps <= 0 or args.max_stall <= 0 or args.clip_seconds <= 0 or not 0 < args.min_frame_ratio <= 1)):
        parser.error("Invalid duration, warmup, cooldown, fps, timeout, stall or frame-ratio setting")
    if args.command == "run":
        args.transports = args.transports.split(",")
        if not args.transports or any(value not in ("tcp", "udp") for value in args.transports):
            parser.error("--transports must be tcp, udp or tcp,udp")
        if not re.fullmatch(r"[1-9]\d*x[1-9]\d*", args.size):
            parser.error("--size must be WIDTHxHEIGHT")
    for name in ([args.mediamtx] if args.command == "serve" else [args.ffmpeg, args.ffprobe]):
        if not shutil.which(name):
            parser.error(f"Missing executable: {name}")
    signal.signal(signal.SIGTERM, interrupt)
    signal.signal(signal.SIGHUP, interrupt)
    try:
        return serve(args) if args.command == "serve" else run(args)
    except KeyboardInterrupt:
        print("Interrupted; owned child processes stopped, partial results preserved", file=sys.stderr)
        return 130
    except (OSError, ValueError, RuntimeError, subprocess.SubprocessError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2
    finally:
        stop_children(CHILDREN)


if __name__ == "__main__":
    sys.exit(main())
