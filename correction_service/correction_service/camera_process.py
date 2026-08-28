"""按任务生命周期启动/停止带硬件 PTS 的下视相机节点并锁定镜头参数。"""

from __future__ import annotations

import logging
import os
import re
import shutil
import signal
import subprocess
import time
from pathlib import Path
from typing import TextIO

from .config import CameraSettings

_CONTROL_VALUE = re.compile(r":\s*(-?\d+)(?:\s|$)")


def parse_v4l2_control_value(output: str) -> int:
    """解析 `name: value`，允许 v4l2-ctl 在数值后附带枚举说明。"""
    matched = _CONTROL_VALUE.search(str(output))
    if matched is None:
        raise ValueError(f"无法解析 V4L2 控制读回：{str(output).strip()}")
    return int(matched.group(1))


class CameraProcessError(RuntimeError):
    """相机被占用、驱动失败或镜头参数未能读回。"""


class CameraProcess:
    """只管理本对象创建的 camera_node 进程组，不触碰视频或飞控服务。"""

    def __init__(
        self,
        settings: CameraSettings,
        lens_controls: dict[str, int],
        log_path: Path,
        logger: logging.Logger,
    ) -> None:
        self._settings = settings
        self._lens_controls = dict(lens_controls)
        self._log_path = log_path
        self._logger = logger
        self._process: subprocess.Popen[bytes] | None = None
        self._log_stream: TextIO | None = None

    @property
    def pid(self) -> int | None:
        """返回当前驱动 PID，供状态与资源测量使用。"""
        return self._process.pid if self._process is not None else None

    def start(self) -> None:
        """确认设备空闲后以独立进程组启动经过标定验收的相机 ROS 节点。"""
        if self._process is not None and self._process.poll() is None:
            raise CameraProcessError("下视相机进程已启动")
        device = Path(self._settings.device)
        if not device.exists():
            raise CameraProcessError(f"下视相机设备不存在：{device}")
        ros2 = shutil.which("ros2")
        if ros2 is None:
            raise CameraProcessError("找不到 ros2 命令")
        fuser = shutil.which("fuser")
        if fuser is not None:
            occupied = subprocess.run(
                [fuser, str(device)],
                capture_output=True,
                check=False,
                timeout=3.0,
            )
            if occupied.returncode == 0 and occupied.stdout.strip():
                users = occupied.stdout.decode(errors="replace").strip()
                raise CameraProcessError(f"下视相机已被其他进程占用（PID {users}）")

        command = [
            ros2,
            "run",
            self._settings.driver_package,
            self._settings.driver_executable,
            "--ros-args",
            "-p",
            f"video_device:={self._settings.device}",
            "-p",
            f"image_width:={self._settings.width}",
            "-p",
            f"image_height:={self._settings.height}",
            "-p",
            f"framerate:={self._settings.fps}",
            "-p",
            f"frame_id:={self._settings.frame_id}",
            "-p",
            f"image_topic:={self._settings.image_topic}",
            "-p",
            f"max_capture_age_ms:={self._settings.max_capture_age_ms}",
        ]
        self._log_path.parent.mkdir(parents=True, exist_ok=True)
        self._log_stream = self._log_path.open("a", encoding="utf-8")
        self._process = subprocess.Popen(
            command,
            stdin=subprocess.DEVNULL,
            stdout=self._log_stream,
            stderr=subprocess.STDOUT,
            start_new_session=True,
            close_fds=True,
        )
        self._logger.info(
            "下视相机节点已启动 pid=%s device=%s topic=%s",
            self._process.pid,
            self._settings.device,
            self._settings.image_topic,
        )

    def ensure_running(self) -> None:
        """把启动阶段或采样阶段的异常退出转成明确任务失败。"""
        if self._process is None:
            raise CameraProcessError("下视相机节点尚未启动")
        return_code = self._process.poll()
        if return_code is not None:
            raise CameraProcessError(
                f"下视相机节点异常退出 code={return_code}，详见 {self._log_path}"
            )

    def apply_lens_controls(self) -> None:
        """开流后逐项写入并读回标定镜头参数，任一不一致即失败。"""
        executable = shutil.which("v4l2-ctl")
        if executable is None:
            raise CameraProcessError("找不到 v4l2-ctl，无法锁定标定镜头参数")
        for name, expected in self._lens_controls.items():
            try:
                changed = subprocess.run(
                    [
                        executable,
                        "-d",
                        self._settings.device,
                        "--set-ctrl",
                        f"{name}={expected}",
                    ],
                    capture_output=True,
                    text=True,
                    timeout=3.0,
                    check=False,
                )
                if changed.returncode != 0:
                    raise CameraProcessError(
                        f"镜头参数 {name} 写入失败：{changed.stderr.strip()}"
                    )
                readback = subprocess.run(
                    [executable, "-d", self._settings.device, "--get-ctrl", name],
                    capture_output=True,
                    text=True,
                    timeout=3.0,
                    check=False,
                )
            except subprocess.TimeoutExpired as exc:
                raise CameraProcessError(f"镜头参数 {name} 操作超时") from exc
            if readback.returncode != 0:
                raise CameraProcessError(
                    f"镜头参数 {name} 读回失败：{readback.stderr.strip()}"
                )
            try:
                actual = parse_v4l2_control_value(readback.stdout)
            except ValueError as exc:
                raise CameraProcessError(
                    f"镜头参数 {name} 读回格式异常：{readback.stdout.strip()}"
                ) from exc
            if actual != expected:
                raise CameraProcessError(
                    f"镜头参数 {name} 读回 {actual}，期望 {expected}"
                )
        self._logger.info("下视相机镜头参数已全部写入并读回确认")

    def stop(self) -> None:
        """先 SIGINT 让 ROS/GStreamer 释放设备，再有限升级信号并确认退出。"""
        process = self._process
        self._process = None
        if process is not None and process.poll() is None:
            for sig, timeout_seconds in (
                (signal.SIGINT, 4.0),
                (signal.SIGTERM, 2.0),
                (signal.SIGKILL, 1.0),
            ):
                try:
                    os.killpg(process.pid, sig)
                except ProcessLookupError:
                    break
                try:
                    process.wait(timeout=timeout_seconds)
                    break
                except subprocess.TimeoutExpired:
                    continue
        if process is not None and process.poll() is None:
            raise CameraProcessError(f"无法终止本任务相机进程组 PID={process.pid}")
        if self._log_stream is not None:
            self._log_stream.flush()
            self._log_stream.close()
            self._log_stream = None
        # 设备节点有时会在 GStreamer 退出后几十毫秒才解除 fuser 映射。
        time.sleep(0.05)
        self._logger.info("下视相机节点已停止并释放设备")
