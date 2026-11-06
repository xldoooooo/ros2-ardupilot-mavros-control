"""注册本地包，并隔离测试使用的 ROS、主机状态和进程清理范围。"""

from __future__ import annotations

import sys
import os
from functools import wraps
from pathlib import Path

import pytest

# ROS 插件收集前即限制发现范围，直接 pytest 与统一入口保持一致。
os.environ["ROS_AUTOMATIC_DISCOVERY_RANGE"] = "LOCALHOST"
if os.environ.get("ROS_DOMAIN_ID", "0") == "0":
    os.environ["ROS_DOMAIN_ID"] = "231"
for name in ("ROS_STATIC_PEERS", "ROS_DISCOVERY_SERVER", "ROS_LOCALHOST_ONLY"):
    os.environ.pop(name, None)

# 各独立源码包的位置，不依赖当前终端的 PYTHONPATH。
PROJECT_ROOT = Path(__file__).resolve().parent.parent
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

VIDEO_SERVICE_ROOT = PROJECT_ROOT / "video_service"
if str(VIDEO_SERVICE_ROOT) not in sys.path:
    sys.path.insert(0, str(VIDEO_SERVICE_ROOT))

CORRECTION_SERVICE_ROOT = PROJECT_ROOT / "correction_service"
if str(CORRECTION_SERVICE_ROOT) not in sys.path:
    sys.path.insert(0, str(CORRECTION_SERVICE_ROOT))


@pytest.fixture(autouse=True)
def isolated_host_state(monkeypatch, tmp_path):
    """主机偏好和通知队列仅写测试目录；显式测试路径仍由各用例决定。"""
    # 传输切换测试会修改进程环境；下一用例重新从隔离发现开始。
    monkeypatch.setenv("ROS_DOMAIN_ID", "231")
    monkeypatch.setenv("ROS_AUTOMATIC_DISCOVERY_RANGE", "LOCALHOST")
    for name in ("ROS_STATIC_PEERS", "ROS_DISCOVERY_SERVER", "ROS_LOCALHOST_ONLY"):
        monkeypatch.delenv(name, raising=False)
    for kind in ("STATE", "CONFIG", "DATA"):
        monkeypatch.setenv(f"XDG_{kind}_HOME", str(tmp_path / kind.lower()))
    module = sys.modules.get("ground_station_core.qt_ui.main_window")
    if module is not None:
        original = module.GroundStationWindow.__init__

        @wraps(original)
        def initialize(window, *args, **kwargs):
            """普通窗口测试不读取或改写操作者的启动偏好。"""
            kwargs.setdefault("startup_preferences_path", tmp_path / "startup.json")
            original(window, *args, **kwargs)

        monkeypatch.setattr(module.GroundStationWindow, "__init__", initialize)


@pytest.fixture(autouse=True)
def isolated_process_cleanup(monkeypatch, tmp_path):
    """保留真实匹配和信号处理，仅允许残留扫描选择本 pytest 的后代。"""
    from ground_station_core import process_manager

    ProcessSupervisor = process_manager.ProcessSupervisor
    monkeypatch.setattr(process_manager, "LOG_DIRECTORY", tmp_path / "process-logs")
    original = ProcessSupervisor.find_related_processes

    def find_related(cls):
        """不把开发者的仿真、RViz 或机载服务当作测试残留。"""
        descendants = cls._with_descendants({os.getpid()}) - {os.getpid()}
        return [(pid, command) for pid, command in original() if pid in descendants]

    monkeypatch.setattr(ProcessSupervisor, "find_related_processes", classmethod(find_related))
