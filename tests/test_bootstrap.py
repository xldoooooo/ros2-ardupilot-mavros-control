"""未手动 source 工作空间时的地面站入口自动引导回归测试。"""

from __future__ import annotations

import os
import subprocess
import sys

from ground_station_core.config import PROJECT_ROOT


def test_direct_launcher_bootstraps_workspace_from_clean_environment() -> None:
    """用户直接运行 ground_station.py 时应自动找到生成的接口包。"""
    clean_environment = {
        "HOME": os.environ["HOME"],
        "PATH": os.environ["PATH"],
        "LANG": os.environ.get("LANG", "C.UTF-8"),
    }
    completed = subprocess.run(
        [
            sys.executable,
            str(PROJECT_ROOT / "ground_station.py"),
            "--check-environment",
        ],
        cwd=PROJECT_ROOT,
        env=clean_environment,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        timeout=20.0,
        check=False,
    )

    assert completed.returncode == 0, completed.stdout
    assert "workspace environment OK" in completed.stdout
    assert "No module named 'guided_interfaces'" not in completed.stdout
