"""地面站直接启动时自动加载 ROS 2 与本工作空间 overlay。"""

from __future__ import annotations

import importlib.util
import os
import sys
from pathlib import Path

from .config import INSTALL_SETUP, PROJECT_ROOT, ros_setup_file
from .process_manager import build_sourced_environment


_BOOTSTRAP_MARKER = "GROUND_STATION_WORKSPACE_BOOTSTRAPPED"


class WorkspaceBootstrapError(RuntimeError):
    """工作空间未构建或 source 后接口仍不可导入。"""


def _interfaces_available() -> bool:
    """检查接口是否确实来自当前仓库的 install 空间。"""
    install_prefix = INSTALL_SETUP.parent.resolve()
    colcon_prefixes = {
        Path(value).expanduser().resolve()
        for value in os.environ.get("COLCON_PREFIX_PATH", "").split(os.pathsep)
        if value
    }
    if install_prefix not in colcon_prefixes:
        return False
    try:
        specification = importlib.util.find_spec("guided_interfaces.msg")
    except ModuleNotFoundError:
        return False
    if specification is None or specification.origin is None:
        return False
    try:
        Path(specification.origin).resolve().relative_to(install_prefix)
    except ValueError:
        return False
    return True


def workspace_setup_files() -> tuple[Path, Path]:
    """返回需要按顺序加载的 ROS 发行版和本工作空间 setup。"""
    return ros_setup_file(), INSTALL_SETUP


def ensure_workspace_environment(entrypoint: Path | None = None) -> None:
    """必要时带 source 后的环境原位重启当前 Python 入口。"""
    if _interfaces_available():
        return

    setup_files = workspace_setup_files()
    missing = [str(path) for path in setup_files if not path.is_file()]
    build_command = (
        f"source {ros_setup_file()} && "
        "colcon build --packages-select guided_interfaces onboard_control guided_sim"
    )
    if missing:
        raise WorkspaceBootstrapError(
            "地面站依赖尚未构建或 ROS 2 未安装；缺少: "
            + ", ".join(missing)
            + f"\n请在仓库根目录执行：\n  {build_command}"
        )

    # marker 防止损坏的 install 空间导致无限 exec；第二次失败给出可执行修复命令。
    if os.environ.get(_BOOTSTRAP_MARKER) == "1":
        raise WorkspaceBootstrapError(
            "已加载工作空间，但仍无法导入 guided_interfaces。"
            f"\n请在 {PROJECT_ROOT} 重新执行：\n  {build_command}"
        )

    environment = build_sourced_environment(setup_files, base_environment=os.environ)
    environment[_BOOTSTRAP_MARKER] = "1"
    script = (entrypoint or Path(sys.argv[0])).expanduser().resolve()
    argv = [sys.executable, str(script), *sys.argv[1:]]
    os.execve(sys.executable, argv, environment)
