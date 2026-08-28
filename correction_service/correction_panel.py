#!/usr/bin/env python3
"""可由地面站分离启动、也可直接运行的 AprilTag-Odin 修正面板入口。"""

from __future__ import annotations

import os
import sys
from pathlib import Path

_PROJECT_ROOT = Path(__file__).resolve().parent.parent
if str(_PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(_PROJECT_ROOT))


def _configure_wayland_window_decorations() -> None:
    """在 Wayland 下选择与现有地面站子面板一致的 Adwaita 装饰。"""
    if os.environ.get(
        "XDG_SESSION_TYPE", ""
    ).casefold() == "wayland" and os.environ.get("WAYLAND_DISPLAY"):
        os.environ.setdefault("QT_WAYLAND_DECORATION", "adwaita")


def main() -> int:
    """配置显示后委托给已安装/源码内的 Qt 面板入口。"""
    _configure_wayland_window_decorations()
    from correction_service.correction_panel import main as panel_main

    return panel_main()


if __name__ == "__main__":
    raise SystemExit(main())
