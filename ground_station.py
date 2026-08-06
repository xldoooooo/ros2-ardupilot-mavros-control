#!/usr/bin/env python3
"""ROS2-ArduPilot 地面站 GUI 入口。"""

from __future__ import annotations

import tkinter as tk

from ground_station_core.gui import GroundStationApp


def main() -> None:
    """创建 Tk 根窗口并运行地面站事件循环。"""
    root = tk.Tk()
    GroundStationApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()
