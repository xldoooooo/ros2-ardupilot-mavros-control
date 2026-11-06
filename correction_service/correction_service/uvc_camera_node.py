"""保留原相机命令入口，以exec加载原生采集节点，不增加常驻Python进程。"""

from __future__ import annotations

import os
from pathlib import Path
import sys

from ament_index_python.packages import get_package_prefix


def main(args: list[str] | None = None) -> None:
    """沿用原ROS参数、进程组和退出码，原生节点拥有fd和读取线程。"""
    executable = Path(get_package_prefix("correction_native")) / "lib/correction_native/uvc_camera_node"
    os.execv(str(executable), [str(executable), *(sys.argv[1:] if args is None else args)])


if __name__ == "__main__":
    main()
