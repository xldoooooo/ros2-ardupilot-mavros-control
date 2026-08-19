"""确保 ROS launch_testing 插件收集测试时仍能导入仓库本地包。"""

from __future__ import annotations

import sys
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parent.parent
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

VIDEO_SERVICE_ROOT = PROJECT_ROOT / "video_service"
if str(VIDEO_SERVICE_ROOT) not in sys.path:
    sys.path.insert(0, str(VIDEO_SERVICE_ROOT))
