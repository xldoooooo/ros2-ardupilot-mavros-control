"""地面站本机启动偏好与 Ubuntu 桌面/登录自启入口。"""

from __future__ import annotations

import json
import math
import os
from pathlib import Path

from .config import PROJECT_ROOT


# 桌面入口 ID 同时用于 Wayland 窗口关联及当前用户的登录自启文件。
DESKTOP_ID = "ros2-ardupilot-ground-station"
# 使用项目内矢量图标，桌面与 Qt 窗口保持一致。
ICON_PATH = PROJECT_ROOT / "ground_station_core" / "qt_ui" / "assets" / "ground-station.svg"
# JSON 属于主机本地偏好，已由 config/ground/*.json 的既有规则忽略。
PREFERENCES_PATH = PROJECT_ROOT / "config" / "ground" / "startup.json"


def load_preferences(path: Path = PREFERENCES_PATH) -> dict[str, object]:
    """读取自动连接与原点偏好；未创建文件时使用既有手动启动行为。"""
    if not path.exists():
        return {}
    preferences = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(preferences, dict):
        raise ValueError("启动偏好必须是 JSON 对象")
    origin = preferences.get("origin")
    if origin is None:
        preferences.pop("origin", None)
    else:
        if not isinstance(origin, list) or len(origin) != 3:
            raise ValueError("启动偏好中的原点必须包含纬度、经度、高度")
        origin = tuple(float(value) for value in origin)
        if not all(map(math.isfinite, origin)) or not (
            -90 <= origin[0] <= 90 and -180 <= origin[1] <= 180
        ):
            raise ValueError("启动偏好中的原点坐标无效")
        preferences["origin"] = origin
    return preferences


def save_preferences(
    auto_connect: bool, origin: tuple[float, float, float], path: Path = PREFERENCES_PATH
) -> None:
    """原子保存用户已选择的自动连接开关与对应实机原点。"""
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(".json.tmp")
    temporary.write_text(
        json.dumps({"auto_connect_aircraft": auto_connect, "origin": origin}, indent=2)
        + "\n",
        encoding="utf-8",
    )
    temporary.replace(path)


def autostart_path() -> Path:
    """返回当前桌面用户的 XDG 登录自启入口。"""
    config_home = Path(os.environ.get("XDG_CONFIG_HOME", Path.home() / ".config"))
    return config_home / "autostart" / f"{DESKTOP_ID}.desktop"


def _desktop_contents() -> str:
    """生成与本工作区绑定的桌面入口，交给既有完整启动脚本。"""
    script = str(PROJECT_ROOT / "scripts" / "ground" / "start_ground_all.sh")
    # Exec 由桌面规范解析；双引号内四个保留字符须转义，百分号须写成 %% 。
    escaped = script
    for character in ("\\", '"', "`", "$"):
        escaped = escaped.replace(character, "\\" + character)
    escaped = escaped.replace("%", "%%")
    return (
        "[Desktop Entry]\nType=Application\nVersion=1.0\n"
        "Name=ArduPilot 地面站\nComment=ROS 2 ArduPilot Ground Station\n"
        f'Exec=/bin/bash "{escaped}"\nIcon={ICON_PATH}\n'
        "Terminal=false\nCategories=Science;Robotics;\n"
        f"StartupWMClass={DESKTOP_ID}\nX-GNOME-Autostart-enabled=true\n"
    )


def install_desktop_launcher() -> Path:
    """安装当前用户的应用菜单入口，无需 sudo 或启动地面站。"""
    data_home = Path(os.environ.get("XDG_DATA_HOME", Path.home() / ".local" / "share"))
    path = data_home / "applications" / f"{DESKTOP_ID}.desktop"
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(_desktop_contents(), encoding="utf-8")
    return path


def set_login_autostart(enabled: bool) -> None:
    """只安装或移除本项目的用户登录自启项，不修改系统登录方式。"""
    path = autostart_path()
    if enabled:
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(_desktop_contents(), encoding="utf-8")
    else:
        path.unlink(missing_ok=True)


if __name__ == "__main__":
    print(install_desktop_launcher())
