"""地面站薄客户端、共享接口和本地 SITL 环境配置。"""

from __future__ import annotations

import os
import shutil
from pathlib import Path
from typing import Iterable


# 仓库根目录；所有项目内路径均从此处推导，避免依赖启动目录。
PROJECT_ROOT = Path(__file__).resolve().parent.parent
INSTALL_SETUP = PROJECT_ROOT / "install" / "setup.bash"
ONBOARD_PARAM_FILE = (
    PROJECT_ROOT
    / "src"
    / "onboard_control"
    / "config"
    / "control.yaml"
)

# 真机继续使用既有 domain 0；本地仿真固定放入仅本机发现的独立 domain，
# 从传输层杜绝同时运行的真机与 SITL 共享同名 topic/service。
HARDWARE_DOMAIN_ID = 0
SIMULATION_DOMAIN_ID = 231
HARDWARE_DISCOVERY_RANGE = "SUBNET"
SIMULATION_DISCOVERY_RANGE = "LOCALHOST"

# GUI 只产生高层意图；所有连续控制参数均在机载 C++ 参数文件中维护。
VELOCITY_SCALE = 0.2  # 每次按键发送的速度增量，单位 m/s（偏航为 rad/s）。
TAKEOFF_ALTITUDE = 0.3  # 默认起飞高度，单位 m。
TAKEOFF_SPEED = 2.5  # 本机 ArduPilot WP_SPD_UP 默认值，单位 m/s。
LAND_SPEED = 0.5  # 本机 ArduPilot LAND_SPD_MS 默认值，单位 m/s。
INTERFACE_PREFIX = os.environ.get("GROUND_STATION_INTERFACE_PREFIX", "/onboard_control")
COMMAND_TTL_MS = 1500  # 高层离散命令和按键意图的网络有效期。
LEASE_DURATION_MS = 1500  # 控制权需由 5 Hz 心跳持续续租。
HEARTBEAT_PERIOD_SECONDS = 0.2
# ControlStatus 2.2 增加滚转角；线级变更要求地面站与机载端同步部署。
INTERFACE_VERSION = "2.2"

# 手动操纵状态块的可调告警阈值；展示层不再散落硬编码数值。
STATUS_RATE_TARGET_HZ = 10.0
STATUS_RATE_TOLERANCE_HZ = 1.0
HARDWARE_BATTERY_WARNING_VOLTAGE = 22.5
HARDWARE_BATTERY_GOOD_VOLTAGE = 23.5
SIMULATION_BATTERY_WARNING_PERCENTAGE = 0.25
SIMULATION_BATTERY_GOOD_PERCENTAGE = 0.50

# 完整实机连接时写入飞控的默认虚拟原点；本地 SITL 与 Wi-Fi 通讯检测不使用。
DEFAULT_GPS_ORIGIN = (30.2489634, 120.2052342, 488.0)


def detect_ros_distro() -> str:
    """按显式环境、Ubuntu 版本和已安装目录依次选择 Humble/Jazzy。"""
    requested = os.environ.get("ROS_DISTRO", "").strip()
    if requested:
        return requested

    ros_root = Path(os.environ.get("ROS_INSTALL_ROOT", "/opt/ros"))
    os_version = ""
    try:
        for line in Path("/etc/os-release").read_text(encoding="utf-8").splitlines():
            if line.startswith("VERSION_ID="):
                os_version = line.partition("=")[2].strip().strip('"')
                break
    except OSError:
        pass

    preferred = {"22.04": "humble", "24.04": "jazzy"}.get(os_version)
    candidates = tuple(item for item in (preferred, "humble", "jazzy") if item)
    for candidate in candidates:
        if (ros_root / candidate / "setup.bash").is_file():
            return candidate
    # 保留原开发环境默认值，让缺失提示仍给出确定路径。
    return preferred or "jazzy"


def ros_setup_file() -> Path:
    """返回当前主机自动选择的 ROS underlay setup。"""
    ros_root = Path(os.environ.get("ROS_INSTALL_ROOT", "/opt/ros"))
    return ros_root / detect_ros_distro() / "setup.bash"


def ros_discovery_environment(discovery_range: str) -> dict[str, str]:
    """生成当前 ROS 发行版支持的 DDS 发现环境变量。"""
    normalized = discovery_range.strip().upper()
    if detect_ros_distro() == "humble":
        return {"ROS_LOCALHOST_ONLY": "1" if normalized == "LOCALHOST" else "0"}
    return {"ROS_AUTOMATIC_DISCOVERY_RANGE": normalized}


def ros_setup_files(extra: Iterable[Path] = ()) -> tuple[Path, ...]:
    """返回子进程需要依次 source 的 ROS/工作空间 setup 文件。"""
    candidates = [ros_setup_file()]
    if INSTALL_SETUP.is_file():
        candidates.append(INSTALL_SETUP)
    candidates.extend(Path(item).expanduser() for item in extra)

    # 保序去重，防止同一工作空间被重复 source。
    result: list[Path] = []
    for candidate in candidates:
        if candidate not in result:
            result.append(candidate)
    return tuple(result)


def find_sim_vehicle() -> Path | None:
    """从环境变量、PATH 和常见源码位置自动定位 sim_vehicle.py。"""
    configured = os.environ.get("GROUND_STATION_SIM_VEHICLE")
    if configured:
        path = Path(configured).expanduser()
        return path if path.is_file() else None
    located = shutil.which("sim_vehicle.py")
    if located:
        return Path(located).resolve()

    candidates = (
        PROJECT_ROOT.parent / "ardupilot" / "Tools" / "autotest" / "sim_vehicle.py",
        Path.home() / "ardupilot" / "Tools" / "autotest" / "sim_vehicle.py",
        Path.home() / "ArduPilot" / "Tools" / "autotest" / "sim_vehicle.py",
        Path("/opt/ardupilot/Tools/autotest/sim_vehicle.py"),
        Path("/usr/local/src/ardupilot/Tools/autotest/sim_vehicle.py"),
    )
    return next((path.resolve() for path in candidates if path.is_file()), None)


def ardupilot_root(sim_vehicle: Path) -> Path:
    """根据 Tools/autotest/sim_vehicle.py 推导 ArduPilot 源码根目录。"""
    try:
        return sim_vehicle.resolve().parents[2]
    except IndexError:
        return sim_vehicle.resolve().parent


def mavros_apm_config() -> Path:
    """返回当前 ROS 发行版的 MAVROS APM 参数文件路径。"""
    ros_root = Path(os.environ.get("ROS_INSTALL_ROOT", "/opt/ros"))
    return ros_root / detect_ros_distro() / "share/mavros/launch/apm_config.yaml"
