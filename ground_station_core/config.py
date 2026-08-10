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
# ExecuteWaypoints 请求结构加入 flight_strategy 后不再与 1.0 线级兼容。
INTERFACE_VERSION = "2.0"

# 完整实机连接时写入飞控的默认虚拟原点；本地 SITL 与 Wi-Fi 通讯检测不使用。
DEFAULT_GPS_ORIGIN = (30.2489634, 120.2052342, 488.0)


def ros_setup_files(extra: Iterable[Path] = ()) -> tuple[Path, ...]:
    """返回子进程需要依次 source 的 ROS/工作空间 setup 文件。"""
    distro = os.environ.get("ROS_DISTRO", "jazzy")
    candidates = [Path(f"/opt/ros/{distro}/setup.bash")]
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
    """从环境变量或 PATH 定位 ArduPilot 的 sim_vehicle.py。"""
    configured = os.environ.get("GROUND_STATION_SIM_VEHICLE")
    if configured:
        path = Path(configured).expanduser()
        return path if path.is_file() else None
    located = shutil.which("sim_vehicle.py")
    return Path(located).resolve() if located else None


def ardupilot_root(sim_vehicle: Path) -> Path:
    """根据 Tools/autotest/sim_vehicle.py 推导 ArduPilot 源码根目录。"""
    try:
        return sim_vehicle.resolve().parents[2]
    except IndexError:
        return sim_vehicle.resolve().parent


def mavros_apm_config() -> Path:
    """返回当前 ROS 发行版的 MAVROS APM 参数文件路径。"""
    distro = os.environ.get("ROS_DISTRO", "jazzy")
    return Path(f"/opt/ros/{distro}/share/mavros/launch/apm_config.yaml")
