"""地面站路径、飞控参数和外部环境配置。"""

from __future__ import annotations

import os
import shutil
from pathlib import Path
from typing import Iterable


# 仓库根目录；所有项目内路径均从此处推导，避免依赖启动目录。
PROJECT_ROOT = Path(__file__).resolve().parent.parent
INSTALL_SETUP = PROJECT_ROOT / "install" / "setup.bash"
PARAM_FILE = (
    PROJECT_ROOT
    / "src"
    / "guided_sim"
    / "params"
    / "keyboard_vel_controller.yaml"
)

# GUI 飞行控制默认值。
VELOCITY_SCALE = 0.2  # 每次按键累加的速度步长，单位 m/s（偏航为 rad/s）。
PUBLISH_TOPIC = "/mavros/setpoint_velocity/cmd_vel_unstamped"
PUBLISH_RATE_HZ = 100.0
TAKEOFF_ALTITUDE = 0.3  # 默认起飞高度，单位 m。
GRAVITY_ACC = 9.8  # 与原 C++ 控制器保持一致的重力加速度，单位 m/s²。

# MAVLink 消息 ID 及期望频率：LOCAL_POSITION_NED、ATTITUDE_QUATERNION、HIGHRES_IMU。
MESSAGE_INTERVALS = ((32, 100.0), (31, 100.0), (105, 100.0))

# 实机链路保留 odin1.sh 的默认值，并允许部署机通过环境变量覆盖路径。
ODIN_SETUP = Path(
    os.environ.get("GROUND_STATION_ODIN_SETUP", "~/ws/install/setup.bash")
).expanduser()
EXTNAV_SETUP = Path(
    os.environ.get("GROUND_STATION_EXTNAV_SETUP", "~/vrpn_mavros/install/setup.bash")
).expanduser()
REAL_FCU_URL = os.environ.get(
    "GROUND_STATION_REAL_FCU_URL", "/dev/ttyTHS1:460800"
)

# GUI 中 GPS 原点的原有默认坐标。
DEFAULT_GPS_ORIGIN = (30.2489634, 120.2052342, 488.0)

# PD+DOB 悬停默认参数；运行时优先复用项目 YAML。
HOVER_PARAM_DEFAULTS = {
    "hover_wn_xy": 2.236,
    "hover_zeta_xy": 0.8,
    "hover_wn_z": 2.236,
    "hover_zeta_z": 0.6,
    "dob_L_xy": 1.5,
    "dob_L_z": 0.6,
    "hover_throttle": 0.2,
    "thrust_ratio": 2.5,
    "uav_weight": 1.7,
}


def load_hover_params() -> dict[str, float]:
    """读取项目 YAML 中的悬停参数，缺项或解析失败时使用安全默认值。"""
    merged = dict(HOVER_PARAM_DEFAULTS)
    if not PARAM_FILE.is_file():
        return merged

    try:
        import yaml

        with PARAM_FILE.open("r", encoding="utf-8") as stream:
            data = yaml.safe_load(stream) or {}
        root = data.get("/**", data) if isinstance(data, dict) else {}
        params = root.get("ros__parameters", root) if isinstance(root, dict) else {}
        for key in HOVER_PARAM_DEFAULTS:
            if key in params:
                merged[key] = float(params[key])
    except (OSError, TypeError, ValueError, ImportError) as exc:
        print(f"[GS] 读取悬停参数失败 ({PARAM_FILE}): {exc}", flush=True)
    return merged


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
