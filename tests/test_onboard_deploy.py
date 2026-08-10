"""机载最小部署脚本、Humble/Jazzy 兼容配置与硬件隔离护栏测试。"""

from __future__ import annotations

import os
import subprocess
from pathlib import Path
from xml.etree import ElementTree

from ground_station_core.config import PROJECT_ROOT


DEPLOY_DIR = PROJECT_ROOT / "src" / "onboard_control" / "deploy"
WORKSPACE_SCRIPT = DEPLOY_DIR / "onboard_workspace.sh"
DEPLOYMENT_GUIDE = DEPLOY_DIR / "ONBOARD_DEPLOYMENT.md"
DRONE_START_DIRECTORY = PROJECT_ROOT / "start_drone"
INTEGRATED_START = PROJECT_ROOT / "start_drone_all.sh"
GROUND_START = PROJECT_ROOT / "start_ground_all.sh"


def test_onboard_workspace_script_has_valid_shell_and_help() -> None:
    """脚本必须可执行、语法有效，并公开全部非破坏性部署阶段。"""
    assert os.access(WORKSPACE_SCRIPT, os.X_OK)
    syntax = subprocess.run(
        ["bash", "-n", str(WORKSPACE_SCRIPT)],
        check=False,
        capture_output=True,
        text=True,
    )
    assert syntax.returncode == 0, syntax.stderr

    help_result = subprocess.run(
        [str(WORKSPACE_SCRIPT), "--help"],
        check=False,
        capture_output=True,
        text=True,
    )
    assert help_result.returncode == 0, help_result.stderr
    for command in ("update", "deps-check", "build", "test", "smoke", "verify"):
        assert command in help_result.stdout


def test_onboard_checkout_and_smoke_test_are_hardware_isolated() -> None:
    """最小检出不得包含地面站，烟雾测试不得发现或控制真实 MAVROS。"""
    script = WORKSPACE_SCRIPT.read_text(encoding="utf-8")
    guide = DEPLOYMENT_GUIDE.read_text(encoding="utf-8")

    for sparse_path in (
        "/src/guided_interfaces/",
        "/src/onboard_control/",
        "/start_drone/",
        "/start_drone_all.sh",
    ):
        assert sparse_path in script
        assert sparse_path in guide
    assert "/ground_station_core/" not in script
    assert "/src/guided_sim/" not in script
    assert "/start_ground_all.sh" not in script
    assert "sparse-checkout set --no-cone" not in script

    assert 'DEFAULT_SMOKE_DOMAIN_ID="231"' in script
    assert "SMOKE_DOMAIN_ID >= 1" in script
    assert "ROS_LOCALHOST_ONLY=1" in script
    assert "ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST" in script
    assert 'SMOKE_MAVROS_PREFIX="/_task08_smoke_mavros"' in script
    assert "^armed: false$" in script
    assert "setpoint_messages=0" in script
    assert "socks5h://127.0.0.1:19080" in guide

    # 部署助手不得包含解锁、起飞、强制覆盖或系统安装操作。
    for forbidden in (
        "/cmd/arming",
        "/cmd/takeoff",
        "COMMAND_TAKEOFF",
        "git reset",
        "git clean",
        "rm -rf",
        "sudo apt",
    ):
        assert forbidden not in script


def test_onboard_runtime_dependencies_and_service_template_are_portable() -> None:
    """launch 运行依赖必须声明，服务模板不得写死用户、Jazzy 或旧 MAVROS 服务。"""
    package_root = ElementTree.parse(
        PROJECT_ROOT / "src" / "onboard_control" / "package.xml"
    ).getroot()
    runtime_dependencies = {
        node.text for node in package_root.findall("exec_depend")
    }
    assert {"ament_index_python", "launch", "launch_ros"} <= runtime_dependencies

    service = (DEPLOY_DIR / "onboard-control.service.example").read_text(
        encoding="utf-8"
    )
    environment = (DEPLOY_DIR / "onboard.env.example").read_text(encoding="utf-8")
    assert "User=ONBOARD_USER" in service
    assert "/opt/ros/jazzy" not in service
    assert "/opt/ros/${ROS_DISTRO}/setup.bash" in service
    assert "mavros.service" not in service
    assert "ROS_DISTRO=humble" in environment
    assert "ONBOARD_WORKSPACE=/home/onboard/ros2-ardupilot-mavros-control" in environment
    assert "ROS_LOCALHOST_ONLY=0" in environment


def test_integrated_start_supervises_all_four_components_without_flight_commands() -> None:
    """一键入口须复用实机参数、拒绝重复进程并只做只读就绪检查。"""
    assert os.access(INTEGRATED_START, os.X_OK)
    syntax = subprocess.run(
        ["bash", "-n", str(INTEGRATED_START)],
        check=False,
        capture_output=True,
        text=True,
    )
    assert syntax.returncode == 0, syntax.stderr

    script = INTEGRATED_START.read_text(encoding="utf-8")
    for command in (
        "ros2 launch mavros apm.launch",
        "ros2 launch odin_ros_driver odin1_ros2.launch.py",
        "ros2 run extnav_bridge extnav_to_vision_pose",
        "ros2 launch onboard_control control.launch.py",
    ):
        assert command in script
    for parameter in (
        "MAVROS_FCU_BAUD:-460800",
        "vision_rate_hz:=40.0",
        "ctrl_rate_hz:=100.0",
        "odin_x:=0.06",
        "odin_y:=-0.03",
        "odin_z:=0.05",
    ):
        assert parameter in script

    assert 'export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"' in script
    assert "refusing to create duplicate flight-stack processes" in script
    assert "kill -INT -- \"-${pid}\"" in script
    assert "message_rates_configured: true" in script
    assert "local_position_valid: true" in script

    for forbidden in (
        "/cmd/arming",
        "/cmd/takeoff",
        "COMMAND_TAKEOFF",
        "set_gp_origin",
        "FlightCommand",
    ):
        assert forbidden not in script


def test_synced_split_launchers_and_local_ground_launcher_are_well_scoped() -> None:
    """分步脚本须完整，地面一键入口不得包含机载或飞行操作。"""
    assert {
        path.name for path in DRONE_START_DIRECTORY.glob("*.sh")
    } == {
        "start_link.sh",
        "start_mavros.sh",
        "start_odin.sh",
        "start_extnav.sh",
    }
    for obsolete_name in (
        "start_all.sh",
        "start_drone.sh",
        "start_ground.sh",
        "check.sh",
    ):
        assert not (PROJECT_ROOT / obsolete_name).exists()

    assert os.access(GROUND_START, os.X_OK)
    syntax = subprocess.run(
        ["bash", "-n", str(GROUND_START)],
        check=False,
        capture_output=True,
        text=True,
    )
    assert syntax.returncode == 0, syntax.stderr

    script = GROUND_START.read_text(encoding="utf-8")
    assert 'export ROS_DOMAIN_ID="0"' in script
    assert "unset ROS_LOCALHOST_ONLY" in script
    assert "ROS_AUTOMATIC_DISCOVERY_RANGE" in script
    assert 'ground_station.py" "$@"' in script
    for forbidden in (
        "/dev/ttyTHS1",
        "mavros apm.launch",
        "odin_ros_driver",
        "extnav_bridge",
        "/cmd/arming",
        "/cmd/takeoff",
    ):
        assert forbidden not in script
