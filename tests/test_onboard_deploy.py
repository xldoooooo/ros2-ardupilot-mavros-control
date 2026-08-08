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

    for sparse_path in ("/src/guided_interfaces/", "/src/onboard_control/"):
        assert sparse_path in script
        assert sparse_path in guide
    assert "/ground_station_core/" not in script
    assert "/src/guided_sim/" not in script

    assert 'DEFAULT_SMOKE_DOMAIN_ID="231"' in script
    assert "SMOKE_DOMAIN_ID >= 1" in script
    assert "ROS_LOCALHOST_ONLY=1" in script
    assert "ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST" in script
    assert 'SMOKE_MAVROS_PREFIX="/_task08_smoke_mavros"' in script
    assert "^armed: false$" in script
    assert "setpoint_messages=0" in script

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
