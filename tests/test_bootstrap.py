"""未手动 source 工作空间时的地面站入口自动引导回归测试。"""

from __future__ import annotations

import os
import subprocess
import sys
from xml.etree import ElementTree

import ground_station_core.config as project_config
from ground_station_core.config import INTERFACE_VERSION, PROJECT_ROOT


def test_direct_launcher_bootstraps_workspace_from_clean_environment() -> None:
    """用户直接运行 ground_station.py 时应自动找到生成的接口包。"""
    clean_environment = {
        "HOME": os.environ["HOME"],
        "PATH": os.environ["PATH"],
        "LANG": os.environ.get("LANG", "C.UTF-8"),
    }
    completed = subprocess.run(
        [
            sys.executable,
            str(PROJECT_ROOT / "ground_station.py"),
            "--check-environment",
        ],
        cwd=PROJECT_ROOT,
        env=clean_environment,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        timeout=20.0,
        check=False,
    )

    assert completed.returncode == 0, completed.stdout
    assert "workspace environment OK" in completed.stdout
    assert "No module named 'guided_interfaces'" not in completed.stdout


def test_protocol_version_is_synchronized_across_deployments() -> None:
    """线级消息变化必须让地面站、机载端和包版本同步升级。"""
    onboard_source = (
        PROJECT_ROOT / "src" / "onboard_control" / "src" / "onboard_control_node.cpp"
    ).read_text(encoding="utf-8")
    package_versions = {
        ElementTree.parse(PROJECT_ROOT / "src" / package / "package.xml")
        .getroot()
        .findtext("version")
        for package in ("guided_interfaces", "onboard_control")
    }

    assert INTERFACE_VERSION == "2.1"
    assert 'kInterfaceVersion[] = "2.1"' in onboard_source
    assert package_versions == {"2.1.0"}


def test_python_runtime_selects_an_installed_humble_underlay(monkeypatch, tmp_path) -> None:
    """未预先 source 时，22.04 部署可由安装根目录自动选择 Humble。"""
    ros_root = tmp_path / "ros"
    setup = ros_root / "humble" / "setup.bash"
    setup.parent.mkdir(parents=True)
    setup.touch()
    monkeypatch.delenv("ROS_DISTRO", raising=False)
    monkeypatch.setenv("ROS_INSTALL_ROOT", str(ros_root))

    assert project_config.detect_ros_distro() == "humble"
    assert project_config.ros_setup_file() == setup
    assert project_config.mavros_apm_config() == (
        ros_root / "humble" / "share" / "mavros" / "launch" / "apm_config.yaml"
    )


def test_sim_vehicle_is_found_from_a_checkout_neighbour(monkeypatch, tmp_path) -> None:
    """常见 ArduPilot 源码布局不应要求用户填写绝对路径。"""
    project_root = tmp_path / "project" / "ros-control"
    sim_vehicle = (
        project_root.parent
        / "ardupilot"
        / "Tools"
        / "autotest"
        / "sim_vehicle.py"
    )
    sim_vehicle.parent.mkdir(parents=True)
    sim_vehicle.touch()
    monkeypatch.setattr(project_config, "PROJECT_ROOT", project_root)
    monkeypatch.delenv("GROUND_STATION_SIM_VEHICLE", raising=False)
    monkeypatch.setenv("PATH", str(tmp_path / "empty-bin"))

    assert project_config.find_sim_vehicle() == sim_vehicle.resolve()
