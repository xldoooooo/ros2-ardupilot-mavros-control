"""未手动 source 工作空间时的地面站入口自动引导回归测试。"""

from __future__ import annotations

import os
import signal
import subprocess
import sys
from xml.etree import ElementTree

from ground_station import _install_termination_signal_handlers, _TERMINATION_SIGNALS
import ground_station_core.config as project_config
from ground_station_core.config import INTERFACE_VERSION, PROJECT_ROOT


def test_terminal_signals_are_forwarded_and_handlers_are_restored() -> None:
    """终端关闭、Ctrl+C 与 TERM 必须进入 GUI 生命周期而非直接杀死进程。"""
    received: list[str] = []
    original = {
        signum: signal.getsignal(signum)
        for signum in _TERMINATION_SIGNALS
    }
    restore = _install_termination_signal_handlers(received.append)
    try:
        for signum in original:
            handler = signal.getsignal(signum)
            assert callable(handler)
            handler(signum, None)
    finally:
        restore()

    assert received == ["SIGHUP", "SIGINT", "SIGQUIT", "SIGTERM"]
    assert all(signal.getsignal(signum) == value for signum, value in original.items())


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


def test_local_cleanup_entry_runs_without_creating_the_gui() -> None:
    """父级启动壳必须能在 GUI 退出后独立执行幂等本地残留扫描。"""
    completed = subprocess.run(
        [
            sys.executable,
            str(PROJECT_ROOT / "ground_station.py"),
            "--cleanup-local-processes",
        ],
        cwd=PROJECT_ROOT,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        timeout=20.0,
        check=False,
    )

    assert completed.returncode == 0, completed.stdout
    assert "PySide6" not in completed.stdout


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

    assert INTERFACE_VERSION == "3.2"
    assert 'kInterfaceVersion[] = "3.2"' in onboard_source
    assert package_versions == {"3.2.0"}

    executor = onboard_source.split(
        "void OnboardControlNode::update_waypoint_executor", 1
    )[1].split("void OnboardControlNode::enforce_safety", 1)[0]
    assert executor.index("publish_waypoint_capture(waypoint)") < executor.index(
        "++waypoint_index_"
    )
    assert "rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local()" in (
        onboard_source
    )
    assert 'video_prefix_ + "/capture", rclcpp::QoS(256).reliable()' in (
        onboard_source
    )


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


def test_mavproxy_is_found_from_project_environment(monkeypatch, tmp_path) -> None:
    """项目 venv 未加入终端 PATH 时仍应为 SITL 找到 MAVProxy。"""
    project_root = tmp_path / "project"
    mavproxy = project_root / ".venv" / "bin" / "mavproxy.py"
    mavproxy.parent.mkdir(parents=True)
    mavproxy.write_text("#!/usr/bin/env python3\n", encoding="utf-8")
    mavproxy.chmod(0o755)

    monkeypatch.setattr(project_config, "PROJECT_ROOT", project_root)
    monkeypatch.setattr(project_config.sys, "executable", str(tmp_path / "python3"))
    monkeypatch.delenv("GROUND_STATION_MAVPROXY", raising=False)
    monkeypatch.setenv("PATH", str(tmp_path / "empty-bin"))
    monkeypatch.setenv("HOME", str(tmp_path / "empty-home"))

    assert project_config.find_mavproxy() == mavproxy.resolve()
