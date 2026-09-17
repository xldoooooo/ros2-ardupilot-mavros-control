"""独立视频服务的安装、依赖与生命周期边界。"""

from __future__ import annotations

import os
import subprocess
from ground_station_core.config import PROJECT_ROOT


INTEGRATED_START = PROJECT_ROOT / "start_onboard_control.sh"


VIDEO_LAUNCHER = PROJECT_ROOT / "start_onboard_video.sh"


VIDEO_STOPPER = PROJECT_ROOT / "stop_onboard_video.sh"


VIDEO_INSTALLER = (
    PROJECT_ROOT / "video_service" / "deploy" / "install_onboard_video_service.sh"
)


VIDEO_SERVICE_UNIT = (
    PROJECT_ROOT / "video_service" / "deploy" / "video-service.service.example"
)


REMOVED_BUNDLED_MEDIAMTX = (
    PROJECT_ROOT / "video_service" / "bin" / "mediamtx" / "mediamtx"
)


def test_video_service_installer_is_one_command_and_preserves_isolation() -> None:
    """一键安装器应收拢系统配置，同时不得管理飞控 unit。"""
    assert os.access(VIDEO_INSTALLER, os.X_OK)
    syntax = subprocess.run(
        ["bash", "-n", str(VIDEO_INSTALLER)],
        check=False,
        capture_output=True,
        text=True,
    )
    assert syntax.returncode == 0, syntax.stderr

    help_result = subprocess.run(
        [str(VIDEO_INSTALLER), "--help"],
        check=False,
        capture_output=True,
        text=True,
    )
    assert help_result.returncode == 0, help_result.stderr
    assert "enable and start" in help_result.stdout
    assert "--install-only" in help_result.stdout

    installer = VIDEO_INSTALLER.read_text(encoding="utf-8")
    for required in (
        "/usr/local/bin/mediamtx",
        "/etc/ros2-ardupilot/camera.conf",
        "/home/share/jpg",
        "systemctl enable --now",
    ):
        assert required in installer
    assert "ros2-ardupilot-onboard.service" not in installer
    assert "apt-get" not in installer
    assert "github.com/bluenviron" not in installer
    assert "[[ ! -e /etc/ros2-ardupilot/camera.conf ]]" in installer


def test_mediamtx_is_a_system_dependency_not_a_repository_binary() -> None:
    """架构相关大文件不得回到 Git，默认路径必须指向系统安装。"""
    assert not REMOVED_BUNDLED_MEDIAMTX.exists()
    config_source = (
        PROJECT_ROOT / "video_service" / "camera_app" / "config.py"
    ).read_text(encoding="utf-8")
    onboard_config = (
        PROJECT_ROOT / "video_service" / "config" / "camera.conf"
    ).read_text(encoding="utf-8")
    assert "/usr/local/bin/mediamtx" in config_source
    assert 'bin" / "mediamtx' not in config_source
    assert "mediamtx_binary = /usr/local/bin/mediamtx" in onboard_config


def test_video_service_has_an_independent_lifecycle() -> None:
    """视频启动器和 unit 不得成为飞行四进程的共同故障域。"""
    integrated_start = INTEGRATED_START.read_text(encoding="utf-8")
    launcher = VIDEO_LAUNCHER.read_text(encoding="utf-8")
    service = VIDEO_SERVICE_UNIT.read_text(encoding="utf-8")
    stopper = VIDEO_STOPPER.read_text(encoding="utf-8")
    active_directives = [
        line.strip()
        for line in service.splitlines()
        if line.strip() and not line.lstrip().startswith("#")
    ]

    assert os.access(VIDEO_LAUNCHER, os.X_OK)
    assert os.access(VIDEO_STOPPER, os.X_OK)
    syntax = subprocess.run(
        ["bash", "-n", str(VIDEO_LAUNCHER)],
        check=False,
        capture_output=True,
        text=True,
    )
    assert syntax.returncode == 0, syntax.stderr
    stop_syntax = subprocess.run(
        ["bash", "-n", str(VIDEO_STOPPER)],
        check=False,
        capture_output=True,
        text=True,
    )
    assert stop_syntax.returncode == 0, stop_syntax.stderr
    stop_help = subprocess.run(
        [str(VIDEO_STOPPER), "--help"],
        check=False,
        capture_output=True,
        text=True,
    )
    assert stop_help.returncode == 0, stop_help.stderr
    assert "--restart" in stop_help.stdout
    assert "never stops or restarts ros2-ardupilot-onboard.service" in stop_help.stdout
    assert "stop_onboard_control.sh" not in stopper
    assert 'systemctl stop "${SERVICE_NAME}"' in stopper
    assert "/dev/v4l/by-id/*-video-index*" in stopper
    assert 'collect_fuser_pids "${rtsp_port}/tcp"' in stopper
    assert "ros2-ardupilot-onboard.service" not in stopper.replace(
        "This script never stops or restarts ros2-ardupilot-onboard.service.", ""
    )
    assert "video_service" not in integrated_start
    assert 'SYSTEM_CAMERA_CONFIG="/etc/ros2-ardupilot/camera.conf"' in launcher
    assert 'SYSTEM_LENS_CONFIG="/etc/ros2-ardupilot/lens.conf"' in launcher
    assert '"${VIDEO_SERVICE_ONBOARD_CONFIG:-}"' in launcher
    assert 'elif [[ -r "${system_config}" ]]' in launcher
    assert "LAN_ROUTE_TIMEOUT_SECONDS=30" in launcher
    assert "ip -4 route show default" in launcher
    assert "!/linkdown/" in launcher
    assert "wait_for_lan_route" in launcher
    assert not any(
        line.startswith(("Requires=", "PartOf=", "BindsTo="))
        for line in active_directives
    )
    assert "User=ONBOARD_USER" in service
    assert "WorkingDirectory=ONBOARD_WORKSPACE_PATH" in service
    assert "Environment=ONBOARD_WORKSPACE=ONBOARD_WORKSPACE_PATH" in service
    assert "${ONBOARD_WORKSPACE}/start_onboard_video.sh" in service
    assert "systemd-time-wait-sync.service" not in service
    assert "time-sync.target" not in service
    assert "Wants=network-online.target" in service
    assert "After=network-online.target" in service


def test_waypoint_capture_precedes_index_advance() -> None:
    """到点抓拍使用当前航点，并保留可靠事件 QoS。"""
    onboard_source = (
        PROJECT_ROOT / "src/onboard_control/src/onboard_control_node.cpp"
    ).read_text(encoding="utf-8")
    executor = onboard_source.split(
        "void OnboardControlNode::update_waypoint_executor", 1
    )[1].split("void OnboardControlNode::enforce_safety", 1)[0]
    assert executor.index("publish_waypoint_capture(waypoint)") < executor.index(
        "++waypoint_index_"
    )
    assert "rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local()" in (
        onboard_source
    )
    assert 'video_prefix_ + "/capture", rclcpp::QoS(256).reliable()' in (onboard_source)
