"""机载最小部署脚本、Humble/Jazzy 兼容配置与硬件隔离护栏测试。"""

from __future__ import annotations

import os
import subprocess
import textwrap
from xml.etree import ElementTree

from ground_station_core.config import PROJECT_ROOT

DEPLOY_DIR = PROJECT_ROOT / "src" / "onboard_control" / "deploy"
WORKSPACE_SCRIPT = DEPLOY_DIR / "onboard_workspace.sh"
DEPLOYMENT_GUIDE = DEPLOY_DIR / "ONBOARD_DEPLOYMENT.md"
DRONE_START_DIRECTORY = PROJECT_ROOT / "start_drone"
INTEGRATED_START = PROJECT_ROOT / "start_onboard_control.sh"
INTEGRATED_STOP = PROJECT_ROOT / "stop_onboard_control.sh"
GROUND_START = PROJECT_ROOT / "start_ground_all.sh"
GROUND_SETUP = PROJECT_ROOT / "setup_ground_station.sh"
ONBOARD_BUILD = PROJECT_ROOT / "build_onboard_control.sh"
RUNTIME_HELPERS = DRONE_START_DIRECTORY / "runtime_common.bash"
VIDEO_LAUNCHER = PROJECT_ROOT / "start_onboard_video.sh"
VIDEO_STOPPER = PROJECT_ROOT / "stop_onboard_video.sh"
VIDEO_INSTALLER = (
    PROJECT_ROOT / "video_service" / "deploy" / "install_onboard_video_service.sh"
)
ONBOARD_INSTALLER = DEPLOY_DIR / "install_onboard_service.sh"
VIDEO_SERVICE_UNIT = (
    PROJECT_ROOT / "video_service" / "deploy" / "video-service.service.example"
)
REMOVED_BUNDLED_MEDIAMTX = (
    PROJECT_ROOT / "video_service" / "bin" / "mediamtx" / "mediamtx"
)


def run_bash(script: str, environment: dict[str, str] | None = None):
    """在隔离 bash 中调用可移植发现函数。"""
    return subprocess.run(
        ["bash", "--noprofile", "--norc", "-c", script],
        check=False,
        capture_output=True,
        text=True,
        env=environment,
    )


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
    assert "bin\" / \"mediamtx" not in config_source
    assert "mediamtx_binary = /usr/local/bin/mediamtx" in onboard_config


def test_onboard_service_installer_builds_and_installs_integrated_unit() -> None:
    """飞控安装器须一键构建和部署四组件 unit，且不含飞行命令。"""
    assert os.access(ONBOARD_INSTALLER, os.X_OK)
    syntax = subprocess.run(
        ["bash", "-n", str(ONBOARD_INSTALLER)],
        check=False,
        capture_output=True,
        text=True,
    )
    assert syntax.returncode == 0, syntax.stderr

    help_result = subprocess.run(
        [str(ONBOARD_INSTALLER), "--help"],
        check=False,
        capture_output=True,
        text=True,
    )
    assert help_result.returncode == 0, help_result.stderr
    installer = ONBOARD_INSTALLER.read_text(encoding="utf-8")
    for required in (
        "build_onboard_control.sh\" --verify",
        "/etc/ros2-ardupilot/onboard.env",
        "start_onboard_control.sh\" --check",
        "systemctl enable --now",
        "systemctl is-active",
    ):
        assert required in installer
    for forbidden in ("/cmd/arming", "/cmd/takeoff", "COMMAND_TAKEOFF"):
        assert forbidden not in installer


def test_root_onboard_build_entry_is_portable_and_safe() -> None:
    """根目录快捷入口须复用部署助手，并明确不管理服务或发送飞行命令。"""
    assert os.access(ONBOARD_BUILD, os.X_OK)
    syntax = subprocess.run(
        ["bash", "-n", str(ONBOARD_BUILD)],
        check=False,
        capture_output=True,
        text=True,
    )
    assert syntax.returncode == 0, syntax.stderr

    help_result = subprocess.run(
        [str(ONBOARD_BUILD), "--help"],
        check=False,
        capture_output=True,
        text=True,
    )
    assert help_result.returncode == 0, help_result.stderr
    assert "./build_onboard_control.sh" in help_result.stdout
    assert "--verify" in help_result.stdout

    script = ONBOARD_BUILD.read_text(encoding="utf-8")
    assert "src/onboard_control/deploy/onboard_workspace.sh" in script
    assert 'export ONBOARD_WORKSPACE="${project_root}"' in script
    assert "command=build" in script
    assert "command=verify" in script
    for forbidden in (
        "systemctl",
        "/cmd/arming",
        "/cmd/takeoff",
        "COMMAND_TAKEOFF",
    ):
        assert forbidden not in script


def test_runtime_discovery_resolves_ros_python_and_unique_serial(tmp_path) -> None:
    """常规 22.04/Humble 布局应无需编辑绝对路径即可被唯一解析。"""
    ros_root = tmp_path / "ros"
    humble_setup = ros_root / "humble" / "setup.bash"
    humble_setup.parent.mkdir(parents=True)
    humble_setup.write_text("export ROS_DISTRO=humble\n", encoding="utf-8")

    project_root = tmp_path / "checkout"
    project_python = project_root / ".venv" / "bin" / "python3"
    project_python.parent.mkdir(parents=True)
    project_python.write_text("#!/usr/bin/env sh\n", encoding="utf-8")
    project_python.chmod(0o755)

    dev_root = tmp_path / "dev"
    serial_target = dev_root / "ttyACM0"
    serial_target.parent.mkdir()
    serial_target.touch()
    stable_device = dev_root / "serial" / "by-id" / "flight-controller"
    stable_device.parent.mkdir(parents=True)
    stable_device.symlink_to(serial_target)

    command = textwrap.dedent(
        f"""
        source {RUNTIME_HELPERS!s}
        runtime_detect_ros_setup humble
        runtime_detect_python {project_root!s}
        runtime_detect_fcu_device
        """
    )
    environment = os.environ.copy()
    environment.update(
        ROS_INSTALL_ROOT=str(ros_root),
        RUNTIME_DEV_ROOT=str(dev_root),
    )
    result = run_bash(command, environment)

    assert result.returncode == 0, result.stderr
    assert result.stdout.splitlines() == [
        str(humble_setup),
        str(project_python),
        str(stable_device),
    ]


def test_runtime_rejects_stale_onboard_install_versions(tmp_path) -> None:
    """源码接口升级后，启动器不得继续运行旧 install 中的二进制。"""
    project_root = tmp_path / "checkout"
    source_manifest = project_root / "src" / "guided_interfaces" / "package.xml"
    installed_manifest = (
        project_root
        / "install"
        / "guided_interfaces"
        / "share"
        / "guided_interfaces"
        / "package.xml"
    )
    source_manifest.parent.mkdir(parents=True)
    installed_manifest.parent.mkdir(parents=True)
    source_manifest.write_text(
        "<package><version>3.1.0</version></package>\n", encoding="utf-8"
    )
    installed_manifest.write_text(
        "<package><version>2.2.0</version></package>\n", encoding="utf-8"
    )

    command = (
        f"source {RUNTIME_HELPERS!s}; "
        f"runtime_verify_workspace_package_install {project_root!s} guided_interfaces"
    )
    stale = run_bash(command)
    assert stale.returncode != 0
    assert "source=3.1.0, installed=2.2.0" in stale.stderr

    installed_manifest.write_text(
        "<package><version>3.1.0</version></package>\n", encoding="utf-8"
    )
    current = run_bash(command)
    assert current.returncode == 0, current.stderr


def test_ubuntu_2204_prefers_humble_when_both_distros_are_installed(
    tmp_path,
) -> None:
    """师兄的 22.04 主机即使残留另一发行版，也应自动使用 Humble。"""
    ros_root = tmp_path / "ros"
    for distro in ("humble", "jazzy"):
        setup = ros_root / distro / "setup.bash"
        setup.parent.mkdir(parents=True)
        setup.touch()
    os_release = tmp_path / "os-release"
    os_release.write_text('VERSION_ID="22.04"\n', encoding="utf-8")

    environment = os.environ.copy()
    environment.update(
        ROS_INSTALL_ROOT=str(ros_root),
        RUNTIME_OS_RELEASE_FILE=str(os_release),
        ROS_DISTRO="jazzy",
    )
    result = run_bash(
        f"source {RUNTIME_HELPERS!s}; runtime_detect_ros_setup",
        environment,
    )

    assert result.returncode == 0, result.stderr
    assert result.stdout.strip() == str(ros_root / "humble" / "setup.bash")


def test_runtime_discovery_rejects_ambiguous_serial_devices(tmp_path) -> None:
    """多个串口候选时必须安全失败，禁止自动猜测飞控。"""
    dev_root = tmp_path / "dev"
    stable_root = dev_root / "serial" / "by-id"
    stable_root.mkdir(parents=True)
    for name in ("controller-a", "controller-b"):
        target = dev_root / f"tty-{name}"
        target.touch()
        (stable_root / name).symlink_to(target)

    environment = os.environ.copy()
    environment["RUNTIME_DEV_ROOT"] = str(dev_root)
    result = run_bash(
        f"source {RUNTIME_HELPERS!s}; runtime_detect_fcu_device",
        environment,
    )

    assert result.returncode != 0
    assert "multiple stable serial devices" in result.stderr
    assert "refusing to guess" in result.stderr


def test_runtime_discovery_finds_the_overlay_that_owns_a_package(tmp_path) -> None:
    """Odin/extnav 应由 ament 索引反查 overlay，不依赖用户名或工作区名称。"""
    prefix = tmp_path / "renamed-workspace" / "install"
    marker = (
        prefix
        / "share"
        / "ament_index"
        / "resource_index"
        / "packages"
        / "fixture_portable_package"
    )
    marker.parent.mkdir(parents=True)
    marker.touch()
    setup = prefix / "setup.bash"
    setup.write_text("true\n", encoding="utf-8")

    environment = os.environ.copy()
    environment["RUNTIME_OVERLAY_SEARCH_ROOTS"] = str(tmp_path)
    result = run_bash(
        f"source {RUNTIME_HELPERS!s}; "
        "runtime_find_package_setup fixture_portable_package",
        environment,
    )

    assert result.returncode == 0, result.stderr
    assert result.stdout.strip() == str(setup)


def test_onboard_checkout_and_smoke_test_are_hardware_isolated() -> None:
    """最小检出不得包含地面站，烟雾测试不得发现或控制真实 MAVROS。"""
    script = WORKSPACE_SCRIPT.read_text(encoding="utf-8")
    guide = DEPLOYMENT_GUIDE.read_text(encoding="utf-8")

    for sparse_path in (
        "/src/guided_interfaces/",
        "/src/onboard_control/",
        "/video_service/",
        "/start_onboard_video.sh",
        "/stop_onboard_video.sh",
        "/start_drone/",
        "/start_onboard_control.sh",
        "/stop_onboard_control.sh",
        "/build_onboard_control.sh",
    ):
        assert sparse_path in script
        assert sparse_path in guide
    assert "video_service/deploy/install_onboard_video_service.sh" in script
    assert "src/onboard_control/deploy/install_onboard_service.sh" in script
    assert "./src/onboard_control/deploy/install_onboard_service.sh" in guide
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
    assert "systemctl stop \"${SERVICE_NAME}\"" in stopper
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
    assert not any(
        line.startswith(("Requires=", "PartOf=", "BindsTo="))
        for line in active_directives
    )
    assert "User=ONBOARD_USER" in service
    assert "WorkingDirectory=ONBOARD_WORKSPACE_PATH" in service
    assert "Environment=ONBOARD_WORKSPACE=ONBOARD_WORKSPACE_PATH" in service
    assert '${ONBOARD_WORKSPACE}/start_onboard_video.sh' in service


def test_onboard_runtime_dependencies_and_service_template_are_portable() -> None:
    """运行依赖必须声明，生产模板须执行四组件入口且不写死用户路径。"""
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
    assert "WorkingDirectory=ONBOARD_WORKSPACE_PATH" in service
    assert '${ONBOARD_WORKSPACE}/start_onboard_control.sh' in service
    assert "mavros.service" not in service
    assert "systemd-time-wait-sync.service" not in service
    assert "time-sync.target" not in service
    assert "After=network-online.target" in service
    assert "ONBOARD_ROS_DISTRO=ONBOARD_ROS_DISTRO_VALUE" in environment
    assert "ONBOARD_WORKSPACE=ONBOARD_WORKSPACE_PATH" in environment
    assert "ROS_LOCALHOST_ONLY=0" in environment
    assert "ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET" in environment
    assert "MAVROS_FCU_DEVICE=" in environment
    assert "ODIN_OVERLAY_SETUP=" in environment
    assert "EXTNAV_OVERLAY_SETUP=" in environment


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
        "runtime_detect_fcu_device",
        "MAVROS_FCU_BAUD:-460800",
        "vision_rate_hz:=40.0",
        "ctrl_rate_hz:=100.0",
        "odin_x:=0.06",
        "odin_y:=-0.03",
        "odin_z:=0.05",
    ):
        assert parameter in script

    assert 'export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"' in script
    assert "runtime_detect_ros_setup" in script
    assert "runtime_ensure_package odin_ros_driver" in script
    assert "runtime_ensure_package extnav_bridge" in script
    assert "ONBOARD_ENV_FILE:-/etc/ros2-ardupilot/onboard.env" in script
    assert 'runtime_source_setup "${onboard_environment_file}"' in script
    assert "--check" in script
    assert "refusing to create duplicate flight-stack processes" in script
    assert "kill -INT -- \"-${pid}\"" in script
    assert "message_rates_configured: true" in script
    assert "local_position_valid: true" in script
    assert "runtime_verify_workspace_package_install" in script
    assert "--no-daemon" in script
    assert "--qos-reliability best_effort" in script
    assert "guided_interfaces/msg/ControlStatus" in script

    for forbidden in (
        "/home/xld",
        "/dev/ttyTHS1",
        'humble_setup="/opt/ros/humble',
        "/cmd/arming",
        "/cmd/takeoff",
        "COMMAND_TAKEOFF",
        "set_gp_origin",
        "FlightCommand",
    ):
        assert forbidden not in script


def test_integrated_stop_cleans_managed_and_manual_flight_stack_processes() -> None:
    """停止入口须覆盖服务与手工进程，分级清理并验证零残留。"""
    assert os.access(INTEGRATED_STOP, os.X_OK)
    syntax = subprocess.run(
        ["bash", "-n", str(INTEGRATED_STOP)],
        check=False,
        capture_output=True,
        text=True,
    )
    assert syntax.returncode == 0, syntax.stderr

    script = INTEGRATED_STOP.read_text(encoding="utf-8")
    for required in (
        "systemctl stop",
        "start_onboard_control\\.sh",
        "mavros_node",
        "odin1_ros2",
        "host_sdk_sample",
        "extnav_to_vision_pose",
        "onboard_control_node",
        "signal_targets INT",
        "signal_targets TERM",
        "signal_targets KILL",
        "systemctl is-active --quiet",
    ):
        assert required in script
    for forbidden in (
        "/cmd/arming",
        "/cmd/takeoff",
        "COMMAND_TAKEOFF",
        "pkill -f ros2",
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
        "start_drone_all.sh",
        "stop_onboard_service.sh",
        "setup_project.sh",
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
    assert "runtime_detect_ros_setup" in script
    assert "runtime_detect_python" in script
    assert "/home/nvidia" not in script
    assert "/opt/ros/jazzy" not in script
    for forbidden in (
        "/dev/ttyTHS1",
        "mavros apm.launch",
        "odin_ros_driver",
        "extnav_bridge",
        "/cmd/arming",
        "/cmd/takeoff",
    ):
        assert forbidden not in script


def test_ground_setup_is_checkout_relative_and_never_contacts_hardware() -> None:
    """完整地面站入口应自动构建本检出，且不接触 MAVROS、串口或飞行命令。"""
    assert os.access(GROUND_SETUP, os.X_OK)
    syntax = subprocess.run(
        ["bash", "-n", str(GROUND_SETUP)],
        check=False,
        capture_output=True,
        text=True,
    )
    assert syntax.returncode == 0, syntax.stderr

    script = GROUND_SETUP.read_text(encoding="utf-8")
    assert "runtime_detect_ros_setup" in script
    assert 'project_root}/.venv' in script
    assert "guided_interfaces onboard_control guided_sim" in script
    assert "--symlink-install" not in script
    assert "ground_station.py --check-environment" in script
    assert "[ground-setup]" in script
    for media_dependency in (
        "ffmpeg",
        "ffprobe",
        "v4l2-ctl",
        "/usr/local/bin/mediamtx",
        "mediamtx --version",
        "video_service/README.md",
    ):
        assert media_dependency in script
    for forbidden in (
        "/home/nvidia",
        "/home/xld",
        "/opt/ros/jazzy",
        "/opt/ros/humble",
        "mavros apm.launch",
        "/cmd/arming",
        "/cmd/takeoff",
        "systemctl enable",
        "install_onboard_service.sh",
        "install_onboard_video_service.sh",
    ):
        assert forbidden not in script
