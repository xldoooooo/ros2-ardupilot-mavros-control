"""独立部署边界的辅助静态契约与隔离生产启动器执行测试。"""

from __future__ import annotations

import os
import subprocess
import shutil

import pytest
from pathlib import Path
from xml.etree import ElementTree

# Repository deployment and source boundaries under test.
PROJECT_ROOT = Path(__file__).resolve().parents[2]
CORRECTION_ROOT = PROJECT_ROOT / "correction_service"
CORRECTION_START = PROJECT_ROOT / "scripts/onboard/start_onboard_correction.sh"
CORRECTION_STOP = PROJECT_ROOT / "scripts/onboard/stop_onboard_correction.sh"


def _text(path: Path) -> str:
    """读取仓库 UTF-8 文本。"""
    return path.read_text(encoding="utf-8")


def test_correction_20_source_manifests_and_generators_are_synchronized() -> None:
    """源码包版本与 2.0 新消息/服务生成清单必须一起升级。"""
    manifests = (
        PROJECT_ROOT / "src/correction_interfaces/package.xml",
        CORRECTION_ROOT / "package.xml",
        CORRECTION_ROOT / "extnav_patch/package.xml",
    )
    for manifest in manifests:
        assert ElementTree.parse(manifest).getroot().findtext("version") == "2.0.0"
    cmake = _text(PROJECT_ROOT / "src/correction_interfaces/CMakeLists.txt")
    for interface in (
        "msg/CalibrationKeyframe.msg",
        "srv/ClearWindow.srv",
        "srv/ApplySavedCorrection.srv",
    ):
        assert interface in cmake


def test_extnav_installer_refuses_running_chain_and_backs_up_before_install() -> None:
    """辅助静态清单：构建和定点备份要求；拒绝运行链由隔离执行验证。"""
    script = _text(CORRECTION_ROOT / "deploy" / "install_extnav_correction.sh")

    assert 'systemctl is-active --quiet "${FLIGHT_SERVICE}"' in script
    assert "pgrep -af '[e]xtnav_to_vision_pose'" in script
    assert script.index("sha256sum -c SHA256SUMS") < script.index(
        'mv -- "${BACKUP_DIR}/staging" "${EXTNAV_PACKAGE_ROOT}"'
    )
    assert 'colcon build --base-paths "${EXTNAV_PACKAGE_ROOT}" --packages-select extnav_bridge' in script
    assert "-DCMAKE_BUILD_TYPE=Release" in script
    assert 'CORRECTION_PACKAGE_VERSION="2.0.0"' in script
    assert "installed ExtnavCorrectionStatus is not interface 2.0" in script
    assert "final_sample_revision" in script
    assert "systemctl restart" not in script
    assert "systemctl start" not in script
    assert "/mavros/cmd/arming" not in script


@pytest.mark.parametrize("running", ["service", "process"])
def test_extnav_installer_refuses_running_chain_before_build_or_write(tmp_path, running):
    """真实安装脚本遇到活动链时退出，不能构建或修改已有桥源码。"""
    target = tmp_path / "external/src/extnav_bridge"
    target.mkdir(parents=True)
    manifest = target / "package.xml"
    manifest.write_text("existing bridge\n")
    tools = tmp_path / "bin"
    tools.mkdir()
    calls = tmp_path / "calls"
    scripts = {
        "id": "printf '1000\\n'\n",
        "systemctl": 'printf "systemctl\\n" >> "$TEST_CALLS"\n[[ "$TEST_RUNNING" == service ]]\n',
        "pgrep": 'printf "pgrep\\n" >> "$TEST_CALLS"\n[[ "$TEST_RUNNING" == process ]]\n',
        "colcon": 'printf "colcon\\n" >> "$TEST_CALLS"\nexit 99\n',
    }
    for name, content in scripts.items():
        executable = tools / name
        executable.write_text("#!/usr/bin/env bash\n" + content)
        executable.chmod(0o755)
    environment = dict(os.environ, PATH=f"{tools}:{os.environ['PATH']}",
                       EXTNAV_WORKSPACE=str(tmp_path / "external"),
                       EXTNAV_BACKUP_ROOT=str(tmp_path / "backups"),
                       TEST_CALLS=str(calls), TEST_RUNNING=running)
    result = subprocess.run(
        ["bash", str(CORRECTION_ROOT / "deploy/install_extnav_correction.sh")],
        env=environment, capture_output=True, text=True, timeout=10,
    )
    assert result.returncode != 0
    assert ("is active" if running == "service" else "still running") in result.stderr
    assert calls.read_text().splitlines() == (["systemctl"] if running == "service" else ["systemctl", "pgrep"])
    assert manifest.read_text() == "existing bridge\n"
    assert not (tmp_path / "backups").exists()


def test_correction_unit_has_no_flight_service_dependency() -> None:
    """辅助静态契约：修正服务须独立部署；idle 行为由节点测试验证。"""
    unit = _text(PROJECT_ROOT / "system/systemd/odin-correction.service")
    installer_path = CORRECTION_ROOT / "deploy" / "install_correction_service.sh"
    installer = _text(installer_path)

    assert os.access(installer_path, os.X_OK)
    help_result = subprocess.run(
        [str(installer_path), "--help"],
        check=False,
        capture_output=True,
        text=True,
    )
    assert help_result.returncode == 0, help_result.stderr
    assert "--install-only" in help_result.stdout

    assert "ExecStart=ONBOARD_WORKSPACE_PATH/scripts/onboard/start_onboard_correction.sh" in unit
    assert "CORRECTION_CAMERA_OVERLAY_SETUP" not in unit
    assert "/etc/ros2-ardupilot" not in unit
    assert "/etc/ros2-ardupilot" not in installer
    assert "project_config_load" in installer
    assert "SupplementaryGroups=video" in unit
    assert "KillMode=control-group" in unit
    assert "ros2-ardupilot-onboard.service" not in unit
    active_lines = tuple(
        line.strip() for line in unit.splitlines() if not line.lstrip().startswith("#")
    )
    for directive in ("Requires=", "PartOf=", "BindsTo="):
        assert not any(line.startswith(directive) for line in active_lines)
    assert 'systemctl is-active --quiet "${FLIGHT_SERVICE}"' in installer
    assert "correction_interfaces correction_native correction_service" in installer
    assert 'CORRECTION_PACKAGE_VERSION="2.0.0"' in installer
    assert "installed CorrectionStatus is not 2.0" in installer
    assert "correction_interfaces/srv/ApplySavedCorrection" in installer
    assert "correction_interfaces/srv/ClearWindow" in installer
    assert 'systemctl enable --now "${SERVICE_NAME}"' in installer
    assert "/mavros/cmd/arming" not in installer


def test_correction_root_launchers_are_independent_and_executable() -> None:
    """修正启停入口必须可独立运行，且不得扩大到其他机载服务。"""
    for path in (CORRECTION_START, CORRECTION_STOP):
        assert os.access(path, os.X_OK)
        syntax = subprocess.run(
            ["bash", "-n", str(path)],
            check=False,
            capture_output=True,
            text=True,
        )
        assert syntax.returncode == 0, syntax.stderr
        help_result = subprocess.run(
            [str(path), "--help"],
            check=False,
            capture_output=True,
            text=True,
        )
        assert help_result.returncode == 0, help_result.stderr

    start = _text(CORRECTION_START)
    stop = _text(CORRECTION_STOP)
    assert "exec ros2 run correction_service correction_node" in start
    assert "ROS_AUTOMATIC_DISCOVERY_RANGE" in start
    assert "correction_service is already running" in start
    assert 'systemctl stop "${SERVICE_NAME}"' in stop
    assert "signal_targets INT" in stop
    assert "signal_targets TERM" in stop
    assert "signal_targets KILL" in stop
    assert "active extnav correction is unchanged" in stop
    combined = start + stop
    for forbidden in (
        "mavros_node",
        "odin1_ros2",
        "extnav_to_vision_pose",
        "onboard_control_node",
        "video-service.service",
        "/mavros/cmd/arming",
        "/mavros/cmd/takeoff",
    ):
        assert forbidden not in combined


def test_onboard_sparse_checkout_and_build_include_correction_packages() -> None:
    """正常部署构建修正包，但飞行启动不能把可选接口变成单点故障。"""
    helper = _text(
        PROJECT_ROOT / "src" / "onboard_control" / "deploy" / "onboard_workspace.sh"
    )
    launcher = _text(PROJECT_ROOT / "scripts/onboard/start_onboard_control.sh")

    assert 'CORRECTION_INTERFACES_SPARSE_PATH="/src/correction_interfaces/"' in helper
    assert 'CORRECTION_NATIVE_SPARSE_PATH="/src/correction_native/"' in helper
    assert 'CORRECTION_SERVICE_SPARSE_PATH="/correction_service/"' in helper
    assert 'CORRECTION_START_SPARSE_PATH="/scripts/onboard/start_onboard_correction.sh"' in helper
    assert 'CORRECTION_STOP_SPARSE_PATH="/scripts/onboard/stop_onboard_correction.sh"' in helper
    assert (
        "guided_interfaces correction_interfaces correction_native onboard_control correction_service"
        in helper
    )
    assert "for onboard_package in guided_interfaces onboard_control" in launcher
    assert (
        "for onboard_package in guided_interfaces correction_interfaces" not in launcher
    )


def test_correction_sources_contain_no_arm_takeoff_or_mode_command_endpoint() -> None:
    """辅助静态检查：覆盖 Python、C++ 与 shell 的直接飞控命令引用。"""
    forbidden = (
        "/mavros/cmd/arming",
        "/mavros/cmd/takeoff",
        "/mavros/set_mode",
        "FlightCommand",
        "MotionIntent",
    )
    source_files = [
        path for path in CORRECTION_ROOT.rglob("*")
        if path.is_file() and (path.suffix in {".py", ".sh", ".cpp", ".hpp", ".h"}
                               or path == CORRECTION_ROOT / "extnav_patch/scripts/extnav_to_vision_pose")
        and "test" not in path.relative_to(CORRECTION_ROOT).parts
    ]
    combined = "\n".join(_text(path) for path in source_files)
    for token in forbidden:
        assert token not in combined


@pytest.mark.parametrize("native_state", ["available", "missing_library", "absent"])
def test_extnav_production_launcher_selects_native_or_identity(tmp_path, native_state):
    """真实启动器在隔离目录检查库并透传参数，不依赖旧 Python fallback。"""
    launcher = tmp_path / "extnav_to_vision_pose"
    shutil.copy2(CORRECTION_ROOT / "extnav_patch/scripts/extnav_to_vision_pose", launcher)
    output = tmp_path / "selected.txt"
    arguments = ["--ros-args", "-r", "__node:=isolated test", "-p", "value:=a b"]
    for name in ("native", "identity"):
        executable = tmp_path / f"extnav_to_vision_pose_{name}"
        executable.write_text(
            "#!/usr/bin/env bash\n"
            f"printf '%s\\n' '{name}' \"$@\" > \"${{LAUNCHER_OUTPUT}}\"\n"
        )
        executable.chmod(0o755)
    if native_state == "absent":
        (tmp_path / "extnav_to_vision_pose_native").unlink()
    ldd = tmp_path / "ldd"
    ldd.write_text(
        "#!/usr/bin/env bash\n"
        + ("echo 'libcorrection_interfaces.so => not found'\nexit 1\n"
           if native_state == "missing_library" else "echo 'all test dependencies resolved'\n")
    )
    ldd.chmod(0o755)
    environment = dict(os.environ, PATH=f"{tmp_path}:{os.defpath}", LAUNCHER_OUTPUT=str(output))
    result = subprocess.run([str(launcher), *arguments], env=environment,
                            capture_output=True, text=True, timeout=5)
    assert result.returncode == 0, result.stderr
    expected = "native" if native_state == "available" else "identity"
    assert output.read_text().splitlines() == [expected, *arguments]
