"""任务 27 的 sparse、extnav 备份和独立 systemd 部署边界回归测试。"""

from __future__ import annotations

from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parent.parent
CORRECTION_ROOT = PROJECT_ROOT / "correction_service"


def _text(path: Path) -> str:
    """读取仓库 UTF-8 文本。"""
    return path.read_text(encoding="utf-8")


def test_extnav_installer_refuses_running_chain_and_backs_up_before_install() -> None:
    """生产 extnav 覆盖前必须先停服务/进程并生成可校验定点备份。"""
    script = _text(CORRECTION_ROOT / "deploy" / "install_extnav_correction.sh")

    assert 'systemctl is-active --quiet "${FLIGHT_SERVICE}"' in script
    assert "pgrep -af '[e]xtnav_to_vision_pose'" in script
    assert script.index("sha256sum -c SHA256SUMS") < script.index(
        'install -m 0644 "${PATCH_SOURCE}"'
    )
    assert "colcon build --packages-select extnav_bridge --symlink-install" in script
    assert "systemctl restart" not in script
    assert "systemctl start" not in script
    assert "/mavros/cmd/arming" not in script


def test_correction_unit_has_no_flight_service_dependency_and_starts_idle_node() -> (
    None
):
    """修正服务必须是独立故障域，并只启动默认 idle 的 correction_node。"""
    unit = _text(CORRECTION_ROOT / "deploy" / "correction-service.service.example")
    installer = _text(CORRECTION_ROOT / "deploy" / "install_correction_service.sh")

    assert "ExecStart=" in unit and "correction_node" in unit
    assert "CORRECTION_CAMERA_OVERLAY_SETUP" in unit
    assert "SupplementaryGroups=video" in unit
    assert "KillMode=control-group" in unit
    assert "ros2-ardupilot-onboard.service" not in unit
    active_lines = tuple(
        line.strip() for line in unit.splitlines() if not line.lstrip().startswith("#")
    )
    for directive in ("Requires=", "PartOf=", "BindsTo="):
        assert not any(line.startswith(directive) for line in active_lines)
    assert 'systemctl is-active --quiet "${FLIGHT_SERVICE}"' in installer
    assert "correction_interfaces correction_service" in installer
    assert 'systemctl enable --now "${SERVICE_NAME}"' in installer
    assert "/mavros/cmd/arming" not in installer
    node = _text(CORRECTION_ROOT / "correction_service" / "node.py")
    constructor = node[node.index("def __init__") : node.index("def _on_odometry")]
    assert "create_subscription(\n            Odometry" not in constructor
    assert "def _start_odometry_capture" in node
    assert "self.destroy_subscription(subscription)" in node


def test_onboard_sparse_checkout_and_build_include_correction_packages() -> None:
    """正常部署构建修正包，但飞行启动不能把可选接口变成单点故障。"""
    helper = _text(
        PROJECT_ROOT / "src" / "onboard_control" / "deploy" / "onboard_workspace.sh"
    )
    launcher = _text(PROJECT_ROOT / "start_onboard_control.sh")

    assert 'CORRECTION_INTERFACES_SPARSE_PATH="/src/correction_interfaces/"' in helper
    assert 'CORRECTION_SERVICE_SPARSE_PATH="/correction_service/"' in helper
    assert (
        "guided_interfaces correction_interfaces onboard_control correction_service"
        in helper
    )
    assert "for onboard_package in guided_interfaces onboard_control" in launcher
    assert (
        "for onboard_package in guided_interfaces correction_interfaces" not in launcher
    )
    extnav = _text(CORRECTION_ROOT / "extnav_patch" / "extnav_to_vision_pose.py")
    assert "except ImportError:" in extnav
    assert "保持 identity passthrough" in extnav


def test_correction_sources_contain_no_arm_takeoff_or_mode_command_endpoint() -> None:
    """新模块只能校准坐标，不得暗含任何飞行控制写入口。"""
    forbidden = (
        "/mavros/cmd/arming",
        "/mavros/cmd/takeoff",
        "/mavros/set_mode",
        "FlightCommand",
        "MotionIntent",
    )
    source_files = tuple(CORRECTION_ROOT.rglob("*.py")) + tuple(
        CORRECTION_ROOT.rglob("*.sh")
    )
    combined = "\n".join(_text(path) for path in source_files)
    for token in forbidden:
        assert token not in combined
