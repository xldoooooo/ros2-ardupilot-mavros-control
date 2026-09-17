"""飞行接口版本与机载包兼容性。"""

from __future__ import annotations

from xml.etree import ElementTree
from ground_station_core.config import INTERFACE_VERSION, PROJECT_ROOT


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

    assert INTERFACE_VERSION == "3.3"
    assert 'kInterfaceVersion[] = "3.3"' in onboard_source
    assert package_versions == {"3.3.0"}
