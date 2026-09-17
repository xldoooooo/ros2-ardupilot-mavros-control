"""按功能闭环执行回归；默认只选择 Git 工作区改动涉及的组。"""

from __future__ import annotations

import argparse
from fnmatch import fnmatchcase
from pathlib import Path
import shlex
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
GROUPS = ("runtime", "flight", "waypoints", "upstream", "video", "correction", "ui")
GROUND = ("runtime", "flight", "waypoints", "upstream", "ui")
# 同一个文件可命中多条规则，取并集；跨模块调用点必须包含消费方闭环。
RULES = {
    "ground_station.py": GROUND,
    "ground_station_core/bootstrap.py": ("runtime",),
    "ground_station_core/environment.py": (
        "runtime",
        "flight",
        "waypoints",
        "upstream",
    ),
    "ground_station_core/process_manager.py": ("runtime",),
    "ground_station_core/event_log.py": GROUND,
    "ground_station_core/config.py": GROUPS,
    "ground_station_core/models.py": GROUND,
    "ground_station_core/__init__.py": GROUND,
    "ground_station_core/ros_controller.py": (
        "flight",
        "waypoints",
        "runtime",
        "upstream",
    ),
    "ground_station_core/waypoint_io.py": ("waypoints",),
    "ground_station_core/upstream/*": ("upstream",),
    "ground_station_core/qt_ui/main_window.py": GROUPS,
    "ground_station_core/qt_ui/operations_panel.py": (
        "flight",
        "runtime",
        "waypoints",
        "upstream",
        "ui",
    ),
    "ground_station_core/qt_ui/state.py": (
        "flight",
        "runtime",
        "waypoints",
        "upstream",
    ),
    "ground_station_core/qt_ui/waypoint_panel.py": ("waypoints",),
    "ground_station_core/qt_ui/upstream_panel.py": ("upstream",),
    "ground_station_core/qt_ui/log_panel.py": ("ui",),
    "ground_station_core/qt_ui/theme.py": GROUND,
    "ground_station_core/qt_ui/widgets.py": GROUND,
    "ground_station_core/qt_ui/window_chrome.py": GROUPS,
    "ground_station_core/qt_ui/__init__.py": GROUPS,
    "video_service/*": ("video",),
    "*onboard_video.sh": ("video",),
    "correction_service/*": ("correction",),
    "src/correction_interfaces/*": ("correction",),
    "*onboard_correction.sh": ("correction",),
    "src/guided_sim/*": ("waypoints", "runtime"),
    "src/onboard_control/src/main.cpp": ("flight", "waypoints", "video"),
    "src/onboard_control/src/onboard_control_node.cpp": (
        "flight",
        "waypoints",
        "video",
    ),
    "src/onboard_control/include/onboard_control/onboard_control_node.hpp": (
        "flight",
        "waypoints",
        "video",
    ),
    "src/onboard_control/src/fcu_reboot.cpp": ("flight",),
    "src/onboard_control/src/dob_controller.cpp": ("flight", "waypoints"),
    "src/onboard_control/include/onboard_control/dob_controller.hpp": ("flight", "waypoints"),
    "src/onboard_control/src/reference_generator.cpp": ("waypoints",),
    "src/onboard_control/include/onboard_control/reference_generator.hpp": (
        "waypoints",
    ),
    "src/onboard_control/include/onboard_control/waypoint_arrival_tracker.hpp": (
        "waypoints",
    ),
    "src/onboard_control/config/*": ("flight", "waypoints"),
    "src/onboard_control/launch/*": ("runtime", "flight"),
    "src/onboard_control/deploy/*": ("runtime", "correction"),
    "src/onboard_control/scripts/*": ("flight",),
    "src/onboard_control/test/test_dob_controller.cpp": ("flight",),
    "src/onboard_control/test/test_reference_generator.cpp": ("waypoints",),
    "src/onboard_control/test/test_waypoint_arrival_tracker.cpp": ("waypoints",),
    "src/onboard_control/CMakeLists.txt": ("runtime", "flight", "waypoints", "video"),
    "src/onboard_control/package.xml": ("runtime", "flight", "video"),
    "src/guided_interfaces/*": ("flight", "waypoints", "upstream", "video", "runtime"),
    "start_drone/*": ("runtime", "flight", "video", "correction"),
    "*onboard_control.sh": ("runtime", "flight", "video", "correction"),
    "reboot_fcu.sh": ("flight",),
    "start_ground_all.sh": ("runtime",),
    "setup_ground_station.sh": ("runtime", "video", "correction"),
    "tests/support/*": GROUPS,
    "tests/conftest.py": GROUPS,
    "tests/__init__.py": GROUPS,
    "tests/run.py": GROUPS,
    "tests/run.sh": GROUPS,
    "tests/test_selection.py": (),
    "tests/test_qt_gui.py": GROUPS,
    "tests/test_bootstrap.py": ("runtime", "flight", "video"),
    "tests/test_onboard_deploy.py": ("runtime", "video"),
    "tests/test_ros_controller.py": ("flight", "waypoints"),
}
# 允许 --changed 的基线早于目录重构：旧路径删除仍归入迁移后的闭环。
for _group in GROUPS:
    for _test in (ROOT / "tests" / _group).glob("test_*.py"):
        RULES.setdefault(f"tests/{_test.name}", (_group,))
# 纯文档/实验记录不触发产品回归；未知代码报错，避免漏测或擅自全量。
IGNORED = ("*.md", "docs/*", "agent/*", ".gitignore")
CPP_TESTS = {
    "flight": ("test_dob_controller",),
    "waypoints": ("test_reference_generator", "test_waypoint_arrival_tracker"),
}


def select_groups(paths: list[str]) -> tuple[set[str], list[str]]:
    """合并改动的闭环，并返回没有归属规则的路径。"""
    selected: set[str] = set()
    unknown = []
    for path in paths:
        if any(fnmatchcase(path, pattern) for pattern in IGNORED):
            continue
        parts = Path(path).parts
        if len(parts) > 2 and parts[0] == "tests" and parts[1] in GROUPS:
            selected.add(parts[1])
            continue
        matches = [
            groups for pattern, groups in RULES.items() if fnmatchcase(path, pattern)
        ]
        if not matches:
            unknown.append(path)
        for groups in matches:
            selected.update(groups)
    return selected, unknown


def changed_paths(base: str) -> list[str]:
    """相对基线包含已提交差异、暂存、未暂存及未跟踪；重命名保留新旧路径。"""
    paths = set()
    for args in (
        ["diff", "--name-only", "--no-renames", "-z", base, "--"],
        ["ls-files", "--others", "--exclude-standard", "-z"],
    ):
        result = subprocess.run(
            ["git", *args], cwd=ROOT, check=True, capture_output=True
        )
        paths.update(value.decode() for value in result.stdout.split(b"\0") if value)
    return sorted(paths)


def commands(groups: set[str], pytest_args: list[str]) -> list[list[str]]:
    """每次 pytest 只收集所选目录；C++ 测试沿用现有 CTest 注册。"""
    paths = [f"tests/{group}" for group in GROUPS if group in groups]
    if "correction" in groups:
        paths.append("correction_service/test")
    result = (
        [[sys.executable, "-m", "pytest", "-q", *paths, *pytest_args]] if paths else []
    )
    cpp = [
        name for group, names in CPP_TESTS.items() if group in groups for name in names
    ]
    if cpp:
        result.append(
            [
                "ctest",
                "--test-dir",
                "build/onboard_control",
                "--output-on-failure",
                "--no-tests=error",
                "-R",
                "^(" + "|".join(cpp) + ")$",
            ]
        )
    return result


def main() -> int:
    """展示选择后执行；失败继续汇总，但保留非零返回码。"""
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--group", nargs="+", choices=GROUPS)
    mode.add_argument("--all", action="store_true")
    mode.add_argument("--changed", nargs="?", const="HEAD", metavar="BASE")
    mode.add_argument("--files", nargs="+", metavar="PATH")
    parser.add_argument(
        "--list", action="store_true", help="仅显示计划，不收集或运行测试"
    )
    args, pytest_args = parser.parse_known_args()
    if pytest_args[:1] == ["--"]:
        pytest_args = pytest_args[1:]
    changed = []
    if args.all or args.group:
        groups = set(GROUPS if args.all else args.group)
    else:
        changed = (
            args.files
            if args.files is not None
            else changed_paths(args.changed or "HEAD")
        )
        groups, unknown = select_groups(changed)
        if unknown:
            parser.error(
                "以下路径尚未分组，请补充 RULES 或显式指定 --group："
                + ", ".join(unknown)
            )
    print(
        "测试组：" + (", ".join(group for group in GROUPS if group in groups) or "无"),
        flush=True,
    )
    plan = commands(groups, pytest_args)
    if args.all or "tests/run.py" in changed or "tests/test_selection.py" in changed:
        plan.insert(
            0, [sys.executable, "-m", "pytest", "-q", "tests/test_selection.py"]
        )
    result = 0
    for command in plan:
        print("+ " + shlex.join(command), flush=True)
        if not args.list:
            result = max(result, subprocess.run(command, cwd=ROOT).returncode != 0)
    if not plan:
        print("没有涉及现有自动化测试的改动；未执行测试。")
    return int(result)


if __name__ == "__main__":
    raise SystemExit(main())
