"""按功能闭环执行回归；默认只选择 Git 工作区改动涉及的组。"""

from __future__ import annotations

import argparse
from fnmatch import fnmatchcase
import json
from pathlib import Path
import shlex
import subprocess
import sys

# 直接脚本执行与模块导入使用同一个仓库内的测试规则。
ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from tests.impact import CHECKS, CPP_TESTS, GROUPS, IGNORED, RULES, SELF_TESTS


def matching_rules(path: str) -> list[tuple[str, tuple[str, ...]]]:
    """返回路径的实际命中规则，供选择与预览共用。"""
    if any(fnmatchcase(path, pattern) for pattern in IGNORED):
        return [("文档/记录", ())]
    parts = Path(path).parts
    if len(parts) > 2 and parts[0] == "tests" and parts[1] in GROUPS:
        if Path(path).name.startswith("test_") and path.endswith(".py") and (ROOT / path).is_file():
            return [("本次测试文件", (path,))]
        return [(f"tests/{parts[1]}/*", (parts[1],))]
    return [(pattern, groups) for pattern, groups in RULES.items()
            if fnmatchcase(path, pattern)]


def select_groups(paths: list[str]) -> tuple[set[str], list[str]]:
    """合并闭环、专项与本次测试文件，返回没有归属规则的路径。"""
    selected: set[str] = set()
    unknown = []
    for path in paths:
        matches = matching_rules(path)
        if not matches:
            unknown.append(path)
        for _pattern, groups in matches:
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


def python_command(paths: list[str], pytest_args: list[str], level: str) -> list[str]:
    """构造 Python 检查，并去除业务目录已覆盖的专项文件或用例。"""
    paths = list(dict.fromkeys(paths))
    paths = [path for path in paths if not any(
        path != other and (path.startswith(other + "/") or path.startswith(other + "::"))
        for other in paths
    )]
    options = list(pytest_args)
    if level != "all":
        options.extend(["-m", "not integration" if level == "unit" else "integration"])
    return [sys.executable, "-m", "pytest", "-q", *paths, *options]


def commands(groups: set[str], pytest_args: list[str], level: str = "all") -> list[list[str]]:
    """每次 pytest 只收集所选目录；C++ 测试沿用现有 CTest 注册。"""
    paths = [f"tests/{group}" for group in GROUPS if group in groups]
    if "correction" in groups:
        paths.append("correction_service/test")
    for check, targets in CHECKS.items():
        if check in groups:
            paths.extend(targets)
    paths.extend(sorted(target for target in groups if target.startswith("tests/")))
    result = [python_command(paths, pytest_args, level)] if paths else []
    cpp = [
        name for group, names in CPP_TESTS.items() if group in groups for name in names
    ]
    if cpp and level != "integration":
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
    if "waypoints" in groups and level != "unit":
        # 两种坐标模式必须分别建 ROS context；不导入可拆卸 FPGA 模块。
        for coordinate in ("identity", "extnav"):
            result.append([
                "env", f"AVOIDANCE_COORD_MODE={coordinate}",
                "ROS_DOMAIN_ID=230", sys.executable, "-m", "pytest", "-q",
                "src/avoidance_bridge/test/test_bridge_ros.py", *pytest_args,
            ])
        result.append([
            "env", "ROS_DOMAIN_ID=230", sys.executable, "-m", "pytest", "-q",
            "src/avoidance_bridge/test/test_backend_selection_ros.py",
            "src/avoidance_bridge/test/test_launch_parameters.py", *pytest_args,
        ])
    return result


def execute(command: list[str]) -> int:
    """执行测试前确认每个 CTest 套件已注册，防止旧构建只跑部分套件。"""
    if command[0] == "ctest":
        inventory = subprocess.run(
            [*command[:3], "--show-only=json-v1"], cwd=ROOT,
            text=True, capture_output=True,
        )
        if inventory.returncode:
            print(inventory.stderr or inventory.stdout, file=sys.stderr)
            return 1
        registered = {item["name"] for item in json.loads(inventory.stdout)["tests"]}
        expected = set(command[-1][2:-2].split("|"))
        missing = expected - registered
        if missing:
            print("缺少 CTest 套件：" + ", ".join(sorted(missing))
                  + "；先运行 bash tests/build_native.sh --packages onboard_control。", file=sys.stderr)
            return 1
    return int(subprocess.run(command, cwd=ROOT).returncode != 0)


def main() -> int:
    """展示选择后执行；失败继续汇总，但保留非零返回码。"""
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--group", nargs="+", choices=(*GROUPS, *CHECKS))
    mode.add_argument("--all", action="store_true")
    mode.add_argument("--changed", nargs="?", const="HEAD", metavar="BASE")
    mode.add_argument("--files", nargs="+", metavar="PATH")
    mode.add_argument("--tests", nargs="+", metavar="TEST",
                      help="定向 Python 文件或 node ID；不附带业务组/C++ 检查")
    parser.add_argument(
        "--list", action="store_true", help="仅显示计划，不收集或运行测试"
    )
    parser.add_argument("--level", choices=("all", "unit", "integration"), default="all",
                        help="功能组内的执行层级；默认完整回归")
    args, pytest_args = parser.parse_known_args()
    if pytest_args[:1] == ["--"]:
        pytest_args = pytest_args[1:]
    changed = []
    if args.tests:
        groups = set()
    elif args.all or args.group:
        groups = set(GROUPS if args.all else args.group)
    else:
        changed = (
            [Path(path).relative_to(ROOT).as_posix() if Path(path).is_absolute()
             else Path(path).as_posix() for path in args.files]
            if args.files is not None
            else changed_paths(args.changed or "HEAD")
        )
        groups, unknown = select_groups(changed)
        if unknown:
            parser.error(
                "以下路径尚未分组，请补充 tests/impact.py 的 RULES 或显式指定 --group："
                + ", ".join(unknown)
            )
    scopes = [group for group in (*GROUPS, *CHECKS) if group in groups]
    scopes.extend(sorted(target for target in groups if target.startswith("tests/")))
    label = "定向 Python" if args.tests else ", ".join(scopes) or "无"
    print("测试范围：" + label, flush=True)
    print(f"层级：{args.level}（业务范围与执行层级分别选择）", flush=True)
    if changed:
        print("选择来源：" + ("本次显式文件" if args.files else
              f"工作区差异，基线 {args.changed or 'HEAD'}"))
        for path in changed:
            for pattern, targets in matching_rules(path):
                print(f"  {path} -> {pattern} -> {', '.join(targets) or '不触发产品组'}")
    plan = ([python_command(args.tests, pytest_args, args.level)] if args.tests
            else commands(groups, pytest_args, args.level))
    self_checks = set(SELF_TESTS) if args.all else set()
    if any(path in changed for path in
           ("tests/run.py", "tests/run.sh", "tests/impact.py", "tests/test_selection.py")):
        self_checks.add("tests/test_selection.py")
    if any(path in changed for path in ("tests/build_native.sh", "tests/test_build_native.py")):
        self_checks.add("tests/test_build_native.py")
    if self_checks:
        # 产品的 -k/-m/路径过滤不能排除入口自身检查。
        plan.insert(0, python_command(sorted(self_checks), [], "all"))
    result = 0
    for command in plan:
        print("+ " + shlex.join(command), flush=True)
        if not args.list:
            result = max(result, execute(command))
    if not plan:
        print("没有涉及现有自动化测试的改动；未执行测试。")
    if any(path.startswith("fpga_avoidance/") for path in changed):
        print("FPGA 模块独立回归：bash fpga_avoidance/test.sh（仅本地软件，不连接板卡）。")
    return int(result)


if __name__ == "__main__":
    raise SystemExit(main())
