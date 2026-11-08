"""测试原生构建入口的选择和实际调用；编译器、ROS overlay 均以临时替身隔离。"""

import json
import os
from pathlib import Path
import shlex
import subprocess
import sys

import pytest

# 只复制测试构建入口，不调用产品部署或控制脚本。
BUILDER = Path(__file__).resolve().parent / "build_native.sh"


@pytest.fixture
def native_builder(tmp_path):
    """在临时工作区记录 colcon 调用，禁止加载主机 ROS 和安装目录。"""
    script = tmp_path / "tests/build_native.sh"
    script.parent.mkdir()
    script.write_text(BUILDER.read_text())
    binary = tmp_path / "bin/colcon"
    binary.parent.mkdir()
    binary.write_text(f"#!{sys.executable}\n" + '''import json, os, sys
with open(os.environ["BUILD_CALLS"], "a") as stream:
    stream.write(json.dumps(sys.argv[1:]) + "\\n")
''')
    binary.chmod(0o755)
    overlay = tmp_path / "planner/install/local_setup.bash"
    overlay.parent.mkdir(parents=True)
    overlay.touch()
    bash_env = tmp_path / "test-env.bash"
    bash_env.write_text('source() { :; }\n')
    calls = tmp_path / "calls.jsonl"
    environment = dict(os.environ, BASH_ENV=str(bash_env), BUILD_CALLS=str(calls),
                       AVOIDANCE_WORKSPACE=str(overlay.parents[1]),
                       PATH=str(binary.parent) + os.pathsep + os.environ["PATH"])

    def invoke(*args):
        """调用真实 Shell 参数分支并返回本次记录的编译请求。"""
        calls.unlink(missing_ok=True)
        result = subprocess.run(["bash", str(script), *args], env=environment,
                                capture_output=True, text=True)
        commands = [json.loads(line) for line in calls.read_text().splitlines()] if calls.exists() else []
        return result, commands

    return invoke, overlay


@pytest.mark.parametrize("packages", [("correction_native",), ("onboard_control",),
                                     ("extnav_bridge",), ()])
def test_native_build_plan_matches_execution_and_keeps_requested_scope(native_builder, packages):
    """定向构建不隐式加入其他产品包；全量入口保留，测试目标强制开启。"""
    invoke, _overlay = native_builder
    args = ("--packages", *packages) if packages else ()
    preview, commands = invoke(*args, "--list")
    assert preview.returncode == 0 and not commands
    planned = [shlex.split(line) for line in preview.stdout.splitlines()]
    result, commands = invoke(*args)
    assert result.returncode == 0, result.stderr
    assert commands == [command[1:] for command in planned]
    assert all("-DBUILD_TESTING=ON" in command for command in commands)
    if packages:
        start = commands[0].index("--packages-up-to") + 1
        stop = commands[0].index("--parallel-workers")
        assert commands[0][start:stop] == (["correction_interfaces"]
                                         if packages == ("extnav_bridge",) else list(packages))
    assert len(commands) == (2 if not packages or "extnav_bridge" in packages else 1)


def test_only_bridge_build_requires_the_independent_planner(native_builder):
    """缺规划器不阻断独立修正构建，桥构建则在编译前明确失败。"""
    invoke, overlay = native_builder
    overlay.unlink()
    result, commands = invoke("--packages", "correction_native")
    assert result.returncode == 0 and len(commands) == 1
    result, commands = invoke("--packages", "avoidance_bridge")
    assert result.returncode != 0 and not commands
    assert "Missing native planner overlay" in result.stderr


@pytest.mark.parametrize("args", [("--packages",), ("--packages", "misspelled"), ("--unknown",)])
def test_invalid_build_selection_does_not_start_compilation(native_builder, args):
    """空参数、未知包或未知选项不能退回完整构建。"""
    invoke, _overlay = native_builder
    result, commands = invoke(*args)
    assert result.returncode == 2 and not commands
