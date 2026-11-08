"""验证增量选择不会漏掉跨模块边界，也不会收集独立模块。"""

import ast
from pathlib import Path
import subprocess

import pytest

from tests import run


@pytest.mark.parametrize(
    ("path", "expected"),
    [
        ("video_service/camera_app/controller.py", {"video"}),
        ("correction_service/correction_service/window.py", {"correction"}),
        ("src/correction_native/src/odometry.cpp", {"correction"}),
        ("src/correction_interfaces/msg/CorrectionStatus.msg", {"correction", "flight", "waypoints"}),
        ("ground_station_core/upstream/protocol.py", {"upstream"}),
        ("ground_station_core/waypoint_io.py", {"waypoints", "upstream"}),
        ("ground_station_core/qt_ui/waypoint_panel.py", {"waypoint-gui"}),
        ("ground_station_core/qt_ui/main_window.py", {"ground-gui"}),
        ("ground_station_core/qt_ui/state.py", {"ground-gui"}),
        ("src/avoidance_bridge/src/avoidance_bridge.cpp", {"runtime", "flight", "waypoints"}),
        ("fpga_avoidance/workspace/src/fpga_uav_planner_ros2/setup.py", set()),
        ("src/onboard_control/src/fcu_reboot.cpp", {"flight"}),
        ("src/onboard_control/src/avoidance_execution.cpp", {"flight", "waypoints"}),
        ("src/onboard_control/include/onboard_control/avoidance_trajectory.hpp", {"flight", "waypoints"}),
        (
            "src/onboard_control/src/onboard_control_node.cpp",
            {"flight", "waypoints", "video"},
        ),
        ("ground_station_core/qt_ui/window_chrome.py", {"gui"}),
        ("ground_station_core/qt_ui/cached_shadow.py", {"gui"}),
        ("tests/support/qt.py", {"gui"}),
        ("tests/impact.py", set()),
        ("tests/build_native.sh", set()),
        ("ground_station_core/qt_ui/log_history.py", {"ui"}),
        ("ground_station_core/startup.py", {"runtime"}),
        ("ground_station_core/ground_media_reader.py", {"upstream"}),
        ("ground_station_core/upstream/status_projector.py", {"upstream", "video"}),
        ("ground_station_core/upstream/mapping.py", {"upstream", "flight"}),
        ("drivers/odin/src/odin_ros_driver/include/fcu_odometry.h", {"runtime", "flight", "correction"}),
        ("scripts/onboard/switch_navigation_mode.sh", {"runtime", "flight", "waypoints"}),
        ("src/onboard_control/src/waypoint_calibration.cpp", {"flight", "waypoints"}),
        ("src/onboard_control/src/control_diagnostics.cpp", {"flight"}),
        ("src/onboard_control/src/tracking_diagnostics.cpp", {"flight"}),
        ("scripts/diagnostics/analyze_hover.py", {"flight"}),
        ("src/onboard_control/src/navigation_frame.cpp", {"flight", "waypoints"}),
        ("tests/video/test_camera_service.py", {"tests/video/test_camera_service.py"}),
        ("tests/test_camera_service.py", {"video"}),
        ("tests/test_ros_controller.py", {"flight", "waypoints"}),
        ("README.md", set()),
        ("scripts/lib/runtime_common.bash", {"runtime", "flight", "video", "correction"}),
        ("scripts/onboard/components/start_link.sh", {"runtime", "flight", "video", "correction"}),
        ("scripts/ground/start_ground_all.sh", {"runtime"}),
        ("scripts/onboard/reboot_fcu.sh", {"flight"}),
        ("project_configuration.py", set(run.GROUPS)),
        ("config/aircraft/uav001/control.yaml", {"flight", "waypoints"}),
        ("config/aircraft/uav002/video/camera.conf", {"video"}),
        ("config/aircraft/uav002/correction/tag_pose.csv", {"correction", "flight", "waypoints"}),
        ("config/aircraft/uav001/aircraft.yaml", {"configuration"}),
        ("config/aircraft.local.env.example", {"configuration"}),
        ("system/systemd/ros2-ardupilot-onboard.service", {"runtime", "video", "correction"}),
    ],
)
def test_independent_and_shared_boundaries(path: str, expected: set[str]) -> None:
    """独立功能只运行自身闭环，共享入口必须覆盖调用方。"""
    assert run.select_groups([path]) == (expected, [])


def test_selection_unions_groups_and_reports_unmapped_code() -> None:
    """未知新增代码不能当作无关文件静默忽略。"""
    assert run.select_groups(
        [
            "video_service/camera_service.py",
            "correction_service/correction_service/node.py",
            "new_module.py",
        ]
    ) == ({"video", "correction"}, ["new_module.py"])


def test_tracked_product_sources_have_an_impact_rule() -> None:
    """用实际产品源码清单发现新增漏项，而非只复述已知规则。"""
    paths = subprocess.check_output(["git", "ls-files", "-z"], cwd=run.ROOT).decode().split("\0")
    product = [path for path in paths
               if (Path(path).suffix in {".py", ".sh", ".bash", ".cpp", ".hpp", ".h", ".msg", ".srv"}
                   or path.startswith("config/"))
               and not path.startswith(("agent/", "dev/", ".agents/"))]
    assert run.select_groups(product)[1] == []


def test_partial_cpp_registration_cannot_pass_as_complete(monkeypatch) -> None:
    """旧构建只注册部分所选套件时，禁止直接执行并报告成功。"""
    calls = []

    def inventory(command, **kwargs):
        """模拟仅有 DOB 的旧 CTest 构建清单。"""
        calls.append(command)
        return subprocess.CompletedProcess(command, 0, '{"tests":[{"name":"test_dob_controller"}]}', "")

    monkeypatch.setattr(run.subprocess, "run", inventory)
    assert run.execute(["ctest", "--test-dir", "build/onboard_control", "-R",
                        "^(test_dob_controller|test_navigation_frame)$"]) == 1
    assert len(calls) == 1


def test_plan_collects_only_selected_directories_and_cpp_suites() -> None:
    """选择发生在 pytest 收集前；包级/C++ 回归也跟随所属功能。"""
    video = run.commands({"video"}, ["-x"])
    assert len(video) == 1
    assert video[0][4:] == ["tests/video", "-x"]
    correction = run.commands({"correction"}, [])
    assert correction[0][4:] == ["tests/correction", "correction_service/test"]
    waypoint = run.commands({"waypoints"}, [])
    assert "test_navigation_frame" in waypoint[1][-1]
    assert len(waypoint) == 5
    assert "src/avoidance_bridge/test/test_backend_selection_ros.py" in waypoint[-1]
    assert "AVOIDANCE_COORD_MODE=identity" in waypoint[2]
    assert "AVOIDANCE_COORD_MODE=extnav" in waypoint[3]
    unit = run.commands({"waypoints"}, [], "unit")
    assert len(unit) == 2
    assert unit[0][-2:] == ["-m", "not integration"]
    integration = run.commands({"flight"}, [], "integration")
    assert len(integration) == 1
    assert integration[0][-2:] == ["-m", "integration"]
    assert "test_reference_generator" in waypoint[1][-1]
    assert "test_waypoint_arrival_tracker" in waypoint[1][-1]
    assert "test_avoidance_trajectory" in waypoint[1][-1]
    assert "test_dob_controller" not in waypoint[1][-1]


def test_gui_changes_cover_callers_without_native_or_media_pipeline_regression() -> None:
    """共享窗口变更覆盖起飞/恢复/倒计时及独立窗口，但不附带算法和推流套件。"""
    groups, unknown = run.select_groups(["ground_station_core/qt_ui/cached_shadow.py"])
    assert not unknown
    plan = run.commands(groups, [])
    assert len(plan) == 1
    paths = plan[0][4:]
    assert "tests/runtime/test_ground_startup.py" in paths
    assert "tests/flight/test_takeoff_voltage.py" in paths
    assert "tests/upstream/test_upstream_countdown.py" in paths
    assert "tests/correction/test_correction_panel.py" in paths
    assert any(path.startswith("tests/video/test_camera_service.py::") for path in paths)
    assert "tests/flight" not in paths
    assert "tests/video/test_public_stream.py" not in paths
    mixed = run.commands(groups | {"video"}, [])[0][4:]
    assert "tests/video" in mixed
    assert not any(path.startswith("tests/video/") for path in mixed)
    assert run.commands(set(run.GROUPS) | set(run.CHECKS), []) == run.commands(set(run.GROUPS), [])
    changes = ["ground_station_core/qt_ui/waypoint_panel.py",
               "tests/waypoints/test_waypoints_gui.py"]
    selected, unknown = run.select_groups(changes)
    assert not unknown
    assert len(run.commands(selected, [])) == 1
    assert "tests/waypoints" not in run.commands(selected, [])[0]


def test_changed_test_file_stays_targeted_but_shared_fixture_selects_consumers() -> None:
    """修改用例只执行自身，组内共享夹具和已删除用例仍验证所属闭环。"""
    target = "tests/waypoints/test_waypoints_gui.py"
    selected, unknown = run.select_groups([target])
    assert not unknown and run.commands(selected, [])[0][4:] == [target]
    assert run.select_groups(["tests/video/conftest.py"]) == ({"video"}, [])
    assert run.select_groups(["tests/waypoints/test_deleted_case.py"]) == ({"waypoints"}, [])


def test_specialized_checks_reference_existing_files_and_cases() -> None:
    """实际解析专项目标，防止文件迁移或函数更名后专项失效。"""
    valid = set(run.GROUPS) | set(run.CHECKS)
    assert all(set(targets) <= valid for targets in run.RULES.values())
    for targets in run.CHECKS.values():
        for target in targets:
            path, *names = target.split("::")
            file = run.ROOT / path
            assert file.exists(), target
            if names:
                functions = {node.name for node in ast.parse(file.read_text()).body
                             if isinstance(node, (ast.FunctionDef, ast.ClassDef))}
                assert names[0] in functions, target


@pytest.mark.parametrize("changed, expected", [
    ("tests/impact.py", "tests/test_selection.py"),
    ("tests/build_native.sh", "tests/test_build_native.py"),
])
def test_metadata_and_build_entry_changes_run_their_own_checks(monkeypatch, capsys, changed, expected):
    """维护映射/构建入口不触发产品全量，预览仍说明命中原因。"""
    calls = []
    monkeypatch.setattr(run, "execute", lambda command: calls.append(command) or 0)
    monkeypatch.setattr(run.sys, "argv", ["run.py", "--files", changed,
                                        "--", "-k", "product_case"])
    assert run.main() == 0
    assert len(calls) == 1 and calls[0][4:] == [expected]
    assert f"{changed} -> {changed}" in capsys.readouterr().out


def test_explicit_python_target_uses_runner_environment_without_implicit_suites(monkeypatch):
    """定向测试保留 pytest 选项，不附带整个业务目录或 C++。"""
    target = "tests/waypoints/test_waypoint_panel.py"
    calls = []
    monkeypatch.setattr(run, "execute", lambda command: calls.append(command) or 0)
    monkeypatch.setattr(run.sys, "argv", ["run.py", "--tests", target, "--", "-x"])
    assert run.main() == 0
    assert len(calls) == 1 and calls[0][4:] == [target, "-x"]


def test_git_selection_includes_commits_index_worktree_and_renames(
    tmp_path: Path, monkeypatch
) -> None:
    """真实临时 Git 仓库覆盖暂存、未暂存、删除、重命名和未跟踪。"""

    def git(*args: str) -> str:
        return subprocess.check_output(["git", *args], cwd=tmp_path, text=True).strip()

    git("init", "-q")
    git("config", "user.name", "Test")
    git("config", "user.email", "test@example.invalid")
    for name in (
        "staged.py",
        "worktree.py",
        "deleted.py",
        "old name.py",
        "committed.py",
    ):
        (tmp_path / name).write_text("original\n")
    git("add", ".")
    git("commit", "-qm", "base")
    base = git("rev-parse", "HEAD")
    (tmp_path / "committed.py").write_text("committed\n")
    git("commit", "-qam", "change")
    (tmp_path / "staged.py").write_text("staged\n")
    git("add", "staged.py")
    (tmp_path / "worktree.py").write_text("worktree\n")
    (tmp_path / "deleted.py").unlink()
    git("mv", "old name.py", "new name.py")
    (tmp_path / "untracked.py").write_text("untracked\n")
    monkeypatch.setattr(run, "ROOT", tmp_path)
    expected = {
        "staged.py",
        "worktree.py",
        "deleted.py",
        "old name.py",
        "new name.py",
        "untracked.py",
    }
    assert set(run.changed_paths("HEAD")) == expected
    assert set(run.changed_paths(base)) == expected | {"committed.py"}


def test_runner_preserves_failure_and_list_does_not_execute(monkeypatch) -> None:
    """不能把子进程失败报告为通过，预览必须没有执行副作用。"""
    calls = []

    def fail(command, **kwargs):
        calls.append(command)
        return subprocess.CompletedProcess(command, 1)

    monkeypatch.setattr(run.subprocess, "run", fail)
    monkeypatch.setattr(run.sys, "argv", ["run.py", "--group", "video", "--list"])
    assert run.main() == 0
    assert not calls
    monkeypatch.setattr(run.sys, "argv", ["run.py", "--group", "video"])
    assert run.main() == 1
    assert len(calls) == 1
