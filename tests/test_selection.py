"""验证增量选择不会漏掉跨模块边界，也不会收集独立模块。"""

from pathlib import Path
import subprocess

import pytest

from tests import run


@pytest.mark.parametrize(
    ("path", "expected"),
    [
        ("video_service/camera_app/controller.py", {"video"}),
        ("correction_service/correction_service/window.py", {"correction"}),
        ("src/correction_interfaces/msg/CorrectionStatus.msg", {"correction"}),
        ("ground_station_core/upstream/protocol.py", {"upstream"}),
        ("ground_station_core/waypoint_io.py", {"waypoints"}),
        ("ground_station_core/qt_ui/waypoint_panel.py", {"waypoints"}),
        ("src/onboard_control/src/fcu_reboot.cpp", {"flight"}),
        (
            "src/onboard_control/src/onboard_control_node.cpp",
            {"flight", "waypoints", "video"},
        ),
        ("ground_station_core/qt_ui/window_chrome.py", set(run.GROUPS)),
        ("tests/video/test_camera_service.py", {"video"}),
        ("tests/test_camera_service.py", {"video"}),
        ("tests/test_ros_controller.py", {"flight", "waypoints"}),
        ("README.md", set()),
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


def test_plan_collects_only_selected_directories_and_cpp_suites() -> None:
    """选择发生在 pytest 收集前；包级/C++ 回归也跟随所属功能。"""
    video = run.commands({"video"}, ["-x"])
    assert len(video) == 1
    assert video[0][4:] == ["tests/video", "-x"]
    correction = run.commands({"correction"}, [])
    assert correction[0][4:] == ["tests/correction", "correction_service/test"]
    waypoint = run.commands({"waypoints"}, [])
    assert "test_reference_generator" in waypoint[1][-1]
    assert "test_waypoint_arrival_tracker" in waypoint[1][-1]
    assert "test_dob_controller" not in waypoint[1][-1]


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
