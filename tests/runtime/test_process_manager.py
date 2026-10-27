"""外部进程分组清理与安全匹配规则测试。"""

import subprocess
import sys
import time

from ground_station_core.process_manager import ProcessSupervisor


def test_named_termination_stops_only_selected_process_and_descendants() -> None:
    """重开 RViz 时只回收对应启动器及其子进程，另一会话继续运行。"""
    supervisor = ProcessSupervisor()
    parent_code = (
        "import subprocess,sys,time\n"
        "subprocess.Popen([sys.executable, '-c', 'import time; time.sleep(60)'])\n"
        "time.sleep(60)\n"
    )
    preview = supervisor.start(
        "pytest_preview_probe",
        [sys.executable, "-c", parent_code],
    )
    retained = supervisor.start(
        "pytest_retained_probe", [sys.executable, "-c", "import time; time.sleep(60)"],
    )
    try:
        deadline = time.monotonic() + 2.0
        pids = {preview.process.pid}
        while time.monotonic() < deadline and len(pids) < 2:
            pids = supervisor._with_descendants({preview.process.pid})
            time.sleep(.01)
        assert len(pids) >= 2
        supervisor.terminate("pytest_preview_probe")
        assert preview.process.poll() is not None
        assert not any(supervisor._pid_alive(pid) for pid in pids)
        assert retained.running
        assert supervisor.get("pytest_preview_probe") is None
        assert supervisor.get("pytest_retained_probe") is retained
    finally:
        supervisor.terminate("pytest_preview_probe")
        supervisor.terminate("pytest_retained_probe")


def test_managed_process_is_reaped_and_verified() -> None:
    """受管进程必须在 terminate_all 返回前被 wait 回收。"""
    supervisor = ProcessSupervisor()
    process = supervisor.start(
        "pytest_cleanup_probe",
        ["python3", "-c", "import time; time.sleep(60)"],
    )
    report = supervisor.terminate_all()

    assert process.process.poll() is not None
    assert report.success
    assert report.managed_stopped == 1
    assert report.remaining == ()


def test_process_matcher_uses_argv_tokens_not_shell_text() -> None:
    """诊断文本和其他项目的 ros2 命令不能被当成本项目残留。"""
    assert not ProcessSupervisor._is_related_argv(
        ["/bin/bash", "-c", "echo mavros_node rviz2 sim_vehicle.py"]
    )
    assert ProcessSupervisor._is_related_argv(
        ["python3", "/project/install/guided_sim/lib/guided_sim/avoidance_preview.py"]
    )
    assert ProcessSupervisor._is_related_argv(
        ["python3", "/project/install/guided_sim/lib/guided_sim/simulation_obstacles.py"]
    )
    assert not ProcessSupervisor._is_related_argv(
        ["/opt/ros/jazzy/bin/ros2", "launch", "mavros", "apm.launch"]
    )
    assert ProcessSupervisor._is_related_argv(
        [
            "/opt/ros/jazzy/bin/ros2",
            "run",
            "mavros",
            "mavros_node",
            "-p",
            "fcu_url:=tcp://127.0.0.1:5762",
        ]
    )
    assert ProcessSupervisor._is_related_argv(
        [
            "/home/nvidia/scq/projects/ardupilot/build/sitl/bin/arducopter",
            "--defaults",
            "/tmp/tmp-project-sitl",
            "--sim-address=127.0.0.1",
            "-I0",
        ]
    )
    assert not ProcessSupervisor._is_related_argv(
        ["/opt/ros/jazzy/bin/ros2", "launch", "nav2_bringup", "navigation_launch.py"]
    )
    assert not ProcessSupervisor._is_related_argv(
        ["/opt/ros/jazzy/lib/mavros/mavros_node", "--ros-args"]
    )
    assert ProcessSupervisor._is_related_argv(
        [
            "/opt/ros/jazzy/lib/mavros/mavros_node",
            "--ros-args",
            "-p",
            "fcu_url:=tcp://127.0.0.1:5762",
        ]
    )


def test_historical_unmanaged_ros_process_is_swept() -> None:
    """即使进程不是当前 GUI 启动，也应按真实 argv 被残留扫描清理。"""
    process = subprocess.Popen(
        ["/bin/bash", "-c", "exec -a onboard_control_node sleep 60"],
        start_new_session=True,
    )
    try:
        report = ProcessSupervisor().terminate_all()
        process.wait(timeout=3.0)
    finally:
        if process.poll() is None:
            process.kill()
            process.wait(timeout=3.0)

    assert process.returncode is not None
    assert process.pid in report.stale_stopped
    assert report.success


def test_parent_exit_cannot_leave_pipe_holding_detached_child_or_deadlock() -> None:
    """组长先退、后代另建 session 且持有 stdout 时，清理仍须有界完成。"""
    child_code = (
        "import signal,time; "
        "signal.signal(signal.SIGINT, signal.SIG_IGN); "
        "signal.signal(signal.SIGTERM, signal.SIG_IGN); "
        "time.sleep(60)"
    )
    parent_code = (
        "import signal,subprocess,sys,time; "
        f"subprocess.Popen([sys.executable, '-c', {child_code!r}], "
        "start_new_session=True); "
        "signal.signal(signal.SIGINT, lambda *_: sys.exit(0)); "
        "print('ready', flush=True); time.sleep(60)"
    )
    supervisor = ProcessSupervisor()
    process = supervisor.start(
        "pytest_detached_pipe_probe",
        [sys.executable, "-c", parent_code],
    )
    time.sleep(0.3)

    started = time.monotonic()
    report = supervisor.terminate_all()
    elapsed = time.monotonic() - started

    assert process.process.poll() is not None
    assert elapsed < 7.0
    assert report.success
    assert report.remaining == ()
