"""外部进程分组清理与安全匹配规则测试。"""

import subprocess

from ground_station_core.process_manager import ProcessSupervisor


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
