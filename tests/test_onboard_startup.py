"""真实 ROS 隔离验证提前拉参数、参数值门和持续 READY 订阅；不连接实机。"""
from __future__ import annotations

import os
from pathlib import Path
import re
import signal
import subprocess
import time

import pytest

ROOT = Path(__file__).resolve().parents[1]


def test_startup_pull_and_readiness_require_actual_valid_parameters(tmp_path):
    """拉表 ACK 不代表就绪；错误/缺失参数继续拒绝，重连重新请求，退出清理。"""
    rclpy = pytest.importorskip("rclpy")
    from geometry_msgs.msg import PoseStamped, TwistStamped
    from guided_interfaces.msg import ControlStatus
    from mavros_msgs.msg import State
    from mavros_msgs.srv import ParamPull
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.parameter import Parameter
    from rclpy.qos import qos_profile_sensor_data

    executable = ROOT / "install/onboard_control/lib/onboard_control/onboard_control_node"
    assert executable.is_file(), "先构建 onboard_control"
    context = Context()
    domain = 210 + os.getpid() % 10
    rclpy.init(context=context, domain_id=domain)
    node = rclpy.create_node("param", namespace="/startup_mavros", context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    env = dict(os.environ, ROS_DOMAIN_ID=str(domain))
    pulls, statuses = [], []
    connected = True

    def pull(request, response):
        pulls.append(request.force_pull)
        response.success = True  # 故意在必要参数不存在时就 ACK。
        response.param_received = 0
        return response

    node.create_service(ParamPull, "/startup_mavros/param/pull", pull)
    state_pub = node.create_publisher(State, "/startup_mavros/state", 10)
    pose_pub = node.create_publisher(PoseStamped, "/startup_mavros/local_position/pose", 10)
    velocity_pub = node.create_publisher(
        TwistStamped, "/startup_mavros/local_position/velocity_local", 10
    )
    node.create_subscription(ControlStatus, "/startup_onboard/status", statuses.append,
                             qos_profile_sensor_data)

    def publish():
        state_pub.publish(State(connected=connected, armed=False, mode="STABILIZE"))
        pose = PoseStamped()
        pose.pose.orientation.w = 1.0
        pose_pub.publish(pose)
        velocity_pub.publish(TwistStamped())

    node.create_timer(0.05, publish)

    def until(predicate, seconds=6):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            executor.spin_once(timeout_sec=0.02)
            if predicate():
                return True
        return False

    log = (tmp_path / "node.log").open("w")
    process = subprocess.Popen([
        str(executable), "--ros-args", "-p", "mavros_prefix:=/startup_mavros",
        "-p", "interface_prefix:=/startup_onboard",
    ], env=env, stdout=log, stderr=subprocess.STDOUT)
    probe = None
    try:
        assert until(lambda: pulls), "没有在发现飞控后提前请求参数"
        assert pulls == [False], "不得强制重拉/清空 MAVROS 参数缓存"
        assert until(lambda: statuses and statuses[-1].local_position_valid)
        assert not statuses[-1].thrust_mode_verified
        node.declare_parameter("GUID_OPTIONS", 0)
        node.declare_parameter("MOT_THST_HOVER", 0.22)
        assert not until(lambda: statuses[-1].thrust_mode_verified, 3)
        node.set_parameters([Parameter("GUID_OPTIONS", value=8)])
        assert until(lambda: statuses[-1].thrust_mode_verified, 3)
        assert pulls == [False], "正常运行不得反复发起整表同步"
        connected = False
        assert until(lambda: not statuses[-1].fcu_connected)
        assert not statuses[-1].thrust_mode_verified
        connected = True
        assert until(lambda: len(pulls) == 2)
        assert pulls == [False, False]

        # 用启动脚本的真实过滤条件验证：字段不可跨消息拼凑，armed=true 不通过。
        expression = re.search(r"readonly readiness_filter='([^']+)'",
                               (ROOT / "start_onboard_control.sh").read_text()).group(1)
        pub = node.create_publisher(ControlStatus, "/startup_probe/status", 10)
        probe = subprocess.Popen([
            "timeout", "10", "ros2", "topic", "echo", "--no-daemon",
            "--qos-profile", "sensor_data", "--once", "--filter", expression,
            "/startup_probe/status", "guided_interfaces/msg/ControlStatus",
        ], env=env, stdout=subprocess.DEVNULL, stderr=log)
        sample = ControlStatus(armed=True, fcu_connected=True, local_position_valid=True,
                               message_rates_configured=True, thrust_mode_verified=True)
        assert until(lambda: pub.get_subscription_count() > 0)
        timer = node.create_timer(.05, lambda: pub.publish(sample))
        assert not until(lambda: probe.poll() is not None, .4)
        sample.armed = False
        sample.thrust_mode_verified = False
        assert not until(lambda: probe.poll() is not None, .4)
        sample.thrust_mode_verified = True
        assert until(lambda: probe.poll() is not None)
        assert probe.returncode == 0
        node.destroy_timer(timer)
        assert all(not s.armed and not s.lease_active for s in statuses)
    finally:
        for child in (probe, process):
            if child is not None and child.poll() is None:
                child.send_signal(signal.SIGINT)
                try:
                    child.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    child.kill()
                    child.wait()
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown(context=context)
        log.close()


def test_readiness_timeout_survives_wall_clock_step(tmp_path):
    """仅给子进程注入墙钟前跳；复现旧 SECONDS 提前退出，验证新相对定时器。"""
    import shutil
    if not shutil.which("cc"):
        pytest.skip("clock injection needs the project's C compiler")
    source = tmp_path / "clock_step.c"
    source.write_text(r'''
/* Simulate one NTP step inside the test process; never change the host clock. */
#define _GNU_SOURCE
#include <dlfcn.h>
#include <time.h>
#include <sys/time.h>
static long offset(void) {
  static double first;
  struct timespec now;
  clock_gettime(CLOCK_MONOTONIC, &now);
  double seconds = now.tv_sec + now.tv_nsec / 1e9;
  if (!first) first = seconds;
  return seconds - first > .15 ? 172800 : 0;
}
int gettimeofday(struct timeval *tv, void *tz) {
  int (*original)(struct timeval *, void *) = dlsym(RTLD_NEXT, "gettimeofday");
  int result = original(tv, tz);
  tv->tv_sec += offset();
  return result;
}
time_t time(time_t *out) {
  time_t (*original)(time_t *) = dlsym(RTLD_NEXT, "time");
  time_t value = original(0) + offset();
  if (out) *out = value;
  return value;
}
''')
    library = tmp_path / "clock_step.so"
    subprocess.run(["cc", "-shared", "-fPIC", str(source), "-o", str(library), "-ldl"],
                   check=True)
    env = dict(os.environ, LD_PRELOAD=str(library))
    started = time.monotonic()
    subprocess.run(["bash", "--noprofile", "--norc", "-c",
                    'deadline=$((SECONDS+120)); while ((SECONDS<deadline)); do sleep .05; done'],
                   env=env, check=True, timeout=3)
    assert time.monotonic() - started < 2, "未复现旧 Bash 墙钟问题"
    script = (ROOT / "start_onboard_control.sh").read_text()
    command = re.search(r"^setsid timeout (.+?) 120 ros2", script, re.M).group(1).split()
    started = time.monotonic()
    # Force repeated wall-clock reads while the relative timer is running.
    result = subprocess.run(["timeout", *command, "0.8", "bash", "--noprofile", "--norc",
                             "-c", 'while :; do echo "$SECONDS" >/dev/null; sleep .05; done'],
                            env=env, timeout=4)
    elapsed = time.monotonic() - started
    assert result.returncode == 124
    assert .65 <= elapsed < 3, elapsed


def test_cancel_launcher_stops_readiness_subscriber_and_components(tmp_path):
    """READY 尚未满足时取消，四组件和新增的持续订阅进程都必须退出。"""
    import shutil
    workspace = tmp_path / "workspace"
    (workspace / "start_drone").mkdir(parents=True)
    (workspace / "install").mkdir()
    (workspace / "install/setup.bash").touch()
    helper = workspace / "start_drone/runtime_common.bash"
    helper.write_text('''# Isolated discovery stubs: no hardware or ROS runtime.
runtime_source_setup() { :; }
runtime_detect_ros_setup() { echo "$ONBOARD_WORKSPACE/install/setup.bash"; }
runtime_detect_fcu_device() { echo /dev/null; }
runtime_verify_workspace_package_install() { :; }
runtime_ensure_package() { :; }
''')
    launcher = workspace / "start_onboard_control.sh"
    shutil.copy2(ROOT / launcher.name, launcher)
    bindir = tmp_path / "bin"
    bindir.mkdir()
    (bindir / "pgrep").write_text('#!/bin/bash\nexit 1\n')
    (bindir / "ros2").write_text('''#!/bin/bash
# Log every fake component/subscriber PID and stay alive until signalled.
if [[ "$1" == pkg ]]; then echo /fake/overlay; exit 0; fi
echo "$$ $*" >> "$TEST_PIDS"
trap 'exit 0' TERM INT
while :; do sleep .05; done
''')
    for path in bindir.iterdir():
        path.chmod(0o755)
    pids = tmp_path / "pids"
    env = dict(os.environ, PATH=f"{bindir}:{os.environ['PATH']}",
               ONBOARD_WORKSPACE=str(workspace), ONBOARD_ENV_FILE=str(tmp_path / "absent"),
               ONBOARD_LOG_ROOT=str(tmp_path / "logs"), TEST_PIDS=str(pids))
    with (tmp_path / "launcher.log").open("w") as log:
        process = subprocess.Popen(["bash", str(launcher)], env=env,
                                   stdout=log, stderr=subprocess.STDOUT)
        try:
            deadline = time.monotonic() + 8
            while time.monotonic() < deadline:
                if pids.exists() and len(pids.read_text().splitlines()) == 5:
                    break
                assert process.poll() is None, "启动器意外退出"
                time.sleep(.05)
            records = pids.read_text().splitlines()
            assert len(records) == 5
            process.terminate()
            assert process.wait(timeout=10) == 130
            for line in records:
                pid = int(line.split()[0])
                with pytest.raises(ProcessLookupError):
                    os.kill(pid, 0)
        finally:
            if process.poll() is None:
                process.terminate()
                process.wait(timeout=10)
