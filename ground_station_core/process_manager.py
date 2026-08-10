"""外部 ROS/SITL 进程的分组启动、日志记录、分级终止与残留校验。"""

from __future__ import annotations

import os
import signal
import subprocess
import threading
import time
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Iterable, TextIO

from .config import PROJECT_ROOT
from .event_log import EventLog, LogLevel


LOG_DIRECTORY = Path("/tmp/ros2_ardupilot_ground_station")


@dataclass
class ManagedProcess:
    """由地面站启动的独立进程组及其日志句柄。"""

    name: str
    process: subprocess.Popen
    command: tuple[str, ...]
    log_path: Path
    log_stream: TextIO
    process_group_id: int
    output_thread: threading.Thread | None = None

    @property
    def running(self) -> bool:
        """返回进程组领头进程是否仍在运行。"""
        return self.process.poll() is None


@dataclass(frozen=True)
class CleanupReport:
    """一次清理操作的可审计结果。"""

    managed_stopped: int = 0
    stale_stopped: tuple[int, ...] = ()
    remaining: tuple[tuple[int, str], ...] = ()
    errors: tuple[str, ...] = ()

    @property
    def success(self) -> bool:
        """只有未发现残留且无终止错误时才算清理成功。"""
        return not self.remaining and not self.errors


def build_sourced_environment(
    setup_files: Iterable[Path],
    base_environment: dict[str, str] | None = None,
) -> dict[str, str]:
    """通过 bash source 多个 setup 文件，并返回可直接传给 Popen 的环境。"""
    setup_paths = tuple(Path(path).expanduser().resolve() for path in setup_files)
    missing = [str(path) for path in setup_paths if not path.is_file()]
    if missing:
        raise FileNotFoundError("缺少环境 setup 文件: " + ", ".join(missing))
    if not setup_paths:
        return dict(base_environment or os.environ)

    script = "set -e\nfor setup_file in \"$@\"; do source \"$setup_file\"; done\nenv -0"
    completed = subprocess.run(
        ["/bin/bash", "-c", script, "ground-station-env", *map(str, setup_paths)],
        env=dict(base_environment or os.environ),
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=False,
        timeout=15.0,
    )
    if completed.returncode != 0:
        detail = completed.stderr.decode("utf-8", errors="replace").strip()
        raise RuntimeError(f"加载 ROS 环境失败: {detail}")

    environment: dict[str, str] = {}
    for item in completed.stdout.split(b"\0"):
        if not item or b"=" not in item:
            continue
        key, value = item.split(b"=", 1)
        environment[key.decode(errors="surrogateescape")] = value.decode(
            errors="surrogateescape"
        )
    return environment


class ProcessSupervisor:
    """跟踪并清理本项目本地仿真进程，不扫描或终止任意 ROS 工作负载。"""

    # 只有本项目独有的节点名可以直接匹配；MAVROS/RViz/SITL 等通用进程
    # 必须再带本项目参数，避免在共享开发机上误杀其他 ROS 工作负载。
    _PROJECT_EXECUTABLES = {
        "keyboard_vel_controller",
        "onboard_control_node",
        "pose_to_tf.py",
    }

    def __init__(self, event_log: EventLog | None = None) -> None:
        """创建线程安全的进程表、共享日志目录和实时日志出口。"""
        self._lock = threading.RLock()
        self._managed: dict[str, ManagedProcess] = {}
        self._events = event_log or EventLog()
        LOG_DIRECTORY.mkdir(parents=True, exist_ok=True)

    @property
    def event_log(self) -> EventLog:
        """返回受管进程输出所使用的结构化日志总线。"""
        return self._events

    @property
    def log_directory(self) -> Path:
        """返回所有外部进程日志目录。"""
        return LOG_DIRECTORY

    def start(
        self,
        name: str,
        command: Iterable[str],
        *,
        cwd: Path = PROJECT_ROOT,
        setup_files: Iterable[Path] = (),
        extra_environment: dict[str, str] | None = None,
        keep_stdin_open: bool = False,
    ) -> ManagedProcess:
        """在独立会话启动命令，使整个后代进程组可被可靠终止。"""
        argv = tuple(str(part) for part in command)
        if not argv:
            raise ValueError("启动命令不能为空")
        working_directory = Path(cwd).expanduser().resolve()
        if not working_directory.is_dir():
            raise FileNotFoundError(f"工作目录不存在: {working_directory}")

        with self._lock:
            existing = self._managed.get(name)
            if existing is not None and existing.running:
                raise RuntimeError(f"进程 {name} 已在运行 (PID={existing.process.pid})")
            if existing is not None:
                if not self._close_record(existing):
                    raise RuntimeError(
                        f"进程 {name} 的旧日志线程仍未退出，拒绝覆盖记录"
                    )
                self._managed.pop(name, None)

            environment = build_sourced_environment(setup_files)
            if extra_environment:
                environment.update(extra_environment)
            log_path = LOG_DIRECTORY / f"{name}.log"
            log_stream = log_path.open("a", encoding="utf-8", buffering=1)
            started_at = datetime.now().isoformat(timespec="seconds")
            log_stream.write(f"\n--- {name} started {started_at} ---\n")
            log_stream.write("command: " + " ".join(argv) + "\n")
            self._events.info(name, f"启动进程：{' '.join(argv)}")
            try:
                process = subprocess.Popen(
                    argv,
                    cwd=working_directory,
                    env=environment,
                    # sim_vehicle.py 启动的 MAVProxy 把 EOF 视为退出命令；该流程
                    # 使用保持打开的管道复刻原终端输入，其余后台节点仍用 DEVNULL。
                    stdin=subprocess.PIPE if keep_stdin_open else subprocess.DEVNULL,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.STDOUT,
                    start_new_session=True,
                    text=True,
                    bufsize=1,
                )
            except Exception as exc:
                log_stream.close()
                self._events.error(name, f"进程启动失败：{exc}")
                raise
            record = ManagedProcess(
                name,
                process,
                argv,
                log_path,
                log_stream,
                process_group_id=os.getpgid(process.pid),
            )
            record.output_thread = threading.Thread(
                target=self._capture_output,
                args=(record,),
                name=f"ground-station-{name}-log",
                daemon=True,
            )
            record.output_thread.start()
            self._managed[name] = record
            return record

    def run_checked(
        self,
        command: Iterable[str],
        *,
        setup_files: Iterable[Path] = (),
        timeout: float = 10.0,
    ) -> subprocess.CompletedProcess[str]:
        """在相同 source 环境中执行短命令，用于初始化前依赖检查。"""
        argv = tuple(str(part) for part in command)
        self._events.debug("preflight", f"执行检查：{' '.join(argv)}")
        result = subprocess.run(
            argv,
            cwd=PROJECT_ROOT,
            env=build_sourced_environment(setup_files),
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            timeout=timeout,
            check=False,
        )
        if result.returncode != 0:
            detail = result.stdout.strip() or "无输出"
            self._events.error(
                "preflight", f"检查失败 (code={result.returncode})：{detail}"
            )
        return result

    def get(self, name: str) -> ManagedProcess | None:
        """返回指定受管进程记录。"""
        with self._lock:
            return self._managed.get(name)

    def terminate_all(self) -> CleanupReport:
        """分三阶段结束受管进程及历史残留，并在返回前再次扫描验证。"""
        self._events.info("process", "开始清理本项目本地仿真进程")
        errors: list[str] = []
        with self._lock:
            records = tuple(self._managed.values())

        # 必须在组长退出前抓取整个后代树。sim_vehicle 会启动新的 xterm/session，
        # 仅凭组长 poll 或单一 PGID 无法覆盖这些后代。
        tracked_groups = {record.process_group_id for record in records}
        tracked_pids = {record.process.pid for record in records}
        tracked_pids = self._with_descendants(tracked_pids)

        # 即使组长已在上一阶段退出，也继续向保存的 PGID 和后代 PID 升级信号。
        for signum, grace in (
            (signal.SIGINT, 2.0),
            (signal.SIGTERM, 2.0),
            (signal.SIGKILL, 1.0),
        ):
            if not self._targets_alive(tracked_groups, tracked_pids):
                break
            self._signal_targets(tracked_groups, tracked_pids, signum, errors)
            self._wait_targets(tracked_groups, tracked_pids, grace)

        managed_stopped = 0
        with self._lock:
            for name, record in tuple(self._managed.items()):
                try:
                    record.process.wait(timeout=0.1)
                except subprocess.TimeoutExpired:
                    errors.append(f"{name}: PID {record.process.pid} 无法结束")
                else:
                    managed_stopped += 1
                if not self._close_record(record):
                    errors.append(
                        f"{name}: 日志读取线程未在时限内退出；已跳过阻塞式流关闭"
                    )
                self._managed.pop(name, None)

        stale_pids = tuple(pid for pid, _ in self.find_related_processes())
        self._terminate_pids(stale_pids, errors)

        remaining = tuple(self.find_related_processes())
        report = CleanupReport(
            managed_stopped=managed_stopped,
            stale_stopped=stale_pids,
            remaining=remaining,
            errors=tuple(errors),
        )
        if report.success:
            self._events.info(
                "process",
                f"清理完成：受管进程 {managed_stopped}，历史残留 {len(stale_pids)}",
            )
        else:
            self._events.error(
                "process",
                f"清理不完整：残留={report.remaining}，错误={report.errors}",
            )
        return report

    def _capture_output(self, record: ManagedProcess) -> None:
        """将子进程输出同时写入磁盘和结构化实时日志。"""
        if record.process.stdout is None:
            return
        try:
            for raw_line in record.process.stdout:
                record.log_stream.write(raw_line)
                line = raw_line.rstrip("\r\n")
                if line:
                    level = self._explicit_output_level(line, record.name)
                    self._events.emit(level, record.name, line)
        except (OSError, ValueError) as exc:
            if record.running:
                self._events.warn(record.name, f"读取进程日志中断：{exc}")

    # SITL/MAVROS 启动期大量例行输出；降为 DEBUG 以免淹没操作事件。
    _VERBOSE_OUTPUT_MARKERS = (
        "embedding file",
        "loaded default parameter file",
        "validate_structures:",
        "included file",
        "param file",
        "using defaults from",
        "home location",
        "bind port",
        "waiting for heartbeat",
        "fcu url:",
        "gcs bridge",
        "plugin loaded",
        "plugin package:",
        "built-in base_node on",
        "built-in static_transform_publisher",
        "udpreclisten",
        "serial1:",
        "serial2:",
        "serial3:",
        "serial4:",
        "log directory:",
        "frame_id set to",
        "time offset",
        "imu: high resolution",
        "imu: setup",
        "gps: store",
        "gp_origin",
        "version: capabilities",
        "command: ",
        "mode: set mode",
        "component_manager",
        "subscription connected",
        "publisher connected",
        "service response",
        "discovered namespace",
    )

    # 这些源在启动和稳态时都会刷屏，默认只保留 WARN/ERROR 为更高可见等级。
    _CHATTY_SOURCES = frozenset(
        {
            "sitl",
            "mavros",
            "mavros_sim",
            "mavproxy",
            "onboard",
            "rviz",
            "guided_sim",
        }
    )

    @classmethod
    def _explicit_output_level(cls, line: str, source: str = "") -> LogLevel:
        """映射子进程输出等级；显式 WARN/ERROR 优先，启动噪音降为 DEBUG。"""
        normalized = line.upper()
        if any(marker in normalized for marker in ("[FATAL]", "[ERROR]")):
            return LogLevel.ERROR
        if any(marker in normalized for marker in ("[WARN]", "[WARNING]")):
            return LogLevel.WARN
        if "[DEBUG]" in normalized:
            return LogLevel.DEBUG
        lowered = line.casefold()
        if any(marker in lowered for marker in cls._VERBOSE_OUTPUT_MARKERS):
            return LogLevel.DEBUG
        # ROS 形如 [INFO] [stamp] [node]: msg 的常规启动信息，对 chatty 源降级。
        if source.casefold() in cls._CHATTY_SOURCES and (
            "[info]" in lowered or "[info " in lowered
        ):
            return LogLevel.DEBUG
        if source.casefold() in cls._CHATTY_SOURCES and cls._looks_like_startup_noise(
            lowered
        ):
            return LogLevel.DEBUG
        return LogLevel.INFO

    @staticmethod
    def _looks_like_startup_noise(lowered_line: str) -> bool:
        """识别无等级标记但仍属启动刷屏的 SITL/MAVROS 行。"""
        noise_prefixes = (
            "embedding ",
            "loaded ",
            "loading ",
            "init ",
            "initialis",
            "initializ",
            "starting ",
            "started ",
            "created ",
            "create ",
            "setup ",
            "config ",
            "configure ",
            "add_server",
            "advertise",
            "subscribed",
            "publishing",
            "published",
            "register",
            "waiting ",
            "connecting ",
            "connected ",
            "detected ",
            "using ",
            "found ",
            "open ",
            "opened ",
            "bind ",
            "listen ",
            "calibrat",
            "ekf3 ",
            "ekf2 ",
            "ahrs:",
            "rcout:",
            "ins:",
            "baro:",
            "compass:",
            "arsp:",
            "flow:",
            "rangefinder:",
            "battery:",
            "scheduler:",
        )
        stripped = lowered_line.lstrip()
        return any(stripped.startswith(prefix) for prefix in noise_prefixes)

    @classmethod
    def find_related_processes(cls) -> list[tuple[int, str]]:
        """扫描外部 ROS/SITL 进程；排除地面站自身及其祖先进程。"""
        excluded = cls._ancestor_pids(os.getpid())
        excluded.add(os.getpid())
        matches: list[tuple[int, str]] = []
        proc_root = Path("/proc")
        for entry in proc_root.iterdir():
            if not entry.name.isdigit():
                continue
            pid = int(entry.name)
            if pid in excluded:
                continue
            try:
                raw = (entry / "cmdline").read_bytes()
            except (FileNotFoundError, PermissionError, ProcessLookupError):
                continue
            argv = [
                token.decode("utf-8", errors="replace")
                for token in raw.split(b"\0")
                if token
            ]
            if not argv or not cls._is_related_argv(argv):
                continue
            matches.append((pid, " ".join(argv)))
        return sorted(matches)

    @classmethod
    def _is_related_argv(cls, argv: list[str]) -> bool:
        """基于 argv token 而非 shell 文本匹配，避免误杀包含关键词的诊断命令。"""
        basenames = [Path(token).name for token in argv]
        if "ground_station.py" in basenames:
            return False
        if any(name in cls._PROJECT_EXECUTABLES for name in basenames):
            return True

        command_line = " ".join(argv)
        if "sim_vehicle.py" in basenames and "GUID_OPTIONS=8" in command_line:
            return True
        if any(name in {"MAVProxy.py", "mavproxy.py"} for name in basenames):
            return "127.0.0.1:5762" in command_line
        if "mavros_node" in basenames:
            return "fcu_url:=tcp://127.0.0.1:5762" in command_line
        if any(name in {"rviz2", "robot_state_publisher"} for name in basenames):
            return "guided_sim" in command_line or "quadcopter.rviz" in command_line
        if "arducopter" in basenames:
            has_temporary_defaults = any(
                token.startswith("/tmp/tmp") for token in argv
            )
            return (
                "--sim-address=127.0.0.1" in command_line
                and "-I0" in argv
                and has_temporary_defaults
            )

        # 只识别本项目独有的包；通用 MAVROS 还必须匹配本地 SITL 端点。
        local_packages = {"guided_sim", "onboard_control"}
        for index, name in enumerate(basenames):
            if name != "ros2" or index + 2 >= len(argv):
                continue
            if (
                argv[index + 1] in {"launch", "run"}
                and argv[index + 2] in local_packages
            ):
                return True
            if (
                argv[index + 1] in {"launch", "run"}
                and argv[index + 2] == "mavros"
                and "fcu_url:=tcp://127.0.0.1:5762" in command_line
            ):
                return True
        return False

    @staticmethod
    def _ancestor_pids(pid: int) -> set[int]:
        """读取 /proc/status 构造祖先 PID 集合，防止清理命令杀死调用终端。"""
        ancestors: set[int] = set()
        current = pid
        while current > 1:
            try:
                status = Path(f"/proc/{current}/status").read_text(
                    encoding="utf-8", errors="replace"
                )
            except (FileNotFoundError, PermissionError):
                break
            parent = 0
            for line in status.splitlines():
                if line.startswith("PPid:"):
                    parent = int(line.split()[1])
                    break
            if parent <= 1 or parent in ancestors:
                break
            ancestors.add(parent)
            current = parent
        return ancestors

    @classmethod
    def _targets_alive(cls, groups: set[int], pids: set[int]) -> bool:
        """判断保存的进程组或后代 PID 是否仍有实际运行成员。"""
        return any(cls._group_alive(group) for group in groups) or any(
            cls._pid_alive(pid) for pid in pids
        )

    @classmethod
    def _wait_targets(
        cls, groups: set[int], pids: set[int], timeout: float
    ) -> None:
        """在统一宽限期内等待所有保存的进程组和后代退出。"""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if not cls._targets_alive(groups, pids):
                return
            time.sleep(0.05)

    @classmethod
    def _signal_targets(
        cls,
        groups: set[int],
        pids: set[int],
        signum: signal.Signals,
        errors: list[str],
    ) -> None:
        """向保存的 PGID 及逃逸到其他 session 的后代发送同一级信号。"""
        for group in sorted(groups):
            try:
                os.killpg(group, signum)
            except ProcessLookupError:
                pass
            except OSError as exc:
                errors.append(f"PGID {group}: {exc}")

        for pid in sorted(pids):
            if not cls._pid_alive(pid):
                continue
            try:
                if os.getpgid(pid) in groups:
                    continue
                os.kill(pid, signum)
            except ProcessLookupError:
                pass
            except OSError as exc:
                errors.append(f"PID {pid}: {exc}")

    @staticmethod
    def _group_alive(group: int) -> bool:
        """扫描非僵尸组成员，不把尚未 wait 的已退出组长误判为运行中。"""
        for entry in Path("/proc").iterdir():
            if not entry.name.isdigit():
                continue
            try:
                raw = (entry / "stat").read_text(
                    encoding="utf-8", errors="replace"
                )
                fields = raw.rsplit(")", 1)[1].split()
                state = fields[0]
                process_group = int(fields[2])
            except (FileNotFoundError, PermissionError, ProcessLookupError, ValueError):
                continue
            if process_group == group and state != "Z":
                return True
        return False

    @classmethod
    def _terminate_pids(cls, pids: Iterable[int], errors: list[str]) -> None:
        """先 TERM 后 KILL 清理未被当前实例跟踪的历史残留 PID。"""
        pending = cls._with_descendants(set(pids))
        for signum, grace in ((signal.SIGTERM, 1.0), (signal.SIGKILL, 0.5)):
            for pid in tuple(pending):
                try:
                    os.kill(pid, signum)
                except ProcessLookupError:
                    pending.discard(pid)
                except OSError as exc:
                    errors.append(f"PID {pid}: {exc}")
                    pending.discard(pid)
            deadline = time.monotonic() + grace
            while pending and time.monotonic() < deadline:
                pending = {pid for pid in pending if cls._pid_alive(pid)}
                time.sleep(0.05)
        for pid in sorted(pending):
            errors.append(f"PID {pid}: SIGKILL 后仍存活")

    @classmethod
    def _with_descendants(cls, roots: set[int]) -> set[int]:
        """展开残留进程的全部后代，防止 launch 父进程退出后留下未知子进程。"""
        if not roots:
            return set()
        parent_map: dict[int, int] = {}
        for entry in Path("/proc").iterdir():
            if not entry.name.isdigit():
                continue
            try:
                status = (entry / "status").read_text(
                    encoding="utf-8", errors="replace"
                )
            except (FileNotFoundError, PermissionError, ProcessLookupError):
                continue
            for line in status.splitlines():
                if line.startswith("PPid:"):
                    parent_map[int(entry.name)] = int(line.split()[1])
                    break

        expanded = set(roots)
        changed = True
        while changed:
            changed = False
            for pid, parent in parent_map.items():
                if parent in expanded and pid not in expanded:
                    expanded.add(pid)
                    changed = True
        expanded.discard(os.getpid())
        return expanded

    @staticmethod
    def _pid_alive(pid: int) -> bool:
        """判断 PID 是否为非僵尸进程；僵尸已不再执行代码，不算运行残留。"""
        try:
            raw = Path(f"/proc/{pid}/stat").read_text(
                encoding="utf-8", errors="replace"
            )
            fields = raw.rsplit(")", 1)[1].split()
        except (FileNotFoundError, PermissionError, ProcessLookupError):
            return False
        return bool(fields) and fields[0] != "Z"

    def _close_record(self, record: ManagedProcess) -> bool:
        """有界关闭日志；读取线程未退时绝不跨线程关闭其缓冲流。"""
        if record.process.stdin is not None:
            try:
                record.process.stdin.close()
            except OSError:
                pass
        if (
            record.output_thread is not None
            and record.output_thread is not threading.current_thread()
        ):
            record.output_thread.join(timeout=1.0)
        if (
            record.output_thread is not None
            and record.output_thread.is_alive()
            and record.output_thread is not threading.current_thread()
        ):
            # TextIOWrapper 正在另一线程执行阻塞 read 时，close() 会永久等待
            # buffered lock。保留 daemon 线程/流交给进程退出回收，先保证 GUI 可退出。
            return False
        try:
            if record.process.stdout is not None:
                record.process.stdout.close()
        except OSError:
            pass
        try:
            record.log_stream.flush()
            record.log_stream.close()
        except OSError:
            pass
        return True
