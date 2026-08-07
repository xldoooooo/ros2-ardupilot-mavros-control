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
                self._close_record(existing)
                self._managed.pop(name, None)

            environment = build_sourced_environment(setup_files)
            if extra_environment:
                environment.update(extra_environment)
            log_path = LOG_DIRECTORY / f"{name}.log"
            log_stream = log_path.open("a", encoding="utf-8", buffering=1)
            log_stream.write(
                f"\n--- {name} started {datetime.now().isoformat(timespec='seconds')} ---\n"
            )
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
            record = ManagedProcess(name, process, argv, log_path, log_stream)
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

        # ROS/SITL 先接收 SIGINT 以执行自身清理，再逐步升级到 SIGKILL。
        for signum, grace in (
            (signal.SIGINT, 2.0),
            (signal.SIGTERM, 2.0),
            (signal.SIGKILL, 1.0),
        ):
            running = [record for record in records if record.running]
            if not running:
                break
            for record in running:
                try:
                    os.killpg(os.getpgid(record.process.pid), signum)
                except ProcessLookupError:
                    pass
                except OSError as exc:
                    errors.append(f"{record.name}: {exc}")
            self._wait_records(running, grace)

        managed_stopped = 0
        with self._lock:
            for name, record in tuple(self._managed.items()):
                try:
                    record.process.wait(timeout=0.1)
                except subprocess.TimeoutExpired:
                    errors.append(f"{name}: PID {record.process.pid} 无法结束")
                else:
                    managed_stopped += 1
                self._close_record(record)
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
                    self._events.emit(self._explicit_output_level(line), record.name, line)
        except (OSError, ValueError) as exc:
            if record.running:
                self._events.warn(record.name, f"读取进程日志中断：{exc}")

    @staticmethod
    def _explicit_output_level(line: str) -> LogLevel:
        """只映射输出自带的标准等级标记；无标记输出按 INFO 记录。"""
        normalized = line.upper()
        if any(marker in normalized for marker in ("[FATAL]", "[ERROR]")):
            return LogLevel.ERROR
        if any(marker in normalized for marker in ("[WARN]", "[WARNING]")):
            return LogLevel.WARN
        if "[DEBUG]" in normalized:
            return LogLevel.DEBUG
        return LogLevel.INFO

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

        # 只识别本项目独有的包；通用 MAVROS 还必须匹配本地 SITL 端点。
        local_packages = {"guided_sim", "onboard_control"}
        for index, name in enumerate(basenames):
            if name != "ros2" or index + 2 >= len(argv):
                continue
            if argv[index + 1] in {"launch", "run"} and argv[index + 2] in local_packages:
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

    @staticmethod
    def _wait_records(records: Iterable[ManagedProcess], timeout: float) -> None:
        """在短宽限期内轮询多个进程，避免逐个 wait 叠加超时。"""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if all(not record.running for record in records):
                return
            time.sleep(0.05)

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
            fields = Path(f"/proc/{pid}/stat").read_text(
                encoding="utf-8", errors="replace"
            ).split()
        except (FileNotFoundError, PermissionError, ProcessLookupError):
            return False
        return len(fields) > 2 and fields[2] != "Z"

    def _close_record(self, record: ManagedProcess) -> None:
        """关闭记录对应的日志文件。"""
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
        try:
            if record.process.stdout is not None:
                record.process.stdout.close()
        except OSError:
            pass
        if (
            record.output_thread is not None
            and record.output_thread.is_alive()
            and record.output_thread is not threading.current_thread()
        ):
            record.output_thread.join(timeout=0.2)
        try:
            record.log_stream.flush()
            record.log_stream.close()
        except OSError:
            pass
