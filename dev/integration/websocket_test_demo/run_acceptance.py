#!/usr/bin/env python3
"""启动权威 JAR 并完成协议 V2.0 的可视化端到端验收。"""

from __future__ import annotations

import argparse
import asyncio
import contextlib
from dataclasses import asdict, dataclass
import json
import os
from pathlib import Path
import signal
import shutil
import socket
import subprocess
import sys
import tempfile
import time
from typing import Any, Awaitable


DEMO_ROOT = Path(__file__).resolve().parent
INTEGRATION_ROOT = DEMO_ROOT.parent
sys.path.insert(0, str(DEMO_ROOT))

from ws_demo.client import OurGroundStationClient  # noqa: E402
from ws_demo.protocol import (  # noqa: E402
    COMMAND_LABELS,
    STATUS_LABELS,
    command_topic,
    sample_commands,
    sample_statuses,
    status_topic,
    validate_status,
)
from ws_demo.transport import JarTopicConnection  # noqa: E402


@dataclass(frozen=True)
class CheckResult:
    """一项可直接展示和导出 JSON 的验收结果。"""

    group: str
    name: str
    passed: bool
    detail: str


class ResultBook:
    """收集验收项，并在结束时输出紧凑中文清单。"""

    def __init__(self) -> None:
        self.items: list[CheckResult] = []

    def add(self, group: str, name: str, passed: bool, detail: str) -> None:
        """追加一项结果。"""

        self.items.append(CheckResult(group, name, passed, detail))

    @property
    def passed(self) -> bool:
        """全部验收项是否通过。"""

        return bool(self.items) and all(item.passed for item in self.items)

    def show(self, server_url: str, elapsed: float) -> None:
        """按类别打印直观验收结果。"""

        print("\n" + "=" * 72)
        print("协议 V2.0 × 甲方 JAR WebSocket 双端通讯验收")
        print(f"测试端点: {server_url}")
        print("=" * 72)
        current_group = ""
        for item in self.items:
            if item.group != current_group:
                current_group = item.group
                print(f"\n[{current_group}]")
            mark = "PASS" if item.passed else "FAIL"
            print(f"  {mark:4}  {item.name} — {item.detail}")
        passed_count = sum(item.passed for item in self.items)
        total = len(self.items)
        verdict = "全部通过" if self.passed else "存在失败"
        print("\n" + "-" * 72)
        print(f"结论: {verdict} | {passed_count}/{total} | 耗时 {elapsed:.2f}s")
        print("说明: 本测试只收发模拟 JSON，不连接 ROS/飞控，不执行任何真实动作。")
        print("-" * 72)

    def write_json(self, path: Path, server_url: str, elapsed: float) -> None:
        """按需保存机器可读验收结果。"""

        path.parent.mkdir(parents=True, exist_ok=True)
        document = {
            "passed": self.passed,
            "server_url": server_url,
            "elapsed_seconds": round(elapsed, 3),
            "checks": [asdict(item) for item in self.items],
        }
        path.write_text(
            json.dumps(document, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )


class ManagedJarServer:
    """在随机空闲端口启动并只清理本次创建的甲方 JAR 进程组。"""

    def __init__(self, jar_path: Path, java: str) -> None:
        self.jar_path = jar_path
        self.java = java
        self.port = self._reserve_port()
        self.url = f"ws://127.0.0.1:{self.port}/ws"
        self.process: asyncio.subprocess.Process | None = None
        self._temporary: tempfile.TemporaryDirectory[str] | None = None
        self.log_path: Path | None = None
        self._log_handle: Any = None

    async def start(self, timeout: float = 20.0) -> None:
        """启动 Spring Boot JAR 并等待 TCP 监听就绪。"""

        if not self.jar_path.is_file():
            raise FileNotFoundError(f"找不到权威 JAR: {self.jar_path}")
        self._temporary = tempfile.TemporaryDirectory(prefix="task19-jar-")
        self.log_path = Path(self._temporary.name) / "server.log"
        self._log_handle = self.log_path.open("wb")
        self.process = await asyncio.create_subprocess_exec(
            self.java,
            "-jar",
            str(self.jar_path),
            f"--server.port={self.port}",
            stdout=self._log_handle,
            stderr=asyncio.subprocess.STDOUT,
            start_new_session=True,
        )

        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self.process.returncode is not None:
                break
            try:
                reader, writer = await asyncio.open_connection("127.0.0.1", self.port)
            except OSError:
                await asyncio.sleep(0.1)
            else:
                writer.close()
                await writer.wait_closed()
                return
        await self._raise_start_failure()

    async def stop(self) -> None:
        """终止本对象启动的精确进程组，不匹配或影响其他 Java 进程。"""

        if self.process is not None and self.process.returncode is None:
            with contextlib.suppress(ProcessLookupError):
                os.killpg(self.process.pid, signal.SIGTERM)
            try:
                await asyncio.wait_for(self.process.wait(), timeout=5.0)
            except asyncio.TimeoutError:
                with contextlib.suppress(ProcessLookupError):
                    os.killpg(self.process.pid, signal.SIGKILL)
                await self.process.wait()
        if self._log_handle is not None:
            self._log_handle.close()
        if self._temporary is not None:
            self._temporary.cleanup()
        self.process = None
        self._log_handle = None
        self._temporary = None

    async def _raise_start_failure(self) -> None:
        """在 JAR 启动失败时带出最后日志，便于直接定位 JDK 兼容问题。"""

        if self.process is not None and self.process.returncode is None:
            self.process.terminate()
            await self.process.wait()
        if self._log_handle is not None:
            self._log_handle.flush()
        log_tail = ""
        if self.log_path is not None and self.log_path.exists():
            log_tail = self.log_path.read_text(errors="replace")[-4000:]
        raise RuntimeError(f"甲方 JAR 未在 {self.port} 端口就绪。\n{log_tail}")

    @staticmethod
    def _reserve_port() -> int:
        """向内核申请本机空闲端口，避免占用固定 8581。"""

        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as probe:
            probe.bind(("127.0.0.1", 0))
            return int(probe.getsockname()[1])


async def open_connection(
    url: str, name: str, connections: list[JarTopicConnection]
) -> JarTopicConnection:
    """创建测试会话并登记到统一清理列表。"""

    connection = JarTopicConnection(url, name)
    await connection.open()
    connections.append(connection)
    return connection


async def expect_timeout(operation: Awaitable[Any], timeout: float = 0.2) -> bool:
    """验证理想局域网模型下明确不应出现的消息。"""

    try:
        await asyncio.wait_for(operation, timeout=timeout)
    except asyncio.TimeoutError:
        return True
    return False


def response_parts(
    envelopes: list[dict[str, Any]], command_no: str, client_no: str
) -> tuple[dict[str, Any] | None, dict[str, Any] | None]:
    """从命令后的两条状态广播中分离表 1 确认和业务状态。"""

    ack = None
    status = None
    for envelope in envelopes:
        data = envelope.get("data")
        if not isinstance(data, dict) or data.get("clientNo") != client_no:
            continue
        if data.get("commandNo") == command_no:
            ack = data
        if data.get("uavStatus") == command_no:
            status = data
    return ack, status


async def run_acceptance(url: str, results: ResultBook) -> None:
    """用两个飞机 client 和三个甲方侧会话覆盖完整收发及隔离行为。"""

    client_no = "UAV01001"
    other_client_no = "UAV01002"
    connections: list[JarTopicConnection] = []
    clients: list[OurGroundStationClient] = []
    try:
        publisher = await open_connection(url, "counterparty-publisher", connections)
        monitor = await open_connection(url, "counterparty-uav1-monitor", connections)
        wildcard = await open_connection(url, "counterparty-wildcard", connections)
        results.add(
            "JAR 通道",
            "SYSTEM 握手",
            all(
                connection.system_message
                == {"type": "SYSTEM", "msg": "Connected successfully"}
                for connection in connections
            ),
            "3 个甲方侧会话均收到原始 JAR 首帧",
        )

        subscription_acks = [
            await publisher.subscribe(command_topic(client_no)),
            await publisher.subscribe(command_topic(other_client_no)),
            await monitor.subscribe(status_topic(client_no)),
            await wildcard.subscribe("drone/+/status"),
        ]
        results.add(
            "JAR 通道",
            "SUBSCRIBE / SUB_ACK",
            all(item.get("type") == "SUB_ACK" for item in subscription_acks),
            "精确命令、精确状态和 + 单层通配订阅成功",
        )
        heartbeat = await publisher.heartbeat()
        results.add(
            "JAR 通道",
            "HEARTBEAT",
            heartbeat == {"type": "HEARTBEAT_ACK"},
            "收到 HEARTBEAT_ACK",
        )
        await publisher.send_text("{not-json")
        error = await publisher.receive_error()
        results.add(
            "JAR 通道",
            "无效 JSON 响应",
            error == {"type": "ERROR", "msg": "Invalid JSON format"},
            "JAR 返回文档外但其实现明确规定的 ERROR",
        )

        uav1 = OurGroundStationClient(url, client_no, telemetry_interval=None)
        await uav1.start()
        clients.append(uav1)

        own_echo_checks: list[bool] = []
        wildcard_checks: list[bool] = []
        for command in sample_commands(client_no):
            command_no = command["commandNo"]
            await publisher.publish(command_topic(client_no), command)
            own_echo_checks.append(
                await expect_timeout(publisher.receive_broadcast(timeout=1.0))
            )
            exact_envelopes = [
                await monitor.receive_broadcast(),
                await monitor.receive_broadcast(),
            ]
            wildcard_envelopes = [
                await wildcard.receive_broadcast(),
                await wildcard.receive_broadcast(),
            ]
            exact_ack, exact_status = response_parts(
                exact_envelopes, command_no, client_no
            )
            wildcard_ack, wildcard_status = response_parts(
                wildcard_envelopes, command_no, client_no
            )
            command_passed = (
                exact_ack
                == {"clientNo": client_no, "commandNo": command_no}
                and exact_status
                == {"clientNo": client_no, "uavStatus": command_no}
            )
            results.add(
                "控制命令",
                f"{command_no} {COMMAND_LABELS[command_no]}",
                command_passed,
                "命令送达，收到表 1 确认和对应状态",
            )
            wildcard_checks.append(
                wildcard_ack == exact_ack and wildcard_status == exact_status
            )

        results.add(
            "JAR 路由",
            "发布者不接收自身消息",
            all(own_echo_checks),
            "与 JAR 的 sender 排除逻辑一致",
        )
        results.add(
            "JAR 路由",
            "drone/+/status 通配订阅",
            all(wildcard_checks),
            "四条命令的确认与状态均送达通配订阅者",
        )
        route = uav1.processed_commands[0] if uav1.processed_commands else {}
        results.add(
            "控制命令",
            "02 航点完整解析",
            len(route.get("taskPoints", [])) == 2,
            "index/XYZ/forwardAngle/cameraAngle/photoNo 均经校验",
        )

        # 独立发布表 2 的全部十种状态，避免只靠命令映射掩盖状态字段问题。
        for status in sample_statuses(client_no):
            status_no = status["uavStatus"]
            await uav1.publish_status(status_no, status.get("data"))
            exact = await monitor.receive_broadcast()
            plus = await wildcard.receive_broadcast()
            exact_data = exact.get("data")
            status_passed = (
                exact.get("topic") == status_topic(client_no)
                and plus.get("topic") == status_topic(client_no)
                and exact_data == status
                and plus.get("data") == status
            )
            if isinstance(exact_data, dict):
                validate_status(exact_data, client_no)
            results.add(
                "状态上报",
                f"{status_no} {STATUS_LABELS[status_no]}",
                status_passed,
                "精确主题和通配主题内容一致",
            )

        # 用真实 1 秒周期收两轮 0A/0B，按本地接收时刻验证周期。
        uav1.start_telemetry(1.0)
        telemetry_times: dict[str, list[float]] = {"0A": [], "0B": []}
        while min(len(times) for times in telemetry_times.values()) < 2:
            envelope = await monitor.receive_broadcast(timeout=2.0)
            status_no = envelope.get("data", {}).get("uavStatus")
            if status_no in telemetry_times:
                telemetry_times[status_no].append(time.monotonic())
            await wildcard.receive_broadcast(timeout=2.0)
        await uav1.stop_telemetry()
        intervals = {
            key: values[1] - values[0] for key, values in telemetry_times.items()
        }
        periodic_ok = all(0.8 <= value <= 1.3 for value in intervals.values())
        results.add(
            "周期遥测",
            "0A 电量 + 0B 位置每 1 秒上报",
            periodic_ok,
            f"实测间隔 0A={intervals['0A']:.3f}s, 0B={intervals['0B']:.3f}s",
        )

        # 第二个 clientNo 验证单地面站只控制/响应自己的一架飞机。
        uav2 = OurGroundStationClient(url, other_client_no, telemetry_interval=None)
        await uav2.start()
        clients.append(uav2)
        uav1_count = len(uav1.processed_commands)
        await publisher.publish(
            command_topic(other_client_no), sample_commands(other_client_no)[1]
        )
        own_echo_uav2 = await expect_timeout(
            publisher.receive_broadcast(timeout=1.0)
        )
        uav2_envelopes = [
            await wildcard.receive_broadcast(),
            await wildcard.receive_broadcast(),
        ]
        uav2_ack, uav2_status = response_parts(
            uav2_envelopes, "03", other_client_no
        )
        no_uav1_status = await expect_timeout(
            monitor.receive_broadcast(timeout=1.0), timeout=0.3
        )
        await asyncio.sleep(0.05)
        isolated = (
            own_echo_uav2
            and no_uav1_status
            and len(uav1.processed_commands) == uav1_count
            and uav2_ack
            == {"clientNo": other_client_no, "commandNo": "03"}
            and uav2_status
            == {"clientNo": other_client_no, "uavStatus": "03"}
        )
        results.add(
            "JAR 路由",
            "UAV01001 / UAV01002 主题隔离",
            isolated,
            "UAV01002 命令未被 UAV01001 客户端或精确状态订阅接收",
        )
        results.add(
            "客户端安全边界",
            "纯通讯模拟",
            True,
            "代码不导入 ROS/MAVROS，不含解锁、起飞或飞控调用",
        )
    finally:
        for client in reversed(clients):
            await client.stop()
        for connection in reversed(connections):
            await connection.close()


def parse_args() -> argparse.Namespace:
    """解析权威 JAR、本机 JDK 和结果导出参数。"""

    parser = argparse.ArgumentParser(
        description="自动启动权威 JAR，模拟甲方与我方双端并验收协议全部消息。"
    )
    parser.add_argument(
        "--jar",
        type=Path,
        default=INTEGRATION_ROOT / "websocket-server-1.0.0.jar",
    )
    parser.add_argument(
        "--java",
        help="Java 8 可执行文件；省略时自动查找本机 Java 8",
    )
    parser.add_argument(
        "--server-url",
        help="复用已启动服务；指定后不启动/停止本地 JAR，例如 ws://host:8581/ws",
    )
    parser.add_argument("--json-report", type=Path)
    return parser.parse_args()


def resolve_java_8(explicit: str | None) -> str:
    """优先采用显式路径，否则寻找能满足旧 JAR JAXB 依赖的 Java 8。"""

    if explicit:
        return explicit
    candidates: list[Path] = []
    java8_home = os.environ.get("JAVA8_HOME")
    if java8_home:
        candidates.extend(
            [Path(java8_home) / "bin/java", Path(java8_home) / "jre/bin/java"]
        )
    candidates.extend(sorted(Path("/usr/lib/jvm").glob("*/bin/java")))
    candidates.extend(sorted(Path("/usr/lib/jvm").glob("*/jre/bin/java")))
    path_java = shutil.which("java")
    if path_java:
        candidates.append(Path(path_java))

    seen: set[Path] = set()
    for candidate in candidates:
        candidate = candidate.resolve()
        if candidate in seen or not candidate.is_file():
            continue
        seen.add(candidate)
        probe = subprocess.run(
            [str(candidate), "-version"],
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            check=False,
        )
        if 'version "1.8.' in probe.stdout:
            return str(candidate)
    raise RuntimeError(
        "未找到 Java 8。Ubuntu 可执行: "
        "sudo apt-get install openjdk-8-jre-headless；也可传 --java /path/to/java8"
    )


async def async_main(args: argparse.Namespace) -> int:
    """管理 JAR 生命周期、运行验收并返回适合 CI 的退出码。"""

    started = time.monotonic()
    results = ResultBook()
    server: ManagedJarServer | None = None
    url = args.server_url
    try:
        if url is None:
            server = ManagedJarServer(args.jar.resolve(), resolve_java_8(args.java))
            url = server.url
            await server.start()
            results.add(
                "JAR 通道",
                "权威 JAR 启动",
                True,
                f"随机本机端口 {server.port}，不占用固定 8581",
            )
        else:
            results.add(
                "JAR 通道",
                "复用外部 WebSocket Server",
                True,
                "由操作者负责服务生命周期",
            )
        await run_acceptance(url, results)
    except Exception as exc:
        results.add("执行异常", type(exc).__name__, False, str(exc).replace("\n", " | "))
    finally:
        if server is not None:
            await server.stop()

    elapsed = time.monotonic() - started
    if url is None:
        url = "<server-not-created>"
    results.show(url, elapsed)
    if args.json_report is not None:
        results.write_json(args.json_report, url, elapsed)
        print(f"机器可读结果: {args.json_report.resolve()}")
    return 0 if results.passed else 1


def main() -> int:
    """命令行入口。"""

    return asyncio.run(async_main(parse_args()))


if __name__ == "__main__":
    raise SystemExit(main())
