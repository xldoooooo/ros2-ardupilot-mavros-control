#!/usr/bin/env python3
"""独立的上位机 WebSocket 协议 V2.0 地面站模拟客户端。

本文件只连接 WebSocket、接收控制主题命令并向状态主题发布模拟回复。
它不导入地面站、ROS、MAVROS 或飞控代码，也不会连接、解锁或起飞真实飞机。
"""

from __future__ import annotations

import argparse
import asyncio
import contextlib
import json
import logging
import math
from collections.abc import Awaitable, Callable, Mapping
from copy import deepcopy
from typing import Any
from urllib.parse import urlparse

# 默认值可直接修改，也都能被同名命令行参数覆盖。
DEFAULT_WEBSOCKET_URL = "ws://127.0.0.1:8581/ws"
DEFAULT_CLIENT_NO = "UAV01001"
DEFAULT_VIDEO_PATH = "/home/share/test.mp4"
DEFAULT_JPG_DIRECTORY = "/home/share/jpg"
DEFAULT_POINT_PICTURE = "/home/share/jpg/test.jpg"

# 与当前地面站的航点输入边界保持一致；本模块不会把航点发给真实飞控。
MAX_WAYPOINT_COUNT = 256
WAYPOINT_HORIZONTAL_LIMIT_METERS = 10_000.0
WAYPOINT_MINIMUM_Z_METERS = 0.1
WAYPOINT_MAXIMUM_Z_METERS = 50.0
LOW_POWER_PERCENTAGE = 20.0

COMMAND_LABELS = {
    "01": "起飞",
    "02": "巡检任务下发",
    "03": "执行巡检任务",
    "05": "一键返航",
    "06": "降落",
    "07": "紧急停机",
}

REQUIRED_POINT_FIELDS = (
    "index",
    "x",
    "y",
    "z",
    "forwardAngle",
    "cameraAngle",
    "photoNo",
)

LOGGER = logging.getLogger("gcs-websocket-client")


class ProtocolError(ValueError):
    """收到不符合协议或当前 clientNo 的业务消息时抛出。"""


def command_topic(client_no: str) -> str:
    """返回指定模拟无人机的控制主题。"""

    return f"drone/{validate_client_no(client_no)}/command"


def status_topic(client_no: str) -> str:
    """返回指定模拟无人机的状态主题。"""

    return f"drone/{validate_client_no(client_no)}/status"


def validate_client_no(value: Any) -> str:
    """拒绝空编号以及会改变主题层级或通配语义的字符。"""

    if not isinstance(value, str) or not value.strip():
        raise ProtocolError("clientNo 必须是非空字符串")
    cleaned = value.strip()
    if any(character in cleaned for character in ("/", "\\", "#", "+")):
        raise ProtocolError("clientNo 不能包含 /、\\、# 或 +")
    return cleaned


def validate_url(value: str) -> str:
    """校验可用于客户端连接的 ws/wss URL。"""

    parsed = urlparse(value.strip())
    if parsed.scheme not in {"ws", "wss"} or not parsed.netloc:
        raise ProtocolError("WebSocket URL 必须使用 ws:// 或 wss:// 并包含主机")
    return value.strip()


def finite_number(value: Any, field: str) -> float:
    """解析有限 JSON 数值并明确排除布尔值、NaN 与无穷大。"""

    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ProtocolError(f"{field} 必须是数值")
    result = float(value)
    if not math.isfinite(result):
        raise ProtocolError(f"{field} 必须是有限数值")
    return result


def validate_command(payload: Any, expected_client_no: str) -> dict[str, Any]:
    """按当前地面站规则校验命令，并返回与网络帧隔离的副本。"""

    if not isinstance(payload, Mapping):
        raise ProtocolError("命令 data 必须是 JSON 对象")
    client_no = validate_client_no(payload.get("clientNo"))
    if client_no != expected_client_no:
        raise ProtocolError(
            f"命令 clientNo={client_no!r} 与当前编号 {expected_client_no!r} 不一致"
        )
    command_no = payload.get("commandNo")
    if command_no not in COMMAND_LABELS:
        raise ProtocolError(f"不支持的 commandNo: {command_no!r}")
    if command_no == "02":
        validate_task_points(payload.get("taskPoints"))
    return deepcopy(dict(payload))


def validate_task_points(value: Any) -> None:
    """校验巡检航点字段、数量、序号和当前地面站的坐标边界。"""

    if not isinstance(value, list) or not value:
        raise ProtocolError("commandNo=02 的 taskPoints 必须是非空数组")
    if len(value) > MAX_WAYPOINT_COUNT:
        raise ProtocolError(f"taskPoints 最多允许 {MAX_WAYPOINT_COUNT} 个航点")

    indexes: set[int] = set()
    for position, point in enumerate(value, start=1):
        if not isinstance(point, Mapping):
            raise ProtocolError(f"taskPoints[{position}] 必须是 JSON 对象")
        missing = [field for field in REQUIRED_POINT_FIELDS if field not in point]
        if missing:
            raise ProtocolError(
                f"taskPoints[{position}] 缺少字段: {', '.join(missing)}"
            )
        index = point["index"]
        if isinstance(index, bool) or not isinstance(index, int) or index < 1:
            raise ProtocolError(f"taskPoints[{position}].index 必须是正整数")
        if index in indexes:
            raise ProtocolError(f"taskPoints index={index} 重复")
        indexes.add(index)

        x = finite_number(point["x"], f"taskPoints[{position}].x")
        y = finite_number(point["y"], f"taskPoints[{position}].y")
        z = finite_number(point["z"], f"taskPoints[{position}].z")
        finite_number(
            point["forwardAngle"], f"taskPoints[{position}].forwardAngle"
        )
        finite_number(point["cameraAngle"], f"taskPoints[{position}].cameraAngle")
        if abs(x) > WAYPOINT_HORIZONTAL_LIMIT_METERS or abs(y) > (
            WAYPOINT_HORIZONTAL_LIMIT_METERS
        ):
            raise ProtocolError(f"taskPoints[{position}] 的 X/Y 超出地面站范围")
        if not WAYPOINT_MINIMUM_Z_METERS <= z <= WAYPOINT_MAXIMUM_Z_METERS:
            raise ProtocolError(
                f"taskPoints[{position}].z 必须在 "
                f"[{WAYPOINT_MINIMUM_Z_METERS}, {WAYPOINT_MAXIMUM_Z_METERS}] m 范围内"
            )

        # cameraAngle 当前只校验，photoNo 原样保留供未来媒体命名扩展。


def decode_object(raw: Any) -> dict[str, Any]:
    """把一个 UTF-8 WebSocket 文本帧严格解析为 JSON 对象。"""

    if not isinstance(raw, str):
        raise ProtocolError("仅支持 UTF-8 JSON 文本帧")
    try:
        value = json.loads(raw)
    except json.JSONDecodeError as exc:
        raise ProtocolError(f"收到无效 JSON：{exc.msg}") from exc
    if not isinstance(value, dict):
        raise ProtocolError("WebSocket 消息必须是 JSON 对象")
    return value


def make_status(
    client_no: str, status: str, data: Mapping[str, Any] | None = None
) -> dict[str, Any]:
    """构造协议 V2.0 状态业务对象。"""

    payload: dict[str, Any] = {"clientNo": client_no, "uavStatus": status}
    if data is not None:
        payload["data"] = deepcopy(dict(data))
    return payload


class GroundStationWebSocketSimulator:
    """维护一个自动重连会话并模拟地面站命令响应时序。"""

    def __init__(self, args: argparse.Namespace) -> None:
        self.url = validate_url(args.url)
        self.client_no = validate_client_no(args.client_no)
        self.power = args.power
        self.telemetry_interval = args.telemetry_interval
        self.takeoff_seconds = args.takeoff_seconds
        self.waypoint_seconds = args.waypoint_seconds
        self.return_seconds = args.return_seconds
        self.landing_seconds = args.landing_seconds
        self.takeoff_height = args.takeoff_height
        self.video_path = args.video_path
        self.jpg_directory = args.jpg_directory
        self.point_picture = args.point_picture
        self.reconnect_seconds = args.reconnect_seconds
        self.initial_handshake_timeout = args.handshake_timeout
        self.maximum_handshake_timeout = args.maximum_handshake_timeout

        self._initial_position = (args.x, args.y, args.z)
        self._position = list(self._initial_position)
        self._route: list[dict[str, Any]] = []
        self._airborne = False
        self._websocket: Any = None
        self._send_lock = asyncio.Lock()
        self._action_task: asyncio.Task[None] | None = None
        self._low_power_reported = False
        self._low_power_return_started = False
        self._handshake_completed = False

    async def run_forever(self) -> None:
        """持续连接服务端；连接失败或断线后按固定间隔重试。"""

        handshake_timeout = self.initial_handshake_timeout
        while True:
            self._handshake_completed = False
            try:
                await self._run_session(handshake_timeout)
            except asyncio.CancelledError:
                raise
            except ModuleNotFoundError as exc:
                if exc.name == "websockets":
                    raise RuntimeError(
                        "缺少 websockets，请先执行：python -m pip install -r requirements.txt"
                    ) from exc
                raise
            except Exception as exc:  # noqa: BLE001 - 网络边界统一进入重连。
                LOGGER.warning("WebSocket 会话中断：%s", exc)

            if self._handshake_completed:
                handshake_timeout = self.initial_handshake_timeout
            else:
                handshake_timeout = min(
                    self.maximum_handshake_timeout, handshake_timeout + 5.0
                )
            LOGGER.info(
                "%.1f 秒后重连 %s（下次握手超时 %.1f 秒）",
                self.reconnect_seconds,
                self.url,
                handshake_timeout,
            )
            await asyncio.sleep(self.reconnect_seconds)

    async def close(self) -> None:
        """取消模拟动作并关闭当前会话，供 Ctrl+C 安全退出。"""

        await self._cancel_action()
        websocket = self._websocket
        self._websocket = None
        if websocket is not None:
            with contextlib.suppress(Exception):
                await websocket.close()

    async def _run_session(self, handshake_timeout: float) -> None:
        """完成 SYSTEM/SUB_ACK 握手，再并行接收命令和发送遥测。"""

        import websockets

        LOGGER.info("正在连接 %s", self.url)
        async with websockets.connect(
            self.url,
            open_timeout=handshake_timeout,
            ping_interval=20,
            ping_timeout=20,
            close_timeout=2,
            max_size=2 * 1024 * 1024,
        ) as websocket:
            raw = await asyncio.wait_for(websocket.recv(), timeout=handshake_timeout)
            system = self._decode_and_log(raw)
            if system.get("type") != "SYSTEM":
                raise ProtocolError("WebSocket 首帧不是 SYSTEM")

            await self._send_json(
                websocket,
                {"type": "SUBSCRIBE", "topic": command_topic(self.client_no)},
            )
            while True:
                raw = await asyncio.wait_for(
                    websocket.recv(), timeout=handshake_timeout
                )
                message = self._decode_and_log(raw)
                if message.get("type") == "SUB_ACK" and message.get("topic") == (
                    command_topic(self.client_no)
                ):
                    break
                if message.get("type") == "ERROR":
                    raise ProtocolError(
                        f"订阅控制主题失败：{message.get('msg', '未知错误')}"
                    )

            self._websocket = websocket
            self._handshake_completed = True
            self._reset_runtime_state()
            LOGGER.info(
                "连接成功，已订阅 %s；状态发布到 %s",
                command_topic(self.client_no),
                status_topic(self.client_no),
            )

            # 每次建立新会话都先报告待机，再立即给出第一组电量和位置。
            await self._publish_status("01")
            telemetry_task = asyncio.create_task(
                self._telemetry_loop(), name="simulated-telemetry"
            )
            try:
                await self._reader_loop(websocket)
            finally:
                telemetry_task.cancel()
                with contextlib.suppress(asyncio.CancelledError):
                    await telemetry_task
                await self._cancel_action()
                self._websocket = None
                self._reset_runtime_state()

    async def _reader_loop(self, websocket: Any) -> None:
        """作为唯一 recv 消费者处理控制主题 BROADCAST 信封。"""

        async for raw in websocket:
            try:
                envelope = self._decode_and_log(raw)
                message_type = envelope.get("type")
                if message_type == "BROADCAST":
                    if envelope.get("topic") != command_topic(self.client_no):
                        continue
                    command = validate_command(envelope.get("data"), self.client_no)
                    await self._handle_command(command)
                elif message_type == "ERROR":
                    LOGGER.warning(
                        "主题服务返回错误：%s", envelope.get("msg", "未知错误")
                    )
                elif message_type not in {"HEARTBEAT_ACK", "SUB_ACK"}:
                    LOGGER.debug("忽略信封类型 %r", message_type)
            except ProtocolError as exc:
                # 协议没有定义失败 ACK；无效消息只拒绝，不伪造扩展响应。
                LOGGER.warning("拒绝无效上位机报文：%s", exc)

    async def _handle_command(self, command: dict[str, Any]) -> None:
        """先发布表 1 格式确认，再启动对应的非阻塞模拟动作。"""

        command_no = command["commandNo"]
        LOGGER.info("收到命令 %s（%s）", command_no, COMMAND_LABELS[command_no])
        await self._publish_business(
            {"clientNo": self.client_no, "commandNo": command_no}
        )

        if command_no == "02":
            self._route = deepcopy(command["taskPoints"])
            await self._publish_status("02")
            LOGGER.info("已保存 %d 个模拟巡检航点", len(self._route))
        elif command_no == "01":
            await self._replace_action("起飞", self._simulate_takeoff)
        elif command_no == "03":
            if not self._route:
                LOGGER.warning("尚未收到有效命令 02；命令 03 只回复接收确认，不启动巡检")
                return
            await self._replace_action("巡检", self._simulate_inspection)
        elif command_no == "05":
            await self._replace_action("返航", self._simulate_return_home)
        elif command_no in {"06", "07"}:
            await self._replace_action("降落", self._simulate_landing)

    async def _replace_action(
        self, label: str, action_factory: Callable[[], Awaitable[None]]
    ) -> None:
        """中断旧模拟动作并启动新动作，使返航/降落能抢占巡检。"""

        await self._cancel_action()
        self._action_task = asyncio.create_task(
            self._guard_action(label, action_factory()), name=f"simulated-{label}"
        )

    async def _cancel_action(self) -> None:
        """取消当前动作并等待其退出，避免旧任务继续发送状态。"""

        task = self._action_task
        self._action_task = None
        if task is None:
            return
        task.cancel()
        with contextlib.suppress(asyncio.CancelledError):
            await task

    async def _guard_action(self, label: str, action: Awaitable[None]) -> None:
        """隔离动作异常，网络接收和遥测不会被模拟逻辑拖垮。"""

        try:
            await action
        except asyncio.CancelledError:
            LOGGER.info("模拟%s动作已被新命令中断", label)
            raise
        except Exception:
            LOGGER.exception("模拟%s动作失败", label)
        finally:
            if self._action_task is asyncio.current_task():
                self._action_task = None

    async def _simulate_takeoff(self) -> None:
        """等待配置时长后把位置更新为模拟起飞高度。"""

        if self._airborne:
            LOGGER.info("模拟飞机已在空中，起飞命令按幂等操作完成")
            return
        LOGGER.info("模拟起飞中，预计 %.1f 秒", self.takeoff_seconds)
        await asyncio.sleep(self.takeoff_seconds)
        self._position[2] = self.takeoff_height
        self._airborne = True
        LOGGER.info("模拟起飞完成")

    async def _simulate_inspection(self) -> None:
        """模拟起飞、逐点巡检、末点降落和 08 完成状态。"""

        route = deepcopy(self._route)
        if not self._airborne:
            LOGGER.info("巡检前模拟起飞，预计 %.1f 秒", self.takeoff_seconds)
            await asyncio.sleep(self.takeoff_seconds)
            self._position[2] = float(route[0]["z"])
            self._airborne = True

        await self._publish_status("03")
        for point in route:
            LOGGER.info(
                "正在模拟飞往巡检点 %s，预计 %.1f 秒",
                point["index"],
                self.waypoint_seconds,
            )
            await asyncio.sleep(self.waypoint_seconds)
            self._position[:] = [
                float(point["x"]),
                float(point["y"]),
                float(point["z"]),
            ]
            await self._publish_status(
                "09",
                {
                    "pointNo": str(point["index"]),
                    "pointName": f"巡检点位 {point['index']}",
                    "pointPic": self.point_picture,
                },
            )

        await self._publish_status("07")
        LOGGER.info("末点模拟降落中，预计 %.1f 秒", self.landing_seconds)
        await asyncio.sleep(self.landing_seconds)
        self._position[2] = 0.0
        self._airborne = False
        await self._publish_status(
            "08",
            {"videoPath": self.video_path, "JPGPath": self.jpg_directory},
        )
        await self._publish_status("01")
        LOGGER.info("模拟巡检完成，已发送 08 和待机 01")

    async def _simulate_return_home(self) -> None:
        """模拟停止当前任务、返回原点、降落；返航不发送 08/09。"""

        await self._publish_status("05")
        LOGGER.info("模拟返航中，预计 %.1f 秒", self.return_seconds)
        await asyncio.sleep(self.return_seconds)
        self._position[0] = 0.0
        self._position[1] = 0.0
        await self._simulate_landing()

    async def _simulate_landing(self) -> None:
        """模拟原地降落并恢复待机状态。"""

        await self._publish_status("07")
        LOGGER.info("模拟降落中，预计 %.1f 秒", self.landing_seconds)
        await asyncio.sleep(self.landing_seconds)
        self._position[2] = 0.0
        self._airborne = False
        await self._publish_status("01")
        LOGGER.info("模拟降落完成，已恢复待机")

    async def _telemetry_loop(self) -> None:
        """按协议每周期连续发布一次 0A 电量和一次 0B 位置。"""

        while True:
            await self._publish_status("0A", {"uavPower": round(self.power, 2)})
            await self._publish_position()
            low_power = self.power < LOW_POWER_PERCENTAGE
            if low_power and not self._low_power_reported:
                await self._publish_status("0C")
                self._low_power_reported = True
            elif not low_power:
                self._low_power_reported = False
                self._low_power_return_started = False

            # 与地面站仿真行为一致：在线且空中低电量时，中断巡检并模拟返航。
            if low_power and self._airborne and not self._low_power_return_started:
                self._low_power_return_started = True
                LOGGER.warning("模拟电量低于 %.1f%%，开始自动返航", LOW_POWER_PERCENTAGE)
                await self._replace_action("低电量返航", self._simulate_return_home)
            await asyncio.sleep(self.telemetry_interval)

    async def _publish_position(self) -> None:
        """发布当前模拟 XYZ，供航点和起降动作立即反馈位置变化。"""

        x, y, z = self._position
        await self._publish_status(
            "0B", {"X": round(x, 3), "Y": round(y, 3), "Z": round(z, 3)}
        )

    async def _publish_status(
        self, status: str, data: Mapping[str, Any] | None = None
    ) -> None:
        """向状态主题发布一条协议状态业务对象。"""

        await self._publish_business(make_status(self.client_no, status, data))

    async def _publish_business(self, payload: Mapping[str, Any]) -> None:
        """给业务对象添加 PUBLISH 信封，并只通过 WebSocket 实际发送。"""

        websocket = self._websocket
        if websocket is None:
            raise ConnectionError("WebSocket 当前未连接")
        await self._send_json(
            websocket,
            {
                "type": "PUBLISH",
                "topic": status_topic(self.client_no),
                "data": deepcopy(dict(payload)),
            },
        )

    async def _send_json(self, websocket: Any, payload: Mapping[str, Any]) -> None:
        """串行发送紧凑 UTF-8 JSON，避免遥测和动作并发写帧。"""

        raw = json.dumps(dict(payload), ensure_ascii=False, separators=(",", ":"))
        async with self._send_lock:
            await websocket.send(raw)
        LOGGER.debug("TX %s", raw)

    @staticmethod
    def _decode_and_log(raw: Any) -> dict[str, Any]:
        """记录收到的精确文本并解析信封。"""

        LOGGER.debug("RX %s", raw if isinstance(raw, str) else "<binary frame>")
        return decode_object(raw)

    def _reset_runtime_state(self) -> None:
        """会话边界回到上电待机状态，同时保留最近一次命令 02 航线。"""

        self._position[:] = self._initial_position
        self._airborne = False
        self._low_power_reported = False
        self._low_power_return_started = False


def finite_argument(value: str) -> float:
    """argparse 使用的有限浮点数解析器。"""

    try:
        result = float(value)
    except ValueError as exc:
        raise argparse.ArgumentTypeError("必须是数值") from exc
    if not math.isfinite(result):
        raise argparse.ArgumentTypeError("必须是有限数值")
    return result


def positive_argument(value: str) -> float:
    """argparse 使用的正浮点数解析器。"""

    result = finite_argument(value)
    if result <= 0.0:
        raise argparse.ArgumentTypeError("必须大于 0")
    return result


def percentage_argument(value: str) -> float:
    """argparse 使用的 0～100 电量百分比解析器。"""

    result = finite_argument(value)
    if not 0.0 <= result <= 100.0:
        raise argparse.ArgumentTypeError("必须在 0～100 范围内")
    return result


def build_argument_parser() -> argparse.ArgumentParser:
    """定义所有可在不修改源码时调整的模拟参数。"""

    parser = argparse.ArgumentParser(
        description=(
            "独立地面站 WebSocket 模拟客户端；只收发协议消息，不连接或控制真实飞机。"
        )
    )
    parser.add_argument("--url", default=DEFAULT_WEBSOCKET_URL, help="WebSocket URL")
    parser.add_argument("--client-no", default=DEFAULT_CLIENT_NO, help="无人机编号")
    parser.add_argument(
        "--power", type=percentage_argument, default=55.6, help="模拟电量百分比"
    )
    parser.add_argument("--x", type=finite_argument, default=0.0, help="初始 X 坐标")
    parser.add_argument("--y", type=finite_argument, default=0.0, help="初始 Y 坐标")
    parser.add_argument("--z", type=finite_argument, default=0.0, help="初始 Z 坐标")
    parser.add_argument(
        "--telemetry-interval",
        type=positive_argument,
        default=1.0,
        help="0A/0B 上报周期秒数，协议默认 1 秒",
    )
    parser.add_argument(
        "--takeoff-seconds", type=positive_argument, default=5.0, help="模拟起飞耗时"
    )
    parser.add_argument(
        "--waypoint-seconds", type=positive_argument, default=5.0, help="每个航点耗时"
    )
    parser.add_argument(
        "--return-seconds", type=positive_argument, default=5.0, help="模拟返航耗时"
    )
    parser.add_argument(
        "--landing-seconds", type=positive_argument, default=5.0, help="模拟降落耗时"
    )
    parser.add_argument(
        "--takeoff-height", type=positive_argument, default=1.5, help="命令 01 的模拟高度"
    )
    parser.add_argument("--video-path", default=DEFAULT_VIDEO_PATH, help="状态 08 视频路径")
    parser.add_argument(
        "--jpg-directory", default=DEFAULT_JPG_DIRECTORY, help="状态 08 图片目录"
    )
    parser.add_argument(
        "--point-picture", default=DEFAULT_POINT_PICTURE, help="状态 09 图片路径"
    )
    parser.add_argument(
        "--reconnect-seconds", type=positive_argument, default=3.0, help="断线重连间隔"
    )
    parser.add_argument(
        "--handshake-timeout", type=positive_argument, default=15.0, help="首次握手超时"
    )
    parser.add_argument(
        "--maximum-handshake-timeout",
        type=positive_argument,
        default=30.0,
        help="连续失败时握手超时上限",
    )
    parser.add_argument(
        "--log-level",
        choices=("DEBUG", "INFO", "WARNING", "ERROR"),
        default="INFO",
        help="DEBUG 会显示完整 WebSocket 收发帧",
    )
    return parser


async def async_main(args: argparse.Namespace) -> None:
    """运行客户端直到 Ctrl+C 或进程收到取消。"""

    if args.maximum_handshake_timeout < args.handshake_timeout:
        raise ProtocolError("maximum-handshake-timeout 不能小于 handshake-timeout")
    simulator = GroundStationWebSocketSimulator(args)
    try:
        await simulator.run_forever()
    finally:
        await simulator.close()


def main() -> int:
    """命令行入口；配置错误返回非零状态，Ctrl+C 正常退出。"""

    parser = build_argument_parser()
    args = parser.parse_args()
    logging.basicConfig(
        level=getattr(logging, args.log_level),
        format="%(asctime)s %(levelname)s %(message)s",
        datefmt="%Y-%m-%d %H:%M:%S",
    )
    try:
        asyncio.run(async_main(args))
    except KeyboardInterrupt:
        LOGGER.info("已停止 WebSocket 模拟客户端")
    except (ProtocolError, RuntimeError) as exc:
        LOGGER.error("启动失败：%s", exc)
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
