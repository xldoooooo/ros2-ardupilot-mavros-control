from __future__ import annotations

import struct
import time
from dataclasses import dataclass
from typing import Protocol, Sequence

from protocol import (
    CMD_ABORT,
    CMD_UAV_H_START,
    CMD_UAV_LOAD_START,
    CMD_PING,
    CMD_START,
)
from qubo_model import (
    N_HW,
    Problem,
    build_uav_compact_h_payload,
    build_uav_compact_payload,
    decode_spins,
    dequantize_q9_22,
    uav_compact_payload_fingerprints,
    uav_compact_payload_info,
    uav_logical_problem_fingerprints,
)


REQUIRED_SW_BUILD_ID = 0x44325257
REQUIRED_SW_VERSION = 0x04010351
REQUIRED_PL_HEARTBEAT_FLAG = 0x00004000
REQUIRED_PL_IDENTITY_FLAG = 0x00008000
REQUIRED_PL_ACCESS_FLAG = 0x00000008
REQUIRED_DMA_LOADER_FLAG = 0x00040000
REQUIRED_UAV_COMPACT_FLAG = 0x00200000
REQUIRED_ACTIVE_N_CORE_FLAG = 0x00400000
START_FLAG_ADAPTIVE_BUDGET = 0x01
START_FLAG_EARLY_RESTART_STOP = 0x02
SEED_STEP = 0x9E3779B9
# Board firmware has a 5 s solve watchdog; keep the host timeout bounded so a
# lost UDP transaction cannot stall ROS/GUI for minutes.
SOLVE_TIMEOUT_S = 8.0
_NO_PROBLEM_TX_LOAD_MODES = {"UAV-CACHE", "ACTIVE-N-CACHE", "MOCK-CACHE"}


def _problem_tx_rtt_ms(load_mode: str, serial_rtt_ms: float) -> float:
    return 0.0 if str(load_mode).upper() in _NO_PROBLEM_TX_LOAD_MODES else float(serial_rtt_ms)


class HostTransactor(Protocol):
    _fpga_problem_crc: int | None
    _fpga_j_crc: int | None
    _fpga_uav_problem_crc: int | None
    _fpga_uav_j_crc: int | None

    def transact(
        self, cmd: int, payload: bytes = b"", timeout: float = 3.0
    ) -> tuple[bytes, float, str]: ...

    def _decode_ping_payload(self, payload: bytes) -> dict: ...

    def _invalidate_fpga_cache(self) -> None: ...


@dataclass(frozen=True)
class FpgaRuntimeStatus:
    status: int
    h_writes: int
    j_commits: int
    build_id: int
    version: int
    node_count: int
    serial_rtt_ms: float
    transport: str = ""


@dataclass(frozen=True)
class FpgaSolveResult:
    status: int
    energy_raw: int
    energy: float
    node_count: int
    iterations: int
    elapsed_us: int
    spins: tuple[int, ...]
    binary: tuple[int, ...]
    packed_spins: int
    serial_rtt_ms: float
    restarts: int = 1
    best_restart: int = 0
    transport: str = ""
    load_mode: str = "CACHE"
    # Firmware-side timing/profile extension (protocol v0x04010351).  Keeping
    # these fields on the returned run lets ROS distinguish network/load time
    # from the actual PL anneal time instead of attributing all latency to FPGA.
    load_decode_us: int = 0
    load_pl_us: int = 0
    load_total_us: int = 0
    board_load_mode: int = 0
    transport_rx_us: int = 0
    hw_cycles: int = 0
    perf_stats: int = 0
    perf_stats2: int = 0
    profile_pl_solve_us: int = 0
    profile_result_read_us: int = 0
    profile_restart_overhead_us: int = 0
    adaptive_info: int = 0
    reduced_budget: int = 0
    issued_iteration_budget: int = 0


@dataclass(frozen=True)
class FpgaBatchResult:
    load_status: FpgaRuntimeStatus
    runs: tuple[FpgaSolveResult, ...]


def _node_count_from_hardware(hardware: int) -> int:
    code = hardware & 0xFF
    return 256 if code == 0 else code


def decode_status_payload(
    payload: bytes, rtt_ms: float, transport: str = ""
) -> FpgaRuntimeStatus:
    if len(payload) < 20:
        raise ValueError(f"状态负载长度错误: {len(payload)}")
    status, counts, build_id, version, hardware = struct.unpack_from("<IIIII", payload)
    return FpgaRuntimeStatus(
        status=status,
        h_writes=counts & 0xFFFF,
        j_commits=(counts >> 16) & 0xFFFF,
        build_id=build_id,
        version=version,
        node_count=_node_count_from_hardware(hardware),
        serial_rtt_ms=rtt_ms,
        transport=transport,
    )


def decode_result_payload(
    payload: bytes,
    rtt_ms: float,
    transport: str = "",
    load_mode: str = "CACHE",
) -> FpgaSolveResult:
    spin_bytes = (N_HW * 2 + 7) // 8
    meta = 8 + spin_bytes
    if len(payload) < meta + 8:
        raise ValueError(f"FPGA结果负载长度错误: {len(payload)}")
    status = struct.unpack_from("<I", payload, 0)[0]
    energy_raw = struct.unpack_from("<i", payload, 4)[0]
    spin_payload = payload[8 : 8 + spin_bytes]
    spins = tuple(decode_spins(spin_payload, N_HW))
    iterations = struct.unpack_from("<H", payload, meta)[0]
    restarts = payload[meta + 2]
    best_restart = payload[meta + 3]
    elapsed_us = struct.unpack_from("<I", payload, meta + 4)[0]
    # Current firmware appends 14 u32 diagnostics after the legacy 8-byte
    # result metadata.  Decode opportunistically so older mock packets remain
    # readable, while protocol-version gating protects real hardware use.
    ext = [0] * 14
    if len(payload) >= meta + 64:
        ext = list(struct.unpack_from("<14I", payload, meta + 8))
    packed = int.from_bytes(spin_payload, "little")
    return FpgaSolveResult(
        status=status,
        energy_raw=energy_raw,
        energy=dequantize_q9_22(energy_raw),
        node_count=N_HW,
        iterations=iterations,
        elapsed_us=elapsed_us,
        spins=spins,
        binary=tuple(1 if spin > 0 else 0 for spin in spins),
        packed_spins=packed,
        serial_rtt_ms=rtt_ms,
        restarts=restarts,
        best_restart=best_restart,
        transport=transport,
        load_mode=load_mode,
        load_decode_us=ext[0],
        load_pl_us=ext[1],
        load_total_us=ext[2],
        board_load_mode=ext[3],
        transport_rx_us=ext[4],
        hw_cycles=ext[5],
        perf_stats=ext[6],
        perf_stats2=ext[7],
        profile_pl_solve_us=ext[8],
        profile_result_read_us=ext[9],
        profile_restart_overhead_us=ext[10],
        adaptive_info=ext[11],
        reduced_budget=ext[12],
        issued_iteration_budget=ext[13],
    )


class FpgaUavSolver:
    """Adapter from the UAV planner contract to the 256-node GUI transport."""

    def __init__(
        self,
        host: HostTransactor,
        adaptive_budget: bool = False,
        early_restart_stop: bool = False,
    ) -> None:
        self.host = host
        self.adaptive_budget = bool(adaptive_budget)
        self.early_restart_stop = bool(early_restart_stop)
        self._last_runtime: FpgaRuntimeStatus | None = None
        self._auto_seed = 0x12802026
        self._warm_start_notice_sent = False
        self._last_payload_signature: tuple[int, str, int, int] | None = None
        self.last_host_profile: dict[str, float | str | int] = {}
        self._last_tx_wall_ms = 0.0
        self._last_decode_ms = 0.0
        self._last_load_mode = ""

    def _log(self, message: str) -> None:
        signal = getattr(self.host, "log_message", None)
        if signal is not None:
            signal.emit(message)
            return
        append = getattr(self.host, "append", None)
        if append is not None:
            append(message)

    def _runtime_from_ping(
        self, info: dict, rtt_ms: float, transport: str
    ) -> FpgaRuntimeStatus:
        counts = int(info.get("counts", 0))
        return FpgaRuntimeStatus(
            status=int(info.get("status", 0)),
            h_writes=counts & 0xFFFF,
            j_commits=(counts >> 16) & 0xFFFF,
            build_id=int(info.get("build_id", 0)),
            version=int(info.get("version", 0)),
            node_count=_node_count_from_hardware(int(info.get("hardware", 0))),
            serial_rtt_ms=rtt_ms,
            transport=transport,
        )

    def _validate_runtime_info(self, info: dict) -> None:
        if (
            int(info["build_id"]) != REQUIRED_SW_BUILD_ID
            or int(info["version"]) != REQUIRED_SW_VERSION
        ):
            raise RuntimeError(
                "固件版本不匹配，已阻止UAV FPGA规划。"
                f"要求build={REQUIRED_SW_BUILD_ID:08X}, "
                f"version={REQUIRED_SW_VERSION:08X}；"
                f"实际build={int(info['build_id']):08X}, "
                f"version={int(info['version']):08X}。"
            )

        flags = int(info.get("transport_flags", 0))
        missing = []
        if (flags & REQUIRED_PL_HEARTBEAT_FLAG) == 0:
            missing.append("PL_HEARTBEAT")
        if (flags & REQUIRED_PL_IDENTITY_FLAG) == 0:
            missing.append("PL_IDENTITY")
        if (flags & REQUIRED_PL_ACCESS_FLAG) == 0:
            missing.append("PL_ACCESS")
        if (flags & REQUIRED_DMA_LOADER_FLAG) == 0:
            missing.append("DMA_LOADER")
        if (flags & REQUIRED_UAV_COMPACT_FLAG) == 0:
            missing.append("UAV_COMPACT")
        if (flags & REQUIRED_ACTIVE_N_CORE_FLAG) == 0:
            missing.append("ACTIVE_N_CORE")
        if missing:
            raise RuntimeError(
                "PL安全门禁未通过，已阻止UAV LOAD/START："
                + ", ".join(missing)
            )

        hardware = int(info["hardware"])
        reported_n = _node_count_from_hardware(hardware)
        reported_words = (hardware >> 8) & 0xFF
        reported_chunks = (hardware >> 16) & 0xFFFF
        expected_words = (N_HW + 31) // 32
        expected_chunks = N_HW * expected_words
        if (reported_n, reported_words, reported_chunks) != (
            N_HW,
            expected_words,
            expected_chunks,
        ):
            raise RuntimeError(
                "PL规模/存储布局不匹配："
                f"PING报告N={reported_n}, words/row={reported_words}, "
                f"chunks={reported_chunks}；要求N={N_HW}, "
                f"words/row={expected_words}, chunks={expected_chunks}。"
            )

    def preflight_host_path(self) -> None:
        """Move one-time host/log initialization out of the first timed solve."""
        if not self._warm_start_notice_sent:
            self._log(
                "UAV提示：256运行时START协议不接收显式warm-start向量；"
                "当前将warm-start信息折叠到批量multi-start的base seed中。"
            )
            self._warm_start_notice_sent = True

    def ping(self, require_current_runtime: bool = True) -> FpgaRuntimeStatus:
        payload, rtt_ms, transport = self.host.transact(CMD_PING, b"", 5.0)
        info = self.host._decode_ping_payload(payload)
        if require_current_runtime:
            self._validate_runtime_info(info)
        runtime = self._runtime_from_ping(info, rtt_ms, transport)
        self._last_runtime = runtime
        return runtime

    def load(self, problem: Problem) -> FpgaRuntimeStatus:
        """Prime the Active-N compact cache with a minimal one-start solve.

        The current firmware intentionally exposes UAV compact load as
        LOAD_START only.  Keeping this method on the compact path prevents old
        diagnostic callers from reaching the legacy dense CMD_LOAD_ISING path.
        """
        problem.validate()
        if self._last_runtime is None:
            self.ping(require_current_runtime=True)

        problem_crc, j_crc = uav_logical_problem_fingerprints(problem)
        payload = build_uav_compact_payload(problem)
        start_payload = self._start_payload(1, 1, self._next_seed_base(None))
        response, rtt_ms, transport = self.host.transact(
            CMD_UAV_LOAD_START, payload + start_payload, timeout=SOLVE_TIMEOUT_S
        )
        result = decode_result_payload(response, rtt_ms, transport, "UAV-COMPACT-LOAD")
        if result.node_count != N_HW or (result.status & 4) == 0:
            raise RuntimeError(
                f"FPGA配置失败: status=0x{result.status:08X}, nodes={result.node_count}"
            )
        runtime = self._last_runtime
        status = FpgaRuntimeStatus(
            status=result.status,
            h_writes=runtime.h_writes if runtime else 0,
            j_commits=runtime.j_commits if runtime else 0,
            build_id=runtime.build_id if runtime else REQUIRED_SW_BUILD_ID,
            version=runtime.version if runtime else REQUIRED_SW_VERSION,
            node_count=N_HW,
            serial_rtt_ms=rtt_ms,
            transport=transport,
        )
        self.host._fpga_uav_problem_crc = problem_crc
        self.host._fpga_uav_j_crc = j_crc
        self.host._fpga_problem_crc = None
        self.host._fpga_j_crc = None
        self._last_runtime = status
        return status

    def _next_seed_base(self, seed: int | None) -> int:
        if seed is not None:
            return int(seed) & 0xFFFFFFFF
        value = self._auto_seed & 0xFFFFFFFF
        self._auto_seed = (self._auto_seed + 0x85EBCA6B) & 0xFFFFFFFF
        return value

    @staticmethod
    def _mix_warm_seed(seed: int, warm_start: Sequence[int] | None) -> int:
        if not warm_start:
            return seed & 0xFFFFFFFF
        mixed = seed & 0xFFFFFFFF
        for index, value in enumerate(warm_start[:N_HW]):
            if int(value):
                mixed ^= ((index + 1) * 0x45D9F3B) & 0xFFFFFFFF
                mixed = ((mixed << 13) | (mixed >> 19)) & 0xFFFFFFFF
        return mixed & 0xFFFFFFFF

    def _start_payload(self, iterations: int, restarts: int, seed: int) -> bytes:
        if not 1 <= int(iterations) <= 65535:
            raise ValueError("iterations必须在1..65535")
        if not 1 <= int(restarts) <= 32:
            raise ValueError("restarts必须在1..32")
        flags = START_FLAG_ADAPTIVE_BUDGET if self.adaptive_budget else 0
        if self.early_restart_stop:
            flags |= START_FLAG_EARLY_RESTART_STOP
        return struct.pack(
            "<HBBI",
            int(iterations),
            int(restarts),
            flags,
            int(seed) & 0xFFFFFFFF,
        )

    def _solve_once(
        self,
        full_payload: bytes,
        h_payload: bytes,
        problem_crc: int,
        j_crc: int,
        iterations: int,
        seed: int,
    ) -> FpgaSolveResult:
        start_payload = self._start_payload(iterations, 1, seed)
        cached_problem = getattr(self.host, "_fpga_uav_problem_crc", None)
        cached_j = getattr(self.host, "_fpga_uav_j_crc", None)
        if cached_problem == problem_crc:
            command = CMD_START
            payload = start_payload
            load_mode = "UAV-CACHE"
        elif cached_j == j_crc:
            command = CMD_UAV_H_START
            if h_payload is None:
                raise RuntimeError("internal: H-cache selected without H payload")
            payload = h_payload + start_payload
            load_mode = "UAV-H-CACHE"
        else:
            command = CMD_UAV_LOAD_START
            payload = full_payload + start_payload
            load_mode = "UAV-COMPACT"

        tx_started = time.perf_counter()
        try:
            response, rtt_ms, transport = self.host.transact(
                command, payload, timeout=SOLVE_TIMEOUT_S
            )
        except RuntimeError as exc:
            if command in (CMD_START, CMD_UAV_H_START) and "问题尚未配置" in str(exc):
                self.host._invalidate_fpga_cache()
                response, rtt_ms, transport = self.host.transact(
                    CMD_UAV_LOAD_START, full_payload + start_payload, timeout=SOLVE_TIMEOUT_S
                )
                load_mode = "UAV-COMPACT-RETRY"
            else:
                raise
        self._last_tx_wall_ms = (time.perf_counter() - tx_started) * 1000.0

        decode_started = time.perf_counter()
        result = decode_result_payload(response, rtt_ms, transport, load_mode)
        self._last_decode_ms = (time.perf_counter() - decode_started) * 1000.0
        self._last_load_mode = load_mode
        if result.node_count != N_HW or (result.status & 4) == 0:
            raise RuntimeError(
                f"FPGA求解未完成: status=0x{result.status:08X}, nodes={result.node_count}"
            )
        self.host._fpga_uav_problem_crc = problem_crc
        self.host._fpga_uav_j_crc = j_crc
        self.host._fpga_problem_crc = None
        self.host._fpga_j_crc = None
        return result

    def _solve_batch_once(
        self,
        problem: Problem,
        full_payload: bytes | None,
        h_payload: bytes | None,
        problem_crc: int,
        j_crc: int,
        iterations: int,
        restarts: int,
        seed: int,
    ) -> FpgaSolveResult:
        """Run all requested restarts inside one firmware transaction.

        The current UAV firmware already implements multi-start internally and
        returns only the globally best spin vector.  The previous ROS adapter
        sent one UDP request *per restart*, which multiplied UDP/PS/result-read
        overhead and was the dominant reason a nominal 2--6 ms PL solve became
        a ~0.9 s end-to-end planning call at 16 restarts.

        Keeping restarts on the board also activates the firmware's adaptive
        restart budget and its "read spins only on new global best" fast path.
        """
        start_payload = self._start_payload(iterations, restarts, seed)
        cached_problem = getattr(self.host, "_fpga_uav_problem_crc", None)
        cached_j = getattr(self.host, "_fpga_uav_j_crc", None)
        if cached_problem == problem_crc:
            command = CMD_START
            payload = start_payload
            load_mode = "UAV-CACHE"
        elif cached_j == j_crc:
            command = CMD_UAV_H_START
            if h_payload is None:
                raise RuntimeError("internal: H-cache selected without H payload")
            payload = h_payload + start_payload
            load_mode = "UAV-H-CACHE"
        else:
            command = CMD_UAV_LOAD_START
            if full_payload is None:
                full_payload = build_uav_compact_payload(problem)
            payload = full_payload + start_payload
            load_mode = "UAV-COMPACT"

        tx_started = time.perf_counter()
        try:
            response, rtt_ms, transport = self.host.transact(
                command, payload, timeout=SOLVE_TIMEOUT_S
            )
        except RuntimeError as exc:
            if command in (CMD_START, CMD_UAV_H_START) and "问题尚未配置" in str(exc):
                self.host._invalidate_fpga_cache()
                if full_payload is None:
                    full_payload = build_uav_compact_payload(problem)
                response, rtt_ms, transport = self.host.transact(
                    CMD_UAV_LOAD_START, full_payload + start_payload, timeout=SOLVE_TIMEOUT_S
                )
                load_mode = "UAV-COMPACT-RETRY"
            else:
                raise
        self._last_tx_wall_ms = (time.perf_counter() - tx_started) * 1000.0

        decode_started = time.perf_counter()
        result = decode_result_payload(response, rtt_ms, transport, load_mode)
        self._last_decode_ms = (time.perf_counter() - decode_started) * 1000.0
        self._last_load_mode = load_mode
        if result.node_count != N_HW or (result.status & 4) == 0:
            raise RuntimeError(
                f"FPGA求解未完成: status=0x{result.status:08X}, nodes={result.node_count}"
            )
        self.host._fpga_uav_problem_crc = problem_crc
        self.host._fpga_uav_j_crc = j_crc
        self.host._fpga_problem_crc = None
        self.host._fpga_j_crc = None
        return result

    def start(
        self,
        iterations: int,
        seed: int | None = None,
        warm_start: Sequence[int] | None = None,
    ) -> FpgaSolveResult:
        if getattr(self.host, "_fpga_uav_problem_crc", None) is None:
            raise RuntimeError("当前没有已缓存的 Active-N compact 问题，请先调用 solve_runs/load。")
        run_seed = self._mix_warm_seed(self._next_seed_base(seed), warm_start)
        payload = self._start_payload(iterations, 1, run_seed)
        response, rtt_ms, transport = self.host.transact(
            CMD_START, payload, timeout=SOLVE_TIMEOUT_S
        )
        result = decode_result_payload(response, rtt_ms, transport, "UAV-CACHE")
        if result.node_count != N_HW or (result.status & 4) == 0:
            raise RuntimeError(
                f"FPGA求解未完成: status=0x{result.status:08X}, nodes={result.node_count}"
            )
        return result

    def solve_runs(
        self,
        problem: Problem,
        iterations: int,
        restarts: int = 1,
        seed: int | None = None,
        warm_starts: Sequence[Sequence[int]] = (),
    ) -> FpgaBatchResult:
        if restarts < 1:
            raise ValueError("restarts必须为正整数")
        if restarts > 32:
            raise ValueError("restarts不能超过32")
        if self._last_runtime is None:
            self.ping(require_current_runtime=True)

        if warm_starts and not self._warm_start_notice_sent:
            self._log(
                "UAV提示：256运行时START协议不接收显式warm-start向量；"
                "当前将warm-start信息折叠到批量multi-start的base seed中。"
            )
            self._warm_start_notice_sent = True

        host_started = time.perf_counter()
        problem.validate()
        payload_started = time.perf_counter()
        fingerprint_started = time.perf_counter()
        problem_crc, j_crc = uav_logical_problem_fingerprints(problem)
        fingerprint_ms = (time.perf_counter() - fingerprint_started) * 1000.0
        cached_problem = getattr(self.host, "_fpga_uav_problem_crc", None)
        cached_j = getattr(self.host, "_fpga_uav_j_crc", None)
        full_payload = None
        h_payload = None
        payload_build_mode = "CACHE"
        payload_prepared_bytes = 0
        if cached_problem == problem_crc:
            payload_build_mode = "CACHE"
        elif cached_j == j_crc:
            payload_build_mode = "H"
            h_payload = build_uav_compact_h_payload(problem)
            payload_prepared_bytes = len(h_payload)
        else:
            payload_build_mode = "FULL"
            full_payload = build_uav_compact_payload(problem)
            payload_prepared_bytes = len(full_payload)
        payload_ms = (time.perf_counter() - payload_started) * 1000.0

        if full_payload is not None:
            info = uav_compact_payload_info(full_payload)
            signature = (
                int(info.get("active_n") or 0),
                str(info.get("format") or ""),
                int(info.get("bytes") or 0),
                int(info.get("shift") or 0),
            )
            self.host._fpga_uav_payload_bytes = int(signature[2])
            if signature != self._last_payload_signature:
                self._log(
                    "UAV FPGA Active-N payload "
                    f"n={signature[0]} format={signature[1]} "
                    f"bytes={signature[2]} shift={signature[3]}"
                )
                self._last_payload_signature = signature
        reported_payload_bytes = int(
            len(full_payload) if full_payload is not None
            else getattr(self.host, "_fpga_uav_payload_bytes", payload_prepared_bytes) or payload_prepared_bytes
        )
        # Firmware performs all restarts in one request and returns the global
        # best result.  This mirrors the final standalone UAV implementation
        # and removes N-1 avoidable UDP round trips.
        base_seed = self._next_seed_base(seed)
        if warm_starts:
            base_seed = self._mix_warm_seed(base_seed, warm_starts[0])
        batch_started = time.perf_counter()
        result = self._solve_batch_once(
            problem,
            full_payload,
            h_payload,
            problem_crc,
            j_crc,
            int(iterations),
            int(restarts),
            base_seed,
        )
        batch_ms = (time.perf_counter() - batch_started) * 1000.0
        load_rtt = _problem_tx_rtt_ms(result.load_mode, result.serial_rtt_ms)
        runtime = self._last_runtime
        load_status = FpgaRuntimeStatus(
            status=result.status,
            h_writes=runtime.h_writes if runtime else 0,
            j_commits=runtime.j_commits if runtime else 0,
            build_id=runtime.build_id if runtime else REQUIRED_SW_BUILD_ID,
            version=runtime.version if runtime else REQUIRED_SW_VERSION,
            node_count=N_HW,
            serial_rtt_ms=load_rtt,
            transport=result.transport,
        )
        self.last_host_profile = {
            "payload_ms": float(payload_ms),
            "fingerprint_ms": float(fingerprint_ms),
            "payload_build_mode": str(payload_build_mode),
            "payload_prepared_bytes": int(payload_prepared_bytes),
            "payload_cache_fastpath": bool(payload_build_mode in {"CACHE", "H"}),
            "batch_ms": float(batch_ms),
            "transport_wall_ms": float(self._last_tx_wall_ms),
            "decode_ms": float(self._last_decode_ms),
            "host_total_ms": float((time.perf_counter() - host_started) * 1000.0),
            "wire_rtt_ms": float(result.serial_rtt_ms),
            "load_mode": str(result.load_mode),
            "payload_bytes": int(reported_payload_bytes),
            "native_wire_payload": bool(getattr(problem, "_native_wire_payload", False)),
            "native_wire_payload_version": int(getattr(problem, "_native_wire_payload_version", 0) or 0),
        }
        return FpgaBatchResult(load_status, (result,))

    def abort(self) -> FpgaRuntimeStatus:
        payload, rtt_ms, transport = self.host.transact(CMD_ABORT, b"", 2.0)
        return decode_status_payload(payload, rtt_ms, transport)


FpgaSolver = FpgaUavSolver
