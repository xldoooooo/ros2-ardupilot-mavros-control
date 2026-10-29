"""Optional C++ fast path for latency-critical deterministic host kernels.

The native library is deliberately dependency-free (C++17 + C ABI) and loaded
with ctypes.  If it is absent or a native call fails, every caller falls back
to the existing Python/NumPy implementation.  This keeps the flight planner
functional before compilation and makes the accelerator a performance feature,
not a correctness dependency.
"""
from __future__ import annotations

import ctypes
import ctypes.util
import os
from pathlib import Path
from typing import Sequence

import numpy as np

_ENABLED = os.environ.get("FPGA_NATIVE_FASTPATH", "auto").strip().lower() not in {"0", "false", "off", "no"}
_LIB = None
_LOAD_ERROR = None
_PATH_CACHE: dict[int, np.ndarray] = {}
_REQUIRED_ABI = 3301


def set_enabled(value: bool) -> None:
    global _ENABLED
    _ENABLED = bool(value)


def _candidate_library_paths() -> list[Path]:
    here = Path(__file__).resolve().parent
    names = ["libfpga_fastpath_native.so"]
    out: list[Path] = []
    explicit = os.environ.get("FPGA_FASTPATH_LIB", "").strip()
    if explicit:
        out.append(Path(explicit).expanduser())
    for name in names:
        out.append(here / "_native" / name)
        out.append(here / name)
    for item in os.environ.get("LD_LIBRARY_PATH", "").split(":"):
        if item:
            out.append(Path(item) / names[0])
    # Source package commonly lives under <ws>/src/.../path_planning.  Search
    # ancestors for catkin devel/lib without assuming a fixed workspace name.
    for parent in here.parents:
        out.append(parent / "devel" / "lib" / names[0])
        out.append(parent / "install" / "lib" / names[0])
    found = ctypes.util.find_library("fpga_fastpath_native")
    if found:
        out.append(Path(found))
    # Stable de-duplication.
    seen = set()
    unique = []
    for p in out:
        key = str(p)
        if key not in seen:
            seen.add(key)
            unique.append(p)
    return unique


def _configure(lib) -> None:
    c_void_p = ctypes.c_void_p
    c_i = ctypes.c_int
    c_i64 = ctypes.c_int64
    c_d = ctypes.c_double
    lib.fpga_native_fastpath_version.argtypes = []
    lib.fpga_native_fastpath_version.restype = c_i
    lib.fpga_udp_recv_native.argtypes = [
        c_i, c_void_p, c_i, c_i, c_i, c_i, c_void_p, c_void_p, c_void_p,
    ]
    lib.fpga_udp_recv_native.restype = c_i
    lib.fpga_voxelize_inflate3d.argtypes = [
        c_void_p, c_i64, c_i, c_i, c_i,
        c_d, c_d, c_d, c_d, c_d, c_d, c_void_p, c_void_p,
    ]
    lib.fpga_voxelize_inflate3d.restype = c_i
    lib.fpga_clearance_limited3d.argtypes = [
        c_void_p, c_i, c_i, c_i, c_i, c_void_p,
    ]
    lib.fpga_clearance_limited3d.restype = c_i
    lib.fpga_chain_blocked_count_xyz3d.argtypes = [
        c_void_p, c_i, c_void_p, c_i, c_i, c_i,
        c_d, c_d, c_d, c_d, c_d, c_i,
    ]
    lib.fpga_chain_blocked_count_xyz3d.restype = c_i
    lib.fpga_candidates3d.argtypes = [
        c_void_p, c_i, c_void_p, c_void_p, c_i, c_i, c_i,
        c_d, c_d, c_d, c_d, c_d, c_i, c_i, c_d, c_void_p,
        c_void_p, c_void_p, c_void_p, c_void_p, c_void_p,
    ]
    lib.fpga_candidates3d.restype = c_i
    lib.fpga_astar26.argtypes = [
        c_void_p, c_void_p, c_i, c_i, c_i,
        c_i, c_i, c_i, c_i, c_i, c_i, c_void_p, c_i,
    ]
    lib.fpga_astar26.restype = c_i
    lib.fpga_blocked_pairs_output_size.argtypes = [c_void_p, c_i]
    lib.fpga_blocked_pairs_output_size.restype = c_i64
    lib.fpga_blocked_pairs3d.argtypes = [
        c_void_p, c_void_p, c_i, c_void_p, c_i, c_i, c_i, c_i, c_void_p, c_i64,
    ]
    lib.fpga_blocked_pairs3d.restype = c_i64
    lib.fpga_segments_blocked_xyz3d.argtypes = [
        c_void_p, c_void_p, c_i, c_void_p, c_i, c_i, c_i,
        c_d, c_d, c_d, c_d, c_d, c_i, c_void_p,
    ]
    lib.fpga_segments_blocked_xyz3d.restype = c_i
    lib.fpga_segments_blocked_cells3d.argtypes = [
        c_void_p, c_void_p, c_i, c_void_p, c_i, c_i, c_i, c_void_p,
    ]
    lib.fpga_segments_blocked_cells3d.restype = c_i
    lib.fpga_dense_trajectory_output_size.argtypes = [c_i, c_d, c_d]
    lib.fpga_dense_trajectory_output_size.restype = c_i64
    lib.fpga_dense_trajectory3d.argtypes = [
        c_void_p, c_void_p, c_i, c_d, c_d, c_d, c_i,
        c_void_p, c_void_p, c_void_p, c_i64, c_void_p,
    ]
    lib.fpga_dense_trajectory3d.restype = c_i
    lib.fpga_uav_compact_payload_active.argtypes = [
        c_void_p, c_void_p, c_i, c_i, c_void_p, c_i,
    ]
    lib.fpga_uav_compact_payload_active.restype = c_i
    lib.fpga_kino_qubo_active.argtypes = [
        c_void_p, c_i, c_void_p, c_void_p, c_void_p, c_i,
        c_void_p, c_void_p, c_void_p,
        c_void_p, c_void_p, c_void_p, c_void_p,
        c_d, c_d, c_d, c_d, c_d, c_d, c_d, c_d, c_d, c_d, c_d,
        c_void_p, c_void_p, c_void_p, c_void_p, c_void_p, c_void_p,
    ]
    lib.fpga_kino_qubo_active.restype = c_i


def _load():
    global _LIB, _LOAD_ERROR
    if _LIB is not None:
        return _LIB
    if _LOAD_ERROR is not None:
        return None
    errors = []
    for path in _candidate_library_paths():
        try:
            if path.is_absolute() and not path.exists():
                continue
            lib = ctypes.CDLL(str(path))
            _configure(lib)
            if int(lib.fpga_native_fastpath_version()) < _REQUIRED_ABI:
                raise RuntimeError(f"native fastpath ABI is too old; require >= {_REQUIRED_ABI}")
            _LIB = lib
            return _LIB
        except Exception as exc:  # noqa: BLE001
            errors.append(f"{path}: {exc}")
    _LOAD_ERROR = "; ".join(errors) if errors else "libfpga_fastpath_native.so not found"
    return None


def available() -> bool:
    return bool(_ENABLED and _load() is not None)


def status() -> dict[str, object]:
    lib = _load() if _ENABLED else None
    return {
        "enabled": bool(_ENABLED),
        "available": bool(lib is not None),
        "version": int(lib.fpga_native_fastpath_version()) if lib is not None else 0,
        "error": _LOAD_ERROR,
    }


def _ptr(array: np.ndarray | None):
    """Return the raw data pointer without exporting a Python buffer.

    NumPy's ``ndarray.ctypes`` path can exercise the legacy buffer protocol.
    On some ROS Noetic / Python 3.8 NumPy builds, repeated short-lived buffer
    exports may leave incompatible buffer bookkeeping behind and later fail in
    ``PyDataMem_UserFREE``.  The native library only needs an address and all
    callers already pass contiguous arrays, so use the public array-interface
    address directly and avoid creating a buffer export entirely.
    """
    if array is None:
        return None
    address = int(array.__array_interface__["data"][0])
    return ctypes.c_void_p(address)


def voxelize_inflate3d(points_xyz, frame, radius_m: float):
    lib = _load() if _ENABLED else None
    if lib is None:
        return None
    points = np.ascontiguousarray(points_xyz, dtype=np.float64)
    if points.ndim != 2 or points.shape[1] != 3:
        return None
    raw = np.zeros((frame.layers, frame.rows, frame.cols), dtype=np.uint8)
    inflated = np.zeros_like(raw)
    rc = lib.fpga_voxelize_inflate3d(
        _ptr(points), int(points.shape[0]),
        int(frame.layers), int(frame.rows), int(frame.cols),
        float(frame.x_min), float(frame.y_min), float(frame.z_min),
        float(frame.resolution_m), float(frame.resolution_z_m), float(radius_m),
        _ptr(raw), _ptr(inflated),
    )
    if rc != 0:
        return None
    return raw, inflated



def clearance_limited3d(occupancy: np.ndarray, max_distance_voxels: float):
    """Exact C++ capped 6-neighbour clearance; None => Python fallback."""
    lib = _load() if _ENABLED else None
    if lib is None or not hasattr(lib, "fpga_clearance_limited3d"):
        return None
    occ = np.ascontiguousarray(occupancy, dtype=np.uint8)
    if occ.ndim != 3:
        return None
    cap = max(1, int(np.ceil(max(0.0, float(max_distance_voxels)))))
    out = np.empty(occ.shape, dtype=np.float32)
    rc = int(lib.fpga_clearance_limited3d(_ptr(occ), *map(int, occ.shape), cap, _ptr(out)))
    return out if rc == 0 else None


def chain_blocked_count_xyz3d(points_xyz, occupancy: np.ndarray, frame, samples: int = 8):
    """Exact C++ equivalent of the planner's fixed-sample dense-chain guard."""
    lib = _load() if _ENABLED else None
    if lib is None or not hasattr(lib, "fpga_chain_blocked_count_xyz3d"):
        return None
    points = np.ascontiguousarray(points_xyz, dtype=np.float64).reshape(-1, 3)
    if points.shape[0] < 2:
        return 0
    occ = np.ascontiguousarray(occupancy, dtype=np.uint8)
    if occ.ndim != 3:
        return None
    rc = int(lib.fpga_chain_blocked_count_xyz3d(
        _ptr(points), int(points.shape[0]), _ptr(occ), *map(int, occ.shape),
        float(frame.x_min), float(frame.y_min), float(frame.z_min),
        float(frame.resolution_m), float(frame.resolution_z_m), max(2, int(samples)),
    ))
    return None if rc < 0 else rc


def candidate_layers3d(
    spine_points_xyz,
    occupancy: np.ndarray,
    clearance: np.ndarray,
    frame,
    *,
    radius: int,
    max_count: int,
    safe_clearance_voxels: float,
    goal_bias=None,
):
    """Bulk C++ candidate enumeration for all free waypoint layers.

    Returns ``(positions, cells, obstacle_costs, reference_costs)`` or ``None``
    when ABI<2501/native is unavailable.  Candidate order intentionally matches
    the NumPy reference implementation: baseline spine voxel first, then lexicographic
    ranking by distance-to-spine, capped-clearance(desc), goal projection(desc).
    """
    lib = _load() if _ENABLED else None
    if lib is None or int(lib.fpga_native_fastpath_version()) < _REQUIRED_ABI:
        return None
    spine = np.ascontiguousarray(spine_points_xyz, dtype=np.float64).reshape(-1, 3)
    if spine.shape[0] <= 0:
        return ([], [], [], [])
    occ = np.ascontiguousarray(occupancy, dtype=np.uint8)
    clr = np.ascontiguousarray(clearance, dtype=np.float32)
    if occ.ndim != 3 or clr.shape != occ.shape:
        return None
    radius = max(1, int(radius))
    max_count = max(1, int(max_count))
    bias = None if goal_bias is None else np.ascontiguousarray(goal_bias, dtype=np.float64).reshape(3)
    counts = np.zeros(spine.shape[0], dtype=np.int32)
    cells = np.empty((spine.shape[0], max_count, 3), dtype=np.int32)
    positions = np.empty((spine.shape[0], max_count, 3), dtype=np.float64)
    obstacle = np.empty((spine.shape[0], max_count), dtype=np.float64)
    reference = np.empty((spine.shape[0], max_count), dtype=np.float64)
    rc = int(lib.fpga_candidates3d(
        _ptr(spine), int(spine.shape[0]), _ptr(occ), _ptr(clr), *map(int, occ.shape),
        float(frame.x_min), float(frame.y_min), float(frame.z_min),
        float(frame.resolution_m), float(frame.resolution_z_m),
        radius, max_count, float(safe_clearance_voxels), _ptr(bias),
        _ptr(counts), _ptr(cells), _ptr(positions), _ptr(obstacle), _ptr(reference),
    ))
    if rc != spine.shape[0]:
        return None
    out_positions = []
    out_cells = []
    out_obstacle = []
    out_reference = []
    for layer, count in enumerate(counts.tolist()):
        count = max(0, min(int(count), max_count))
        out_cells.append([
            (int(v[0]), int(v[1]), int(v[2])) for v in cells[layer, :count]
        ])
        out_positions.append([
            (float(v[0]), float(v[1]), float(v[2])) for v in positions[layer, :count]
        ])
        out_obstacle.append([float(v) for v in obstacle[layer, :count]])
        out_reference.append([float(v) for v in reference[layer, :count]])
    return out_positions, out_cells, out_obstacle, out_reference


def astar26(occupancy: np.ndarray, start, goal, bias: np.ndarray | None = None):
    lib = _load() if _ENABLED else None
    if lib is None:
        return None
    occ = np.ascontiguousarray(occupancy, dtype=np.uint8)
    if occ.ndim != 3:
        return None
    bias_arr = None if bias is None else np.ascontiguousarray(bias, dtype=np.float64)
    if bias_arr is not None and bias_arr.shape != occ.shape:
        return None
    cap = int(occ.size)
    buf = _PATH_CACHE.get(cap)
    if buf is None:
        buf = np.empty((cap, 3), dtype=np.int32)
        if len(_PATH_CACHE) > 8:
            _PATH_CACHE.clear()
        _PATH_CACHE[cap] = buf
    n = lib.fpga_astar26(
        _ptr(occ), _ptr(bias_arr), *map(int, occ.shape),
        int(start[0]), int(start[1]), int(start[2]),
        int(goal[0]), int(goal[1]), int(goal[2]),
        _ptr(buf), cap,
    )
    if n < 0:
        return None
    if n == 0:
        return ()
    return tuple((int(z), int(r), int(c)) for z, r, c in buf[:n])


def blocked_pairs3d(candidate_cells: Sequence[Sequence[Sequence[int]]], occupancy: np.ndarray, samples: int = 8):
    lib = _load() if _ENABLED else None
    if lib is None or len(candidate_cells) < 2:
        return None
    counts = [len(layer) for layer in candidate_cells]
    offsets = np.zeros(len(counts) + 1, dtype=np.int32)
    offsets[1:] = np.cumsum(counts, dtype=np.int32)
    if int(offsets[-1]) <= 0:
        return None
    flat = np.ascontiguousarray(
        np.asarray([cell for layer in candidate_cells for cell in layer], dtype=np.int32).reshape(-1, 3)
    )
    occ = np.ascontiguousarray(occupancy, dtype=np.uint8)
    required = int(lib.fpga_blocked_pairs_output_size(_ptr(offsets), len(counts)))
    if required < 0:
        return None
    out = np.empty(required, dtype=np.uint8)
    rc = int(lib.fpga_blocked_pairs3d(
        _ptr(flat), _ptr(offsets), len(counts), _ptr(occ), *map(int, occ.shape),
        max(2, int(samples)), _ptr(out), required,
    ))
    if rc != required:
        return None
    result = []
    pos = 0
    for m, n in zip(counts[:-1], counts[1:]):
        block = out[pos:pos + m*n].reshape(m, n)
        result.append([[bool(v) for v in row] for row in block])
        pos += m*n
    return result


def segments_blocked_cells3d(a_zyx, b_zyx, occupancy: np.ndarray):
    """Exact ABI-3101 batch equivalent of ``_segment_clear_cells``.

    Returns a list where True means the integer-cell segment intersects an
    occupied/out-of-bounds voxel.  ``None`` keeps the Python reference path.
    """
    lib = _load() if _ENABLED else None
    if lib is None or int(lib.fpga_native_fastpath_version()) < 3101 or not hasattr(lib, "fpga_segments_blocked_cells3d"):
        return None
    a = np.ascontiguousarray(a_zyx, dtype=np.int32).reshape(-1, 3)
    b = np.ascontiguousarray(b_zyx, dtype=np.int32).reshape(-1, 3)
    if a.shape != b.shape:
        return None
    occ = np.ascontiguousarray(occupancy, dtype=np.uint8)
    if occ.ndim != 3:
        return None
    out = np.empty(a.shape[0], dtype=np.uint8)
    rc = int(lib.fpga_segments_blocked_cells3d(
        _ptr(a), _ptr(b), int(a.shape[0]), _ptr(occ), *map(int, occ.shape), _ptr(out),
    ))
    if rc != a.shape[0]:
        return None
    return [bool(v) for v in out]


def dense_trajectory3d(points_xyz, velocities_xyz, dt: float, dt_sample: float, max_vel: float, *, linear_only: bool = False):
    """ABI-3201 exact dense Hermite/linear trajectory generator.

    Returns ``(dense, dense_vel, dense_acc, max_step_speed)`` as NumPy arrays,
    or ``None`` to preserve the Python reference path.
    """
    lib = _load() if _ENABLED else None
    if lib is None or int(lib.fpga_native_fastpath_version()) < 3201 or not hasattr(lib, "fpga_dense_trajectory3d"):
        return None
    points = np.ascontiguousarray(np.asarray(points_xyz, dtype=np.float64).reshape(-1, 3))
    vel = np.ascontiguousarray(np.asarray(velocities_xyz, dtype=np.float64).reshape(-1, 3))
    if points.shape != vel.shape or points.shape[0] <= 0:
        return None
    count = int(lib.fpga_dense_trajectory_output_size(int(points.shape[0]), float(dt), float(dt_sample)))
    if count <= 0 or count > 5_000_000:
        return None
    dense = np.empty((count, 3), dtype=np.float64)
    dense_vel = np.empty((count, 3), dtype=np.float64)
    dense_acc = np.empty((count, 3), dtype=np.float64)
    vmax = ctypes.c_double(0.0)
    rc = int(lib.fpga_dense_trajectory3d(
        _ptr(points), _ptr(vel), int(points.shape[0]), float(dt), float(dt_sample), float(max_vel), int(bool(linear_only)),
        _ptr(dense), _ptr(dense_vel), _ptr(dense_acc), int(count), ctypes.byref(vmax),
    ))
    if rc != count:
        return None
    return dense, dense_vel, dense_acc, float(vmax.value)


def segments_blocked_xyz3d(a_xyz, b_xyz, occupancy: np.ndarray, frame, samples: int = 8):
    lib = _load() if _ENABLED else None
    if lib is None:
        return None
    a = np.ascontiguousarray(a_xyz, dtype=np.float64).reshape(-1, 3)
    b = np.ascontiguousarray(b_xyz, dtype=np.float64).reshape(-1, 3)
    if a.shape != b.shape:
        return None
    occ = np.ascontiguousarray(occupancy, dtype=np.uint8)
    out = np.empty(a.shape[0], dtype=np.uint8)
    rc = int(lib.fpga_segments_blocked_xyz3d(
        _ptr(a), _ptr(b), int(a.shape[0]), _ptr(occ), *map(int, occ.shape),
        float(frame.x_min), float(frame.y_min), float(frame.z_min),
        float(frame.resolution_m), float(frame.resolution_z_m), max(2, int(samples)), _ptr(out),
    ))
    if rc != a.shape[0]:
        return None
    return [bool(v) for v in out]


def udp_recv_native(fd: int, capacity: int, spin_us: int, poll_us: int, deadline_us: int):
    """Receive one UDP datagram entirely in native C++.

    Returns ``(data, phase, elapsed_us, spin_elapsed_us)`` where phase is
    ``"spin"`` or ``"blocking"``.  Returns ``None`` on timeout/unavailable so
    callers can fall back to the proven Python receive path.
    """
    lib = _load() if _ENABLED else None
    if lib is None or int(lib.fpga_native_fastpath_version()) < _REQUIRED_ABI:
        return None
    capacity = max(256, min(int(capacity), 65536))
    buf = ctypes.create_string_buffer(capacity)
    phase = ctypes.c_int(0)
    elapsed = ctypes.c_int(0)
    spin_elapsed = ctypes.c_int(0)
    rc = int(lib.fpga_udp_recv_native(
        int(fd), ctypes.cast(buf, ctypes.c_void_p), capacity,
        max(0, int(spin_us)), max(50, int(poll_us)), max(1, int(deadline_us)),
        ctypes.byref(phase), ctypes.byref(elapsed), ctypes.byref(spin_elapsed),
    ))
    if rc == 0:
        return None
    if rc < 0:
        raise OSError(-rc, f"native UDP receive failed rc={rc}")
    return (
        bytes(buf.raw[:rc]),
        "spin" if phase.value == 1 else "blocking",
        int(elapsed.value),
        int(spin_elapsed.value),
    )


def uav_compact_payload_active(h, j, n: int, *, h_only: bool = False):
    """Build the existing Active-N UAV wire payload in C++ (ABI3001).

    Returns bytes identical to the Python payload builder, or None so callers
    can fall back without changing the FPGA protocol.
    """
    lib = _load() if _ENABLED else None
    if lib is None or int(lib.fpga_native_fastpath_version()) < 3001 or not hasattr(lib, "fpga_uav_compact_payload_active"):
        return None
    n = int(n)
    if n < 2 or n > 256:
        return None
    ha = np.ascontiguousarray(np.asarray(h, dtype=np.float64).reshape(-1)[:n])
    ja = np.ascontiguousarray(np.asarray(j, dtype=np.float64)[:n, :n])
    if ha.size != n or ja.shape != (n, n):
        return None
    upper = n * (n - 1) // 2
    cap = 4 + 4 * n if h_only else 4 + 4 * n + 4 * upper
    out = np.empty(cap, dtype=np.uint8)
    rc = int(lib.fpga_uav_compact_payload_active(
        _ptr(ha), _ptr(ja), n, int(bool(h_only)), _ptr(out), int(out.size)
    ))
    if rc < 0 or rc > out.size:
        return None
    return out[:rc].tobytes()


def kino_qubo_active(
    *,
    fixed_positions,
    candidate_positions,
    candidate_obstacle_costs,
    candidate_reference_costs,
    candidate_blocked_pairs,
    candidate_endpoint_blocked,
    dt: float,
    w_vel: float,
    w_acc: float,
    w_obs: float,
    w_spine: float,
    w_len: float,
    w_start_vel: float,
    w_start_acc: float,
    w_end_vel: float,
    segment_penalty: float,
    start_vel_xyz,
    start_acc_xyz,
    a_mult: float,
):
    """Build the identical active-N kinodynamic QUBO in C++.

    Returns a dict containing active QUBO/Ising arrays and scaling metadata, or
    ``None`` when ABI 3001 is unavailable.  The caller retains the Python
    incremental builder as a correctness fallback.
    """
    lib = _load() if _ENABLED else None
    if lib is None or int(lib.fpga_native_fastpath_version()) < 3001 or not hasattr(lib, 'fpga_kino_qubo_active'):
        return None
    free_layers = len(candidate_positions)
    if free_layers < 1:
        return None
    counts = np.asarray([len(layer) for layer in candidate_positions], dtype=np.int32)
    offsets = np.zeros(free_layers + 1, dtype=np.int32)
    offsets[1:] = np.cumsum(counts, dtype=np.int32)
    n = int(offsets[-1])
    if n <= 0 or n > 256:
        return None
    candidates = np.ascontiguousarray(
        np.asarray([p for layer in candidate_positions for p in layer], dtype=np.float64).reshape(n, 3)
    )
    obstacle = np.ascontiguousarray(
        np.asarray([v for layer in candidate_obstacle_costs for v in layer], dtype=np.float64).reshape(n)
    )
    has_reference = candidate_reference_costs is not None and float(w_spine) > 0.0
    reference = None
    if has_reference:
        reference = np.ascontiguousarray(
            np.asarray([v for layer in candidate_reference_costs for v in layer], dtype=np.float64).reshape(n)
        )
    pair_total = int(sum(int(counts[i]) * int(counts[i+1]) for i in range(max(0, free_layers - 1))))
    blocked = np.zeros(pair_total, dtype=np.uint8)
    pos = 0
    for li in range(free_layers - 1):
        a, b = int(counts[li]), int(counts[li+1])
        block = np.asarray(candidate_blocked_pairs[li], dtype=np.uint8)[:a, :b].reshape(-1)
        blocked[pos:pos + block.size] = block
        pos += block.size
    first = np.zeros(int(counts[0]), dtype=np.uint8)
    last = np.zeros(int(counts[-1]), dtype=np.uint8)
    if candidate_endpoint_blocked is not None:
        f, l = candidate_endpoint_blocked
        first[:min(first.size, len(f))] = np.asarray(f[:first.size], dtype=np.uint8)
        last[:min(last.size, len(l))] = np.asarray(l[:last.size], dtype=np.uint8)
    start = np.ascontiguousarray(fixed_positions[0], dtype=np.float64).reshape(3)
    goal = np.ascontiguousarray(fixed_positions[-1], dtype=np.float64).reshape(3)
    start_vel = np.ascontiguousarray(start_vel_xyz, dtype=np.float64).reshape(3)
    start_acc = np.ascontiguousarray(start_acc_xyz, dtype=np.float64).reshape(3)
    linear = np.empty(n, dtype=np.float64)
    matrix = np.empty((n, n), dtype=np.float64)
    h = np.empty(n, dtype=np.float64)
    j = np.empty((n, n), dtype=np.float64)
    info = np.empty(4, dtype=np.float64)
    hashes = np.empty(2, dtype=np.uint64)
    rc = int(lib.fpga_kino_qubo_active(
        _ptr(offsets), free_layers, _ptr(candidates), _ptr(obstacle), _ptr(reference), int(bool(has_reference)),
        _ptr(blocked), _ptr(first), _ptr(last), _ptr(start), _ptr(goal), _ptr(start_vel), _ptr(start_acc),
        float(dt), float(w_vel), float(w_acc), float(w_obs), float(w_spine), float(w_len),
        float(w_start_vel), float(w_start_acc), float(w_end_vel), float(segment_penalty), float(a_mult),
        _ptr(linear), _ptr(matrix), _ptr(h), _ptr(j), _ptr(info), _ptr(hashes),
    ))
    if rc != n:
        return None
    return {
        'n': n,
        'linear': linear,
        'matrix': matrix,
        'h': h,
        'j': j,
        'objective_scale': float(info[0]),
        'one_hot_penalty': float(info[1]),
        'segment_penalty_used': float(info[2]),
        'bfp_rescale': float(info[3]),
        'native_abi': int(lib.fpga_native_fastpath_version()),
        'logical_problem_fingerprints': (int(hashes[0]), int(hashes[1])),
    }
