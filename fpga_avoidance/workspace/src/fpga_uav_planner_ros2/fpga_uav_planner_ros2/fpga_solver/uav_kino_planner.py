"""Kinodynamic waypoint planning modelled as a layer-wise one-hot QUBO.

The original WSL kinodynamic A* (path_searching/kinodynamic_astar.cpp) searches
in the continuous 6-D state (position + velocity) under the double-integrator
model

    p' = p + v*tau + 0.5*u*tau^2,   v' = v + u*tau,    u = acceleration input,

and minimises  (|u|^2 + w_time)*tau  subject to |v| <= v_max, |u| <= a_max,
safety against the point cloud and altitude bounds.  This module maps the same
kinematic objective onto the 256-spin Ising/QUBO machine:

    x[i][j] = 1  <=>  waypoint layer i is placed at candidate position q[i][j]
    per-layer one-hot:  sum_j x[i][j] = 1

With a *fixed* time step dt between layers the path energy

    E = sum_i w_len*|p[i+1]-p[i]|                 (geometric path length)
      + sum_i w_vel*|p[i+1]-p[i]|^2/dt^2          (velocity energy)
      + sum_i w_acc*|p[i+2]-2p[i+1]+p[i]|^2/dt^4  (acceleration energy)
      + obstacle / segment-safety / boundary costs

is exactly quadratic in the binary variables (squared distances expand into
per-layer linear terms plus pairwise dot products), so the FPGA annealer can
directly trade obstacle clearance against smoothness and acceleration, i.e.
the combinatorial-optimisation part that the Ising machine is good at, instead
of just picking among pre-built whole paths.

Input/output align with the original dyn_obs_planner A* node:
  in   : start (pos + vel + acc), goal (pos + vel), point-cloud occupancy,
         altitude bounds, max velocity / acceleration
  out  : dense 0.01 s position samples (nav_msgs/Path style), plus waypoints
         with per-waypoint velocity/acceleration and path_size = len+1.
"""

from __future__ import annotations

import heapq
import hashlib
import binascii
import math
import time
from collections import OrderedDict, deque
from dataclasses import dataclass, field, replace
from functools import lru_cache
from typing import Callable, Optional, Sequence, Tuple

import numpy as np

from dyn_ising_adapter import GridFrame3D, cell3d_to_xyz, xyz_to_cell3d
from fpga_uav_solver import FpgaSolver
from executable_trajectory import (
    executable_segment_limits, executable_segments_clear, hermite_coefficients,
    sample_executable_segment, motion_durations, clamped_velocities, reference_durations,
)
from qubo_model import N_HW, SCALE, Problem, make_qubo, make_qubo_from_terms, qubo_value

try:
    import native_fastpath as _native_fastpath
except Exception:  # pragma: no cover - optional accelerator
    _native_fastpath = None

Cell3D = Tuple[int, int, int]  # z, row, col
Point3 = Tuple[float, float, float]


def _norm_rows_xyz(values: np.ndarray) -> np.ndarray:
    """Euclidean row norm without numpy.linalg private-module aliases."""
    arr = np.asarray(values, dtype=np.float64)
    return np.sqrt(np.sum(arr * arr, axis=-1))


def _pairwise_xyz_distance(left: np.ndarray, right: np.ndarray) -> np.ndarray:
    """Pairwise Euclidean distance, algebraically identical to linalg.norm."""
    delta = np.asarray(left, dtype=np.float64)[:, None, :] - np.asarray(right, dtype=np.float64)[None, :, :]
    return np.sqrt(np.sum(delta * delta, axis=2))


def validate_grid3d(grid: np.ndarray, *, trusted_binary: bool = False) -> np.ndarray:
    """Normalise a 3D occupancy grid to uint8 {0,1}.

    The runtime uses a no-copy fast path for grids produced internally by
    ``problem3d_from_request``.  That adapter already returns C-contiguous
    uint8 binary occupancy from the native voxeliser/inflater.  Re-running
    ``(array != 0).astype(uint8)`` on every rolling replan was therefore a full
    extra map scan and allocation.  External callers keep the original strict
    normalisation by default.
    """
    array = np.asarray(grid)
    if array.ndim != 3 or array.size == 0:
        raise ValueError("3D map must be a non-empty [z,row,col] array")
    if bool(trusted_binary) and array.dtype == np.uint8 and array.flags.c_contiguous:
        return array
    return (array != 0).astype(np.uint8)


@dataclass(frozen=True)
class KinoPlanResult:
    start_zyx: Cell3D
    goal_zyx: Cell3D
    waypoints_xyz: tuple[Point3, ...]
    waypoint_velocities_xyz: tuple[Point3, ...]
    waypoint_accelerations_xyz: tuple[Point3, ...]
    path_xyz: tuple[Point3, ...]  # dense 0.01 s samples
    velocities_xyz: tuple[Point3, ...]
    accelerations_xyz: tuple[Point3, ...]
    path_size: int  # len(path_xyz) + 1, aligned with original A* node
    dt_s: float
    dt_sample_s: float
    total_cost: float
    source: str  # "fpga" | "deterministic" | "direct"
    onehot_valid: bool
    variables: int
    candidate_counts: tuple[int, ...]
    planning_elapsed_ms: float
    fpga_elapsed_ms: float
    serial_rtt_ms: float
    load_rtt_ms: float
    fpga_starts: int
    valid_starts: int
    max_vel_mps: float
    max_acc_mps2: float
    vel_limit_mps: float
    acc_limit_mps2: float
    vel_ok: bool
    acc_ok: bool
    attempts: int = 1
    details: dict[str, object] = field(default_factory=dict)
    # Exact execution coefficients: segment, XYZ axis, ascending local-second power.
    polynomial_coefficients_xyz: tuple = ()
    # Each cubic uses its own local clock; legacy sampled-only plans leave this empty.
    segment_durations_s: tuple[float, ...] = ()
    # Local map-window endpoints must not complete the business waypoint.
    reaches_goal: bool = True


def _clearance_zyx(occupancy: np.ndarray) -> np.ndarray:
    """Distance (voxels) from every voxel to the nearest occupied voxel."""
    layers, rows, cols = occupancy.shape
    occupied = np.argwhere(occupancy != 0)
    dist = np.full((layers, rows, cols), float("inf"), dtype=np.float64)
    if occupied.size == 0:
        return np.full((layers, rows, cols), float(layers + rows + cols))
    queue: deque[tuple[int, int, int]] = deque()
    for (z, r, c) in occupied:
        dist[z, r, c] = 0.0
        queue.append((int(z), int(r), int(c)))
    moves = (
        (1, 0, 0), (-1, 0, 0),
        (0, 1, 0), (0, -1, 0),
        (0, 0, 1), (0, 0, -1),
    )
    while queue:
        z, r, c = queue.popleft()
        d = dist[z, r, c] + 1.0
        for dz, dr, dc in moves:
            nz, nr, nc = z + dz, r + dr, c + dc
            if 0 <= nz < layers and 0 <= nr < rows and 0 <= nc < cols and d < dist[nz, nr, nc]:
                dist[nz, nr, nc] = d
                queue.append((nz, nr, nc))
    return dist


def _clearance_zyx_fast(occupancy: np.ndarray) -> np.ndarray:
    """Numpy frontier-BFS distance to the nearest occupied voxel (faster)."""
    layers, rows, cols = occupancy.shape
    dist = np.full((layers, rows, cols), np.iinfo(np.int32).max, dtype=np.int32)
    occ = np.asarray(occupancy != 0)
    dist[occ] = 0
    frontier = np.argwhere(occ)
    if frontier.size == 0:
        return np.full((layers, rows, cols), float(layers + rows + cols), dtype=np.float64)
    distance = 0
    deltas = (
        np.array((1, 0, 0)), np.array((-1, 0, 0)),
        np.array((0, 1, 0)), np.array((0, -1, 0)),
        np.array((0, 0, 1)), np.array((0, 0, -1)),
    )
    while frontier.size:
        distance += 1
        next_frontier: list[np.ndarray] = []
        for delta in deltas:
            candidate = frontier + delta
            # keep only in-bounds
            inside = (
                (candidate[:, 0] >= 0) & (candidate[:, 0] < layers)
                & (candidate[:, 1] >= 0) & (candidate[:, 1] < rows)
                & (candidate[:, 2] >= 0) & (candidate[:, 2] < cols)
            )
            cand = candidate[inside]
            if cand.size == 0:
                continue
            idx = (cand[:, 0], cand[:, 1], cand[:, 2])
            unvisited = dist[idx] == np.iinfo(np.int32).max
            if unvisited.any():
                dist[idx[0][unvisited], idx[1][unvisited], idx[2][unvisited]] = distance
                next_frontier.append(cand[unvisited])
        if not next_frontier:
            break
        frontier = np.concatenate(next_frontier, axis=0)
    return dist.astype(np.float64)


def _clearance_zyx_limited(occupancy: np.ndarray, max_distance_voxels: float) -> np.ndarray:
    """Fast capped 6-neighbour clearance for online replanning.

    Prefer the exact dependency-free C++ kernel when available; the NumPy
    implementation below remains the correctness fallback. Both implement the
    same capped Manhattan-distance shells.
    """
    if _native_fastpath is not None:
        try:
            native = _native_fastpath.clearance_limited3d(occupancy, max_distance_voxels)
        except Exception:
            native = None
        if native is not None:
            return native
    occ = np.asarray(occupancy != 0)
    cap = max(1, int(math.ceil(max(0.0, float(max_distance_voxels)))))
    dist = np.full(occ.shape, float(cap), dtype=np.float32)
    dist[occ] = 0.0
    visited = occ.copy()
    frontier = occ.copy()
    for distance in range(1, cap + 1):
        if not frontier.any():
            break
        neigh = np.zeros_like(frontier)
        neigh[1:, :, :] |= frontier[:-1, :, :]
        neigh[:-1, :, :] |= frontier[1:, :, :]
        neigh[:, 1:, :] |= frontier[:, :-1, :]
        neigh[:, :-1, :] |= frontier[:, 1:, :]
        neigh[:, :, 1:] |= frontier[:, :, :-1]
        neigh[:, :, :-1] |= frontier[:, :, 1:]
        new = neigh & ~visited
        if not new.any():
            break
        dist[new] = float(distance)
        visited |= new
        frontier = new
    return dist


_KINO_MOVES_26: tuple[tuple[int, int, int], ...] = tuple(
    (dz, dr, dc)
    for dz in (-1, 0, 1)
    for dr in (-1, 0, 1)
    for dc in (-1, 0, 1)
    if not (dz == 0 and dr == 0 and dc == 0)
)



_ASTAR_MOVES = tuple(
    (int(dz), int(dr), int(dc), math.sqrt(float(dz * dz + dr * dr + dc * dc)))
    for dz, dr, dc in _KINO_MOVES_26
)


def _astar_diag_heuristic(z: int, r: int, c: int, goal: Cell3D) -> float:
    """Exact obstacle-free 26-neighbour grid distance."""
    a = abs(int(z) - int(goal[0]))
    b = abs(int(r) - int(goal[1]))
    d = abs(int(c) - int(goal[2]))
    lo = min(a, b, d)
    hi = max(a, b, d)
    mid = a + b + d - lo - hi
    return (
        float(hi - mid)
        + float(mid - lo) * math.sqrt(2.0)
        + float(lo) * math.sqrt(3.0)
    )


@dataclass
class _AStarWorkspace:
    """Reusable exact-A* arrays for one occupancy shape."""
    shape: tuple[int, int, int]
    g_score: np.ndarray
    parent: np.ndarray
    seen_gen: np.ndarray
    closed_gen: np.ndarray
    generation: int = 0

    @classmethod
    def create(cls, shape: tuple[int, int, int]) -> "_AStarWorkspace":
        size = int(shape[0] * shape[1] * shape[2])
        return cls(
            shape=shape,
            g_score=np.empty(size, dtype=np.float64),
            parent=np.empty(size, dtype=np.int32),
            seen_gen=np.zeros(size, dtype=np.uint32),
            closed_gen=np.zeros(size, dtype=np.uint32),
            generation=0,
        )

    def next_generation(self) -> np.uint32:
        self.generation = (int(self.generation) + 1) & 0xFFFFFFFF
        if self.generation == 0:
            self.seen_gen.fill(0)
            self.closed_gen.fill(0)
            self.generation = 1
        return np.uint32(self.generation)


_ASTAR_WORKSPACES: "OrderedDict[tuple[int, int, int], _AStarWorkspace]" = OrderedDict()
_ASTAR_WORKSPACE_LIMIT = 8


def _astar_workspace(shape: tuple[int, int, int]) -> _AStarWorkspace:
    ws = _ASTAR_WORKSPACES.pop(shape, None)
    if ws is None:
        ws = _AStarWorkspace.create(shape)
    _ASTAR_WORKSPACES[shape] = ws
    while len(_ASTAR_WORKSPACES) > _ASTAR_WORKSPACE_LIMIT:
        _ASTAR_WORKSPACES.popitem(last=False)
    return ws


def _fast_spine_astar(
    occupancy: np.ndarray,
    start: Cell3D,
    goal: Cell3D,
    bias: np.ndarray | None = None,
) -> tuple[Cell3D, ...]:
    """Exact flat-array 26-neighbour A* with reusable work buffers."""
    if _native_fastpath is not None:
        try:
            native_path = _native_fastpath.astar26(occupancy, start, goal, bias=bias)
        except Exception:
            native_path = None
        if native_path is not None:
            return native_path

    layers, rows, cols = occupancy.shape
    stride0 = rows * cols
    stride1 = cols
    size = layers * stride0
    shape = (int(layers), int(rows), int(cols))
    ws = _astar_workspace(shape)
    gen = ws.next_generation()
    g_score = ws.g_score
    parent = ws.parent
    seen_gen = ws.seen_gen
    closed_gen = ws.closed_gen

    def index(cell: Cell3D) -> int:
        return int(cell[0] * stride0 + cell[1] * stride1 + cell[2])

    start_idx = index(start)
    goal_idx = index(goal)
    g_score[start_idx] = 0.0
    parent[start_idx] = -1
    seen_gen[start_idx] = gen

    open_heap: list[tuple[float, float, int]] = [
        (_astar_diag_heuristic(start[0], start[1], start[2], goal), 0.0, start_idx)
    ]
    bias_flat = None if bias is None else np.asarray(bias, dtype=np.float64).reshape(-1)
    current: int | None = None

    while open_heap:
        _f, current_g, current = heapq.heappop(open_heap)
        if current == goal_idx:
            break
        if seen_gen[current] != gen or current_g > g_score[current] + 1.0e-9:
            continue
        closed_gen[current] = gen

        z = current // stride0
        rem = current - z * stride0
        r = rem // cols
        c = rem - r * cols

        for dz, dr, dc, move_cost in _ASTAR_MOVES:
            nz = z + dz
            nr = r + dr
            nc = c + dc
            if (
                nz < 0 or nz >= layers
                or nr < 0 or nr >= rows
                or nc < 0 or nc >= cols
            ):
                continue
            nxt = int(nz * stride0 + nr * stride1 + nc)
            if closed_gen[nxt] == gen or occupancy[nz, nr, nc]:
                continue
            extra = float(bias_flat[nxt]) if bias_flat is not None else 0.0
            tentative = current_g + move_cost + extra
            old = g_score[nxt] if seen_gen[nxt] == gen else float("inf")
            if tentative + 1.0e-9 >= old:
                continue

            seen_gen[nxt] = gen
            g_score[nxt] = tentative
            parent[nxt] = current
            heuristic = _astar_diag_heuristic(nz, nr, nc, goal)
            heapq.heappush(open_heap, (tentative + heuristic, tentative, nxt))

    if current != goal_idx:
        return ()

    path: list[Cell3D] = []
    node = goal_idx
    while node != -1:
        z = node // stride0
        rem = node - z * stride0
        r = rem // cols
        c = rem - r * cols
        path.append((int(z), int(r), int(c)))
        node = int(parent[node])
    path.reverse()
    return tuple(path)

def _fast_spine_astar_windowed(
    occupancy: np.ndarray,
    start: Cell3D,
    goal: Cell3D,
    bias: np.ndarray | None = None,
    *,
    margin_voxels: int = 8,
    fallback_full: bool = True,
) -> tuple[Cell3D, ...]:
    """Run A* in a conservative start/goal window before full-grid fallback."""
    m = max(1, int(margin_voxels))
    shape = occupancy.shape
    lo = [max(0, min(start[i], goal[i]) - m) for i in range(3)]
    hi = [min(shape[i], max(start[i], goal[i]) + m + 1) for i in range(3)]
    if all(lo[i] == 0 and hi[i] == shape[i] for i in range(3)):
        return _fast_spine_astar(occupancy, start, goal, bias=bias)
    sl = tuple(slice(lo[i], hi[i]) for i in range(3))
    local_occ = occupancy[sl]
    local_bias = None if bias is None else np.asarray(bias)[sl]
    local_start = tuple(start[i] - lo[i] for i in range(3))
    local_goal = tuple(goal[i] - lo[i] for i in range(3))
    local = _fast_spine_astar(local_occ, local_start, local_goal, bias=local_bias)
    if local:
        return tuple(tuple(cell[i] + lo[i] for i in range(3)) for cell in local)
    if fallback_full:
        return _fast_spine_astar(occupancy, start, goal, bias=bias)
    return ()


def _previous_path_spine(
    previous_path_xyz: Sequence[Sequence[float]],
    occupancy: np.ndarray,
    start: Cell3D,
    goal: Cell3D,
    frame: GridFrame3D,
) -> tuple[tuple[Cell3D, ...], str]:
    """Reuse the valid suffix of the previous rolling path when possible.

    This is a conservative 3-D ROS adaptation of the reference planner's
    Micro-Bypass idea.  If the old suffix is still clear, no new global A* is
    needed.  If a newly occupied voxel blocks the suffix, run one small A* only
    to a clear rejoin point after the last blocked voxel, then splice the still
    valid suffix back on.  Any ambiguity falls back to the normal global A*.
    """
    if not previous_path_xyz:
        return (), "astar"
    cells: list[Cell3D] = []
    for value in previous_path_xyz:
        if len(value) < 3:
            continue
        try:
            cell = xyz_to_cell3d(float(value[0]), float(value[1]), float(value[2]), frame)
        except (TypeError, ValueError):
            continue
        if not (0 <= cell[0] < occupancy.shape[0] and 0 <= cell[1] < occupancy.shape[1]
                and 0 <= cell[2] < occupancy.shape[2]):
            continue
        if not cells or cells[-1] != cell:
            cells.append(cell)
    if len(cells) < 2 or cells[-1] != goal:
        return (), "astar"

    # Trim everything already flown.  Nearest-cell matching is robust to odom
    # being between dense trajectory samples.
    nearest = min(
        range(len(cells)),
        key=lambda i: (
            (cells[i][0] - start[0]) ** 2
            + (cells[i][1] - start[1]) ** 2
            + (cells[i][2] - start[2]) ** 2
        ),
    )
    suffix = cells[nearest:]
    if not suffix:
        return (), "astar"
    if suffix[0] != start:
        suffix.insert(0, start)

    suffix_blocked = _segments_blocked_cells_batch(suffix[:-1], suffix[1:], occupancy)
    blocked = [i for i, is_blocked in enumerate(suffix_blocked, start=1) if is_blocked]
    if not blocked:
        return tuple(suffix), "previous-safe"

    # Rejoin several sparse waypoints after the last blocked *segment*.  The
    # entire retained tail is segment-revalidated before it is accepted.
    rejoin_i = min(len(suffix) - 1, max(blocked) + 2)
    while rejoin_i < len(suffix) - 1 and occupancy[suffix[rejoin_i]]:
        rejoin_i += 1
    if occupancy[suffix[rejoin_i]]:
        return (), "astar"
    tail = suffix[rejoin_i:]
    if any(occupancy[cell] for cell in tail):
        return (), "astar"
    if any(_segments_blocked_cells_batch(tail[:-1], tail[1:], occupancy)):
        return (), "astar"
    if not bool(occupancy[start]):
        suffix_remaining = [0.0] * len(suffix)
        for k in range(len(suffix) - 2, -1, -1):
            suffix_remaining[k] = suffix_remaining[k + 1] + _spine_length_m(
                (suffix[k], suffix[k + 1]), frame
            )
        best_i = None
        best_cost = float("inf")
        los_indices = list(range(rejoin_i, len(suffix)))
        los_blocked = _segments_blocked_cells_batch(
            [start] * len(los_indices), [suffix[i] for i in los_indices], occupancy
        )
        for los_i, is_blocked in zip(los_indices, los_blocked):
            if is_blocked:
                continue
            rejoin_cell = suffix[los_i]
            bridge = _spine_length_m((start, rejoin_cell), frame)
            total_cost = bridge + suffix_remaining[los_i]
            # Prefer a farther rejoin only as a tie-breaker; the primary key is
            # actual metric route length.
            if (
                total_cost + 1.0e-9 < best_cost
                or (abs(total_cost - best_cost) <= 1.0e-9 and (best_i is None or los_i > best_i))
            ):
                best_i = los_i
                best_cost = total_cost
        if best_i is not None:
            rejoin_cell = suffix[best_i]
            combined_los = [start]
            if rejoin_cell != start:
                combined_los.append(rejoin_cell)
            for cell in suffix[best_i + 1 :]:
                if combined_los[-1] != cell:
                    combined_los.append(cell)
            if not any(_segments_blocked_cells_batch(combined_los[:-1], combined_los[1:], occupancy)):
                return tuple(combined_los), "micro-bypass"

    patch = _fast_spine_astar_windowed(
        occupancy, start, suffix[rejoin_i], margin_voxels=6, fallback_full=True
    )
    if not patch:
        return (), "astar"
    combined = list(patch)
    for cell in tail[1:]:
        if combined[-1] != cell:
            combined.append(cell)
    if any(_segments_blocked_cells_batch(combined[:-1], combined[1:], occupancy)):
        return (), "astar"
    return tuple(combined), "micro-bypass"


@lru_cache(maxsize=4096)
def _segment_relative_samples(dz: int, dr: int, dc: int) -> np.ndarray:
    """Cached conservative voxel samples for a relative segment."""
    delta = np.asarray((int(dz), int(dr), int(dc)), dtype=np.int64)
    steps = max(1, int(np.max(np.abs(delta))))
    ts = np.linspace(0.0, 1.0, steps * 2 + 1, dtype=np.float64)[:, None]
    rel = np.rint(ts * delta.astype(np.float64)[None, :]).astype(np.int64)
    # Consecutive duplicate rounded samples do not add safety information.
    if len(rel) > 1:
        keep = np.ones(len(rel), dtype=bool)
        keep[1:] = np.any(rel[1:] != rel[:-1], axis=1)
        rel = rel[keep]
    rel.setflags(write=False)
    return rel


def _segment_cells_cached(a: Cell3D, b: Cell3D) -> np.ndarray:
    dz = int(b[0]) - int(a[0])
    dr = int(b[1]) - int(a[1])
    dc = int(b[2]) - int(a[2])
    rel = _segment_relative_samples(dz, dr, dc)
    return rel + np.asarray(a, dtype=np.int64)[None, :]


def _segment_clear_cells(
    a: Cell3D,
    b: Cell3D,
    occupancy: np.ndarray,
) -> bool:
    """Conservative voxel line-of-sight check for rolling-path shortcutting."""
    cells = _segment_cells_cached(a, b)
    valid = (
        (cells[:, 0] >= 0) & (cells[:, 0] < occupancy.shape[0])
        & (cells[:, 1] >= 0) & (cells[:, 1] < occupancy.shape[1])
        & (cells[:, 2] >= 0) & (cells[:, 2] < occupancy.shape[2])
    )
    if not bool(np.all(valid)):
        return False
    return not bool(np.any(occupancy[cells[:, 0], cells[:, 1], cells[:, 2]]))


def _segments_blocked_cells_batch(
    a_cells: Sequence[Cell3D],
    b_cells: Sequence[Cell3D],
    occupancy: np.ndarray,
) -> list[bool]:
    """Batch exact integer-cell LOS checker with Python fallback.

    ABI-3101 uses the same ties-to-even 2*span+1 voxel samples as
    ``_segment_clear_cells``.  The helper is deliberately boolean-blocked so
    rolling-spine code can batch dozens of visibility tests without changing
    the established safety semantics.
    """
    if len(a_cells) != len(b_cells):
        raise ValueError("segment endpoint batch length mismatch")
    if not a_cells:
        return []
    if _native_fastpath is not None:
        try:
            native = _native_fastpath.segments_blocked_cells3d(a_cells, b_cells, occupancy)
        except Exception:
            native = None
        if native is not None:
            return [bool(v) for v in native]
    return [not _segment_clear_cells(a, b, occupancy) for a, b in zip(a_cells, b_cells)]


def _segment_clear_with_endpoint_grace(
    a: Cell3D,
    b: Cell3D,
    inflated: np.ndarray,
    raw: np.ndarray,
    *,
    grace_voxels: int = 2,
) -> bool:
    """Near-goal LOS that tolerates inflated start/goal pockets only."""
    cells = _segment_cells_cached(a, b)
    valid = (
        (cells[:, 0] >= 0) & (cells[:, 0] < raw.shape[0])
        & (cells[:, 1] >= 0) & (cells[:, 1] < raw.shape[1])
        & (cells[:, 2] >= 0) & (cells[:, 2] < raw.shape[2])
    )
    if not bool(np.all(valid)):
        return False
    z, y, x = cells[:, 0], cells[:, 1], cells[:, 2]
    if bool(np.any(raw[z, y, x])):
        return False
    blocked = np.asarray(inflated[z, y, x] != 0)
    if not bool(np.any(blocked)):
        return True
    grace = max(0, int(grace_voxels))
    blocked_cells = cells[blocked]
    da = np.max(np.abs(blocked_cells - np.asarray(a, dtype=np.int64)), axis=1)
    db = np.max(np.abs(blocked_cells - np.asarray(b, dtype=np.int64)), axis=1)
    return bool(np.all((da <= grace) | (db <= grace)))


def _shortcut_spine_cells(
    spine: Sequence[Cell3D],
    occupancy: np.ndarray,
    *,
    max_lookahead: int = 24,
) -> tuple[Cell3D, ...]:
    """Greedy line-of-sight shortening for a revalidated rolling spine."""
    if len(spine) <= 2:
        return tuple(spine)
    lookahead = max(2, int(max_lookahead))
    out: list[Cell3D] = [spine[0]]
    i = 0
    last = len(spine) - 1
    while i < last:
        upper = min(last, i + lookahead)
        chosen = i + 1
        candidates = list(range(upper, i + 1, -1))
        blocked = _segments_blocked_cells_batch(
            [spine[i]] * len(candidates),
            [spine[j] for j in candidates],
            occupancy,
        )
        for j, is_blocked in zip(candidates, blocked):
            if not is_blocked:
                chosen = j
                break
        out.append(spine[chosen])
        i = chosen
    return tuple(out)


def _shortcut_waypoints_xyz(
    waypoints: Sequence[Point3],
    occupancy: np.ndarray,
    frame: GridFrame3D,
    *,
    max_lookahead: int = 12,
) -> tuple[Point3, ...]:
    """Delete visibility-redundant selected waypoints without changing safety.

    The Ising solution may contain a small lateral zig-zag that is still within
    the quality tolerance.  A final greedy LOS pass can only remove waypoints;
    every accepted replacement segment is checked against the same occupancy
    used by the QUBO.  Dynamics are re-timed and re-validated afterwards, so
    this improves geometry without relaxing collision/velocity/acceleration
    guards.
    """
    if len(waypoints) <= 2:
        return tuple(waypoints)
    lookahead = max(2, int(max_lookahead))
    out: list[Point3] = [tuple(waypoints[0])]
    i = 0
    last = len(waypoints) - 1
    while i < last:
        upper = min(last, i + lookahead)
        chosen = i + 1
        for j in range(upper, i + 1, -1):
            if not _segment_blocked_zyx(waypoints[i], waypoints[j], occupancy, frame):
                chosen = j
                break
        out.append(tuple(waypoints[chosen]))
        i = chosen
    return tuple(out)


def _spine_length_m(spine: Sequence[Cell3D], frame: GridFrame3D) -> float:
    """Metric length of a voxel backbone."""
    if len(spine) < 2:
        return 0.0
    total = 0.0
    for a, b in zip(spine, spine[1:]):
        dz = (b[0] - a[0]) * frame.resolution_z_m
        dy = (b[1] - a[1]) * frame.resolution_m
        dx = (b[2] - a[2]) * frame.resolution_m
        total += math.sqrt(dx * dx + dy * dy + dz * dz)
    return total


def _spine_turn_ratio(spine: Sequence[Cell3D]) -> float:
    if len(spine) < 3:
        return 0.0
    previous = None
    turns = 0
    steps = 0
    for a, b in zip(spine, spine[1:]):
        direction = (
            int(math.copysign(1, b[0] - a[0])) if b[0] != a[0] else 0,
            int(math.copysign(1, b[1] - a[1])) if b[1] != a[1] else 0,
            int(math.copysign(1, b[2] - a[2])) if b[2] != a[2] else 0,
        )
        if previous is not None:
            steps += 1
            if direction != previous:
                turns += 1
        previous = direction
    return turns / max(1, steps)


def _adaptive_problem_shape(
    spine: Sequence[Cell3D],
    frame: GridFrame3D,
    layer_cap: int,
    candidate_cap: int,
    enabled: bool,
) -> tuple[int, int, str, float, float]:
    """Choose an Active-N budget from route complexity.

    This is the 3-D ROS analogue of the standalone UAV planner's
    Micro/Ultra/Portal idea: short rolling routes should *shrink* the Ising
    problem instead of always consuming all 256 spins.
    """
    layer_cap = max(2, min(int(layer_cap), 254))
    candidate_cap = max(2, min(int(candidate_cap), N_HW))
    length_m = _spine_length_m(spine, frame)
    turn_ratio = _spine_turn_ratio(spine)
    if not enabled:
        layers, candidates, profile = layer_cap, candidate_cap, "manual"
    elif length_m <= 2.5 and len(spine) <= 40:
        layers, candidates, profile = min(layer_cap, 8), min(candidate_cap, 4), "near-tiny-wide"
    elif length_m <= 4.0 and len(spine) <= 64:
        layers, candidates, profile = min(layer_cap, 8), min(candidate_cap, 4), "near-wide"
    elif length_m <= 8.0 and len(spine) <= 72 and turn_ratio <= 0.30:
        # Most rolling replans in the current bag are short and nearly straight.
        # The exact A* backbone is always retained as candidate 0, so 8x3 keeps
        # a guaranteed-feasible reference plus two lateral alternatives while
        # cutting the QUBO from 40 to only 24 active spins.
        layers, candidates, profile = min(layer_cap, 8), min(candidate_cap, 3), "micro"
    elif length_m <= 8.0 and len(spine) <= 84 and turn_ratio <= 0.60:
        # Mildly turning short routes use 8x4=32 rather than 10x4=40.  Fewer
        # longitudinal layers reduce host QUBO construction, wire size and PL
        # update work, and also suppress layer-to-layer zig-zagging.
        layers, candidates, profile = min(layer_cap, 8), min(candidate_cap, 4), "ultra"
    elif length_m <= 8.0 and len(spine) <= 104:
        layers, candidates, profile = min(layer_cap, 8), min(candidate_cap, 4), "ultra-turn"
    elif length_m <= 15.0 and len(spine) <= 132 and turn_ratio <= 0.50:
        # 12x4=48 is enough for a known-feasible corridor in ordinary medium
        # routes.  Harder medium routes below retain a larger 14x5 model.
        layers, candidates, profile = min(layer_cap, 12), min(candidate_cap, 4), "compact-lite"
    elif length_m <= 15.0 and len(spine) <= 152:
        layers, candidates, profile = min(layer_cap, 14), min(candidate_cap, 5), "compact"
    elif length_m <= 25.0 and turn_ratio <= 0.45:
        layers, candidates, profile = min(layer_cap, 16), min(candidate_cap, 5), "wide-lite"
    elif length_m <= 25.0:
        layers, candidates, profile = min(layer_cap, 18), min(candidate_cap, 6), "wide"
    else:
        layers, candidates, profile = layer_cap, candidate_cap, "full"
    while layers * candidates > N_HW and layers > 2:
        layers -= 1
    if layers * candidates > N_HW:
        candidates = max(2, N_HW // layers)
    return layers, candidates, profile, float(length_m), float(turn_ratio)


def _adaptive_solver_profiles(
    variables: int,
    requested_iterations: int,
    requested_restarts: int,
    enabled: bool,
) -> list[tuple[int, int]]:
    requested_iterations = max(1, min(65535, int(requested_iterations)))
    requested_restarts = max(1, min(32, int(requested_restarts)))
    if not enabled:
        return [(requested_iterations, requested_restarts)]

    n = max(1, int(variables))
    if n <= 24:
        base = [(540, 8)]
    elif n <= 32:
        base = [(640, 8)]
    elif n <= 48:
        base = [(730, 8)]
    elif n <= 80:
        base = [(430, 8)]
    elif n <= 112:
        base = [(650, 3), (950, 4)]
    elif n <= 160:
        base = [(800, 3), (1150, 4)]
    elif n <= 192:
        base = [(950, 4), (1400, 5)]
    else:
        base = [(1200, 4), (1800, 6)]
    out: list[tuple[int, int]] = []
    for iterations, restarts in base:
        out.append((min(requested_iterations, iterations), min(requested_restarts, restarts)))
    # Avoid two identical attempts when the user deliberately requested a
    # smaller manual ceiling.
    if len(out) > 1 and out[1] == out[0]:
        out = out[:1]
    return out


def _quality_tolerance_m(
    spine_geom_len: float,
    resolution_m: float,
    quality_gap_ratio: float,
    size_profile: str,
    spine_source: str,
) -> float:
    """Adaptive absolute geometry guard around the known-safe spine."""
    length = max(0.0, float(spine_geom_len))
    res = max(1.0e-6, float(resolution_m))
    ratio = max(0.0, float(quality_gap_ratio))
    profile = str(size_profile or "")
    source = str(spine_source or "")
    tight = (
        profile.startswith("near-")
        or profile == "micro-bypass"
        or source in ("micro-bypass", "micro-bypass-raw")
    )
    if tight:
        ratio = min(ratio, 0.03)
        cap = 1.0 * res
    else:
        cap = 2.0 * res
    floor = 0.5 * res
    return min(cap, max(floor, ratio * max(length, 1.0e-9)))


def _chain_blocked_count(
    points_xyz: Sequence[Point3],
    occupancy: np.ndarray,
    frame: GridFrame3D,
    samples: int = 8,
) -> int:
    """Number of consecutive-pair straight segments that hit occupied voxels
    (vectorised over the whole chain)."""
    if len(points_xyz) < 2:
        return 0
    if _native_fastpath is not None:
        try:
            native_count = _native_fastpath.chain_blocked_count_xyz3d(
                points_xyz, occupancy, frame, samples=samples
            )
        except Exception:
            native_count = None
        if native_count is not None:
            return int(native_count)
    layers, rows, cols = occupancy.shape
    occ = np.asarray(occupancy != 0)
    a = np.asarray(points_xyz[:-1], dtype=np.float64)
    b = np.asarray(points_xyz[1:], dtype=np.float64)
    steps = max(2, int(samples))
    ts = (np.arange(1, steps) / steps)[None, :, None]
    pts = a[:, None, :] + (b - a)[:, None, :] * ts
    pts = pts.reshape(-1, 3)
    col = np.floor((pts[:, 0] - frame.x_min) / frame.resolution_m).astype(np.int64)
    row = np.floor((pts[:, 1] - frame.y_min) / frame.resolution_m).astype(np.int64)
    zl = np.floor((pts[:, 2] - frame.z_min) / frame.resolution_z_m).astype(np.int64)
    inside = (zl >= 0) & (zl < layers) & (row >= 0) & (row < rows) & (col >= 0) & (col < cols)
    blocked = ~inside
    if inside.any():
        idx = (zl[inside], row[inside], col[inside])
        blocked[inside] |= occ[idx]
    seg_count = len(points_xyz) - 1
    return int(blocked.reshape(seg_count, steps - 1).any(axis=1).sum())


def _blocked_pairs_bulk(
    candidate_cells: Sequence[Sequence[Cell3D]],
    occupancy: np.ndarray,
    frame: GridFrame3D,
    samples: int = 8,
) -> list[list[list[bool]]]:
    """Vectorised straight-segment blocking lookup between adjacent layers.

    ``candidate_cells`` are voxel indices in **(z,row,col)** order.  An older
    implementation accidentally treated those indices as metric **(x,y,z)**
    coordinates and converted them through ``frame`` a second time.  On maps
    whose origin/resolution are not numerically close to the voxel indices this
    marks essentially arbitrary links as blocked, degrading both feasibility
    and path length.  Work directly in voxel space here; ``frame`` is retained
    in the signature for API compatibility.
    """
    if _native_fastpath is not None:
        try:
            native_blocked = _native_fastpath.blocked_pairs3d(
                candidate_cells, occupancy, samples=samples
            )
        except Exception:
            native_blocked = None
        if native_blocked is not None:
            return native_blocked

    del frame
    layers, rows, cols = occupancy.shape
    occ = np.asarray(occupancy != 0)
    neighbour_deltas = np.asarray([
        (0, 0, 0),
        (0, 0, 1), (0, 0, -1),
        (0, 1, 0), (0, -1, 0),
        (1, 0, 0), (-1, 0, 0),
    ], dtype=np.int64)
    out: list[list[list[bool]]] = []
    for layer in range(len(candidate_cells) - 1):
        left = np.asarray(candidate_cells[layer], dtype=np.float64)
        right = np.asarray(candidate_cells[layer + 1], dtype=np.float64)
        m = len(left)
        n = len(right)
        # Interpolate directly in voxel coordinates (z,row,col).
        a = left[:, None, :]  # (m,1,3)
        b = right[None, :, :]  # (1,n,3)
        max_voxel_span = int(np.ceil(np.max(np.abs(b - a)))) if m and n else 1
        steps = max(2, int(samples), max_voxel_span * 2 + 1)
        ts = (np.arange(1, steps) / steps)[None, None, :, None]  # (1,1,s-1,1)
        pts = a[:, :, None, :] + (b - a)[:, :, None, :] * ts  # (m,n,s-1,3)
        # Nearest-voxel sampling is symmetric for both segment directions and
        # avoids the systematic low-index bias introduced by floor().
        cells = np.rint(pts.reshape(-1, 3)).astype(np.int64)
        expanded = cells[:, None, :] + neighbour_deltas[None, :, :]
        valid = (
            (expanded[:, :, 0] >= 0) & (expanded[:, :, 0] < layers)
            & (expanded[:, :, 1] >= 0) & (expanded[:, :, 1] < rows)
            & (expanded[:, :, 2] >= 0) & (expanded[:, :, 2] < cols)
        )
        # A centre sample outside the map is blocked.  Neighbour samples that
        # fall outside simply do not contribute to the grazing check.
        centre_inside = valid[:, 0]
        blocked = ~centre_inside
        hit = np.zeros(cells.shape[0], dtype=bool)
        for k in range(expanded.shape[1]):
            sel = valid[:, k]
            if sel.any():
                hit[sel] |= occ[
                    expanded[sel, k, 0],
                    expanded[sel, k, 1],
                    expanded[sel, k, 2],
                ]
        blocked |= hit
        blocked = blocked.reshape(m, n, steps - 1).any(axis=2)
        out.append([[bool(blocked[i, j]) for j in range(n)] for i in range(m)])
    return out


def _segment_blocked_zyx(
    a_xyz: Point3,
    b_xyz: Point3,
    occupancy: np.ndarray,
    frame: GridFrame3D,
    samples: int = 8,
) -> bool:
    # Match the bulk edge checker: never let a long metric segment rely on only
    # eight samples.  This removes the mismatch where waypoint validation passed
    # but the 50-Hz dense trajectory later discovered an obstacle.
    try:
        a_cell = xyz_to_cell3d(a_xyz[0], a_xyz[1], a_xyz[2], frame)
        b_cell = xyz_to_cell3d(b_xyz[0], b_xyz[1], b_xyz[2], frame)
        voxel_span = max(abs(b_cell[i] - a_cell[i]) for i in range(3))
    except Exception:
        voxel_span = 1
    steps = max(2, int(samples), int(voxel_span) * 2 + 1)
    for k in range(1, steps):
        t = k / float(steps)
        x = a_xyz[0] + (b_xyz[0] - a_xyz[0]) * t
        y = a_xyz[1] + (b_xyz[1] - a_xyz[1]) * t
        z = a_xyz[2] + (b_xyz[2] - a_xyz[2]) * t
        cell = xyz_to_cell3d(x, y, z, frame)
        if not (0 <= cell[0] < occupancy.shape[0] and 0 <= cell[1] < occupancy.shape[1]
                and 0 <= cell[2] < occupancy.shape[2]):
            return True
        if occupancy[cell]:
            return True
    return False


def _resample_spine_xyz(spine: Sequence[Cell3D], frame: GridFrame3D, layers: int) -> list[Point3]:
    """Resample the voxel chain into `layers` uniformly spaced points (m)."""
    points = [cell3d_to_xyz(cell, frame) for cell in spine]
    if len(points) <= 1:
        return [points[0]] * layers
    arc = [0.0]
    for left, right in zip(points, points[1:]):
        arc.append(arc[-1] + math.dist(left, right))
    total = arc[-1]
    out: list[Point3] = []
    for index in range(layers):
        target = total * index / max(1, layers - 1)
        if target <= 0.0:
            out.append(points[0])
            continue
        if target >= total:
            out.append(points[-1])
            continue
        # binary search for segment
        lo, hi = 0, len(arc) - 1
        while lo + 1 < hi:
            mid = (lo + hi) // 2
            if arc[mid] <= target:
                lo = mid
            else:
                hi = mid
        seg_len = max(arc[hi] - arc[lo], 1.0e-12)
        t = (target - arc[lo]) / seg_len
        p = points[lo]
        q = points[hi]
        out.append((p[0] + (q[0] - p[0]) * t, p[1] + (q[1] - p[1]) * t, p[2] + (q[2] - p[2]) * t))
    return out


def _candidate_voxels(
    spine_point: Point3,
    occupancy: np.ndarray,
    frame: GridFrame3D,
    *,
    radius: int,
    max_count: int,
    clearance: np.ndarray,
    safe_clearance_voxels: float,
    goal_bias: Optional[Point3] = None,
) -> list[tuple[Cell3D, Point3, float]]:
    """Free voxels in a Chebyshev cube around the spine point.

    Returns list of (cell, xyz, obstacle_cost).  Vectorised with numpy.
    """
    layers, rows, cols = occupancy.shape
    radius = max(1, int(radius))
    center = xyz_to_cell3d(spine_point[0], spine_point[1], spine_point[2], frame)
    z0, z1 = max(0, center[0] - radius), min(layers, center[0] + radius + 1)
    r0, r1 = max(0, center[1] - radius), min(rows, center[1] + radius + 1)
    c0, c1 = max(0, center[2] - radius), min(cols, center[2] + radius + 1)
    if z1 <= z0 or r1 <= r0 or c1 <= c0:
        return []
    cube = occupancy[z0:z1, r0:r1, c0:c1]
    free = np.argwhere(cube == 0)
    if free.size == 0:
        return []
    cells_z = free[:, 0] + z0
    cells_r = free[:, 1] + r0
    cells_c = free[:, 2] + c0
    clear = clearance[cells_z, cells_r, cells_c].astype(np.float64)
    xyz = np.stack([
        frame.x_min + (cells_c + 0.5) * frame.resolution_m,
        frame.y_min + (cells_r + 0.5) * frame.resolution_m,
        frame.z_min + (cells_z + 0.5) * frame.resolution_z_m,
    ], axis=1)
    spine_arr = np.asarray(spine_point, dtype=np.float64)
    dist_spine = np.sqrt(np.sum((xyz - spine_arr) ** 2, axis=1))
    if goal_bias is not None:
        bias = np.asarray(goal_bias, dtype=np.float64)
        goal_proj = xyz @ bias
    else:
        goal_proj = np.zeros(free.shape[0])
    # Keep the exact A* skeleton candidate whenever possible.  The previous
    # clearance-first ordering could use all of a small candidate budget on
    # farther high-clearance voxels and accidentally remove the known shortest
    # feasible backbone.  That both lengthened paths and made failed anneals
    # more likely to fall back to a deterministic route.
    center_match = np.flatnonzero(
        (cells_z == center[0]) & (cells_r == center[1]) & (cells_c == center[2])
    )
    baseline_idx = int(center_match[0]) if center_match.size else int(np.argmin(dist_spine))

    # Distance to the exact spine is the primary quality signal.  Clearance is
    # a tie-breaker because obstacles were already inflated for vehicle radius;
    # forward progress is the final tie-breaker (larger projection is better).
    order = np.lexsort((-goal_proj, -np.minimum(clear, safe_clearance_voxels), dist_spine))
    selected: list[int] = [baseline_idx]
    for idx in order:
        idx_i = int(idx)
        if idx_i == baseline_idx:
            continue
        selected.append(idx_i)
        if len(selected) >= max_count:
            break
    out: list[tuple[Cell3D, Point3, float]] = []
    for idx in selected[:max_count]:
        penalty = max(0.0, float(safe_clearance_voxels) - float(clear[idx])) ** 2
        out.append(((int(cells_z[idx]), int(cells_r[idx]), int(cells_c[idx])),
                    (float(xyz[idx, 0]), float(xyz[idx, 1]), float(xyz[idx, 2])),
                    penalty))
    return out


class _QuadraticAccumulator:
    """Accumulates linear and bilinear coefficients of a polynomial in one-hot
    binary variables x[var].  dot-product terms between layer positions are
    expanded using x_j^2 = x_j, so every term stays quadratic.  Dense numpy
    storage keeps QUBO construction fast."""

    def __init__(self, variable_count: int) -> None:
        self.variable_count = variable_count
        self.linear = np.zeros(variable_count, dtype=np.float64)
        self.matrix = np.zeros((variable_count, variable_count), dtype=np.float64)

    def add_bilinear(self, u: int, v: int, coefficient: float) -> None:
        if abs(coefficient) < 1.0e-12:
            return
        self.matrix[u, v] += coefficient
        self.matrix[v, u] += coefficient

    def add_dot(
        self,
        layer_a: int,
        layer_b: int,
        weight: float,
        positions: Sequence[object],
        var_map: dict[int, tuple[int, int] | None],
    ) -> None:
        """Add `weight * dot(p_layer_a, p_layer_b)`.

        positions[layer] is either a fixed (x, y, z) tuple (fixed layer) or a
        list of candidate (x, y, z) tuples (variable layer, one-hot).
        """
        if abs(weight) < 1.0e-12:
            return
        pa = positions[layer_a]
        pb = positions[layer_b]
        ma = var_map[layer_a]
        mb = var_map[layer_b]
        if ma is None and mb is None:
            return  # constant
        qa = np.asarray(pa, dtype=np.float64)
        qb = np.asarray(pb, dtype=np.float64)
        if layer_a == layer_b and ma is not None:
            # same-layer dot: sum_j x_j |q_j|^2  (linear, x_j^2 = x_j)
            offset, count = ma
            self.linear[offset:offset + count] += weight * np.einsum("ij,ij->i", qa, qa)
            return
        if ma is None:
            fixed = qa
            offset, count = mb
            self.linear[offset:offset + count] += weight * (qb @ fixed)
            return
        if mb is None:
            fixed = qb
            offset, count = ma
            self.linear[offset:offset + count] += weight * (qa @ fixed)
            return
        offset_a, count_a = ma
        offset_b, count_b = mb
        # bilinear matrix: weight * (qa @ qb.T)
        block = weight * (qa @ qb.T)
        self.matrix[offset_a:offset_a + count_a, offset_b:offset_b + count_b] += block
        self.matrix[offset_b:offset_b + count_b, offset_a:offset_a + count_a] += block.T

    def add_linear(self, var: int, coefficient: float) -> None:
        self.linear[var] += coefficient

    def add_linear_slice(self, offset: int, coefficients: Sequence[float]) -> None:
        self.linear[offset:offset + len(coefficients)] += np.asarray(coefficients, dtype=np.float64)

    def add_bilinear_block(self, rows: slice, cols: slice, block: np.ndarray) -> None:
        self.matrix[rows, cols] += block
        self.matrix[cols, rows] += block.T

    def to_matrix(self) -> np.ndarray:
        n = self.variable_count
        matrix = self.matrix.copy()
        diag_idx = np.arange(n, dtype=np.intp)
        matrix[diag_idx, diag_idx] += self.linear
        return matrix

    def max_abs_term(self) -> float:
        return max(
            float(np.max(np.abs(self.linear), initial=0.0)) if self.variable_count else 0.0,
            float(np.max(np.abs(self.matrix), initial=0.0)) if self.variable_count else 0.0,
        )

    def max_touch_magnitude(self) -> float:
        """Max objective change caused by flipping one variable (0->1/1->0)."""
        touch = np.abs(self.linear) + np.abs(self.matrix).sum(axis=1)
        return float(np.max(touch, initial=0.0)) if self.variable_count else 0.0

    def scale(self, factor: float) -> None:
        if factor <= 0.0 or factor == 1.0:
            return
        self.linear /= factor
        self.matrix /= factor



@dataclass
class _KinoQuboStaticEntry:
    """Candidate-geometry dependent part of the kinodynamic QUBO.

    Rolling replans usually alter start state / clearance costs much more often
    than the corridor topology itself.  All variable-variable geometry terms
    are therefore cached and only the dynamic unary part is rebuilt.
    """
    variable_count: int
    var_map: dict[int, tuple[int, int] | None]
    linear: np.ndarray
    matrix: np.ndarray
    build_ms: float
    hits: int = 0


def _kino_qubo_static_key(
    candidate_positions: Sequence[Sequence[Point3]],
    *,
    dt: float,
    w_vel: float,
    w_acc: float,
    w_len: float,
    w_start_vel: float,
    w_start_acc: float,
    w_end_vel: float,
) -> bytes:
    """Stable compact key for the QUBO terms that cannot change with start,
    goal, obstacle clearance or endpoint segment state."""
    counts = np.asarray([len(layer) for layer in candidate_positions], dtype=np.int16)
    if counts.size:
        flat = np.ascontiguousarray(
            np.asarray([p for layer in candidate_positions for p in layer], dtype=np.float64).reshape(-1, 3)
        )
    else:
        flat = np.empty((0, 3), dtype=np.float64)
    params = np.asarray(
        [dt, w_vel, w_acc, w_len, w_start_vel, w_start_acc, w_end_vel],
        dtype=np.float64,
    )
    digest = hashlib.blake2b(digest_size=16)
    digest.update(counts.tobytes())
    digest.update(flat.tobytes())
    digest.update(params.tobytes())
    return digest.digest()


def _kino_var_layout(candidate_positions: Sequence[Sequence[Point3]]):
    free_layers = len(candidate_positions)
    total_layers = free_layers + 2
    var_map: dict[int, tuple[int, int] | None] = {0: None, total_layers - 1: None}
    offset = 0
    for layer in range(1, total_layers - 1):
        count = len(candidate_positions[layer - 1])
        var_map[layer] = (offset, count)
        offset += count
    return free_layers, total_layers, var_map, offset


def _build_kino_static_entry(
    candidate_positions: Sequence[Sequence[Point3]],
    *,
    dt: float,
    w_vel: float,
    w_acc: float,
    w_len: float,
    w_start_vel: float,
    w_start_acc: float,
    w_end_vel: float,
) -> _KinoQuboStaticEntry:
    started = time.perf_counter()
    free_layers, total_layers, var_map, variable_count = _kino_var_layout(candidate_positions)
    # Fixed coordinates are deliberately zero here.  The helper below accepts
    # only variable-variable terms, so their actual value never enters the
    # cached basis.
    positions: list[object] = [(0.0, 0.0, 0.0)]
    positions.extend(candidate_positions)
    positions.append((0.0, 0.0, 0.0))
    acc = _QuadraticAccumulator(variable_count)
    dt2 = dt * dt
    dt4 = dt2 * dt2

    def static_dot(a: int, b: int, weight: float) -> None:
        if var_map[a] is not None and var_map[b] is not None:
            acc.add_dot(a, b, weight, positions, var_map)

    for layer in range(total_layers - 1):
        weight = w_vel / dt2
        static_dot(layer, layer, weight)
        static_dot(layer + 1, layer + 1, weight)
        static_dot(layer, layer + 1, -2.0 * weight)

    for layer in range(total_layers - 2):
        weight = w_acc / dt4
        static_dot(layer, layer, weight)
        static_dot(layer + 1, layer + 1, 4.0 * weight)
        static_dot(layer + 2, layer + 2, weight)
        static_dot(layer, layer + 1, -4.0 * weight)
        static_dot(layer, layer + 2, 2.0 * weight)
        static_dot(layer + 1, layer + 2, -4.0 * weight)

    if w_start_vel > 0.0 and free_layers >= 1:
        static_dot(1, 1, w_start_vel)

    if w_start_acc > 0.0 and free_layers >= 2:
        weight = w_start_acc / dt4
        static_dot(2, 2, weight)
        static_dot(1, 1, 4.0 * weight)
        static_dot(2, 1, -4.0 * weight)

    if w_end_vel > 0.0 and free_layers >= 1:
        static_dot(total_layers - 2, total_layers - 2, w_end_vel)

    # Only interior variable<->variable geometric length belongs to the static
    # basis.  Start/goal distances are unary and rebuilt from the live endpoints.
    if w_len > 0.0:
        for layer in range(1, total_layers - 2):
            left_map = var_map[layer]
            right_map = var_map[layer + 1]
            if left_map is None or right_map is None:
                continue
            left = np.asarray(candidate_positions[layer - 1], dtype=np.float64)
            right = np.asarray(candidate_positions[layer], dtype=np.float64)
            offset_l, count_l = left_map
            offset_r, count_r = right_map
            delta = left[:count_l, None, :] - right[None, :count_r, :]
            d = np.sqrt(np.sum(delta * delta, axis=2))
            acc.add_bilinear_block(
                slice(offset_l, offset_l + count_l),
                slice(offset_r, offset_r + count_r),
                w_len * d,
            )

    return _KinoQuboStaticEntry(
        variable_count=variable_count,
        var_map=dict(var_map),
        linear=acc.linear.copy(),
        matrix=acc.matrix.copy(),
        build_ms=(time.perf_counter() - started) * 1000.0,
    )


def _build_kino_qubo_incremental(
    *,
    fixed_positions: Sequence[Point3],
    candidate_positions: Sequence[Sequence[Point3]],
    candidate_obstacle_costs: Sequence[Sequence[float]],
    candidate_reference_costs: Sequence[Sequence[float]] | None,
    candidate_blocked_pairs: Sequence[Sequence[Sequence[bool]]],
    candidate_endpoint_blocked: tuple[Sequence[bool], Sequence[bool]] | None,
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
    start_vel_xyz: Point3,
    start_acc_xyz: Point3,
    goal_vel_xyz: Point3,
    a_mult: float,
    name: str,
    static_cache: OrderedDict[bytes, _KinoQuboStaticEntry],
    cache_size: int,
) -> tuple[Problem, dict[int, tuple[int, int]], float, float]:
    free_layers, total_layers, var_map, variable_count = _kino_var_layout(candidate_positions)
    key = _kino_qubo_static_key(
        candidate_positions,
        dt=dt,
        w_vel=w_vel,
        w_acc=w_acc,
        w_len=w_len,
        w_start_vel=w_start_vel,
        w_start_acc=w_start_acc,
        w_end_vel=w_end_vel,
    )
    entry = static_cache.get(key)
    static_hit = entry is not None
    if entry is None or entry.variable_count != variable_count:
        entry = _build_kino_static_entry(
            candidate_positions,
            dt=dt,
            w_vel=w_vel,
            w_acc=w_acc,
            w_len=w_len,
            w_start_vel=w_start_vel,
            w_start_acc=w_start_acc,
            w_end_vel=w_end_vel,
        )
        static_cache[key] = entry
        static_cache.move_to_end(key)
        while len(static_cache) > max(1, int(cache_size)):
            static_cache.popitem(last=False)
    else:
        entry.hits += 1
        static_cache.move_to_end(key)

    positions: list[object] = list(fixed_positions[:1])
    positions.extend(candidate_positions)
    positions.append(fixed_positions[-1])
    # Copying at n<=256 is cheaper and safer than mutating the cached basis.
    acc = _QuadraticAccumulator(variable_count)
    acc.linear[:] = entry.linear
    acc.matrix[:, :] = entry.matrix
    dt2 = dt * dt
    dt4 = dt2 * dt2

    def dynamic_dot(a: int, b: int, weight: float) -> None:
        ma = var_map[a]
        mb = var_map[b]
        if (ma is None) != (mb is None):
            acc.add_dot(a, b, weight, positions, var_map)

    # Dynamic endpoint contributions of velocity / acceleration objectives.
    for layer in range(total_layers - 1):
        weight = w_vel / dt2
        dynamic_dot(layer, layer + 1, -2.0 * weight)

    for layer in range(total_layers - 2):
        weight = w_acc / dt4
        dynamic_dot(layer, layer + 1, -4.0 * weight)
        dynamic_dot(layer, layer + 2, 2.0 * weight)
        dynamic_dot(layer + 1, layer + 2, -4.0 * weight)

    start_target = (
        fixed_positions[0][0] + start_vel_xyz[0] * dt,
        fixed_positions[0][1] + start_vel_xyz[1] * dt,
        fixed_positions[0][2] + start_vel_xyz[2] * dt,
    )
    if w_start_vel > 0.0 and free_layers >= 1:
        offset1, _count1 = var_map[1]
        qa = np.asarray(candidate_positions[0], dtype=np.float64)
        target = np.asarray(start_target, dtype=np.float64)
        acc.add_linear_slice(offset1, -2.0 * w_start_vel * (qa @ target))

    if w_start_acc > 0.0 and free_layers >= 2:
        weight = w_start_acc / dt4
        dynamic_dot(2, 0, 2.0 * weight)
        dynamic_dot(1, 0, -4.0 * weight)
        offset1, _count1 = var_map[1]
        offset2, _count2 = var_map[2]
        q1 = np.asarray(candidate_positions[0], dtype=np.float64)
        q2 = np.asarray(candidate_positions[1], dtype=np.float64)
        avec = np.asarray(start_acc_xyz, dtype=np.float64)
        acc.add_linear_slice(offset1, 4.0 * weight * (q1 @ avec) * dt2)
        acc.add_linear_slice(offset2, -2.0 * weight * (q2 @ avec) * dt2)

    if w_end_vel > 0.0 and free_layers >= 1:
        dynamic_dot(total_layers - 2, total_layers - 1, -2.0 * w_end_vel)

    # Dynamic obstacle/reference unary terms.
    for layer in range(free_layers):
        offset_l, count_l = var_map[layer + 1]
        costs = np.asarray(candidate_obstacle_costs[layer][:count_l], dtype=np.float64)
        if costs.size:
            costs = np.where(costs > 0.0, costs, 0.0)
            acc.add_linear_slice(offset_l, w_obs * costs)
    if candidate_reference_costs is not None and w_spine > 0.0:
        for layer in range(min(free_layers, len(candidate_reference_costs))):
            offset_l, count_l = var_map[layer + 1]
            costs = np.asarray(candidate_reference_costs[layer][:count_l], dtype=np.float64)
            if costs.size:
                costs = np.where(costs > 0.0, costs, 0.0)
                acc.add_linear_slice(offset_l, w_spine * costs)

    # Dynamic geometric path length only at fixed endpoints.
    if w_len > 0.0 and free_layers >= 1:
        start = np.asarray(fixed_positions[0], dtype=np.float64)
        goal = np.asarray(fixed_positions[-1], dtype=np.float64)
        first = np.asarray(candidate_positions[0], dtype=np.float64)
        last = np.asarray(candidate_positions[-1], dtype=np.float64)
        offset_first, count_first = var_map[1]
        offset_last, count_last = var_map[total_layers - 2]
        acc.add_linear_slice(offset_first, w_len * _norm_rows_xyz(first[:count_first] - start[None, :]))
        acc.add_linear_slice(offset_last, w_len * _norm_rows_xyz(last[:count_last] - goal[None, :]))

    objective_scale = max(1.0, acc.max_abs_term())
    acc.scale(objective_scale)
    max_touch = acc.max_touch_magnitude()
    one_hot_penalty = max(a_mult * max_touch + 1.0, 10.0)
    for layer in range(free_layers):
        offset_l, count_l = var_map[layer + 1]
        acc.linear[offset_l:offset_l + count_l] -= one_hot_penalty
        if count_l > 1:
            block = np.full((count_l, count_l), 2.0 * one_hot_penalty, dtype=np.float64)
            diag_idx = np.arange(count_l, dtype=np.intp)
            block[diag_idx, diag_idx] = 0.0
            rows = slice(offset_l, offset_l + count_l)
            # add_bilinear() mirrors a coefficient into both matrix halves.  A
            # full symmetric block is therefore the exact vectorised equivalent.
            acc.matrix[rows, rows] += block

    segment_penalty_used = min(max(0.0, float(segment_penalty)), one_hot_penalty * 0.5)
    if candidate_endpoint_blocked is not None and free_layers:
        first_blocked, last_blocked = candidate_endpoint_blocked
        offset_first, count_first = var_map[1]
        offset_last, count_last = var_map[total_layers - 2]
        if count_first:
            mask = np.asarray(first_blocked[:count_first], dtype=bool)
            acc.linear[offset_first:offset_first + count_first] += mask.astype(np.float64) * segment_penalty_used
        if count_last:
            mask = np.asarray(last_blocked[:count_last], dtype=bool)
            acc.linear[offset_last:offset_last + count_last] += mask.astype(np.float64) * segment_penalty_used
    for layer in range(free_layers - 1):
        offset_l, count_l = var_map[layer + 1]
        offset_r, count_r = var_map[layer + 2]
        if not count_l or not count_r:
            continue
        mask = np.asarray(candidate_blocked_pairs[layer], dtype=np.float64)[:count_l, :count_r]
        if mask.size:
            block = mask * segment_penalty_used
            acc.add_bilinear_block(
                slice(offset_l, offset_l + count_l),
                slice(offset_r, offset_r + count_r),
                block,
            )

    # V2: accumulator already stores the QUBO diagonal (linear) and symmetric
    # zero-diagonal off terms. Convert them directly to Ising h/J instead of
    # materialising Q and immediately rebuilding Q+Q.T inside make_qubo().
    problem = make_qubo_from_terms(
        acc.linear, acc.matrix, variable_count, name, max_energy=1.0e9, keep_numpy=True
    )
    h_arr = np.asarray(problem.h, dtype=np.float64)
    j_arr = np.asarray(problem.j, dtype=np.float64)
    max_abs = max(float(np.max(np.abs(h_arr), initial=0.0)),
                  float(np.max(np.abs(j_arr), initial=0.0)))
    if max_abs > 0.6:
        factor = 0.5 / max_abs
        # h/j are freshly allocated by make_qubo(keep_numpy=True); scale them
        # in place to avoid another padded 256x256 allocation on every replan.
        h_arr *= factor
        j_arr *= factor
        problem.h = h_arr
        problem.j = j_arr
        problem.fpga_scale = float(problem.fpga_scale) * factor
        problem.metadata = dict(problem.metadata)
        problem.metadata["bfp_rescale"] = factor
        problem._validated = True
        problem._load_payload_cache = None
        problem._uav_compact_payload_cache = None

    problem.metadata.update({
        "encoding": "kino waypoint layer one-hot",
        "objective": (
            "sum_i (w_len*|dp_i| + w_vel*|dp_i|^2/dt^2 + w_acc*|d2p_i|^2/dt^4) "
            "+ w_obs*clearance + w_spine*reference + segment_penalty + A*(sum_j x_ij - 1)^2"
        ),
        "layers": total_layers,
        "free_layers": free_layers,
        "variables": variable_count,
        "dt_s": dt,
        "one_hot_penalty": one_hot_penalty,
        "objective_scale": objective_scale,
        "w_vel": w_vel,
        "w_acc": w_acc,
        "w_obs": w_obs,
        "w_spine": w_spine,
        "w_len": w_len,
        "incremental_qubo": True,
        "incremental_qubo_static_hit": bool(static_hit),
        "incremental_qubo_static_build_ms": float(0.0 if static_hit else entry.build_ms),
        "incremental_qubo_static_hits": int(entry.hits),
        "incremental_qubo_cache_entries": int(len(static_cache)),
    })
    return problem, var_map, one_hot_penalty, 0.0


def _build_kino_qubo_reference(
    *,
    fixed_positions: Sequence[Point3],  # layers 0 and last are fixed
    candidate_positions: Sequence[Sequence[Point3]],
    candidate_obstacle_costs: Sequence[Sequence[float]],
    candidate_reference_costs: Sequence[Sequence[float]] | None = None,
    candidate_blocked_pairs: Sequence[Sequence[Sequence[bool]]],
    candidate_endpoint_blocked: tuple[Sequence[bool], Sequence[bool]] | None = None,
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
    start_vel_xyz: Point3,
    start_acc_xyz: Point3,
    goal_vel_xyz: Point3,
    a_mult: float = 3.0,
    name: str = "uav_kino_waypoint",
) -> tuple[Problem, dict[int, tuple[int, int]], float, float]:
    """Build the layer-wise one-hot QUBO.

    Layer layout: layer 0 = start (fixed), layers 1..F free, layer F+1 = goal
    (fixed).  Returns (problem, var_map, one_hot_penalty, objective_offset).
    """
    free_layers = len(candidate_positions)
    total_layers = free_layers + 2
    positions: list[object] = list(fixed_positions[:1])  # layer 0
    positions.extend(candidate_positions)  # layers 1..F
    positions.append(fixed_positions[-1])  # last layer
    var_map: dict[int, tuple[int, int] | None] = {0: None, total_layers - 1: None}
    variable_count = sum(len(item) for item in candidate_positions)
    offset = 0
    for layer in range(1, total_layers - 1):
        count = len(candidate_positions[layer - 1])
        var_map[layer] = (offset, count)
        offset += count

    acc = _QuadraticAccumulator(variable_count)
    dt2 = dt * dt
    dt4 = dt2 * dt2

    # --- velocity energy: w_vel * |p[i+1]-p[i]|^2 / dt^2 --------------------
    for layer in range(total_layers - 1):
        weight = w_vel / dt2
        acc.add_dot(layer, layer, weight, positions, var_map)
        acc.add_dot(layer + 1, layer + 1, weight, positions, var_map)
        acc.add_dot(layer, layer + 1, -2.0 * weight, positions, var_map)

    # --- acceleration energy: w_acc * |p[i+2]-2p[i+1]+p[i]|^2 / dt^4 --------
    for layer in range(total_layers - 2):
        weight = w_acc / dt4
        acc.add_dot(layer, layer, weight, positions, var_map)
        acc.add_dot(layer + 1, layer + 1, 4.0 * weight, positions, var_map)
        acc.add_dot(layer + 2, layer + 2, weight, positions, var_map)
        acc.add_dot(layer, layer + 1, -4.0 * weight, positions, var_map)
        acc.add_dot(layer, layer + 2, 2.0 * weight, positions, var_map)
        acc.add_dot(layer + 1, layer + 2, -4.0 * weight, positions, var_map)

    # --- start velocity: |p1 - (p0 + v0*dt)|^2 -------------------------------
    start_target = (
        fixed_positions[0][0] + start_vel_xyz[0] * dt,
        fixed_positions[0][1] + start_vel_xyz[1] * dt,
        fixed_positions[0][2] + start_vel_xyz[2] * dt,
    )
    if w_start_vel > 0.0 and free_layers >= 1:
        acc.add_dot(1, 1, w_start_vel, positions, var_map)
        offset1, _count1 = var_map[1]
        for index, q in enumerate(candidate_positions[0]):
            acc.add_linear(offset1 + index, -2.0 * w_start_vel * (q[0] * start_target[0] + q[1] * start_target[1] + q[2] * start_target[2]))

    # --- start acceleration: |(p2 - 2p1 + p0)/dt^2 - a0|^2 -------------------
    if w_start_acc > 0.0 and free_layers >= 2:
        weight = w_start_acc / dt4
        acc.add_dot(2, 2, weight, positions, var_map)
        acc.add_dot(1, 1, 4.0 * weight, positions, var_map)
        acc.add_dot(2, 1, -4.0 * weight, positions, var_map)
        acc.add_dot(2, 0, 2.0 * weight, positions, var_map)
        acc.add_dot(1, 0, -4.0 * weight, positions, var_map)
        offset1, _count1 = var_map[1]
        offset2, _count2 = var_map[2]
        for index, q in enumerate(candidate_positions[0]):
            acc.add_linear(offset1 + index, 4.0 * weight * (q[0] * start_acc_xyz[0] + q[1] * start_acc_xyz[1] + q[2] * start_acc_xyz[2]) * dt2)
        for index, q in enumerate(candidate_positions[1]):
            acc.add_linear(offset2 + index, -2.0 * weight * (q[0] * start_acc_xyz[0] + q[1] * start_acc_xyz[1] + q[2] * start_acc_xyz[2]) * dt2)

    # --- end velocity: |p_last - p_{last-1}|^2 (goal velocity ~ 0) -----------
    if w_end_vel > 0.0 and free_layers >= 1:
        acc.add_dot(total_layers - 1, total_layers - 1, w_end_vel, positions, var_map)
        acc.add_dot(total_layers - 2, total_layers - 2, w_end_vel, positions, var_map)
        acc.add_dot(total_layers - 2, total_layers - 1, -2.0 * w_end_vel, positions, var_map)

    # --- obstacle cost (linear per candidate) ---------------------------------
    for layer in range(free_layers):
        offset_l, count_l = var_map[layer + 1]
        for index, cost in enumerate(candidate_obstacle_costs[layer]):
            if cost > 0.0:
                acc.add_linear(offset_l + index, w_obs * cost)

    # --- explicit geometric path length --------------------------------------
    # For adjacent one-hot layers, Euclidean segment length is exactly a QUBO
    # term: fixed<->candidate edges become linear coefficients and
    # candidate<->candidate edges become bilinear coefficients. Unlike the
    # squared-velocity objective this directly rewards a shorter route, allowing
    # the Ising solution to improve on the discrete A* backbone instead of only
    # imitating it through a large spine penalty.
    if w_len > 0.0:
        for layer in range(total_layers - 1):
            left_map = var_map[layer]
            right_map = var_map[layer + 1]
            left = np.asarray(positions[layer], dtype=np.float64)
            right = np.asarray(positions[layer + 1], dtype=np.float64)
            if left_map is None and right_map is None:
                continue
            if left_map is None:
                offset_r, count_r = right_map
                d = _norm_rows_xyz(right - left[None, :])
                acc.add_linear_slice(offset_r, w_len * d[:count_r])
                continue
            if right_map is None:
                offset_l, count_l = left_map
                d = _norm_rows_xyz(left - right[None, :])
                acc.add_linear_slice(offset_l, w_len * d[:count_l])
                continue
            offset_l, count_l = left_map
            offset_r, count_r = right_map
            delta = left[:count_l, None, :] - right[None, :count_r, :]
            d = np.sqrt(np.sum(delta * delta, axis=2))
            acc.add_bilinear_block(
                slice(offset_l, offset_l + count_l),
                slice(offset_r, offset_r + count_r),
                w_len * d,
            )

    # --- reference-spine regularisation (linear per candidate) ----------------
    # The A* backbone is a known feasible, usually short route.  A light unary
    # penalty for drifting far from that backbone removes gratuitous zig-zags
    # without preventing the QUBO from moving laterally for clearance/smoothing.
    if candidate_reference_costs is not None and w_spine > 0.0:
        for layer in range(min(free_layers, len(candidate_reference_costs))):
            offset_l, count_l = var_map[layer + 1]
            costs = candidate_reference_costs[layer]
            for index in range(min(count_l, len(costs))):
                cost = float(costs[index])
                if cost > 0.0:
                    acc.add_linear(offset_l + index, w_spine * cost)

    # --- one-hot per free layer: A*(sum_j x_j - 1)^2 --------------------------
    # Objective coefficients are normalised to O(1) and the constraint penalty
    # is derived from the maximum objective change a single-variable flip can
    # cause, so the annealer still resolves path quality instead of being
    # drowned by position-magnitude terms (|q|^2 can be hundreds of m^2).
    objective_scale = max(1.0, acc.max_abs_term())
    acc.scale(objective_scale)
    max_touch = acc.max_touch_magnitude()
    # one-hot penalty A = a_mult * max_touch.  a_mult is deliberately
    # configurable: a stronger A makes the annealer keep each layer at exactly
    # one candidate (fewer "broken" layers for the argmax decode), at the cost
    # of a harder landscape; 1.5x is the relaxed default.
    one_hot_penalty = max(a_mult * max_touch + 1.0, 10.0)
    for layer in range(free_layers):
        offset_l, count_l = var_map[layer + 1]
        for index in range(count_l):
            acc.add_linear(offset_l + index, -one_hot_penalty)
        for ja in range(count_l):
            for jb in range(ja + 1, count_l):
                acc.add_bilinear(offset_l + ja, offset_l + jb, 2.0 * one_hot_penalty)

    # --- segment-safety penalty (bilinear between adjacent layers) -----------
    # Constraint-like term, clamped well below the one-hot penalty so the
    # annealer keeps resolving the real objective.
    segment_penalty = min(max(0.0, float(segment_penalty)), one_hot_penalty * 0.5)
    # Start->layer1 and layerN->goal have one fixed endpoint, therefore the
    # same safety constraint becomes a linear term.  This was missing in the
    # previous ROS model and caused valid-looking anneals to be rejected only
    # during the final host validation.
    if candidate_endpoint_blocked is not None and free_layers:
        first_blocked, last_blocked = candidate_endpoint_blocked
        offset_first, count_first = var_map[1]
        offset_last, count_last = var_map[total_layers - 2]
        for index in range(min(count_first, len(first_blocked))):
            if bool(first_blocked[index]):
                acc.add_linear(offset_first + index, segment_penalty)
        for index in range(min(count_last, len(last_blocked))):
            if bool(last_blocked[index]):
                acc.add_linear(offset_last + index, segment_penalty)
    for layer in range(free_layers - 1):
        offset_l, count_l = var_map[layer + 1]
        offset_r, count_r = var_map[layer + 2]
        for ja in range(count_l):
            for jb in range(count_r):
                if candidate_blocked_pairs[layer][ja][jb]:
                    acc.add_bilinear(offset_l + ja, offset_r + jb, segment_penalty)

    matrix = acc.to_matrix()
    # High max_energy for make_qubo: avoid its energy-bound shrink, then apply
    # our own uniform rescale to a moderate BFP-friendly amplitude.  Scaling to
    # ~0.5 (instead of 0.03) keeps coefficients large enough that the annealer
    # can resolve the one-hot constraints, while BFP16's 5e-5 error guard still
    # passes (shift=6), so the payload stays compact (BFP16 / sparse-BFP16).
    problem = make_qubo(matrix, variable_count, name, max_energy=1.0e9, keep_numpy=True)
    h_arr = np.asarray(problem.h, dtype=np.float64)
    j_arr = np.asarray(problem.j, dtype=np.float64)
    max_abs = max(float(np.max(np.abs(h_arr), initial=0.0)),
                  float(np.max(np.abs(j_arr), initial=0.0)))
    if max_abs > 0.6:
        factor = 0.5 / max_abs
        problem.h = h_arr * factor
        problem.j = j_arr * factor
        problem.fpga_scale = float(problem.fpga_scale) * factor
        problem.metadata = dict(problem.metadata)
        problem.metadata["bfp_rescale"] = factor
        # Uniform scaling preserves all invariants already checked by
        # make_qubo (finite, symmetric J, zero diagonal).  Marking the problem
        # validated avoids rescanning the padded 256x256 matrix before every
        # Active-N payload build.
        problem._validated = True
        problem._load_payload_cache = None
        problem._uav_compact_payload_cache = None
    problem.metadata.update(
        {
            "encoding": "kino waypoint layer one-hot",
            "objective": (
                "sum_i (w_len*|dp_i| + w_vel*|dp_i|^2/dt^2 + w_acc*|d2p_i|^2/dt^4) "
                "+ w_obs*clearance + w_spine*reference + segment_penalty + A*(sum_j x_ij - 1)^2"
            ),
            "layers": total_layers,
            "free_layers": free_layers,
            "variables": variable_count,
            "dt_s": dt,
            "one_hot_penalty": one_hot_penalty,
            "objective_scale": objective_scale,
            "w_vel": w_vel,
            "w_acc": w_acc,
            "w_obs": w_obs,
            "w_spine": w_spine,
            "w_len": w_len,
        }
    )
    return problem, var_map, one_hot_penalty, 0.0



def _build_kino_qubo_native(**kwargs):
    """ABI-2801 full active-N C++ QUBO builder.

    The native kernel evaluates the same velocity/acceleration/length/obstacle/
    one-hot/segment polynomial as the Python reference.  It is intentionally
    optional: any ABI/shape failure returns to the proven incremental Python
    path without changing flight behaviour.
    """
    if _native_fastpath is None or not _native_fastpath.available():
        return None
    native = _native_fastpath.kino_qubo_active(
        fixed_positions=kwargs["fixed_positions"],
        candidate_positions=kwargs["candidate_positions"],
        candidate_obstacle_costs=kwargs["candidate_obstacle_costs"],
        candidate_reference_costs=kwargs.get("candidate_reference_costs"),
        candidate_blocked_pairs=kwargs["candidate_blocked_pairs"],
        candidate_endpoint_blocked=kwargs.get("candidate_endpoint_blocked"),
        dt=kwargs["dt"], w_vel=kwargs["w_vel"], w_acc=kwargs["w_acc"],
        w_obs=kwargs["w_obs"], w_spine=kwargs["w_spine"], w_len=kwargs["w_len"],
        w_start_vel=kwargs["w_start_vel"], w_start_acc=kwargs["w_start_acc"],
        w_end_vel=kwargs["w_end_vel"], segment_penalty=kwargs["segment_penalty"],
        start_vel_xyz=kwargs["start_vel_xyz"], start_acc_xyz=kwargs["start_acc_xyz"],
        a_mult=kwargs.get("a_mult", 3.0),
    )
    if native is None:
        return None
    n=int(native["n"])
    free_layers,total_layers,var_map,variable_count=_kino_var_layout(kwargs["candidate_positions"])
    if n != variable_count:
        return None
    # Keep the legacy 256-padded Problem ABI so every solver/diagnostic/export
    # path remains compatible.  The expensive polynomial construction itself
    # has already happened in C++ on active N only.
    h0=np.zeros(N_HW,dtype=np.float64); h0[:n]=native["h"]
    j0=np.zeros((N_HW,N_HW),dtype=np.float64); j0[:n,:n]=native["j"]
    source=np.asarray(native["matrix"],dtype=np.float64).copy()
    source[np.arange(n,dtype=np.intp),np.arange(n,dtype=np.intp)]=native["linear"]
    factor=float(native.get("bfp_rescale",1.0) or 1.0)
    problem=Problem(
        kwargs.get("name","uav_kino_waypoint"),"qubo",n,h0,j0,source,
        {
            "objective":"x^TQx",
            "encoding":"kino waypoint layer one-hot",
            "layers":total_layers,"free_layers":free_layers,"variables":n,
            "dt_s":float(kwargs["dt"]),
            "one_hot_penalty":float(native["one_hot_penalty"]),
            "objective_scale":float(native["objective_scale"]),
            "w_vel":float(kwargs["w_vel"]),"w_acc":float(kwargs["w_acc"]),
            "w_obs":float(kwargs["w_obs"]),"w_spine":float(kwargs["w_spine"]),
            "w_len":float(kwargs["w_len"]),
            "native_qubo":True,"native_qubo_version":int(native.get("native_abi",3001)),
            "incremental_qubo":True,"incremental_qubo_static_hit":False,
            "incremental_qubo_static_build_ms":0.0,"incremental_qubo_static_hits":0,
            "incremental_qubo_cache_entries":0,
            **({"bfp_rescale":factor} if factor != 1.0 else {}),
        },
        factor,
    )
    problem._validated=True
    native_fp=native.get("logical_problem_fingerprints")
    if native_fp is not None and len(native_fp)==2:
        problem._uav_logical_fingerprint_cache=(int(native_fp[0]),int(native_fp[1]))
    return problem,var_map,float(native["one_hot_penalty"]),0.0


def build_kino_qubo(
    *,
    static_cache: OrderedDict[bytes, _KinoQuboStaticEntry] | None = None,
    incremental_qubo_cache_size: int = 32,
    **kwargs,
) -> tuple[Problem, dict[int, tuple[int, int]], float, float]:
    """Host-accelerated QUBO builder with Python reference fallback."""
    try:
        native_result=_build_kino_qubo_native(**kwargs)
        if native_result is not None:
            return native_result
    except Exception:
        # Native QUBO is a performance feature, never a correctness dependency.
        pass
    if static_cache is None:
        problem, var_map, penalty, offset = _build_kino_qubo_reference(**kwargs)
        problem.metadata = dict(problem.metadata)
        problem.metadata.update({
            "incremental_qubo": False,
            "incremental_qubo_static_hit": False,
            "incremental_qubo_static_build_ms": 0.0,
            "incremental_qubo_static_hits": 0,
            "incremental_qubo_cache_entries": 0,
        })
        return problem, var_map, penalty, offset
    try:
        return _build_kino_qubo_incremental(
            static_cache=static_cache,
            cache_size=max(1, int(incremental_qubo_cache_size)),
            **kwargs,
        )
    except Exception:
        # Preserve the stable branch semantics if an experimental cache path
        # ever sees an unexpected shape/type at runtime.
        problem, var_map, penalty, offset = _build_kino_qubo_reference(**kwargs)
        problem.metadata = dict(problem.metadata)
        problem.metadata.update({
            "incremental_qubo": False,
            "incremental_qubo_fallback": True,
            "incremental_qubo_static_hit": False,
            "incremental_qubo_static_build_ms": 0.0,
            "incremental_qubo_static_hits": 0,
            "incremental_qubo_cache_entries": len(static_cache),
        })
        return problem, var_map, penalty, offset


def decode_kino_spins(
    binary: Sequence[int],
    var_map: dict[int, tuple[int, int] | None],
    total_layers: int,
) -> tuple[Optional[list[int]], bool]:
    """Return (chosen candidate index per free layer, onehot_valid)."""
    chosen: list[int] = []
    valid = True
    for layer in range(1, total_layers - 1):
        offset, count = var_map[layer]
        selected = [index for index in range(count) if int(binary[offset + index]) > 0]
        if len(selected) == 1:
            chosen.append(selected[0])
        elif len(selected) > 1:
            valid = False
            chosen.append(max(selected, key=lambda index: int(binary[offset + index])))
        else:
            valid = False
            chosen.append(0)
    return chosen, valid


def _central_velocities(points: Sequence[Point3], dt: float) -> list[Point3]:
    n = len(points)
    velocities: list[Point3] = []
    for index in range(n):
        if index == 0:
            left = points[0]
            right = points[1] if n > 1 else points[0]
        elif index == n - 1:
            left = points[n - 2]
            right = points[n - 1]
        else:
            left = points[index - 1]
            right = points[index + 1]
        velocities.append(
            ((right[0] - left[0]) / (2.0 * dt),
             (right[1] - left[1]) / (2.0 * dt),
             (right[2] - left[2]) / (2.0 * dt))
        )
    return velocities


def _central_accelerations(points: Sequence[Point3], velocities: Sequence[Point3], dt: float) -> list[Point3]:
    n = len(points)
    accelerations: list[Point3] = []
    for index in range(n):
        if index == 0 or index == n - 1:
            v_left = velocities[index]
            v_right = velocities[1] if index == 0 else velocities[n - 2]
            accelerations.append(((v_right[0] - v_left[0]) / dt, (v_right[1] - v_left[1]) / dt, (v_right[2] - v_left[2]) / dt))
        else:
            v_left = velocities[index - 1]
            v_right = velocities[index + 1]
            accelerations.append(((v_right[0] - v_left[0]) / (2.0 * dt), (v_right[1] - v_left[1]) / (2.0 * dt), (v_right[2] - v_left[2]) / (2.0 * dt)))
    return accelerations


def _dense_cubic_hermite(
    points: Sequence[Point3],
    velocities: Sequence[Point3],
    dt: float,
    dt_sample: float,
) -> list[Point3]:
    """Piecewise cubic Hermite interpolation sampled at dt_sample."""
    samples: list[Point3] = [points[0]]
    n = len(points)
    for index in range(n - 1):
        p0 = points[index]
        p1 = points[index + 1]
        v0 = velocities[index]
        v1 = velocities[index + 1]
        step = dt_sample / max(dt, 1.0e-12)
        t = step
        while t < 1.0 - 1.0e-9:
            t2 = t * t
            t3 = t2 * t
            h00 = 2.0 * t3 - 3.0 * t2 + 1.0
            h10 = t3 - 2.0 * t2 + t
            h01 = -2.0 * t3 + 3.0 * t2
            h11 = t3 - t2
            samples.append((
                h00 * p0[0] + h10 * dt * v0[0] + h01 * p1[0] + h11 * dt * v1[0],
                h00 * p0[1] + h10 * dt * v0[1] + h01 * p1[1] + h11 * dt * v1[1],
                h00 * p0[2] + h10 * dt * v0[2] + h01 * p1[2] + h11 * dt * v1[2],
            ))
            t += step
        samples.append(p1)
    return samples


def _dense_linear(
    points: Sequence[Point3],
    velocities: Sequence[Point3],
    dt: float,
    dt_sample: float,
) -> tuple[list[Point3], list[Point3]]:
    """Piecewise-linear dense samples (constant segment velocity).  Used as a
    fallback when cubic-Hermite interpolation overshoots the speed limit."""
    samples: list[Point3] = [points[0]]
    vel_samples: list[Point3] = []
    n = len(points)
    if n < 2:
        return samples, [velocities[0] if velocities else (0.0, 0.0, 0.0)]
    for index in range(n - 1):
        p0 = points[index]
        p1 = points[index + 1]
        # Linear interpolation has a constant *segment* velocity.  Reusing the
        # central waypoint velocity here under-reported corner speeds and made
        # acceleration diagnostics inconsistent with the published positions.
        v = (
            (p1[0] - p0[0]) / max(dt, 1.0e-12),
            (p1[1] - p0[1]) / max(dt, 1.0e-12),
            (p1[2] - p0[2]) / max(dt, 1.0e-12),
        )
        if index == 0:
            vel_samples.append(v)  # velocity for samples[0]
        step = dt_sample / max(dt, 1.0e-12)
        t = step
        while t < 1.0 - 1.0e-9:
            samples.append((
                p0[0] + (p1[0] - p0[0]) * t,
                p0[1] + (p1[1] - p0[1]) * t,
                p0[2] + (p1[2] - p0[2]) * t,
            ))
            vel_samples.append(v)
            t += step
        # Keep the segment endpoint exactly, matching _dense_cubic_hermite.
        samples.append(p1)
        vel_samples.append(v)
    return samples, vel_samples


def _build_dense(
    waypoints: Sequence[Point3],
    velocities: Sequence[Point3],
    dt: float,
    dt_sample: float,
    max_vel: float,
) -> tuple[Sequence[Point3], Sequence[Point3], Sequence[Point3], float]:
    """Hermite-densify with an ABI-3201 native exact fast path.

    The Python implementation remains the correctness fallback.  The native
    kernel preserves the same sample clock, Hermite coefficients, linear
    overspeed fallback and dense acceleration operator; it mainly removes the
    long Python loops when ``trajectory_dt`` expands to seconds.
    """
    if _native_fastpath is not None:
        native = _native_fastpath.dense_trajectory3d(
            waypoints, velocities, dt, dt_sample, max_vel, linear_only=False
        )
        if native is not None:
            dense, dense_vel, dense_acc, dense_max_speed = native
            return dense, dense_vel, dense_acc, float(dense_max_speed)
    dense = _dense_cubic_hermite(waypoints, velocities, dt, dt_sample)
    dense_vel = _sample_velocities(waypoints, velocities, dt, dt_sample)
    dense_max_speed = _dense_step_speed(dense, dt_sample)
    if dense_max_speed > max_vel * 1.05 + 1.0e-6:
        dense, dense_vel = _dense_linear(waypoints, velocities, dt, dt_sample)
        dense_max_speed = _dense_step_speed(dense, dt_sample)
    dense_acc = _central_accelerations(dense, dense_vel, dt_sample)
    return dense, dense_vel, dense_acc, float(dense_max_speed)


def _dense_step_speed(points: Sequence[Point3], dt_sample: float) -> float:
    if len(points) < 2:
        return 0.0
    arr = np.asarray(points, dtype=np.float64)
    step = np.sqrt(np.sum(np.diff(arr, axis=0) ** 2, axis=1))
    return float(np.max(step) / max(dt_sample, 1.0e-12))


def _sample_velocities(
    points: Sequence[Point3],
    velocities: Sequence[Point3],
    dt: float,
    dt_sample: float,
) -> list[Point3]:
    samples: list[Point3] = []
    n = len(points)
    for index in range(n - 1):
        p0 = points[index]
        p1 = points[index + 1]
        v0 = velocities[index]
        v1 = velocities[index + 1]
        step = dt_sample / max(dt, 1.0e-12)
        t = 0.0
        while t < 1.0 - 1.0e-9:
            v = (
                (1.0 - t) * v0[0] + t * v1[0],
                (1.0 - t) * v0[1] + t * v1[1],
                (1.0 - t) * v0[2] + t * v1[2],
            )
            samples.append(v)
            t += step
    samples.append(velocities[-1])
    return samples


def _repair_blocked_segments(
    chosen: list[int],
    candidate_positions: Sequence[Sequence[Point3]],
    spine_points: Sequence[Point3],
    occupancy: np.ndarray,
    frame: GridFrame3D,
    max_passes: int = 4,
) -> list[int]:
    """Deterministic host repair: replace candidates of blocked adjacent-layer
    segments with safe alternatives (preferring candidates near the spine).

    Unlike a one-sided greedy, this searches *joint* (left, right) replacements
    per blocked segment, so a detour through the spine voxel can be taken even
    when neither side alone can fix the pair.  Mirrors the BFS repair helpers
    used for the 2D encodings; keeps the FPGA-chosen path when possible.
    """
    chosen = list(chosen)
    free_layers = len(chosen)
    for _pass in range(max_passes):
        changed = False
        for layer in range(free_layers - 1):
            left = chosen[layer]
            right = chosen[layer + 1]
            a = candidate_positions[layer][left]
            b = candidate_positions[layer + 1][right]
            if not _segment_blocked_zyx(a, b, occupancy, frame):
                continue
            n_left = len(candidate_positions[layer])
            n_right = len(candidate_positions[layer + 1])
            spine_l = spine_points[layer]
            spine_r = spine_points[layer + 1]
            # 1) joint search for any safe (j, k) pair, spine-near first
            best: tuple[int, int] | None = None
            best_key: tuple[float, int] | None = None
            for j in range(n_left):
                aj = candidate_positions[layer][j]
                dj = math.dist(aj, spine_l)
                for k in range(n_right):
                    if _segment_blocked_zyx(aj, candidate_positions[layer + 1][k], occupancy, frame):
                        continue
                    key = (dj + math.dist(candidate_positions[layer + 1][k], spine_r),
                           (j != left) + (k != right))
                    if best is None or key < best_key:
                        best = (j, k)
                        best_key = key
            if best is not None:
                if best != (left, right):
                    chosen[layer], chosen[layer + 1] = best
                    changed = True
                continue
            # 2) no safe pair inside this layer window: pull the right side to
            #    its spine voxel (spine adjacency is the connectivity guarantee)
            spine_idx = min(range(n_right),
                            key=lambda k: math.dist(candidate_positions[layer + 1][k], spine_r))
            if spine_idx != right:
                chosen[layer + 1] = spine_idx
                changed = True
        if not changed:
            break
    return chosen


def _repair_endpoint_segments(
    chosen: list[int],
    candidate_positions: Sequence[Sequence[Point3]],
    start_xyz: Point3,
    goal_xyz: Point3,
    occupancy: np.ndarray,
    frame: GridFrame3D,
) -> list[int]:
    """Repair start->layer0 / last-layer->goal links without discarding the
    whole FPGA path.  Candidates are ranked by displacement from the selected
    point, so this is a minimal local correction rather than a second planner.
    """
    chosen = list(chosen)
    if not chosen:
        return chosen

    def _rank(layer: int, current: int) -> list[int]:
        ref = candidate_positions[layer][current]
        return sorted(
            range(len(candidate_positions[layer])),
            key=lambda idx: math.dist(ref, candidate_positions[layer][idx]),
        )

    # Start endpoint: require start->candidate and, when present, candidate->next.
    first = chosen[0]
    first_pt = candidate_positions[0][first]
    if _segment_blocked_zyx(start_xyz, first_pt, occupancy, frame):
        for idx in _rank(0, first):
            point = candidate_positions[0][idx]
            if _segment_blocked_zyx(start_xyz, point, occupancy, frame):
                continue
            if len(chosen) > 1:
                nxt = candidate_positions[1][chosen[1]]
                if _segment_blocked_zyx(point, nxt, occupancy, frame):
                    continue
            chosen[0] = idx
            break

    # Goal endpoint: symmetric check.
    last_layer = len(chosen) - 1
    last = chosen[last_layer]
    last_pt = candidate_positions[last_layer][last]
    if _segment_blocked_zyx(last_pt, goal_xyz, occupancy, frame):
        for idx in _rank(last_layer, last):
            point = candidate_positions[last_layer][idx]
            if _segment_blocked_zyx(point, goal_xyz, occupancy, frame):
                continue
            if len(chosen) > 1:
                prev = candidate_positions[last_layer - 1][chosen[last_layer - 1]]
                if _segment_blocked_zyx(prev, point, occupancy, frame):
                    continue
            chosen[last_layer] = idx
            break
    return chosen


def _validate_path(
    waypoints_xyz: Sequence[Point3],
    occupancy: np.ndarray,
    frame: GridFrame3D,
    *,
    segment_samples: int = 8,
) -> bool:
    for left, right in zip(waypoints_xyz, waypoints_xyz[1:]):
        if _segment_blocked_zyx(left, right, occupancy, frame, samples=segment_samples):
            return False
    return True


def _anchor_exact_endpoints_if_safe(
    waypoints_xyz: Sequence[Point3],
    exact_start: Point3,
    exact_goal: Point3,
    occupancy: np.ndarray,
    frame: GridFrame3D,
) -> tuple[list[Point3], bool, bool]:
    """Restore continuous ROS endpoints without weakening fallback safety.

    The endpoint logic keeps the direct-replacement fast path, but treats start and goal
    bridges independently.  The previous implementation discarded a safe goal
    bridge whenever the start endpoint could not also be anchored under the
    full-path validator.  That was unnecessarily coupled: the original
    fallback segments are already accepted, and a centre<->exact bridge wholly
    inside the same free voxel can be validated locally without changing any
    other segment.

    Returns ``(path, start_anchored, goal_anchored)``.
    """
    out = [tuple(map(float, p)) for p in waypoints_xyz]
    if len(out) < 2:
        return out, False, False
    exact_start = tuple(map(float, exact_start))
    exact_goal = tuple(map(float, exact_goal))
    eps = 1.0e-12
    start_same = math.dist(out[0], exact_start) <= eps
    goal_same = math.dist(out[-1], exact_goal) <= eps
    if start_same and goal_same:
        return out, True, True

    # Preferred form: preserve waypoint count by replacing both endpoints when
    # the complete replacement passes the existing validator.
    candidate = list(out)
    candidate[0] = exact_start
    candidate[-1] = exact_goal
    if _validate_path(candidate, occupancy, frame):
        return candidate, True, True

    bridged = list(out)
    start_ok = start_same
    goal_ok = goal_same

    def _same_free_cell(a: Point3, b: Point3) -> bool:
        try:
            ca = xyz_to_cell3d(a[0], a[1], a[2], frame)
            cb = xyz_to_cell3d(b[0], b[1], b[2], frame)
        except Exception:
            return False
        if ca != cb:
            return False
        z, r, c = cb
        return (
            0 <= z < occupancy.shape[0]
            and 0 <= r < occupancy.shape[1]
            and 0 <= c < occupancy.shape[2]
            and not bool(occupancy[z, r, c])
        )

    # Only validate the newly added short segment.  All original fallback
    # segments remain byte-for-byte/geometrically unchanged and have already
    # passed their normal safety gates.
    if not start_ok and _same_free_cell(bridged[0], exact_start):
        if not _segment_blocked_zyx(exact_start, bridged[0], occupancy, frame, samples=8):
            bridged.insert(0, exact_start)
            start_ok = True

    if not goal_ok and _same_free_cell(bridged[-1], exact_goal):
        if not _segment_blocked_zyx(bridged[-1], exact_goal, occupancy, frame, samples=8):
            bridged.append(exact_goal)
            goal_ok = True

    return bridged, bool(start_ok), bool(goal_ok)


def _tighten_trajectory_dt(
    waypoints: Sequence[Point3], current_dt: float, max_vel: float, max_acc: float,
    *, floor_s: float = 0.18,
) -> float:
    """Shrink an overly conservative solve dt using the *selected* geometry.

    The initial dt is based on the voxel spine's worst turn.  The final selected
    path is often smoother, so it can be safely re-timed.  We solve the actual
    central-difference velocity/acceleration constraints rather than relying only
    on the simple second-difference estimate, preserving the existing hard guards.
    """
    if len(waypoints) < 2:
        return float(current_dt)
    max_segment = max((math.dist(a, b) for a, b in zip(waypoints, waypoints[1:])), default=0.0)
    second_diff = 0.0
    for a, b, c in zip(waypoints, waypoints[1:], waypoints[2:]):
        second_diff = max(second_diff, math.sqrt(
            (c[0] - 2.0*b[0] + a[0])**2 +
            (c[1] - 2.0*b[1] + a[1])**2 +
            (c[2] - 2.0*b[2] + a[2])**2
        ))
    dt_v = max_segment / max(0.3, 0.90 * max_vel)
    dt_a = math.sqrt(second_diff / max(0.3, 0.95 * max_acc)) if second_diff > 0.0 else 0.0
    target = min(float(current_dt), max(float(floor_s), dt_v, dt_a))
    # The central-acceleration operator used by the published trajectory is not
    # exactly the same as the raw second-difference formula at endpoints.  Scale
    # up analytically until the actual waypoint dynamics fit with margin.
    for _ in range(3):
        vel = _central_velocities(waypoints, target)
        acc = _central_accelerations(waypoints, vel, target)
        vmax = max((math.hypot(v[0], v[1], v[2]) for v in vel), default=0.0)
        amax = max((math.hypot(a[0], a[1], a[2]) for a in acc), default=0.0)
        scale = max(
            1.0,
            vmax / max(0.3, 0.90 * max_vel),
            math.sqrt(amax / max(0.3, 0.95 * max_acc)) if amax > 0.0 else 1.0,
        )
        if scale <= 1.0005:
            break
        target *= scale
    return min(float(current_dt), target)


def _retime_selected_trajectory(
    waypoints: Sequence[Point3],
    base_dt: float,
    dt_sample: float,
    max_vel: float,
    max_acc: float,
    *,
    speed_ratio_target: float = 0.96,
    max_expansions: int = 4,
    linear_only: bool = False,
) -> tuple[float, list[Point3], list[Point3], list[Point3], list[Point3], list[Point3], float, float, int]:
    """Retiming-only dynamics repair for a fixed geometric path."""
    speed_ratio_target = min(1.0, max(0.75, float(speed_ratio_target)))
    trajectory_dt = _tighten_trajectory_dt(waypoints, base_dt, max_vel, max_acc)
    expansions = 0
    dense: list[Point3] = []
    dense_vel: list[Point3] = []
    dense_acc: list[Point3] = []
    velocities: list[Point3] = []
    accelerations: list[Point3] = []
    dense_max_speed = 0.0
    waypoint_max_acc = 0.0
    for pass_index in range(max(1, int(max_expansions)) + 1):
        velocities = _central_velocities(waypoints, trajectory_dt)
        accelerations = _central_accelerations(waypoints, velocities, trajectory_dt)
        waypoint_max_acc = max(
            (math.hypot(a[0], a[1], a[2]) for a in accelerations), default=0.0
        )
        if linear_only:
            native_linear = None
            if _native_fastpath is not None:
                native_linear = _native_fastpath.dense_trajectory3d(
                    waypoints, velocities, trajectory_dt, dt_sample, max_vel, linear_only=True
                )
            if native_linear is not None:
                dense, dense_vel, dense_acc, dense_max_speed = native_linear
            else:
                dense, dense_vel = _dense_linear(
                    waypoints, velocities, trajectory_dt, dt_sample
                )
                dense_max_speed = _dense_step_speed(dense, dt_sample)
                dense_acc = _central_accelerations(dense, dense_vel, dt_sample)
        else:
            dense, dense_vel, dense_acc, dense_max_speed = _build_dense(
                waypoints, velocities, trajectory_dt, dt_sample, max_vel
            )
        speed_ratio = dense_max_speed / max(0.3, speed_ratio_target * max_vel)
        acc_ratio = waypoint_max_acc / max(0.3, 1.02 * max_acc)
        if speed_ratio <= 1.0 + 1.0e-6 and acc_ratio <= 1.0 + 1.0e-6:
            break
        if pass_index >= max_expansions:
            break
        # v ~ 1/dt and a ~ 1/dt^2.  Apply only the analytically necessary
        # increase plus a tiny numeric guard, rather than jumping 1.35x and
        # rebuilding the optimisation problem.
        scale = max(1.01, speed_ratio, math.sqrt(max(0.0, acc_ratio))) * 1.005
        trajectory_dt *= scale
        expansions += 1
    return (
        trajectory_dt, velocities, accelerations, dense, dense_vel, dense_acc,
        float(dense_max_speed), float(waypoint_max_acc), int(expansions),
    )


def _near_goal_direct_result(
    *,
    started: float,
    start_zyx: Cell3D,
    goal_zyx: Cell3D,
    frame: GridFrame3D,
    occupancy: np.ndarray,
    raw_occupancy: np.ndarray | None,
    endpoint_grace_voxels: int,
    start_vel_xyz: Point3,
    goal_vel_xyz: Point3,
    cruise_speed_mps: float,
    dt_sample: float,
    max_vel: float,
    max_acc: float,
    publish_speed_ratio: float = 0.96,
    max_cross_speed_mps: float = 0.5,
    exact_start_xyz: Point3 | None = None,
    exact_goal_xyz: Point3 | None = None,
) -> KinoPlanResult | None:
    """Return the geometrically shortest straight near-goal trajectory.

    The fast path is used only when the complete start->goal segment is clear in
    the inflated occupancy.  Three collinear waypoints keep the geometry exactly
    straight while providing a midpoint for the existing velocity/acceleration
    model.  No A*, QUBO or FPGA transaction is needed for this trivial case.
    """
    safety_mode = "inflated"
    if not _segment_clear_cells(start_zyx, goal_zyx, occupancy):
        if raw_occupancy is None or not _segment_clear_with_endpoint_grace(
            start_zyx, goal_zyx, occupancy, raw_occupancy,
            grace_voxels=endpoint_grace_voxels,
        ):
            return None
        safety_mode = "raw-endpoint-grace"
    start_xyz = exact_start_xyz if exact_start_xyz is not None else cell3d_to_xyz(start_zyx, frame)
    goal_xyz = exact_goal_xyz if exact_goal_xyz is not None else cell3d_to_xyz(goal_zyx, frame)
    midpoint = tuple((a + b) * 0.5 for a, b in zip(start_xyz, goal_xyz))
    waypoints: list[Point3] = [start_xyz, midpoint, goal_xyz]
    direct_length = math.dist(start_xyz, goal_xyz)
    cross_speed = 0.0
    if direct_length > 1.0e-9:
        unit = tuple((b - a) / direct_length for a, b in zip(start_xyz, goal_xyz))
        forward_speed = sum(float(v) * float(u) for v, u in zip(start_vel_xyz, unit))
        cross = tuple(float(v) - forward_speed * float(u) for v, u in zip(start_vel_xyz, unit))
        cross_speed = math.sqrt(sum(value * value for value in cross))
        # A straight bypass should not demand an instantaneous large heading
        # change from the measured state.  If the current lateral velocity is
        # incompatible, fall back to the tiny kinodynamic QUBO which explicitly
        # carries the start-velocity term.
        if cross_speed > max(0.0, float(max_cross_speed_mps)) or forward_speed < -0.2:
            return None
    target_speed = min(max(0.3, cruise_speed_mps), max(0.3, 0.90 * max_vel))
    base_dt = max(0.18, (0.5 * direct_length) / target_speed)
    (
        trajectory_dt, velocities, accelerations, dense, dense_vel, dense_acc,
        dense_max_speed, waypoint_max_acc, retime_expansions,
    ) = _retime_selected_trajectory(
        waypoints, base_dt, dt_sample, max_vel, max_acc,
        speed_ratio_target=publish_speed_ratio, max_expansions=5
    )
    if safety_mode == "inflated":
        if _chain_blocked_count(dense, occupancy, frame) != 0:
            return None
    else:
        # Geometry was already proven raw-clear with inflated hits restricted to
        # the endpoint grace pockets.  The dense trajectory is collinear with
        # the same segment, so only re-check raw collision here.
        if raw_occupancy is None or _chain_blocked_count(dense, raw_occupancy, frame) != 0:
            return None
    dense_max_acc = max(
        (math.hypot(a[0], a[1], a[2]) for a in dense_acc), default=0.0
    )
    return KinoPlanResult(
        start_zyx=start_zyx, goal_zyx=goal_zyx,
        waypoints_xyz=tuple(waypoints),
        waypoint_velocities_xyz=tuple(velocities),
        waypoint_accelerations_xyz=tuple(accelerations),
        path_xyz=tuple(dense), velocities_xyz=tuple(dense_vel),
        accelerations_xyz=tuple(dense_acc), path_size=len(dense) + 1,
        dt_s=float(trajectory_dt), dt_sample_s=float(dt_sample),
        total_cost=float(direct_length), source="direct", onehot_valid=True,
        variables=0, candidate_counts=(),
        planning_elapsed_ms=(time.perf_counter() - started) * 1000.0,
        fpga_elapsed_ms=0.0, serial_rtt_ms=0.0, load_rtt_ms=0.0,
        fpga_starts=0, valid_starts=0, max_vel_mps=float(dense_max_speed),
        max_acc_mps2=float(waypoint_max_acc), vel_limit_mps=float(max_vel),
        acc_limit_mps2=float(max_acc),
        vel_ok=bool(dense_max_speed <= max_vel * 1.005 + 1.0e-6),
        acc_ok=bool(waypoint_max_acc <= max_acc * 1.05 + 1.0e-6),
        attempts=0,
        details={
            "encoding": "near-goal-direct",
            "near_goal_direct": True,
            "exact_terminal_goal": bool(exact_goal_xyz is not None),
            "terminal_goal_target_xyz": [float(v) for v in goal_xyz],
            "terminal_goal_grid_center_xyz": [float(v) for v in cell3d_to_xyz(goal_zyx, frame)],
            "terminal_goal_quantization_error_m": float(math.dist(goal_xyz, cell3d_to_xyz(goal_zyx, frame))),
            "direct_length_m": float(direct_length),
            "direct_distance_m": float(direct_length),
            "near_goal_direct_safety_mode": safety_mode,
            "near_goal_direct_cross_speed_actual_mps": float(cross_speed),
            "spine_source": "near-direct",
            "size_profile": "near-direct",
            "fallback_reason": "none",
            "force_spine_refresh": False,
            "spine_shortcut_applied": False,
            "spine_shortcut_ratio": 1.0,
            "spine_nodes_before_shortcut": 2,
            "clearance_ms": 0.0, "spine_ms": 0.0, "candidate_ms": 0.0,
            "segment_ms": 0.0, "qubo_ms": 0.0, "solve_host_ms": 0.0,
            "solve_dt_s": float(base_dt),
            "trajectory_dt_s": float(trajectory_dt),
            "retime_expansions": int(retime_expansions),
            "w_len": 0.0, "w_spine": 0.0,
            "board_load_decode_ms": 0.0, "board_load_pl_ms": 0.0,
            "board_load_total_ms": 0.0, "board_transport_rx_ms": 0.0,
            "board_pl_solve_ms": 0.0, "board_result_read_ms": 0.0,
            "board_restart_overhead_ms": 0.0, "issued_iteration_budget": 0,
            "reduced_budgets": [], "adaptive_infos": [],
            "early_restart_stopped": False,
            "dense_fd_max_acc_mps2": float(dense_max_acc),
            "published_waypoint_geom_len_m": float(direct_length),
            "spine_geom_len_m": float(direct_length),
            "selected_geom_len_m": float(direct_length),
        },
    )


class FpgaKinoPathPlanner:
    """Kinodynamic waypoint planner running its QUBO on the Ising machine."""

    def __init__(self, solver: FpgaSolver, clearance_cache_policy: str = "auto") -> None:
        self.solver = solver
        self._clearance_cache_policy = str(clearance_cache_policy or "auto").strip().lower()
        if self._clearance_cache_policy not in {"auto", "live", "versioned"}:
            self._clearance_cache_policy = "auto"
        self._rolling_seed = 0x4B494E4F
        self._qubo_static_cache: OrderedDict[bytes, _KinoQuboStaticEntry] = OrderedDict()
        # Exact one-entry clearance cache. Rolling replans often see identical
        # voxel occupancy even when a fresh PointCloud2 message arrived. Reuse
        # only on byte-identical occupancy + same cap, so safety semantics do
        # not change.
        self._clearance_cache_key: object | None = None
        self._clearance_cache_value: np.ndarray | None = None
        # Live PointCloud occupancy may change on almost every
        # replan, so an exact whole-grid CRC probe can become pure overhead.
        # Keep the exact cache for genuinely static/repeated grids, but after
        # CLEARANCE_CACHE_MISS_THRESHOLD consecutive misses temporarily bypass
        # CRC probing for CLEARANCE_CACHE_BYPASS_FRAMES replans.  This changes
        # only whether we *attempt* cache lookup; the clearance field itself is
        # still computed by the exact same native/Python routine.
        self._clearance_cache_miss_streak = 0
        self._clearance_cache_bypass_remaining = 0
        # A live map that misses repeatedly can permanently skip the
        # exact one-entry CRC probe for this planner lifetime. Clearance is
        # still recomputed exactly on every frame; no stale field is reused.
        self._clearance_cache_permanent_bypass = bool(self._clearance_cache_policy == "live")

    _CLEARANCE_CACHE_MISS_THRESHOLD = 8
    _CLEARANCE_CACHE_BYPASS_FRAMES = 64

    def _clearance_with_adaptive_cache(
        self, occupancy: np.ndarray, safe_voxels: float, cache_token: object | None = None
    ) -> tuple[np.ndarray, dict[str, object]]:
        """Return the exact capped clearance plus cache diagnostics.

        The cache remains byte-exact when probed (shape + cap + CRC32).  In a
        live map, repeated misses trigger a temporary CRC bypass so we do not
        scan the entire occupancy twice merely to discover that the map changed.
        Correctness is unchanged because bypassed frames compute clearance
        directly instead of reusing anything.
        """
        cap = max(1, int(math.ceil(max(0.0, float(safe_voxels)))))
        diag: dict[str, object] = {
            "clearance_cache_hit": False,
            "clearance_cache_probe": False,
            "clearance_cache_bypassed": False,
            "clearance_cache_bypass_remaining": int(self._clearance_cache_bypass_remaining),
            "clearance_cache_miss_streak": int(self._clearance_cache_miss_streak),
            "clearance_cache_permanent_bypass": bool(self._clearance_cache_permanent_bypass),
            "clearance_cache_policy": str(self._clearance_cache_policy),
            "clearance_cache_identity_mode": "live" if self._clearance_cache_policy == "live" else "crc",
            "clearance_cache_token_present": bool(cache_token is not None),
            "clearance_cache_token_hit": False,
            "clearance_crc_ms": 0.0,
            "clearance_kernel_ms": 0.0,
        }

        # Provenance-based cache identity. The ROS node supplies
        # a token that changes whenever the processed cloud snapshot, grid frame,
        # start cell or goal cell changes.  Matching tokens therefore mean the
        # inflated occupancy used by clearance is exactly the same by construction,
        # so a whole-grid CRC scan is unnecessary.  External callers that do not
        # provide a token fall back to the exact CRC policy below.
        if self._clearance_cache_policy == "versioned" and cache_token is not None:
            diag["clearance_cache_identity_mode"] = "versioned"
            diag["clearance_cache_probe"] = True
            key = ("versioned", tuple(int(v) for v in occupancy.shape), cap, cache_token)
            hit = self._clearance_cache_key == key and self._clearance_cache_value is not None
            diag["clearance_cache_hit"] = bool(hit)
            diag["clearance_cache_token_hit"] = bool(hit)
            if hit:
                return self._clearance_cache_value, diag
            kernel_started = time.perf_counter()
            clearance = _clearance_zyx_limited(occupancy, safe_voxels)
            diag["clearance_kernel_ms"] = (time.perf_counter() - kernel_started) * 1000.0
            self._clearance_cache_key = key
            self._clearance_cache_value = clearance
            self._clearance_cache_miss_streak = 0
            self._clearance_cache_bypass_remaining = 0
            return clearance, diag

        if self._clearance_cache_policy == "versioned":
            diag["clearance_cache_identity_mode"] = "crc-fallback"

        if self._clearance_cache_permanent_bypass:
            diag["clearance_cache_bypassed"] = True
            diag["clearance_cache_permanent_bypass"] = True
            kernel_started = time.perf_counter()
            clearance = _clearance_zyx_limited(occupancy, safe_voxels)
            diag["clearance_kernel_ms"] = (time.perf_counter() - kernel_started) * 1000.0
            return clearance, diag

        if self._clearance_cache_bypass_remaining > 0:
            self._clearance_cache_bypass_remaining -= 1
            diag["clearance_cache_bypassed"] = True
            if diag.get("clearance_cache_identity_mode") == "crc":
                diag["clearance_cache_identity_mode"] = "crc-bypass"
            kernel_started = time.perf_counter()
            clearance = _clearance_zyx_limited(occupancy, safe_voxels)
            diag["clearance_kernel_ms"] = (time.perf_counter() - kernel_started) * 1000.0
            diag["clearance_cache_bypass_remaining"] = int(self._clearance_cache_bypass_remaining)
            return clearance, diag

        diag["clearance_cache_probe"] = True
        crc_started = time.perf_counter()
        occ_view = np.ascontiguousarray(occupancy, dtype=np.uint8)
        occ_crc = int(binascii.crc32(memoryview(occ_view)) & 0xFFFFFFFF)
        diag["clearance_crc_ms"] = (time.perf_counter() - crc_started) * 1000.0
        key = (tuple(int(v) for v in occupancy.shape), cap, occ_crc)
        hit = self._clearance_cache_key == key and self._clearance_cache_value is not None
        diag["clearance_cache_hit"] = bool(hit)
        if hit:
            self._clearance_cache_miss_streak = 0
            diag["clearance_cache_miss_streak"] = 0
            return self._clearance_cache_value, diag

        kernel_started = time.perf_counter()
        clearance = _clearance_zyx_limited(occupancy, safe_voxels)
        diag["clearance_kernel_ms"] = (time.perf_counter() - kernel_started) * 1000.0
        self._clearance_cache_key = key
        self._clearance_cache_value = clearance
        self._clearance_cache_miss_streak += 1
        if self._clearance_cache_miss_streak >= self._CLEARANCE_CACHE_MISS_THRESHOLD:
            self._clearance_cache_permanent_bypass = True
            self._clearance_cache_bypass_remaining = self._CLEARANCE_CACHE_BYPASS_FRAMES
            self._clearance_cache_miss_streak = 0
        diag["clearance_cache_bypass_remaining"] = int(self._clearance_cache_bypass_remaining)
        diag["clearance_cache_miss_streak"] = int(self._clearance_cache_miss_streak)
        return clearance, diag

    def plan_executable(
        self,
        grid: np.ndarray,
        start_zyx: Cell3D,
        goal_zyx: Cell3D,
        frame: GridFrame3D,
        *,
        reaches_goal: bool = True,
        max_vel_xy: float | None = None,
        max_vel_z: float | None = None,
        max_acc_xy: float | None = None,
        max_acc_z: float | None = None,
        max_duration: float = 60.0,
        max_samples: int = 10000,
        execution_reference=None,
        **kwargs,
    ) -> KinoPlanResult:
        """Build measured-start, zero-terminal Hermite cubics.
        Export only analytically bounded, continuously collision-certified curves.
        """
        started = time.perf_counter()
        occupancy = validate_grid3d(grid, trusted_binary=bool(kwargs.get("trusted_binary_grid", False)))
        if occupancy.shape != (frame.layers, frame.rows, frame.cols):
            raise ValueError("execution grid/frame dimensions differ")
        for cell in (start_zyx, goal_zyx):
            if len(cell) != 3 or any(index < 0 or index >= size for index, size in zip(cell, occupancy.shape)):
                raise ValueError("execution endpoint outside grid")
            if occupancy[cell]:
                raise ValueError("execution endpoint is occupied")
        exact_start = kwargs.get("exact_start_xyz")
        exact_goal = kwargs.get("exact_goal_xyz")
        start = np.asarray(cell3d_to_xyz(start_zyx, frame) if exact_start is None else exact_start, dtype=np.float64)
        goal = np.asarray(cell3d_to_xyz(goal_zyx, frame) if exact_goal is None else exact_goal, dtype=np.float64)
        sv = np.asarray(kwargs.get("start_vel_xyz", (0.0, 0.0, 0.0)), dtype=np.float64)
        for label, value, cell in (("start", start, start_zyx), ("goal", goal, goal_zyx)):
            if value.shape != (3,) or not np.isfinite(value).all() or xyz_to_cell3d(*value, frame) != cell:
                raise ValueError(f"invalid exact execution {label}")
        if sv.shape != (3,) or not np.isfinite(sv).all():
            raise ValueError("invalid measured execution start velocity")
        max_vel = float(kwargs.get("max_vel", 2.0))
        max_acc = float(kwargs.get("max_acc", 2.0))
        limits = {
            "speed": max_vel, "acceleration": max_acc,
            "speed_xy": max_vel if max_vel_xy is None else float(max_vel_xy),
            "speed_z": max_vel if max_vel_z is None else float(max_vel_z),
            "acceleration_xy": max_acc if max_acc_xy is None else float(max_acc_xy),
            "acceleration_z": max_acc if max_acc_z is None else float(max_acc_z),
        }
        if any(not math.isfinite(value) or value <= 0.0 for value in limits.values()):
            raise ValueError("execution velocity/acceleration limits must be finite and positive")
        if (np.linalg.norm(sv) > limits["speed"] + 1.0e-7 or
                np.linalg.norm(sv[:2]) > limits["speed_xy"] + 1.0e-7 or
                abs(sv[2]) > limits["speed_z"] + 1.0e-7):
            raise RuntimeError("measured start velocity exceeds execution limits")
        cancel = kwargs.get("cancel_callback")
        if cancel is not None and cancel():
            raise RuntimeError("executable trajectory planning cancelled")
        options = dict(kwargs)
        options["goal_vel_xyz"] = (0.0, 0.0, 0.0)
        if start_zyx == goal_zyx:
            # Same-cell motion may need to brake away from and return to the
            # goal. The legacy straight shortcut rejects such measured speeds;
            # feed exact geometry into the common Hermite validator instead.
            distance = float(np.linalg.norm(goal - start))
            result = KinoPlanResult(
                start_zyx=start_zyx, goal_zyx=goal_zyx,
                waypoints_xyz=(tuple(start), tuple(goal)),
                waypoint_velocities_xyz=(), waypoint_accelerations_xyz=(),
                path_xyz=(), velocities_xyz=(), accelerations_xyz=(), path_size=0,
                dt_s=max(0.05, distance / max_vel),
                dt_sample_s=float(options.get("dt_sample", 0.01)),
                total_cost=distance, source="direct", onehot_valid=True,
                variables=0, candidate_counts=(), planning_elapsed_ms=0.0,
                fpga_elapsed_ms=0.0, serial_rtt_ms=0.0, load_rtt_ms=0.0,
                fpga_starts=0, valid_starts=0, max_vel_mps=0.0,
                max_acc_mps2=0.0, vel_limit_mps=max_vel,
                acc_limit_mps2=max_acc, vel_ok=False, acc_ok=False, attempts=0,
                details={"encoding": "same-cell-exact-hermite", "solver_transactions": 0,
                         "fallback_reason": "none"},
            )
        else:
            result = self.plan(grid, start_zyx, goal_zyx, frame, **options)
        if cancel is not None and cancel():
            raise RuntimeError("executable trajectory planning cancelled")
        points = np.asarray(result.waypoints_xyz, dtype=np.float64).copy()
        if len(points) < 2 or len(points) > 257 or not np.isfinite(points).all():
            raise RuntimeError("invalid or oversized native execution waypoints")
        # Always bind the contract to the measured continuous position, even
        # when the geometry planner's legacy fallback retained a voxel centre.
        points[0], points[-1] = start, goal
        original_point_count = len(points)
        # Repeated geometry nodes carry no motion and must not create zero-duration knots.
        points = points[np.r_[True, np.linalg.norm(np.diff(points, axis=0), axis=1) > 1.0e-9]]
        if len(points) == 1:
            points = np.array([start, goal])
        points[0], points[-1] = start, goal
        dt_sample = float(options.get("dt_sample", 0.01))
        if not math.isfinite(dt_sample) or dt_sample <= 0.0:
            raise ValueError("execution sample interval must be finite and positive")
        failure = "no bounded collision-free Hermite trajectory"
        accepted = None
        cruise = min(float(kwargs.get("cruise_speed_mps", 1.5)), limits["speed"], limits["speed_xy"])
        # Leave room for spline curvature; exact analytic checks below remain authoritative.
        acceleration = 0.7 * limits["acceleration"]
        for _ in range(2):
            prior_durations = reference_durations(points, execution_reference)
            for expansion in range(25):
                if cancel is not None and cancel():
                    raise RuntimeError("executable trajectory planning cancelled")
                if expansion == 0 and prior_durations is not None:
                    durations = prior_durations
                else:
                    # Reduce cruise by 10% per retry, recomputing times around fixed measured speed.
                    target_cruise = cruise * 0.9 ** (expansion - (prior_durations is not None))
                    durations = motion_durations(points, sv, target_cruise, acceleration)
                    if len(points) == 2:
                        # Brake-and-return acceleration is 4*v/T even at zero displacement.
                        durations = np.maximum(durations, max(4 * np.linalg.norm(sv[:2]) / limits["acceleration_xy"],
                                                              4 * abs(sv[2]) / limits["acceleration_z"]))
                velocities = clamped_velocities(points, durations, sv)
                coefficients = hermite_coefficients(points, velocities, durations)
                maxima = executable_segment_limits(coefficients, durations)
                ratios = [value / limits[name] for name, value in maxima.items()]
                if max(ratios) <= 1.0 + 1.0e-7:
                    if executable_segments_clear(coefficients, durations, occupancy, frame, cancel_callback=cancel):
                        accepted = coefficients, durations, velocities, maxima, "clamped-c2", expansion
                        break
                    failure = "native Hermite curve collides with occupied voxels or leaves map"
                    continue
                failure = "native Hermite curve exceeds analytic execution dynamics"
            if accepted is not None:
                break
            # A certified local chord can remove an unnecessary sharp/short knot without stopping.
            simplified = [points[0]]
            for i in range(1, len(points) - 1):
                chord = np.zeros((1, 3, 4))
                chord[0, :, 0] = simplified[-1]
                chord[0, :, 1] = points[i + 1] - simplified[-1]
                if not executable_segments_clear(chord, [1.0], occupancy, frame, cancel_callback=cancel):
                    simplified.append(points[i])
            simplified.append(points[-1])
            if len(simplified) == len(points):
                break
            points = np.asarray(simplified)
        if accepted is None:
            raise RuntimeError(failure)
        coefficients, durations, velocities, maxima, tangent_mode, expansion = accepted
        # Match the native protocol's finite output envelope before allocating dense samples.
        if not math.isfinite(max_duration) or max_duration <= 0.0 or sum(durations) > max_duration:
            raise RuntimeError("executable trajectory exceeds output duration limit")
        if max_samples < 2 or sum(math.ceil(duration / dt_sample) for duration in durations) + 1 > max_samples:
            raise RuntimeError("executable trajectory exceeds output sample limit")
        dense, dense_v, dense_a = [], [], []
        waypoint_a = []
        for index, (c, duration) in enumerate(zip(coefficients, durations)):
            times = np.arange(0.0, duration, dt_sample)
            p, v, a = sample_executable_segment(c, times)
            dense.extend(map(tuple, p))
            dense_v.extend(map(tuple, v))
            dense_a.extend(map(tuple, a))
            waypoint_a.append(tuple(a[0]))
        p, v, a = sample_executable_segment(coefficients[-1], np.array((durations[-1],)))
        dense.append(tuple(p[0]))
        dense_v.append(tuple(v[0]))
        dense_a.append(tuple(a[0]))
        waypoint_a.append(tuple(a[0]))
        transports = tuple(str(item).upper() for item in result.details.get("solver_transports", ()))
        hardware_transactions = int(result.details.get("hardware_transactions_total", 0))
        source = result.source
        if source == "fpga" and not hardware_transactions:
            source = "mock" if any("MOCK" in item for item in transports) else "unverified"
        details = dict(result.details)
        details.update({
            "execution_validated": True, "interpolation": "cubic-hermite",
            "execution_tangent_mode": tangent_mode, "execution_retime_expansions": expansion,
            "execution_limits": limits, "execution_maxima": maxima,
            "execution_collision": "subdivided-bezier-hull-occupied-voxels",
            "execution_continuity": "C2",
            "execution_removed_knots": original_point_count - len(points),
            "execution_timing_reference": prior_durations is not None and expansion == 0,
            "hardware_transactions": hardware_transactions, "solver_transports": transports,
            "solver_transactions": int(result.details.get("solver_transactions_total", result.details.get("solver_transactions", 0))),
            "legacy_geometry_source": result.source,
            "legacy_sampled_trajectory_discarded": True,
        })
        return replace(
            result, waypoints_xyz=tuple(map(tuple, points)), waypoint_velocities_xyz=tuple(map(tuple, velocities)),
            waypoint_accelerations_xyz=tuple(waypoint_a), path_xyz=tuple(dense), velocities_xyz=tuple(dense_v),
            accelerations_xyz=tuple(dense_a), path_size=len(dense) + 1, dt_s=durations[0], dt_sample_s=dt_sample,
            source=source, planning_elapsed_ms=(time.perf_counter() - started) * 1000.0,
            max_vel_mps=maxima["speed"], max_acc_mps2=maxima["acceleration"], vel_ok=True, acc_ok=True,
            polynomial_coefficients_xyz=tuple(tuple(tuple(float(v) for v in axis) for axis in c) for c in coefficients),
            segment_durations_s=durations, reaches_goal=bool(reaches_goal), details=details,
        )

    def plan(
        self,
        grid: np.ndarray,
        start_zyx: Cell3D,
        goal_zyx: Cell3D,
        frame: GridFrame3D,
        *,
        start_vel_xyz: Sequence[float] = (0.0, 0.0, 0.0),
        start_acc_xyz: Sequence[float] = (0.0, 0.0, 0.0),
        goal_vel_xyz: Sequence[float] = (0.0, 0.0, 0.0),
        waypoint_count: int = 16,
        candidates_per_layer: int = 16,
        dt: float = 0.6,
        dt_auto: bool = True,
        cruise_speed_mps: float = 1.5,
        dt_sample: float = 0.01,
        max_vel: float = 2.0,
        max_acc: float = 2.0,
        publish_speed_ratio: float = 0.96,
        qubo_dt_quantum_s: float = 0.02,
        strict_spine_latency_fuse_ms: float = 3.5,
        candidate_cpp_accel: bool = True,
        incremental_qubo: bool = True,
        incremental_qubo_cache_size: int = 32,
        w_vel: float = 1.0,
        w_acc: float = 0.5,
        w_obs: float = 40.0,
        w_spine: float = 0.45,
        w_len: float = 0.35,
        w_start_vel: float = 1.0,
        w_start_acc: float = 1.0,
        w_end_vel: float = 1.0,
        segment_penalty: float = 1000.0,
        a_mult: float = 3.0,
        candidate_radius: int = 2,
        safe_clearance_m: float = 0.3,
        iterations: int = 480,
        restarts: int = 4,
        adaptive_problem_size: bool = True,
        adaptive_solver_profile: bool = True,
        quality_gap_ratio: float = 0.05,
        quality_guard: bool = True,
        previous_path_xyz: Sequence[Sequence[float]] = (),
        force_spine_refresh: bool = False,
        shortcut_reused_spine: bool = True,
        near_goal_direct_m: float = 4.0,
        near_goal_tiny_m: float = 4.0,
        near_goal_direct_cross_speed_mps: float = 0.8,
        near_goal_direct_endpoint_grace_m: float = 0.4,
        near_hysteresis_distance_m: float = 4.5,
        near_hysteresis_detour_ratio: float = 1.06,
        near_hysteresis_improvement_ratio: float = 0.02,
        exact_start_xyz: Sequence[float] | None = None,
        exact_goal_xyz: Sequence[float] | None = None,
        seed: int | None = None,
        raw_grid: np.ndarray | None = None,
        trusted_binary_grid: bool = False,
        clearance_cache_token: object | None = None,
        trace_callback: Callable[[str, dict[str, object]], None] | None = None,
        cancel_callback: Callable[[], bool] | None = None,
    ) -> KinoPlanResult:
        started = time.perf_counter()
        solver_transactions_total = 0
        hardware_transactions_total = 0
        stage_ms: dict[str, float] = {}
        stage_t = started
        grid_validation_started = time.perf_counter()
        grid_array = np.asarray(grid)
        grid_validation_copy_avoided = bool(
            trusted_binary_grid and grid_array.dtype == np.uint8
            and grid_array.ndim == 3 and grid_array.size > 0
            and grid_array.flags.c_contiguous
        )
        occupancy = validate_grid3d(grid_array, trusted_binary=trusted_binary_grid)
        grid_validation_ms = (time.perf_counter() - grid_validation_started) * 1000.0
        raw_grid_validation_ms = 0.0
        raw_grid_validation_copy_avoided = False
        if occupancy[start_zyx] or occupancy[goal_zyx]:
            raise ValueError("三维起点/终点位于障碍体素")
        if start_zyx == goal_zyx:
            raise ValueError("三维起点和终点不能相同")
        layer_cap = max(2, min(int(waypoint_count), 254))
        candidate_cap = max(2, min(int(candidates_per_layer), N_HW))
        dt = max(0.1, float(dt))
        cruise_speed_mps = max(0.3, float(cruise_speed_mps))
        dt_sample = max(0.005, float(dt_sample))
        publish_speed_ratio = min(1.0, max(0.75, float(publish_speed_ratio)))
        qubo_dt_quantum_s = max(0.0, min(0.10, float(qubo_dt_quantum_s)))
        strict_spine_latency_fuse_ms = max(0.0, float(strict_spine_latency_fuse_ms))
        candidate_cpp_accel = bool(candidate_cpp_accel)
        incremental_qubo = bool(incremental_qubo)
        incremental_qubo_cache_size = max(1, min(128, int(incremental_qubo_cache_size)))
        if not incremental_qubo and self._qubo_static_cache:
            self._qubo_static_cache.clear()
        if seed is None:
            seed_base = int(self._rolling_seed) & 0xFFFFFFFF
            self._rolling_seed = (self._rolling_seed + 0x9E3779B9) & 0xFFFFFFFF
        else:
            seed_base = int(seed) & 0xFFFFFFFF
        sv = (float(start_vel_xyz[0]), float(start_vel_xyz[1]), float(start_vel_xyz[2]))
        sa = (float(start_acc_xyz[0]), float(start_acc_xyz[1]), float(start_acc_xyz[2]))
        gv = (float(goal_vel_xyz[0]), float(goal_vel_xyz[1]), float(goal_vel_xyz[2]))

        grid_start_xyz = cell3d_to_xyz(start_zyx, frame)
        grid_goal_xyz = cell3d_to_xyz(goal_zyx, frame)

        def _exact_in_cell(value: Sequence[float] | None, cell: Cell3D, fallback: Point3) -> Point3:
            if value is None or len(value) < 3:
                return fallback
            point = (float(value[0]), float(value[1]), float(value[2]))
            if not all(math.isfinite(v) for v in point):
                return fallback
            try:
                return point if xyz_to_cell3d(point[0], point[1], point[2], frame) == cell else fallback
            except Exception:
                return fallback

        exact_start = _exact_in_cell(exact_start_xyz, start_zyx, grid_start_xyz)
        exact_goal = _exact_in_cell(exact_goal_xyz, goal_zyx, grid_goal_xyz)
        exact_terminal_goal = math.dist(exact_goal, grid_goal_xyz) > 1.0e-12

        raw_occupancy: np.ndarray | None = None
        if raw_grid is not None:
            raw_validation_started = time.perf_counter()
            raw_array = np.asarray(raw_grid)
            raw_grid_validation_copy_avoided = bool(
                trusted_binary_grid and raw_array.dtype == np.uint8
                and raw_array.ndim == 3 and raw_array.size > 0
                and raw_array.flags.c_contiguous
            )
            raw_occupancy = validate_grid3d(raw_array, trusted_binary=trusted_binary_grid)
            raw_grid_validation_ms = (time.perf_counter() - raw_validation_started) * 1000.0
        direct_distance_m = math.dist(exact_start, exact_goal)
        if near_goal_direct_m > 0.0 and direct_distance_m <= float(near_goal_direct_m):
            grace_voxels = int(math.ceil(
                max(0.0, float(near_goal_direct_endpoint_grace_m))
                / max(frame.resolution_m, frame.resolution_z_m, 1.0e-9)
            ))
            direct_result = _near_goal_direct_result(
                started=started, start_zyx=start_zyx, goal_zyx=goal_zyx,
                frame=frame, occupancy=occupancy, raw_occupancy=raw_occupancy,
                endpoint_grace_voxels=grace_voxels, start_vel_xyz=sv,
                goal_vel_xyz=gv, cruise_speed_mps=cruise_speed_mps,
                dt_sample=dt_sample, max_vel=max_vel, max_acc=max_acc,
                publish_speed_ratio=publish_speed_ratio,
                max_cross_speed_mps=near_goal_direct_cross_speed_mps,
                exact_start_xyz=exact_start, exact_goal_xyz=exact_goal,
            )
            if direct_result is not None:
                return direct_result

        # 1) global connectivity spine via voxel A* on the inflated grid, with
        #    a raw-grid fallback: when inflation isolates start/goal (e.g. the
        #    UAV sits inside the inflated ground plane), search on the raw grid
        #    with a clearance-biased cost and let the QUBO penalties / segment
        #    checks enforce safety instead of failing outright.
        search = occupancy
        safe_voxels = max(0.0, float(safe_clearance_m) / max(frame.resolution_m, frame.resolution_z_m, 1.0e-9))
        # Adaptive exact-cache policy. Live PointCloud maps in
        # repeated real benchmarks can produce all cache misses, so after a
        # short miss streak we bypass whole-grid CRC probing for a bounded
        # number of replans and compute the exact clearance directly.
        clearance_cap = max(1, int(math.ceil(max(0.0, safe_voxels))))
        clearance_call_started = time.perf_counter()
        stage_ms["pre_clearance_setup_ms"] = (clearance_call_started - started) * 1000.0
        clearance, clearance_diag = self._clearance_with_adaptive_cache(occupancy, safe_voxels, cache_token=clearance_cache_token)
        clearance_call_finished = time.perf_counter()
        stage_ms["clearance_compute_ms"] = (clearance_call_finished - clearance_call_started) * 1000.0
        clearance_cache_hit = bool(clearance_diag.get("clearance_cache_hit", False))
        # Keep historical clearance_ms semantics for backwards comparison:
        # it spans plan entry through clearance completion. The two detailed
        # fields above separate setup/preemption from the actual clearance call.
        stage_ms["clearance_ms"] = (clearance_call_finished - stage_t) * 1000.0
        stage_t = time.perf_counter()
        if bool(force_spine_refresh):
            spine, spine_source = (), "astar-refresh"
        else:
            spine, spine_source = _previous_path_spine(
                previous_path_xyz, occupancy, start_zyx, goal_zyx, frame
            )
        strict_safe = bool(spine)
        if not spine and not occupancy[start_zyx] and not occupancy[goal_zyx]:
            spine = _fast_spine_astar_windowed(
                occupancy, start_zyx, goal_zyx, margin_voxels=10, fallback_full=True
            )
            if spine:
                strict_safe = True
                spine_source = "astar"

        if not spine and raw_occupancy is not None and not bool(force_spine_refresh):
            raw_prev, raw_prev_source = _previous_path_spine(
                previous_path_xyz, raw_occupancy, start_zyx, goal_zyx, frame
            )
            if raw_prev:
                spine = raw_prev
                search = raw_occupancy
                strict_safe = False
                spine_source = (
                    "previous-raw" if raw_prev_source == "previous-safe"
                    else "micro-bypass-raw"
                )

        if not spine and raw_occupancy is not None:
            # clearance-biased search on the raw grid: prefer high-clearance
            # voxels but allow threading 1-voxel gaps when inflation closed them
            penalty = np.maximum(0.0, safe_voxels - clearance)
            bias = np.where(raw_occupancy == 0, penalty * penalty, 0.0)
            spine = _fast_spine_astar_windowed(
                raw_occupancy, start_zyx, goal_zyx, bias=bias,
                margin_voxels=10, fallback_full=True,
            )
            if spine:
                search = raw_occupancy
                strict_safe = False
                spine_source = "astar-raw"
        if not spine:
            raise RuntimeError("三维体素A*骨架无解：起点与终点不连通")

        spine_nodes_before_shortcut = len(spine)
        spine_length_before_shortcut_m = _spine_length_m(spine, frame)
        spine_shortcut_applied = False
        if bool(shortcut_reused_spine) and spine_source in (
            "previous-safe", "previous-raw", "micro-bypass", "micro-bypass-raw",
            "astar", "astar-raw",
        ):
            shortened = _shortcut_spine_cells(
                spine, search,
                max_lookahead=(16 if spine_source in ("astar", "astar-raw") else 24),
            )
            shortened_length = _spine_length_m(shortened, frame)
            if (
                len(shortened) >= 2
                and shortened_length <= spine_length_before_shortcut_m + 1.0e-9
                and (len(shortened) < len(spine) or shortened_length + 1.0e-9 < spine_length_before_shortcut_m)
            ):
                spine = shortened
                spine_shortcut_applied = True

        near_hysteresis_refresh = False
        current_spine_len = _spine_length_m(spine, frame)
        if (
            direct_distance_m <= max(0.0, float(near_hysteresis_distance_m))
            and direct_distance_m > 1.0e-6
            and spine_source in ("previous-safe", "previous-raw", "micro-bypass", "micro-bypass-raw")
            and current_spine_len > direct_distance_m * max(1.0, float(near_hysteresis_detour_ratio))
        ):
            near_bias = None
            if raw_occupancy is not None and search is raw_occupancy:
                near_penalty = np.maximum(0.0, safe_voxels - clearance)
                near_bias = np.where(raw_occupancy == 0, near_penalty * near_penalty, 0.0)
            fresh_near = _fast_spine_astar_windowed(
                search, start_zyx, goal_zyx, bias=near_bias,
                margin_voxels=8, fallback_full=True,
            )
            if fresh_near:
                fresh_near = _shortcut_spine_cells(fresh_near, search, max_lookahead=24)
                fresh_len = _spine_length_m(fresh_near, frame)
                improve = min(0.25, max(0.0, float(near_hysteresis_improvement_ratio)))
                if fresh_len + 1.0e-9 < current_spine_len * (1.0 - improve):
                    spine = fresh_near
                    current_spine_len = fresh_len
                    near_hysteresis_refresh = True
                    spine_source = "near-refresh-raw" if search is raw_occupancy else "near-refresh"

        free_layers, per_layer, size_profile, spine_length_m, turn_ratio = _adaptive_problem_shape(
            spine,
            frame,
            layer_cap,
            candidate_cap,
            bool(adaptive_problem_size),
        )
        if adaptive_problem_size and near_goal_tiny_m > 0.0 and spine_length_m <= float(near_goal_tiny_m):
            if spine_length_m <= 2.5:
                free_layers = min(free_layers, layer_cap, 8)
                per_layer = min(per_layer, candidate_cap, 4)
                size_profile = "near-tiny-wide"
            else:
                free_layers = min(free_layers, layer_cap, 8)
                per_layer = min(per_layer, candidate_cap, 4)
                size_profile = "near-wide"
        # Reused/rejoined suffixes already contain a known feasible reference
        # path, so follow the reference Micro-Bypass budget instead of letting
        # voxel-level turn noise expand the model.
        if adaptive_problem_size and spine_source in ("previous-safe", "previous-raw"):
            free_layers = min(free_layers, layer_cap, 8)
            per_layer = min(per_layer, candidate_cap, 3)
            size_profile = "reuse-micro"
        elif adaptive_problem_size and spine_source in ("near-refresh", "near-refresh-raw"):
            free_layers = min(free_layers, layer_cap, 8)
            per_layer = min(per_layer, candidate_cap, 3)
            size_profile = "near-refresh-micro"
        elif adaptive_problem_size and spine_source in ("micro-bypass", "micro-bypass-raw"):
            free_layers = min(free_layers, layer_cap, 8)
            per_layer = min(per_layer, candidate_cap, 4)
            size_profile = "micro-bypass"
        stage_ms["spine_ms"] = (time.perf_counter() - stage_t) * 1000.0
        spine_latency_fuse_triggered = bool(
            strict_spine_latency_fuse_ms > 0.0
            and strict_safe
            and stage_ms["spine_ms"] >= strict_spine_latency_fuse_ms
        )
        stage_t = time.perf_counter()
        if trace_callback is not None:
            trace_callback("kino_start", {
                "start_zyx": list(start_zyx), "goal_zyx": list(goal_zyx),
                "free_layers": free_layers, "per_layer": per_layer,
                "size_profile": size_profile, "spine_length_m": spine_length_m,
                "turn_ratio": turn_ratio, "spine_source": spine_source, "dt": dt,
                "max_vel": max_vel, "max_acc": max_acc,
                "start_vel": list(sv), "start_acc": list(sa),
                "blocked_voxels": int(np.count_nonzero(occupancy)),
            })
        spine_xyz = _resample_spine_xyz(spine, frame, free_layers + 2)
        # Preserve continuous odometry/goal coordinates at the trajectory
        # endpoints.  Grid cells remain the search state, but publishing the
        # voxel centre makes RViz stop up to half a voxel away from the actual
        # goal.  The exact points are guaranteed to map to the same cleared
        # start/goal cells, so obstacle semantics do not change.
        if spine_xyz:
            spine_xyz[0] = exact_start
            spine_xyz[-1] = exact_goal

        # 1b) auto time step: dt = waypoint spacing / cruise speed, with a
        # floor large enough that voxel-quantised waypoints satisfy the accel
        # limit (a ~ dp/dt^2 with dp ~ 0.15 m voxel jitter needs dt >= ~0.35 s)
        if dt <= 0.0 or dt_auto:
            # Use actual backbone arc length instead of start-goal Euclidean
            # distance.  Curved routes otherwise begin with an unrealistically
            # small dt and pay one or more expensive solve retries merely to
            # satisfy the speed limit.
            sampled_length = sum(math.dist(a, b) for a, b in zip(spine_xyz, spine_xyz[1:]))
            segment_lengths = [math.dist(a, b) for a, b in zip(spine_xyz, spine_xyz[1:])]
            spacing = sampled_length / max(1, len(segment_lengths))
            max_segment = max(segment_lengths, default=spacing)
            second_diff = 0.0
            for a, b, c in zip(spine_xyz, spine_xyz[1:], spine_xyz[2:]):
                second_diff = max(
                    second_diff,
                    math.sqrt(
                        (c[0] - 2.0 * b[0] + a[0]) ** 2
                        + (c[1] - 2.0 * b[1] + a[1]) ** 2
                        + (c[2] - 2.0 * b[2] + a[2]) ** 2
                    ),
                )
            dt_cruise = spacing / max(0.3, cruise_speed_mps)
            # The previous 1.0 m/s cruise target and 0.90 velocity margin left
            # the published FPGA trajectory at only ~0.8 m/s median despite a
            # 2 m/s limit.  Use more of the admissible dynamic envelope while
            # keeping explicit velocity/acceleration guards and retry attempts.
            dt_vel = max_segment / max(0.3, 0.95 * max_vel)
            dt_acc = math.sqrt(second_diff / max(0.3, 0.92 * max_acc)) if second_diff > 0.0 else 0.0
            dt = max(0.24, dt_cruise, dt_vel, dt_acc)

        dt_unquantized = float(dt)
        if dt_auto and qubo_dt_quantum_s > 0.0:
            q = float(qubo_dt_quantum_s)
            dt = max(0.10, math.floor(dt / q + 0.5) * q)

        if spine_latency_fuse_triggered:
            fuse_t = time.perf_counter()
            fuse_waypoints = list(spine_xyz)
            (
                fuse_dt, fuse_vel, fuse_acc, fuse_dense, fuse_dense_vel, fuse_dense_acc,
                fuse_max_speed, fuse_waypoint_max_acc, fuse_expansions,
            ) = _retime_selected_trajectory(
                fuse_waypoints, dt, dt_sample, max_vel, max_acc,
                speed_ratio_target=publish_speed_ratio,
                max_expansions=6,
                linear_only=True,
            )
            fuse_retime_ms = (time.perf_counter() - fuse_t) * 1000.0
            fuse_collision_t = time.perf_counter()
            fuse_blocked = _chain_blocked_count(fuse_dense, occupancy, frame)
            fuse_collision_ms = (time.perf_counter() - fuse_collision_t) * 1000.0
            fuse_dense_max_acc = max(
                (math.hypot(a[0], a[1], a[2]) for a in fuse_dense_acc),
                default=0.0,
            )
            fuse_vel_ok = fuse_max_speed <= max_vel * 1.005 + 1.0e-6
            fuse_acc_ok = fuse_waypoint_max_acc <= max_acc * 1.05 + 1.0e-6
            if fuse_blocked == 0 and fuse_vel_ok and fuse_acc_ok:
                fuse_geom_len = sum(
                    math.dist(a, b)
                    for a, b in zip(fuse_waypoints, fuse_waypoints[1:])
                )
                fuse_stage_ms = dict(stage_ms)
                fuse_stage_ms.update({
                    "candidate_ms": 0.0,
                    "segment_ms": 0.0,
                    "qubo_ms": 0.0,
                    "solve_host_ms": 0.0,
                    "retime_ms": float(fuse_retime_ms),
                    "dense_collision_ms": float(fuse_collision_ms),
                })
                return KinoPlanResult(
                    start_zyx=start_zyx,
                    goal_zyx=goal_zyx,
                    waypoints_xyz=tuple(fuse_waypoints),
                    waypoint_velocities_xyz=tuple(fuse_vel),
                    waypoint_accelerations_xyz=tuple(fuse_acc),
                    path_xyz=tuple(fuse_dense),
                    velocities_xyz=tuple(fuse_dense_vel),
                    accelerations_xyz=tuple(fuse_dense_acc),
                    path_size=len(fuse_dense) + 1,
                    dt_s=float(fuse_dt),
                    dt_sample_s=float(dt_sample),
                    total_cost=float(fuse_geom_len),
                    source="deterministic",
                    onehot_valid=False,
                    variables=0,
                    candidate_counts=(),
                    planning_elapsed_ms=(time.perf_counter() - started) * 1000.0,
                    fpga_elapsed_ms=0.0,
                    serial_rtt_ms=0.0,
                    load_rtt_ms=0.0,
                    fpga_starts=0,
                    valid_starts=0,
                    max_vel_mps=float(fuse_max_speed),
                    max_acc_mps2=float(fuse_waypoint_max_acc),
                    vel_limit_mps=float(max_vel),
                    acc_limit_mps2=float(max_acc),
                    vel_ok=True,
                    acc_ok=True,
                    attempts=0,
                    details={
                        "encoding": "strict-safe-spine-latency-fuse",
                        "native_cpp_accel": bool(
                            _native_fastpath is not None and _native_fastpath.available()
                        ),
                        **fuse_stage_ms,
                        "solve_dt_s": float(dt),
                        "dt_unquantized_s": float(dt_unquantized),
                        "qubo_dt_quantum_s": float(qubo_dt_quantum_s),
                        "publish_speed_ratio": float(publish_speed_ratio),
                        "trajectory_dt_s": float(fuse_dt),
                        "retime_expansions": int(fuse_expansions),
                        "host_payload_ms": 0.0,
                        "host_transport_wall_ms": 0.0,
                        "host_decode_ms": 0.0,
                        "host_solver_total_ms": 0.0,
                        "host_payload_bytes": 0,
                        "host_load_modes": [],
                        "solver_transactions": 0,
                        "solver_profiles": [],
                        "spine_nodes": len(spine),
                        "spine_nodes_before_shortcut": int(spine_nodes_before_shortcut),
                        "near_hysteresis_refresh": bool(near_hysteresis_refresh),
                        "spine_source": spine_source,
                        "force_spine_refresh": bool(force_spine_refresh),
                        "spine_shortcut_applied": bool(spine_shortcut_applied),
                        "spine_length_before_shortcut_m": float(spine_length_before_shortcut_m),
                        "spine_shortcut_ratio": float(
                            spine_length_m / max(spine_length_before_shortcut_m, 1.0e-9)
                        ),
                        "strict_safe": True,
                        "raw_spine_used": False,
                        "inflated_blocked_segments": 0,
                        "retry_inflated_gate_required": True,
                        "fallback_reason": "spine-budget",
                        "size_profile": "spine-fuse",
                        "spine_length_m": float(spine_length_m),
                        "spine_turn_ratio": float(turn_ratio),
                        "spine_geom_len_m": float(fuse_geom_len),
                        "selected_geom_len_m": float(fuse_geom_len),
                        "published_waypoint_geom_len_m": float(fuse_geom_len),
                        "direct_distance_m": float(direct_distance_m),
                        "near_goal_direct": False,
                        "near_goal_direct_safety_mode": "not-used",
                        "near_goal_direct_cross_speed_actual_mps": None,
                        "dense_max_speed_mps": float(fuse_max_speed),
                        "waypoint_max_speed_mps": float(max(
                            (math.hypot(v[0], v[1], v[2]) for v in fuse_vel),
                            default=0.0,
                        )),
                        "dense_fd_max_acc_mps2": float(fuse_dense_max_acc),
                        "spine_latency_fuse_ms": float(strict_spine_latency_fuse_ms),
                        "spine_latency_fuse_triggered": True,
                        "board_load_decode_ms": 0.0,
                        "board_load_pl_ms": 0.0,
                        "board_load_total_ms": 0.0,
                        "board_transport_rx_ms": 0.0,
                        "board_pl_solve_ms": 0.0,
                        "board_result_read_ms": 0.0,
                        "board_restart_overhead_ms": 0.0,
                        "issued_iteration_budget": 0,
                        "reduced_budgets": [],
                        "adaptive_infos": [],
                        "early_restart_stopped": False,
                    },
                )
            # Safety/dynamics validation failed unexpectedly; continue through
            # the normal FPGA path rather than weakening any guard.
            spine_latency_fuse_triggered = False

        # 2) candidate positions per free layer (Chebyshev cube around spine)
        goal_xyz = exact_goal
        bias_dir: Point3 | None = (
            (goal_xyz[0] - spine_xyz[0][0]),
            (goal_xyz[1] - spine_xyz[0][1]),
            (goal_xyz[2] - spine_xyz[0][2]),
        )
        fixed_start_xyz = exact_start
        candidate_positions: list[list[Point3]] = []
        candidate_cells: list[list[Cell3D]] = []
        candidate_obstacle_costs: list[list[float]] = []
        candidate_reference_costs: list[list[float]] = []
        used_radius = int(candidate_radius)
        candidate_native_cpp = False
        while True:
            candidate_positions = []
            candidate_cells = []
            candidate_obstacle_costs = []
            candidate_reference_costs = []
            failed = False

            native_candidates = None
            if candidate_cpp_accel and _native_fastpath is not None:
                try:
                    native_candidates = _native_fastpath.candidate_layers3d(
                        spine_xyz[1:free_layers + 1], search, clearance, frame,
                        radius=used_radius, max_count=per_layer,
                        safe_clearance_voxels=safe_voxels, goal_bias=bias_dir,
                    )
                except Exception:
                    native_candidates = None
            if native_candidates is not None:
                (candidate_positions, candidate_cells,
                 candidate_obstacle_costs, candidate_reference_costs) = native_candidates
                failed = any(len(layer_items) < 1 for layer_items in candidate_cells)
                candidate_native_cpp = not failed
            else:
                for layer in range(1, free_layers + 1):
                    spine_point = spine_xyz[layer]
                    voxels = _candidate_voxels(
                        spine_point, search, frame,
                        radius=used_radius, max_count=per_layer,
                        clearance=clearance, safe_clearance_voxels=safe_voxels,
                        goal_bias=bias_dir,
                    )
                    if len(voxels) < 1:
                        failed = True
                        break
                    cells = [item[0] for item in voxels]
                    positions = [item[1] for item in voxels]
                    costs = [item[2] for item in voxels]
                    candidate_positions.append(positions)
                    candidate_cells.append(cells)
                    candidate_obstacle_costs.append(costs)
                    candidate_reference_costs.append([
                        (point[0] - spine_point[0]) ** 2
                        + (point[1] - spine_point[1]) ** 2
                        + (point[2] - spine_point[2]) ** 2
                        for point in positions
                    ])
            if not failed:
                break
            used_radius += 1
            candidate_native_cpp = False
            if used_radius > 6:
                raise RuntimeError(f"三维候选体素不足：半径{used_radius}内存在空候选层")
        stage_ms["candidate_ms"] = (time.perf_counter() - stage_t) * 1000.0
        stage_t = time.perf_counter()

        # 3) segment-safety lookup.  When inflated occupancy disconnected the
        # start pocket and we deliberately fell back to the raw grid, evaluate
        # candidate links against that same search grid instead of penalising
        # every raw-grid route with the stricter (unreachable) inflated map.
        safe_check_grid = occupancy if strict_safe else search
        blocked_pairs = _blocked_pairs_bulk(candidate_cells, safe_check_grid, frame)
        endpoint_blocked = None
        if _native_fastpath is not None:
            try:
                native_endpoints = _native_fastpath.segments_blocked_xyz3d(
                    [fixed_start_xyz] * len(candidate_positions[0]) + list(candidate_positions[-1]),
                    list(candidate_positions[0]) + [goal_xyz] * len(candidate_positions[-1]),
                    safe_check_grid, frame, samples=8,
                )
            except Exception:
                native_endpoints = None
            if native_endpoints is not None:
                cut = len(candidate_positions[0])
                endpoint_blocked = (native_endpoints[:cut], native_endpoints[cut:])
        if endpoint_blocked is None:
            endpoint_blocked = (
                [
                    _segment_blocked_zyx(fixed_start_xyz, point, safe_check_grid, frame)
                    for point in candidate_positions[0]
                ],
                [
                    _segment_blocked_zyx(point, goal_xyz, safe_check_grid, frame)
                    for point in candidate_positions[-1]
                ],
            )

        fixed_positions: list[Point3] = [fixed_start_xyz, goal_xyz]
        total_layers = free_layers + 2
        spine_cells_resampled = _resample_spine_cells(spine, total_layers)
        spine_chosen: list[int] = []
        for layer in range(free_layers):
            spine_cell = spine_cells_resampled[layer + 1]
            indices = [index for index, cell in enumerate(candidate_cells[layer]) if cell == spine_cell]
            # _candidate_voxels explicitly preserves the baseline as index 0.
            spine_chosen.append(indices[0] if indices else 0)
        stage_ms["segment_ms"] = (time.perf_counter() - stage_t) * 1000.0
        stage_t = time.perf_counter()

        attempts: list[tuple[float, float]] = [(dt, w_acc)]
        result: KinoPlanResult | None = None
        last_error: Exception | None = None
        for attempt_index, (attempt_dt, attempt_w_acc) in enumerate(attempts):
            if cancel_callback is not None and cancel_callback():
                raise RuntimeError("kinodynamic规划已取消")
            qubo_started = time.perf_counter()
            problem, var_map, _penalty, _offset = build_kino_qubo(
                fixed_positions=fixed_positions,
                candidate_positions=candidate_positions,
                candidate_obstacle_costs=candidate_obstacle_costs,
                candidate_reference_costs=candidate_reference_costs,
                candidate_blocked_pairs=blocked_pairs,
                candidate_endpoint_blocked=endpoint_blocked,
                dt=attempt_dt,
                w_vel=w_vel,
                w_acc=attempt_w_acc,
                w_obs=w_obs,
                w_spine=w_spine,
                w_len=w_len,
                w_start_vel=w_start_vel,
                w_start_acc=w_start_acc,
                w_end_vel=w_end_vel,
                segment_penalty=segment_penalty,
                start_vel_xyz=sv,
                start_acc_xyz=sa,
                goal_vel_xyz=gv,
                a_mult=a_mult,
                name=f"uav_kino_{free_layers}layers_{problem_n_hint(free_layers, per_layer)}",
                static_cache=(self._qubo_static_cache if incremental_qubo else None),
                incremental_qubo_cache_size=incremental_qubo_cache_size,
            )
            stage_ms["qubo_ms"] = stage_ms.get("qubo_ms", 0.0) + (time.perf_counter() - qubo_started) * 1000.0
            qubo_static_hit = bool(problem.metadata.get("incremental_qubo_static_hit", False))
            qubo_static_build_ms = float(problem.metadata.get("incremental_qubo_static_build_ms", 0.0) or 0.0)
            qubo_cache_entries = int(problem.metadata.get("incremental_qubo_cache_entries", 0) or 0)
            if trace_callback is not None:
                trace_callback("kino_qubo_start", {
                    "variables": problem.n, "layers": total_layers,
                    "dt": attempt_dt, "iterations": iterations, "restarts": restarts,
                })
            # One complete deterministic backbone assignment is a much better
            # seed hint than the old tuple of partial one-bit assignments.
            spine_binary = [0] * problem.n
            for layer in range(free_layers):
                offset_l, _count_l = var_map[layer + 1]
                spine_binary[offset_l + spine_chosen[layer]] = 1
            warm_starts = (tuple(spine_binary),)

            profiles = _adaptive_solver_profiles(
                problem.n, iterations, restarts, bool(adaptive_solver_profile)
            )
            decoded: list[tuple[list[int], bool, float, bool, bool, int, float]] = []
            all_runs = []
            profile_used: list[tuple[int, int]] = []
            fpga_elapsed = 0.0
            serial_rtt = 0.0
            load_rtt = 0.0
            host_payload_ms = 0.0
            host_fingerprint_ms = 0.0
            host_transport_wall_ms = 0.0
            host_decode_ms = 0.0
            host_solver_total_ms = 0.0
            host_payload_bytes = 0
            host_payload_prepared_bytes = 0
            host_payload_cache_fastpath = False
            host_payload_build_modes: list[str] = []
            native_wire_payload = False
            native_wire_payload_version = 0
            host_load_modes: list[str] = []

            spine_wp = [fixed_start_xyz]
            spine_wp.extend(candidate_positions[layer][spine_chosen[layer]] for layer in range(free_layers))
            spine_wp.append(goal_xyz)
            spine_geom_len = sum(math.dist(a, b) for a, b in zip(spine_wp, spine_wp[1:]))

            solve_started = time.perf_counter()
            solve_decode_score_ms = 0.0
            for profile_index, (profile_iterations, profile_restarts) in enumerate(profiles):
                batch = self.solver.solve_runs(
                    problem,
                    profile_iterations,
                    profile_restarts,
                    seed=(seed_base + attempt_index * 0x6D2B79F5 + profile_index * 0x9E3779B9),
                    warm_starts=warm_starts,
                )
                solver_transactions_total += 1
                hardware_transactions_total += int(any(str(run.transport).upper() in {"UDP", "UART"} for run in batch.runs))
                profile_post_started = time.perf_counter()
                profile_used.append((profile_iterations, profile_restarts))
                host_prof = getattr(self.solver, "last_host_profile", {}) or {}
                host_payload_ms += float(host_prof.get("payload_ms", 0.0) or 0.0)
                host_fingerprint_ms += float(host_prof.get("fingerprint_ms", 0.0) or 0.0)
                host_payload_prepared_bytes += int(host_prof.get("payload_prepared_bytes", 0) or 0)
                host_payload_cache_fastpath = bool(host_payload_cache_fastpath or host_prof.get("payload_cache_fastpath", False))
                host_payload_build_modes.append(str(host_prof.get("payload_build_mode", "") or ""))
                native_wire_payload = bool(native_wire_payload or host_prof.get("native_wire_payload", False))
                native_wire_payload_version = max(native_wire_payload_version, int(host_prof.get("native_wire_payload_version", 0) or 0))
                host_transport_wall_ms += float(host_prof.get("transport_wall_ms", 0.0) or 0.0)
                host_decode_ms += float(host_prof.get("decode_ms", 0.0) or 0.0)
                host_solver_total_ms += float(host_prof.get("host_total_ms", 0.0) or 0.0)
                host_payload_bytes = max(host_payload_bytes, int(host_prof.get("payload_bytes", 0) or 0))
                host_load_modes.append(str(host_prof.get("load_mode", "") or ""))
                if profile_index == 0:
                    load_rtt = float(batch.load_status.serial_rtt_ms)
                for run in batch.runs:
                    all_runs.append(run)
                    fpga_elapsed += float(run.elapsed_us) / 1000.0
                    serial_rtt += float(run.serial_rtt_ms)
                    chosen_run, valid = decode_kino_spins(run.binary, var_map, total_layers)
                    off_layers = 0
                    for layer in range(1, total_layers - 1):
                        offset_l, count_l = var_map[layer]
                        selected = sum(
                            1 for index in range(count_l)
                            if int(run.binary[offset_l + index]) > 0
                        )
                        if selected != 1:
                            off_layers += 1
                    objective_run = float(qubo_value(problem, run.spins) or 0.0)
                    wp = [fixed_start_xyz]
                    wp.extend(
                        candidate_positions[layer][chosen_run[layer]]
                        for layer in range(free_layers)
                    )
                    wp.append(goal_xyz)
                    run_vel = _central_velocities(wp, attempt_dt)
                    run_acc = _central_accelerations(wp, run_vel, attempt_dt)
                    vmax = max((math.hypot(v[0], v[1], v[2]) for v in run_vel), default=0.0)
                    amax = max((math.hypot(a[0], a[1], a[2]) for a in run_acc), default=0.0)
                    geom_len = sum(math.dist(a, b) for a, b in zip(wp, wp[1:]))
                    decoded.append((
                        chosen_run,
                        valid,
                        objective_run,
                        vmax <= max_vel * 1.05 + 1.0e-6,
                        amax <= max_acc * 1.05 + 1.0e-6,
                        off_layers,
                        geom_len,
                    ))

                # Latency-first stopping rule adapted from the standalone
                # Ultra Backbone planner.  Escalate only when the first result
                # is invalid or materially longer than the known A* backbone.
                profile_good = [item for item in decoded if item[1] and item[3] and item[4]]
                stop_after_profile = False
                if profile_good:
                    best_geom = min(item[6] for item in profile_good)
                    tolerance = _quality_tolerance_m(
                        spine_geom_len, frame.resolution_m, quality_gap_ratio,
                        size_profile, spine_source,
                    )
                    if best_geom <= spine_geom_len + tolerance:
                        stop_after_profile = True
                solve_decode_score_ms += (time.perf_counter() - profile_post_started) * 1000.0
                if stop_after_profile:
                    break
            stage_ms["solve_host_ms"] = stage_ms.get("solve_host_ms", 0.0) + (time.perf_counter() - solve_started) * 1000.0
            if not decoded:
                last_error = RuntimeError("FPGA/Mock未返回kinodynamic运行结果")
                continue

            # prefer runs that satisfy one-hot AND the vel/acc limits (avoids
            # retrying the whole QUBO build just because one annealed run was
            # jerky), then fall back to one-hot-valid, then to any run.
            # A synthetic smooth spine run with a huge objective guarantees the
            # first attempt always has a limit-satisfying option.
            spine_vel = _central_velocities(spine_wp, attempt_dt)
            spine_acc = _central_accelerations(spine_wp, spine_vel, attempt_dt)
            spine_vmax = max((math.hypot(v[0], v[1], v[2]) for v in spine_vel), default=0.0)
            spine_amax = max((math.hypot(a[0], a[1], a[2]) for a in spine_acc), default=0.0)
            decoded.append((
                spine_chosen, True, 1.0e12,
                spine_vmax <= max_vel * 1.05 + 1.0e-6,
                spine_amax <= max_acc * 1.05 + 1.0e-6,
                0, spine_geom_len,
            ))
            real_decoded = [item for item in decoded if item[2] < 1.0e11]
            good_runs = [item for item in real_decoded if item[1] and item[3] and item[4]]
            near_good_runs = [item for item in real_decoded if not item[1] and item[3] and item[4]]
            valid_runs = [item for item in real_decoded if item[1]]
            # the synthetic spine run only enters through the full `decoded`
            # list (last resort), never through good/near_good, so a real FPGA
            # argmax path is preferred whenever it satisfies the limits
            pool = good_runs or near_good_runs or valid_runs or decoded
            chosen, onehot_valid, objective, vel_ok, acc_ok, _off, chosen_geom_len = min(
                pool,
                key=lambda item: (0 if item[1] else 1, item[6], item[2]),
            )
            source = "fpga"
            if objective >= 1.0e11:
                # only the synthetic spine run was usable
                source = "deterministic"
                onehot_valid = True
            elif not onehot_valid:
                onehot_valid = False

            quality_forced = False
            raw_fpga_geom_len = float(chosen_geom_len) if source == "fpga" else float("nan")
            quality_recovery_cells: tuple[Cell3D, ...] = ()
            quality_recovery_ms = 0.0
            quality_recovery_geom_len = float("nan")
            quality_tolerance = _quality_tolerance_m(
                spine_geom_len, frame.resolution_m, quality_gap_ratio,
                size_profile, spine_source,
            )
            pre_shortcut_quality_gap = bool(
                quality_guard
                and source == "fpga"
                and chosen_geom_len > spine_geom_len + quality_tolerance
            )

            # deterministic host repair of blocked adjacent-layer segments
            # (validated against the search grid; when the inflated spine was
            # unreachable the raw spine is used and inflated violations near
            # the start/goal pockets are unavoidable and only reported)
            safe_check_grid = occupancy if strict_safe else search
            repaired = _repair_blocked_segments(
                list(chosen), candidate_positions, spine_xyz[1:], safe_check_grid, frame
            )
            repaired = _repair_endpoint_segments(
                repaired, candidate_positions, fixed_start_xyz, goal_xyz, safe_check_grid, frame
            )
            repaired_count = sum(1 for left, right in zip(repaired, chosen) if left != right)
            chosen = repaired

            # Decode the selected free-layer waypoints.  Safety is checked
            # against the same grid used to construct the QUBO; the inflated
            # map is reported separately below when a raw-grid fallback was
            # required for global connectivity.
            waypoints = [fixed_start_xyz]
            for layer in range(free_layers):
                waypoints.append(candidate_positions[layer][chosen[layer]])
            waypoints.append(goal_xyz)
            fallback_reason = "none"
            post_shortcut_applied = False
            post_shortcut_ms = 0.0
            post_shortcut_ratio = 1.0
            if source == "fpga" and len(waypoints) > 2:
                post_t = time.perf_counter()
                before_len = sum(math.dist(a, b) for a, b in zip(waypoints, waypoints[1:]))
                shortened_waypoints = _shortcut_waypoints_xyz(
                    waypoints, safe_check_grid, frame, max_lookahead=12
                )
                after_len = sum(
                    math.dist(a, b) for a, b in zip(shortened_waypoints, shortened_waypoints[1:])
                )
                if (
                    len(shortened_waypoints) >= 2
                    and after_len <= before_len + 1.0e-9
                    and (
                        len(shortened_waypoints) < len(waypoints)
                        or after_len + 1.0e-9 < before_len
                    )
                ):
                    waypoints = list(shortened_waypoints)
                    post_shortcut_applied = True
                    post_shortcut_ratio = float(after_len / max(before_len, 1.0e-9))
                post_shortcut_ms = (time.perf_counter() - post_t) * 1000.0

            waypoint_geom_len = sum(math.dist(a, b) for a, b in zip(waypoints, waypoints[1:]))
            final_quality_gap = bool(
                quality_guard
                and source == "fpga"
                and waypoint_geom_len > spine_geom_len + quality_tolerance
            )
            quality_guard_rescued = bool(
                pre_shortcut_quality_gap and source == "fpga" and not final_quality_gap
            )
            if final_quality_gap:
                if (
                    direct_distance_m <= 4.5
                    and direct_distance_m > 1.0e-6
                    and spine_source in ("previous-safe", "previous-raw", "micro-bypass", "micro-bypass-raw")
                    and spine_geom_len > direct_distance_m * 1.04
                ):
                    qrec_t = time.perf_counter()
                    recovery_bias = None
                    if raw_occupancy is not None and search is raw_occupancy:
                        penalty = np.maximum(0.0, safe_voxels - clearance)
                        recovery_bias = np.where(raw_occupancy == 0, 0.25 * penalty * penalty, 0.0)
                    fresh = _fast_spine_astar_windowed(
                        search, start_zyx, goal_zyx, bias=recovery_bias,
                        margin_voxels=8, fallback_full=False,
                    )
                    if fresh:
                        fresh = _shortcut_spine_cells(fresh, search, max_lookahead=24)
                        fresh_len = _spine_length_m(fresh, frame)
                        if fresh_len + 1.0e-9 < spine_geom_len * 0.98:
                            quality_recovery_cells = tuple(fresh)
                            quality_recovery_geom_len = float(fresh_len)
                    quality_recovery_ms = (time.perf_counter() - qrec_t) * 1000.0

                if quality_recovery_cells:
                    waypoints = [cell3d_to_xyz(cell, frame) for cell in quality_recovery_cells]
                    waypoint_geom_len = float(quality_recovery_geom_len)
                    source = "deterministic"
                    onehot_valid = False
                    quality_forced = True
                    fallback_reason = "quality-refresh"
                else:
                    waypoints = [cell3d_to_xyz(cell, frame) for cell in spine_cells_resampled]
                    waypoint_geom_len = spine_geom_len
                    source = "deterministic"
                    onehot_valid = False
                    quality_forced = True
                    fallback_reason = "quality-gap"
            blocked_after_repair = _chain_blocked_count(waypoints, safe_check_grid, frame)
            if blocked_after_repair > 0 or not _validate_path(waypoints, safe_check_grid, frame):
                # Last-resort deterministic backbone.  Reuse the resampled
                # cells already computed above instead of running the
                # resampling routine again.
                spine_waypoints = [cell3d_to_xyz(cell, frame) for cell in spine_cells_resampled]
                waypoints = spine_waypoints
                waypoint_geom_len = spine_geom_len
                source = "deterministic"
                onehot_valid = False
                fallback_reason = "waypoint"

            (
                waypoints, fallback_exact_start_anchored, fallback_exact_goal_anchored
            ) = _anchor_exact_endpoints_if_safe(
                waypoints, exact_start, exact_goal, safe_check_grid, frame
            )
            fallback_exact_endpoint_anchored = bool(
                fallback_exact_start_anchored and fallback_exact_goal_anchored
            )
            waypoint_geom_len = sum(math.dist(a, b) for a, b in zip(waypoints, waypoints[1:]))

            retime_t = time.perf_counter()
            (
                trajectory_dt, velocities, accelerations, dense, dense_vel, dense_acc,
                dense_max_speed, waypoint_max_acc, retime_expansions,
            ) = _retime_selected_trajectory(
                waypoints, attempt_dt, dt_sample, max_vel, max_acc,
                speed_ratio_target=publish_speed_ratio, max_expansions=4,
                linear_only=(source == "deterministic"),
            )
            stage_ms["retime_ms"] = stage_ms.get("retime_ms", 0.0) + (
                time.perf_counter() - retime_t
            ) * 1000.0
            waypoint_max_vel = max((math.hypot(v[0], v[1], v[2]) for v in velocities), default=0.0)

            # Cubic Hermite can bow laterally and clip a corner even when the
            # waypoint segments are safe.  First retry linear interpolation of
            # the SAME FPGA-selected waypoints.  A safe linear fallback is
            # still an FPGA solution and must not be mislabeled deterministic.
            dense_check_t = time.perf_counter()
            if source != "deterministic" and _chain_blocked_count(dense, safe_check_grid, frame) > 0:
                linear_dense, linear_vel = _dense_linear(waypoints, velocities, trajectory_dt, dt_sample)
                if _chain_blocked_count(linear_dense, safe_check_grid, frame) == 0:
                    dense = linear_dense
                    dense_vel = linear_vel
                    dense_max_speed = _dense_step_speed(dense, dt_sample)
                    dense_acc = _central_accelerations(dense, dense_vel, dt_sample)
                    fallback_reason = "dense-linear"
                else:
                    spine_waypoints = [cell3d_to_xyz(cell, frame) for cell in spine_cells_resampled]
                    waypoints = spine_waypoints
                    (
                        waypoints, dense_spine_exact_start_anchored, dense_spine_exact_goal_anchored
                    ) = _anchor_exact_endpoints_if_safe(
                        waypoints, exact_start, exact_goal, safe_check_grid, frame
                    )
                    fallback_exact_start_anchored = bool(dense_spine_exact_start_anchored)
                    fallback_exact_goal_anchored = bool(dense_spine_exact_goal_anchored)
                    fallback_exact_endpoint_anchored = bool(
                        fallback_exact_start_anchored and fallback_exact_goal_anchored
                    )
                    waypoint_geom_len = sum(math.dist(a, b) for a, b in zip(waypoints, waypoints[1:]))
                    fallback_retime_t = time.perf_counter()
                    (
                        trajectory_dt, velocities, accelerations, dense, dense_vel, dense_acc,
                        dense_max_speed, waypoint_max_acc, fallback_retime_expansions,
                    ) = _retime_selected_trajectory(
                        waypoints, attempt_dt, dt_sample, max_vel, max_acc,
                        speed_ratio_target=publish_speed_ratio, max_expansions=4,
                        linear_only=True,
                    )
                    stage_ms["retime_ms"] = stage_ms.get("retime_ms", 0.0) + (
                        time.perf_counter() - fallback_retime_t
                    ) * 1000.0
                    retime_expansions += fallback_retime_expansions
                    waypoint_max_vel = max((math.hypot(v[0], v[1], v[2]) for v in velocities), default=0.0)
                    source = "deterministic"
                    onehot_valid = False
                    fallback_reason = "dense-spine"
            stage_ms["dense_collision_ms"] = stage_ms.get("dense_collision_ms", 0.0) + (
                time.perf_counter() - dense_check_t
            ) * 1000.0

            # Velocity is enforced on the actually published dense trajectory,
            # not only on central waypoint velocities.  This fixes the old
            # situation where CSV/RViz observed >2 m/s while vel_ok remained
            # true.  Acceleration remains the fixed-dt waypoint dynamics
            # constraint; dense finite-difference acceleration is exposed for
            # diagnostics because piecewise-linear corners are discontinuous.
            dense_max_acc = max((math.hypot(a[0], a[1], a[2]) for a in dense_acc), default=0.0)
            vel_ok = dense_max_speed <= max_vel * 1.005 + 1.0e-6
            acc_ok = waypoint_max_acc <= max_acc * 1.05 + 1.0e-6
            inflated_blocked = _chain_blocked_count(dense, occupancy, frame)

            total_cost = objective if source == "fpga" else float("nan")
            candidate_counts = tuple(len(item) for item in candidate_positions)
            single_candidate_layers = sum(1 for count in candidate_counts if int(count) == 1)
            requested_starts = sum(max(1, int(getattr(run, "restarts", 1))) for run in all_runs)
            board_load_decode_us = sum(int(getattr(run, "load_decode_us", 0)) for run in all_runs)
            board_load_pl_us = sum(int(getattr(run, "load_pl_us", 0)) for run in all_runs)
            board_load_total_us = sum(int(getattr(run, "load_total_us", 0)) for run in all_runs)
            board_transport_rx_us = sum(int(getattr(run, "transport_rx_us", 0)) for run in all_runs)
            board_pl_solve_us = sum(int(getattr(run, "profile_pl_solve_us", 0)) for run in all_runs)
            board_result_read_us = sum(int(getattr(run, "profile_result_read_us", 0)) for run in all_runs)
            board_restart_overhead_us = sum(
                int(getattr(run, "profile_restart_overhead_us", 0)) for run in all_runs
            )
            issued_iteration_budget = sum(
                int(getattr(run, "issued_iteration_budget", 0)) for run in all_runs
            )
            reduced_budgets = [int(getattr(run, "reduced_budget", 0)) for run in all_runs]
            adaptive_infos = [int(getattr(run, "adaptive_info", 0)) for run in all_runs]
            result = KinoPlanResult(
                start_zyx=start_zyx,
                goal_zyx=goal_zyx,
                waypoints_xyz=tuple(waypoints),
                waypoint_velocities_xyz=tuple(velocities),
                waypoint_accelerations_xyz=tuple(accelerations),
                path_xyz=tuple(dense),
                velocities_xyz=tuple(dense_vel),
                accelerations_xyz=tuple(dense_acc),
                path_size=len(dense) + 1,
                dt_s=trajectory_dt,
                dt_sample_s=dt_sample,
                total_cost=float(total_cost),
                source=source,
                onehot_valid=bool(onehot_valid),
                variables=int(problem.n),
                candidate_counts=candidate_counts,
                planning_elapsed_ms=(time.perf_counter() - started) * 1000.0,
                fpga_elapsed_ms=fpga_elapsed,
                serial_rtt_ms=serial_rtt,
                load_rtt_ms=load_rtt,
                fpga_starts=int(requested_starts),
                valid_starts=len(valid_runs),
                max_vel_mps=float(dense_max_speed),
                max_acc_mps2=float(waypoint_max_acc),
                vel_limit_mps=float(max_vel),
                acc_limit_mps2=float(max_acc),
                vel_ok=bool(vel_ok),
                acc_ok=bool(acc_ok),
                attempts=attempt_index + 1,
                details={
                    "encoding": "kino-waypoint-layer-onehot",
                    "native_cpp_accel": bool(
                        _native_fastpath is not None and _native_fastpath.available()
                    ),
                    "candidate_native_cpp": bool(candidate_native_cpp),
                    "candidate_cpp_requested": bool(candidate_cpp_accel),
                    "incremental_qubo": bool(incremental_qubo),
                    "native_qubo": bool(problem.metadata.get("native_qubo", False)),
                    "native_qubo_version": int(problem.metadata.get("native_qubo_version", 0) or 0),
                    "incremental_qubo_static_hit": bool(locals().get("qubo_static_hit", False)),
                    "incremental_qubo_static_build_ms": float(locals().get("qubo_static_build_ms", 0.0)),
                    "incremental_qubo_cache_entries": int(locals().get("qubo_cache_entries", len(self._qubo_static_cache))),
                    "incremental_qubo_cache_size": int(incremental_qubo_cache_size),
                    "clearance_cache_hit": bool(locals().get("clearance_cache_hit", False)),
                    "clearance_cache_probe": bool(locals().get("clearance_diag", {}).get("clearance_cache_probe", False)),
                    "clearance_cache_bypassed": bool(locals().get("clearance_diag", {}).get("clearance_cache_bypassed", False)),
                    "clearance_cache_bypass_remaining": int(locals().get("clearance_diag", {}).get("clearance_cache_bypass_remaining", 0) or 0),
                    "clearance_cache_miss_streak": int(locals().get("clearance_diag", {}).get("clearance_cache_miss_streak", 0) or 0),
                    "clearance_cache_permanent_bypass": bool(locals().get("clearance_diag", {}).get("clearance_cache_permanent_bypass", False)),
                    "clearance_cache_policy": str(locals().get("clearance_diag", {}).get("clearance_cache_policy", self._clearance_cache_policy)),
                    "clearance_cache_identity_mode": str(locals().get("clearance_diag", {}).get("clearance_cache_identity_mode", "unknown")),
                    "clearance_cache_token_present": bool(locals().get("clearance_diag", {}).get("clearance_cache_token_present", False)),
                    "clearance_cache_token_hit": bool(locals().get("clearance_diag", {}).get("clearance_cache_token_hit", False)),
                    "clearance_crc_ms": float(locals().get("clearance_diag", {}).get("clearance_crc_ms", 0.0) or 0.0),
                    "clearance_kernel_ms": float(locals().get("clearance_diag", {}).get("clearance_kernel_ms", 0.0) or 0.0),
                    "grid_validation_ms": float(locals().get("grid_validation_ms", 0.0) or 0.0),
                    "raw_grid_validation_ms": float(locals().get("raw_grid_validation_ms", 0.0) or 0.0),
                    "grid_validation_copy_avoided": bool(locals().get("grid_validation_copy_avoided", False)),
                    "raw_grid_validation_copy_avoided": bool(locals().get("raw_grid_validation_copy_avoided", False)),
                    "trusted_binary_grid": bool(trusted_binary_grid),
                    "native_clearance": bool(
                        _native_fastpath is not None and _native_fastpath.available()
                    ),
                    "native_chain_collision": bool(
                        _native_fastpath is not None and _native_fastpath.available()
                    ),
                    "native_spine_segments": bool(
                        _native_fastpath is not None and _native_fastpath.available()
                    ),
                    "native_retime_dense": bool(
                        _native_fastpath is not None
                        and _native_fastpath.available()
                        and int(_native_fastpath.status().get("version", 0) or 0) >= 3201
                    ),
                    **stage_ms,
                    "solve_dt_s": float(attempt_dt),
                    "dt_unquantized_s": float(dt_unquantized),
                    "qubo_dt_quantum_s": float(qubo_dt_quantum_s),
                    "publish_speed_ratio": float(publish_speed_ratio),
                    "trajectory_dt_s": float(trajectory_dt),
                    "host_payload_ms": float(host_payload_ms),
                    "host_fingerprint_ms": float(host_fingerprint_ms),
                    "host_payload_prepared_bytes": int(host_payload_prepared_bytes),
                    "host_payload_cache_fastpath": bool(host_payload_cache_fastpath),
                    "host_payload_build_modes": host_payload_build_modes,
                    "native_wire_payload": bool(native_wire_payload),
                    "native_wire_payload_version": int(native_wire_payload_version),
                    "host_transport_wall_ms": float(host_transport_wall_ms),
                    "host_decode_ms": float(host_decode_ms),
                    "host_solver_total_ms": float(host_solver_total_ms),
                    "solve_decode_score_ms": float(locals().get("solve_decode_score_ms", 0.0)),
                    "solve_host_unaccounted_ms": float(max(0.0, stage_ms.get("solve_host_ms", 0.0) - host_solver_total_ms - locals().get("solve_decode_score_ms", 0.0))),
                    "host_payload_bytes": int(host_payload_bytes),
                    "host_load_modes": host_load_modes,
                    "solver_transactions": int(len(profile_used)),
                    "solver_transports": [str(run.transport) for run in all_runs],
                    "solver_transactions_total": solver_transactions_total,
                    "hardware_transactions_total": hardware_transactions_total,
                    "solver_profiles": [list(item) for item in profile_used],
                    "spine_nodes": len(spine),
                    "spine_nodes_before_shortcut": int(spine_nodes_before_shortcut),
            "near_hysteresis_refresh": bool(near_hysteresis_refresh),
                    "spine_source": spine_source,
                    "force_spine_refresh": bool(force_spine_refresh),
                    "spine_shortcut_applied": bool(spine_shortcut_applied),
                    "spine_length_before_shortcut_m": float(spine_length_before_shortcut_m),
                    "spine_shortcut_ratio": float(
                        spine_length_m / max(spine_length_before_shortcut_m, 1.0e-9)
                    ),
                    "candidate_radius_voxels": used_radius,
                    "single_candidate_layers": int(single_candidate_layers),
                    "objective": objective,
                    "repaired_candidates": repaired_count,
                    "strict_safe": bool(strict_safe),
                    "raw_spine_used": bool(not strict_safe and raw_occupancy is not None),
                    "inflated_blocked_segments": int(inflated_blocked),
                    "retry_inflated_gate_required": bool(strict_safe),
                    "real_valid": len([i for i in decoded if i[1] and i[2] < 1.0e11]),
                    "real_good": len([i for i in decoded if i[1] and i[3] and i[4] and i[2] < 1.0e11]),
                    "real_near_good": len([i for i in decoded if not i[1] and i[3] and i[4] and i[2] < 1.0e11]),
                    "avg_off_layers": (
                        sum(i[5] for i in decoded if i[2] < 1.0e11)
                        / max(1, len([i for i in decoded if i[2] < 1.0e11]))),
                    "a_mult": float(a_mult),
                    "w_spine": float(w_spine),
                    "w_len": float(w_len),
                    "near_hysteresis_distance_m": float(near_hysteresis_distance_m),
                    "near_hysteresis_detour_ratio": float(near_hysteresis_detour_ratio),
                    "near_hysteresis_improvement_ratio": float(near_hysteresis_improvement_ratio),
                    "near_goal_direct": False,
                    "exact_terminal_goal": bool(exact_terminal_goal),
                    "fallback_exact_endpoint_anchored": bool(locals().get("fallback_exact_endpoint_anchored", True)),
                    "fallback_exact_start_anchored": bool(locals().get("fallback_exact_start_anchored", True)),
                    "fallback_exact_goal_anchored": bool(locals().get("fallback_exact_goal_anchored", True)),
                    "native_clearance_cap2": bool(
                        clearance_cap == 2
                        and _native_fastpath is not None
                        and _native_fastpath.available()
                        and int(_native_fastpath.status().get("version", 0) or 0) >= 3301
                    ),
                    "terminal_goal_target_xyz": [float(v) for v in exact_goal],
                    "terminal_goal_grid_center_xyz": [float(v) for v in grid_goal_xyz],
                    "terminal_goal_quantization_error_m": float(math.dist(exact_goal, grid_goal_xyz)),
                    "spine_latency_fuse_ms": float(strict_spine_latency_fuse_ms),
                    "spine_latency_fuse_triggered": False,
                    "direct_distance_m": float(direct_distance_m),
                    "near_goal_direct_safety_mode": "not-used",
                    "near_goal_direct_cross_speed_actual_mps": None,
                    "retime_expansions": int(retime_expansions),
                    "quality_guard": bool(quality_guard),
                    "quality_tolerance_m": float(quality_tolerance),
                    "raw_fpga_geom_len_m": float(raw_fpga_geom_len),
                    "pre_shortcut_quality_gap": bool(pre_shortcut_quality_gap),
                    "quality_guard_rescued": bool(quality_guard_rescued),
                    "quality_recovery_used": bool(quality_recovery_cells),
                    "quality_recovery_ms": float(quality_recovery_ms),
                    "quality_recovery_geom_len_m": float(quality_recovery_geom_len),
                    "one_hot_penalty": float(problem.metadata.get("one_hot_penalty", 0.0)),
                    "blocked_after_repair": int(blocked_after_repair),
                    "fallback_reason": fallback_reason,
                    "chosen_is_spine": bool(objective >= 1.0e11),
                    "load_modes": [run.load_mode for run in all_runs],
                    "size_profile": size_profile,
                    "spine_length_m": float(spine_length_m),
                    "spine_turn_ratio": float(turn_ratio),
                    "solver_profiles": [list(item) for item in profile_used],
                    "requested_starts": int(requested_starts),
                    "spine_geom_len_m": float(spine_geom_len),
                    "selected_geom_len_m": float(chosen_geom_len),
                    "published_waypoint_geom_len_m": float(waypoint_geom_len),
                    "post_shortcut_applied": bool(post_shortcut_applied),
                    "post_shortcut_ratio": float(post_shortcut_ratio),
                    "post_shortcut_ms": float(post_shortcut_ms),
                    "dense_max_speed_mps": float(dense_max_speed),
                    "waypoint_max_speed_mps": float(waypoint_max_vel),
                    "dense_fd_max_acc_mps2": float(dense_max_acc),
                    # Board-internal profile timing.  `fpga_elapsed_ms` above is
                    # the firmware multistart wall time; these counters expose
                    # how much was actual PL solving versus load/result overhead.
                    "board_load_decode_ms": board_load_decode_us / 1000.0,
                    "board_load_pl_ms": board_load_pl_us / 1000.0,
                    "board_load_total_ms": board_load_total_us / 1000.0,
                    "board_transport_rx_ms": board_transport_rx_us / 1000.0,
                    "board_pl_solve_ms": board_pl_solve_us / 1000.0,
                    "board_result_read_ms": board_result_read_us / 1000.0,
                    "board_restart_overhead_ms": board_restart_overhead_us / 1000.0,
                    "issued_iteration_budget": int(issued_iteration_budget),
                    "reduced_budgets": reduced_budgets,
                    "adaptive_infos": adaptive_infos,
                    "early_restart_stopped": any((value & 0x40000000) != 0 for value in adaptive_infos),
                },
            )
            retry_collision_ok = (inflated_blocked == 0) if strict_safe else True
            if vel_ok and acc_ok and retry_collision_ok:
                break
        if result is None:
            raise last_error or RuntimeError("kinodynamic规划失败")
        if trace_callback is not None:
            trace_callback("kino_done", {
                "source": result.source, "path_size": result.path_size,
                "max_vel": result.max_vel_mps, "max_acc": result.max_acc_mps2,
                "vel_ok": result.vel_ok, "acc_ok": result.acc_ok,
                "attempts": result.attempts, "fpga_ms": result.fpga_elapsed_ms,
            })
        return result


def problem_n_hint(free_layers: int, per_layer: int) -> int:
    return free_layers * per_layer


def _resample_spine_cells(spine: Sequence[Cell3D], layers: int) -> list[Cell3D]:
    """Pick `layers` evenly spaced voxels from the spine chain."""
    if len(spine) <= 1:
        return [spine[0]] * layers
    indices = [int(round(index * (len(spine) - 1) / max(1, layers - 1))) for index in range(layers)]
    return [spine[index] for index in indices]
