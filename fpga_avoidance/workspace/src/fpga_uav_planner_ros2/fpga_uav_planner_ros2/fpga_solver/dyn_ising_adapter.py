"""Build metric occupancy grids and resolve the original planner's request parameters."""
from __future__ import annotations

import math
from functools import lru_cache
from dataclasses import asdict, dataclass
from typing import Any, Sequence, Tuple

import numpy as np

try:
    import native_fastpath as _native_fastpath
except Exception:  # pragma: no cover - optional accelerator
    _native_fastpath = None


Cell = Tuple[int, int]
Cell3D = Tuple[int, int, int]
Point3 = Tuple[float, float, float]

# Cell units: tolerate only floating error in an intended integer-aligned origin.
_GRID_ALIGNMENT_TOLERANCE = 1.0e-10


def _axis_cells(values, origin: float, resolution: float):
    """Quantise aligned grids globally before subtracting their integer offset."""
    origin_units = origin / resolution
    offset = round(origin_units)
    if abs(origin_units - offset) <= _GRID_ALIGNMENT_TOLERANCE:
        # Subtracting a floating local origin before floor can move boundary points one cell.
        scaled = values / resolution
    else:
        scaled = (values - origin) / resolution
        offset = 0
    if np.isscalar(scaled):
        return math.floor(scaled) - offset
    return np.floor(scaled).astype(np.int64) - offset


@dataclass(frozen=True)
class GridFrame:
    frame_id: str
    resolution_m: float
    x_min: float
    y_min: float
    rows: int
    cols: int
    height_center_m: float

    @property
    def x_max(self) -> float:
        return self.x_min + self.cols * self.resolution_m

    @property
    def y_max(self) -> float:
        return self.y_min + self.rows * self.resolution_m

    def to_dict(self) -> dict[str, object]:
        data = asdict(self)
        data["x_max"] = self.x_max
        data["y_max"] = self.y_max
        return data


@dataclass(frozen=True)
class IsingGridProblem:
    frame: GridFrame
    grid: np.ndarray
    start: Cell
    goal: Cell
    previous_path: tuple[Cell, ...] = ()
    metadata: dict[str, object] | None = None

    def to_debug_dict(self) -> dict[str, object]:
        return {
            "frame": self.frame.to_dict(),
            "shape": [int(self.grid.shape[0]), int(self.grid.shape[1])],
            "start": list(self.start),
            "goal": list(self.goal),
            "blocked": int(np.count_nonzero(self.grid)),
            "previous_path_len": len(self.previous_path),
            "metadata": dict(self.metadata or {}),
        }


@dataclass(frozen=True)
class GridFrame3D:
    frame_id: str
    resolution_m: float
    resolution_z_m: float
    x_min: float
    y_min: float
    z_min: float
    layers: int
    rows: int
    cols: int

    @property
    def x_max(self) -> float:
        return self.x_min + self.cols * self.resolution_m

    @property
    def y_max(self) -> float:
        return self.y_min + self.rows * self.resolution_m

    @property
    def z_max(self) -> float:
        return self.z_min + self.layers * self.resolution_z_m

    def to_dict(self) -> dict[str, object]:
        data = asdict(self)
        data["x_max"] = self.x_max
        data["y_max"] = self.y_max
        data["z_max"] = self.z_max
        data["axis_order"] = "zyx"
        return data


@dataclass(frozen=True)
class IsingGrid3DProblem:
    frame: GridFrame3D
    grid3d: np.ndarray
    start: Cell3D
    goal: Cell3D
    previous_path: tuple[Cell3D, ...] = ()
    metadata: dict[str, object] | None = None
    raw_grid3d: np.ndarray | None = None

    @property
    def is_3d(self) -> bool:
        return True

    @property
    def grid(self) -> np.ndarray:
        return self.grid3d

    def to_debug_dict(self) -> dict[str, object]:
        raw_info = {}
        if self.raw_grid3d is not None:
            raw_info = {
                "raw_blocked_voxels": int(np.count_nonzero(self.raw_grid3d)),
            }
        return {
            "frame": self.frame.to_dict(),
            "shape_zyx": [int(x) for x in self.grid3d.shape],
            "start_zyx": list(self.start),
            "goal_zyx": list(self.goal),
            "blocked_voxels": int(np.count_nonzero(self.grid3d)),
            "previous_path_len": len(self.previous_path),
            "metadata": dict(self.metadata or {}),
            **raw_info,
        }


def _float_value(payload: dict[str, Any], key: str, default: float) -> float:
    value = payload.get(key, default)
    try:
        value = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{key}必须是数字") from exc
    if not math.isfinite(value):
        raise ValueError(f"{key}必须是有限数字")
    return value


def _positive_float_value(payload: dict[str, Any], key: str, default: float) -> float:
    value = _float_value(payload, key, default)
    if value <= 0:
        raise ValueError(f"{key}必须为正数")
    return value


def _nonnegative_float_value(payload: dict[str, Any], key: str, default: float) -> float:
    value = _float_value(payload, key, default)
    if value < 0:
        raise ValueError(f"{key}不能为负数")
    return value


def _int_value(payload: dict[str, Any], key: str, default: int) -> int:
    value = payload.get(key, default)
    try:
        return int(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{key}必须是整数") from exc


def _bounded_int_value(
    payload: dict[str, Any],
    key: str,
    default: int,
    *,
    minimum: int,
    maximum: int | None = None,
) -> int:
    value = _int_value(payload, key, default)
    if value < minimum:
        raise ValueError(f"{key}不能小于{minimum}")
    if maximum is not None and value > maximum:
        raise ValueError(f"{key}不能大于{maximum}")
    return value


def _is_sequence(value: Any) -> bool:
    return isinstance(value, Sequence) and not isinstance(value, (str, bytes, bytearray))


def _shape2(value: Any, name: str) -> tuple[int, int]:
    rows, cols = _cell(value, name)
    if rows <= 0 or cols <= 0:
        raise ValueError(f"{name}必须是正整数[row, col]")
    return rows, cols


def _shape3(value: Any, name: str) -> tuple[int, int, int]:
    layers, rows, cols = _cell3(value, name)
    if layers <= 0 or rows <= 0 or cols <= 0:
        raise ValueError(f"{name}必须是正整数[z, row, col]")
    return layers, rows, cols


def _point3(value: Any, name: str) -> Point3:
    if not _is_sequence(value) or len(value) < 3:
        raise ValueError(f"{name}必须是[x, y, z]")
    try:
        point = (float(value[0]), float(value[1]), float(value[2]))
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{name}必须是数字坐标") from exc
    if not all(math.isfinite(item) for item in point):
        raise ValueError(f"{name}包含非有限坐标")
    return point


def _cell(value: Any, name: str) -> Cell:
    if not _is_sequence(value) or len(value) < 2:
        raise ValueError(f"{name}必须是[row, col]")
    try:
        return int(value[0]), int(value[1])
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{name}必须是整数cell") from exc


def _cell3(value: Any, name: str) -> Cell3D:
    if not _is_sequence(value) or len(value) < 3:
        raise ValueError(f"{name}必须是[z, row, col]")
    try:
        return int(value[0]), int(value[1]), int(value[2])
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{name}必须是整数3D cell") from exc


def _points3(value: Any, name: str) -> list[Point3]:
    if value is None:
        return []
    if not _is_sequence(value):
        raise ValueError(f"{name}必须是点列表")
    return [_point3(item, f"{name}[{index}]") for index, item in enumerate(value)]


def _points3_numpy(value: Any, name: str, *, assume_finite: bool = False) -> np.ndarray:
    """Validate a point list while preserving a NumPy N×3 fast path.

    The embedded ROS node already extracts PointCloud2 into a contiguous NumPy
    array.  Converting it back to thousands of Python tuples and immediately
    into NumPy again during voxelisation was pure overhead.
    """
    if value is None:
        return np.empty((0, 3), dtype=np.float64)
    try:
        points = np.asarray(value, dtype=np.float64)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{name}必须是N×3点列表") from exc
    if points.size == 0:
        return np.empty((0, 3), dtype=np.float64)
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError(f"{name}必须是N×3点列表")
    if not assume_finite and not np.all(np.isfinite(points)):
        raise ValueError(f"{name}包含非有限数字")
    return np.ascontiguousarray(points, dtype=np.float64)


def xyz_to_cell(x: float, y: float, frame: GridFrame) -> Cell:
    col = int(math.floor((float(x) - frame.x_min) / frame.resolution_m))
    row = int(math.floor((float(y) - frame.y_min) / frame.resolution_m))
    return row, col


def xyz_to_cell3d(x: float, y: float, z: float, frame: GridFrame3D) -> Cell3D:
    """Use the same stable voxel mapping for endpoints and bulk obstacle points."""
    col = _axis_cells(float(x), frame.x_min, frame.resolution_m)
    row = _axis_cells(float(y), frame.y_min, frame.resolution_m)
    layer = _axis_cells(float(z), frame.z_min, frame.resolution_z_m)
    return layer, row, col


def cell_to_xyz(cell: Cell, frame: GridFrame, z: float | None = None) -> Point3:
    row, col = cell
    return (
        frame.x_min + (col + 0.5) * frame.resolution_m,
        frame.y_min + (row + 0.5) * frame.resolution_m,
        frame.height_center_m if z is None else float(z),
    )


def cell3d_to_xyz(cell: Cell3D, frame: GridFrame3D) -> Point3:
    layer, row, col = cell
    return (
        frame.x_min + (col + 0.5) * frame.resolution_m,
        frame.y_min + (row + 0.5) * frame.resolution_m,
        frame.z_min + (layer + 0.5) * frame.resolution_z_m,
    )


def inside_grid(cell: Cell, shape: tuple[int, int]) -> bool:
    return 0 <= cell[0] < shape[0] and 0 <= cell[1] < shape[1]


def inside_grid3d(cell: Cell3D, shape: tuple[int, int, int]) -> bool:
    return 0 <= cell[0] < shape[0] and 0 <= cell[1] < shape[1] and 0 <= cell[2] < shape[2]


def inflate_obstacles(obstacles: np.ndarray, radius: int) -> np.ndarray:
    source = np.asarray(obstacles, dtype=bool)
    if source.ndim != 2:
        raise ValueError("obstacles必须是二维矩阵")
    radius = max(0, int(radius))
    if radius <= 0:
        return source.copy()
    rows, cols = source.shape
    result = source.copy()
    for drow in range(-radius, radius + 1):
        for dcol in range(-radius, radius + 1):
            if drow * drow + dcol * dcol > radius * radius:
                continue
            source_rows = slice(max(0, -drow), min(rows, rows - drow))
            source_cols = slice(max(0, -dcol), min(cols, cols - dcol))
            target_rows = slice(max(0, drow), min(rows, rows + drow))
            target_cols = slice(max(0, dcol), min(cols, cols + dcol))
            result[target_rows, target_cols] |= source[source_rows, source_cols]
    return result


@lru_cache(maxsize=32)
def _inflation_offsets_3d(
    radius_m: float, resolution_m: float, resolution_z_m: float
) -> tuple[tuple[int, int, int], ...]:
    """Return the metric-spherical dilation offsets for a fixed grid setup.

    In the flight node radius/resolution are static ROS parameters, so the old
    implementation needlessly rebuilt the same ~hundreds of candidate offsets
    and repeated sqrt() tests on every replan.
    """
    radius_m = max(0.0, float(radius_m))
    resolution_m = max(float(resolution_m), 1.0e-9)
    resolution_z_m = max(float(resolution_z_m), 1.0e-9)
    max_dz = int(math.ceil(radius_m / resolution_z_m))
    max_dr = int(math.ceil(radius_m / resolution_m))
    radius2 = radius_m * radius_m + 1.0e-15
    offsets: list[tuple[int, int, int]] = []
    for dz in range(-max_dz, max_dz + 1):
        z2 = (dz * resolution_z_m) ** 2
        for drow in range(-max_dr, max_dr + 1):
            zr2 = z2 + (drow * resolution_m) ** 2
            for dcol in range(-max_dr, max_dr + 1):
                if zr2 + (dcol * resolution_m) ** 2 <= radius2:
                    offsets.append((dz, drow, dcol))
    return tuple(offsets)


def inflate_obstacles_3d(
    obstacles: np.ndarray,
    *,
    radius_m: float,
    resolution_m: float,
    resolution_z_m: float,
) -> np.ndarray:
    source = np.asarray(obstacles, dtype=bool)
    if source.ndim != 3:
        raise ValueError("obstacles必须是三维矩阵")
    radius_m = max(0.0, float(radius_m))
    if radius_m <= 0.0:
        return source.copy()
    layers, rows, cols = source.shape
    result = source.copy()
    # Round immutable ROS parameters before caching so numerically equivalent
    # values share the same offset table.
    offsets = _inflation_offsets_3d(
        round(radius_m, 9), round(float(resolution_m), 9), round(float(resolution_z_m), 9)
    )
    for dz, drow, dcol in offsets:
        source_z = slice(max(0, -dz), min(layers, layers - dz))
        source_rows = slice(max(0, -drow), min(rows, rows - drow))
        source_cols = slice(max(0, -dcol), min(cols, cols - dcol))
        target_z = slice(max(0, dz), min(layers, layers + dz))
        target_rows = slice(max(0, drow), min(rows, rows + drow))
        target_cols = slice(max(0, dcol), min(cols, cols + dcol))
        result[target_z, target_rows, target_cols] |= source[source_z, source_rows, source_cols]
    return result


def path_xyz_to_cells(path_xyz: Sequence[Sequence[float]], frame: GridFrame) -> tuple[Cell, ...]:
    cells: list[Cell] = []
    for point in path_xyz:
        x, y, _z = _point3(point, "previous_path_xyz")
        cell = xyz_to_cell(x, y, frame)
        if inside_grid(cell, (frame.rows, frame.cols)) and (not cells or cells[-1] != cell):
            cells.append(cell)
    return tuple(cells)


def path_xyz_to_cells3d(path_xyz: Sequence[Sequence[float]], frame: GridFrame3D) -> tuple[Cell3D, ...]:
    cells: list[Cell3D] = []
    for point in path_xyz:
        x, y, z = _point3(point, "previous_path_xyz")
        cell = xyz_to_cell3d(x, y, z, frame)
        if inside_grid3d(cell, (frame.layers, frame.rows, frame.cols)) and (not cells or cells[-1] != cell):
            cells.append(cell)
    return tuple(cells)


def path_cells_to_cells(path_cells: Any, shape: tuple[int, int]) -> tuple[Cell, ...]:
    if path_cells is None:
        return ()
    if not _is_sequence(path_cells):
        raise ValueError("previous_path必须是cell列表")
    cells: list[Cell] = []
    for index, value in enumerate(path_cells):
        cell = _cell(value, f"previous_path[{index}]")
        if inside_grid(cell, shape) and (not cells or cells[-1] != cell):
            cells.append(cell)
    return tuple(cells)


def path_cells3d_to_cells3d(path_cells: Any, shape: tuple[int, int, int]) -> tuple[Cell3D, ...]:
    if path_cells is None:
        return ()
    if not _is_sequence(path_cells):
        raise ValueError("previous_path_3d必须是cell列表")
    cells: list[Cell3D] = []
    for index, value in enumerate(path_cells):
        cell = _cell3(value, f"previous_path_3d[{index}]")
        if inside_grid3d(cell, shape) and (not cells or cells[-1] != cell):
            cells.append(cell)
    return tuple(cells)


def cells_to_xyz_path(path: Sequence[Cell], frame: GridFrame, z: float | None = None) -> list[list[float]]:
    return [list(cell_to_xyz(cell, frame, z)) for cell in path]


def cells3d_to_xyz_path(path: Sequence[Cell3D], frame: GridFrame3D) -> list[list[float]]:
    return [list(cell3d_to_xyz(cell, frame)) for cell in path]


def _axis_bounds(
    start: Point3,
    goal: Point3,
    resolution_m: float,
    margin_m: float,
    window_radius_m: float,
    max_axis_cells: int,
) -> tuple[float, float, float, float, Point3, bool]:
    dx, dy = goal[0] - start[0], goal[1] - start[1]
    distance_xy = math.hypot(dx, dy)
    clipped = False
    effective_goal = goal
    if window_radius_m > 0 and distance_xy > window_radius_m:
        scale = window_radius_m / max(distance_xy, 1.0e-9)
        effective_goal = (
            start[0] + dx * scale,
            start[1] + dy * scale,
            goal[2],
        )
        clipped = True
    xs = [start[0], effective_goal[0]]
    ys = [start[1], effective_goal[1]]
    x_min = math.floor((min(xs) - margin_m) / resolution_m) * resolution_m
    y_min = math.floor((min(ys) - margin_m) / resolution_m) * resolution_m
    x_max = math.ceil((max(xs) + margin_m) / resolution_m) * resolution_m
    y_max = math.ceil((max(ys) + margin_m) / resolution_m) * resolution_m
    cols = max(2, int(math.ceil((x_max - x_min) / resolution_m)))
    rows = max(2, int(math.ceil((y_max - y_min) / resolution_m)))
    if rows > max_axis_cells or cols > max_axis_cells:
        raise ValueError(
            f"自动局部栅格过大: {rows}x{cols}，请调大resolution_m、减小window_radius_m，"
            f"或提高max_axis_cells"
        )
    return x_min, y_min, rows, cols, effective_goal, clipped


def make_grid_frame(
    start_xyz: Point3,
    goal_xyz: Point3,
    *,
    frame_id: str = "camera_init",
    resolution_m: float = 0.2,
    window_margin_m: float = 2.0,
    window_radius_m: float = 8.0,
    max_axis_cells: int = 160,
    height_center_m: float | None = None,
) -> tuple[GridFrame, Point3, bool]:
    if resolution_m <= 0:
        raise ValueError("resolution_m必须为正数")
    if max_axis_cells < 2:
        raise ValueError("max_axis_cells必须不小于2")
    x_min, y_min, rows, cols, effective_goal, clipped = _axis_bounds(
        start_xyz,
        goal_xyz,
        resolution_m,
        max(0.0, window_margin_m),
        max(0.0, window_radius_m),
        max_axis_cells,
    )
    frame = GridFrame(
        frame_id=str(frame_id or "camera_init"),
        resolution_m=float(resolution_m),
        x_min=float(x_min),
        y_min=float(y_min),
        rows=int(rows),
        cols=int(cols),
        height_center_m=float(start_xyz[2] if height_center_m is None else height_center_m),
    )
    return frame, effective_goal, clipped


def pointcloud_to_grid(
    points_xyz: Sequence[Point3],
    frame: GridFrame,
    *,
    height_min_m: float,
    height_max_m: float,
) -> np.ndarray:
    if height_min_m > height_max_m:
        raise ValueError("height_min_m不能大于height_max_m")
    grid = np.zeros((frame.rows, frame.cols), dtype=np.uint8)
    for x, y, z in points_xyz:
        if z < height_min_m or z > height_max_m:
            continue
        cell = xyz_to_cell(x, y, frame)
        if inside_grid(cell, grid.shape):
            grid[cell] = 1
    return grid


def make_grid_frame3d(
    start_xyz: Point3,
    goal_xyz: Point3,
    *,
    frame_id: str = "camera_init",
    resolution_m: float = 0.2,
    resolution_z_m: float | None = None,
    window_margin_m: float = 2.0,
    window_radius_m: float = 8.0,
    z_margin_m: float = 1.0,
    z_min_m: float | None = None,
    z_max_m: float | None = None,
    max_axis_cells: int = 160,
    max_z_cells: int = 32,
) -> tuple[GridFrame3D, Point3, bool]:
    resolution_z_m = float(resolution_m if resolution_z_m is None else resolution_z_m)
    if resolution_m <= 0 or resolution_z_m <= 0:
        raise ValueError("resolution_m/resolution_z_m必须为正数")
    if max_axis_cells < 2 or max_z_cells < 2:
        raise ValueError("max_axis_cells/max_z_cells必须不小于2")
    x_min, y_min, rows, cols, effective_goal, clipped = _axis_bounds(
        start_xyz,
        goal_xyz,
        resolution_m,
        max(0.0, window_margin_m),
        max(0.0, window_radius_m),
        max_axis_cells,
    )
    if z_min_m is None:
        z_min = math.floor((min(start_xyz[2], effective_goal[2]) - max(0.0, z_margin_m)) / resolution_z_m) * resolution_z_m
    else:
        z_min = float(z_min_m)
    if z_max_m is None:
        z_max = math.ceil((max(start_xyz[2], effective_goal[2]) + max(0.0, z_margin_m)) / resolution_z_m) * resolution_z_m
    else:
        z_max = float(z_max_m)
    if z_max <= z_min:
        raise ValueError("z_max_m必须大于z_min_m")
    layers = max(2, int(math.ceil((z_max - z_min) / resolution_z_m)))
    if layers > max_z_cells:
        raise ValueError(
            f"自动高度层过多: {layers}，请调大resolution_z_m、减小z_margin_m，"
            f"或提高max_z_cells"
        )
    frame = GridFrame3D(
        frame_id=str(frame_id or "camera_init"),
        resolution_m=float(resolution_m),
        resolution_z_m=float(resolution_z_m),
        x_min=float(x_min),
        y_min=float(y_min),
        z_min=float(z_min),
        layers=int(layers),
        rows=int(rows),
        cols=int(cols),
    )
    return frame, effective_goal, clipped


def pointcloud_to_grid3d(
    points_xyz: Sequence[Point3],
    frame: GridFrame3D,
    *,
    assume_finite: bool = False,
) -> np.ndarray:
    """Voxelise a point cloud with a vectorised NumPy path.

    The ROS node commonly forwards several thousand points per replan.  The
    former Python loop called xyz_to_cell3d once per point; this implementation
    performs the same floor-based mapping in bulk and writes unique valid
    voxels in one indexed assignment.
    """
    grid = np.zeros((frame.layers, frame.rows, frame.cols), dtype=np.uint8)
    if points_xyz is None or len(points_xyz) == 0:
        return grid
    points = np.asarray(points_xyz, dtype=np.float64)
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError("points_xyz必须是N×3点列表")
    if not assume_finite:
        finite = np.all(np.isfinite(points), axis=1)
        points = points[finite]
        if points.size == 0:
            return grid
    cols = _axis_cells(points[:, 0], frame.x_min, frame.resolution_m)
    rows = _axis_cells(points[:, 1], frame.y_min, frame.resolution_m)
    layers = _axis_cells(points[:, 2], frame.z_min, frame.resolution_z_m)
    valid = (
        (layers >= 0) & (layers < frame.layers)
        & (rows >= 0) & (rows < frame.rows)
        & (cols >= 0) & (cols < frame.cols)
    )
    if np.any(valid):
        grid[layers[valid], rows[valid], cols[valid]] = 1
    return grid


def _grid_from_payload(payload: dict[str, Any]) -> tuple[np.ndarray, Cell, Cell, GridFrame, tuple[Cell, ...], dict[str, object]]:
    grid_value = payload.get("grid", payload.get("occupancy"))
    if grid_value is None:
        raise ValueError("直接grid模式必须提供grid或occupancy")
    try:
        grid_array = np.asarray(grid_value)
    except (TypeError, ValueError) as exc:
        raise ValueError("grid/occupancy必须是规则的数字数组") from exc
    if not (
        np.issubdtype(grid_array.dtype, np.number)
        or np.issubdtype(grid_array.dtype, np.bool_)
    ):
        raise ValueError("grid/occupancy必须只包含数字")
    if not np.all(np.isfinite(grid_array.astype(float, copy=False))):
        raise ValueError("grid/occupancy包含非有限数字")
    if np.any(grid_array < 0):
        raise ValueError("grid/occupancy不能包含负数")
    if grid_array.size == 0:
        raise ValueError("grid必须是非空二维矩阵")
    grid_array = (grid_array != 0).astype(np.uint8)
    if grid_array.ndim == 1:
        if "grid_shape" not in payload:
            raise ValueError("扁平grid/occupancy必须同时提供grid_shape")
        rows, cols = _shape2(payload["grid_shape"], "grid_shape")
        if grid_array.size != rows * cols:
            raise ValueError(
                f"grid_shape={rows}x{cols}需要{rows * cols}个元素，"
                f"实际为{grid_array.size}个"
            )
        grid = grid_array.reshape((rows, cols))
    else:
        grid = grid_array
    if grid.ndim != 2 or not grid.size:
        raise ValueError("grid必须是非空二维矩阵")
    if "grid_shape" in payload:
        expected_rows, expected_cols = _shape2(payload["grid_shape"], "grid_shape")
        if grid.shape != (expected_rows, expected_cols):
            raise ValueError(
                f"grid实际形状为{grid.shape[0]}x{grid.shape[1]}，"
                f"与grid_shape={expected_rows}x{expected_cols}不一致"
            )
    start = _cell(payload.get("start", payload.get("start_cell")), "start")
    goal = _cell(payload.get("goal", payload.get("goal_cell")), "goal")
    if not inside_grid(start, grid.shape) or not inside_grid(goal, grid.shape):
        raise ValueError("start/goal超出grid")
    frame_payload = payload.get("frame") or {}
    if not isinstance(frame_payload, dict):
        raise ValueError("frame必须是对象")
    frame_source = dict(payload)
    frame_source.update(frame_payload)
    resolution_m = _positive_float_value(frame_source, "resolution_m", 1.0)
    x_min = _float_value(frame_source, "x_min", 0.0)
    y_min = _float_value(frame_source, "y_min", 0.0)
    height_center_m = _float_value(frame_source, "height_center_m", 0.0)
    frame = GridFrame(
        frame_id=str(frame_source.get("frame_id", "camera_init")),
        resolution_m=resolution_m,
        x_min=x_min,
        y_min=y_min,
        rows=int(grid.shape[0]),
        cols=int(grid.shape[1]),
        height_center_m=height_center_m,
    )
    previous = path_cells_to_cells(
        payload.get("previous_path", payload.get("previous_path_cells", [])),
        grid.shape,
    )
    raw_grid = (grid != 0).astype(np.uint8)
    if payload.get("grid_already_inflated", False):
        inflated_cells = 0
        inflated_grid = raw_grid
    elif "inflate_cells" in payload:
        inflated_cells = _int_value(payload, "inflate_cells", 0)
        if inflated_cells < 0:
            raise ValueError("inflate_cells不能为负数")
        inflated_grid = inflate_obstacles(raw_grid, inflated_cells).astype(np.uint8)
    elif "uav_radius_m" in payload or "safe_dist_m" in payload or "safety_margin_m" in payload:
        if "uav_radius_m" in payload:
            radius_m = _nonnegative_float_value(payload, "uav_radius_m", 0.0)
        else:
            radius_m = _nonnegative_float_value(payload, "safe_dist_m", 0.0)
        margin_m = _nonnegative_float_value(payload, "safety_margin_m", 0.0)
        inflated_cells = int(math.ceil((radius_m + margin_m) / resolution_m))
        inflated_grid = inflate_obstacles(raw_grid, inflated_cells).astype(np.uint8)
    else:
        inflated_cells = 0
        inflated_grid = raw_grid
    metadata = {
        "source": "direct_grid",
        "raw_blocked_cells": int(np.count_nonzero(raw_grid)),
        "inflated_cells": int(inflated_cells),
        "goal_clipped": False,
    }
    return inflated_grid, start, goal, frame, previous, metadata


def _grid3d_from_payload(payload: dict[str, Any]) -> tuple[np.ndarray, Cell3D, Cell3D, GridFrame3D, tuple[Cell3D, ...], dict[str, object]]:
    grid_value = payload.get("grid3d", payload.get("occupancy_3d", payload.get("grid")))
    if grid_value is None:
        raise ValueError("三维grid模式必须提供grid3d、occupancy_3d或三维grid")
    try:
        grid_array = np.asarray(grid_value)
    except (TypeError, ValueError) as exc:
        raise ValueError("grid3d/occupancy_3d必须是规则的数字数组") from exc
    if not (
        np.issubdtype(grid_array.dtype, np.number)
        or np.issubdtype(grid_array.dtype, np.bool_)
    ):
        raise ValueError("grid3d/occupancy_3d必须只包含数字")
    if not np.all(np.isfinite(grid_array.astype(float, copy=False))):
        raise ValueError("grid3d/occupancy_3d包含非有限数字")
    if np.any(grid_array < 0):
        raise ValueError("grid3d/occupancy_3d不能包含负数")
    if grid_array.size == 0:
        raise ValueError("grid3d必须是非空三维矩阵")
    grid_array = (grid_array != 0).astype(np.uint8)
    shape_value = payload.get("grid_shape_3d", payload.get("shape_zyx"))
    if grid_array.ndim == 1:
        if shape_value is None:
            raise ValueError("扁平grid3d必须同时提供grid_shape_3d")
        layers, rows, cols = _shape3(shape_value, "grid_shape_3d")
        if grid_array.size != layers * rows * cols:
            raise ValueError(
                f"grid_shape_3d={layers}x{rows}x{cols}需要{layers * rows * cols}个元素，"
                f"实际为{grid_array.size}个"
            )
        grid = grid_array.reshape((layers, rows, cols))
    else:
        grid = grid_array
    if grid.ndim != 3 or not grid.size:
        raise ValueError("grid3d必须是非空三维矩阵，轴顺序为[z,row,col]")
    if shape_value is not None:
        expected = _shape3(shape_value, "grid_shape_3d")
        if grid.shape != expected:
            raise ValueError(
                f"grid3d实际形状为{grid.shape}，与grid_shape_3d={expected}不一致"
            )

    start = _cell3(
        payload.get("start_zyx", payload.get("start_cell_3d", payload.get("start"))),
        "start_zyx",
    )
    goal = _cell3(
        payload.get("goal_zyx", payload.get("goal_cell_3d", payload.get("goal"))),
        "goal_zyx",
    )
    if not inside_grid3d(start, grid.shape) or not inside_grid3d(goal, grid.shape):
        raise ValueError("start_zyx/goal_zyx超出grid3d")

    frame_payload = payload.get("frame_3d", payload.get("frame") or {})
    if not isinstance(frame_payload, dict):
        raise ValueError("frame_3d/frame必须是对象")
    frame_source = dict(payload)
    frame_source.update(frame_payload)
    resolution_m = _positive_float_value(frame_source, "resolution_m", 1.0)
    resolution_z_m = _positive_float_value(frame_source, "resolution_z_m", resolution_m)
    frame = GridFrame3D(
        frame_id=str(frame_source.get("frame_id", "camera_init")),
        resolution_m=resolution_m,
        resolution_z_m=resolution_z_m,
        x_min=_float_value(frame_source, "x_min", 0.0),
        y_min=_float_value(frame_source, "y_min", 0.0),
        z_min=_float_value(frame_source, "z_min", 0.0),
        layers=int(grid.shape[0]),
        rows=int(grid.shape[1]),
        cols=int(grid.shape[2]),
    )

    previous = path_cells3d_to_cells3d(
        payload.get("previous_path_zyx", payload.get("previous_path_3d", [])),
        grid.shape,
    )
    raw_grid = (grid != 0).astype(np.uint8)
    if payload.get("grid_already_inflated", False):
        inflate_radius_m = 0.0
        inflated_grid = raw_grid
    elif "inflate_cells_3d" in payload:
        cells = _int_value(payload, "inflate_cells_3d", 0)
        if cells < 0:
            raise ValueError("inflate_cells_3d不能为负数")
        inflate_radius_m = cells * min(resolution_m, resolution_z_m)
        inflated_grid = inflate_obstacles_3d(
            raw_grid,
            radius_m=inflate_radius_m,
            resolution_m=resolution_m,
            resolution_z_m=resolution_z_m,
        ).astype(np.uint8)
    elif "uav_radius_m" in payload or "safe_dist_m" in payload or "safety_margin_m" in payload:
        if "uav_radius_m" in payload:
            radius_m = _nonnegative_float_value(payload, "uav_radius_m", 0.0)
        else:
            radius_m = _nonnegative_float_value(payload, "safe_dist_m", 0.0)
        margin_m = _nonnegative_float_value(payload, "safety_margin_m", 0.0)
        inflate_radius_m = radius_m + margin_m
        inflated_grid = inflate_obstacles_3d(
            raw_grid,
            radius_m=inflate_radius_m,
            resolution_m=resolution_m,
            resolution_z_m=resolution_z_m,
        ).astype(np.uint8)
    else:
        inflate_radius_m = 0.0
        inflated_grid = raw_grid

    metadata = {
        "source": "direct_grid_3d",
        "axis_order": "zyx",
        "raw_blocked_voxels": int(np.count_nonzero(raw_grid)),
        "inflated_radius_m": float(inflate_radius_m),
        "goal_clipped": False,
        "_raw_grid3d": raw_grid,
    }
    return inflated_grid, start, goal, frame, previous, metadata


def _payload_requests_3d(payload: dict[str, Any]) -> bool:
    mode = str(payload.get("planning_mode", payload.get("mode", ""))).lower()
    if mode in {"3d", "uav_3d", "voxel", "voxel3d", "kinodynamic", "kino", "3d-kino", "uav-kino"}:
        return True
    if any(key in payload for key in ("grid3d", "occupancy_3d", "grid_shape_3d", "shape_zyx", "resolution_z_m", "z_min_m", "z_max_m", "max_z_cells")):
        return True
    if "grid" in payload:
        try:
            return np.asarray(payload["grid"]).ndim == 3
        except Exception:
            return False
    return False


def problem3d_from_request(payload: dict[str, Any]) -> IsingGrid3DProblem:
    has_grid3d = "grid3d" in payload or "occupancy_3d" in payload
    if not has_grid3d and "grid" in payload:
        try:
            has_grid3d = np.asarray(payload["grid"]).ndim == 3
        except (TypeError, ValueError) as exc:
            raise ValueError("三维grid必须是规则的数字数组") from exc
    if has_grid3d:
        grid, start, goal, frame, previous, metadata = _grid3d_from_payload(payload)
        grid[start] = 0
        grid[goal] = 0
        raw_grid = metadata.pop("_raw_grid3d", None)
        if raw_grid is not None:
            raw_grid = np.asarray(raw_grid, dtype=np.uint8).copy()
            raw_grid[start] = 0
            raw_grid[goal] = 0
        return IsingGrid3DProblem(frame, grid, start, goal, previous, metadata, raw_grid)

    start_xyz = _point3(payload.get("start_xyz"), "start_xyz")
    goal_xyz = _point3(payload.get("goal_xyz"), "goal_xyz")
    points_validated_numpy = bool(payload.get("_points_validated_numpy", False))
    points_xyz = _points3_numpy(
        payload.get("points_xyz", payload.get("cloud_points_xyz", [])),
        "points_xyz",
        assume_finite=points_validated_numpy,
    )
    resolution_m = _positive_float_value(payload, "resolution_m", 0.2)
    resolution_z_m = _positive_float_value(payload, "resolution_z_m", resolution_m)
    z_min_m = _float_value(payload, "z_min_m", 0.0) if "z_min_m" in payload else None
    z_max_m = _float_value(payload, "z_max_m", 0.0) if "z_max_m" in payload else None
    frame, effective_goal_xyz, clipped = make_grid_frame3d(
        start_xyz,
        goal_xyz,
        frame_id=str(payload.get("frame_id", "camera_init")),
        resolution_m=resolution_m,
        resolution_z_m=resolution_z_m,
        window_margin_m=_float_value(payload, "window_margin_m", 2.0),
        window_radius_m=_float_value(payload, "window_radius_m", 8.0),
        z_margin_m=_float_value(payload, "z_margin_m", 1.0),
        z_min_m=z_min_m,
        z_max_m=z_max_m,
        max_axis_cells=_int_value(payload, "max_axis_cells", 160),
        max_z_cells=_int_value(payload, "max_z_cells", 32),
    )
    if "uav_radius_m" in payload:
        radius_m = _nonnegative_float_value(payload, "uav_radius_m", 0.45)
    else:
        radius_m = _nonnegative_float_value(payload, "safe_dist_m", 0.45)
    margin_m = _nonnegative_float_value(payload, "safety_margin_m", 0.0)
    inflate_radius_m = radius_m + margin_m

    native_pair = None
    if _native_fastpath is not None and bool(payload.get("native_cpp_accel", True)):
        try:
            native_pair = _native_fastpath.voxelize_inflate3d(
                points_xyz, frame, inflate_radius_m
            )
        except Exception:
            native_pair = None
    if native_pair is not None:
        raw_grid, grid = native_pair
        native_cpp_preprocess = True
    else:
        raw_grid = pointcloud_to_grid3d(
            points_xyz, frame, assume_finite=points_validated_numpy
        )
        grid = inflate_obstacles_3d(
            raw_grid,
            radius_m=inflate_radius_m,
            resolution_m=resolution_m,
            resolution_z_m=resolution_z_m,
        ).astype(np.uint8)
        native_cpp_preprocess = False
    start = xyz_to_cell3d(start_xyz[0], start_xyz[1], start_xyz[2], frame)
    goal = xyz_to_cell3d(effective_goal_xyz[0], effective_goal_xyz[1], effective_goal_xyz[2], frame)
    if not inside_grid3d(start, grid.shape):
        raise ValueError(f"start_xyz映射后超出grid3d: {start}")
    if not inside_grid3d(goal, grid.shape):
        raise ValueError(f"goal_xyz映射后超出grid3d: {goal}")
    grid[start] = 0
    grid[goal] = 0
    raw_grid = raw_grid.copy()
    raw_grid[start] = 0
    raw_grid[goal] = 0
    previous = path_xyz_to_cells3d(payload.get("previous_path_xyz", []), frame)
    metadata = {
        "source": "dyn_pointcloud_3d",
        "axis_order": "zyx",
        "input_points": len(points_xyz),
        "raw_blocked_voxels": int(np.count_nonzero(raw_grid)),
        "inflated_radius_m": float(inflate_radius_m),
        "uav_radius_m": radius_m,
        "safety_margin_m": margin_m,
        "goal_xyz": list(goal_xyz),
        "effective_goal_xyz": list(effective_goal_xyz),
        "goal_clipped": bool(clipped),
        "native_cpp_preprocess": bool(native_cpp_preprocess),
    }
    return IsingGrid3DProblem(frame, grid, start, goal, previous, metadata, raw_grid)


def problem_from_request(payload: dict[str, Any]) -> IsingGridProblem | IsingGrid3DProblem:
    if not isinstance(payload, dict):
        raise ValueError("请求必须是JSON对象")
    if _payload_requests_3d(payload):
        return problem3d_from_request(payload)
    if "grid" in payload or "occupancy" in payload:
        grid, start, goal, frame, previous, metadata = _grid_from_payload(payload)
        grid[start] = 0
        grid[goal] = 0
        return IsingGridProblem(frame, grid, start, goal, previous, metadata)

    start_xyz = _point3(payload.get("start_xyz"), "start_xyz")
    goal_xyz = _point3(payload.get("goal_xyz"), "goal_xyz")
    points_xyz = _points3(
        payload.get("points_xyz", payload.get("cloud_points_xyz", [])),
        "points_xyz",
    )
    resolution_m = _positive_float_value(payload, "resolution_m", 0.2)
    height_center_m = _float_value(payload, "height_center_m", start_xyz[2])
    if "height_min_m" in payload or "height_max_m" in payload:
        height_min_m = _float_value(payload, "height_min_m", height_center_m - 0.4)
        height_max_m = _float_value(payload, "height_max_m", height_center_m + 0.4)
    else:
        half_width = _nonnegative_float_value(payload, "height_half_width_m", 0.4)
        height_min_m = height_center_m - half_width
        height_max_m = height_center_m + half_width
    frame, effective_goal_xyz, clipped = make_grid_frame(
        start_xyz,
        goal_xyz,
        frame_id=str(payload.get("frame_id", "camera_init")),
        resolution_m=resolution_m,
        window_margin_m=_float_value(payload, "window_margin_m", 2.0),
        window_radius_m=_float_value(payload, "window_radius_m", 8.0),
        max_axis_cells=_int_value(payload, "max_axis_cells", 160),
        height_center_m=height_center_m,
    )
    raw_grid = pointcloud_to_grid(
        points_xyz,
        frame,
        height_min_m=height_min_m,
        height_max_m=height_max_m,
    )
    if "uav_radius_m" in payload:
        radius_m = _nonnegative_float_value(payload, "uav_radius_m", 0.45)
    else:
        radius_m = _nonnegative_float_value(payload, "safe_dist_m", 0.45)
    margin_m = _nonnegative_float_value(payload, "safety_margin_m", 0.0)
    inflate_cells = int(math.ceil((radius_m + margin_m) / resolution_m))
    grid = inflate_obstacles(raw_grid, inflate_cells).astype(np.uint8)
    start = xyz_to_cell(start_xyz[0], start_xyz[1], frame)
    goal = xyz_to_cell(effective_goal_xyz[0], effective_goal_xyz[1], frame)
    if not inside_grid(start, grid.shape):
        raise ValueError(f"start_xyz映射后超出grid: {start}")
    if not inside_grid(goal, grid.shape):
        raise ValueError(f"goal_xyz映射后超出grid: {goal}")
    grid[start] = 0
    grid[goal] = 0
    previous = path_xyz_to_cells(payload.get("previous_path_xyz", []), frame)
    metadata = {
        "source": "dyn_pointcloud",
        "input_points": len(points_xyz),
        "raw_blocked_cells": int(np.count_nonzero(raw_grid)),
        "inflated_cells": inflate_cells,
        "uav_radius_m": radius_m,
        "safety_margin_m": margin_m,
        "height_min_m": height_min_m,
        "height_max_m": height_max_m,
        "goal_xyz": list(goal_xyz),
        "effective_goal_xyz": list(effective_goal_xyz),
        "goal_clipped": bool(clipped),
    }
    return IsingGridProblem(frame, grid, start, goal, previous, metadata)


def planner_kwargs_from_request(payload: dict[str, Any]) -> dict[str, object]:
    return {
        "block_size": _bounded_int_value(payload, "block_size", 3, minimum=3, maximum=12),
        "iterations": _bounded_int_value(payload, "iterations", 480, minimum=1, maximum=65535),
        "restarts": _bounded_int_value(payload, "restarts", 3, minimum=1, maximum=32),
        "encoding": str(payload.get("encoding", "ultra")),
        "planning_horizon": _bounded_int_value(payload, "planning_horizon", 24, minimum=4),
        "planning_margin": _bounded_int_value(payload, "planning_margin", 4, minimum=0),
        "max_planning_margin": _bounded_int_value(payload, "max_planning_margin", 12, minimum=4),
        "macro_block_size": _bounded_int_value(payload, "macro_block_size", 12, minimum=2),
        "top_route_limit": _bounded_int_value(payload, "top_route_limit", 96, minimum=2, maximum=256),
    }


def planner3d_kwargs_from_request(payload: dict[str, Any]) -> dict[str, object]:
    connectivity = _bounded_int_value(payload, "connectivity", 26, minimum=6, maximum=26)
    if "candidate_limit" in payload:
        candidate_limit = _bounded_int_value(payload, "candidate_limit", 8, minimum=2, maximum=256)
    else:
        candidate_limit = _bounded_int_value(payload, "top_route_limit", 8, minimum=2, maximum=256)
    return {
        "iterations": _bounded_int_value(payload, "iterations", 480, minimum=1, maximum=65535),
        "restarts": _bounded_int_value(payload, "restarts", 4, minimum=1, maximum=32),
        "candidate_limit": candidate_limit,
        "connectivity": 6 if connectivity <= 6 else 26,
        "z_step_weight": _positive_float_value(payload, "z_step_weight", 1.0),
        "vertical_penalty": _nonnegative_float_value(payload, "vertical_penalty", 0.12),
        "turn_penalty": _nonnegative_float_value(payload, "turn_penalty", 0.04),
        "altitude_change_penalty": _nonnegative_float_value(payload, "altitude_change_penalty", 0.05),
        "seed": payload.get("seed"),
    }


def kino_kwargs_from_request(payload: dict[str, Any]) -> dict[str, object]:
    """Kinodynamic waypoint QUBO parameters (aligned with the original A* node
    kinematics: max_vel/max_acc from search/*, time weight w_time)."""
    return {
        "start_vel_xyz": payload.get("start_vel_xyz", [0.0, 0.0, 0.0]),
        "start_acc_xyz": payload.get("start_acc_xyz", [0.0, 0.0, 0.0]),
        "goal_vel_xyz": payload.get("goal_vel_xyz", [0.0, 0.0, 0.0]),
        "waypoint_count": _bounded_int_value(payload, "waypoint_count", 32, minimum=2, maximum=254),
        "candidates_per_layer": _bounded_int_value(payload, "candidates_per_layer", 8, minimum=2, maximum=256),
        "dt": max(0.0, _float_value(payload, "dt_s", 0.0)),
        "dt_auto": bool(payload.get("dt_auto", True)),
        "cruise_speed_mps": _positive_float_value(payload, "cruise_speed_mps", 1.5),
        "dt_sample": _positive_float_value(payload, "dt_sample_s", 0.01),
        "max_vel": _positive_float_value(payload, "max_vel_mps", 2.0),
        "max_acc": _positive_float_value(payload, "max_acc_mps2", 2.0),
        "publish_speed_ratio": min(1.0, max(0.75, _positive_float_value(
            payload, "publish_speed_ratio", 0.96
        ))),
        "qubo_dt_quantum_s": min(0.10, _nonnegative_float_value(
            payload, "qubo_dt_quantum_s", 0.02
        )),
        "strict_spine_latency_fuse_ms": _nonnegative_float_value(
            payload, "strict_spine_latency_fuse_ms", 3.5
        ),
        "candidate_cpp_accel": bool(payload.get("candidate_cpp_accel", True)),
        "incremental_qubo": bool(payload.get("incremental_qubo", True)),
        "incremental_qubo_cache_size": _bounded_int_value(
            payload, "incremental_qubo_cache_size", 32, minimum=1, maximum=128
        ),
        "w_vel": _nonnegative_float_value(payload, "w_vel", 1.0),
        "w_acc": _nonnegative_float_value(payload, "w_acc", 2.0),
        "w_spine": _nonnegative_float_value(payload, "w_spine", 0.50),
        "w_len": _nonnegative_float_value(payload, "w_len", 0.30),
        "w_obs": _nonnegative_float_value(payload, "w_obs", 40.0),
        "w_start_vel": _nonnegative_float_value(payload, "w_start_vel", 1.0),
        "w_start_acc": _nonnegative_float_value(payload, "w_start_acc", 1.0),
        "w_end_vel": _nonnegative_float_value(payload, "w_end_vel", 1.0),
        "segment_penalty": _nonnegative_float_value(payload, "segment_penalty", 1000.0),
        "a_mult": _positive_float_value(payload, "a_mult", 3.0),
        "candidate_radius": _bounded_int_value(payload, "candidate_radius", 2, minimum=1, maximum=8),
        "safe_clearance_m": _nonnegative_float_value(payload, "safe_clearance_m", 0.3),
        "iterations": _bounded_int_value(payload, "iterations", 2000, minimum=1, maximum=65535),
        "restarts": _bounded_int_value(payload, "restarts", 8, minimum=1, maximum=32),
        "adaptive_problem_size": bool(payload.get("adaptive_problem_size", True)),
        "adaptive_solver_profile": bool(payload.get("adaptive_solver_profile", True)),
        "quality_gap_ratio": _nonnegative_float_value(payload, "quality_gap_ratio", 0.08),
        "quality_guard": bool(payload.get("quality_guard", True)),
        "previous_path_xyz": _points3(payload.get("previous_path_xyz", []), "previous_path_xyz"),
        "force_spine_refresh": bool(payload.get("force_spine_refresh", False)),
        "shortcut_reused_spine": bool(payload.get("shortcut_reused_spine", True)),
        "near_goal_direct_m": _nonnegative_float_value(payload, "near_goal_direct_m", 3.0),
        "near_goal_tiny_m": _nonnegative_float_value(payload, "near_goal_tiny_m", 4.0),
        "near_goal_direct_cross_speed_mps": _nonnegative_float_value(
            payload, "near_goal_direct_cross_speed_mps", 0.5
        ),
        "near_goal_direct_endpoint_grace_m": _nonnegative_float_value(
            payload, "near_goal_direct_endpoint_grace_m", 0.4
        ),
        "near_hysteresis_distance_m": _nonnegative_float_value(
            payload, "near_hysteresis_distance_m", 4.5
        ),
        "near_hysteresis_detour_ratio": max(1.0, _float_value(
            payload, "near_hysteresis_detour_ratio", 1.06
        )),
        "near_hysteresis_improvement_ratio": min(0.25, _nonnegative_float_value(
            payload, "near_hysteresis_improvement_ratio", 0.02
        )),
        "seed": payload.get("seed"),
    }


def result_response(
    *,
    request_id: object,
    problem: IsingGridProblem,
    path: Sequence[Cell],
    status: str,
    ok: bool,
    metrics: dict[str, object] | None = None,
    message: str = "",
) -> dict[str, object]:
    path_cells = [list(cell) for cell in path]
    return {
        "request_id": request_id,
        "ok": bool(ok),
        "status": str(status),
        "message": str(message),
        "frame": problem.frame.to_dict(),
        "problem": problem.to_debug_dict(),
        "path_cells": path_cells,
        "path_xyz": cells_to_xyz_path(path, problem.frame),
        "metrics": dict(metrics or {}),
    }


def result3d_response(
    *,
    request_id: object,
    problem: IsingGrid3DProblem,
    path: Sequence[Cell3D],
    status: str,
    ok: bool,
    metrics: dict[str, object] | None = None,
    message: str = "",
) -> dict[str, object]:
    path_cells_3d = [list(cell) for cell in path]
    path_cells_2d = [[int(cell[1]), int(cell[2])] for cell in path]
    return {
        "request_id": request_id,
        "ok": bool(ok),
        "status": str(status),
        "message": str(message),
        "frame": problem.frame.to_dict(),
        "frame_3d": problem.frame.to_dict(),
        "problem": problem.to_debug_dict(),
        "path_cells": path_cells_2d,
        "path_cells_3d": path_cells_3d,
        "path_zyx": path_cells_3d,
        "path_xyz": cells3d_to_xyz_path(path, problem.frame),
        "metrics": dict(metrics or {}),
    }


def kino_response(
    *,
    request_id: object,
    problem: IsingGrid3DProblem,
    result: Any,
    status: str,
    ok: bool,
    metrics: dict[str, object] | None = None,
    message: str = "",
) -> dict[str, object]:
    """Kinodynamic response.  Outputs align with the original dyn_obs_planner
    A* node conventions:

      path_xyz           -> nav_msgs/Path poses (dense 0.01 s samples)
      path_size          -> std_msgs/Int64 data (len(path_xyz) + 1)
      waypoints_xyz      -> sparse waypoint list
      velocities_xyz / accelerations_xyz -> per-dense-sample derivatives
    """
    waypoints_zyx = [
        list(xyz_to_cell3d(float(w[0]), float(w[1]), float(w[2]), problem.frame))
        for w in result.waypoints_xyz
    ]
    waypoints_2d = [[int(w[1]), int(w[2])] for w in waypoints_zyx]
    return {
        "request_id": request_id,
        "ok": bool(ok),
        "status": str(status),
        "message": str(message),
        "frame": problem.frame.to_dict(),
        "frame_3d": problem.frame.to_dict(),
        "problem": problem.to_debug_dict(),
        "path_cells": waypoints_2d,
        "path_cells_3d": waypoints_zyx,
        "path_zyx": waypoints_zyx,
        "path_xyz": [list(p) for p in result.path_xyz],
        "path_size": int(result.path_size),
        "waypoints_xyz": [list(p) for p in result.waypoints_xyz],
        "waypoint_velocities_xyz": [list(v) for v in result.waypoint_velocities_xyz],
        "waypoint_accelerations_xyz": [list(a) for a in result.waypoint_accelerations_xyz],
        "velocities_xyz": [list(v) for v in result.velocities_xyz],
        "accelerations_xyz": [list(a) for a in result.accelerations_xyz],
        "metrics": dict(metrics or {}),
    }
