"""Exact Hermite dynamics and conservative occupied-voxel curve certificates.

All coefficients use local seconds and ascending powers. Occupancy is already
inflated by the caller; this module never clears occupied endpoint cells.
"""

from __future__ import annotations

import math
from typing import Callable, Sequence

import numpy as np

from dyn_ising_adapter import GridFrame3D

# An unresolved hull is rejected at this depth; it is never accepted by sampling.
_COLLISION_MAX_DEPTH = 20
# Bound work for a curve touching an obstacle or leaving the known map window.
_COLLISION_MAX_HULLS = 20000


def hermite_coefficients(points: np.ndarray, velocities: np.ndarray, duration) -> np.ndarray:
    """Construct the actual per-segment, per-axis cubic coefficients."""
    duration = np.asarray(duration, dtype=np.float64)
    if duration.ndim:
        duration = duration[:, None]
    delta = points[1:] - points[:-1]
    result = np.empty((len(points) - 1, 3, 4), dtype=np.float64)
    result[:, :, 0] = points[:-1]
    result[:, :, 1] = velocities[:-1]
    result[:, :, 2] = 3.0 * delta / duration**2 - (2.0 * velocities[:-1] + velocities[1:]) / duration
    result[:, :, 3] = -2.0 * delta / duration**3 + (velocities[:-1] + velocities[1:]) / duration**2
    return result


def motion_durations(points, initial_velocity, cruise, acceleration):
    """Allocate chord times with forward acceleration and backward terminal braking."""
    distance = np.linalg.norm(np.diff(points, axis=0), axis=1)
    speed = np.full(len(points), cruise)
    speed[0] = np.linalg.norm(initial_velocity)
    for i, length in enumerate(distance):
        speed[i + 1] = min(speed[i + 1], math.sqrt(speed[i] ** 2 + 2 * acceleration * length))
    speed[-1] = 0.0
    for i in range(len(points) - 2, 0, -1):
        speed[i] = min(speed[i], math.sqrt(speed[i + 1] ** 2 + 2 * acceleration * distance[i]))
    if len(points) == 2:
        # A single cubic also handles measured braking away from and back to a same-cell goal.
        return np.array([max(0.05, 2 * distance[0] / cruise,
                             math.sqrt(6 * distance[0] / acceleration),
                             4 * speed[0] / acceleration)])
    return 2 * distance / np.maximum(speed[:-1] + speed[1:], np.finfo(float).eps)


def clamped_velocities(points, durations, initial_velocity):
    """Solve interior velocities for C2 cubics, preserving measured start and zero finish."""
    n = len(points)
    diagonal = np.ones(n)
    upper = np.zeros(n - 1)
    lower = np.zeros(n - 1)
    rhs = np.zeros((n, 3))
    rhs[0] = initial_velocity
    delta = np.diff(points, axis=0)
    for i in range(1, n - 1):
        left, right = durations[i - 1:i + 1]
        lower[i - 1], diagonal[i], upper[i] = right, 2 * (left + right), left
        rhs[i] = 3 * (right * delta[i - 1] / left + left * delta[i] / right)
    # Thomas elimination is linear in the waypoint count and solves all three axes together.
    for i in range(1, n):
        factor = lower[i - 1] / diagonal[i - 1]
        diagonal[i] -= factor * upper[i - 1]
        rhs[i] -= factor * rhs[i - 1]
    velocity = rhs / diagonal[:, None]
    for i in range(n - 2, -1, -1):
        velocity[i] -= upper[i] * velocity[i + 1] / diagonal[i]
    return velocity


def reference_durations(points, reference):
    """Project new knots spatially onto the last curve; its wall-clock phase is unknown."""
    if reference is None or not len(reference[0]):
        return None
    positions, times = [], []
    offset = 0.0
    for coefficients, duration in zip(*reference):
        local = np.linspace(0.0, duration, min(201, max(3, math.ceil(duration / 0.05) + 1)))
        p, _, _ = sample_executable_segment(np.asarray(coefficients), local)
        positions.extend(p[:-1])
        times.extend(offset + local[:-1])
        offset += duration
    positions.append(p[-1])
    times.append(offset)
    positions, times = np.asarray(positions), np.asarray(times)
    chord = np.diff(positions, axis=0)
    length2 = np.sum(chord * chord, axis=1)
    projected = []
    for point in points:
        fraction = np.clip(np.sum((point - positions[:-1]) * chord, axis=1) /
                           np.maximum(length2, np.finfo(float).eps), 0.0, 1.0)
        distance2 = np.sum((positions[:-1] + fraction[:, None] * chord - point) ** 2, axis=1)
        index = int(np.argmin(distance2))
        projected.append(times[index] + fraction[index] * (times[index + 1] - times[index]))
    duration = np.diff(projected)
    return duration if np.all(duration > 0.0) else None


def sample_executable_segment(coefficients: np.ndarray, times: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Evaluate one cubic and its exact first two derivatives."""
    t = np.asarray(times, dtype=np.float64)[:, None]
    c = np.asarray(coefficients, dtype=np.float64)
    position = c[:, 0] + t * (c[:, 1] + t * (c[:, 2] + t * c[:, 3]))
    velocity = c[:, 1] + t * (2.0 * c[:, 2] + t * 3.0 * c[:, 3])
    acceleration = 2.0 * c[:, 2] + t * 6.0 * c[:, 3]
    return position, velocity, acceleration


def _maximum_speed(coefficients: np.ndarray, duration: float, axes: Sequence[int]) -> float:
    """Find every stationary squared-speed root, plus both endpoints."""
    c = coefficients[list(axes)]
    v, a, b = c[:, 1], 2.0 * c[:, 2], 3.0 * c[:, 3]
    derivative = np.array((2.0 * np.dot(v, a),
                           2.0 * (np.dot(a, a) + 2.0 * np.dot(v, b)),
                           6.0 * np.dot(a, b), 4.0 * np.dot(b, b)))
    # Trim exact zero high coefficients only: small, valid cubics still matter.
    nonzero = np.flatnonzero(derivative)
    roots = np.polynomial.polynomial.polyroots(derivative[:nonzero[-1] + 1]) if len(nonzero) else ()
    times = [0.0, duration]
    for root in roots:
        if abs(float(root.imag)) <= 1.0e-8 * max(1.0, abs(float(root.real))):
            if 0.0 < float(root.real) < duration:
                times.append(float(root.real))
    return max(float(np.linalg.norm(v + a * t + b * t * t)) for t in times)


def executable_segment_limits(coefficients: Sequence, durations: Sequence[float]) -> dict[str, float]:
    """Return analytic total/XY/Z speed and acceleration maxima for all cubics."""
    maxima = {name: 0.0 for name in ("speed", "acceleration", "speed_xy", "speed_z", "acceleration_xy", "acceleration_z")}
    for c, duration in zip(np.asarray(coefficients, dtype=np.float64), durations):
        for suffix, axes in (("", (0, 1, 2)), ("_xy", (0, 1)), ("_z", (2,))):
            maxima["speed" + suffix] = max(maxima["speed" + suffix], _maximum_speed(c, float(duration), axes))
            # Acceleration is affine; a convex norm attains its maximum at an endpoint.
            selected = c[list(axes)]
            acc = max(float(np.linalg.norm(2.0 * selected[:, 2] + 6.0 * selected[:, 3] * t)) for t in (0.0, float(duration)))
            maxima["acceleration" + suffix] = max(maxima["acceleration" + suffix], acc)
    return maxima


def executable_segments_clear(
    coefficients: Sequence,
    durations: Sequence[float],
    occupancy: np.ndarray,
    frame: GridFrame3D,
    *,
    cancel_callback: Callable[[], bool] | None = None,
) -> bool:
    """Certify complete cubics with subdivided Bezier convex hulls.
    Reject occupied or unresolved hulls instead of accepting sampled gaps.
    """
    curves = np.asarray(coefficients, dtype=np.float64)
    times = np.asarray(durations, dtype=np.float64)
    grid = np.asarray(occupancy)
    if curves.ndim != 3 or curves.shape[1:] != (3, 4) or len(curves) != len(times) or not len(curves):
        return False
    if not np.isfinite(curves).all() or not np.isfinite(times).all() or np.any(times <= 0.0):
        return False
    if grid.shape != (frame.layers, frame.rows, frame.cols):
        return False
    origin = np.array((frame.x_min, frame.y_min, frame.z_min), dtype=np.float64)
    resolution = np.array((frame.resolution_m, frame.resolution_m, frame.resolution_z_m), dtype=np.float64)
    shape = np.array((frame.cols, frame.rows, frame.layers))
    if not np.isfinite(origin).all() or not np.isfinite(resolution).all() or np.any(resolution <= 0.0):
        return False
    upper = origin + resolution * shape
    hull_count = 0
    for c, duration in zip(curves, times):
        endpoints, velocities, _ = sample_executable_segment(c, np.array((0.0, duration)))
        if np.any(endpoints < origin) or np.any(endpoints >= upper):
            return False
        control = np.array((endpoints[0], endpoints[0] + velocities[0] * duration / 3.0,
                            endpoints[1] - velocities[1] * duration / 3.0, endpoints[1]))
        stack = [(control, 0)]
        while stack:
            if cancel_callback is not None and cancel_callback():
                raise RuntimeError("executable trajectory planning cancelled")
            hull, depth = stack.pop()
            hull_count += 1
            if hull_count > _COLLISION_MAX_HULLS:
                return False
            lo, hi = hull.min(axis=0), hull.max(axis=0)
            inside = bool(np.all(lo >= origin) and np.all(hi < upper))
            # Include both touching cells, with an outward numerical guard.
            low_cell = np.maximum(0, np.floor((lo - origin) / resolution - 1.0e-10).astype(int))
            high_cell = np.minimum(shape - 1, np.floor((hi - origin) / resolution + 1.0e-10).astype(int))
            if inside:
                occupied = grid[low_cell[2]:high_cell[2] + 1,
                                low_cell[1]:high_cell[1] + 1,
                                low_cell[0]:high_cell[0] + 1]
                if not np.any(occupied):
                    continue
            if depth >= _COLLISION_MAX_DEPTH:
                return False
            a = (hull[0] + hull[1]) * 0.5
            b = (hull[1] + hull[2]) * 0.5
            d = (hull[2] + hull[3]) * 0.5
            e, f = (a + b) * 0.5, (b + d) * 0.5
            midpoint = (e + f) * 0.5
            stack.append((np.array((midpoint, f, d, hull[3])), depth + 1))
            stack.append((np.array((hull[0], a, e, midpoint)), depth + 1))
    return True
