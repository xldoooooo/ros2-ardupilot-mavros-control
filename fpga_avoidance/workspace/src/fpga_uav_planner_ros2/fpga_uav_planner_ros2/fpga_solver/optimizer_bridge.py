from __future__ import annotations

"""Unified software optimizer bridge for the AC7020C host.

Available software optimizers:
- Kaiwu simulated annealing (SA only)
- Local simulated annealing
- Local tabu search
- Multi-start greedy local search

All results are re-evaluated with the original problem objective before
ranking, so FPGA and software values use the same convention.
"""

from dataclasses import dataclass, asdict
import math
import random
from time import perf_counter
from typing import Any, Dict, List

import numpy as np

from qubo_model import (
    N_HW,
    Problem,
    ising_energy,
    maxcut_value,
    qubo_value,
)


@dataclass
class SoftwareResult:
    model: str
    objective: float
    spins: List[int]
    binary: List[int]
    solver_runtime_s: float
    total_runtime_s: float
    candidate_count: int
    details: Dict[str, Any]

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)


def _objective(problem: Problem, spins: List[int]) -> float:
    padded = list(spins) + [-1] * (N_HW - len(spins))
    if problem.kind == "maxcut":
        return float(maxcut_value(problem, padded))
    if problem.kind == "qubo":
        return float(qubo_value(problem, padded))
    return float(ising_energy(problem, padded, original=True))


def _maximize(problem: Problem) -> bool:
    return problem.kind == "maxcut"


def _better(problem: Problem, lhs: float, rhs: float, eps: float = 1e-12) -> bool:
    if _maximize(problem):
        return lhs > rhs + eps
    return lhs < rhs - eps


def _improvement(problem: Problem, delta: float) -> float:
    return delta if _maximize(problem) else -delta


def _random_spins(rng: random.Random, n: int) -> List[int]:
    return [1 if rng.getrandbits(1) else -1 for _ in range(n)]


def _flip_delta(problem: Problem, spins: List[int], index: int) -> float:
    """Return objective(new)-objective(current) for flipping one variable."""
    n = problem.n

    if problem.kind == "maxcut":
        matrix = problem.source_matrix
        spin_i = spins[index]
        delta = 0.0
        for j in range(n):
            if j == index:
                continue
            weight = float(matrix[index][j])
            delta += weight if spin_i == spins[j] else -weight
        return delta

    if problem.kind == "qubo":
        matrix = problem.source_matrix
        binary = [1 if spin > 0 else 0 for spin in spins]
        x_i = binary[index]
        linear = float(matrix[index][index])
        for j in range(n):
            if j == index:
                continue
            linear += (
                float(matrix[index][j]) + float(matrix[j][index])
            ) * binary[j]
        return float((1 - 2 * x_i) * linear)

    # ising_energy() uses:
    #   E = -sum(h_i s_i) - sum_ij(J_ij s_i s_j)
    # J is symmetric, so each off-diagonal pair is counted twice.
    spin_i = spins[index]
    interaction = 0.0
    for j in range(n):
        if j != index:
            interaction += float(problem.j[index][j]) * spins[j]

    delta_scaled = (
        2.0 * float(problem.h[index]) * spin_i
        + 4.0 * spin_i * interaction
    )
    scale = float(problem.fpga_scale) if problem.fpga_scale else 1.0
    return delta_scaled / scale


def _binary(spins: List[int]) -> List[int]:
    return [1 if spin > 0 else 0 for spin in spins]


def _result(
    model: str,
    problem: Problem,
    best_spins: List[int],
    runtime_s: float,
    candidate_count: int,
    details: Dict[str, Any],
) -> SoftwareResult:
    objective = _objective(problem, best_spins)
    return SoftwareResult(
        model=model,
        objective=float(objective),
        spins=list(best_spins),
        binary=_binary(best_spins),
        solver_runtime_s=float(runtime_s),
        total_runtime_s=float(runtime_s),
        candidate_count=int(candidate_count),
        details=details,
    )


def solve_local_sa(
    problem: Problem,
    iterations: int,
    restarts: int,
    seed: int,
) -> SoftwareResult:
    start_time = perf_counter()
    iterations = max(1, int(iterations))
    restarts = max(1, int(restarts))

    global_best_spins: List[int] | None = None
    global_best_value: float | None = None
    best_restart = 0

    for restart in range(restarts):
        rng = random.Random((int(seed) + 0x9E3779B9 * restart) & 0xFFFFFFFF)
        spins = _random_spins(rng, problem.n)
        current = _objective(problem, spins)
        local_best_spins = list(spins)
        local_best_value = current

        samples = [
            abs(_flip_delta(problem, spins, rng.randrange(problem.n)))
            for _ in range(min(64, max(8, problem.n)))
        ]
        nonzero = [value for value in samples if value > 1e-12]
        scale = float(np.median(nonzero)) if nonzero else 1.0
        temperature_start = max(scale * 4.0, 1e-6)
        temperature_end = max(temperature_start * 1e-3, 1e-9)
        cooling = (
            (temperature_end / temperature_start) ** (1.0 / max(1, iterations - 1))
            if iterations > 1
            else 1.0
        )
        temperature = temperature_start

        for _step in range(iterations):
            index = rng.randrange(problem.n)
            delta = _flip_delta(problem, spins, index)
            gain = _improvement(problem, delta)

            accept = gain >= 0.0
            if not accept:
                probability = math.exp(max(-700.0, gain / max(temperature, 1e-12)))
                accept = rng.random() < probability

            if accept:
                spins[index] = -spins[index]
                current += delta
                # Periodically remove accumulated floating-point drift.
                if (_step & 255) == 255:
                    current = _objective(problem, spins)

                if _better(problem, current, local_best_value):
                    local_best_value = current
                    local_best_spins = list(spins)

            temperature *= cooling

        exact_local = _objective(problem, local_best_spins)
        if global_best_value is None or _better(problem, exact_local, global_best_value):
            global_best_value = exact_local
            global_best_spins = local_best_spins
            best_restart = restart

    assert global_best_spins is not None
    runtime = perf_counter() - start_time
    return _result(
        "local_sa",
        problem,
        global_best_spins,
        runtime,
        restarts,
        {
            "backend": "BuiltInSimulatedAnnealing",
            "iterations": iterations,
            "restarts": restarts,
            "best_restart": best_restart,
            "seed": int(seed) & 0xFFFFFFFF,
        },
    )


def solve_tabu(
    problem: Problem,
    iterations: int,
    restarts: int,
    seed: int,
) -> SoftwareResult:
    start_time = perf_counter()
    iterations = max(1, int(iterations))
    restarts = max(1, int(restarts))
    tenure_base = max(5, int(round(math.sqrt(problem.n))))

    global_best_spins: List[int] | None = None
    global_best_value: float | None = None
    best_restart = 0

    for restart in range(restarts):
        rng = random.Random((int(seed) ^ 0xA5A5A5A5 ^ (restart * 7919)) & 0xFFFFFFFF)
        spins = _random_spins(rng, problem.n)
        current = _objective(problem, spins)
        restart_best_spins = list(spins)
        restart_best_value = current
        tabu_until = [0] * problem.n

        for step in range(iterations):
            selected_index = -1
            selected_value: float | None = None
            selected_delta = 0.0

            for index in range(problem.n):
                delta = _flip_delta(problem, spins, index)
                candidate_value = current + delta

                aspiration = (
                    global_best_value is None
                    or _better(problem, candidate_value, global_best_value)
                )
                if step < tabu_until[index] and not aspiration:
                    continue

                if (
                    selected_value is None
                    or _better(problem, candidate_value, selected_value)
                    or (
                        abs(candidate_value - selected_value) <= 1e-12
                        and rng.random() < 0.5
                    )
                ):
                    selected_index = index
                    selected_value = candidate_value
                    selected_delta = delta

            if selected_index < 0 or selected_value is None:
                break

            spins[selected_index] = -spins[selected_index]
            current += selected_delta
            if (step & 127) == 127:
                current = _objective(problem, spins)

            jitter = rng.randrange(max(1, tenure_base // 2 + 1))
            tabu_until[selected_index] = step + tenure_base + jitter

            if _better(problem, current, restart_best_value):
                restart_best_value = current
                restart_best_spins = list(spins)

            if global_best_value is None or _better(problem, current, global_best_value):
                global_best_value = current
                global_best_spins = list(spins)
                best_restart = restart

        exact_restart = _objective(problem, restart_best_spins)
        if global_best_value is None or _better(problem, exact_restart, global_best_value):
            global_best_value = exact_restart
            global_best_spins = restart_best_spins
            best_restart = restart

    assert global_best_spins is not None
    runtime = perf_counter() - start_time
    return _result(
        "tabu",
        problem,
        global_best_spins,
        runtime,
        restarts,
        {
            "backend": "BuiltInTabuSearch",
            "iterations": iterations,
            "restarts": restarts,
            "tabu_tenure": tenure_base,
            "best_restart": best_restart,
            "seed": int(seed) & 0xFFFFFFFF,
        },
    )


def solve_greedy(
    problem: Problem,
    iterations: int,
    restarts: int,
    seed: int,
) -> SoftwareResult:
    start_time = perf_counter()
    iterations = max(1, int(iterations))
    # Greedy is cheap; use at least four starts while respecting the UI value.
    effective_restarts = max(4, int(restarts))

    global_best_spins: List[int] | None = None
    global_best_value: float | None = None
    best_restart = 0
    total_moves = 0

    for restart in range(effective_restarts):
        rng = random.Random((int(seed) + restart * 104729) & 0xFFFFFFFF)
        spins = _random_spins(rng, problem.n)
        current = _objective(problem, spins)

        for _step in range(iterations):
            selected_index = -1
            selected_delta = 0.0
            selected_gain = 0.0

            for index in range(problem.n):
                delta = _flip_delta(problem, spins, index)
                gain = _improvement(problem, delta)
                if gain > selected_gain + 1e-12:
                    selected_gain = gain
                    selected_delta = delta
                    selected_index = index

            if selected_index < 0:
                break

            spins[selected_index] = -spins[selected_index]
            current += selected_delta
            total_moves += 1

        exact_value = _objective(problem, spins)
        if global_best_value is None or _better(problem, exact_value, global_best_value):
            global_best_value = exact_value
            global_best_spins = list(spins)
            best_restart = restart

    assert global_best_spins is not None
    runtime = perf_counter() - start_time
    return _result(
        "greedy",
        problem,
        global_best_spins,
        runtime,
        effective_restarts,
        {
            "backend": "BuiltInMultiStartGreedy",
            "iterations_limit": iterations,
            "restarts": effective_restarts,
            "accepted_moves": total_moves,
            "best_restart": best_restart,
            "seed": int(seed) & 0xFFFFFFFF,
        },
    )


def _ranking_key(problem: Problem, result: SoftwareResult):
    return -result.objective if _maximize(problem) else result.objective


def solve_with_optimizers(
    problem: Problem,
    selection: str = "all",
    iterations: int = 2000,
    restarts: int = 4,
    seed: int = 0x12802026,
) -> SoftwareResult:
    """Run one or several software optimizers and return the best exact result."""
    selection = str(selection).strip().lower()
    allowed = {"all", "kaiwu_sa", "local_sa", "tabu", "greedy"}
    if selection not in allowed:
        raise ValueError(f"未知软件优化器选择: {selection}")

    requested = (
        ["kaiwu_sa", "local_sa", "tabu", "greedy"]
        if selection == "all"
        else [selection]
    )

    total_start = perf_counter()
    results: List[SoftwareResult] = []
    errors: Dict[str, str] = {}

    for optimizer_name in requested:
        try:
            if optimizer_name == "kaiwu_sa":
                from kaiwu_bridge import solve_with_kaiwu

                kaiwu_result = solve_with_kaiwu(problem, "sa")
                results.append(
                    SoftwareResult(
                        model="kaiwu_sa",
                        objective=float(kaiwu_result.objective),
                        spins=list(kaiwu_result.spins),
                        binary=list(kaiwu_result.binary),
                        solver_runtime_s=float(kaiwu_result.solver_runtime_s),
                        total_runtime_s=float(kaiwu_result.total_runtime_s),
                        candidate_count=int(kaiwu_result.candidate_count),
                        details=dict(kaiwu_result.details),
                    )
                )
            elif optimizer_name == "local_sa":
                results.append(
                    solve_local_sa(problem, iterations, restarts, seed)
                )
            elif optimizer_name == "tabu":
                results.append(
                    solve_tabu(problem, iterations, restarts, seed ^ 0x13579BDF)
                )
            elif optimizer_name == "greedy":
                results.append(
                    solve_greedy(problem, iterations, restarts, seed ^ 0x2468ACE0)
                )
        except Exception as exc:
            errors[optimizer_name] = f"{type(exc).__name__}: {exc}"
            if selection != "all":
                raise

    if not results:
        detail = "；".join(f"{key}={value}" for key, value in errors.items())
        raise RuntimeError("所有软件优化器均失败：" + detail)

    # Re-evaluate every returned solution using the exact host objective.
    for result in results:
        exact = _objective(problem, result.spins)
        if abs(exact - result.objective) > 1e-7:
            result.details["reported_objective_before_recalc"] = result.objective
            result.objective = exact

    results.sort(key=lambda item: _ranking_key(problem, item))
    best = results[0]
    elapsed = perf_counter() - total_start

    ranking = [
        {
            "rank": index + 1,
            "optimizer": result.model,
            "objective": float(result.objective),
            "solver_ms": float(result.solver_runtime_s * 1000.0),
            "total_ms": float(result.total_runtime_s * 1000.0),
            "candidates": int(result.candidate_count),
        }
        for index, result in enumerate(results)
    ]

    merged_details: Dict[str, Any] = {
        "backend": "MultiOptimizerSuite",
        "selected_suite": selection,
        "best_optimizer": best.model,
        "best_solver_runtime_s": float(best.solver_runtime_s),
        "iterations": int(iterations),
        "restarts": int(restarts),
        "seed": int(seed) & 0xFFFFFFFF,
        "ranking": ranking,
        "errors": errors,
        "best_details": best.details,
    }

    value_convention = best.details.get("value_convention")
    if value_convention:
        merged_details["value_convention"] = value_convention

    return SoftwareResult(
        model=best.model,
        objective=float(best.objective),
        spins=list(best.spins),
        binary=list(best.binary),
        # For a suite, report honest total software search time.
        solver_runtime_s=float(elapsed),
        total_runtime_s=float(elapsed),
        candidate_count=sum(item.candidate_count for item in results),
        details=merged_details,
    )
