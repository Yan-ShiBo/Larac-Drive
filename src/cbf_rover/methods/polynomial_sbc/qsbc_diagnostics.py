"""Diagnostic-only sampled LP probes for q-SBC expressivity."""

from __future__ import annotations

import json
from math import cos, pi, sin
from pathlib import Path
from typing import Any, Iterable, Mapping, Sequence

import numpy as np
from scipy.optimize import linprog

from cbf_rover.adapters.training.sac_reference import file_sha256
from cbf_rover.domain.planar import CANONICAL_SCENARIO, CANONICAL_SYSTEM

from .algebra import exact_expected_next_polynomial
from .formal_regions import PHYSICAL_X_BREAKS, PHYSICAL_Y_BREAKS
from .polynomial import (
    FrozenController,
    NumericPolynomial,
    Power2D,
    SparsePolynomial2D,
    canonical_total_degree_powers,
    content_sha256,
    controller_pieces,
    load_frozen_controller,
)


QSBC_LP_DIAGNOSTIC_ID = "DX2_SAMPLED_QSBC_EXPRESSIVITY_LP_V1"


def _write_json(path: Path, payload: Mapping[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.tmp")
    temporary.write_text(
        json.dumps(
            payload,
            indent=2,
            sort_keys=True,
            ensure_ascii=False,
            allow_nan=False,
        )
        + "\n",
        encoding="utf-8",
    )
    temporary.replace(path)


def _design_matrix(
    points: np.ndarray,
    powers: Sequence[Power2D],
) -> np.ndarray:
    values = np.asarray(points, dtype=np.float64)
    if values.ndim != 2 or values.shape[1] != 2:
        raise ValueError("points must have shape (n,2)")
    return np.column_stack(
        [
            values[:, 0] ** x_power * values[:, 1] ** y_power
            for x_power, y_power in powers
        ]
    )


def _evaluate_numeric_many(
    polynomial: NumericPolynomial,
    points: np.ndarray,
) -> np.ndarray:
    result = np.zeros(points.shape[0], dtype=np.float64)
    for (x_power, y_power), coefficient in polynomial.items():
        result += (
            coefficient
            * points[:, 0] ** x_power
            * points[:, 1] ** y_power
        )
    return result


def _normalized(points_m: np.ndarray) -> np.ndarray:
    normalization = CANONICAL_SCENARIO.normalization
    return np.column_stack(
        (
            (points_m[:, 0] - normalization.center_x_m)
            / normalization.scale_x_m,
            (points_m[:, 1] - normalization.center_y_m)
            / normalization.scale_y_m,
        )
    )


def _closed_box_points(
    x_low: float,
    x_high: float,
    y_low: float,
    y_high: float,
    *,
    x_count: int,
    y_count: int,
) -> np.ndarray:
    x_values = np.linspace(x_low, x_high, x_count, dtype=np.float64)
    y_values = np.linspace(y_low, y_high, y_count, dtype=np.float64)
    grid_x, grid_y = np.meshgrid(x_values, y_values, indexing="ij")
    return np.column_stack((grid_x.ravel(), grid_y.ravel()))


def _global_grid(
    *,
    x_count: int,
    y_count: int,
    controller: FrozenController,
) -> np.ndarray:
    scenario = CANONICAL_SCENARIO
    x_values = np.unique(
        np.concatenate(
            (
                np.linspace(
                    scenario.domain.x_min,
                    scenario.domain.x_max,
                    x_count,
                    dtype=np.float64,
                ),
                np.asarray(PHYSICAL_X_BREAKS, dtype=np.float64),
                np.asarray(
                    [
                        piece.x_min_m
                        for piece in controller_pieces(controller)
                    ]
                    + [
                        piece.x_max_m
                        for piece in controller_pieces(controller)
                    ]
                    + [
                        scenario.initial.center_x
                        - scenario.initial.radius,
                        scenario.initial.center_x,
                        scenario.initial.center_x
                        + scenario.initial.radius,
                        scenario.goal.center_x - scenario.goal.radius,
                        scenario.goal.center_x,
                        scenario.goal.center_x + scenario.goal.radius,
                    ],
                    dtype=np.float64,
                ),
            )
        )
    )
    y_values = np.unique(
        np.concatenate(
            (
                np.linspace(
                    scenario.domain.y_min,
                    scenario.domain.y_max,
                    y_count,
                    dtype=np.float64,
                ),
                np.asarray(PHYSICAL_Y_BREAKS, dtype=np.float64),
                np.asarray(
                    [
                        scenario.initial.center_y
                        - scenario.initial.radius,
                        scenario.initial.center_y,
                        scenario.initial.center_y
                        + scenario.initial.radius,
                        scenario.goal.center_y - scenario.goal.radius,
                        scenario.goal.center_y,
                        scenario.goal.center_y + scenario.goal.radius,
                    ],
                    dtype=np.float64,
                ),
            )
        )
    )
    grid_x, grid_y = np.meshgrid(x_values, y_values, indexing="ij")
    return np.column_stack((grid_x.ravel(), grid_y.ravel()))


def _initial_disk_points(angle_count: int = 96) -> np.ndarray:
    initial = CANONICAL_SCENARIO.initial
    rows = [(initial.center_x, initial.center_y)]
    for radial_fraction in (0.25, 0.5, 0.75, 1.0):
        radius = radial_fraction * initial.radius
        rows.extend(
            (
                initial.center_x + radius * cos(2.0 * pi * index / angle_count),
                initial.center_y + radius * sin(2.0 * pi * index / angle_count),
            )
            for index in range(angle_count)
        )
    return np.asarray(rows, dtype=np.float64)


def _unsafe_points(points_per_axis: int = 31) -> np.ndarray:
    return np.vstack(
        [
            _closed_box_points(
                unsafe.x_min,
                unsafe.x_max,
                unsafe.y_min,
                unsafe.y_max,
                x_count=points_per_axis,
                y_count=points_per_axis,
            )
            for unsafe in CANONICAL_SCENARIO.unsafe_components
        ]
    )


def _nonterminal_mask(points_m: np.ndarray) -> np.ndarray:
    scenario = CANONICAL_SCENARIO
    goal_distance_squared = (
        (points_m[:, 0] - scenario.goal.center_x) ** 2
        + (points_m[:, 1] - scenario.goal.center_y) ** 2
    )
    goal = goal_distance_squared <= scenario.goal.radius**2
    unsafe = np.zeros(points_m.shape[0], dtype=bool)
    for region in scenario.unsafe_components:
        unsafe |= (
            (region.x_min <= points_m[:, 0])
            & (points_m[:, 0] <= region.x_max)
            & (region.y_min <= points_m[:, 1])
            & (points_m[:, 1] <= region.y_max)
        )
    return ~(goal | unsafe)


def _piece_masks(
    points_m: np.ndarray,
    controller: FrozenController,
) -> list[tuple[Any, np.ndarray]]:
    unassigned = np.ones(points_m.shape[0], dtype=bool)
    result: list[tuple[Any, np.ndarray]] = []
    for piece in controller_pieces(controller):
        mask = (
            unassigned
            & (piece.x_min_m <= points_m[:, 0])
            & (points_m[:, 0] <= piece.x_max_m)
        )
        result.append((piece, mask))
        unassigned &= ~mask
    if bool(np.any(unassigned)):
        raise ValueError("diagnostic grid lies outside controller partition")
    return result


def _expected_design_matrix(
    points_m: np.ndarray,
    normalized_points: np.ndarray,
    controller: FrozenController,
    powers: Sequence[Power2D],
) -> np.ndarray:
    result = np.empty(
        (normalized_points.shape[0], len(powers)),
        dtype=np.float64,
    )
    for piece, mask in _piece_masks(points_m, controller):
        active_points = normalized_points[mask]
        for index, power in enumerate(powers):
            expected = exact_expected_next_polynomial(
                SparsePolynomial2D((power,), (1.0,)),
                controller,
                CANONICAL_SYSTEM,
                piece=piece,
            )
            result[mask, index] = _evaluate_numeric_many(
                expected,
                active_points,
            )
    return result


def _sampled_lp(
    *,
    degree: int,
    epsbar: float,
    controller: FrozenController,
    full_points_m: np.ndarray,
    initial_points_m: np.ndarray,
    unsafe_points_m: np.ndarray,
    drift_points_m: np.ndarray,
    time_limit_s: float,
) -> dict[str, Any]:
    powers = canonical_total_degree_powers(degree)
    full = _design_matrix(_normalized(full_points_m), powers)
    initial = _design_matrix(_normalized(initial_points_m), powers)
    unsafe = _design_matrix(_normalized(unsafe_points_m), powers)
    drift_normalized = _normalized(drift_points_m)
    drift_now = _design_matrix(drift_normalized, powers)
    drift_next = _expected_design_matrix(
        drift_points_m,
        drift_normalized,
        controller,
        powers,
    )
    zero_q = np.zeros((full.shape[0], 1), dtype=np.float64)
    constraints = [
        np.hstack((-full, zero_q)),
        np.hstack(
            (
                initial,
                -np.ones((initial.shape[0], 1), dtype=np.float64),
            )
        ),
        np.hstack(
            (
                -unsafe,
                np.zeros((unsafe.shape[0], 1), dtype=np.float64),
            )
        ),
        np.hstack(
            (
                -(drift_now - drift_next),
                np.zeros((drift_now.shape[0], 1), dtype=np.float64),
            )
        ),
    ]
    right_hand_side = np.concatenate(
        (
            np.zeros(full.shape[0], dtype=np.float64),
            np.zeros(initial.shape[0], dtype=np.float64),
            -np.ones(unsafe.shape[0], dtype=np.float64),
            -float(epsbar) * np.ones(drift_now.shape[0], dtype=np.float64),
        )
    )
    objective = np.zeros(len(powers) + 1, dtype=np.float64)
    objective[-1] = 1.0
    result = linprog(
        objective,
        A_ub=np.vstack(constraints),
        b_ub=right_hand_side,
        bounds=[(-1000.0, 1000.0)] * len(powers) + [(0.0, 1.0)],
        method="highs",
        options={
            "presolve": True,
            "time_limit": float(time_limit_s),
        },
    )
    payload: dict[str, Any] = {
        "degree": degree,
        "epsbar": float(epsbar),
        "status_code": int(result.status),
        "status": "sample_feasible" if result.success else "not_sample_feasible",
        "solver_message": str(result.message),
        "solver": "SciPy_HiGHS_LP",
        "formal_claim_allowed": False,
    }
    if result.success:
        coefficients = np.asarray(result.x[:-1], dtype=np.float64)
        q_value = float(result.x[-1])
        sampled_margins = {
            "W_on_X": float(np.min(full @ coefficients)),
            "q_minus_W_on_X0": float(
                np.min(q_value - initial @ coefficients)
            ),
            "W_minus_1_on_Xu": float(
                np.min(unsafe @ coefficients - 1.0)
            ),
            "drift_minus_epsbar_on_Psi": float(
                np.min(
                    (drift_now - drift_next) @ coefficients
                    - float(epsbar)
                )
            ),
        }
        barrier = SparsePolynomial2D(powers, tuple(coefficients))
        payload.update(
            {
                "q": q_value,
                "coefficient_max_abs": float(
                    np.max(np.abs(coefficients))
                ),
                "sampled_margins": sampled_margins,
                "diagnostic_barrier": barrier.payload(),
            }
        )
    return payload


def diagnose_qsbc_expressivity(
    *,
    frozen_controller_path: str | Path,
    output_dir: str | Path,
    degrees: Iterable[int] = (4, 6, 8, 10, 12),
    epsbar_values: Iterable[float] = (0.0, 1.0e-6),
    grid_x_count: int = 101,
    grid_y_count: int = 81,
    time_limit_s: float = 300.0,
) -> Path:
    """Run sampled LPs without publishing any certificate candidate."""

    if grid_x_count < 11 or grid_y_count < 11:
        raise ValueError("diagnostic grid counts must each be at least 11")
    degree_schedule = tuple(int(value) for value in degrees)
    if not degree_schedule or any(value < 1 for value in degree_schedule):
        raise ValueError("diagnostic degrees must be positive")
    epsbar_schedule = tuple(float(value) for value in epsbar_values)
    if not epsbar_schedule or any(value < 0.0 for value in epsbar_schedule):
        raise ValueError("diagnostic epsbar values must be nonnegative")

    controller_path = Path(frozen_controller_path).resolve()
    controller = load_frozen_controller(controller_path)
    destination = Path(output_dir).resolve()
    destination.mkdir(parents=True, exist_ok=False)
    full_points_m = _global_grid(
        x_count=grid_x_count,
        y_count=grid_y_count,
        controller=controller,
    )
    drift_points_m = full_points_m[_nonterminal_mask(full_points_m)]
    initial_points_m = _initial_disk_points()
    unsafe_points_m = _unsafe_points()
    attempts: list[dict[str, Any]] = []
    for degree in degree_schedule:
        for epsbar in epsbar_schedule:
            attempt = _sampled_lp(
                degree=degree,
                epsbar=epsbar,
                controller=controller,
                full_points_m=full_points_m,
                initial_points_m=initial_points_m,
                unsafe_points_m=unsafe_points_m,
                drift_points_m=drift_points_m,
                time_limit_s=time_limit_s,
            )
            attempt_path = destination / (
                f"degree_{degree}_epsbar_{float(epsbar).hex()}.json"
            )
            _write_json(attempt_path, attempt)
            attempts.append(
                {
                    "degree": degree,
                    "epsbar": epsbar,
                    "status": attempt["status"],
                    "q": attempt.get("q"),
                    "file": attempt_path.name,
                    "file_sha256": file_sha256(attempt_path),
                }
            )
    report = {
        "schema_version": 1,
        "kind": "sampled_qsbc_expressivity_diagnostic",
        "decision_id": QSBC_LP_DIAGNOSTIC_ID,
        "status": "diagnosed",
        "claim_level": "diagnostic_only",
        "controller": {
            "file": controller_path.name,
            "file_sha256": file_sha256(controller_path),
            "content_sha256": controller.sha256,
        },
        "grid": {
            "requested_x_count": grid_x_count,
            "requested_y_count": grid_y_count,
            "full_point_count": int(full_points_m.shape[0]),
            "initial_point_count": int(initial_points_m.shape[0]),
            "unsafe_point_count": int(unsafe_points_m.shape[0]),
            "drift_point_count": int(drift_points_m.shape[0]),
            "includes_formal_breakpoints": True,
        },
        "degree_schedule": list(degree_schedule),
        "epsbar_schedule": list(epsbar_schedule),
        "attempts": attempts,
        "interpretation_boundary": {
            "sampled_constraints_are_not_formal": True,
            "diagnostic_barriers_are_not_artifacts": True,
            "does_not_change_controller": True,
            "does_not_establish_qsbc": True,
            "does_not_authorize_runtime_use": True,
        },
    }
    report["report_content_sha256"] = content_sha256(report)
    report_path = destination / "diagnostic_report.json"
    _write_json(report_path, report)
    return report_path
