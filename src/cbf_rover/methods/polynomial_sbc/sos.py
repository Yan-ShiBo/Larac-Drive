"""Convex SOS/SDP synthesis for a frozen polynomial controller.

Only W coefficients, q, epsbar, and SOS Gram matrices are variables.  The
controller enters as immutable numeric data, preserving C1 convexity.
"""

from __future__ import annotations

from dataclasses import dataclass
import json
from math import comb
from pathlib import Path
import re
import sys
from typing import Any, Iterable, Mapping, Sequence

import numpy as np

from cbf_rover.adapters.training.sac_reference import file_sha256
from cbf_rover.domain.planar import CANONICAL_SCENARIO, CANONICAL_SYSTEM

from .algebra import (
    degree as numeric_degree,
    exact_expected_next_polynomial,
)
from .certificate import QSBCertificate
from .certificate_validation import validate_qsbc_values
from .formal_regions import (
    goal_exterior_guard,
    normalized_box,
    safe_drift_cells,
)
from .polynomial import (
    FrozenController,
    NumericPolynomial,
    PolynomialControllerPiece,
    Power2D,
    SparsePolynomial2D,
    canonical_total_degree_powers,
    content_sha256,
    controller_max_degree,
    controller_piece_for_x,
    controller_pieces,
    controller_switch_x_breaks,
    load_frozen_controller,
)


SymbolicPolynomial = dict[Power2D, Any]
W_DEGREE_SCHEDULE = (4, 6)


@dataclass(frozen=True, slots=True)
class SOSSynthesisOptions:
    time_limit_s: float = 600.0
    threads: int = 8
    coefficient_bound: float = 1000.0
    epsbar_min: float = 1.0e-6
    epsbar_max: float = 1.0
    q_lexicographic_tolerance: float = 1.0e-7
    solver_relative_gap: float = 1.0e-8
    localize_putinar_coordinates: bool = True
    normalize_guard_coefficients: bool = True


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


def _is_number(value: Any) -> bool:
    return isinstance(value, (int, float, np.integer, np.floating))


def _expr_add(left: Any, right: Any) -> Any:
    if _is_number(left) and _is_number(right):
        return float(left) + float(right)
    from mosek.fusion import Expr

    return Expr.add(left, right)


def _expr_scale(value: Any, factor: float) -> Any:
    if _is_number(value):
        return float(value) * float(factor)
    from mosek.fusion import Expr

    return Expr.mul(float(factor), value)


def _expr_sum(values: Iterable[Any]) -> Any:
    result: Any = 0.0
    for value in values:
        result = _expr_add(result, value)
    return result


def _symbolic_add(*polynomials: Mapping[Power2D, Any]) -> SymbolicPolynomial:
    result: SymbolicPolynomial = {}
    for polynomial in polynomials:
        for power, coefficient in polynomial.items():
            if power in result:
                result[power] = _expr_add(result[power], coefficient)
            else:
                result[power] = coefficient
    return result


def _symbolic_scale(
    polynomial: Mapping[Power2D, Any],
    factor: float,
) -> SymbolicPolynomial:
    return {
        power: _expr_scale(coefficient, factor)
        for power, coefficient in polynomial.items()
    }


def _symbolic_subtract(
    left: Mapping[Power2D, Any],
    right: Mapping[Power2D, Any],
) -> SymbolicPolynomial:
    return _symbolic_add(left, _symbolic_scale(right, -1.0))


def _numeric_scaled_by_expression(
    polynomial: Mapping[Power2D, float],
    expression: Any,
) -> SymbolicPolynomial:
    return {
        power: _expr_scale(expression, float(coefficient))
        for power, coefficient in polynomial.items()
        if coefficient != 0.0
    }


def _symbolic_times_numeric(
    symbolic: Mapping[Power2D, Any],
    numeric: Mapping[Power2D, float],
) -> SymbolicPolynomial:
    result: SymbolicPolynomial = {}
    for (sx, sy), symbolic_coefficient in symbolic.items():
        for (nx, ny), numeric_coefficient in numeric.items():
            power = (sx + nx, sy + ny)
            term = _expr_scale(symbolic_coefficient, numeric_coefficient)
            if power in result:
                result[power] = _expr_add(result[power], term)
            else:
                result[power] = term
    return result


def _symbolic_degree(polynomial: Mapping[Power2D, Any]) -> int:
    return max((sum(power) for power in polynomial), default=0)


def _symbolic_affine_pullback(
    polynomial: Mapping[Power2D, Any],
    *,
    x_offset: float,
    x_scale: float,
    y_offset: float,
    y_scale: float,
) -> SymbolicPolynomial:
    """Substitute ``xi = offset + scale*z`` into a symbolic polynomial."""

    if x_scale <= 0.0 or y_scale <= 0.0:
        raise ValueError("affine pullback scales must be strictly positive")
    result: SymbolicPolynomial = {}
    for (x_power, y_power), coefficient in polynomial.items():
        for local_x_power in range(x_power + 1):
            x_factor = (
                comb(x_power, local_x_power)
                * x_offset ** (x_power - local_x_power)
                * x_scale**local_x_power
            )
            for local_y_power in range(y_power + 1):
                factor = (
                    x_factor
                    * comb(y_power, local_y_power)
                    * y_offset ** (y_power - local_y_power)
                    * y_scale**local_y_power
                )
                if factor == 0.0:
                    continue
                power = (local_x_power, local_y_power)
                term = _expr_scale(coefficient, factor)
                if power in result:
                    result[power] = _expr_add(result[power], term)
                else:
                    result[power] = term
    return result


def _numeric_affine_pullback(
    polynomial: Mapping[Power2D, float],
    *,
    x_offset: float,
    x_scale: float,
    y_offset: float,
    y_scale: float,
) -> NumericPolynomial:
    transformed = _symbolic_affine_pullback(
        polynomial,
        x_offset=x_offset,
        x_scale=x_scale,
        y_offset=y_offset,
        y_scale=y_scale,
    )
    return {
        power: float(coefficient)
        for power, coefficient in transformed.items()
        if float(coefficient) != 0.0
    }


def _box_affine_map(
    box: tuple[float, float, float, float],
) -> tuple[float, float, float, float]:
    x_low, x_high, y_low, y_high = box
    if x_low >= x_high or y_low >= y_high:
        raise ValueError("local-coordinate box must have positive width")
    return (
        0.5 * (x_low + x_high),
        0.5 * (x_high - x_low),
        0.5 * (y_low + y_high),
        0.5 * (y_high - y_low),
    )


def _unit_rectangle_guards() -> tuple[NumericPolynomial, NumericPolynomial]:
    return (
        {(0, 0): 1.0, (2, 0): -1.0},
        {(0, 0): 1.0, (0, 2): -1.0},
    )


def _unit_disk_guard() -> NumericPolynomial:
    return {
        (0, 0): 1.0,
        (2, 0): -1.0,
        (0, 2): -1.0,
    }


def _normalize_guard(polynomial: NumericPolynomial) -> NumericPolynomial:
    maximum = max((abs(value) for value in polynomial.values()), default=0.0)
    if maximum <= 0.0:
        raise ValueError("a semialgebraic guard must be nonzero")
    return {
        power: coefficient / maximum
        for power, coefficient in polynomial.items()
    }


def _gram_polynomial(
    matrix: Any,
    basis: Sequence[Power2D],
) -> SymbolicPolynomial:
    result: SymbolicPolynomial = {}
    for left_index, left_power in enumerate(basis):
        for right_index, right_power in enumerate(basis):
            power = (
                left_power[0] + right_power[0],
                left_power[1] + right_power[1],
            )
            value = matrix.index(left_index, right_index)
            if power in result:
                result[power] = _expr_add(result[power], value)
            else:
                result[power] = value
    return result


def _safe_name(value: str) -> str:
    return re.sub(r"[^A-Za-z0-9_]", "_", value)


def _new_sos(
    model: Any,
    *,
    name: str,
    basis_degree: int,
    grams: list[tuple[str, Any, tuple[Power2D, ...]]],
) -> SymbolicPolynomial:
    from mosek.fusion import Domain

    basis = canonical_total_degree_powers(basis_degree)
    matrix = model.variable(
        f"Q_{_safe_name(name)}",
        Domain.inPSDCone(len(basis)),
    )
    grams.append((name, matrix, basis))
    return _gram_polynomial(matrix, basis)


def _add_putinar_constraint(
    model: Any,
    *,
    name: str,
    polynomial: SymbolicPolynomial,
    guards: Sequence[NumericPolynomial],
    grams: list[tuple[str, Any, tuple[Power2D, ...]]],
) -> None:
    from mosek.fusion import Domain

    target_degree = _symbolic_degree(polynomial)
    residual = _new_sos(
        model,
        name=f"{name}_residual",
        basis_degree=(target_degree + 1) // 2,
        grams=grams,
    )
    representation = dict(residual)
    for index, guard in enumerate(guards):
        guard_degree = numeric_degree(guard)
        remaining_degree = target_degree - guard_degree
        if remaining_degree < 0:
            continue
        multiplier = _new_sos(
            model,
            name=f"{name}_multiplier_{index}",
            basis_degree=remaining_degree // 2,
            grams=grams,
        )
        representation = _symbolic_add(
            representation,
            _symbolic_times_numeric(multiplier, guard),
        )
    support = sorted(
        set(polynomial).union(representation),
        key=lambda power: (sum(power), power[0], power[1]),
    )
    for power in support:
        difference = _expr_add(
            polynomial.get(power, 0.0),
            _expr_scale(representation.get(power, 0.0), -1.0),
        )
        model.constraint(
            f"coef_{_safe_name(name)}_{power[0]}_{power[1]}",
            difference,
            Domain.equalsTo(0.0),
        )


def _build_symbolic_problem(
    model: Any,
    *,
    controller: FrozenController,
    barrier_degree: int,
    options: SOSSynthesisOptions,
) -> tuple[
    Any,
    Any,
    Any,
    tuple[Power2D, ...],
    list[tuple[str, Any, tuple[Power2D, ...]]],
]:
    from mosek.fusion import Domain

    barrier_powers = canonical_total_degree_powers(barrier_degree)
    w_variable = model.variable(
        "W_coefficients",
        len(barrier_powers),
        Domain.inRange(-options.coefficient_bound, options.coefficient_bound),
    )
    q_variable = model.variable("q", 1, Domain.inRange(0.0, 1.0))
    epsbar_variable = model.variable(
        "epsbar",
        1,
        Domain.inRange(options.epsbar_min, options.epsbar_max),
    )
    q_expression = q_variable.index(0)
    epsbar_expression = epsbar_variable.index(0)
    barrier_symbolic = {
        power: w_variable.index(index)
        for index, power in enumerate(barrier_powers)
    }
    grams: list[tuple[str, Any, tuple[Power2D, ...]]] = []

    if not options.localize_putinar_coordinates:
        raise ValueError("G6 requires local Putinar coordinates")
    full_box_guards = _unit_rectangle_guards()
    _add_putinar_constraint(
        model,
        name="W_nonnegative_X",
        polynomial=barrier_symbolic,
        guards=full_box_guards,
        grams=grams,
    )
    q_minus_w = _symbolic_add(
        {(0, 0): q_expression},
        _symbolic_scale(barrier_symbolic, -1.0),
    )
    initial = CANONICAL_SCENARIO.initial
    normalization = CANONICAL_SCENARIO.normalization
    q_minus_w_local = _symbolic_affine_pullback(
        q_minus_w,
        x_offset=(
            initial.center_x - normalization.center_x_m
        )
        / normalization.scale_x_m,
        x_scale=initial.radius / normalization.scale_x_m,
        y_offset=(
            initial.center_y - normalization.center_y_m
        )
        / normalization.scale_y_m,
        y_scale=initial.radius / normalization.scale_y_m,
    )
    _add_putinar_constraint(
        model,
        name="q_minus_W_X0",
        polynomial=q_minus_w_local,
        guards=(_unit_disk_guard(),),
        grams=grams,
    )
    for unsafe in CANONICAL_SCENARIO.unsafe_components:
        unsafe_box = normalized_box(unsafe)
        unsafe_map = _box_affine_map(unsafe_box)
        _add_putinar_constraint(
            model,
            name=f"W_minus_1_{unsafe.name}",
            polynomial=_symbolic_affine_pullback(
                _symbolic_add(
                    barrier_symbolic,
                    {(0, 0): -1.0},
                ),
                x_offset=unsafe_map[0],
                x_scale=unsafe_map[1],
                y_offset=unsafe_map[2],
                y_scale=unsafe_map[3],
            ),
            guards=_unit_rectangle_guards(),
            grams=grams,
        )

    expected_by_piece: dict[str, SymbolicPolynomial] = {}
    for piece in controller_pieces(controller):
        expected_symbolic: SymbolicPolynomial = {}
        for index, power in enumerate(barrier_powers):
            unit_barrier = SparsePolynomial2D((power,), (1.0,))
            expected_basis = exact_expected_next_polynomial(
                unit_barrier,
                controller,
                CANONICAL_SYSTEM,
                piece=piece,
            )
            expected_symbolic = _symbolic_add(
                expected_symbolic,
                _numeric_scaled_by_expression(
                    expected_basis,
                    w_variable.index(index),
                ),
            )
        expected_by_piece[piece.piece_id] = expected_symbolic
    goal_guard = goal_exterior_guard()
    for cell in safe_drift_cells(controller_switch_x_breaks(controller)):
        piece = _controller_piece_for_cell(controller, cell.x_min, cell.x_max)
        drift_global = _symbolic_add(
            barrier_symbolic,
            _symbolic_scale(expected_by_piece[piece.piece_id], -1.0),
            {(0, 0): _expr_scale(epsbar_expression, -1.0)},
        )
        cell_map = _box_affine_map(normalized_box(cell))
        drift_local = _symbolic_affine_pullback(
            drift_global,
            x_offset=cell_map[0],
            x_scale=cell_map[1],
            y_offset=cell_map[2],
            y_scale=cell_map[3],
        )
        local_goal_guard = _numeric_affine_pullback(
            goal_guard,
            x_offset=cell_map[0],
            x_scale=cell_map[1],
            y_offset=cell_map[2],
            y_scale=cell_map[3],
        )
        if options.normalize_guard_coefficients:
            local_goal_guard = _normalize_guard(local_goal_guard)
        cell_guards = (*_unit_rectangle_guards(), local_goal_guard)
        _add_putinar_constraint(
            model,
            name=f"drift_{piece.piece_id}_{cell.name}",
            polynomial=drift_local,
            guards=cell_guards,
            grams=grams,
        )
    return (
        w_variable,
        q_variable,
        epsbar_variable,
        barrier_powers,
        grams,
    )


def _controller_piece_for_cell(
    controller: FrozenController,
    x_min_m: float,
    x_max_m: float,
) -> PolynomialControllerPiece:
    """Select the unique piece covering a drift-cell interior."""

    piece = controller_piece_for_x(
        controller,
        0.5 * (float(x_min_m) + float(x_max_m)),
    )
    if piece.x_min_m > x_min_m or piece.x_max_m < x_max_m:
        raise ValueError("drift cell crosses a controller switch")
    return piece


def _optimal(status: Any) -> bool:
    return str(status).casefold().endswith("optimal")


def _gram_payload(
    grams: Sequence[tuple[str, Any, tuple[Power2D, ...]]],
) -> tuple[dict[str, np.ndarray], list[dict[str, Any]]]:
    arrays: dict[str, np.ndarray] = {}
    metadata: list[dict[str, Any]] = []
    for index, (name, matrix, basis) in enumerate(grams):
        key = f"gram_{index:04d}"
        level = np.asarray(matrix.level(), dtype=np.float64).reshape(
            len(basis),
            len(basis),
        )
        arrays[key] = level
        metadata.append(
            {
                "array_key": key,
                "name": name,
                "basis": [list(power) for power in basis],
                "shape": list(level.shape),
                "minimum_eigenvalue": float(
                    np.min(np.linalg.eigvalsh(0.5 * (level + level.T)))
                ),
            }
        )
    return arrays, metadata


def solve_qsbc_degree(
    *,
    frozen_controller_path: str | Path,
    barrier_degree: int,
    output_dir: str | Path,
    options: SOSSynthesisOptions = SOSSynthesisOptions(),
) -> Path:
    """Solve one degree exactly once, then independently replay final values."""

    if barrier_degree not in W_DEGREE_SCHEDULE:
        raise ValueError(f"barrier degree must be one of {W_DEGREE_SCHEDULE}")
    destination = Path(output_dir).resolve()
    destination.mkdir(parents=True, exist_ok=False)
    controller = load_frozen_controller(frozen_controller_path)
    import mosek
    from mosek.fusion import Domain, Model, ObjectiveSense

    manifest: dict[str, Any] = {
        "schema_version": 1,
        "kind": "g5_sos_synthesis_attempt",
        "synthesis_variant": "G6_LOCAL_COORDINATE_PUTINAR_MOSEK_V1",
        "barrier_degree": barrier_degree,
        "controller_content_sha256": controller.sha256,
        "controller_artifact_kind": controller.payload_without_hash()["kind"],
        "controller_partition": [
            {
                "piece_id": piece.piece_id,
                "physical_x_interval_m": [
                    piece.x_min_m,
                    piece.x_max_m,
                ],
            }
            for piece in controller_pieces(controller)
        ],
        "options": {
            "time_limit_s": options.time_limit_s,
            "threads": options.threads,
            "coefficient_bound": options.coefficient_bound,
            "epsbar_min": options.epsbar_min,
            "epsbar_max": options.epsbar_max,
            "q_lexicographic_tolerance": options.q_lexicographic_tolerance,
            "solver_relative_gap": options.solver_relative_gap,
            "localize_putinar_coordinates": (
                options.localize_putinar_coordinates
            ),
            "normalize_guard_coefficients": (
                options.normalize_guard_coefficients
            ),
        },
        "solver": {
            "name": "MOSEK Fusion",
            "version": list(mosek.Env.getversion()),
        },
        "status": "building",
    }
    manifest_path = destination / "manifest.json"
    _write_json(manifest_path, manifest)
    with Model(f"cbf_rover_qsbc_W{barrier_degree}") as model:
        model.setLogHandler(sys.stdout)
        model.setSolverParam("optimizerMaxTime", float(options.time_limit_s))
        model.setSolverParam("numThreads", int(options.threads))
        model.setSolverParam(
            "intpntCoTolRelGap",
            float(options.solver_relative_gap),
        )
        (
            w_variable,
            q_variable,
            epsbar_variable,
            barrier_powers,
            grams,
        ) = _build_symbolic_problem(
            model,
            controller=controller,
            barrier_degree=barrier_degree,
            options=options,
        )
        q_expression = q_variable.index(0)
        epsbar_expression = epsbar_variable.index(0)
        model.objective(ObjectiveSense.Minimize, q_expression)
        stage1_task = destination / "stage1_min_q.ptf"
        model.writeTask(str(stage1_task))
        try:
            model.solve()
        except Exception as error:
            manifest.update(
                {
                    "status": "solver_error",
                    "phase": "minimize_q",
                    "error_type": type(error).__name__,
                    "error": str(error),
                }
            )
            _write_json(manifest_path, manifest)
            return manifest_path
        stage1_status = model.getPrimalSolutionStatus()
        manifest["stage1_solution_status"] = str(stage1_status)
        if not _optimal(stage1_status):
            manifest["status"] = "infeasible_or_unknown"
            _write_json(manifest_path, manifest)
            return manifest_path
        q_star = float(q_variable.level()[0])
        model.constraint(
            "lexicographic_q_cap",
            q_expression,
            Domain.lessThan(q_star + options.q_lexicographic_tolerance),
        )
        model.objective(ObjectiveSense.Maximize, epsbar_expression)
        stage2_task = destination / "stage2_max_epsbar.ptf"
        model.writeTask(str(stage2_task))
        try:
            model.solve()
        except Exception as error:
            manifest.update(
                {
                    "status": "solver_error",
                    "phase": "maximize_epsbar",
                    "error_type": type(error).__name__,
                    "error": str(error),
                }
            )
            _write_json(manifest_path, manifest)
            return manifest_path
        stage2_status = model.getPrimalSolutionStatus()
        manifest["stage2_solution_status"] = str(stage2_status)
        if not _optimal(stage2_status):
            manifest["status"] = "infeasible_or_unknown"
            _write_json(manifest_path, manifest)
            return manifest_path

        coefficients = tuple(float(value) for value in w_variable.level())
        q_value = float(q_variable.level()[0])
        epsbar_value = float(epsbar_variable.level()[0])
        gram_arrays, gram_metadata = _gram_payload(grams)
        grams_path = destination / "sos_grams.npz"
        np.savez_compressed(grams_path, **gram_arrays)
        barrier = SparsePolynomial2D(barrier_powers, coefficients)
        candidate_payload = {
            "schema_version": 1,
            "kind": "q_sbc_numeric_candidate",
            "controller_content_sha256": controller.sha256,
            "barrier": barrier.payload(),
            "q": q_value,
            "q_float64_hex": q_value.hex(),
            "epsbar": epsbar_value,
            "epsbar_float64_hex": epsbar_value.hex(),
        }
        candidate_payload["content_sha256"] = content_sha256(
            candidate_payload
        )
        candidate_path = destination / "numeric_candidate.json"
        _write_json(candidate_path, candidate_payload)
        manifest.update(
            {
                "status": "solver_optimal_pending_independent_validation",
                "q_stage1": q_star,
                "q": q_value,
                "epsbar": epsbar_value,
                "W_term_count": len(coefficients),
                "drift_polynomial_degree": barrier_degree
                * max(1, controller_max_degree(controller)),
                "gram_count": len(grams),
                "gram_metadata": gram_metadata,
                "gram_artifact": {
                    "file": grams_path.name,
                    "sha256": file_sha256(grams_path),
                },
                "tasks": [
                    {
                        "file": stage1_task.name,
                        "sha256": file_sha256(stage1_task),
                    },
                    {
                        "file": stage2_task.name,
                        "sha256": file_sha256(stage2_task),
                    },
                ],
                "numeric_candidate": {
                    "file": candidate_path.name,
                    "sha256": file_sha256(candidate_path),
                },
            }
        )
        _write_json(manifest_path, manifest)

    validation_path = validate_qsbc_values(
        barrier=barrier,
        q=q_value,
        epsbar=epsbar_value,
        controller=controller,
        output_dir=destination / "independent_validation",
    )
    validation = json.loads(validation_path.read_text(encoding="utf-8"))
    manifest["independent_validation"] = {
        "file": str(validation_path.relative_to(destination)),
        "sha256": file_sha256(validation_path),
        "status": validation["status"],
        "first_slice_ready": validation["first_slice_ready"],
    }
    if validation["status"] == "validated":
        certificate = QSBCertificate(
            certificate_id=(
                f"CBF_ROVER_G4_W{barrier_degree}_"
                f"{controller.sha256[:12]}"
            ),
            barrier=barrier,
            q=q_value,
            epsbar=epsbar_value,
            controller_content_sha256=controller.sha256,
            solver={
                "method": "local_coordinate_Putinar_SOS_SDP",
                "synthesis_variant": manifest["synthesis_variant"],
                "solver": "MOSEK Fusion",
                "solver_version": manifest["solver"]["version"],
                "barrier_degree": barrier_degree,
                "controller_degree": controller_max_degree(controller),
                "controller_artifact_kind": manifest[
                    "controller_artifact_kind"
                ],
                "controller_partition": manifest["controller_partition"],
                "gram_artifact_sha256": manifest["gram_artifact"]["sha256"],
            },
            validation={
                "validator_id": validation["validator_id"],
                "validation_report_sha256": file_sha256(validation_path),
                "status": validation["status"],
                "first_slice_ready": validation["first_slice_ready"],
            },
        )
        certificate_path = destination / "certificate.json"
        certificate.save(certificate_path)
        manifest["certificate"] = {
            "file": certificate_path.name,
            "sha256": file_sha256(certificate_path),
            "content_sha256": certificate.sha256,
        }
        manifest["status"] = (
            "ready"
            if validation["first_slice_ready"]
            else "formal_valid_not_ready"
        )
    else:
        manifest["status"] = (
            "validation_violated"
            if validation["status"] == "violated"
            else "validation_unknown"
        )
    manifest["manifest_content_sha256"] = content_sha256(manifest)
    _write_json(manifest_path, manifest)
    return manifest_path


def solve_qsbc_schedule(
    *,
    frozen_controller_path: str | Path,
    output_dir: str | Path,
    options: SOSSynthesisOptions = SOSSynthesisOptions(),
) -> Path:
    """Run the finite G5 degree schedule; never increase it automatically."""

    destination = Path(output_dir).resolve()
    destination.mkdir(parents=True, exist_ok=False)
    controller = load_frozen_controller(frozen_controller_path)
    attempts: list[dict[str, Any]] = []
    selected: dict[str, Any] | None = None
    for degree in W_DEGREE_SCHEDULE:
        attempt_path = solve_qsbc_degree(
            frozen_controller_path=frozen_controller_path,
            barrier_degree=degree,
            output_dir=destination / f"W_degree_{degree}",
            options=options,
        )
        attempt = json.loads(attempt_path.read_text(encoding="utf-8"))
        attempts.append(
            {
                "degree": degree,
                "manifest": str(attempt_path.relative_to(destination)),
                "manifest_file_sha256": file_sha256(attempt_path),
                "manifest_content_sha256": attempt.get(
                    "manifest_content_sha256"
                ),
                "status": attempt["status"],
                "q": attempt.get("q"),
                "epsbar": attempt.get("epsbar"),
            }
        )
        if attempt["status"] == "ready":
            selected = attempts[-1]
            break
    payload = {
        "schema_version": 1,
        "kind": "qsbc_degree_schedule",
        "synthesis_variant": "G6_LOCAL_COORDINATE_PUTINAR_MOSEK_V1",
        "controller_content_sha256": controller.sha256,
        "controller_artifact_kind": controller.payload_without_hash()["kind"],
        "options": {
            "time_limit_s": options.time_limit_s,
            "threads": options.threads,
            "coefficient_bound": options.coefficient_bound,
            "epsbar_min": options.epsbar_min,
            "epsbar_max": options.epsbar_max,
            "q_lexicographic_tolerance": options.q_lexicographic_tolerance,
            "solver_relative_gap": options.solver_relative_gap,
            "localize_putinar_coordinates": (
                options.localize_putinar_coordinates
            ),
            "normalize_guard_coefficients": (
                options.normalize_guard_coefficients
            ),
        },
        "schedule": list(W_DEGREE_SCHEDULE),
        "status": "ready" if selected is not None else "not_ready",
        "selected": selected,
        "attempts": attempts,
        "automatic_degree_escalation_beyond_schedule": False,
    }
    payload["manifest_content_sha256"] = content_sha256(payload)
    manifest_path = destination / "schedule_manifest.json"
    _write_json(manifest_path, payload)
    return manifest_path
