"""G8 q-SBC synthesis with Bernstein-positive scalar S-procedure terms.

G7 covered the curved goal exterior with rectangular supersets.  G8 instead
keeps the exact semialgebraic implication: on a goal-boundary box it proves

``drift - lambda * goal_exterior_guard >= 0, lambda >= 0``.

The final W/q/epsbar values are still replayed by the independent validator,
which reads none of the multiplier values or solver status.
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any, Mapping, Sequence

import numpy as np
from scipy.optimize import linprog

from cbf_rover.adapters.training.sac_reference import file_sha256
from cbf_rover.domain.planar import CANONICAL_SCENARIO

from .algebra import constant, negate
from .bernstein_lp import (
    Box,
    _barrier_basis_polynomials,
    _drift_basis_polynomials,
    _goal_box_class,
    _piece_for_box,
    _polynomial_family_bernstein_matrix,
    _split_box_both_axes,
    _uniform_boxes,
)
from .certificate import QSBCertificate
from .certificate_validation import validate_qsbc_values
from .formal_regions import (
    goal_exterior_guard,
    initial_disk_guard,
    normalized_box,
    safe_drift_cells,
)
from .polynomial import (
    FrozenController,
    NumericPolynomial,
    PolynomialControllerPiece,
    SparsePolynomial2D,
    canonical_total_degree_powers,
    content_sha256,
    controller_max_degree,
    controller_pieces,
    controller_switch_x_breaks,
    load_frozen_controller,
)


G8_SYNTHESIS_ID = "G8_BERNSTEIN_SCALAR_MULTIPLIER_LP_W14_V1"
G8_BARRIER_DEGREE = 14


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


def _save_manifest(path: Path, payload: Mapping[str, Any]) -> None:
    material = dict(payload)
    material.pop("manifest_content_sha256", None)
    material["manifest_content_sha256"] = content_sha256(material)
    _write_json(path, material)


def _uniform_refinement(box: Box, depth: int) -> tuple[Box, ...]:
    if depth < 0:
        raise ValueError("refinement depth must be nonnegative")
    if depth == 0:
        return (box,)
    return tuple(
        refined
        for child in _split_box_both_axes(box)
        for refined in _uniform_refinement(child, depth - 1)
    )


def refine_goal_boundary_box(
    box: Box,
    *,
    depth: int,
    maximum_depth: int,
) -> tuple[tuple[Box, str], ...]:
    """Split only goal-boundary boxes and retain their exact classification."""

    if depth < 0 or maximum_depth < depth:
        raise ValueError("invalid goal refinement depth")
    classification = _goal_box_class(box)
    if classification != "boundary" or depth == maximum_depth:
        return ((box, classification),)
    return tuple(
        refined
        for child in _split_box_both_axes(box)
        for refined in refine_goal_boundary_box(
            child,
            depth=depth + 1,
            maximum_depth=maximum_depth,
        )
    )


def _drift_partition(
    controller: FrozenController,
    *,
    base_refinement_depth: int,
    goal_boundary_refinement_depth: int,
) -> tuple[tuple[PolynomialControllerPiece, Box, str], ...]:
    result: list[tuple[PolynomialControllerPiece, Box, str]] = []
    for cell in safe_drift_cells(controller_switch_x_breaks(controller)):
        for base_box in _uniform_refinement(
            normalized_box(cell),
            base_refinement_depth,
        ):
            for box, goal_class in refine_goal_boundary_box(
                base_box,
                depth=0,
                maximum_depth=goal_boundary_refinement_depth,
            ):
                if goal_class == "inside":
                    continue
                result.append(
                    (_piece_for_box(controller, box), box, goal_class)
                )
    return tuple(result)


def _embedded_family(
    *,
    polynomials: Sequence[NumericPolynomial],
    variable_indices: Sequence[int],
    box: Box,
    variable_count: int,
) -> np.ndarray:
    if len(polynomials) != len(variable_indices):
        raise ValueError("polynomial/variable index count mismatch")
    local = _polynomial_family_bernstein_matrix(polynomials, box)
    result = np.zeros(
        (local.shape[0], variable_count),
        dtype=np.float64,
    )
    for local_column, global_column in enumerate(variable_indices):
        result[:, global_column] += local[:, local_column]
    return result


def synthesize_multiplier_bernstein_qsbc(
    *,
    frozen_controller_path: str | Path,
    output_dir: str | Path,
    barrier_degree: int = G8_BARRIER_DEGREE,
    epsbar: float = 1.0e-6,
    synthesis_margin: float = 2.0e-7,
    time_limit_s: float = 900.0,
    full_box_parts_per_axis: int = 4,
    base_refinement_depth: int = 1,
    goal_boundary_refinement_depth: int = 2,
    multiplier_upper_bound: float = 100_000.0,
) -> Path:
    """Solve G8 and independently replay the resulting scalar values."""

    if barrier_degree != G8_BARRIER_DEGREE:
        raise ValueError(f"G8 freezes barrier_degree={G8_BARRIER_DEGREE}")
    if epsbar <= 0.0 or synthesis_margin <= 0.0:
        raise ValueError("epsbar and synthesis margin must be positive")
    if time_limit_s <= 0.0 or multiplier_upper_bound <= 0.0:
        raise ValueError("time and multiplier bounds must be positive")

    controller_path = Path(frozen_controller_path).resolve()
    controller = load_frozen_controller(controller_path)
    destination = Path(output_dir).resolve()
    destination.mkdir(parents=True, exist_ok=False)
    manifest_path = destination / "manifest.json"
    manifest: dict[str, Any] = {
        "schema_version": 1,
        "kind": "g8_bernstein_scalar_multiplier_synthesis",
        "synthesis_id": G8_SYNTHESIS_ID,
        "status": "building",
        "barrier_degree": barrier_degree,
        "epsbar": epsbar,
        "synthesis_margin": synthesis_margin,
        "controller_content_sha256": controller.sha256,
        "controller_file_sha256": file_sha256(controller_path),
        "options": {
            "time_limit_s": time_limit_s,
            "coefficient_bound": 1000.0,
            "multiplier_upper_bound": multiplier_upper_bound,
            "full_box_parts_per_axis": full_box_parts_per_axis,
            "base_refinement_depth": base_refinement_depth,
            "goal_boundary_refinement_depth": (
                goal_boundary_refinement_depth
            ),
            "initial_disk_scalar_s_procedure": True,
            "goal_exterior_scalar_s_procedure": True,
            "rectangular_goal_superset_constraints": False,
        },
        "solver": {
            "name": "SciPy HiGHS LP",
            "method": "highs",
            "primal_feasibility_tolerance": 1.0e-9,
            "dual_feasibility_tolerance": 1.0e-9,
        },
        "independent_validator_reads_multipliers": False,
    }
    _save_manifest(manifest_path, manifest)

    powers = canonical_total_degree_powers(barrier_degree)
    barrier_basis = _barrier_basis_polynomials(powers)
    w_indices = tuple(range(len(powers)))
    q_index = len(powers)
    drift_partition = _drift_partition(
        controller,
        base_refinement_depth=base_refinement_depth,
        goal_boundary_refinement_depth=goal_boundary_refinement_depth,
    )
    boundary_rows = tuple(
        index
        for index, (_, _, goal_class) in enumerate(drift_partition)
        if goal_class == "boundary"
    )
    initial_lambda_index = q_index + 1
    boundary_lambda_indices = {
        row_index: initial_lambda_index + 1 + offset
        for offset, row_index in enumerate(boundary_rows)
    }
    variable_count = (
        len(powers) + 2 + len(boundary_lambda_indices)
    )

    constraint_matrices: list[np.ndarray] = []
    constraint_bounds: list[np.ndarray] = []
    constraint_counts: dict[str, int] = {}

    def add_positive_family(
        name: str,
        *,
        polynomials: Sequence[NumericPolynomial],
        variable_indices: Sequence[int],
        box: Box,
        constant_value: float,
        required: float,
    ) -> None:
        positive_matrix = _embedded_family(
            polynomials=polynomials,
            variable_indices=variable_indices,
            box=box,
            variable_count=variable_count,
        )
        constraint_matrices.append(-positive_matrix)
        constraint_bounds.append(
            np.full(
                positive_matrix.shape[0],
                float(constant_value) - float(required),
                dtype=np.float64,
            )
        )
        constraint_counts[name] = (
            constraint_counts.get(name, 0)
            + positive_matrix.shape[0]
        )

    for box in _uniform_boxes(full_box_parts_per_axis):
        add_positive_family(
            "W_nonnegative_X",
            polynomials=barrier_basis,
            variable_indices=w_indices,
            box=box,
            constant_value=0.0,
            required=synthesis_margin,
        )

    initial = CANONICAL_SCENARIO.initial
    normalization = CANONICAL_SCENARIO.normalization
    initial_box = (
        (
            initial.center_x
            - initial.radius
            - normalization.center_x_m
        )
        / normalization.scale_x_m,
        (
            initial.center_x
            + initial.radius
            - normalization.center_x_m
        )
        / normalization.scale_x_m,
        (
            initial.center_y
            - initial.radius
            - normalization.center_y_m
        )
        / normalization.scale_y_m,
        (
            initial.center_y
            + initial.radius
            - normalization.center_y_m
        )
        / normalization.scale_y_m,
    )
    add_positive_family(
        "q_minus_W_on_X0_scalar_S_procedure",
        polynomials=(
            *(negate(polynomial) for polynomial in barrier_basis),
            constant(1.0),
            negate(initial_disk_guard()),
        ),
        variable_indices=(
            *w_indices,
            q_index,
            initial_lambda_index,
        ),
        box=initial_box,
        constant_value=0.0,
        required=synthesis_margin,
    )

    for unsafe in CANONICAL_SCENARIO.unsafe_components:
        add_positive_family(
            f"W_minus_1_{unsafe.name}",
            polynomials=barrier_basis,
            variable_indices=w_indices,
            box=normalized_box(unsafe),
            constant_value=-1.0,
            required=synthesis_margin,
        )

    drift_basis_by_piece = {
        piece.piece_id: _drift_basis_polynomials(
            controller=controller,
            piece=piece,
            powers=powers,
        )
        for piece in controller_pieces(controller)
    }
    goal_guard = goal_exterior_guard()
    drift_class_counts = {
        "outside": 0,
        "boundary": 0,
        "inside_skipped": 0,
    }
    for row_index, (piece, box, goal_class) in enumerate(drift_partition):
        drift_class_counts[goal_class] += 1
        drift_basis = drift_basis_by_piece[piece.piece_id]
        if goal_class == "boundary":
            add_positive_family(
                f"drift_{piece.piece_id}_goal_scalar_S_procedure",
                polynomials=(*drift_basis, negate(goal_guard)),
                variable_indices=(
                    *w_indices,
                    boundary_lambda_indices[row_index],
                ),
                box=box,
                constant_value=-epsbar,
                required=synthesis_margin,
            )
        else:
            add_positive_family(
                f"drift_{piece.piece_id}_outside_goal",
                polynomials=drift_basis,
                variable_indices=w_indices,
                box=box,
                constant_value=-epsbar,
                required=synthesis_margin,
            )

    matrix = np.vstack(constraint_matrices)
    right_hand_side = np.concatenate(constraint_bounds)
    objective = np.zeros(variable_count, dtype=np.float64)
    objective[q_index] = 1.0
    variable_bounds = (
        [(-1000.0, 1000.0)] * len(powers)
        + [(0.0, 1.0)]
        + [(0.0, multiplier_upper_bound)]
        * (1 + len(boundary_lambda_indices))
    )
    manifest["partition"] = {
        "full_X_box_count": full_box_parts_per_axis**2,
        "unsafe_box_count": len(CANONICAL_SCENARIO.unsafe_components),
        "active_drift_box_count": len(drift_partition),
        "drift_goal_class_counts": drift_class_counts,
        "scalar_multiplier_count": 1 + len(boundary_lambda_indices),
        "constraint_counts": constraint_counts,
        "total_linear_inequalities": int(matrix.shape[0]),
        "decision_variable_count": int(matrix.shape[1]),
    }
    manifest["status"] = "solving"
    _save_manifest(manifest_path, manifest)

    result = linprog(
        objective,
        A_ub=matrix,
        b_ub=right_hand_side,
        bounds=variable_bounds,
        method="highs",
        options={
            "presolve": True,
            "time_limit": float(time_limit_s),
            "primal_feasibility_tolerance": 1.0e-9,
            "dual_feasibility_tolerance": 1.0e-9,
        },
    )
    manifest["solver_result"] = {
        "status_code": int(result.status),
        "success": bool(result.success),
        "message": str(result.message),
        "iterations": int(result.nit),
    }
    if not result.success:
        manifest["status"] = "infeasible_or_unknown"
        _save_manifest(manifest_path, manifest)
        return manifest_path

    coefficients = tuple(float(value) for value in result.x[: len(powers)])
    q_value = float(result.x[q_index])
    multiplier_values = tuple(
        float(value) for value in result.x[initial_lambda_index:]
    )
    minimum_lp_slack = float(
        np.min(right_hand_side - matrix @ result.x)
    )
    barrier = SparsePolynomial2D(powers, coefficients)
    candidate = {
        "schema_version": 1,
        "kind": "g8_qsbc_numeric_candidate",
        "synthesis_id": G8_SYNTHESIS_ID,
        "controller_content_sha256": controller.sha256,
        "barrier": barrier.payload(),
        "q": q_value,
        "q_float64_hex": q_value.hex(),
        "epsbar": epsbar,
        "epsbar_float64_hex": epsbar.hex(),
        "minimum_lp_slack": minimum_lp_slack,
        "multiplier_summary": {
            "count": len(multiplier_values),
            "minimum": min(multiplier_values),
            "maximum": max(multiplier_values),
            "upper_bound": multiplier_upper_bound,
        },
        "formal_claim_allowed_before_independent_validation": False,
    }
    candidate["content_sha256"] = content_sha256(candidate)
    candidate_path = destination / "numeric_candidate.json"
    _write_json(candidate_path, candidate)
    manifest.update(
        {
            "status": "optimal_pending_independent_validation",
            "q": q_value,
            "minimum_lp_slack": minimum_lp_slack,
            "coefficient_max_abs": max(abs(value) for value in coefficients),
            "multiplier_minimum": min(multiplier_values),
            "multiplier_maximum": max(multiplier_values),
            "numeric_candidate": {
                "file": candidate_path.name,
                "file_sha256": file_sha256(candidate_path),
                "content_sha256": candidate["content_sha256"],
            },
        }
    )
    _save_manifest(manifest_path, manifest)

    validation_path = validate_qsbc_values(
        barrier=barrier,
        q=q_value,
        epsbar=epsbar,
        controller=controller,
        output_dir=destination / "independent_validation",
    )
    validation = json.loads(validation_path.read_text(encoding="utf-8"))
    manifest["independent_validation"] = {
        "file": str(validation_path.relative_to(destination)),
        "file_sha256": file_sha256(validation_path),
        "content_sha256": validation["report_content_sha256"],
        "status": validation["status"],
        "first_slice_ready": validation["first_slice_ready"],
    }
    if validation["status"] != "validated":
        manifest["status"] = (
            "validation_violated"
            if validation["status"] == "violated"
            else "validation_unknown"
        )
        _save_manifest(manifest_path, manifest)
        return manifest_path

    certificate = QSBCertificate(
        certificate_id=f"CBF_ROVER_G8_W14_{controller.sha256[:12]}",
        barrier=barrier,
        q=q_value,
        epsbar=epsbar,
        controller_content_sha256=controller.sha256,
        solver={
            "method": "Bernstein_scalar_multiplier_LP",
            "synthesis_id": G8_SYNTHESIS_ID,
            "solver": "SciPy HiGHS",
            "barrier_degree": barrier_degree,
            "controller_degree": controller_max_degree(controller),
            "minimum_lp_slack": minimum_lp_slack,
            "constraint_count": int(matrix.shape[0]),
            "scalar_multiplier_count": len(multiplier_values),
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
        "file_sha256": file_sha256(certificate_path),
        "content_sha256": certificate.sha256,
    }
    manifest["status"] = (
        "ready"
        if validation["first_slice_ready"]
        else "formal_valid_not_ready"
    )
    _save_manifest(manifest_path, manifest)
    return manifest_path
