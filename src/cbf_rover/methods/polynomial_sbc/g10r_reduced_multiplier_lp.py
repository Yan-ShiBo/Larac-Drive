"""G10R redundancy-reduced Chebyshev q-SBC synthesis.

G10R is an independent route beside G10.  It keeps the same frozen model,
controller, barrier space, finite schedule, scalar bounds, and V3 release gate.
The only mathematical change in the synthesis encoding is the exact
elimination of the two rectangle-guard multipliers attached to each box.

Those multipliers are redundant because the residual is already certified on
that same box.  In local coordinates ``s,t in [0,1]``, both
``4*s*(1-s)`` and ``4*t*(1-t)`` have nonnegative Bernstein coefficients.
Products with a nonnegative tensor-Bernstein multiplier, sums, and degree
elevation remain in the nonnegative Bernstein cone.  Hence setting both box
multipliers to zero preserves the projection of the LP feasible set onto the
barrier, common margin, and curved-set multiplier variables.

The initial-disk and goal-exterior multipliers are not eliminated: their
guards change sign on their enclosing boxes and therefore genuinely encode
the curved subsets.  HiGHS remains candidate generation only; V3 independently
reconstructs and proves the original G4 obligations.
"""

from __future__ import annotations

from fractions import Fraction
import gc
import json
from math import comb
from pathlib import Path
from typing import Any, Sequence

import numpy as np
from scipy.optimize import linprog

from cbf_rover.adapters.training.sac_reference import file_sha256
from cbf_rover.domain.planar import CANONICAL_SCENARIO

from .algebra import constant, negate
from .bernstein_lp import Box, _uniform_boxes
from .bernstein_multiplier_lp import _drift_partition
from .certificate import QSBCertificate
from .chebyshev import (
    chebyshev_coefficients_to_monomial,
    total_degree_chebyshev_basis,
)
from .formal_regions import (
    goal_exterior_guard,
    initial_disk_guard,
    normalized_box,
)
from .g10_sparse_multiplier_lp import (
    G10_BASE_REFINEMENT_DEPTH,
    G10_COEFFICIENT_BOUND,
    G10_CONTROLLER_CONTENT_SHA256,
    G10_CONTROLLER_FILE_SHA256,
    G10_EPSBAR,
    G10_FULL_BOX_PARTS_PER_AXIS,
    G10_GOAL_BOUNDARY_REFINEMENT_DEPTH,
    G10_MULTIPLIER_BOUND,
    G10_NUMERIC_FEASIBILITY_TOLERANCE,
    G10_Q,
    G10_RHO_BOUNDS,
    G10_STAGE_SCHEDULE,
    G10_VALIDATOR_ID,
    G10StageSpec,
    _G10Problem,
    _SparseLPBuilder,
    _append_multiplier_terms,
    _assert_frozen_c2,
    _candidate_vector,
    _chebyshev_drift_basis,
    _initial_box,
    _local_multiplier_material,
    _maximum_bound_violation,
    _save_manifest,
    _v3_release_gate,
    _write_json,
)
from .polynomial import (
    FrozenController,
    NumericPolynomial,
    content_sha256,
    controller_max_degree,
    controller_pieces,
    load_frozen_controller,
)


G10R_SYNTHESIS_ID = "G10R_CHEBYSHEV_SHAPE_MULTIPLIER_SPARSE_LP_V1"
G10R_REDUNDANCY_PROOF_ID = "G10R_SAME_BOX_GUARD_PROJECTION_EQUIVALENCE_V1"
G10R_FROZEN_ACTIVE_DRIFT_BOX_COUNT = 202
G10R_FROZEN_GOAL_BOUNDARY_BOX_COUNT = 28
G10R_FROZEN_OBLIGATION_COUNT = 222
G10R_RETAINED_SHAPE_MULTIPLIER_COUNT = 29
G10R_REMOVED_BOX_GUARD_MULTIPLIER_COUNT = 444


def _box_guard_product_weight(multiplier_degree: int, index: int) -> Fraction:
    """Return the exact positive coefficient in the redundancy identity.

    For the univariate Bernstein basis, the identity is

    ``B_i^m(s) * 4*s*(1-s) = weight * B_(i+1)^(m+2)(s)``.
    """

    if multiplier_degree < 0 or not 0 <= index <= multiplier_degree:
        raise ValueError("invalid Bernstein box-guard index")
    return Fraction(
        4 * comb(multiplier_degree, index),
        comb(multiplier_degree + 2, index + 1),
    )


def _frozen_decision_variable_counts(stage: G10StageSpec) -> tuple[int, int]:
    """Return ``(legacy_G10, reduced_G10R)`` frozen decision counts."""

    barrier_count = (stage.barrier_degree + 1) * (stage.barrier_degree + 2) // 2
    multiplier_coefficient_count = (stage.multiplier_degree + 1) ** 2
    reduced = (
        barrier_count
        + 1
        + G10R_RETAINED_SHAPE_MULTIPLIER_COUNT
        * multiplier_coefficient_count
    )
    legacy = (
        reduced
        + G10R_REMOVED_BOX_GUARD_MULTIPLIER_COUNT
        * multiplier_coefficient_count
    )
    return legacy, reduced


def _add_reduced_obligation(
    *,
    builder: _SparseLPBuilder,
    name: str,
    box: Box,
    core_terms: Sequence[tuple[int, NumericPolynomial]],
    constant_value: float,
    rho_index: int,
    multiplier_degree: int,
    shape_guard: NumericPolynomial | None = None,
) -> None:
    """Add an obligation with only a genuine curved-set multiplier."""

    terms = list(core_terms)
    if shape_guard is not None:
        multiplier_basis, _, _ = _local_multiplier_material(
            box,
            multiplier_degree,
        )
        _append_multiplier_terms(
            builder=builder,
            terms=terms,
            multiplier_basis=multiplier_basis,
            guard=shape_guard,
        )
    terms.append((rho_index, constant(-1.0)))
    builder.add_positive_family(
        name,
        indexed_polynomials=terms,
        box=box,
        constant_value=constant_value,
    )


def _build_g10r_problem(
    *,
    controller: FrozenController,
    stage: G10StageSpec,
    coefficient_bound: float = G10_COEFFICIENT_BOUND,
    multiplier_upper_bound: float = G10_MULTIPLIER_BOUND,
) -> _G10Problem:
    """Build the projection-equivalent G10R stage directly as sparse CSR."""

    if coefficient_bound <= 0.0 or multiplier_upper_bound <= 0.0:
        raise ValueError("G10R coefficient bounds must be positive")
    if stage not in G10_STAGE_SCHEDULE:
        raise ValueError("G10R stage is outside the frozen finite schedule")
    chebyshev_indices, barrier_basis = total_degree_chebyshev_basis(
        stage.barrier_degree
    )
    w_indices = tuple(range(len(barrier_basis)))
    rho_index = len(w_indices)
    builder = _SparseLPBuilder(
        core_bounds=(
            *((-float(coefficient_bound), float(coefficient_bound)) for _ in w_indices),
            G10_RHO_BOUNDS,
        ),
        multiplier_upper_bound=multiplier_upper_bound,
    )
    w_terms = tuple(zip(w_indices, barrier_basis))

    for box in _uniform_boxes(G10_FULL_BOX_PARTS_PER_AXIS):
        _add_reduced_obligation(
            builder=builder,
            name="W_nonnegative_X",
            box=box,
            core_terms=w_terms,
            constant_value=0.0,
            rho_index=rho_index,
            multiplier_degree=stage.multiplier_degree,
        )

    _add_reduced_obligation(
        builder=builder,
        name="q_minus_W_on_X0",
        box=_initial_box(),
        core_terms=tuple(
            (index, negate(polynomial)) for index, polynomial in w_terms
        ),
        constant_value=G10_Q,
        rho_index=rho_index,
        multiplier_degree=stage.multiplier_degree,
        shape_guard=initial_disk_guard(),
    )

    for unsafe in CANONICAL_SCENARIO.unsafe_components:
        _add_reduced_obligation(
            builder=builder,
            name=f"W_minus_1_{unsafe.name}",
            box=normalized_box(unsafe),
            core_terms=w_terms,
            constant_value=-1.0,
            rho_index=rho_index,
            multiplier_degree=stage.multiplier_degree,
        )

    drift_partition = _drift_partition(
        controller,
        base_refinement_depth=G10_BASE_REFINEMENT_DEPTH,
        goal_boundary_refinement_depth=G10_GOAL_BOUNDARY_REFINEMENT_DEPTH,
    )
    drift_basis_by_piece = {
        piece.piece_id: _chebyshev_drift_basis(
            controller=controller,
            piece=piece,
            barrier_basis=barrier_basis,
        )
        for piece in controller_pieces(controller)
    }
    drift_class_counts = {"outside": 0, "boundary": 0}
    for piece, box, goal_class in drift_partition:
        drift_class_counts[goal_class] += 1
        _add_reduced_obligation(
            builder=builder,
            name=f"drift_{piece.piece_id}_{goal_class}",
            box=box,
            core_terms=tuple(
                zip(w_indices, drift_basis_by_piece[piece.piece_id])
            ),
            constant_value=-G10_EPSBAR,
            rho_index=rho_index,
            multiplier_degree=stage.multiplier_degree,
            shape_guard=(
                goal_exterior_guard() if goal_class == "boundary" else None
            ),
        )

    active_obligation_count = (
        G10_FULL_BOX_PARTS_PER_AXIS**2
        + 1
        + len(CANONICAL_SCENARIO.unsafe_components)
        + len(drift_partition)
    )
    if len(drift_partition) != G10R_FROZEN_ACTIVE_DRIFT_BOX_COUNT:
        raise ValueError("G10R frozen active drift partition mismatch")
    if drift_class_counts["boundary"] != G10R_FROZEN_GOAL_BOUNDARY_BOX_COUNT:
        raise ValueError("G10R frozen goal-boundary partition mismatch")
    if active_obligation_count != G10R_FROZEN_OBLIGATION_COUNT:
        raise ValueError("G10R frozen obligation count mismatch")
    if builder.multiplier_polynomial_count != G10R_RETAINED_SHAPE_MULTIPLIER_COUNT:
        raise ValueError("G10R retained shape multiplier count mismatch")

    matrix, right_hand_side, bounds = builder.finalize()
    objective = np.zeros(matrix.shape[1], dtype=np.float64)
    objective[rho_index] = -1.0
    legacy_variable_count, expected_reduced_variable_count = (
        _frozen_decision_variable_counts(stage)
    )
    if matrix.shape[1] != expected_reduced_variable_count:
        raise ValueError("G10R reduced decision variable count mismatch")
    metadata = {
        "matrix_format": "csr",
        "global_dense_matrix_constructed": False,
        "decision_variable_count": int(matrix.shape[1]),
        "legacy_G10_decision_variable_count": legacy_variable_count,
        "removed_decision_variable_count": (
            legacy_variable_count - expected_reduced_variable_count
        ),
        "total_linear_inequalities": int(matrix.shape[0]),
        "matrix_nonzero_count": int(matrix.nnz),
        "matrix_csr_bytes": int(
            matrix.data.nbytes + matrix.indices.nbytes + matrix.indptr.nbytes
        ),
        "barrier_coefficient_count": len(w_indices),
        "rho_variable_count": 1,
        "multiplier_polynomial_count": builder.multiplier_polynomial_count,
        "multiplier_coefficient_count": builder.multiplier_coefficient_count,
        "retained_shape_multiplier_polynomial_count": (
            builder.multiplier_polynomial_count
        ),
        "removed_redundant_box_guard_multiplier_polynomial_count": (
            G10R_REMOVED_BOX_GUARD_MULTIPLIER_COUNT
        ),
        "rectangle_guard_multiplier_count": 0,
        "projection_equivalence_proof_id": G10R_REDUNDANCY_PROOF_ID,
        "active_obligation_count": active_obligation_count,
        "full_X_box_count": G10_FULL_BOX_PARTS_PER_AXIS**2,
        "unsafe_box_count": len(CANONICAL_SCENARIO.unsafe_components),
        "active_drift_box_count": len(drift_partition),
        "drift_goal_class_counts": drift_class_counts,
        "constraint_counts": builder.constraint_counts,
    }
    return _G10Problem(
        matrix=matrix,
        right_hand_side=right_hand_side,
        objective=objective,
        bounds=bounds,
        chebyshev_indices=chebyshev_indices,
        chebyshev_basis=barrier_basis,
        rho_index=rho_index,
        metadata=metadata,
    )


def solve_g10r_stage(
    *,
    frozen_controller_path: str | Path,
    output_dir: str | Path,
    stage: G10StageSpec,
) -> Path:
    """Solve one G10R stage and invoke V3 for a feasible positive margin."""

    if stage not in G10_STAGE_SCHEDULE:
        raise ValueError("G10R stage is outside the frozen finite schedule")
    controller_path = Path(frozen_controller_path).resolve()
    controller = load_frozen_controller(controller_path)
    controller_file_hash = file_sha256(controller_path)
    _assert_frozen_c2(
        controller,
        controller_file_sha256=controller_file_hash,
    )
    destination = Path(output_dir).resolve()
    destination.mkdir(parents=True, exist_ok=False)
    manifest_path = destination / "manifest.json"
    manifest: dict[str, Any] = {
        "schema_version": 1,
        "kind": "g10r_redundancy_reduced_shape_multiplier_synthesis",
        "synthesis_id": G10R_SYNTHESIS_ID,
        "projection_equivalence_proof_id": G10R_REDUNDANCY_PROOF_ID,
        "status": "building",
        "stage": {
            "stage_id": stage.stage_id,
            "barrier_degree": stage.barrier_degree,
            "multiplier_degree": stage.multiplier_degree,
            "time_limit_s": stage.time_limit_s,
        },
        "fixed_scalars": {
            "q": G10_Q,
            "q_float64_hex": G10_Q.hex(),
            "q_exactly_no_greater_than_one_tenth": (
                Fraction.from_float(G10_Q) <= Fraction(1, 10)
            ),
            "epsbar": G10_EPSBAR,
            "epsbar_float64_hex": G10_EPSBAR.hex(),
            "epsbar_semantics": "exact_binary64_artifact_value",
        },
        "controller": {
            "file_sha256": controller_file_hash,
            "content_sha256": controller.sha256,
            "frozen_file_sha256": G10_CONTROLLER_FILE_SHA256,
            "frozen_content_sha256": G10_CONTROLLER_CONTENT_SHA256,
        },
        "options": {
            "barrier_basis": "total_degree_tensor_chebyshev",
            "coefficient_bound": G10_COEFFICIENT_BOUND,
            "rho_bounds": list(G10_RHO_BOUNDS),
            "retained_multiplier_basis": "local_tensor_bernstein",
            "multiplier_upper_bound": G10_MULTIPLIER_BOUND,
            "box_guard_multipliers_per_obligation": 0,
            "same_box_rectangle_guards_eliminated_as_redundant": True,
            "initial_disk_polynomial_multiplier": True,
            "goal_exterior_polynomial_multiplier_per_boundary_box": True,
            "full_box_parts_per_axis": G10_FULL_BOX_PARTS_PER_AXIS,
            "base_refinement_depth": G10_BASE_REFINEMENT_DEPTH,
            "goal_boundary_refinement_depth": (
                G10_GOAL_BOUNDARY_REFINEMENT_DEPTH
            ),
            "objective": "maximize_common_rho_with_q_fixed",
        },
        "solver": {
            "name": "SciPy HiGHS LP",
            "method": "highs",
            "primal_feasibility_tolerance": 1.0e-9,
            "dual_feasibility_tolerance": 1.0e-9,
            "solver_status_is_not_formal_evidence": True,
        },
        "independent_validation": {
            "required": G10_VALIDATOR_ID,
            "reads_solver_status": False,
            "reads_multiplier_values": False,
            "acceptance_tolerance": 0.0,
        },
    }
    _save_manifest(manifest_path, manifest)

    problem = _build_g10r_problem(controller=controller, stage=stage)
    problem_metadata = dict(problem.metadata)
    manifest["problem"] = problem_metadata
    manifest["status"] = "solving"
    _save_manifest(manifest_path, manifest)
    result = linprog(
        problem.objective,
        A_ub=problem.matrix,
        b_ub=problem.right_hand_side,
        bounds=problem.bounds,
        method="highs",
        options={
            "presolve": True,
            "time_limit": float(stage.time_limit_s),
            "primal_feasibility_tolerance": 1.0e-9,
            "dual_feasibility_tolerance": 1.0e-9,
        },
    )
    manifest["solver_result"] = {
        "status_code": int(result.status),
        "success": bool(result.success),
        "message": str(result.message),
        "iterations": int(result.nit),
        "optimality_required_for_formal_claim": False,
    }
    values = _candidate_vector(result, problem.matrix.shape[1])
    if values is None:
        manifest["status"] = "infeasible_or_unknown_no_numeric_candidate"
        _save_manifest(manifest_path, manifest)
        return manifest_path

    rho = float(values[problem.rho_index])
    minimum_lp_slack = float(
        np.min(problem.right_hand_side - problem.matrix @ values)
    )
    maximum_bound_violation = _maximum_bound_violation(values, problem.bounds)
    numeric_incumbent_feasible = (
        minimum_lp_slack >= -G10_NUMERIC_FEASIBILITY_TOLERANCE
        and maximum_bound_violation <= G10_NUMERIC_FEASIBILITY_TOLERANCE
    )
    manifest["numeric_incumbent"] = {
        "rho": rho,
        "rho_float64_hex": rho.hex(),
        "minimum_lp_slack": minimum_lp_slack,
        "maximum_bound_violation": maximum_bound_violation,
        "feasibility_tolerance": G10_NUMERIC_FEASIBILITY_TOLERANCE,
        "within_numeric_solver_tolerance": numeric_incumbent_feasible,
        "formal_claim_allowed": False,
    }
    if not rho > 0.0:
        manifest["status"] = "not_ready_nonpositive_rho"
        _save_manifest(manifest_path, manifest)
        return manifest_path
    if not numeric_incumbent_feasible:
        manifest["status"] = "not_ready_numeric_incumbent_outside_solver_tolerance"
        _save_manifest(manifest_path, manifest)
        return manifest_path

    coefficient_count = len(problem.chebyshev_indices)
    chebyshev_coefficients = tuple(
        float(value) for value in values[:coefficient_count]
    )
    barrier = chebyshev_coefficients_to_monomial(
        indices=problem.chebyshev_indices,
        coefficients=chebyshev_coefficients,
        basis=problem.chebyshev_basis,
    )
    multiplier_values = values[problem.rho_index + 1 :]
    candidate = {
        "schema_version": 1,
        "kind": "g10r_qsbc_numeric_candidate",
        "synthesis_id": G10R_SYNTHESIS_ID,
        "projection_equivalence_proof_id": G10R_REDUNDANCY_PROOF_ID,
        "stage_id": stage.stage_id,
        "controller_content_sha256": controller.sha256,
        "chebyshev_barrier": {
            "basis": "total_degree_tensor_chebyshev",
            "indices": [list(index) for index in problem.chebyshev_indices],
            "coefficients_decimal": list(chebyshev_coefficients),
            "coefficients_float64_hex": [
                value.hex() for value in chebyshev_coefficients
            ],
        },
        "barrier": barrier.payload(),
        "q": G10_Q,
        "q_float64_hex": G10_Q.hex(),
        "epsbar": G10_EPSBAR,
        "epsbar_float64_hex": G10_EPSBAR.hex(),
        "rho": rho,
        "rho_float64_hex": rho.hex(),
        "minimum_lp_slack": minimum_lp_slack,
        "retained_shape_multiplier_summary": {
            "polynomial_count": problem.metadata["multiplier_polynomial_count"],
            "coefficient_count": int(multiplier_values.size),
            "minimum": float(np.min(multiplier_values)),
            "maximum": float(np.max(multiplier_values)),
            "values_not_consumed_by_independent_validator": True,
        },
        "removed_rectangle_multiplier_polynomial_count": (
            G10R_REMOVED_BOX_GUARD_MULTIPLIER_COUNT
        ),
        "formal_claim_allowed_before_independent_validation": False,
        "validation_subject_sha256": content_sha256(barrier.payload()),
    }
    candidate["content_sha256"] = content_sha256(candidate)
    candidate_path = destination / "numeric_candidate.json"
    _write_json(candidate_path, candidate)
    manifest["numeric_candidate"] = {
        "file": candidate_path.name,
        "file_sha256": file_sha256(candidate_path),
        "content_sha256": candidate["content_sha256"],
    }
    manifest["status"] = "positive_rho_pending_v3"
    _save_manifest(manifest_path, manifest)

    del multiplier_values, values, result, problem
    gc.collect()

    from .rational_certificate_validation import validate_qsbc_values_v3

    validation_path = validate_qsbc_values_v3(
        barrier=barrier,
        q=G10_Q,
        epsbar=G10_EPSBAR,
        controller=controller,
        output_dir=destination / "independent_validation_v3",
    )
    validation = json.loads(validation_path.read_text(encoding="utf-8"))
    release_gate_passed, release_gate_failures = _v3_release_gate(
        validation=validation,
        barrier=barrier,
        controller=controller,
        validation_directory=validation_path.parent,
    )
    manifest["independent_validation"] = {
        "file": str(validation_path.relative_to(destination)),
        "file_sha256": file_sha256(validation_path),
        "content_sha256": validation["report_content_sha256"],
        "validator_id": validation["validator_id"],
        "status": validation["status"],
        "first_slice_ready": validation["first_slice_ready"],
        "acceptance_tolerance": validation["formal_acceptance_tolerance"],
        "barrier_payload_sha256": validation.get("barrier_payload_sha256"),
        "controller_content_sha256": validation.get("controller_content_sha256"),
        "release_gate_passed": release_gate_passed,
        "release_gate_failures": list(release_gate_failures),
    }
    if not release_gate_passed:
        manifest["status"] = "validation_release_gate_failed"
        _save_manifest(manifest_path, manifest)
        return manifest_path

    certificate = QSBCertificate(
        certificate_id=f"CBF_ROVER_G10R_{stage.stage_id}_{controller.sha256[:12]}",
        barrier=barrier,
        q=G10_Q,
        epsbar=G10_EPSBAR,
        controller_content_sha256=controller.sha256,
        schema_version=2,
        solver={
            "method": "sparse_Bernstein_shape_multiplier_LP",
            "synthesis_id": G10R_SYNTHESIS_ID,
            "projection_equivalence_proof_id": G10R_REDUNDANCY_PROOF_ID,
            "solver": "SciPy HiGHS",
            "stage_id": stage.stage_id,
            "barrier_basis": "total_degree_tensor_chebyshev",
            "barrier_degree": stage.barrier_degree,
            "multiplier_degree": stage.multiplier_degree,
            "controller_degree": controller_max_degree(controller),
            "rho": rho,
            "minimum_lp_slack": minimum_lp_slack,
            "constraint_count": problem_metadata["total_linear_inequalities"],
            "decision_variable_count": problem_metadata["decision_variable_count"],
            "matrix_nonzero_count": problem_metadata["matrix_nonzero_count"],
        },
        validation={
            "validator_id": validation["validator_id"],
            "validation_report_sha256": file_sha256(validation_path),
            "validation_report_content_sha256": validation[
                "report_content_sha256"
            ],
            "status": validation["status"],
            "first_slice_ready": validation["first_slice_ready"],
            "formal_acceptance_tolerance": 0.0,
            "barrier_payload_sha256": validation["barrier_payload_sha256"],
            "controller_content_sha256": validation[
                "controller_content_sha256"
            ],
            "q_float64_hex": validation["q_float64_hex"],
            "epsbar_float64_hex": validation["epsbar_float64_hex"],
        },
    )
    certificate_path = destination / "certificate.json"
    certificate.save(certificate_path)
    manifest["certificate"] = {
        "file": certificate_path.name,
        "file_sha256": file_sha256(certificate_path),
        "content_sha256": certificate.sha256,
    }
    manifest["status"] = "ready"
    _save_manifest(manifest_path, manifest)
    return manifest_path


def synthesize_g10r_qsbc_schedule(
    *,
    frozen_controller_path: str | Path,
    output_dir: str | Path,
) -> Path:
    """Run the unchanged finite G10 schedule through the reduced G10R LP."""

    destination = Path(output_dir).resolve()
    destination.mkdir(parents=True, exist_ok=False)
    manifest_path = destination / "schedule_manifest.json"
    schedule_manifest: dict[str, Any] = {
        "schema_version": 1,
        "kind": "g10r_finite_qsbc_schedule",
        "synthesis_id": G10R_SYNTHESIS_ID,
        "projection_equivalence_proof_id": G10R_REDUNDANCY_PROOF_ID,
        "status": "running",
        "q": G10_Q,
        "q_float64_hex": G10_Q.hex(),
        "epsbar": G10_EPSBAR,
        "epsbar_float64_hex": G10_EPSBAR.hex(),
        "schedule": [
            {
                "stage_id": stage.stage_id,
                "barrier_degree": stage.barrier_degree,
                "multiplier_degree": stage.multiplier_degree,
                "time_limit_s": stage.time_limit_s,
            }
            for stage in G10_STAGE_SCHEDULE
        ],
        "automatic_escalation_beyond_schedule_allowed": False,
        "attempts": [],
    }
    _save_manifest(manifest_path, schedule_manifest)
    for stage in G10_STAGE_SCHEDULE:
        stage_directory = destination / stage.stage_id
        try:
            stage_manifest_path = solve_g10r_stage(
                frozen_controller_path=frozen_controller_path,
                output_dir=stage_directory,
                stage=stage,
            )
        except Exception as error:
            schedule_manifest["attempts"].append(
                {
                    "stage_id": stage.stage_id,
                    "status": "execution_error",
                    "error_type": type(error).__name__,
                    "error_message": str(error),
                    "formal_claim_allowed": False,
                }
            )
            schedule_manifest["status"] = "not_ready"
            schedule_manifest["terminal_reason"] = (
                "G10R_stage_execution_error_fail_closed"
            )
            _save_manifest(manifest_path, schedule_manifest)
            return manifest_path
        stage_manifest = json.loads(stage_manifest_path.read_text(encoding="utf-8"))
        schedule_manifest["attempts"].append(
            {
                "stage_id": stage.stage_id,
                "status": stage_manifest["status"],
                "rho": stage_manifest.get("numeric_incumbent", {}).get("rho"),
                "manifest": str(stage_manifest_path.relative_to(destination)),
                "manifest_file_sha256": file_sha256(stage_manifest_path),
            }
        )
        if stage_manifest["status"] == "ready":
            schedule_manifest["status"] = "ready"
            schedule_manifest["selected_stage"] = stage.stage_id
            schedule_manifest["certificate"] = {
                "file": str(
                    (stage_directory / "certificate.json").relative_to(destination)
                ),
                "file_sha256": file_sha256(stage_directory / "certificate.json"),
            }
            _save_manifest(manifest_path, schedule_manifest)
            return manifest_path
        _save_manifest(manifest_path, schedule_manifest)
    schedule_manifest["status"] = "not_ready"
    schedule_manifest["terminal_reason"] = (
        "all_finite_G10R_stages_failed_V3_ready_gate"
    )
    _save_manifest(manifest_path, schedule_manifest)
    return manifest_path
