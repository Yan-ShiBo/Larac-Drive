"""G10 sparse Chebyshev q-SBC synthesis with polynomial multipliers.

G10 keeps the confirmed M1/S1/P1/D1/N1 model, the frozen C2 controller, and
the G4 theorem unchanged.  It fixes a binary64 risk bound strictly below
``1/10`` and maximizes a common positivity margin.  Every rectangular domain
guard and each curved X0/goal guard receives an independent polynomial that is
nonnegative by construction in a local tensor Bernstein basis.

HiGHS only proposes numeric candidates.  A positive solver margin is merely a
gate to the solver-independent exact-rational V3 replay, which reads neither
the LP matrix nor any multiplier.
"""

from __future__ import annotations

from dataclasses import dataclass
from fractions import Fraction
import gc
import json
from math import comb, isfinite, nextafter
from pathlib import Path
from typing import Any, Mapping, Sequence

import numpy as np
from scipy.optimize import linprog
from scipy.sparse import coo_matrix, csr_matrix

from cbf_rover.adapters.training.sac_reference import file_sha256
from cbf_rover.domain.planar import CANONICAL_SCENARIO, CANONICAL_SYSTEM

from .algebra import (
    X_VARIABLE,
    Y_VARIABLE,
    add,
    constant,
    exact_expected_next_polynomial,
    multiply,
    negate,
    power,
    scale,
)
from .bernstein_lp import (
    Box,
    _polynomial_family_bernstein_matrix,
    _uniform_boxes,
)
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
from .polynomial import (
    FrozenController,
    FrozenPiecewisePolynomialController,
    NumericPolynomial,
    PolynomialControllerPiece,
    Power2D,
    SparsePolynomial2D,
    content_sha256,
    controller_max_degree,
    controller_pieces,
    load_frozen_controller,
)


G10_SYNTHESIS_ID = "G10_CHEBYSHEV_POLYNOMIAL_MULTIPLIER_SPARSE_LP_V1"
G10_CONTROLLER_ID = "CBF_ROVER_C2_LOWER_ROUTE_V1"
G10_CONTROLLER_CONTENT_SHA256 = (
    "b0cacedfcd4b884961a786b24e84b2f0f33bbc4ac81aef0b1ff1b0dcfe285984"
)
G10_CONTROLLER_FILE_SHA256 = (
    "df0dbfd2cc5b3aa0a049b5dbb6e7d0360c6b260801e183e95acfb7a9bccf715d"
)
G10_VALIDATOR_ID = "G5_EXACT_RATIONAL_BERNSTEIN_REPLAY_V3"
G10_Q = nextafter(0.1, 0.0)
G10_EPSBAR = 1.0e-6
G10_NUMERIC_FEASIBILITY_TOLERANCE = 1.0e-9
G10_COEFFICIENT_BOUND = 1000.0
G10_MULTIPLIER_BOUND = 100_000.0
G10_RHO_BOUNDS = (-1.0, 1.0)
G10_FULL_BOX_PARTS_PER_AXIS = 4
G10_BASE_REFINEMENT_DEPTH = 1
G10_GOAL_BOUNDARY_REFINEMENT_DEPTH = 2


@dataclass(frozen=True, slots=True)
class G10StageSpec:
    """One immutable member of the finite G10 schedule."""

    stage_id: str
    barrier_degree: int
    multiplier_degree: int
    time_limit_s: float

    def __post_init__(self) -> None:
        if self.barrier_degree not in (14, 16):
            raise ValueError("G10 barrier degree must be 14 or 16")
        if self.multiplier_degree not in (2, 4):
            raise ValueError("G10 multiplier degree must be 2 or 4")
        if not isfinite(self.time_limit_s) or self.time_limit_s <= 0.0:
            raise ValueError("G10 time limit must be positive and finite")


G10_STAGE_SCHEDULE = (
    G10StageSpec("W14_M2", 14, 2, 900.0),
    G10StageSpec("W16_M2", 16, 2, 3600.0),
    G10StageSpec("W16_M4", 16, 4, 7200.0),
)


@dataclass(frozen=True, slots=True)
class _G10Problem:
    matrix: csr_matrix
    right_hand_side: np.ndarray
    objective: np.ndarray
    bounds: tuple[tuple[float, float], ...]
    chebyshev_indices: tuple[Power2D, ...]
    chebyshev_basis: tuple[NumericPolynomial, ...]
    rho_index: int
    metadata: Mapping[str, Any]


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


def _numeric_to_sparse(polynomial: NumericPolynomial) -> SparsePolynomial2D:
    if not polynomial:
        return SparsePolynomial2D(((0, 0),), (0.0,))
    powers = tuple(
        sorted(
            polynomial,
            key=lambda item: (sum(item), item[0], item[1]),
        )
    )
    return SparsePolynomial2D(
        powers=powers,
        coefficients=tuple(float(polynomial[item]) for item in powers),
    )


def _chebyshev_drift_basis(
    *,
    controller: FrozenController,
    piece: PolynomialControllerPiece,
    barrier_basis: Sequence[NumericPolynomial],
) -> tuple[NumericPolynomial, ...]:
    return tuple(
        add(
            basis_polynomial,
            negate(
                exact_expected_next_polynomial(
                    _numeric_to_sparse(basis_polynomial),
                    controller,
                    CANONICAL_SYSTEM,
                    piece=piece,
                )
            ),
        )
        for basis_polynomial in barrier_basis
    )


def _axis_local_coordinate(
    variable: NumericPolynomial,
    low: float,
    high: float,
) -> NumericPolynomial:
    if not low < high:
        raise ValueError("local Bernstein interval must be ordered")
    return scale(add(variable, constant(-low)), 1.0 / (high - low))


def _axis_bernstein_basis(
    coordinate: NumericPolynomial,
    degree: int,
) -> tuple[NumericPolynomial, ...]:
    complement = add(constant(1.0), negate(coordinate))
    return tuple(
        scale(
            multiply(
                power(coordinate, index),
                power(complement, degree - index),
            ),
            float(comb(degree, index)),
        )
        for index in range(degree + 1)
    )


def _local_multiplier_material(
    box: Box,
    degree: int,
) -> tuple[tuple[NumericPolynomial, ...], NumericPolynomial, NumericPolynomial]:
    x_low, x_high, y_low, y_high = box
    local_x = _axis_local_coordinate(X_VARIABLE, x_low, x_high)
    local_y = _axis_local_coordinate(Y_VARIABLE, y_low, y_high)
    x_basis = _axis_bernstein_basis(local_x, degree)
    y_basis = _axis_bernstein_basis(local_y, degree)
    basis = tuple(
        multiply(x_polynomial, y_polynomial)
        for x_polynomial in x_basis
        for y_polynomial in y_basis
    )
    guard_x = scale(
        multiply(local_x, add(constant(1.0), negate(local_x))),
        4.0,
    )
    guard_y = scale(
        multiply(local_y, add(constant(1.0), negate(local_y))),
        4.0,
    )
    return basis, guard_x, guard_y


class _SparseLPBuilder:
    """Append local dense Bernstein blocks into one global sparse CSR LP."""

    def __init__(
        self,
        *,
        core_bounds: Sequence[tuple[float, float]],
        multiplier_upper_bound: float,
    ) -> None:
        self.bounds = list(core_bounds)
        self.multiplier_upper_bound = float(multiplier_upper_bound)
        self.row_parts: list[np.ndarray] = []
        self.column_parts: list[np.ndarray] = []
        self.data_parts: list[np.ndarray] = []
        self.rhs_parts: list[np.ndarray] = []
        self.row_count = 0
        self.constraint_counts: dict[str, int] = {}
        self.multiplier_polynomial_count = 0
        self.multiplier_coefficient_count = 0

    def allocate_multiplier(self, coefficient_count: int) -> tuple[int, ...]:
        if coefficient_count <= 0:
            raise ValueError("multiplier coefficient count must be positive")
        start = len(self.bounds)
        self.bounds.extend(
            (0.0, self.multiplier_upper_bound)
            for _ in range(coefficient_count)
        )
        self.multiplier_polynomial_count += 1
        self.multiplier_coefficient_count += coefficient_count
        return tuple(range(start, start + coefficient_count))

    def add_positive_family(
        self,
        name: str,
        *,
        indexed_polynomials: Sequence[tuple[int, NumericPolynomial]],
        box: Box,
        constant_value: float,
    ) -> None:
        variable_indices = tuple(item[0] for item in indexed_polynomials)
        if len(set(variable_indices)) != len(variable_indices):
            raise ValueError("one residual may contain each LP variable only once")
        local = _polynomial_family_bernstein_matrix(
            tuple(item[1] for item in indexed_polynomials),
            box,
        )
        local_rows, local_columns = np.nonzero(local)
        if local_rows.size:
            self.row_parts.append(local_rows.astype(np.int64) + self.row_count)
            self.column_parts.append(
                np.asarray(variable_indices, dtype=np.int64)[local_columns]
            )
            self.data_parts.append(-local[local_rows, local_columns])
        self.rhs_parts.append(
            np.full(local.shape[0], float(constant_value), dtype=np.float64)
        )
        self.row_count += int(local.shape[0])
        self.constraint_counts[name] = (
            self.constraint_counts.get(name, 0) + int(local.shape[0])
        )

    def finalize(self) -> tuple[csr_matrix, np.ndarray, tuple[tuple[float, float], ...]]:
        if self.row_count <= 0 or not self.rhs_parts:
            raise ValueError("G10 LP must contain at least one constraint")
        if self.data_parts:
            rows = np.concatenate(self.row_parts)
            columns = np.concatenate(self.column_parts)
            data = np.concatenate(self.data_parts)
        else:
            rows = np.empty(0, dtype=np.int64)
            columns = np.empty(0, dtype=np.int64)
            data = np.empty(0, dtype=np.float64)
        matrix = coo_matrix(
            (data, (rows, columns)),
            shape=(self.row_count, len(self.bounds)),
            dtype=np.float64,
        ).tocsr()
        matrix.sum_duplicates()
        matrix.eliminate_zeros()
        return matrix, np.concatenate(self.rhs_parts), tuple(self.bounds)


def _append_multiplier_terms(
    *,
    builder: _SparseLPBuilder,
    terms: list[tuple[int, NumericPolynomial]],
    multiplier_basis: Sequence[NumericPolynomial],
    guard: NumericPolynomial,
) -> None:
    indices = builder.allocate_multiplier(len(multiplier_basis))
    terms.extend(
        (
            variable_index,
            negate(multiply(basis_polynomial, guard)),
        )
        for variable_index, basis_polynomial in zip(indices, multiplier_basis)
    )


def _add_guarded_obligation(
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
    multiplier_basis, guard_x, guard_y = _local_multiplier_material(
        box,
        multiplier_degree,
    )
    terms = list(core_terms)
    _append_multiplier_terms(
        builder=builder,
        terms=terms,
        multiplier_basis=multiplier_basis,
        guard=guard_x,
    )
    _append_multiplier_terms(
        builder=builder,
        terms=terms,
        multiplier_basis=multiplier_basis,
        guard=guard_y,
    )
    if shape_guard is not None:
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


def _initial_box() -> Box:
    initial = CANONICAL_SCENARIO.initial
    normalization = CANONICAL_SCENARIO.normalization
    return (
        (initial.center_x - initial.radius - normalization.center_x_m)
        / normalization.scale_x_m,
        (initial.center_x + initial.radius - normalization.center_x_m)
        / normalization.scale_x_m,
        (initial.center_y - initial.radius - normalization.center_y_m)
        / normalization.scale_y_m,
        (initial.center_y + initial.radius - normalization.center_y_m)
        / normalization.scale_y_m,
    )


def _assert_frozen_c2(
    controller: FrozenController,
    *,
    controller_file_sha256: str,
) -> None:
    if not isinstance(controller, FrozenPiecewisePolynomialController):
        raise ValueError("G10 requires the frozen piecewise C2 controller")
    if controller.controller_id != G10_CONTROLLER_ID:
        raise ValueError("G10 controller identity mismatch")
    if controller.sha256 != G10_CONTROLLER_CONTENT_SHA256:
        raise ValueError("G10 controller content hash mismatch")
    if controller_file_sha256 != G10_CONTROLLER_FILE_SHA256:
        raise ValueError("G10 controller file hash mismatch")
    decisions = tuple(controller.construction.get("decision_ids", ()))
    required = {
        "DX1_PB1_STRUCTURAL_MISMATCH_DIAGNOSTIC_V1",
        "C2_FIXED_LOWER_PIECEWISE_POLYNOMIAL_V1",
        "C2V1_PIECEWISE_BERNSTEIN_AND_1000_ROLLOUTS",
        "C1_FREEZE_CONTROLLER_THEN_CERTIFY",
    }
    if not required.issubset(decisions):
        raise ValueError("G10 controller lacks the frozen C2V1 provenance")


def _build_g10_problem(
    *,
    controller: FrozenController,
    stage: G10StageSpec,
    coefficient_bound: float = G10_COEFFICIENT_BOUND,
    multiplier_upper_bound: float = G10_MULTIPLIER_BOUND,
) -> _G10Problem:
    """Build one G10 stage without allocating a global dense matrix."""

    if coefficient_bound <= 0.0 or multiplier_upper_bound <= 0.0:
        raise ValueError("G10 coefficient bounds must be positive")
    if stage not in G10_STAGE_SCHEDULE:
        raise ValueError("G10 stage is outside the frozen finite schedule")
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
        _add_guarded_obligation(
            builder=builder,
            name="W_nonnegative_X",
            box=box,
            core_terms=w_terms,
            constant_value=0.0,
            rho_index=rho_index,
            multiplier_degree=stage.multiplier_degree,
        )

    _add_guarded_obligation(
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
        _add_guarded_obligation(
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
        _add_guarded_obligation(
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

    matrix, right_hand_side, bounds = builder.finalize()
    objective = np.zeros(matrix.shape[1], dtype=np.float64)
    objective[rho_index] = -1.0
    metadata = {
        "matrix_format": "csr",
        "global_dense_matrix_constructed": False,
        "decision_variable_count": int(matrix.shape[1]),
        "total_linear_inequalities": int(matrix.shape[0]),
        "matrix_nonzero_count": int(matrix.nnz),
        "matrix_csr_bytes": int(
            matrix.data.nbytes + matrix.indices.nbytes + matrix.indptr.nbytes
        ),
        "barrier_coefficient_count": len(w_indices),
        "rho_variable_count": 1,
        "multiplier_polynomial_count": builder.multiplier_polynomial_count,
        "multiplier_coefficient_count": builder.multiplier_coefficient_count,
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


def _candidate_vector(result: Any, expected_size: int) -> np.ndarray | None:
    raw = getattr(result, "x", None)
    if raw is None:
        return None
    values = np.asarray(raw, dtype=np.float64)
    if values.shape != (expected_size,) or not np.isfinite(values).all():
        return None
    return values


def _maximum_bound_violation(
    values: np.ndarray,
    bounds: Sequence[tuple[float, float]],
) -> float:
    return float(
        max(
            max(low - float(value), float(value) - high, 0.0)
            for value, (low, high) in zip(values, bounds)
        )
    )


def _v3_release_gate(
    *,
    validation: Mapping[str, Any],
    barrier: SparsePolynomial2D,
    controller: FrozenController,
    validation_directory: Path,
) -> tuple[bool, tuple[str, ...]]:
    """Cross-bind every formal subject before a G10 certificate is published."""

    declared_report_hash = str(validation.get("report_content_sha256", ""))
    report_material = dict(validation)
    report_material.pop("report_content_sha256", None)
    expected_q = Fraction.from_float(G10_Q)
    expected_epsbar = Fraction.from_float(G10_EPSBAR)
    tree_bindings_valid = True
    for group_name in ("constraints", "controller_constraints"):
        group = validation.get(group_name)
        if not isinstance(group, Mapping):
            tree_bindings_valid = False
            continue
        for item in group.values():
            if not isinstance(item, Mapping):
                tree_bindings_valid = False
                continue
            tree_name = item.get("tree_file")
            expected_tree_hash = item.get("tree_file_sha256")
            if not isinstance(tree_name, str) or Path(tree_name).name != tree_name:
                tree_bindings_valid = False
                continue
            tree_path = validation_directory / tree_name
            if (
                not isinstance(expected_tree_hash, str)
                or len(expected_tree_hash) != 64
                or not tree_path.is_file()
                or file_sha256(tree_path) != expected_tree_hash
            ):
                tree_bindings_valid = False
    checks = {
        "report_schema_version": validation.get("schema_version") == 2,
        "report_content_sha256": (
            len(declared_report_hash) == 64
            and content_sha256(report_material) == declared_report_hash
        ),
        "validator_id": validation.get("validator_id") == G10_VALIDATOR_ID,
        "status": validation.get("status") == "validated",
        "first_slice_ready": validation.get("first_slice_ready") is True,
        "formal_acceptance_tolerance": (
            validation.get("formal_acceptance_tolerance") == 0.0
        ),
        "controller_content_sha256": (
            validation.get("controller_content_sha256") == controller.sha256
        ),
        "barrier_payload_sha256": (
            validation.get("barrier_payload_sha256")
            == content_sha256(barrier.payload())
        ),
        "barrier_degree": validation.get("barrier_degree") == barrier.degree,
        "barrier_term_count": (
            validation.get("barrier_term_count") == len(barrier.powers)
        ),
        "q_float64_hex": validation.get("q_float64_hex") == G10_Q.hex(),
        "q_exact": validation.get("q_exact") == str(expected_q),
        "epsbar_float64_hex": (
            validation.get("epsbar_float64_hex") == G10_EPSBAR.hex()
        ),
        "epsbar_exact": validation.get("epsbar_exact") == str(expected_epsbar),
        "theorem_scalar_gate": validation.get("theorem_scalar_gate") is True,
        "proof_tree_file_bindings": tree_bindings_valid,
    }
    failures = tuple(name for name, passed in checks.items() if not passed)
    return not failures, failures


def solve_g10_stage(
    *,
    frozen_controller_path: str | Path,
    output_dir: str | Path,
    stage: G10StageSpec,
) -> Path:
    """Solve one G10 stage and invoke V3 only for a positive-rho candidate."""

    if stage not in G10_STAGE_SCHEDULE:
        raise ValueError("G10 stage is outside the frozen finite schedule")
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
        "kind": "g10_sparse_polynomial_multiplier_synthesis",
        "synthesis_id": G10_SYNTHESIS_ID,
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
            "multiplier_basis": "local_tensor_bernstein",
            "multiplier_upper_bound": G10_MULTIPLIER_BOUND,
            "box_guard_multipliers_per_obligation": 2,
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
            "required": "G5_EXACT_RATIONAL_BERNSTEIN_REPLAY_V3",
            "reads_solver_status": False,
            "reads_multiplier_values": False,
            "acceptance_tolerance": 0.0,
        },
    }
    _save_manifest(manifest_path, manifest)

    problem = _build_g10_problem(controller=controller, stage=stage)
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
        "kind": "g10_qsbc_numeric_candidate",
        "synthesis_id": G10_SYNTHESIS_ID,
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
        "multiplier_summary": {
            "polynomial_count": problem.metadata["multiplier_polynomial_count"],
            "coefficient_count": int(multiplier_values.size),
            "minimum": float(np.min(multiplier_values)),
            "maximum": float(np.max(multiplier_values)),
            "values_not_consumed_by_independent_validator": True,
        },
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
        certificate_id=f"CBF_ROVER_G10_{stage.stage_id}_{controller.sha256[:12]}",
        barrier=barrier,
        q=G10_Q,
        epsbar=G10_EPSBAR,
        controller_content_sha256=controller.sha256,
        schema_version=2,
        solver={
            "method": "sparse_Bernstein_polynomial_multiplier_LP",
            "synthesis_id": G10_SYNTHESIS_ID,
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


def synthesize_g10_qsbc_schedule(
    *,
    frozen_controller_path: str | Path,
    output_dir: str | Path,
) -> Path:
    """Run the finite W14/M2 -> W16/M2 -> W16/M4 G10 schedule."""

    destination = Path(output_dir).resolve()
    destination.mkdir(parents=True, exist_ok=False)
    manifest_path = destination / "schedule_manifest.json"
    schedule_manifest: dict[str, Any] = {
        "schema_version": 1,
        "kind": "g10_finite_qsbc_schedule",
        "synthesis_id": G10_SYNTHESIS_ID,
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
            stage_manifest_path = solve_g10_stage(
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
                "G10_stage_execution_error_fail_closed"
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
                "file": str((stage_directory / "certificate.json").relative_to(destination)),
                "file_sha256": file_sha256(stage_directory / "certificate.json"),
            }
            _save_manifest(manifest_path, schedule_manifest)
            return manifest_path
        _save_manifest(manifest_path, schedule_manifest)
    schedule_manifest["status"] = "not_ready"
    schedule_manifest["terminal_reason"] = "all_finite_G10_stages_failed_V3_ready_gate"
    _save_manifest(manifest_path, schedule_manifest)
    return manifest_path
