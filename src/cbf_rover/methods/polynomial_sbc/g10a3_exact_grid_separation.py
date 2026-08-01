"""Finite exact-rational grid separation for a global W16 q-SBC.

G10A3 is a candidate-generation fallback.  Its finite grids, selected point
rows, and LP statuses are never formal evidence; one unchanged, subject-bound
V3 validation remains the sole publication authority.
"""

from __future__ import annotations

from collections.abc import Callable, Iterator, Mapping, Sequence
from dataclasses import dataclass
from fractions import Fraction
import gc
import json
from math import isfinite, lcm
from pathlib import Path
from time import monotonic
from typing import Any

import numpy as np
from scipy.sparse import csr_matrix, vstack

from cbf_rover.adapters.training.sac_reference import file_sha256
from cbf_rover.methods.polynomial_sbc.certificate import QSBCertificate
from cbf_rover.methods.polynomial_sbc.chebyshev import (
    chebyshev_coefficients_to_monomial,
    total_degree_chebyshev_basis,
)
from cbf_rover.methods.polynomial_sbc.g10_sparse_multiplier_lp import (
    G10_CONTROLLER_FILE_SHA256,
    G10_CONTROLLER_CONTENT_SHA256,
    G10_EPSBAR,
    G10_NUMERIC_FEASIBILITY_TOLERANCE,
    G10_Q,
    G10_VALIDATOR_ID,
    _maximum_bound_violation,
    _assert_frozen_c2,
    _save_manifest,
    _v3_release_gate,
    _write_json,
)
from cbf_rover.methods.polynomial_sbc.g10a_active_set_lp import (
    G10A_FEEDBACK_NUMERIC_MARGIN,
    G10A_FROZEN_QSBC_FAMILY_COUNT,
    G10A_G9_ROLE,
    _ActiveRowStore,
    _ConstraintFamily,
    _IncrementalOrRebuildSolver,
    _classify_box,
    _constraint_families,
    _g9_ranked_rows,
    _load_g9_seed,
    _v3_feedback_binding_failures,
    _validate_candidate_v3,
)
from cbf_rover.methods.polynomial_sbc.g10a2_exact_preseparation import (
    G10A2ProjectionProblem,
    G10A2_LEXICOGRAPHIC_T_TOLERANCE,
    _exact_candidate_residuals,
    _g9_depth3_geometry_binding,
    _numeric_solve_record,
    _peak_rss_bytes,
    build_g10a2_projection_problem,
)
from cbf_rover.methods.polynomial_sbc.polynomial import (
    FrozenController,
    Power2D,
    SparsePolynomial2D,
    content_sha256,
    controller_max_degree,
    load_frozen_controller,
)
from cbf_rover.methods.polynomial_sbc.rational_certificate_validation import (
    _fraction_from_artifact,
    canonical_rational_model_payload,
)


G10A3_SYNTHESIS_ID = "G10A3_EXACT_RATIONAL_GRID_MULTIPOINT_LP_V1"
G10A3_PROOF_ROUTE_ID = "G10A3_G65_EXACT_GRID_UNCHANGED_V3_V1"
G10A3_CATALOG_ID = "G10A3_FIXED_RATIONAL_TENSOR_GRID_G17_G33_G65_V1"
G10A3_MAX_FULL_V3_CALLS = 1
G10A3_MAX_HARD_POINT_ROWS = 42 * (8 + 16 + 32)
G10A3_CATALOG_GENERATION_POINT_YIELDS = 42 * (17**2 + 33**2 + 65**2)
G10A3_MAX_CANDIDATE_SCAN_POINT_VISITS = 42 * (
    17**2 + 33**2 + 65**2 + 65**2
)
G10A3_MAX_TOTAL_GRID_POINT_YIELDS = (
    G10A3_CATALOG_GENERATION_POINT_YIELDS
    + G10A3_MAX_CANDIDATE_SCAN_POINT_VISITS
)
G10A3_AGGREGATE_LP_TIME_LIMIT_S = 1800.0
G10A3_AGGREGATE_GRID_TIME_LIMIT_S = 3600.0
G10A3_STAGE_WALL_TIME_LIMIT_S = 10800.0
G10A3_MAX_RSS_BYTES = 1024**3
G10A3_MAX_MASTER_LP_ROWS = 3006
G10A3_MAX_EPHEMERAL_LP_ROWS = 3007
G10A3_MAX_CSR_NONZEROS = 400_000
G10A3_MAX_DECISION_VARIABLES = 308


@dataclass(frozen=True, slots=True)
class G10A3StageSpec:
    stage_id: str
    barrier_degree: int
    max_lp_solves: int
    aggregate_lp_time_limit_s: float
    aggregate_grid_time_limit_s: float
    stage_wall_time_limit_s: float

    def __post_init__(self) -> None:
        if self.barrier_degree != 16:
            raise ValueError("G10A3 first slice is frozen to W16")
        if self.max_lp_solves != 8:
            raise ValueError("G10A3 requires four fixed two-phase candidates")
        limits = (
            self.aggregate_lp_time_limit_s,
            self.aggregate_grid_time_limit_s,
            self.stage_wall_time_limit_s,
        )
        if any(not isfinite(value) or value <= 0.0 for value in limits):
            raise ValueError("G10A3 time limits must be positive and finite")


@dataclass(frozen=True, slots=True)
class G10A3GridPassSpec:
    pass_id: str
    nodes_per_axis: int
    strata_per_axis: int
    max_new_points_per_family: int

    def __post_init__(self) -> None:
        if self.nodes_per_axis not in (17, 33, 65):
            raise ValueError("G10A3 grid must be G17, G33, or G65")
        if self.strata_per_axis not in (4, 8):
            raise ValueError("G10A3 strata must be 4x4 or 8x8")
        if (self.nodes_per_axis - 1) % self.strata_per_axis:
            raise ValueError("G10A3 grid intervals must divide into strata")
        if self.max_new_points_per_family <= 0:
            raise ValueError("G10A3 point cap must be positive")


G10A3_STAGE = G10A3StageSpec(
    stage_id="W16_EXACT_GRID_MULTIPOINT",
    barrier_degree=16,
    max_lp_solves=8,
    aggregate_lp_time_limit_s=G10A3_AGGREGATE_LP_TIME_LIMIT_S,
    aggregate_grid_time_limit_s=G10A3_AGGREGATE_GRID_TIME_LIMIT_S,
    stage_wall_time_limit_s=G10A3_STAGE_WALL_TIME_LIMIT_S,
)

G10A3_GRID_PASS_SCHEDULE = (
    G10A3GridPassSpec("G17_S8", 17, 4, 8),
    G10A3GridPassSpec("G33_S16", 33, 8, 16),
    G10A3GridPassSpec("G65_S32", 65, 8, 32),
)


@dataclass(frozen=True, slots=True)
class G10A3GridPoint:
    family_name: str
    nodes_per_axis: int
    strata_per_axis: int
    i: int
    j: int
    stratum_i: int
    stratum_j: int
    point_exact: tuple[Fraction, Fraction]
    domain_class: str
    point_sha256: str

    def payload(self) -> dict[str, Any]:
        return {
            "schema_version": 1,
            "kind": "g10a3_exact_grid_point_identity",
            "catalog_id": G10A3_CATALOG_ID,
            "family_name": self.family_name,
            "nodes_per_axis": self.nodes_per_axis,
            "strata_per_axis": self.strata_per_axis,
            "indices": [self.i, self.j],
            "stratum_indices": [self.stratum_i, self.stratum_j],
            "point_exact": [str(value) for value in self.point_exact],
            "domain_class": self.domain_class,
        }


@dataclass(frozen=True, slots=True)
class G10A3GridCatalog:
    payload: Mapping[str, Any]
    family_contract_sha256: str
    content_sha256: str
    point_counts_by_grid: Mapping[str, int]
    expected_point_set_sha256_by_grid: Mapping[str, str]
    catalog_generation_point_yields: int
    formal_claim_allowed: bool = False


class G10A3GridBudgetExceeded(RuntimeError):
    """Raised before an incomplete grid scan can be treated as a gate."""


@dataclass(frozen=True, slots=True)
class G10A3ExactPointCut:
    family_name: str
    family_contract_sha256: str
    nodes_per_axis: int
    strata_per_axis: int
    i: int
    j: int
    stratum_i: int
    stratum_j: int
    point_exact: tuple[Fraction, Fraction]
    point_sha256: str
    domain_class: str
    pass_id: str
    exact_row_sha256: str
    exact_coefficient_space_candidate_residual: Fraction
    exact_artifact_candidate_residual: Fraction
    exact_artifact_minus_coefficient_space_residual: Fraction
    candidate_numeric_violation_float64_hex: str
    candidate_barrier_sha256: str
    candidate_values_sha256: str
    numeric_margin_exact: Fraction
    constraint_definition_id: str
    selection_event_sha256: str
    rho_coefficient_is_zero: bool = True
    formal_claim_allowed: bool = False

    def payload(self) -> dict[str, Any]:
        return {
            "family_name": self.family_name,
            "family_contract_sha256": self.family_contract_sha256,
            "nodes_per_axis": self.nodes_per_axis,
            "strata_per_axis": self.strata_per_axis,
            "indices": [self.i, self.j],
            "stratum_indices": [self.stratum_i, self.stratum_j],
            "point_exact": [str(value) for value in self.point_exact],
            "point_sha256": self.point_sha256,
            "domain_class": self.domain_class,
            "pass_id": self.pass_id,
            "exact_row_sha256": self.exact_row_sha256,
            "exact_coefficient_space_candidate_residual": str(
                self.exact_coefficient_space_candidate_residual
            ),
            "exact_artifact_candidate_residual": str(
                self.exact_artifact_candidate_residual
            ),
            "exact_artifact_minus_coefficient_space_residual": str(
                self.exact_artifact_minus_coefficient_space_residual
            ),
            "candidate_numeric_violation_float64_hex": (
                self.candidate_numeric_violation_float64_hex
            ),
            "candidate_barrier_sha256": self.candidate_barrier_sha256,
            "candidate_values_sha256": self.candidate_values_sha256,
            "numeric_margin_exact": str(self.numeric_margin_exact),
            "constraint_definition_id": self.constraint_definition_id,
            "selection_event_sha256": self.selection_event_sha256,
            "rho_coefficient_float64_hex": 0.0.hex(),
            "formal_claim_allowed": False,
        }


@dataclass(frozen=True, slots=True)
class G10A3CandidateReplaySubject:
    candidate_barrier_sha256: str
    candidate_values: tuple[float, ...]
    candidate_values_sha256: str
    exact_artifact_residuals: Mapping[
        str,
        Mapping[Power2D, Fraction],
    ]


@dataclass(frozen=True, slots=True)
class G10A3CandidateDiskArtifact:
    path: Path
    file_sha256: str
    content_sha256: str
    candidate_barrier_sha256: str
    candidate_values_sha256: str


@dataclass(frozen=True, slots=True)
class G10A3GridScanResult:
    pass_id: str
    candidate_barrier_sha256: str
    visited_point_count: int
    active_point_count: int
    margin_violating_point_count: int
    coefficient_space_negative_point_count: int
    negative_point_count: int
    minimum_exact_coefficient_space_residual: Fraction | None
    minimum_exact_residual: Fraction | None
    scanned_point_set_sha256: str
    expected_point_set_sha256: str
    point_set_binding_passed: bool
    whole_catalog_traversed: bool
    selected_cuts: tuple[G10A3ExactPointCut, ...]
    gate_passed: bool
    formal_claim_allowed: bool = False


def _g10a3_candidate_values_sha256(values: Sequence[float]) -> str:
    """Bind every binary64 decision value, including rho, t, and z."""

    numeric_values = np.asarray(values, dtype=np.float64)
    if numeric_values.ndim != 1 or not numeric_values.size:
        raise ValueError("G10A3 candidate values must be a nonempty vector")
    if not np.isfinite(numeric_values).all():
        raise ValueError("G10A3 candidate values must be finite")
    return content_sha256(
        {
            "schema_version": 1,
            "kind": "g10a3_full_float64_solver_candidate",
            "decision_variable_count": int(numeric_values.size),
            "values_float64_hex": [
                float(value).hex() for value in numeric_values
            ],
        }
    )


def _grid_stratum(index: int, *, nodes_per_axis: int, strata_per_axis: int) -> int:
    if index == nodes_per_axis - 1:
        return strata_per_axis - 1
    return (index * strata_per_axis) // (nodes_per_axis - 1)


def _g10a3_grid_point_at(
    family: _ConstraintFamily,
    *,
    nodes_per_axis: int,
    strata_per_axis: int,
    i: int,
    j: int,
) -> G10A3GridPoint:
    if nodes_per_axis not in (17, 33, 65):
        raise ValueError("G10A3 grid must be G17, G33, or G65")
    if strata_per_axis not in (4, 8):
        raise ValueError("G10A3 strata must be 4x4 or 8x8")
    intervals = nodes_per_axis - 1
    if intervals % strata_per_axis:
        raise ValueError("G10A3 grid intervals must divide into strata")
    if not 0 <= i < nodes_per_axis or not 0 <= j < nodes_per_axis:
        raise ValueError("G10A3 grid indices are out of range")
    x_low, x_high, y_low, y_high = family.root_box_exact
    x_value = x_low + (x_high - x_low) * Fraction(i, intervals)
    y_value = y_low + (y_high - y_low) * Fraction(j, intervals)
    stratum_i = _grid_stratum(
        i,
        nodes_per_axis=nodes_per_axis,
        strata_per_axis=strata_per_axis,
    )
    stratum_j = _grid_stratum(
        j,
        nodes_per_axis=nodes_per_axis,
        strata_per_axis=strata_per_axis,
    )
    domain_class = _classify_box(
        family,
        (x_value, x_value, y_value, y_value),
    )
    point = G10A3GridPoint(
        family_name=family.name,
        nodes_per_axis=nodes_per_axis,
        strata_per_axis=strata_per_axis,
        i=i,
        j=j,
        stratum_i=stratum_i,
        stratum_j=stratum_j,
        point_exact=(x_value, y_value),
        domain_class=domain_class,
        point_sha256="",
    )
    return G10A3GridPoint(
        family_name=point.family_name,
        nodes_per_axis=point.nodes_per_axis,
        strata_per_axis=point.strata_per_axis,
        i=point.i,
        j=point.j,
        stratum_i=point.stratum_i,
        stratum_j=point.stratum_j,
        point_exact=point.point_exact,
        domain_class=point.domain_class,
        point_sha256=content_sha256(point.payload()),
    )


def iter_g10a3_grid_points(
    family: _ConstraintFamily,
    *,
    nodes_per_axis: int,
    strata_per_axis: int,
) -> Iterator[G10A3GridPoint]:
    """Yield one canonical exact tensor grid without materializing the catalog."""

    if nodes_per_axis not in (17, 33, 65):
        raise ValueError("G10A3 grid must be G17, G33, or G65")
    if strata_per_axis not in (4, 8):
        raise ValueError("G10A3 strata must be 4x4 or 8x8")
    intervals = nodes_per_axis - 1
    if intervals % strata_per_axis:
        raise ValueError("G10A3 grid intervals must divide into strata")
    for i in range(nodes_per_axis):
        for j in range(nodes_per_axis):
            yield _g10a3_grid_point_at(
                family,
                nodes_per_axis=nodes_per_axis,
                strata_per_axis=strata_per_axis,
                i=i,
                j=j,
            )


def g10a3_point_set_sha256(
    *,
    nodes_per_axis: int,
    point_identity_sha256s: Sequence[str],
) -> str:
    return content_sha256(
        {
            "schema_version": 1,
            "kind": "g10a3_exact_grid_point_set",
            "catalog_id": G10A3_CATALOG_ID,
            "nodes_per_axis": nodes_per_axis,
            "point_identity_sha256s": list(point_identity_sha256s),
        }
    )


def _validate_sha256(value: str, *, name: str) -> None:
    if len(value) != 64 or any(character not in "0123456789abcdef" for character in value):
        raise ValueError(f"G10A3 {name} must be a lowercase SHA-256")


def _evaluate_exact_polynomial(
    polynomial: Mapping[tuple[int, int], Fraction],
    point: tuple[Fraction, Fraction],
) -> Fraction:
    x_value, y_value = point
    return sum(
        (
            coefficient * x_value**x_power * y_value**y_power
            for (x_power, y_power), coefficient in polynomial.items()
        ),
        Fraction(0),
    )


@dataclass(frozen=True, slots=True)
class _ScaledExactGridPolynomialValues:
    denominator: int
    numerators: tuple[tuple[int, ...], ...]

    def fraction_at(self, i: int, j: int) -> Fraction:
        return Fraction(self.numerators[i][j], self.denominator)


def _scaled_exact_grid_polynomial_values(
    polynomial: Mapping[tuple[int, int], Fraction],
    *,
    x_values: Sequence[Fraction],
    y_values: Sequence[Fraction],
) -> _ScaledExactGridPolynomialValues:
    """Evaluate a rational polynomial on a tensor grid with integer Horner steps."""

    if not x_values or not y_values:
        raise ValueError("G10A3 scaled exact grid requires nonempty axes")
    if any(
        x_power < 0 or y_power < 0
        for x_power, y_power in polynomial
    ):
        raise ValueError("G10A3 exact polynomial powers must be nonnegative")
    x_degree = max((power[0] for power in polynomial), default=0)
    y_degree = max((power[1] for power in polynomial), default=0)
    x_denominator = lcm(*(value.denominator for value in x_values))
    y_denominator = lcm(*(value.denominator for value in y_values))
    exact_coefficients = {
        power: Fraction(coefficient)
        for power, coefficient in polynomial.items()
        if coefficient
    }
    coefficient_denominator = lcm(
        *(value.denominator for value in exact_coefficients.values())
    )
    common_denominator = (
        coefficient_denominator
        * x_denominator**x_degree
        * y_denominator**y_degree
    )
    integer_coefficients = [
        [0 for _ in range(x_degree + 1)]
        for _ in range(y_degree + 1)
    ]
    for (x_power, y_power), coefficient in exact_coefficients.items():
        integer_coefficients[y_power][x_power] = (
            coefficient.numerator
            * (coefficient_denominator // coefficient.denominator)
            * x_denominator ** (x_degree - x_power)
            * y_denominator ** (y_degree - y_power)
        )
    x_integers = tuple(
        value.numerator * (x_denominator // value.denominator)
        for value in x_values
    )
    y_integers = tuple(
        value.numerator * (y_denominator // value.denominator)
        for value in y_values
    )
    rows: list[tuple[int, ...]] = []
    for x_integer in x_integers:
        coefficients_in_y: list[int] = []
        for y_power in range(y_degree + 1):
            value = 0
            for x_power in range(x_degree, -1, -1):
                value = (
                    value * x_integer
                    + integer_coefficients[y_power][x_power]
                )
            coefficients_in_y.append(value)
        row: list[int] = []
        for y_integer in y_integers:
            value = 0
            for y_power in range(y_degree, -1, -1):
                value = value * y_integer + coefficients_in_y[y_power]
            row.append(value)
        rows.append(tuple(row))
    return _ScaledExactGridPolynomialValues(
        denominator=common_denominator,
        numerators=tuple(rows),
    )


def _exact_coefficient_space_residuals(
    *,
    families: Sequence[_ConstraintFamily],
    candidate_chebyshev_coefficients: Sequence[float],
) -> dict[str, Mapping[Power2D, Fraction]]:
    """Reconstruct the exact LP-row residual before float basis conversion."""

    coefficients = tuple(
        _fraction_from_artifact(value)
        for value in candidate_chebyshev_coefficients
    )
    if not coefficients:
        raise ValueError("G10A3 coefficient-space residuals require coefficients")
    residuals: dict[str, Mapping[Power2D, Fraction]] = {}
    for family in families:
        if len(family.basis) != len(coefficients):
            raise ValueError("G10A3 family basis and coefficient counts differ")
        residual: dict[Power2D, Fraction] = {}
        constant = _fraction_from_artifact(family.constant_value)
        if constant:
            residual[(0, 0)] = constant
        for basis_polynomial, coefficient in zip(
            family.basis,
            coefficients,
            strict=True,
        ):
            if not coefficient:
                continue
            for power, basis_coefficient in basis_polynomial.items():
                residual[power] = residual.get(power, Fraction(0)) + (
                    coefficient
                    * _fraction_from_artifact(basis_coefficient)
                )
        residuals[family.name] = {
            power: value for power, value in residual.items() if value
        }
    if len(residuals) != len(families):
        raise ValueError("G10A3 coefficient-space family names are not unique")
    return residuals


def _g10a3_numeric_point_row_replay(
    *,
    exact_row: Sequence[Fraction],
    constant_exact: Fraction,
    candidate_values: Sequence[float],
    variable_count: int,
    rho_index: int,
) -> tuple[np.ndarray, float, float, float]:
    numeric_candidate = np.asarray(candidate_values, dtype=np.float64)
    if (
        numeric_candidate.shape != (variable_count,)
        or len(exact_row) != rho_index
        or variable_count <= rho_index
        or not np.isfinite(numeric_candidate).all()
    ):
        raise ValueError("G10A3 numeric point replay layout is invalid")
    row = np.zeros(variable_count, dtype=np.float64)
    row[:rho_index] = -np.asarray(
        [float(value) for value in exact_row],
        dtype=np.float64,
    )
    row[rho_index:] = 0.0
    rhs = float(constant_exact) - G10A_FEEDBACK_NUMERIC_MARGIN
    left_hand_side = float(row @ numeric_candidate)
    return row, rhs, left_hand_side, left_hand_side - rhs


def _exact_artifact_point_row(
    family: _ConstraintFamily,
    point: tuple[Fraction, Fraction],
) -> tuple[tuple[Fraction, ...], Fraction]:
    row = tuple(
        _evaluate_exact_polynomial(
            {
                power: _fraction_from_artifact(coefficient)
                for power, coefficient in polynomial.items()
            },
            point,
        )
        for polynomial in family.basis
    )
    return row, _fraction_from_artifact(family.constant_value)


def _exact_point_row_sha256(
    *,
    family_name: str,
    point: tuple[Fraction, Fraction],
    row: Sequence[Fraction],
    constant_value: Fraction,
) -> str:
    return content_sha256(
        {
            "schema_version": 1,
            "kind": "g10a3_exact_artifact_point_row",
            "family_name": family_name,
            "point_exact": [str(value) for value in point],
            "basis_values_exact": [str(value) for value in row],
            "constant_value_exact": str(constant_value),
        }
    )


def _point_cut_definition_material(
    *,
    family_contract_sha256: str,
    point: G10A3GridPoint,
    numeric_margin_exact: Fraction,
    exact_row_sha256: str,
) -> dict[str, Any]:
    return {
        "schema_version": 1,
        "kind": "g10a3_exact_point_cut_definition",
        "synthesis_id": G10A3_SYNTHESIS_ID,
        "family_contract_sha256": family_contract_sha256,
        "family_name": point.family_name,
        "point_exact": [str(value) for value in point.point_exact],
        "exact_row_sha256": exact_row_sha256,
        "numeric_margin_exact": str(numeric_margin_exact),
        "rho_coefficient_float64_hex": 0.0.hex(),
    }


def _point_cut_selection_material(
    *,
    definition_id: str,
    candidate_barrier_sha256: str,
    candidate_values_sha256: str,
    exact_coefficient_space_candidate_residual: Fraction,
    exact_artifact_candidate_residual: Fraction,
    exact_artifact_minus_coefficient_space_residual: Fraction,
    candidate_numeric_violation_float64_hex: str,
    pass_id: str,
    point_sha256: str,
) -> dict[str, Any]:
    return {
        "schema_version": 1,
        "kind": "g10a3_exact_point_cut_selection_event",
        "constraint_definition_id": definition_id,
        "candidate_barrier_sha256": candidate_barrier_sha256,
        "candidate_values_sha256": candidate_values_sha256,
        "exact_coefficient_space_candidate_residual": str(
            exact_coefficient_space_candidate_residual
        ),
        "exact_artifact_candidate_residual": str(
            exact_artifact_candidate_residual
        ),
        "exact_artifact_minus_coefficient_space_residual": str(
            exact_artifact_minus_coefficient_space_residual
        ),
        "candidate_numeric_violation_float64_hex": (
            candidate_numeric_violation_float64_hex
        ),
        "pass_id": pass_id,
        "point_sha256": point_sha256,
    }


def scan_g10a3_exact_grid(
    *,
    families: Sequence[_ConstraintFamily],
    exact_coefficient_space_residuals: Mapping[
        str,
        Mapping[tuple[int, int], Fraction],
    ],
    exact_artifact_residuals: Mapping[
        str,
        Mapping[tuple[int, int], Fraction],
    ],
    candidate_values: Sequence[float],
    candidate_values_sha256: str,
    variable_count: int,
    rho_index: int,
    candidate_barrier_sha256: str,
    family_contract_sha256: str,
    pass_spec: G10A3GridPassSpec,
    expected_point_set_sha256: str,
    existing_constraint_definition_ids: frozenset[str],
    select_points: bool = True,
    deadline: float | None = None,
    clock: Callable[[], float] = monotonic,
) -> G10A3GridScanResult:
    """Traverse a complete exact grid and select one worst new point per stratum."""

    if not families:
        raise ValueError("G10A3 exact scan requires at least one family")
    _validate_sha256(candidate_barrier_sha256, name="candidate barrier hash")
    _validate_sha256(candidate_values_sha256, name="candidate values hash")
    _validate_sha256(family_contract_sha256, name="family contract hash")
    _validate_sha256(expected_point_set_sha256, name="expected point-set hash")
    expected_names = {family.name for family in families}
    if (
        set(exact_coefficient_space_residuals) != expected_names
        or set(exact_artifact_residuals) != expected_names
    ):
        raise ValueError("G10A3 dual exact residual family set mismatch")
    numeric_candidate = np.asarray(candidate_values, dtype=np.float64)
    if (
        numeric_candidate.shape != (variable_count,)
        or variable_count <= rho_index
        or not np.isfinite(numeric_candidate).all()
        or any(len(family.basis) != rho_index for family in families)
    ):
        raise ValueError("G10A3 scan candidate decision vector is invalid")
    if _g10a3_candidate_values_sha256(numeric_candidate) != (
        candidate_values_sha256
    ):
        raise ValueError("G10A3 scan candidate decision-vector hash mismatch")
    if float(numeric_candidate[rho_index]).hex() != 0.0.hex():
        raise ValueError("G10A3 scan candidate rho is not canonical +0.0")
    numeric_coefficients = numeric_candidate[:rho_index]
    margin_exact = _fraction_from_artifact(G10A_FEEDBACK_NUMERIC_MARGIN)
    visited = 0
    active = 0
    margin_violating = 0
    coefficient_space_negative = 0
    negative = 0
    minimum_coefficient_space: Fraction | None = None
    minimum: Fraction | None = None
    scanned_hashes: list[str] = []
    selected: list[G10A3ExactPointCut] = []
    for family in families:
        coefficient_space_polynomial = exact_coefficient_space_residuals[
            family.name
        ]
        artifact_polynomial = exact_artifact_residuals[family.name]
        x_low, x_high, y_low, y_high = family.root_box_exact
        intervals = pass_spec.nodes_per_axis - 1
        x_values = tuple(
            x_low + (x_high - x_low) * Fraction(i, intervals)
            for i in range(pass_spec.nodes_per_axis)
        )
        y_values = tuple(
            y_low + (y_high - y_low) * Fraction(j, intervals)
            for j in range(pass_spec.nodes_per_axis)
        )
        scaled_coefficient_space_values = _scaled_exact_grid_polynomial_values(
            coefficient_space_polynomial,
            x_values=x_values,
            y_values=y_values,
        )
        scaled_artifact_values = _scaled_exact_grid_polynomial_values(
            artifact_polynomial,
            x_values=x_values,
            y_values=y_values,
        )
        family_coefficient_minimum_numerator: int | None = None
        family_artifact_minimum_numerator: int | None = None
        candidates_by_stratum: dict[
            tuple[int, int],
            list[tuple[G10A3GridPoint, int, int]],
        ] = {}
        for point in iter_g10a3_grid_points(
            family,
            nodes_per_axis=pass_spec.nodes_per_axis,
            strata_per_axis=pass_spec.strata_per_axis,
        ):
            if deadline is not None and visited % 64 == 0 and clock() > deadline:
                raise G10A3GridBudgetExceeded(
                    "G10A3 exact grid deadline expired during traversal"
                )
            visited += 1
            scanned_hashes.append(point.point_sha256)
            if point.domain_class == "outside":
                continue
            active += 1
            coefficient_space_numerator = (
                scaled_coefficient_space_values.numerators[point.i][point.j]
            )
            artifact_numerator = scaled_artifact_values.numerators[
                point.i
            ][point.j]
            family_coefficient_minimum_numerator = (
                coefficient_space_numerator
                if family_coefficient_minimum_numerator is None
                else min(
                    family_coefficient_minimum_numerator,
                    coefficient_space_numerator,
                )
            )
            family_artifact_minimum_numerator = (
                artifact_numerator
                if family_artifact_minimum_numerator is None
                else min(
                    family_artifact_minimum_numerator,
                    artifact_numerator,
                )
            )
            if coefficient_space_numerator < 0:
                coefficient_space_negative += 1
            if artifact_numerator < 0:
                negative += 1
            if (
                coefficient_space_numerator * margin_exact.denominator
                >= margin_exact.numerator
                * scaled_coefficient_space_values.denominator
            ):
                continue
            margin_violating += 1
            if not select_points:
                continue
            stratum = (point.stratum_i, point.stratum_j)
            candidates_by_stratum.setdefault(stratum, []).append(
                (point, coefficient_space_numerator, artifact_numerator)
            )
        if family_coefficient_minimum_numerator is not None:
            family_coefficient_minimum = Fraction(
                family_coefficient_minimum_numerator,
                scaled_coefficient_space_values.denominator,
            )
            minimum_coefficient_space = (
                family_coefficient_minimum
                if minimum_coefficient_space is None
                else min(
                    minimum_coefficient_space,
                    family_coefficient_minimum,
                )
            )
        if family_artifact_minimum_numerator is not None:
            family_artifact_minimum = Fraction(
                family_artifact_minimum_numerator,
                scaled_artifact_values.denominator,
            )
            minimum = (
                family_artifact_minimum
                if minimum is None
                else min(minimum, family_artifact_minimum)
            )
        if select_points:
            stratum_winners: list[G10A3ExactPointCut] = []
            for stratum in sorted(candidates_by_stratum):
                candidates = sorted(
                    candidates_by_stratum[stratum],
                    key=lambda item: (item[1], item[0].i, item[0].j),
                )
                for (
                    point,
                    coefficient_space_numerator,
                    artifact_numerator,
                ) in candidates:
                    coefficient_space_residual = Fraction(
                        coefficient_space_numerator,
                        scaled_coefficient_space_values.denominator,
                    )
                    artifact_residual = Fraction(
                        artifact_numerator,
                        scaled_artifact_values.denominator,
                    )
                    conversion_delta = (
                        artifact_residual - coefficient_space_residual
                    )
                    exact_row, constant_exact = _exact_artifact_point_row(
                        family,
                        point.point_exact,
                    )
                    replayed_coefficient_space_residual = (
                        sum(
                            value * _fraction_from_artifact(coefficient)
                            for value, coefficient in zip(
                                exact_row,
                                numeric_coefficients,
                                strict=True,
                            )
                        )
                        + constant_exact
                    )
                    if (
                        replayed_coefficient_space_residual
                        != coefficient_space_residual
                    ):
                        raise ValueError(
                            "G10A3 coefficient-space scan replay mismatch"
                        )
                    _, _, _, numeric_violation = (
                        _g10a3_numeric_point_row_replay(
                            exact_row=exact_row,
                            constant_exact=constant_exact,
                            candidate_values=numeric_candidate,
                            variable_count=variable_count,
                            rho_index=rho_index,
                        )
                    )
                    if (
                        not np.isfinite(numeric_violation)
                        or numeric_violation
                        <= G10_NUMERIC_FEASIBILITY_TOLERANCE
                    ):
                        continue
                    exact_row_hash = _exact_point_row_sha256(
                        family_name=family.name,
                        point=point.point_exact,
                        row=exact_row,
                        constant_value=constant_exact,
                    )
                    definition_material = _point_cut_definition_material(
                        family_contract_sha256=family_contract_sha256,
                        point=point,
                        numeric_margin_exact=margin_exact,
                        exact_row_sha256=exact_row_hash,
                    )
                    definition_id = content_sha256(definition_material)
                    if definition_id in existing_constraint_definition_ids:
                        continue
                    selection_material = _point_cut_selection_material(
                        definition_id=definition_id,
                        candidate_barrier_sha256=candidate_barrier_sha256,
                        candidate_values_sha256=candidate_values_sha256,
                        exact_coefficient_space_candidate_residual=(
                            coefficient_space_residual
                        ),
                        exact_artifact_candidate_residual=artifact_residual,
                        exact_artifact_minus_coefficient_space_residual=(
                            conversion_delta
                        ),
                        candidate_numeric_violation_float64_hex=(
                            numeric_violation.hex()
                        ),
                        pass_id=pass_spec.pass_id,
                        point_sha256=point.point_sha256,
                    )
                    stratum_winners.append(
                        G10A3ExactPointCut(
                            family_name=family.name,
                            family_contract_sha256=family_contract_sha256,
                            nodes_per_axis=point.nodes_per_axis,
                            strata_per_axis=point.strata_per_axis,
                            i=point.i,
                            j=point.j,
                            stratum_i=point.stratum_i,
                            stratum_j=point.stratum_j,
                            point_exact=point.point_exact,
                            point_sha256=point.point_sha256,
                            domain_class=point.domain_class,
                            pass_id=pass_spec.pass_id,
                            exact_row_sha256=exact_row_hash,
                            exact_coefficient_space_candidate_residual=(
                                coefficient_space_residual
                            ),
                            exact_artifact_candidate_residual=(
                                artifact_residual
                            ),
                            exact_artifact_minus_coefficient_space_residual=(
                                conversion_delta
                            ),
                            candidate_numeric_violation_float64_hex=(
                                numeric_violation.hex()
                            ),
                            candidate_barrier_sha256=(
                                candidate_barrier_sha256
                            ),
                            candidate_values_sha256=(
                                candidate_values_sha256
                            ),
                            numeric_margin_exact=margin_exact,
                            constraint_definition_id=definition_id,
                            selection_event_sha256=content_sha256(
                                selection_material
                            ),
                        )
                    )
                    break
            selected.extend(
                sorted(
                    stratum_winners,
                    key=lambda cut: (
                        cut.exact_coefficient_space_candidate_residual,
                        cut.i,
                        cut.j,
                    ),
                )[: pass_spec.max_new_points_per_family]
            )
    scanned_hash = g10a3_point_set_sha256(
        nodes_per_axis=pass_spec.nodes_per_axis,
        point_identity_sha256s=tuple(scanned_hashes),
    )
    binding_passed = scanned_hash == expected_point_set_sha256
    return G10A3GridScanResult(
        pass_id=pass_spec.pass_id,
        candidate_barrier_sha256=candidate_barrier_sha256,
        visited_point_count=visited,
        active_point_count=active,
        margin_violating_point_count=margin_violating,
        coefficient_space_negative_point_count=(
            coefficient_space_negative
        ),
        negative_point_count=negative,
        minimum_exact_coefficient_space_residual=(
            minimum_coefficient_space
        ),
        minimum_exact_residual=minimum,
        scanned_point_set_sha256=scanned_hash,
        expected_point_set_sha256=expected_point_set_sha256,
        point_set_binding_passed=binding_passed,
        whole_catalog_traversed=True,
        selected_cuts=tuple(selected),
        gate_passed=(binding_passed and negative == 0),
    )


def build_g10a3_numeric_point_cut_rows(
    *,
    families: Sequence[_ConstraintFamily],
    cuts: Sequence[G10A3ExactPointCut],
    candidate_barrier_sha256: str,
    candidate_values: Sequence[float],
    candidate_values_sha256: str,
    exact_artifact_residuals: Mapping[
        str,
        Mapping[tuple[int, int], Fraction],
    ],
    variable_count: int,
    rho_index: int,
) -> tuple[csr_matrix, np.ndarray, tuple[Mapping[str, Any], ...]]:
    """Project selected exact point definitions into hard float LP rows."""

    _validate_sha256(candidate_barrier_sha256, name="candidate barrier hash")
    _validate_sha256(candidate_values_sha256, name="candidate values hash")
    numeric_candidate = np.asarray(candidate_values, dtype=np.float64)
    if (
        numeric_candidate.shape != (variable_count,)
        or not np.isfinite(numeric_candidate).all()
    ):
        raise ValueError("G10A3 candidate decision vector is invalid")
    if variable_count <= rho_index:
        raise ValueError("G10A3 variable layout must include fixed rho and suffix")
    if _g10a3_candidate_values_sha256(numeric_candidate) != (
        candidate_values_sha256
    ):
        raise ValueError("G10A3 candidate decision-vector hash mismatch")
    if float(numeric_candidate[rho_index]).hex() != 0.0.hex():
        raise ValueError("G10A3 candidate rho is not canonical +0.0")
    coefficients = numeric_candidate[:rho_index]
    families_by_name = {family.name: family for family in families}
    if len(families_by_name) != len(families):
        raise ValueError("G10A3 numeric rows require unique family names")
    if set(exact_artifact_residuals) != set(families_by_name):
        raise ValueError("G10A3 artifact residual family set mismatch")
    rows: list[np.ndarray] = []
    rhs_values: list[float] = []
    ledger: list[Mapping[str, Any]] = []
    for cut in cuts:
        family = families_by_name.get(cut.family_name)
        if family is None:
            raise ValueError("G10A3 cut references an unknown family")
        if cut.candidate_barrier_sha256 != candidate_barrier_sha256:
            raise ValueError("G10A3 cut candidate hash mismatch")
        if cut.candidate_values_sha256 != candidate_values_sha256:
            raise ValueError("G10A3 cut candidate decision-vector hash mismatch")
        point_box = (
            cut.point_exact[0],
            cut.point_exact[0],
            cut.point_exact[1],
            cut.point_exact[1],
        )
        if _classify_box(family, point_box) != cut.domain_class:
            raise ValueError("G10A3 cut point domain classification mismatch")
        exact_row, constant_exact = _exact_artifact_point_row(
            family,
            cut.point_exact,
        )
        exact_row_hash = _exact_point_row_sha256(
            family_name=family.name,
            point=cut.point_exact,
            row=exact_row,
            constant_value=constant_exact,
        )
        if exact_row_hash != cut.exact_row_sha256:
            raise ValueError("G10A3 exact point-row hash mismatch")
        point = G10A3GridPoint(
            family_name=cut.family_name,
            nodes_per_axis=cut.nodes_per_axis,
            strata_per_axis=cut.strata_per_axis,
            i=cut.i,
            j=cut.j,
            stratum_i=cut.stratum_i,
            stratum_j=cut.stratum_j,
            point_exact=cut.point_exact,
            domain_class=cut.domain_class,
            point_sha256=cut.point_sha256,
        )
        definition_id = content_sha256(
            _point_cut_definition_material(
                family_contract_sha256=cut.family_contract_sha256,
                point=point,
                numeric_margin_exact=cut.numeric_margin_exact,
                exact_row_sha256=exact_row_hash,
            )
        )
        if definition_id != cut.constraint_definition_id:
            raise ValueError("G10A3 point-cut definition hash mismatch")
        selection_id = content_sha256(
            _point_cut_selection_material(
                definition_id=definition_id,
                candidate_barrier_sha256=candidate_barrier_sha256,
                candidate_values_sha256=candidate_values_sha256,
                exact_coefficient_space_candidate_residual=(
                    cut.exact_coefficient_space_candidate_residual
                ),
                exact_artifact_candidate_residual=(
                    cut.exact_artifact_candidate_residual
                ),
                exact_artifact_minus_coefficient_space_residual=(
                    cut.exact_artifact_minus_coefficient_space_residual
                ),
                candidate_numeric_violation_float64_hex=(
                    cut.candidate_numeric_violation_float64_hex
                ),
                pass_id=cut.pass_id,
                point_sha256=cut.point_sha256,
            )
        )
        if selection_id != cut.selection_event_sha256:
            raise ValueError("G10A3 point-cut selection hash mismatch")
        (
            row,
            rhs,
            numeric_row_left_hand_side,
            numeric_violation,
        ) = _g10a3_numeric_point_row_replay(
            exact_row=exact_row,
            constant_exact=constant_exact,
            candidate_values=numeric_candidate,
            variable_count=variable_count,
            rho_index=rho_index,
        )
        candidate_row_replay_exact = (
            sum(
                value * _fraction_from_artifact(coefficient)
                for value, coefficient in zip(exact_row, coefficients, strict=True)
            )
            + constant_exact
        )
        if (
            candidate_row_replay_exact
            != cut.exact_coefficient_space_candidate_residual
        ):
            raise ValueError("G10A3 exact candidate row replay mismatch")
        artifact_row_replay_exact = _evaluate_exact_polynomial(
            exact_artifact_residuals[family.name],
            cut.point_exact,
        )
        conversion_delta_exact = (
            artifact_row_replay_exact - candidate_row_replay_exact
        )
        if (
            artifact_row_replay_exact
            != cut.exact_artifact_candidate_residual
            or conversion_delta_exact
            != cut.exact_artifact_minus_coefficient_space_residual
        ):
            raise ValueError("G10A3 artifact conversion replay mismatch")
        candidate_row_replay_residual = float(candidate_row_replay_exact)
        exact_exclusion_margin = (
            _fraction_from_artifact(G10A_FEEDBACK_NUMERIC_MARGIN)
            - candidate_row_replay_exact
        )
        if numeric_violation.hex() != cut.candidate_numeric_violation_float64_hex:
            raise ValueError("G10A3 numeric candidate row replay mismatch")
        if (
            not np.isfinite(numeric_violation)
            or numeric_violation <= G10_NUMERIC_FEASIBILITY_TOLERANCE
        ):
            raise ValueError(
                "G10A3 numeric point row does not exclude the candidate "
                "outside numeric feasibility tolerance"
            )
        rows.append(row)
        rhs_values.append(rhs)
        ledger.append(
            {
                **cut.payload(),
                "exact_row_sha256": exact_row_hash,
                "numeric_row_float64_hex": [value.hex() for value in row],
                "right_hand_side_float64_hex": rhs.hex(),
                "rho_index": rho_index,
                "variable_count": variable_count,
                "candidate_row_replay_residual": candidate_row_replay_residual,
                "candidate_exact_row_replay_residual": str(
                    candidate_row_replay_exact
                ),
                "candidate_exact_exclusion_margin": str(
                    exact_exclusion_margin
                ),
                "candidate_artifact_replay_residual": float(
                    artifact_row_replay_exact
                ),
                "artifact_minus_coefficient_space_residual": float(
                    conversion_delta_exact
                ),
                "candidate_numeric_row_left_hand_side": (
                    numeric_row_left_hand_side
                ),
                "candidate_numeric_row_left_hand_side_float64_hex": (
                    numeric_row_left_hand_side.hex()
                ),
                "candidate_numeric_violation": numeric_violation,
                "candidate_numeric_violation_float64_hex": (
                    numeric_violation.hex()
                ),
                "rho_t_z_columns_bitwise_zero": all(
                    value.hex() == 0.0.hex() for value in row[rho_index:]
                ),
                "formal_claim_allowed": False,
            }
        )
    if not rows:
        return (
            csr_matrix((0, variable_count), dtype=np.float64),
            np.empty(0, dtype=np.float64),
            (),
        )
    return (
        csr_matrix(np.vstack(rows)),
        np.asarray(rhs_values, dtype=np.float64),
        tuple(ledger),
    )


def _numeric_ledger_rows_sha256(rows: Sequence[Mapping[str, Any]]) -> str:
    return content_sha256(
        {
            "schema_version": 1,
            "kind": "g10a3_selected_numeric_point_rows",
            "rows": [
                {
                    "constraint_definition_id": row.get(
                        "constraint_definition_id"
                    ),
                    "numeric_row_float64_hex": row.get(
                        "numeric_row_float64_hex"
                    ),
                    "right_hand_side_float64_hex": row.get(
                        "right_hand_side_float64_hex"
                    ),
                }
                for row in rows
            ],
        }
    )


def build_g10a3_cut_ledger_payload(
    *,
    grid_catalog_sha256: str,
    rows: Sequence[Mapping[str, Any]],
) -> dict[str, Any]:
    """Build a self-hashed ledger for every selected non-formal point row."""

    _validate_sha256(grid_catalog_sha256, name="grid catalog hash")
    material: dict[str, Any] = {
        "schema_version": 1,
        "kind": "g10a3_exact_selected_point_cut_ledger",
        "synthesis_id": G10A3_SYNTHESIS_ID,
        "grid_catalog_sha256": grid_catalog_sha256,
        "row_count": len(rows),
        "numeric_rows_sha256": _numeric_ledger_rows_sha256(rows),
        "rows": [dict(row) for row in rows],
        "formal_claim_allowed": False,
    }
    material["content_sha256"] = content_sha256(material)
    return material


def validate_g10a3_cut_ledger(
    payload: Mapping[str, Any],
    *,
    expected_grid_catalog_sha256: str,
    families: Sequence[_ConstraintFamily],
    candidate_replay_subjects: Mapping[
        str,
        G10A3CandidateReplaySubject,
    ],
) -> tuple[str, ...]:
    """Reconstruct every ledger row from the expected catalog and families."""

    failures: list[str] = []
    try:
        _validate_sha256(
            expected_grid_catalog_sha256,
            name="expected grid catalog hash",
        )
    except ValueError:
        return ("expected_grid_catalog_sha256",)
    families_by_name = {family.name: family for family in families}
    if not families or len(families_by_name) != len(families):
        return ("frozen_families",)
    expected_family_contract_sha256 = content_sha256(
        _family_contract_payload(families)
    )
    checks = {
        "schema_version": payload.get("schema_version") == 1,
        "kind": payload.get("kind")
        == "g10a3_exact_selected_point_cut_ledger",
        "synthesis_id": payload.get("synthesis_id") == G10A3_SYNTHESIS_ID,
        "grid_catalog_sha256": payload.get("grid_catalog_sha256")
        == expected_grid_catalog_sha256,
        "formal_claim_allowed": payload.get("formal_claim_allowed") is False,
    }
    failures.extend(name for name, passed in checks.items() if not passed)
    rows_value = payload.get("rows")
    rows = list(rows_value) if isinstance(rows_value, list) else []
    if not isinstance(rows_value, list):
        failures.append("rows")
    if payload.get("row_count") != len(rows):
        failures.append("row_count")
    if len(rows) > G10A3_MAX_HARD_POINT_ROWS:
        failures.append("hard_point_row_cap")
    if payload.get("numeric_rows_sha256") != _numeric_ledger_rows_sha256(rows):
        failures.append("numeric_rows_sha256")
    definition_ids: list[str] = []
    for index, row_value in enumerate(rows):
        if not isinstance(row_value, Mapping):
            failures.append(f"row_{index}_mapping")
            continue
        row = row_value
        definition_id = row.get("constraint_definition_id")
        if isinstance(definition_id, str):
            definition_ids.append(definition_id)
        family = families_by_name.get(row.get("family_name"))
        if family is None:
            failures.append(f"row_{index}_frozen_family")
            continue
        if (
            row.get("family_contract_sha256")
            != expected_family_contract_sha256
        ):
            failures.append(f"row_{index}_family_contract_sha256")
        nodes_per_axis = row.get("nodes_per_axis")
        strata_per_axis = row.get("strata_per_axis")
        indices = row.get("indices")
        expected_pass_ids = {
            (17, 4): "G17_S8",
            (33, 8): "G33_S16",
            (65, 8): "G65_S32",
        }
        point: G10A3GridPoint | None = None
        if (
            type(nodes_per_axis) is int
            and type(strata_per_axis) is int
            and isinstance(indices, list)
            and len(indices) == 2
            and all(type(value) is int for value in indices)
            and (nodes_per_axis, strata_per_axis) in expected_pass_ids
        ):
            try:
                point = _g10a3_grid_point_at(
                    family,
                    nodes_per_axis=nodes_per_axis,
                    strata_per_axis=strata_per_axis,
                    i=indices[0],
                    j=indices[1],
                )
            except ValueError:
                point = None
        if point is None:
            failures.append(f"row_{index}_exact_grid_point")
            continue
        expected_point_fields = {
            "family_name": point.family_name,
            "nodes_per_axis": point.nodes_per_axis,
            "strata_per_axis": point.strata_per_axis,
            "indices": [point.i, point.j],
            "stratum_indices": [point.stratum_i, point.stratum_j],
            "point_exact": [str(value) for value in point.point_exact],
            "domain_class": point.domain_class,
            "point_sha256": point.point_sha256,
        }
        if any(
            row.get(name) != value
            for name, value in expected_point_fields.items()
        ):
            failures.append(f"row_{index}_point_sha256")
        if row.get("pass_id") != expected_pass_ids[
            (point.nodes_per_axis, point.strata_per_axis)
        ]:
            failures.append(f"row_{index}_pass_id")
        exact_row, constant_exact = _exact_artifact_point_row(
            family,
            point.point_exact,
        )
        exact_row_sha256 = _exact_point_row_sha256(
            family_name=family.name,
            point=point.point_exact,
            row=exact_row,
            constant_value=constant_exact,
        )
        if row.get("exact_row_sha256") != exact_row_sha256:
            failures.append(f"row_{index}_exact_row_sha256")
        margin_exact = _fraction_from_artifact(G10A_FEEDBACK_NUMERIC_MARGIN)
        if row.get("numeric_margin_exact") != str(margin_exact):
            failures.append(f"row_{index}_numeric_margin_exact")
        definition_material = _point_cut_definition_material(
            family_contract_sha256=expected_family_contract_sha256,
            point=point,
            numeric_margin_exact=margin_exact,
            exact_row_sha256=exact_row_sha256,
        )
        recomputed_definition_id = content_sha256(definition_material)
        if definition_id != recomputed_definition_id:
            failures.append(f"row_{index}_constraint_definition_id")
        exact_residual_fields = (
            "exact_coefficient_space_candidate_residual",
            "exact_artifact_candidate_residual",
            "exact_artifact_minus_coefficient_space_residual",
        )
        parsed_residuals: dict[str, Fraction] = {}
        for field in exact_residual_fields:
            try:
                parsed_residuals[field] = Fraction(row.get(field))
            except (TypeError, ValueError, ZeroDivisionError):
                failures.append(f"row_{index}_{field}")
        exact_coefficient_space_residual = parsed_residuals.get(
            "exact_coefficient_space_candidate_residual"
        )
        exact_artifact_residual = parsed_residuals.get(
            "exact_artifact_candidate_residual"
        )
        exact_conversion_delta = parsed_residuals.get(
            "exact_artifact_minus_coefficient_space_residual"
        )
        if (
            exact_coefficient_space_residual is not None
            and exact_coefficient_space_residual >= margin_exact
        ):
            failures.append(f"row_{index}_selected_residual_margin")
        if (
            exact_coefficient_space_residual is not None
            and exact_artifact_residual is not None
            and exact_conversion_delta is not None
            and exact_conversion_delta
            != exact_artifact_residual - exact_coefficient_space_residual
        ):
            failures.append(f"row_{index}_conversion_residual_delta")
        candidate_barrier_sha256 = row.get("candidate_barrier_sha256")
        if (
            not isinstance(candidate_barrier_sha256, str)
            or len(candidate_barrier_sha256) != 64
            or any(
                character not in "0123456789abcdef"
                for character in candidate_barrier_sha256
            )
        ):
            failures.append(f"row_{index}_candidate_barrier_sha256")
        candidate_values_sha256 = row.get("candidate_values_sha256")
        if (
            not isinstance(candidate_values_sha256, str)
            or len(candidate_values_sha256) != 64
            or any(
                character not in "0123456789abcdef"
                for character in candidate_values_sha256
            )
        ):
            failures.append(f"row_{index}_candidate_values_sha256")
        replay_subject = (
            candidate_replay_subjects.get(candidate_values_sha256)
            if isinstance(candidate_values_sha256, str)
            else None
        )
        replay_values: np.ndarray | None = None
        if replay_subject is not None:
            try:
                replay_values = np.asarray(
                    replay_subject.candidate_values,
                    dtype=np.float64,
                )
            except (TypeError, ValueError):
                replay_values = None
        replay_subject_valid = (
            replay_subject is not None
            and replay_values is not None
            and replay_values.ndim == 1
            and replay_values.size > len(exact_row)
            and np.isfinite(replay_values).all()
            and replay_subject.candidate_barrier_sha256
            == candidate_barrier_sha256
            and replay_subject.candidate_values_sha256
            == candidate_values_sha256
            and _g10a3_candidate_values_sha256(replay_values)
            == candidate_values_sha256
            and family.name in replay_subject.exact_artifact_residuals
        )
        if (
            not replay_subject_valid
        ):
            failures.append(f"row_{index}_candidate_replay_subject")
        else:
            assert replay_subject is not None
            assert replay_values is not None
            replayed_coefficient_space_residual = (
                sum(
                    value * _fraction_from_artifact(coefficient)
                    for value, coefficient in zip(
                        exact_row,
                        replay_values[: len(exact_row)],
                        strict=True,
                    )
                )
                + constant_exact
            )
            replayed_artifact_residual = _evaluate_exact_polynomial(
                replay_subject.exact_artifact_residuals[family.name],
                point.point_exact,
            )
            if (
                exact_coefficient_space_residual
                != replayed_coefficient_space_residual
            ):
                failures.append(
                    f"row_{index}_coefficient_space_subject_replay"
                )
            if exact_artifact_residual != replayed_artifact_residual:
                failures.append(f"row_{index}_artifact_subject_replay")
        selection_material = _point_cut_selection_material(
            definition_id=recomputed_definition_id,
            candidate_barrier_sha256=candidate_barrier_sha256,
            candidate_values_sha256=candidate_values_sha256,
            exact_coefficient_space_candidate_residual=(
                exact_coefficient_space_residual
                if exact_coefficient_space_residual is not None
                else Fraction(0)
            ),
            exact_artifact_candidate_residual=(
                exact_artifact_residual
                if exact_artifact_residual is not None
                else Fraction(0)
            ),
            exact_artifact_minus_coefficient_space_residual=(
                exact_conversion_delta
                if exact_conversion_delta is not None
                else Fraction(0)
            ),
            candidate_numeric_violation_float64_hex=row.get(
                "candidate_numeric_violation_float64_hex"
            ),
            pass_id=row.get("pass_id"),
            point_sha256=point.point_sha256,
        )
        if row.get("selection_event_sha256") != content_sha256(
            selection_material
        ):
            failures.append(f"row_{index}_selection_event_sha256")
        numeric_row = row.get("numeric_row_float64_hex")
        rho_index = row.get("rho_index")
        variable_count = row.get("variable_count")
        if (
            not isinstance(numeric_row, list)
            or not isinstance(rho_index, int)
            or not isinstance(variable_count, int)
            or len(numeric_row) != variable_count
            or not 0 <= rho_index < variable_count
            or rho_index != len(exact_row)
            or any(value != 0.0.hex() for value in numeric_row[rho_index:])
            or row.get("rho_t_z_columns_bitwise_zero") is not True
        ):
            failures.append(f"row_{index}_zero_solver_suffix")
        if isinstance(numeric_row, list) and type(rho_index) is int:
            expected_prefix = [(-float(value)).hex() for value in exact_row]
            if numeric_row[:rho_index] != expected_prefix:
                failures.append(f"row_{index}_numeric_row_prefix")
        expected_rhs = (
            float(constant_exact) - G10A_FEEDBACK_NUMERIC_MARGIN
        ).hex()
        if row.get("right_hand_side_float64_hex") != expected_rhs:
            failures.append(f"row_{index}_right_hand_side")
        if row.get("rho_coefficient_float64_hex") != 0.0.hex():
            failures.append(f"row_{index}_rho_coefficient")
        if exact_coefficient_space_residual is not None:
            expected_replay = float(exact_coefficient_space_residual)
            expected_exact_exclusion = (
                margin_exact - exact_coefficient_space_residual
            )
            if row.get("candidate_row_replay_residual") != expected_replay:
                failures.append(f"row_{index}_candidate_replay_residual")
            if row.get("candidate_exact_row_replay_residual") != str(
                exact_coefficient_space_residual
            ):
                failures.append(f"row_{index}_candidate_exact_replay")
            if row.get("candidate_exact_exclusion_margin") != str(
                expected_exact_exclusion
            ):
                failures.append(f"row_{index}_candidate_exact_exclusion")
        if exact_artifact_residual is not None and row.get(
            "candidate_artifact_replay_residual"
        ) != float(exact_artifact_residual):
            failures.append(f"row_{index}_candidate_artifact_replay_residual")
        if exact_conversion_delta is not None and row.get(
            "artifact_minus_coefficient_space_residual"
        ) != float(exact_conversion_delta):
            failures.append(f"row_{index}_conversion_replay_residual")
        if (
            replay_subject_valid
            and replay_values is not None
            and isinstance(numeric_row, list)
            and type(variable_count) is int
            and type(rho_index) is int
            and len(numeric_row) == variable_count
        ):
            try:
                numeric_values = np.asarray(
                    [float.fromhex(value) for value in numeric_row],
                    dtype=np.float64,
                )
                if replay_values.shape != (variable_count,):
                    raise ValueError("candidate decision-vector size mismatch")
                if float(replay_values[rho_index]).hex() != 0.0.hex():
                    raise ValueError("candidate rho is not canonical +0.0")
                numeric_rhs = float.fromhex(
                    row.get("right_hand_side_float64_hex")
                )
                numeric_lhs = float(numeric_values @ replay_values)
                numeric_violation = numeric_lhs - numeric_rhs
            except (TypeError, ValueError):
                failures.append(f"row_{index}_numeric_candidate_replay")
            else:
                numeric_checks = {
                    "candidate_numeric_row_left_hand_side": numeric_lhs,
                    "candidate_numeric_row_left_hand_side_float64_hex": (
                        numeric_lhs.hex()
                    ),
                    "candidate_numeric_violation": numeric_violation,
                    "candidate_numeric_violation_float64_hex": (
                        numeric_violation.hex()
                    ),
                }
                if any(
                    row.get(name) != value
                    for name, value in numeric_checks.items()
                ):
                    failures.append(f"row_{index}_numeric_candidate_replay")
                if (
                    not np.isfinite(numeric_violation)
                    or numeric_violation
                    <= G10_NUMERIC_FEASIBILITY_TOLERANCE
                ):
                    failures.append(f"row_{index}_numeric_candidate_exclusion")
        if row.get("formal_claim_allowed") is not False:
            failures.append(f"row_{index}_formal_claim_allowed")
    if len(definition_ids) != len(set(definition_ids)):
        failures.append("duplicate_constraint_definition_id")
    material = dict(payload)
    declared_hash = material.pop("content_sha256", None)
    if declared_hash != content_sha256(material):
        failures.append("content_sha256")
    return tuple(dict.fromkeys(failures))


def _family_target_degrees(family: _ConstraintFamily) -> tuple[int, int]:
    powers = {power for polynomial in family.basis for power in polynomial}
    if not powers:
        raise ValueError("G10A3 family basis must not be empty")
    return max(power[0] for power in powers), max(power[1] for power in powers)


def _normalized_float64_hex(value: float) -> str:
    numeric_value = float(value)
    if not np.isfinite(numeric_value):
        raise ValueError("G10A3 family coefficients must be finite")
    if numeric_value == 0.0:
        numeric_value = 0.0
    return numeric_value.hex()


def _family_basis_payload(family: _ConstraintFamily) -> list[dict[str, Any]]:
    return [
        {
            "basis_index": basis_index,
            "terms": [
                {
                    "power": [power[0], power[1]],
                    "coefficient_float64_hex": _normalized_float64_hex(
                        coefficient
                    ),
                }
                for power, coefficient in sorted(polynomial.items())
            ],
        }
        for basis_index, polynomial in enumerate(family.basis)
    ]


def _family_contract_payload(
    families: Sequence[_ConstraintFamily],
) -> dict[str, Any]:
    return {
        "schema_version": 2,
        "kind": "g10a3_frozen_qsbc_family_contract",
        "synthesis_id": G10A3_SYNTHESIS_ID,
        "formal_model_contract_sha256": content_sha256(
            canonical_rational_model_payload()
        ),
        "controller_content_sha256": G10_CONTROLLER_CONTENT_SHA256,
        "family_count": len(families),
        "families": [
            {
                "name": family.name,
                "root_box_exact": [
                    str(value) for value in family.root_box_exact
                ],
                "domain_kind": family.domain_kind,
                "piece_id": family.piece_id,
                "constant_float64_hex": _normalized_float64_hex(
                    family.constant_value
                ),
                "target_degrees": list(_family_target_degrees(family)),
                "basis_polynomial_count": len(family.basis),
                "normalized_full_basis": _family_basis_payload(family),
            }
            for family in families
        ],
    }


def build_g10a3_grid_catalog(
    families: Sequence[_ConstraintFamily],
) -> G10A3GridCatalog:
    """Hash the complete fixed G17/G33/G65 geometry without storing all points."""

    if not families:
        raise ValueError("G10A3 grid catalog requires at least one family")
    if len({family.name for family in families}) != len(families):
        raise ValueError("G10A3 grid catalog family names must be unique")
    family_contract = _family_contract_payload(families)
    family_contract_hash = content_sha256(family_contract)
    counts: dict[str, int] = {}
    expected_hashes: dict[str, str] = {}
    for pass_spec in G10A3_GRID_PASS_SCHEDULE:
        point_hashes = tuple(
            point.point_sha256
            for family in families
            for point in iter_g10a3_grid_points(
                family,
                nodes_per_axis=pass_spec.nodes_per_axis,
                strata_per_axis=pass_spec.strata_per_axis,
            )
        )
        key = str(pass_spec.nodes_per_axis)
        counts[key] = len(point_hashes)
        expected_hashes[key] = g10a3_point_set_sha256(
            nodes_per_axis=pass_spec.nodes_per_axis,
            point_identity_sha256s=point_hashes,
        )
    root_set = {
        "schema_version": 1,
        "kind": "g10a3_v3_qsbc_root_set",
        "roots": {
            family.name: [str(value) for value in family.root_box_exact]
            for family in families
        },
    }
    payload: dict[str, Any] = {
        "schema_version": 1,
        "kind": "g10a3_fixed_exact_rational_grid_catalog",
        "catalog_id": G10A3_CATALOG_ID,
        "grids": [17, 33, 65],
        "strata_per_axis_by_grid": {"17": 4, "33": 8, "65": 8},
        "family_contract": family_contract,
        "family_contract_sha256": family_contract_hash,
        "v3_root_set": root_set,
        "v3_root_set_sha256": content_sha256(root_set),
        "point_counts_by_grid": counts,
        "catalog_generation_point_yields": sum(counts.values()),
        "expected_point_set_sha256_by_grid": expected_hashes,
        "point_payloads_embedded": False,
        "formal_claim_allowed": False,
    }
    catalog_hash = content_sha256(payload)
    payload["catalog_content_sha256"] = catalog_hash
    return G10A3GridCatalog(
        payload=payload,
        family_contract_sha256=family_contract_hash,
        content_sha256=catalog_hash,
        point_counts_by_grid=counts,
        expected_point_set_sha256_by_grid=expected_hashes,
        catalog_generation_point_yields=sum(counts.values()),
    )


def g10a3_resource_failures(
    matrix: csr_matrix,
    *,
    rss_bytes: int | None,
    ephemeral_lexicographic_row: bool = False,
) -> tuple[str, ...]:
    """Return every failed hard resource cap for one concrete LP matrix."""

    row_limit = (
        G10A3_MAX_EPHEMERAL_LP_ROWS
        if ephemeral_lexicographic_row
        else G10A3_MAX_MASTER_LP_ROWS
    )
    checks = {
        "decision_variable_cap": matrix.shape[1] <= G10A3_MAX_DECISION_VARIABLES,
        "linear_row_cap": matrix.shape[0] <= row_limit,
        "CSR_nonzero_cap": matrix.nnz <= G10A3_MAX_CSR_NONZEROS,
        "RSS_measurement_available": rss_bytes is not None,
        "RSS_cap": rss_bytes is not None and rss_bytes <= G10A3_MAX_RSS_BYTES,
    }
    return tuple(name for name, passed in checks.items() if not passed)


def g10a3_publication_gate(
    *,
    final_grid_gate_passed: bool,
    complete_grid_visit_count: int,
    expected_grid_visit_count: int,
    point_set_binding_passed: bool,
    cut_ledger_passed: bool,
    resource_gate_passed: bool,
    full_v3_call_count: int,
    validation_status: str | None,
    global_polynomial_status: str | None,
    first_slice_ready: bool | None,
    controller_contract_status: str | None,
    geometry_contract_status: str | None,
    theorem_scalar_gate: bool | None,
    formal_acceptance_tolerance: float | None,
    binding_failures: Sequence[str],
    v3_release_gate_passed: bool,
) -> tuple[bool, tuple[str, ...]]:
    """Apply every G10A3-only gate around the unchanged V3 release gate."""

    checks = {
        "final_exact_grid_gate_passed": final_grid_gate_passed is True,
        "complete_final_grid_visit_count": (
            complete_grid_visit_count == expected_grid_visit_count
            and expected_grid_visit_count > 0
        ),
        "final_grid_point_set_binding": point_set_binding_passed is True,
        "cut_ledger_binding": cut_ledger_passed is True,
        "hard_resource_gate": resource_gate_passed is True,
        "exactly_one_full_v3_call": full_v3_call_count == G10A3_MAX_FULL_V3_CALLS,
        "V3_status": validation_status == "validated",
        "V3_global_polynomial_status": global_polynomial_status == "validated",
        "V3_first_slice_ready": first_slice_ready is True,
        "V3_controller_contract": controller_contract_status == "validated",
        "V3_geometry_contract": geometry_contract_status == "validated",
        "V3_theorem_scalar_gate": theorem_scalar_gate is True,
        "V3_zero_formal_acceptance_tolerance": formal_acceptance_tolerance == 0.0,
        "V3_subject_and_tree_bindings": not binding_failures,
        "unchanged_V3_release_gate": v3_release_gate_passed is True,
    }
    failures = tuple(name for name, passed in checks.items() if not passed)
    return not failures, failures


def _record_g10a3_resource_sample(
    *,
    manifest: dict[str, Any],
    label: str,
    matrix: csr_matrix,
    ephemeral_lexicographic_row: bool = False,
) -> None:
    rss_bytes = _peak_rss_bytes()
    failures = g10a3_resource_failures(
        matrix,
        rss_bytes=rss_bytes,
        ephemeral_lexicographic_row=ephemeral_lexicographic_row,
    )
    manifest.setdefault("g10a3_resource_samples", []).append(
        {
            "label": label,
            "rss_bytes": rss_bytes,
            "rss_limit_bytes": G10A3_MAX_RSS_BYTES,
            "matrix_row_count": int(matrix.shape[0]),
            "matrix_column_count": int(matrix.shape[1]),
            "matrix_nonzero_count": int(matrix.nnz),
            "failures": list(failures),
        }
    )
    if failures:
        raise G10A3GridBudgetExceeded(
            f"G10A3 hard resource gate failed at {label}: {failures}"
        )


def _check_g10a3_stage_deadline(
    *,
    stage_deadline: float,
    clock: Callable[[], float] = monotonic,
) -> None:
    if clock() > stage_deadline:
        raise G10A3GridBudgetExceeded(
            "G10A3 stage wall-time budget expired; current subject discarded"
        )


def _solve_g10a3_two_phase_projection(
    *,
    projection: G10A2ProjectionProblem,
    master_matrix: csr_matrix,
    master_right_hand_side: np.ndarray,
    seed_values: np.ndarray,
    candidate_index: int,
    first_solve_index: int,
    cumulative_lp_time_s: float,
    stage_deadline: float,
    phase_records: list[dict[str, Any]],
    manifest: dict[str, Any],
    clock: Callable[[], float] = monotonic,
) -> dict[str, Any]:
    """Solve one G10A3 pair under G10A3 labels and resource caps."""

    if master_matrix.shape[0] != master_right_hand_side.size:
        raise ValueError("G10A3 master matrix/right-hand-side mismatch")
    if master_matrix.shape[1] != projection.variable_count:
        raise ValueError("G10A3 master matrix decision layout mismatch")
    _check_g10a3_stage_deadline(
        stage_deadline=stage_deadline,
        clock=clock,
    )
    _record_g10a3_resource_sample(
        manifest=manifest,
        label=f"g10a3_candidate_{candidate_index:02d}_primary",
        matrix=master_matrix,
    )
    lex_row = np.zeros(projection.variable_count, dtype=np.float64)
    lex_row[projection.t_index] = 1.0
    secondary_matrix = vstack(
        (master_matrix, csr_matrix(lex_row.reshape(1, -1))),
        format="csr",
    )
    _record_g10a3_resource_sample(
        manifest=manifest,
        label=(
            f"g10a3_candidate_{candidate_index:02d}_"
            "secondary_ephemeral_lexicographic"
        ),
        matrix=secondary_matrix,
        ephemeral_lexicographic_row=True,
    )

    remaining_lp_s = (
        G10A3_STAGE.aggregate_lp_time_limit_s - cumulative_lp_time_s
    )
    remaining_wall_s = stage_deadline - clock()
    delegated_primary_s = min(remaining_lp_s, remaining_wall_s)
    if delegated_primary_s <= 0.0:
        raise G10A3GridBudgetExceeded(
            "G10A3 aggregate LP or stage wall-time budget expired before "
            "primary"
        )
    primary_solver = _IncrementalOrRebuildSolver(
        matrix=master_matrix,
        right_hand_side=master_right_hand_side,
        objective=projection.primary_objective,
        bounds=projection.bounds,
        seed_values=seed_values,
    )
    primary = primary_solver.solve(time_limit_s=delegated_primary_s)
    cumulative_lp_time_s += primary.solve_time_s
    phase_records.append(
        _numeric_solve_record(
            solve_index=first_solve_index,
            candidate_index=candidate_index,
            phase="primary_min_Linf_seed_distance",
            result=primary,
            matrix=master_matrix,
            cumulative_lp_time_s=cumulative_lp_time_s,
        )
    )
    _check_g10a3_stage_deadline(
        stage_deadline=stage_deadline,
        clock=clock,
    )
    if (
        primary.values is None
        or not primary.success
        or primary.aggregate_budget_exceeded
        or cumulative_lp_time_s > G10A3_STAGE.aggregate_lp_time_limit_s
    ):
        raise RuntimeError(
            "G10A3 primary projection LP produced no accepted vector"
        )
    primary_values = np.asarray(primary.values, dtype=np.float64)
    primary_minimum_slack = float(
        np.min(master_right_hand_side - master_matrix @ primary_values)
    )
    primary_bound_violation = _maximum_bound_violation(
        primary_values,
        projection.bounds,
    )
    if (
        not np.isfinite(primary_values).all()
        or float(primary_values[projection.rho_index]).hex() != 0.0.hex()
        or primary_minimum_slack < -G10_NUMERIC_FEASIBILITY_TOLERANCE
        or primary_bound_violation > G10_NUMERIC_FEASIBILITY_TOLERANCE
    ):
        raise RuntimeError(
            "G10A3 primary projection vector failed numeric replay"
        )
    phase_records[-1]["minimum_master_slack"] = primary_minimum_slack
    phase_records[-1]["maximum_bound_violation"] = primary_bound_violation
    t_primary = float(primary_values[projection.t_index])
    t_upper = float(
        np.nextafter(t_primary, np.inf)
        + G10A2_LEXICOGRAPHIC_T_TOLERANCE
    )
    secondary_rhs = np.concatenate(
        (master_right_hand_side, np.asarray([t_upper], dtype=np.float64))
    )
    remaining_lp_s = (
        G10A3_STAGE.aggregate_lp_time_limit_s - cumulative_lp_time_s
    )
    remaining_wall_s = stage_deadline - clock()
    delegated_secondary_s = min(remaining_lp_s, remaining_wall_s)
    if delegated_secondary_s <= 0.0:
        raise G10A3GridBudgetExceeded(
            "G10A3 aggregate LP or stage wall-time budget expired before "
            "secondary"
        )
    secondary_solver = _IncrementalOrRebuildSolver(
        matrix=secondary_matrix,
        right_hand_side=secondary_rhs,
        objective=projection.secondary_objective,
        bounds=projection.bounds,
        seed_values=primary_values,
    )
    secondary = secondary_solver.solve(time_limit_s=delegated_secondary_s)
    cumulative_lp_time_s += secondary.solve_time_s
    phase_records.append(
        _numeric_solve_record(
            solve_index=first_solve_index + 1,
            candidate_index=candidate_index,
            phase="secondary_min_L1_with_frozen_Linf",
            result=secondary,
            matrix=secondary_matrix,
            cumulative_lp_time_s=cumulative_lp_time_s,
        )
    )
    _record_g10a3_resource_sample(
        manifest=manifest,
        label=(
            f"g10a3_candidate_{candidate_index:02d}_"
            "secondary_ephemeral_complete"
        ),
        matrix=secondary_matrix,
        ephemeral_lexicographic_row=True,
    )
    _check_g10a3_stage_deadline(
        stage_deadline=stage_deadline,
        clock=clock,
    )
    if (
        secondary.values is None
        or not secondary.success
        or secondary.aggregate_budget_exceeded
        or cumulative_lp_time_s > G10A3_STAGE.aggregate_lp_time_limit_s
    ):
        raise RuntimeError(
            "G10A3 secondary projection LP produced no accepted vector"
        )
    values = np.asarray(secondary.values, dtype=np.float64).copy()
    if not np.isfinite(values).all():
        raise ValueError("G10A3 secondary projection vector is non-finite")
    if float(values[projection.rho_index]).hex() != 0.0.hex():
        raise ValueError("G10A3 fixed rho dummy is not canonical +0.0")
    values[projection.rho_index] = 0.0
    minimum_slack = float(np.min(secondary_rhs - secondary_matrix @ values))
    maximum_bound_violation = _maximum_bound_violation(
        values,
        projection.bounds,
    )
    if (
        minimum_slack < -G10_NUMERIC_FEASIBILITY_TOLERANCE
        or maximum_bound_violation > G10_NUMERIC_FEASIBILITY_TOLERANCE
        or values[projection.t_index]
        > t_upper + G10_NUMERIC_FEASIBILITY_TOLERANCE
    ):
        raise RuntimeError("G10A3 projected candidate failed numeric replay")
    return {
        "values": values,
        "primary": primary,
        "secondary": secondary,
        "t_primary": t_primary,
        "t_upper": t_upper,
        "minimum_slack": minimum_slack,
        "maximum_bound_violation": maximum_bound_violation,
        "l1_value": float(np.sum(values[projection.z_start_index :])),
        "cumulative_lp_time_s": cumulative_lp_time_s,
    }


def _candidate_payload(
    *,
    candidate_index: int,
    source_pass_id: str,
    chebyshev_indices: Sequence[Power2D],
    chebyshev_coefficients: Sequence[float],
    barrier: SparsePolynomial2D,
    controller: FrozenController,
    solve: Mapping[str, Any],
    projection_rho_index: int,
    projection_t_index: int,
) -> dict[str, Any]:
    values = np.asarray(solve["values"], dtype=np.float64)
    if (
        values.ndim != 1
        or values.size <= projection_t_index
        or len(chebyshev_coefficients) != projection_rho_index
        or not np.isfinite(values).all()
        or any(
            float(left).hex() != float(right).hex()
            for left, right in zip(
                values[:projection_rho_index],
                chebyshev_coefficients,
                strict=True,
            )
        )
    ):
        raise ValueError("G10A3 candidate payload decision layout mismatch")
    candidate_values_sha256 = _g10a3_candidate_values_sha256(values)
    payload: dict[str, Any] = {
        "schema_version": 1,
        "kind": "g10a3_qsbc_numeric_candidate",
        "synthesis_id": G10A3_SYNTHESIS_ID,
        "proof_route_id": G10A3_PROOF_ROUTE_ID,
        "candidate_index": candidate_index,
        "source_pass_id": source_pass_id,
        "controller_content_sha256": controller.sha256,
        "chebyshev_barrier": {
            "basis": "total_degree_tensor_chebyshev",
            "indices": [list(index) for index in chebyshev_indices],
            "coefficients_decimal": list(chebyshev_coefficients),
            "coefficients_float64_hex": [
                float(value).hex() for value in chebyshev_coefficients
            ],
        },
        "barrier": barrier.payload(),
        "barrier_payload_sha256": content_sha256(barrier.payload()),
        "full_solver_candidate": {
            "decision_layout": "[Chebyshev_c,rho,t,L1_z]",
            "decision_variable_count": int(values.size),
            "values_decimal": [float(value) for value in values],
            "values_float64_hex": [float(value).hex() for value in values],
            "values_sha256": candidate_values_sha256,
        },
        "q_float64_hex": G10_Q.hex(),
        "epsbar_float64_hex": G10_EPSBAR.hex(),
        "rho": 0.0,
        "rho_float64_hex": 0.0.hex(),
        "projection_objective": {
            "primary": "min_max_abs_chebyshev_distance_to_G9_seed",
            "secondary": "min_sum_abs_chebyshev_distance_to_G9_seed",
            "t_primary_optimum": float(solve["t_primary"]),
            "t_secondary_value": float(values[projection_t_index]),
            "t_secondary_upper_bound": float(solve["t_upper"]),
            "lexicographic_t_tolerance": G10A2_LEXICOGRAPHIC_T_TOLERANCE,
            "l1_secondary_value": float(solve["l1_value"]),
        },
        "minimum_active_lp_slack": float(solve["minimum_slack"]),
        "maximum_bound_violation": float(solve["maximum_bound_violation"]),
        "primary_solver_backend": solve["primary"].backend,
        "primary_solver_status": solve["primary"].backend_status,
        "secondary_solver_backend": solve["secondary"].backend,
        "secondary_solver_status": solve["secondary"].backend_status,
        "formal_claim_allowed_before_unique_full_V3": False,
    }
    payload["content_sha256"] = content_sha256(payload)
    return payload


def _grid_scan_payload(result: G10A3GridScanResult) -> dict[str, Any]:
    payload: dict[str, Any] = {
        "schema_version": 1,
        "kind": "g10a3_exact_rational_grid_scan",
        "synthesis_id": G10A3_SYNTHESIS_ID,
        "pass_id": result.pass_id,
        "candidate_barrier_sha256": result.candidate_barrier_sha256,
        "visited_point_count": result.visited_point_count,
        "active_point_count": result.active_point_count,
        "selection_residual_semantics": "exact_coefficient_space_row_c_plus_constant",
        "formal_gate_residual_semantics": "exact_binary64_barrier_artifact",
        "coefficient_space_margin_violating_point_count": (
            result.margin_violating_point_count
        ),
        "coefficient_space_negative_point_count": (
            result.coefficient_space_negative_point_count
        ),
        "artifact_negative_point_count": result.negative_point_count,
        "minimum_exact_coefficient_space_residual": (
            None
            if result.minimum_exact_coefficient_space_residual is None
            else str(result.minimum_exact_coefficient_space_residual)
        ),
        "minimum_exact_artifact_residual": (
            None
            if result.minimum_exact_residual is None
            else str(result.minimum_exact_residual)
        ),
        "scanned_point_set_sha256": result.scanned_point_set_sha256,
        "expected_point_set_sha256": result.expected_point_set_sha256,
        "point_set_binding_passed": result.point_set_binding_passed,
        "whole_catalog_traversed": result.whole_catalog_traversed,
        "selected_cut_count": len(result.selected_cuts),
        "selected_cuts": [cut.payload() for cut in result.selected_cuts],
        "gate_passed": result.gate_passed,
        "formal_claim_allowed": False,
    }
    payload["report_content_sha256"] = content_sha256(payload)
    return payload


def _save_g10a3_cut_ledger(
    path: Path,
    *,
    grid_catalog_sha256: str,
    families: Sequence[_ConstraintFamily],
    candidate_replay_subjects: Mapping[
        str,
        G10A3CandidateReplaySubject,
    ],
    rows: Sequence[Mapping[str, Any]],
) -> dict[str, Any]:
    payload = build_g10a3_cut_ledger_payload(
        grid_catalog_sha256=grid_catalog_sha256,
        rows=rows,
    )
    failures = validate_g10a3_cut_ledger(
        payload,
        expected_grid_catalog_sha256=grid_catalog_sha256,
        families=families,
        candidate_replay_subjects=candidate_replay_subjects,
    )
    if failures:
        raise ValueError(f"G10A3 cut ledger failed self-validation: {failures}")
    _write_json(path, payload)
    return payload


def _load_g10a3_trusted_cut_ledger(
    path: Path,
    *,
    expected_payload: Mapping[str, Any],
    expected_file_sha256: str,
    expected_content_sha256: str,
    expected_row_count: int,
) -> tuple[Mapping[str, Any] | None, tuple[str, ...]]:
    """Reload a ledger without adopting any disk value as trusted state."""

    failures: list[str] = []
    expected_rows = expected_payload.get("rows")
    if (
        type(expected_row_count) is not int
        or expected_row_count < 0
        or type(expected_payload.get("row_count")) is not int
        or expected_payload.get("row_count") != expected_row_count
        or not isinstance(expected_rows, list)
        or len(expected_rows) != expected_row_count
    ):
        failures.append("cut_ledger_trusted_row_count")
    expected_material = dict(expected_payload)
    expected_declared_hash = expected_material.pop("content_sha256", None)
    if (
        expected_declared_hash != expected_content_sha256
        or content_sha256(expected_material) != expected_content_sha256
    ):
        failures.append("cut_ledger_trusted_content")
    try:
        disk_value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError):
        return None, tuple(
            dict.fromkeys([*failures, "cut_ledger_disk_reload"])
        )
    if not isinstance(disk_value, Mapping):
        return None, tuple(
            dict.fromkeys([*failures, "cut_ledger_disk_schema"])
        )
    try:
        disk_file_sha256 = file_sha256(path)
    except OSError:
        failures.append("cut_ledger_file_unreadable")
    else:
        if disk_file_sha256 != expected_file_sha256:
            failures.append("cut_ledger_file_changed")
    if dict(disk_value) != dict(expected_payload):
        failures.append("cut_ledger_subject_changed")
    disk_rows = disk_value.get("rows")
    if (
        type(disk_value.get("row_count")) is not int
        or disk_value.get("row_count") != expected_row_count
        or not isinstance(disk_rows, list)
        or len(disk_rows) != expected_row_count
    ):
        failures.append("cut_ledger_row_count_changed")
    disk_material = dict(disk_value)
    disk_declared_hash = disk_material.pop("content_sha256", None)
    if (
        disk_declared_hash != expected_content_sha256
        or content_sha256(disk_material) != expected_content_sha256
    ):
        failures.append("cut_ledger_content_changed")
    return disk_value, tuple(dict.fromkeys(failures))


def _load_g10a3_candidate_replay_subject_from_disk(
    *,
    artifact: G10A3CandidateDiskArtifact,
    families: Sequence[_ConstraintFamily],
    controller: FrozenController,
) -> tuple[G10A3CandidateReplaySubject | None, tuple[str, ...]]:
    """Reload one trusted candidate artifact and reconstruct its subject."""

    label = artifact.candidate_values_sha256[:12]
    failures: list[str] = []
    for name, value in (
        ("file_sha256", artifact.file_sha256),
        ("content_sha256", artifact.content_sha256),
        ("barrier_sha256", artifact.candidate_barrier_sha256),
        ("values_sha256", artifact.candidate_values_sha256),
    ):
        try:
            _validate_sha256(value, name=f"candidate {name}")
        except ValueError:
            failures.append(f"candidate_{label}_{name}_expected")
    if artifact.path.name != "numeric_candidate.json":
        failures.append(f"candidate_{label}_path_contract")
    try:
        payload = json.loads(artifact.path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError):
        return None, tuple(
            dict.fromkeys(
                [*failures, f"candidate_{label}_disk_reload"]
            )
        )
    if not isinstance(payload, Mapping):
        return None, tuple(
            dict.fromkeys(
                [*failures, f"candidate_{label}_disk_schema"]
            )
        )
    try:
        actual_file_hash = file_sha256(artifact.path)
    except OSError:
        actual_file_hash = None
    if actual_file_hash != artifact.file_sha256:
        failures.append(f"candidate_{label}_file_changed")
    material = dict(payload)
    declared_content_hash = material.pop("content_sha256", None)
    if (
        declared_content_hash != artifact.content_sha256
        or content_sha256(material) != artifact.content_sha256
    ):
        failures.append(f"candidate_{label}_content_changed")
    payload_checks = {
        "schema_version": payload.get("schema_version") == 1,
        "kind": payload.get("kind") == "g10a3_qsbc_numeric_candidate",
        "synthesis_id": payload.get("synthesis_id") == G10A3_SYNTHESIS_ID,
        "proof_route_id": (
            payload.get("proof_route_id") == G10A3_PROOF_ROUTE_ID
        ),
        "controller_content_sha256": (
            payload.get("controller_content_sha256") == controller.sha256
        ),
    }
    failures.extend(
        f"candidate_{label}_{name}"
        for name, passed in payload_checks.items()
        if not passed
    )

    try:
        barrier_payload = payload["barrier"]
        if not isinstance(barrier_payload, Mapping):
            raise TypeError("candidate barrier is not a mapping")
        barrier = SparsePolynomial2D.from_payload(barrier_payload)
    except (KeyError, TypeError, ValueError):
        barrier = None
        failures.append(f"candidate_{label}_barrier_reconstruction")
    if barrier is not None:
        barrier_hash = content_sha256(barrier.payload())
        if (
            barrier_hash != artifact.candidate_barrier_sha256
            or payload.get("barrier_payload_sha256") != barrier_hash
        ):
            failures.append(f"candidate_{label}_barrier_binding")

    full_candidate = payload.get("full_solver_candidate")
    candidate_values: tuple[float, ...] | None = None
    if not isinstance(full_candidate, Mapping):
        failures.append(f"candidate_{label}_full_solver_candidate")
    else:
        decimal_values = full_candidate.get("values_decimal")
        hex_values = full_candidate.get("values_float64_hex")
        if (
            not isinstance(decimal_values, list)
            or not isinstance(hex_values, list)
            or len(decimal_values) != G10A3_MAX_DECISION_VARIABLES
            or len(hex_values) != G10A3_MAX_DECISION_VARIABLES
            or full_candidate.get("decision_variable_count")
            != G10A3_MAX_DECISION_VARIABLES
        ):
            failures.append(f"candidate_{label}_full_solver_layout")
        else:
            try:
                candidate_values = tuple(float(value) for value in decimal_values)
            except (TypeError, ValueError):
                candidate_values = None
            if (
                candidate_values is None
                or not np.isfinite(candidate_values).all()
                or [value.hex() for value in candidate_values] != hex_values
            ):
                candidate_values = None
                failures.append(f"candidate_{label}_full_solver_binary64")
            elif (
                full_candidate.get("values_sha256")
                != artifact.candidate_values_sha256
                or _g10a3_candidate_values_sha256(candidate_values)
                != artifact.candidate_values_sha256
            ):
                failures.append(f"candidate_{label}_full_solver_hash")

    chebyshev = payload.get("chebyshev_barrier")
    coefficient_count = len(families[0].basis) if families else 0
    if not isinstance(chebyshev, Mapping):
        failures.append(f"candidate_{label}_chebyshev_subject")
    else:
        coefficient_decimals = chebyshev.get("coefficients_decimal")
        coefficient_hex = chebyshev.get("coefficients_float64_hex")
        if (
            not isinstance(coefficient_decimals, list)
            or not isinstance(coefficient_hex, list)
            or len(coefficient_decimals) != coefficient_count
            or len(coefficient_hex) != coefficient_count
        ):
            failures.append(f"candidate_{label}_chebyshev_layout")
        else:
            try:
                coefficients = tuple(
                    float(value) for value in coefficient_decimals
                )
            except (TypeError, ValueError):
                coefficients = ()
            if (
                [value.hex() for value in coefficients] != coefficient_hex
                or candidate_values is None
                or [value.hex() for value in candidate_values[:coefficient_count]]
                != coefficient_hex
            ):
                failures.append(f"candidate_{label}_chebyshev_binding")

    if failures or barrier is None or candidate_values is None:
        return None, tuple(dict.fromkeys(failures))
    try:
        artifact_residuals = _exact_candidate_residuals(
            barrier=barrier,
            controller=controller,
            families=families,
        )
    except (KeyError, TypeError, ValueError):
        return None, (f"candidate_{label}_artifact_residuals",)
    return (
        G10A3CandidateReplaySubject(
            candidate_barrier_sha256=artifact.candidate_barrier_sha256,
            candidate_values=candidate_values,
            candidate_values_sha256=artifact.candidate_values_sha256,
            exact_artifact_residuals=artifact_residuals,
        ),
        (),
    )


def _g10a3_publication_disk_release_gate(
    *,
    validation_path: Path,
    ledger_path: Path,
    grid_catalog_path: Path,
    final_grid_path: Path,
    pending_certificate_path: Path,
    expected_validation_file_sha256: str,
    expected_validation_content_sha256: str,
    expected_ledger_payload: Mapping[str, Any],
    expected_ledger_file_sha256: str,
    expected_ledger_content_sha256: str,
    expected_ledger_row_count: int,
    expected_grid_catalog_file_sha256: str,
    expected_final_grid_file_sha256: str,
    expected_final_grid_content_sha256: str,
    expected_pending_certificate_file_sha256: str,
    expected_certificate: QSBCertificate,
    expected_grid_catalog: G10A3GridCatalog,
    expected_final_scan: G10A3GridScanResult,
    families: Sequence[_ConstraintFamily],
    expected_candidate_artifacts: Mapping[
        tuple[str, str],
        G10A3CandidateDiskArtifact,
    ],
    barrier: SparsePolynomial2D,
    controller: FrozenController,
    full_v3_call_count: int,
    resource_gate_passed: bool,
) -> tuple[bool, tuple[str, ...]]:
    """Re-read and replay every G10A3 publication subject from disk."""

    failures: list[str] = []

    def load_mapping(path: Path, label: str) -> Mapping[str, Any] | None:
        try:
            value = json.loads(path.read_text(encoding="utf-8"))
        except (OSError, UnicodeError, json.JSONDecodeError):
            failures.append(f"{label}_disk_reload")
            return None
        if not isinstance(value, Mapping):
            failures.append(f"{label}_disk_schema")
            return None
        return value

    def check_file_hash(path: Path, expected: str, label: str) -> None:
        try:
            actual = file_sha256(path)
        except OSError:
            failures.append(f"{label}_file_unreadable")
            return
        if actual != expected:
            failures.append(f"{label}_file_changed")

    disk_catalog = load_mapping(grid_catalog_path, "grid_catalog")
    catalog_gate_passed = False
    if disk_catalog is not None:
        check_file_hash(
            grid_catalog_path,
            expected_grid_catalog_file_sha256,
            "grid_catalog",
        )
        catalog_material = dict(disk_catalog)
        declared_catalog_hash = catalog_material.pop(
            "catalog_content_sha256",
            None,
        )
        catalog_gate_passed = (
            dict(disk_catalog) == dict(expected_grid_catalog.payload)
            and declared_catalog_hash == expected_grid_catalog.content_sha256
            and content_sha256(catalog_material)
            == expected_grid_catalog.content_sha256
            and disk_catalog.get("family_contract_sha256")
            == expected_grid_catalog.family_contract_sha256
            and content_sha256(disk_catalog.get("family_contract"))
            == expected_grid_catalog.family_contract_sha256
        )
        if not catalog_gate_passed:
            failures.append("grid_catalog_subject_or_content_changed")

    disk_ledger, trusted_ledger_failures = (
        _load_g10a3_trusted_cut_ledger(
            ledger_path,
            expected_payload=expected_ledger_payload,
            expected_file_sha256=expected_ledger_file_sha256,
            expected_content_sha256=expected_ledger_content_sha256,
            expected_row_count=expected_ledger_row_count,
        )
    )
    failures.extend(trusted_ledger_failures)
    ledger_gate_passed = False
    if disk_ledger is not None:
        referenced_candidates: set[tuple[str, str]] = set()
        ledger_rows = disk_ledger.get("rows")
        if isinstance(ledger_rows, list):
            for row in ledger_rows:
                if not isinstance(row, Mapping):
                    continue
                candidate_barrier_hash = row.get(
                    "candidate_barrier_sha256"
                )
                candidate_values_hash = row.get("candidate_values_sha256")
                if (
                    isinstance(candidate_barrier_hash, str)
                    and isinstance(candidate_values_hash, str)
                ):
                    referenced_candidates.add(
                        (candidate_barrier_hash, candidate_values_hash)
                    )
        disk_candidate_subjects: dict[
            str,
            G10A3CandidateReplaySubject,
        ] = {}
        candidate_disk_gate_passed = True
        for candidate_key in sorted(referenced_candidates):
            artifact = expected_candidate_artifacts.get(candidate_key)
            if artifact is None:
                candidate_disk_gate_passed = False
                failures.append(
                    "candidate_artifact_missing_for_ledger_subject_"
                    f"{candidate_key[1][:12]}"
                )
                continue
            if (
                artifact.candidate_barrier_sha256 != candidate_key[0]
                or artifact.candidate_values_sha256 != candidate_key[1]
            ):
                candidate_disk_gate_passed = False
                failures.append(
                    "candidate_artifact_expected_key_mismatch_"
                    f"{candidate_key[1][:12]}"
                )
                continue
            subject, candidate_failures = (
                _load_g10a3_candidate_replay_subject_from_disk(
                    artifact=artifact,
                    families=families,
                    controller=controller,
                )
            )
            failures.extend(candidate_failures)
            if subject is None:
                candidate_disk_gate_passed = False
                continue
            previous = disk_candidate_subjects.get(
                subject.candidate_values_sha256
            )
            if previous is not None and previous != subject:
                candidate_disk_gate_passed = False
                failures.append(
                    "candidate_artifact_subject_collision_"
                    f"{candidate_key[1][:12]}"
                )
                continue
            disk_candidate_subjects[
                subject.candidate_values_sha256
            ] = subject
        ledger_failures = validate_g10a3_cut_ledger(
            disk_ledger,
            expected_grid_catalog_sha256=(
                expected_grid_catalog.content_sha256
            ),
            families=families,
            candidate_replay_subjects=disk_candidate_subjects,
        )
        ledger_gate_passed = (
            not trusted_ledger_failures
            and candidate_disk_gate_passed
            and not ledger_failures
        )
        failures.extend(
            f"cut_ledger.{failure}" for failure in ledger_failures
        )

    disk_final_grid = load_mapping(final_grid_path, "final_grid")
    final_grid_gate_passed = False
    if disk_final_grid is not None:
        check_file_hash(
            final_grid_path,
            expected_final_grid_file_sha256,
            "final_grid",
        )
        final_grid_material = dict(disk_final_grid)
        declared_final_grid_hash = final_grid_material.pop(
            "report_content_sha256",
            None,
        )
        expected_final_payload = _grid_scan_payload(expected_final_scan)
        final_grid_gate_passed = (
            dict(disk_final_grid) == expected_final_payload
            and declared_final_grid_hash
            == expected_final_grid_content_sha256
            and content_sha256(final_grid_material)
            == expected_final_grid_content_sha256
            and disk_final_grid.get("pass_id") == "G65_FINAL_GATE"
            and disk_final_grid.get("candidate_barrier_sha256")
            == content_sha256(barrier.payload())
            and disk_final_grid.get("visited_point_count")
            == expected_grid_catalog.point_counts_by_grid["65"]
            and disk_final_grid.get("scanned_point_set_sha256")
            == expected_grid_catalog.expected_point_set_sha256_by_grid["65"]
            and disk_final_grid.get("expected_point_set_sha256")
            == expected_grid_catalog.expected_point_set_sha256_by_grid["65"]
            and disk_final_grid.get("point_set_binding_passed") is True
            and disk_final_grid.get("whole_catalog_traversed") is True
            and disk_final_grid.get("artifact_negative_point_count") == 0
            and disk_final_grid.get("selected_cut_count") == 0
            and disk_final_grid.get("selected_cuts") == []
            and disk_final_grid.get("gate_passed") is True
            and disk_final_grid.get("formal_claim_allowed") is False
        )
        if not final_grid_gate_passed:
            failures.append("final_grid_subject_or_gate_changed")

    disk_validation = load_mapping(validation_path, "validation_report")
    binding_failures: tuple[str, ...] = ("validation_report_missing",)
    v3_release_passed = False
    if disk_validation is not None:
        check_file_hash(
            validation_path,
            expected_validation_file_sha256,
            "validation_report",
        )
        if disk_validation.get("report_content_sha256") != (
            expected_validation_content_sha256
        ):
            failures.append("validation_report_content_changed")
        try:
            binding_failures = _v3_feedback_binding_failures(
                validation=disk_validation,
                validation_directory=validation_path.parent,
                barrier=barrier,
                controller=controller,
                families=families,
            )
            v3_release_passed, v3_release_failures = _v3_release_gate(
                validation=disk_validation,
                barrier=barrier,
                controller=controller,
                validation_directory=validation_path.parent,
            )
        except (KeyError, TypeError, ValueError, OSError) as error:
            binding_failures = (
                f"disk_V3_replay_{type(error).__name__}",
            )
            v3_release_failures = binding_failures
        failures.extend(
            f"V3_binding.{failure}" for failure in binding_failures
        )
        failures.extend(
            f"V3_release.{failure}" for failure in v3_release_failures
        )

    disk_certificate: QSBCertificate | None
    try:
        disk_certificate = QSBCertificate.load(
            pending_certificate_path,
            controller=controller,
        )
    except (OSError, UnicodeError, json.JSONDecodeError, ValueError):
        disk_certificate = None
        failures.append("pending_certificate_disk_reload")
    if disk_certificate is not None:
        check_file_hash(
            pending_certificate_path,
            expected_pending_certificate_file_sha256,
            "pending_certificate",
        )
        if (
            disk_certificate != expected_certificate
            or disk_certificate.sha256 != expected_certificate.sha256
        ):
            failures.append("pending_certificate_subject_changed")
        solver = disk_certificate.solver
        validation_binding = disk_certificate.validation
        trusted_validation = expected_certificate.validation
        disk_validation_binding = (
            disk_validation if disk_validation is not None else {}
        )
        trusted_barrier_hash = content_sha256(barrier.payload())
        certificate_bindings = {
            "pending_certificate_schema": disk_certificate.schema_version == 2,
            "pending_certificate_trusted_barrier_object": (
                disk_certificate.barrier == barrier
                and expected_certificate.barrier == barrier
            ),
            "pending_certificate_trusted_barrier_hash": (
                content_sha256(disk_certificate.barrier.payload())
                == trusted_barrier_hash
                and validation_binding.get("barrier_payload_sha256")
                == trusted_barrier_hash
                and disk_validation_binding.get("barrier_payload_sha256")
                == trusted_barrier_hash
            ),
            "pending_certificate_trusted_q_binary64": (
                disk_certificate.q.hex() == G10_Q.hex()
                and expected_certificate.q.hex() == G10_Q.hex()
                and validation_binding.get("q_float64_hex") == G10_Q.hex()
                and disk_validation_binding.get("q_float64_hex")
                == G10_Q.hex()
            ),
            "pending_certificate_trusted_epsbar_binary64": (
                disk_certificate.epsbar.hex() == G10_EPSBAR.hex()
                and expected_certificate.epsbar.hex() == G10_EPSBAR.hex()
                and validation_binding.get("epsbar_float64_hex")
                == G10_EPSBAR.hex()
                and disk_validation_binding.get("epsbar_float64_hex")
                == G10_EPSBAR.hex()
            ),
            "pending_certificate_trusted_controller": (
                disk_certificate.controller_content_sha256
                == controller.sha256
                and expected_certificate.controller_content_sha256
                == controller.sha256
                and validation_binding.get("controller_content_sha256")
                == controller.sha256
                and disk_validation_binding.get("controller_content_sha256")
                == controller.sha256
            ),
            "pending_certificate_trusted_validation_subject": (
                dict(validation_binding) == dict(trusted_validation)
            ),
            "pending_certificate_validation_status_binding": (
                validation_binding.get("status") == "validated"
                and validation_binding.get("status")
                == disk_validation_binding.get("status")
                and validation_binding.get("global_polynomial_status")
                == "validated"
                and validation_binding.get("global_polynomial_status")
                == disk_validation_binding.get("global_polynomial_status")
            ),
            "pending_certificate_validation_first_slice_binding": (
                validation_binding.get("first_slice_ready") is True
                and disk_validation_binding.get("first_slice_ready") is True
            ),
            "pending_certificate_validation_zero_tolerance_binding": (
                validation_binding.get("formal_acceptance_tolerance") == 0.0
                and disk_validation_binding.get(
                    "formal_acceptance_tolerance"
                )
                == 0.0
            ),
            "pending_certificate_validation_validator_binding": (
                validation_binding.get("validator_id") == G10_VALIDATOR_ID
                and validation_binding.get("validator_id")
                == disk_validation_binding.get("validator_id")
            ),
            "pending_certificate_grid_catalog_binding": (
                solver.get("grid_catalog_sha256")
                == expected_grid_catalog.content_sha256
            ),
            "pending_certificate_ledger_file_binding": (
                solver.get("selected_cut_ledger_file_sha256")
                == expected_ledger_file_sha256
            ),
            "pending_certificate_ledger_content_binding": (
                solver.get("selected_cut_ledger_content_sha256")
                == expected_ledger_content_sha256
            ),
            "pending_certificate_final_grid_binding": (
                solver.get("final_grid_report_sha256")
                == expected_final_grid_content_sha256
            ),
            "pending_certificate_validation_content_binding": (
                validation_binding.get("validation_report_content_sha256")
                == expected_validation_content_sha256
                and validation_binding.get(
                    "validation_report_content_sha256"
                )
                == disk_validation_binding.get("report_content_sha256")
            ),
            "pending_certificate_validation_file_binding": (
                validation_binding.get("validation_report_sha256")
                == expected_validation_file_sha256
            ),
        }
        failures.extend(
            name for name, passed in certificate_bindings.items() if not passed
        )

    validation_for_gate = (
        disk_validation if disk_validation is not None else {}
    )
    disk_visit_value = (
        disk_final_grid.get("visited_point_count")
        if disk_final_grid is not None
        else None
    )
    disk_visit_count = (
        disk_visit_value if type(disk_visit_value) is int else -1
    )
    publication_passed, publication_failures = g10a3_publication_gate(
        final_grid_gate_passed=final_grid_gate_passed,
        complete_grid_visit_count=disk_visit_count,
        expected_grid_visit_count=(
            expected_grid_catalog.point_counts_by_grid["65"]
        ),
        point_set_binding_passed=(
            disk_final_grid is not None
            and disk_final_grid.get("point_set_binding_passed") is True
        ),
        cut_ledger_passed=ledger_gate_passed and catalog_gate_passed,
        resource_gate_passed=resource_gate_passed,
        full_v3_call_count=full_v3_call_count,
        validation_status=validation_for_gate.get("status"),
        global_polynomial_status=validation_for_gate.get(
            "global_polynomial_status"
        ),
        first_slice_ready=validation_for_gate.get("first_slice_ready"),
        controller_contract_status=validation_for_gate.get(
            "controller_contract_status"
        ),
        geometry_contract_status=validation_for_gate.get(
            "geometry_contract_status"
        ),
        theorem_scalar_gate=validation_for_gate.get("theorem_scalar_gate"),
        formal_acceptance_tolerance=validation_for_gate.get(
            "formal_acceptance_tolerance"
        ),
        binding_failures=binding_failures,
        v3_release_gate_passed=v3_release_passed,
    )
    failures.extend(
        f"publication.{failure}" for failure in publication_failures
    )
    return publication_passed and not failures, tuple(
        dict.fromkeys(failures)
    )


def finalize_g10a3_pending_certificate(
    *,
    pending_path: str | Path,
    certificate_path: str | Path,
    expected_certificate: QSBCertificate,
    controller: FrozenController,
) -> Path:
    pending = Path(pending_path)
    certificate = Path(certificate_path)
    if (
        pending.name != "certificate.pending.json"
        or certificate.name != "certificate.json"
        or pending.parent != certificate.parent
    ):
        raise ValueError("G10A3 certificate promotion path contract mismatch")
    if not pending.is_file():
        raise FileNotFoundError("G10A3 pending certificate is missing")
    if certificate.exists():
        raise FileExistsError("G10A3 final certificate already exists")
    pending.replace(certificate)
    try:
        reconstructed = QSBCertificate.load(
            certificate,
            controller=controller,
        )
    except (OSError, UnicodeError, json.JSONDecodeError, ValueError) as error:
        raise ValueError(
            "G10A3 final certificate reconstruction failed"
        ) from error
    if (
        reconstructed != expected_certificate
        or reconstructed.sha256 != expected_certificate.sha256
    ):
        raise ValueError("G10A3 final certificate subject changed after rename")
    return certificate


def cleanup_g10a3_publication_artifacts(
    destination: str | Path,
) -> tuple[str, ...]:
    root = Path(destination)
    targets = (
        root / ".certificate.pending.json.tmp",
        root / "certificate.pending.json",
        root / ".certificate.json.tmp",
        root / "certificate.json",
    )
    failures: list[str] = []
    for target in targets:
        try:
            target.unlink(missing_ok=True)
        except OSError as error:
            failures.append(f"{target.name}:{type(error).__name__}:{error}")
        if target.exists():
            failures.append(f"{target.name}:still_exists_after_cleanup")
    return tuple(failures)


def _write_g10a3_pending_certificate(
    *,
    destination: Path,
    barrier: SparsePolynomial2D,
    controller: FrozenController,
    validation: Mapping[str, Any],
    validation_path: Path,
    solver_backend: str,
    selected_point_row_count: int,
    selected_cut_ledger_file_sha256: str,
    selected_cut_ledger_content_sha256: str,
    grid_catalog_sha256: str,
    v3_root_set_sha256: str,
    final_grid_report_sha256: str,
) -> tuple[Path, QSBCertificate]:
    _validate_sha256(
        selected_cut_ledger_file_sha256,
        name="selected cut-ledger file hash",
    )
    _validate_sha256(
        selected_cut_ledger_content_sha256,
        name="selected cut-ledger content hash",
    )
    publication_fields = {
        "schema_version": validation.get("schema_version") == 2,
        "validator_id": validation.get("validator_id") == G10_VALIDATOR_ID,
        "status": validation.get("status") == "validated",
        "global_polynomial_status": validation.get(
            "global_polynomial_status"
        )
        == "validated",
        "first_slice_ready": validation.get("first_slice_ready") is True,
        "formal_acceptance_tolerance": validation.get(
            "formal_acceptance_tolerance"
        )
        == 0.0,
    }
    if not all(publication_fields.values()):
        raise ValueError(
            "G10A3 publication called without a schema-v2 validated V3 subject"
        )
    certificate = QSBCertificate(
        certificate_id=f"CBF_ROVER_G10A3_W16_{controller.sha256[:12]}",
        barrier=barrier,
        q=G10_Q,
        epsbar=G10_EPSBAR,
        controller_content_sha256=controller.sha256,
        schema_version=2,
        solver={
            "method": "exact_rational_grid_multipoint_numeric_LP",
            "synthesis_id": G10A3_SYNTHESIS_ID,
            "proof_route_id": G10A3_PROOF_ROUTE_ID,
            "solver_backend": solver_backend,
            "stage_id": G10A3_STAGE.stage_id,
            "barrier_basis": "total_degree_tensor_chebyshev",
            "barrier_degree": G10A3_STAGE.barrier_degree,
            "controller_degree": controller_max_degree(controller),
            "rho": 0.0,
            "selected_point_cut_count": selected_point_row_count,
            "selected_cut_ledger_file_sha256": (
                selected_cut_ledger_file_sha256
            ),
            "selected_cut_ledger_content_sha256": (
                selected_cut_ledger_content_sha256
            ),
            "grid_catalog_sha256": grid_catalog_sha256,
            "v3_root_set_sha256": v3_root_set_sha256,
            "final_grid_report_sha256": final_grid_report_sha256,
            "grid_and_LP_are_not_formal_evidence": True,
            "full_V3_call_count": 1,
        },
        validation={
            "validator_id": validation["validator_id"],
            "validation_report_sha256": file_sha256(validation_path),
            "validation_report_content_sha256": validation[
                "report_content_sha256"
            ],
            "status": validation["status"],
            "global_polynomial_status": validation[
                "global_polynomial_status"
            ],
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
    pending_path = destination / "certificate.pending.json"
    if pending_path.exists() or (destination / "certificate.json").exists():
        raise FileExistsError("G10A3 publication path is not empty")
    certificate.save(pending_path)
    return pending_path, certificate


def synthesize_g10a3_qsbc_schedule(
    *,
    frozen_controller_path: str | Path,
    g9_seed_path: str | Path,
    output_dir: str | Path,
) -> Path:
    """Run the finite W16 grid-separation route and at most one full V3."""

    destination = Path(output_dir).resolve()
    destination.mkdir(parents=True, exist_ok=False)
    manifest_path = destination / "schedule_manifest.json"
    pending_certificate_path = destination / "certificate.pending.json"
    final_certificate_path = destination / "certificate.json"
    stage_started = monotonic()
    stage_deadline = stage_started + G10A3_STAGE.stage_wall_time_limit_s
    manifest: dict[str, Any] = {
        "schema_version": 1,
        "kind": "g10a3_finite_exact_grid_qsbc_schedule",
        "synthesis_id": G10A3_SYNTHESIS_ID,
        "proof_route_id": G10A3_PROOF_ROUTE_ID,
        "status": "building",
        "fixed_scalars": {
            "q": G10_Q,
            "q_float64_hex": G10_Q.hex(),
            "epsbar": G10_EPSBAR,
            "epsbar_float64_hex": G10_EPSBAR.hex(),
            "rho": 0.0,
            "rho_float64_hex": 0.0.hex(),
        },
        "stage": {
            "stage_id": G10A3_STAGE.stage_id,
            "barrier_degree": G10A3_STAGE.barrier_degree,
            "max_lp_solves": G10A3_STAGE.max_lp_solves,
            "aggregate_lp_time_limit_s": (
                G10A3_STAGE.aggregate_lp_time_limit_s
            ),
            "aggregate_exact_grid_time_limit_s": (
                G10A3_STAGE.aggregate_grid_time_limit_s
            ),
            "stage_wall_time_limit_s": G10A3_STAGE.stage_wall_time_limit_s,
            "max_full_V3_calls": G10A3_MAX_FULL_V3_CALLS,
        },
        "hard_resource_caps": {
            "max_rss_bytes": G10A3_MAX_RSS_BYTES,
            "max_selected_point_rows": G10A3_MAX_HARD_POINT_ROWS,
            "catalog_generation_point_yields": (
                G10A3_CATALOG_GENERATION_POINT_YIELDS
            ),
            "max_candidate_scan_point_visits": (
                G10A3_MAX_CANDIDATE_SCAN_POINT_VISITS
            ),
            "max_total_grid_point_yields": (
                G10A3_MAX_TOTAL_GRID_POINT_YIELDS
            ),
            "max_master_lp_rows": G10A3_MAX_MASTER_LP_ROWS,
            "max_ephemeral_lp_rows": G10A3_MAX_EPHEMERAL_LP_ROWS,
            "max_csr_nonzeros": G10A3_MAX_CSR_NONZEROS,
            "max_decision_variables": G10A3_MAX_DECISION_VARIABLES,
        },
        "grid_pass_schedule": [
            {
                "pass_id": item.pass_id,
                "nodes_per_axis": item.nodes_per_axis,
                "strata_per_axis": item.strata_per_axis,
                "max_new_points_per_family": (
                    item.max_new_points_per_family
                ),
            }
            for item in G10A3_GRID_PASS_SCHEDULE
        ],
        "automatic_degree_grid_row_or_V3_escalation_allowed": False,
        "LP_phases": [],
        "candidates": [],
        "grid_passes": [],
        "full_V3_call_count": 0,
        "resource_samples": [],
        "g10a3_resource_samples": [],
    }
    _save_manifest(manifest_path, manifest)
    try:
        controller_path = Path(frozen_controller_path).resolve()
        controller = load_frozen_controller(controller_path)
        controller_file_hash = file_sha256(controller_path)
        _assert_frozen_c2(
            controller,
            controller_file_sha256=controller_file_hash,
        )
        seed = _load_g9_seed(g9_seed_path)
        if not seed.chebyshev_by_power:
            raise ValueError("G10A3 requires the bound G9 Chebyshev seed vector")
        if seed.barrier.degree > G10A3_STAGE.barrier_degree:
            raise ValueError("G10A3 G9 seed degree exceeds frozen W16")

        chebyshev_indices, barrier_basis = total_degree_chebyshev_basis(
            G10A3_STAGE.barrier_degree
        )
        coefficient_count = len(chebyshev_indices)
        if coefficient_count != 153:
            raise ValueError("G10A3 W16 coefficient catalog is not frozen 153")
        families = _constraint_families(
            controller=controller,
            barrier_basis=barrier_basis,
        )
        if len(families) != G10A_FROZEN_QSBC_FAMILY_COUNT:
            raise ValueError("G10A3 q-SBC family count is not frozen 42")
        anchors, dormant_rows = _g9_ranked_rows(
            seed=seed,
            controller=controller,
            families=families,
        )
        geometry_binding_hash, geometry_binding_count = (
            _g9_depth3_geometry_binding(
                families=families,
                anchors=anchors,
                dormant_rows=dormant_rows,
            )
        )

        catalog_started = monotonic()
        grid_catalog = build_g10a3_grid_catalog(families)
        catalog_elapsed_s = monotonic() - catalog_started
        cumulative_grid_time_s = catalog_elapsed_s
        if cumulative_grid_time_s > G10A3_STAGE.aggregate_grid_time_limit_s:
            raise G10A3GridBudgetExceeded(
                "G10A3 grid catalog exceeded aggregate exact-grid budget"
            )
        expected_counts = {
            "17": G10A_FROZEN_QSBC_FAMILY_COUNT * 17**2,
            "33": G10A_FROZEN_QSBC_FAMILY_COUNT * 33**2,
            "65": G10A_FROZEN_QSBC_FAMILY_COUNT * 65**2,
        }
        if dict(grid_catalog.point_counts_by_grid) != expected_counts:
            raise ValueError("G10A3 frozen grid catalog count mismatch")
        if (
            grid_catalog.catalog_generation_point_yields
            != G10A3_CATALOG_GENERATION_POINT_YIELDS
        ):
            raise ValueError("G10A3 catalog generation-yield count mismatch")
        grid_catalog_path = destination / "exact_grid_catalog.json"
        _write_json(grid_catalog_path, grid_catalog.payload)

        core_store = _ActiveRowStore(
            families=families,
            variable_count=coefficient_count + 1,
            rho_index=coefficient_count,
        )
        _, _, accepted_anchors = core_store.append(anchors)
        if len(accepted_anchors) != G10A_FROZEN_QSBC_FAMILY_COUNT:
            raise ValueError("G10A3 did not bind one G9 anchor per family")
        seed_coefficients = tuple(
            float(seed.chebyshev_by_power.get(power, 0.0))
            for power in chebyshev_indices
        )
        projection = build_g10a2_projection_problem(
            seed_coefficients=seed_coefficients,
            anchor_core_matrix=core_store.matrix,
            anchor_right_hand_side=core_store.right_hand_side,
        )
        if (
            projection.variable_count != G10A3_MAX_DECISION_VARIABLES
            or projection.matrix.shape[0]
            != G10A_FROZEN_QSBC_FAMILY_COUNT + 4 * coefficient_count
        ):
            raise ValueError("G10A3 frozen projection layout mismatch")
        master_matrix = projection.matrix.copy().tocsr()
        master_rhs = projection.right_hand_side.copy()
        _record_g10a3_resource_sample(
            manifest=manifest,
            label="initial_projection_problem",
            matrix=master_matrix,
        )

        manifest["inputs"] = {
            "controller": {
                "file_sha256": controller_file_hash,
                "content_sha256": controller.sha256,
                "frozen_file_sha256": G10_CONTROLLER_FILE_SHA256,
                "frozen_content_sha256": G10_CONTROLLER_CONTENT_SHA256,
            },
            "g9_seed": {
                "role": G10A_G9_ROLE,
                "file_sha256": seed.file_sha256,
                "report_content_sha256": seed.report_content_sha256,
                "source_mode": seed.source_mode,
                "source_probe_file_sha256": seed.source_probe_file_sha256,
                "barrier_payload_sha256": content_sha256(
                    seed.barrier.payload()
                ),
                "barrier_is_used_as_formal_subject": False,
            },
            "g9_depth3_eligible_geometry_sha256": geometry_binding_hash,
            "g9_depth3_eligible_geometry_count": geometry_binding_count,
            "g9_dormant_rows_used_by_G10A3": 0,
        }
        manifest["grid_catalog"] = {
            "file": grid_catalog_path.name,
            "file_sha256": file_sha256(grid_catalog_path),
            "content_sha256": grid_catalog.content_sha256,
            "family_contract_sha256": (
                grid_catalog.family_contract_sha256
            ),
            "v3_root_set_sha256": grid_catalog.payload[
                "v3_root_set_sha256"
            ],
            "point_counts_by_grid": dict(
                grid_catalog.point_counts_by_grid
            ),
            "catalog_generation_point_yields": (
                grid_catalog.catalog_generation_point_yields
            ),
            "expected_point_set_sha256_by_grid": dict(
                grid_catalog.expected_point_set_sha256_by_grid
            ),
            "catalog_elapsed_s": catalog_elapsed_s,
            "point_payloads_embedded": False,
        }
        manifest["projection_problem"] = {
            "layout": "[153 Chebyshev c, fixed rho, Linf t, 153 L1 z]",
            "coefficient_count": coefficient_count,
            "rho_index": projection.rho_index,
            "t_index": projection.t_index,
            "z_start_index": projection.z_start_index,
            "decision_variable_count": projection.variable_count,
            "anchor_row_count": len(accepted_anchors),
            "regularizer_row_count": projection.regularizer_row_count,
            "primary_objective": "min_t",
            "secondary_objective": "min_sum_z_given_t_bound",
            "pure_L1_objective_allowed": False,
            "rho_bounds": [0.0, 0.0],
            "projection_kernel_reused_from_G10A2": True,
        }
        ledger_path = destination / "selected_point_cut_ledger.json"
        all_ledger_rows: list[Mapping[str, Any]] = []
        candidate_replay_subjects: dict[
            str,
            G10A3CandidateReplaySubject,
        ] = {}
        candidate_disk_artifacts: dict[
            tuple[str, str],
            G10A3CandidateDiskArtifact,
        ] = {}
        trusted_ledger_payload = _save_g10a3_cut_ledger(
            ledger_path,
            grid_catalog_sha256=grid_catalog.content_sha256,
            families=families,
            candidate_replay_subjects=candidate_replay_subjects,
            rows=all_ledger_rows,
        )
        trusted_ledger_file_sha256 = file_sha256(ledger_path)
        manifest["status"] = "solving"
        _save_manifest(manifest_path, manifest)

        cumulative_lp_time_s = 0.0
        catalog_generation_point_yields = (
            grid_catalog.catalog_generation_point_yields
        )
        candidate_scan_point_visits = 0
        total_grid_point_yields = catalog_generation_point_yields
        current_seed_values = projection.seed_values.copy()
        existing_definition_ids: set[str] = set()
        final_barrier: SparsePolynomial2D | None = None
        final_coefficients: tuple[float, ...] | None = None
        final_solve: Mapping[str, Any] | None = None
        final_candidate_directory: Path | None = None
        for candidate_index in range(1, 5):
            source_pass_id = (
                "INITIAL_G9_ANCHORS"
                if candidate_index == 1
                else G10A3_GRID_PASS_SCHEDULE[candidate_index - 2].pass_id
            )
            _check_g10a3_stage_deadline(stage_deadline=stage_deadline)
            _record_g10a3_resource_sample(
                manifest=manifest,
                label=f"candidate_{candidate_index:02d}_master",
                matrix=master_matrix,
            )
            solve = _solve_g10a3_two_phase_projection(
                projection=projection,
                master_matrix=master_matrix,
                master_right_hand_side=master_rhs,
                seed_values=current_seed_values,
                candidate_index=candidate_index,
                first_solve_index=2 * candidate_index - 1,
                cumulative_lp_time_s=cumulative_lp_time_s,
                stage_deadline=stage_deadline,
                phase_records=manifest["LP_phases"],
                manifest=manifest,
            )
            cumulative_lp_time_s = float(solve["cumulative_lp_time_s"])
            values = np.asarray(solve["values"], dtype=np.float64)
            candidate_values = tuple(float(value) for value in values)
            candidate_values_sha256 = _g10a3_candidate_values_sha256(
                candidate_values
            )
            current_seed_values = values.copy()
            coefficients = tuple(
                float(value) for value in values[:coefficient_count]
            )
            barrier = chebyshev_coefficients_to_monomial(
                indices=chebyshev_indices,
                coefficients=coefficients,
                basis=barrier_basis,
            )
            barrier_hash = content_sha256(barrier.payload())
            candidate_directory = destination / f"candidate_{candidate_index:02d}"
            candidate_directory.mkdir(parents=False, exist_ok=False)
            candidate_payload = _candidate_payload(
                candidate_index=candidate_index,
                source_pass_id=source_pass_id,
                chebyshev_indices=chebyshev_indices,
                chebyshev_coefficients=coefficients,
                barrier=barrier,
                controller=controller,
                solve=solve,
                projection_rho_index=projection.rho_index,
                projection_t_index=projection.t_index,
            )
            candidate_path = candidate_directory / "numeric_candidate.json"
            _write_json(candidate_path, candidate_payload)
            candidate_file_hash = file_sha256(candidate_path)
            candidate_key = (barrier_hash, candidate_values_sha256)
            candidate_disk_artifacts.setdefault(
                candidate_key,
                G10A3CandidateDiskArtifact(
                    path=candidate_path,
                    file_sha256=candidate_file_hash,
                    content_sha256=str(candidate_payload["content_sha256"]),
                    candidate_barrier_sha256=barrier_hash,
                    candidate_values_sha256=candidate_values_sha256,
                ),
            )
            manifest["candidates"].append(
                {
                    "candidate_index": candidate_index,
                    "source_pass_id": source_pass_id,
                    "file": str(candidate_path.relative_to(destination)),
                    "file_sha256": candidate_file_hash,
                    "content_sha256": candidate_payload["content_sha256"],
                    "barrier_payload_sha256": barrier_hash,
                    "candidate_values_sha256": candidate_values_sha256,
                    "master_row_count": int(master_matrix.shape[0]),
                    "master_nonzero_count": int(master_matrix.nnz),
                    "selected_point_row_count": len(all_ledger_rows),
                }
            )
            _save_manifest(manifest_path, manifest)
            final_barrier = barrier
            final_coefficients = coefficients
            final_solve = solve
            final_candidate_directory = candidate_directory
            if candidate_index == 4:
                break

            pass_spec = G10A3_GRID_PASS_SCHEDULE[candidate_index - 1]
            scan_started = monotonic()
            remaining_grid_s = (
                G10A3_STAGE.aggregate_grid_time_limit_s
                - cumulative_grid_time_s
            )
            if remaining_grid_s <= 0.0:
                raise G10A3GridBudgetExceeded(
                    "G10A3 aggregate exact-grid budget exhausted"
                )
            scan_deadline = min(
                monotonic() + remaining_grid_s,
                stage_deadline,
            )
            exact_artifact_residuals = _exact_candidate_residuals(
                barrier=barrier,
                controller=controller,
                families=families,
            )
            exact_coefficient_space_residuals = (
                _exact_coefficient_space_residuals(
                    families=families,
                    candidate_chebyshev_coefficients=coefficients,
                )
            )
            candidate_replay_subjects[candidate_values_sha256] = (
                G10A3CandidateReplaySubject(
                    candidate_barrier_sha256=barrier_hash,
                    candidate_values=candidate_values,
                    candidate_values_sha256=candidate_values_sha256,
                    exact_artifact_residuals=exact_artifact_residuals,
                )
            )
            scan = scan_g10a3_exact_grid(
                families=families,
                exact_coefficient_space_residuals=(
                    exact_coefficient_space_residuals
                ),
                exact_artifact_residuals=exact_artifact_residuals,
                candidate_values=candidate_values,
                candidate_values_sha256=candidate_values_sha256,
                variable_count=projection.variable_count,
                rho_index=projection.rho_index,
                candidate_barrier_sha256=barrier_hash,
                family_contract_sha256=(
                    grid_catalog.family_contract_sha256
                ),
                pass_spec=pass_spec,
                expected_point_set_sha256=(
                    grid_catalog.expected_point_set_sha256_by_grid[
                        str(pass_spec.nodes_per_axis)
                    ]
                ),
                existing_constraint_definition_ids=frozenset(
                    existing_definition_ids
                ),
                deadline=scan_deadline,
            )
            scan_elapsed_s = monotonic() - scan_started
            cumulative_grid_time_s += scan_elapsed_s
            candidate_scan_point_visits += scan.visited_point_count
            total_grid_point_yields = (
                catalog_generation_point_yields
                + candidate_scan_point_visits
            )
            expected_visits = (
                G10A_FROZEN_QSBC_FAMILY_COUNT
                * pass_spec.nodes_per_axis**2
            )
            if (
                cumulative_grid_time_s
                > G10A3_STAGE.aggregate_grid_time_limit_s
                or candidate_scan_point_visits
                > G10A3_MAX_CANDIDATE_SCAN_POINT_VISITS
                or total_grid_point_yields
                > G10A3_MAX_TOTAL_GRID_POINT_YIELDS
            ):
                raise G10A3GridBudgetExceeded(
                    "G10A3 exact-grid time or point-yield cap exceeded"
                )
            if (
                not scan.whole_catalog_traversed
                or not scan.point_set_binding_passed
                or scan.visited_point_count != expected_visits
            ):
                raise ValueError("G10A3 grid pass catalog binding failed")
            scan_payload = _grid_scan_payload(scan)
            scan_path = candidate_directory / f"grid_{pass_spec.pass_id}.json"
            _write_json(scan_path, scan_payload)
            cut_matrix, cut_rhs, ledger_rows = (
                build_g10a3_numeric_point_cut_rows(
                    families=families,
                    cuts=scan.selected_cuts,
                    candidate_barrier_sha256=barrier_hash,
                    candidate_values=candidate_values,
                    candidate_values_sha256=candidate_values_sha256,
                    exact_artifact_residuals=exact_artifact_residuals,
                    variable_count=projection.variable_count,
                    rho_index=projection.rho_index,
                )
            )
            selected_ids = {
                cut.constraint_definition_id for cut in scan.selected_cuts
            }
            if (
                len(selected_ids) != len(scan.selected_cuts)
                or selected_ids.intersection(existing_definition_ids)
                or cut_matrix.shape[0] != len(scan.selected_cuts)
            ):
                raise ValueError("G10A3 selected point-cut identity mismatch")
            existing_definition_ids.update(selected_ids)
            all_ledger_rows.extend(ledger_rows)
            if len(all_ledger_rows) > G10A3_MAX_HARD_POINT_ROWS:
                raise G10A3GridBudgetExceeded(
                    "G10A3 selected hard point-row cap exceeded"
                )
            master_matrix = vstack((master_matrix, cut_matrix), format="csr")
            master_rhs = np.concatenate((master_rhs, cut_rhs))
            _record_g10a3_resource_sample(
                manifest=manifest,
                label=f"after_{pass_spec.pass_id}",
                matrix=master_matrix,
            )
            trusted_ledger_payload = _save_g10a3_cut_ledger(
                ledger_path,
                grid_catalog_sha256=grid_catalog.content_sha256,
                families=families,
                candidate_replay_subjects=candidate_replay_subjects,
                rows=all_ledger_rows,
            )
            trusted_ledger_file_sha256 = file_sha256(ledger_path)
            manifest["grid_passes"].append(
                {
                    "pass_id": pass_spec.pass_id,
                    "candidate_index": candidate_index,
                    "candidate_barrier_sha256": barrier_hash,
                    "report": str(scan_path.relative_to(destination)),
                    "report_file_sha256": file_sha256(scan_path),
                    "report_content_sha256": scan_payload[
                        "report_content_sha256"
                    ],
                    "scan_elapsed_s": scan_elapsed_s,
                    "aggregate_exact_grid_time_used_s": (
                        cumulative_grid_time_s
                    ),
                    "visited_point_count": scan.visited_point_count,
                    "aggregate_candidate_scan_point_visits": (
                        candidate_scan_point_visits
                    ),
                    "aggregate_total_grid_point_yields": (
                        total_grid_point_yields
                    ),
                    "selected_new_point_count": len(scan.selected_cuts),
                    "aggregate_selected_point_count": len(all_ledger_rows),
                    "whole_catalog_traversed": scan.whole_catalog_traversed,
                    "point_set_binding_passed": (
                        scan.point_set_binding_passed
                    ),
                    "ledger_content_sha256": trusted_ledger_payload[
                        "content_sha256"
                    ],
                    "formal_claim_allowed": False,
                }
            )
            _save_manifest(manifest_path, manifest)

        if (
            final_barrier is None
            or final_coefficients is None
            or final_solve is None
            or final_candidate_directory is None
        ):
            raise RuntimeError("G10A3 finite candidate schedule produced no subject")
        final_grid_spec = G10A3GridPassSpec(
            "G65_FINAL_GATE",
            65,
            8,
            32,
        )
        gate_started = monotonic()
        remaining_grid_s = (
            G10A3_STAGE.aggregate_grid_time_limit_s
            - cumulative_grid_time_s
        )
        if remaining_grid_s <= 0.0:
            raise G10A3GridBudgetExceeded(
                "G10A3 exact-grid budget expired before final gate"
            )
        final_barrier_hash = content_sha256(final_barrier.payload())
        final_candidate_values = tuple(
            float(value) for value in final_solve["values"]
        )
        final_candidate_values_sha256 = _g10a3_candidate_values_sha256(
            final_candidate_values
        )
        final_artifact_residuals = _exact_candidate_residuals(
            barrier=final_barrier,
            controller=controller,
            families=families,
        )
        final_coefficient_space_residuals = _exact_coefficient_space_residuals(
            families=families,
            candidate_chebyshev_coefficients=final_coefficients,
        )
        final_scan = scan_g10a3_exact_grid(
            families=families,
            exact_coefficient_space_residuals=(
                final_coefficient_space_residuals
            ),
            exact_artifact_residuals=final_artifact_residuals,
            candidate_values=final_candidate_values,
            candidate_values_sha256=final_candidate_values_sha256,
            variable_count=projection.variable_count,
            rho_index=projection.rho_index,
            candidate_barrier_sha256=final_barrier_hash,
            family_contract_sha256=grid_catalog.family_contract_sha256,
            pass_spec=final_grid_spec,
            expected_point_set_sha256=(
                grid_catalog.expected_point_set_sha256_by_grid["65"]
            ),
            existing_constraint_definition_ids=frozenset(),
            select_points=False,
            deadline=min(monotonic() + remaining_grid_s, stage_deadline),
        )
        gate_elapsed_s = monotonic() - gate_started
        cumulative_grid_time_s += gate_elapsed_s
        candidate_scan_point_visits += final_scan.visited_point_count
        total_grid_point_yields = (
            catalog_generation_point_yields + candidate_scan_point_visits
        )
        expected_final_visits = G10A_FROZEN_QSBC_FAMILY_COUNT * 65**2
        if (
            cumulative_grid_time_s > G10A3_STAGE.aggregate_grid_time_limit_s
            or candidate_scan_point_visits
            != G10A3_MAX_CANDIDATE_SCAN_POINT_VISITS
            or total_grid_point_yields
            != G10A3_MAX_TOTAL_GRID_POINT_YIELDS
        ):
            raise G10A3GridBudgetExceeded(
                "G10A3 final exact-grid time or point-yield gate failed"
            )
        final_scan_payload = _grid_scan_payload(final_scan)
        final_scan_path = final_candidate_directory / "final_G65_exact_grid.json"
        _write_json(final_scan_path, final_scan_payload)
        manifest["final_exact_G65_grid_gate"] = {
            "candidate_barrier_sha256": final_barrier_hash,
            "report": str(final_scan_path.relative_to(destination)),
            "report_file_sha256": file_sha256(final_scan_path),
            "report_content_sha256": final_scan_payload[
                "report_content_sha256"
            ],
            "scan_elapsed_s": gate_elapsed_s,
            "aggregate_exact_grid_time_used_s": cumulative_grid_time_s,
            "visited_point_count": final_scan.visited_point_count,
            "expected_visited_point_count": expected_final_visits,
            "aggregate_candidate_scan_point_visits": (
                candidate_scan_point_visits
            ),
            "aggregate_total_grid_point_yields": total_grid_point_yields,
            "coefficient_space_negative_point_count": (
                final_scan.coefficient_space_negative_point_count
            ),
            "artifact_negative_point_count": final_scan.negative_point_count,
            "gate_uses_artifact_residual_only": True,
            "whole_catalog_traversed": final_scan.whole_catalog_traversed,
            "point_set_binding_passed": final_scan.point_set_binding_passed,
            "gate_passed": final_scan.gate_passed,
            "formal_claim_allowed": False,
        }
        disk_ledger_payload, trusted_ledger_failures = (
            _load_g10a3_trusted_cut_ledger(
                ledger_path,
                expected_payload=trusted_ledger_payload,
                expected_file_sha256=trusted_ledger_file_sha256,
                expected_content_sha256=str(
                    trusted_ledger_payload["content_sha256"]
                ),
                expected_row_count=len(all_ledger_rows),
            )
        )
        ledger_failure_list = list(trusted_ledger_failures)
        if disk_ledger_payload is not None:
            ledger_failure_list.extend(
                validate_g10a3_cut_ledger(
                    disk_ledger_payload,
                    expected_grid_catalog_sha256=(
                        grid_catalog.content_sha256
                    ),
                    families=families,
                    candidate_replay_subjects=candidate_replay_subjects,
                )
            )
        ledger_failures = tuple(dict.fromkeys(ledger_failure_list))
        if not final_scan.gate_passed:
            manifest["status"] = "not_ready"
            manifest["terminal_reason"] = (
                "final_complete_G65_exact_grid_gate_failed_no_V3"
            )
            manifest["aggregate_lp_time_used_s"] = cumulative_lp_time_s
            manifest["aggregate_exact_grid_time_used_s"] = (
                cumulative_grid_time_s
            )
            manifest["catalog_generation_point_yields"] = (
                catalog_generation_point_yields
            )
            manifest["candidate_scan_point_visits"] = (
                candidate_scan_point_visits
            )
            manifest["total_grid_point_yields"] = total_grid_point_yields
            manifest["selected_point_cut_ledger"] = {
                "file": ledger_path.name,
                "file_sha256": trusted_ledger_file_sha256,
                "content_sha256": trusted_ledger_payload[
                    "content_sha256"
                ],
                "row_count": len(all_ledger_rows),
                "validation_failures": list(ledger_failures),
            }
            manifest["stage_wall_time_used_s"] = monotonic() - stage_started
            _save_manifest(manifest_path, manifest)
            return manifest_path
        if ledger_failures:
            raise ValueError(
                f"G10A3 final cut ledger binding failed: {ledger_failures}"
            )

        _check_g10a3_stage_deadline(stage_deadline=stage_deadline)
        gc.collect()
        _record_g10a3_resource_sample(
            manifest=manifest,
            label="before_unique_full_V3",
            matrix=master_matrix,
        )
        manifest["full_V3_call_count"] = 1
        _save_manifest(manifest_path, manifest)
        validation_path = _validate_candidate_v3(
            barrier=final_barrier,
            controller=controller,
            output_dir=final_candidate_directory
            / "independent_validation_v3",
        )
        _record_g10a3_resource_sample(
            manifest=manifest,
            label="after_unique_full_V3",
            matrix=master_matrix,
        )
        _check_g10a3_stage_deadline(stage_deadline=stage_deadline)
        validation = json.loads(validation_path.read_text(encoding="utf-8"))
        binding_failures = _v3_feedback_binding_failures(
            validation=validation,
            validation_directory=validation_path.parent,
            barrier=final_barrier,
            controller=controller,
            families=families,
        )
        v3_release_passed, v3_release_failures = _v3_release_gate(
            validation=validation,
            barrier=final_barrier,
            controller=controller,
            validation_directory=validation_path.parent,
        )
        publication_passed, publication_failures = g10a3_publication_gate(
            final_grid_gate_passed=final_scan.gate_passed,
            complete_grid_visit_count=final_scan.visited_point_count,
            expected_grid_visit_count=expected_final_visits,
            point_set_binding_passed=final_scan.point_set_binding_passed,
            cut_ledger_passed=not ledger_failures,
            resource_gate_passed=True,
            full_v3_call_count=manifest["full_V3_call_count"],
            validation_status=validation.get("status"),
            global_polynomial_status=validation.get(
                "global_polynomial_status"
            ),
            first_slice_ready=validation.get("first_slice_ready"),
            controller_contract_status=validation.get(
                "controller_contract_status"
            ),
            geometry_contract_status=validation.get(
                "geometry_contract_status"
            ),
            theorem_scalar_gate=validation.get("theorem_scalar_gate"),
            formal_acceptance_tolerance=validation.get(
                "formal_acceptance_tolerance"
            ),
            binding_failures=binding_failures,
            v3_release_gate_passed=v3_release_passed,
        )
        manifest["unique_full_V3"] = {
            "candidate_barrier_sha256": final_barrier_hash,
            "file": str(validation_path.relative_to(destination)),
            "file_sha256": file_sha256(validation_path),
            "report_content_sha256": validation.get(
                "report_content_sha256"
            ),
            "validator_id": validation.get("validator_id"),
            "status": validation.get("status"),
            "global_polynomial_status": validation.get(
                "global_polynomial_status"
            ),
            "first_slice_ready": validation.get("first_slice_ready"),
            "subject_and_tree_binding_failures": list(binding_failures),
            "unchanged_V3_release_gate_passed": v3_release_passed,
            "unchanged_V3_release_gate_failures": list(
                v3_release_failures
            ),
            "G10A3_publication_gate_passed": publication_passed,
            "G10A3_publication_gate_failures": list(publication_failures),
        }
        manifest["aggregate_lp_time_used_s"] = cumulative_lp_time_s
        manifest["aggregate_exact_grid_time_used_s"] = cumulative_grid_time_s
        manifest["catalog_generation_point_yields"] = (
            catalog_generation_point_yields
        )
        manifest["candidate_scan_point_visits"] = candidate_scan_point_visits
        manifest["total_grid_point_yields"] = total_grid_point_yields
        manifest["selected_point_cut_ledger"] = {
            "file": ledger_path.name,
            "file_sha256": trusted_ledger_file_sha256,
            "content_sha256": trusted_ledger_payload["content_sha256"],
            "row_count": len(all_ledger_rows),
            "unchanged_V3_reads_this_ledger": False,
        }
        if not publication_passed:
            manifest["status"] = "not_ready"
            manifest["terminal_reason"] = (
                "unique_unchanged_V3_did_not_pass_all_release_gates_no_retry"
            )
            manifest["stage_wall_time_used_s"] = monotonic() - stage_started
            _save_manifest(manifest_path, manifest)
            return manifest_path

        _record_g10a3_resource_sample(
            manifest=manifest,
            label="before_publication",
            matrix=master_matrix,
        )
        _check_g10a3_stage_deadline(stage_deadline=stage_deadline)
        written_pending_path, expected_certificate = (
            _write_g10a3_pending_certificate(
                destination=destination,
                barrier=final_barrier,
                controller=controller,
                validation=validation,
                validation_path=validation_path,
                solver_backend=final_solve["secondary"].backend,
                selected_point_row_count=len(all_ledger_rows),
                selected_cut_ledger_file_sha256=str(
                    manifest["selected_point_cut_ledger"]["file_sha256"]
                ),
                selected_cut_ledger_content_sha256=str(
                    trusted_ledger_payload["content_sha256"]
                ),
                grid_catalog_sha256=grid_catalog.content_sha256,
                v3_root_set_sha256=str(
                    grid_catalog.payload["v3_root_set_sha256"]
                ),
                final_grid_report_sha256=final_scan_payload[
                    "report_content_sha256"
                ],
            )
        )
        if written_pending_path != pending_certificate_path:
            raise ValueError("G10A3 pending certificate path binding mismatch")
        _record_g10a3_resource_sample(
            manifest=manifest,
            label="after_pending_certificate_write",
            matrix=master_matrix,
        )
        _check_g10a3_stage_deadline(stage_deadline=stage_deadline)
        pending_certificate_file_hash = file_sha256(
            pending_certificate_path
        )
        disk_release_passed, disk_release_failures = (
            _g10a3_publication_disk_release_gate(
                validation_path=validation_path,
                ledger_path=ledger_path,
                grid_catalog_path=grid_catalog_path,
                final_grid_path=final_scan_path,
                pending_certificate_path=pending_certificate_path,
                expected_validation_file_sha256=str(
                    manifest["unique_full_V3"]["file_sha256"]
                ),
                expected_validation_content_sha256=str(
                    validation["report_content_sha256"]
                ),
                expected_ledger_payload=trusted_ledger_payload,
                expected_ledger_file_sha256=str(
                    manifest["selected_point_cut_ledger"]["file_sha256"]
                ),
                expected_ledger_content_sha256=str(
                    trusted_ledger_payload["content_sha256"]
                ),
                expected_ledger_row_count=len(all_ledger_rows),
                expected_grid_catalog_file_sha256=str(
                    manifest["grid_catalog"]["file_sha256"]
                ),
                expected_final_grid_file_sha256=str(
                    manifest["final_exact_G65_grid_gate"][
                        "report_file_sha256"
                    ]
                ),
                expected_final_grid_content_sha256=str(
                    final_scan_payload["report_content_sha256"]
                ),
                expected_pending_certificate_file_sha256=(
                    pending_certificate_file_hash
                ),
                expected_certificate=expected_certificate,
                expected_grid_catalog=grid_catalog,
                expected_final_scan=final_scan,
                families=families,
                expected_candidate_artifacts=candidate_disk_artifacts,
                barrier=final_barrier,
                controller=controller,
                full_v3_call_count=manifest["full_V3_call_count"],
                resource_gate_passed=True,
            )
        )
        if not disk_release_passed:
            manifest["publication_disk_release_gate"] = {
                "passed": False,
                "failures": list(disk_release_failures),
            }
            raise ValueError(
                "G10A3 publication disk release gate failed: "
                f"{disk_release_failures}"
            )
        certificate_path = finalize_g10a3_pending_certificate(
            pending_path=pending_certificate_path,
            certificate_path=final_certificate_path,
            expected_certificate=expected_certificate,
            controller=controller,
        )
        manifest["publication_disk_release_gate"] = {
            "passed": True,
            "failures": [],
            "pending_certificate_file_sha256": (
                pending_certificate_file_hash
            ),
            "final_certificate_reloaded_after_atomic_rename": True,
        }
        _record_g10a3_resource_sample(
            manifest=manifest,
            label="after_certificate_commit",
            matrix=master_matrix,
        )
        _check_g10a3_stage_deadline(stage_deadline=stage_deadline)
        manifest["certificate"] = {
            "file": certificate_path.name,
            "file_sha256": file_sha256(certificate_path),
            "atomic_promotion_from": pending_certificate_path.name,
        }
        _record_g10a3_resource_sample(
            manifest=manifest,
            label="before_ready_manifest",
            matrix=master_matrix,
        )
        _check_g10a3_stage_deadline(stage_deadline=stage_deadline)
        manifest["status"] = "ready"
        manifest["terminal_reason"] = (
            "unique_unchanged_V3_validated_after_complete_G65_grid_gate"
        )
        manifest["stage_wall_time_used_s"] = monotonic() - stage_started
        _save_manifest(manifest_path, manifest)
        return manifest_path
    except Exception as error:
        cleanup_failures = cleanup_g10a3_publication_artifacts(destination)
        manifest["status"] = "not_ready"
        manifest["terminal_reason"] = "G10A3_execution_error_fail_closed"
        manifest["execution_error"] = {
            "type": type(error).__name__,
            "message": str(error),
        }
        manifest["publication_artifact_cleanup"] = {
            "route_owned_pending_exists": pending_certificate_path.exists(),
            "route_owned_certificate_exists": final_certificate_path.exists(),
            "cleanup_failures": list(cleanup_failures),
            "fail_closed": True,
        }
        manifest["stage_wall_time_used_s"] = monotonic() - stage_started
        _save_manifest(manifest_path, manifest)
        return manifest_path
