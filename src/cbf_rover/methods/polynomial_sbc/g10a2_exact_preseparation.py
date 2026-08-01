"""G10A2 deterministic exact-Bernstein pre-separation.

G10A2 is an independent, finite W16 candidate-generation route.  It keeps the
same frozen controller, q, epsbar, 42 q-SBC residual families, and unchanged V3
validator as G10A.  Before the sole full V3 call, it scans a fixed exact dyadic
catalog and adds only selected *individual* Bernstein-coefficient inequalities.

The preseparator is never a publication authority.  Its exact arithmetic may
reject a candidate or decide that the finite D5 catalog is ready for V3, but it
cannot write a certificate.  Only a newly executed, subject-bound, zero-
tolerance V3 report may release a schema-v2 certificate.
"""

from __future__ import annotations

from collections.abc import Callable, Mapping, Sequence
from dataclasses import dataclass
from fractions import Fraction
import gc
import json
from math import comb, isfinite
from pathlib import Path
from time import monotonic
from typing import Any, Literal

import numpy as np
from scipy.sparse import csr_matrix, hstack, vstack

from cbf_rover.adapters.training.sac_reference import file_sha256

from .bernstein_lp import _polynomial_family_bernstein_matrix
from .certificate import QSBCertificate
from .chebyshev import (
    chebyshev_coefficients_to_monomial,
    total_degree_chebyshev_basis,
)
from .g10_sparse_multiplier_lp import (
    G10_COEFFICIENT_BOUND,
    G10_CONTROLLER_CONTENT_SHA256,
    G10_CONTROLLER_FILE_SHA256,
    G10_EPSBAR,
    G10_NUMERIC_FEASIBILITY_TOLERANCE,
    G10_Q,
    G10_VALIDATOR_ID,
    _assert_frozen_c2,
    _maximum_bound_violation,
    _save_manifest,
    _v3_release_gate,
    _write_json,
)
from .g10a_active_set_lp import (
    G10A_FEEDBACK_NUMERIC_MARGIN,
    G10A_FROZEN_QSBC_FAMILY_COUNT,
    G10A_G9_ROLE,
    _ActiveRowStore,
    _ConstraintFamily,
    _IncrementalOrRebuildSolver,
    _classify_box,
    _constraint_families,
    _dyadic_exact_boxes,
    _g9_ranked_rows,
    _load_g9_seed,
    _v3_feedback_binding_failures,
    _validate_candidate_v3,
)
from .polynomial import (
    FrozenController,
    Power2D,
    SparsePolynomial2D,
    content_sha256,
    controller_max_degree,
    controller_pieces,
    load_frozen_controller,
)
from .rational_certificate_validation import (
    RationalBox,
    _add as _rational_add,
    _constant as _rational_constant,
    _fraction_from_artifact,
    _fraction_text,
    _negate as _rational_negate,
    canonical_rational_model_payload,
    exact_expected_next_polynomial_rational,
    rational_sparse_polynomial,
)


G10A2_SYNTHESIS_ID = "G10A2_EXACT_BERNSTEIN_PRESEPARATION_LP_V1"
G10A2_PROOF_ROUTE_ID = "G10A2_D5_EXACT_PRESEPARATION_UNCHANGED_V3_V1"
G10A2_CATALOG_ID = "G10A2_FIXED_DYADIC_CATALOG_D3_D4_D5_V1"
G10A2_SPLIT_RULE_ID = "widest_axis_x_on_tie_exact_midpoint"
G10A2_EXACT_GATE_DEPTH = 5
G10A2_MAX_FULL_V3_CALLS = 1
G10A2_PRESEPARATOR_TIME_LIMIT_S = 3600.0
G10A2_AGGREGATE_LP_TIME_LIMIT_S = 1800.0
G10A2_STAGE_WALL_TIME_LIMIT_S = 10800.0
G10A2_MAX_RSS_BYTES = 1024**3
G10A2_MAX_HARD_CUT_ROWS = 7056
G10A2_MAX_MASTER_LP_ROWS = 7710
G10A2_MAX_EPHEMERAL_LP_ROWS = 7711
G10A2_MAX_CSR_NONZEROS = 1_087_873
G10A2_MAX_DECISION_VARIABLES = 308
G10A2_LEXICOGRAPHIC_T_TOLERANCE = 1.0e-9


@dataclass(frozen=True, slots=True)
class G10A2StageSpec:
    stage_id: str
    barrier_degree: int
    max_lp_solves: int
    aggregate_lp_time_limit_s: float
    aggregate_preseparator_time_limit_s: float
    stage_wall_time_limit_s: float

    def __post_init__(self) -> None:
        if self.barrier_degree != 16:
            raise ValueError("G10A2 first slice is frozen to W16")
        if self.max_lp_solves != 8:
            raise ValueError("G10A2 requires four fixed two-phase LP candidates")
        limits = (
            self.aggregate_lp_time_limit_s,
            self.aggregate_preseparator_time_limit_s,
            self.stage_wall_time_limit_s,
        )
        if any(not isfinite(value) or value <= 0.0 for value in limits):
            raise ValueError("G10A2 time limits must be positive and finite")


@dataclass(frozen=True, slots=True)
class G10A2PassSpec:
    pass_id: str
    depth: int
    max_new_cuts_per_active_box: int

    def __post_init__(self) -> None:
        if self.depth not in (3, 4, 5):
            raise ValueError("G10A2 pass depth must be 3, 4, or 5")
        if self.max_new_cuts_per_active_box <= 0:
            raise ValueError("G10A2 cut cap must be positive")


G10A2_STAGE = G10A2StageSpec(
    stage_id="W16_EXACT_PRESEPARATION",
    barrier_degree=16,
    max_lp_solves=8,
    aggregate_lp_time_limit_s=G10A2_AGGREGATE_LP_TIME_LIMIT_S,
    aggregate_preseparator_time_limit_s=G10A2_PRESEPARATOR_TIME_LIMIT_S,
    stage_wall_time_limit_s=G10A2_STAGE_WALL_TIME_LIMIT_S,
)
G10A2_PASS_SCHEDULE = (
    G10A2PassSpec("D3_ONE_PER_BOX", 3, 1),
    G10A2PassSpec("D4_TWO_PER_BOX", 4, 2),
    G10A2PassSpec("D5_FOUR_PER_BOX", 5, 4),
)


DomainClass = Literal["inside", "outside", "boundary"]
BernsteinTensor = tuple[tuple[Fraction, ...], ...]


class G10A2PreseparatorBudgetExceeded(RuntimeError):
    """Raised before a partial scan can be mistaken for a complete gate."""


@dataclass(frozen=True, slots=True)
class G10A2ExactCut:
    family_name: str
    family_contract_sha256: str
    box_exact: RationalBox
    box_sha256: str
    depth: int
    domain_class: DomainClass
    bernstein_index: tuple[int, int]
    target_degrees: tuple[int, int]
    exact_candidate_coefficient: Fraction
    candidate_barrier_sha256: str
    constraint_definition_id: str
    selection_event_sha256: str
    classification: str
    rho_coefficient_is_zero: bool = True
    formal_claim_allowed: bool = False

    def payload(self) -> dict[str, Any]:
        return {
            "family_name": self.family_name,
            "family_contract_sha256": self.family_contract_sha256,
            "box_exact": [_fraction_text(value) for value in self.box_exact],
            "box_sha256": self.box_sha256,
            "depth": self.depth,
            "domain_class": self.domain_class,
            "bernstein_index": list(self.bernstein_index),
            "target_degrees": list(self.target_degrees),
            "exact_candidate_coefficient": _fraction_text(
                self.exact_candidate_coefficient
            ),
            "candidate_barrier_sha256": self.candidate_barrier_sha256,
            "constraint_definition_id": self.constraint_definition_id,
            "selection_event_sha256": self.selection_event_sha256,
            "classification": self.classification,
            "rho_coefficient_float64_hex": 0.0.hex(),
            "formal_claim_allowed": False,
        }


@dataclass(frozen=True, slots=True)
class G10A2ScanResult:
    family_name: str
    depth: int
    target_degrees: tuple[int, int]
    visited_leaf_count: int
    active_leaf_count: int
    inspected_coefficient_count: int
    negative_coefficient_count: int
    minimum_exact_coefficient: Fraction | None
    scanned_leaf_identity_sha256s: tuple[str, ...]
    scanned_leaf_set_sha256: str
    selected_cuts: tuple[G10A2ExactCut, ...]
    gate_passed: bool
    peak_rss_bytes: int | None = None
    formal_claim_allowed: bool = False

    def payload(self) -> dict[str, Any]:
        return {
            "family_name": self.family_name,
            "depth": self.depth,
            "target_degrees": list(self.target_degrees),
            "visited_leaf_count": self.visited_leaf_count,
            "active_leaf_count": self.active_leaf_count,
            "inspected_coefficient_count": self.inspected_coefficient_count,
            "negative_coefficient_count": self.negative_coefficient_count,
            "minimum_exact_coefficient": (
                None
                if self.minimum_exact_coefficient is None
                else _fraction_text(self.minimum_exact_coefficient)
            ),
            "scanned_leaf_identity_sha256s": list(
                self.scanned_leaf_identity_sha256s
            ),
            "scanned_leaf_set_sha256": self.scanned_leaf_set_sha256,
            "selected_cut_count": len(self.selected_cuts),
            "selected_cuts": [cut.payload() for cut in self.selected_cuts],
            "gate_passed": self.gate_passed,
            "peak_rss_bytes": self.peak_rss_bytes,
            "formal_claim_allowed": False,
        }


@dataclass(frozen=True, slots=True)
class G10A2GeometryCatalog:
    payload: Mapping[str, Any]
    family_contract_sha256: str
    content_sha256: str
    depth_box_counts: Mapping[str, int]
    expected_leaf_set_sha256_by_depth: Mapping[str, str]
    total_box_count: int
    formal_claim_allowed: bool = False


@dataclass(frozen=True, slots=True)
class G10A2ProjectionProblem:
    """Frozen two-phase numeric projection layout.

    Core candidate columns are ``[c, rho]``.  The solver-only suffix is
    ``[t, z_0, ..., z_(n-1)]``.  Formal V3 never reads the suffix.
    """

    coefficient_count: int
    rho_index: int
    t_index: int
    z_start_index: int
    variable_count: int
    matrix: csr_matrix
    right_hand_side: np.ndarray
    primary_objective: np.ndarray
    secondary_objective: np.ndarray
    bounds: tuple[tuple[float, float], ...]
    seed_values: np.ndarray
    regularizer_row_count: int


def build_g10a2_projection_problem(
    *,
    seed_coefficients: Sequence[float],
    anchor_core_matrix: csr_matrix,
    anchor_right_hand_side: np.ndarray,
) -> G10A2ProjectionProblem:
    """Build min-Linf then min-L1 projection rows around the frozen G9 seed."""

    seed = np.asarray(seed_coefficients, dtype=np.float64)
    coefficient_count = int(seed.size)
    if coefficient_count <= 0 or not np.isfinite(seed).all():
        raise ValueError("G10A2 seed coefficient vector is invalid")
    if np.max(np.abs(seed)) > G10_COEFFICIENT_BOUND:
        raise ValueError("G10A2 G9 seed lies outside the frozen coefficient box")
    core = anchor_core_matrix.tocsr()
    anchor_rhs = np.asarray(anchor_right_hand_side, dtype=np.float64)
    if core.shape != (anchor_rhs.size, coefficient_count + 1):
        raise ValueError("G10A2 anchor core layout must be [c, rho]")
    if not np.isfinite(core.data).all() or not np.isfinite(anchor_rhs).all():
        raise ValueError("G10A2 anchor system contains a non-finite value")

    rho_index = coefficient_count
    t_index = rho_index + 1
    z_start = t_index + 1
    variable_count = z_start + coefficient_count
    suffix_width = variable_count - core.shape[1]
    extended_anchors = hstack(
        (
            core,
            csr_matrix((core.shape[0], suffix_width), dtype=np.float64),
        ),
        format="csr",
    )
    anchor_suffix = extended_anchors[:, t_index:].toarray()
    if any(value.hex() != 0.0.hex() for value in anchor_suffix.flat):
        raise ValueError("G10A2 anchor t/z columns are not bitwise zero")

    regularizer = np.zeros(
        (4 * coefficient_count, variable_count),
        dtype=np.float64,
    )
    regularizer_rhs = np.zeros(4 * coefficient_count, dtype=np.float64)
    for index, seed_value in enumerate(seed):
        row = 4 * index
        regularizer[row, index] = 1.0
        regularizer[row, t_index] = -1.0
        regularizer_rhs[row] = seed_value
        regularizer[row + 1, index] = -1.0
        regularizer[row + 1, t_index] = -1.0
        regularizer_rhs[row + 1] = -seed_value
        regularizer[row + 2, index] = 1.0
        regularizer[row + 2, z_start + index] = -1.0
        regularizer_rhs[row + 2] = seed_value
        regularizer[row + 3, index] = -1.0
        regularizer[row + 3, z_start + index] = -1.0
        regularizer_rhs[row + 3] = -seed_value

    matrix = vstack((extended_anchors, csr_matrix(regularizer)), format="csr")
    right_hand_side = np.concatenate((anchor_rhs, regularizer_rhs))
    primary = np.zeros(variable_count, dtype=np.float64)
    primary[t_index] = 1.0
    secondary = np.zeros(variable_count, dtype=np.float64)
    secondary[z_start:] = 1.0
    deviation_bound = 2.0 * G10_COEFFICIENT_BOUND
    bounds = (
        *((-G10_COEFFICIENT_BOUND, G10_COEFFICIENT_BOUND) for _ in seed),
        (0.0, 0.0),
        (0.0, deviation_bound),
        *((0.0, deviation_bound) for _ in seed),
    )
    seed_values = np.zeros(variable_count, dtype=np.float64)
    seed_values[:coefficient_count] = seed
    return G10A2ProjectionProblem(
        coefficient_count=coefficient_count,
        rho_index=rho_index,
        t_index=t_index,
        z_start_index=z_start,
        variable_count=variable_count,
        matrix=matrix,
        right_hand_side=right_hand_side,
        primary_objective=primary,
        secondary_objective=secondary,
        bounds=tuple(bounds),
        seed_values=seed_values,
        regularizer_row_count=4 * coefficient_count,
    )


def _peak_rss_bytes() -> int | None:
    """Read the process RSS high-water mark from Linux ``/proc``.

    A current-RSS sample can miss a transient allocation made by the unchanged
    V3 validator.  ``VmHWM`` is monotone for the process lifetime, so every
    later resource gate also covers all earlier LP, exact-scan, and V3 work.
    """

    status_path = Path("/proc/self/status")
    if not status_path.is_file():
        return None
    for line in status_path.read_text(encoding="utf-8").splitlines():
        if line.startswith("VmHWM:"):
            fields = line.split()
            if len(fields) == 3 and fields[2] == "kB":
                return int(fields[1]) * 1024
    return None


def g10a2_resource_failures(
    matrix: csr_matrix,
    *,
    rss_bytes: int | None,
    ephemeral_lexicographic_row: bool = False,
) -> tuple[str, ...]:
    """Return hard resource-cap failures for one concrete solver matrix."""

    row_limit = (
        G10A2_MAX_EPHEMERAL_LP_ROWS
        if ephemeral_lexicographic_row
        else G10A2_MAX_MASTER_LP_ROWS
    )
    checks = {
        "decision_variable_cap": matrix.shape[1] <= G10A2_MAX_DECISION_VARIABLES,
        "linear_row_cap": matrix.shape[0] <= row_limit,
        "CSR_nonzero_cap": matrix.nnz <= G10A2_MAX_CSR_NONZEROS,
        "RSS_measurement_available": rss_bytes is not None,
        "RSS_cap": rss_bytes is not None and rss_bytes <= G10A2_MAX_RSS_BYTES,
    }
    return tuple(name for name, passed in checks.items() if not passed)


def _validate_sha256(value: str, *, name: str) -> None:
    if len(value) != 64 or any(character not in "0123456789abcdef" for character in value):
        raise ValueError(f"{name} must be a lowercase SHA-256 digest")


def g10a2_leaf_set_sha256(
    *,
    depth: int,
    leaf_identity_sha256s: Sequence[str],
) -> str:
    """Hash one canonical exact-dyadic leaf identity set."""

    if depth < 0:
        raise ValueError("G10A2 leaf-set depth must be nonnegative")
    hashes = tuple(str(value) for value in leaf_identity_sha256s)
    for value in hashes:
        _validate_sha256(value, name="leaf_identity_sha256")
    if len(set(hashes)) != len(hashes):
        raise ValueError("G10A2 leaf identity set contains a duplicate")
    material = {
        "schema_version": 1,
        "kind": "g10a2_exact_dyadic_scanned_leaf_set",
        "catalog_id": G10A2_CATALOG_ID,
        "split_rule_id": G10A2_SPLIT_RULE_ID,
        "depth": depth,
        "leaf_count": len(hashes),
        "leaf_identity_sha256s": sorted(hashes),
    }
    return content_sha256(material)


def g10a2_leaf_set_binding_gate(
    *,
    expected_leaf_set_sha256: str,
    scanned_leaf_set_sha256: str,
    expected_leaf_count: int,
    scanned_leaf_count: int,
) -> tuple[bool, tuple[str, ...]]:
    """Fail closed unless runtime traversal matches the frozen leaf identity set."""

    checks = {
        "expected_leaf_set_sha256_format": (
            len(expected_leaf_set_sha256) == 64
            and all(
                character in "0123456789abcdef"
                for character in expected_leaf_set_sha256
            )
        ),
        "scanned_leaf_set_sha256_format": (
            len(scanned_leaf_set_sha256) == 64
            and all(
                character in "0123456789abcdef"
                for character in scanned_leaf_set_sha256
            )
        ),
        "leaf_count": scanned_leaf_count == expected_leaf_count,
        "leaf_identity_set": (
            scanned_leaf_set_sha256 == expected_leaf_set_sha256
        ),
    }
    failures = tuple(name for name, passed in checks.items() if not passed)
    return not failures, failures


def exact_bernstein_tensor(
    polynomial: Mapping[Power2D, Fraction],
    box: RationalBox,
    *,
    target_degree_x: int | None = None,
    target_degree_y: int | None = None,
) -> BernsteinTensor:
    """Return the exact indexed tensor-Bernstein coefficients on ``box``."""

    x_low, x_high, y_low, y_high = box
    if not x_low < x_high or not y_low < y_high:
        raise ValueError("G10A2 Bernstein box must have positive width")
    actual_degree_x = max((power[0] for power in polynomial), default=0)
    actual_degree_y = max((power[1] for power in polynomial), default=0)
    degree_x = actual_degree_x if target_degree_x is None else target_degree_x
    degree_y = actual_degree_y if target_degree_y is None else target_degree_y
    if degree_x < actual_degree_x or degree_y < actual_degree_y:
        raise ValueError("G10A2 target degree cannot be below the polynomial degree")

    width_x = x_high - x_low
    width_y = y_high - y_low
    local: dict[Power2D, Fraction] = {}
    for (x_power, y_power), coefficient in polynomial.items():
        if coefficient == 0:
            continue
        for local_x in range(x_power + 1):
            x_factor = (
                comb(x_power, local_x)
                * x_low ** (x_power - local_x)
                * width_x**local_x
            )
            for local_y in range(y_power + 1):
                power = (local_x, local_y)
                factor = (
                    coefficient
                    * x_factor
                    * comb(y_power, local_y)
                    * y_low ** (y_power - local_y)
                    * width_y**local_y
                )
                local[power] = local.get(power, Fraction(0)) + factor

    rows: list[tuple[Fraction, ...]] = []
    for bernstein_x in range(degree_x + 1):
        row: list[Fraction] = []
        for bernstein_y in range(degree_y + 1):
            value = Fraction(0)
            for local_x in range(bernstein_x + 1):
                x_ratio = Fraction(
                    comb(bernstein_x, local_x),
                    comb(degree_x, local_x),
                )
                for local_y in range(bernstein_y + 1):
                    value += (
                        local.get((local_x, local_y), Fraction(0))
                        * x_ratio
                        * Fraction(
                            comb(bernstein_y, local_y),
                            comb(degree_y, local_y),
                        )
                    )
            row.append(value)
        rows.append(tuple(row))
    return tuple(rows)


def _split_exact_bernstein_line(
    values: Sequence[Fraction],
) -> tuple[tuple[Fraction, ...], tuple[Fraction, ...]]:
    if not values:
        raise ValueError("G10A2 cannot split an empty Bernstein line")
    work = list(values)
    left = [work[0]]
    right = [work[-1]]
    while len(work) > 1:
        work = [
            (work[index] + work[index + 1]) / 2
            for index in range(len(work) - 1)
        ]
        left.append(work[0])
        right.append(work[-1])
    return tuple(left), tuple(reversed(right))


def split_exact_bernstein_tensor(
    tensor: BernsteinTensor,
    *,
    axis: Literal["x", "y"],
) -> tuple[BernsteinTensor, BernsteinTensor]:
    """Split an exact Bernstein tensor at the selected axis midpoint."""

    if not tensor or not tensor[0]:
        raise ValueError("G10A2 cannot split an empty Bernstein tensor")
    width_y = len(tensor[0])
    if any(len(row) != width_y for row in tensor):
        raise ValueError("G10A2 Bernstein tensor must be rectangular")
    if axis == "y":
        left_rows: list[tuple[Fraction, ...]] = []
        right_rows: list[tuple[Fraction, ...]] = []
        for row in tensor:
            left, right = _split_exact_bernstein_line(row)
            left_rows.append(left)
            right_rows.append(right)
        return tuple(left_rows), tuple(right_rows)
    if axis != "x":
        raise ValueError("G10A2 Bernstein split axis must be x or y")

    left_columns: list[tuple[Fraction, ...]] = []
    right_columns: list[tuple[Fraction, ...]] = []
    for y_index in range(width_y):
        line = tuple(row[y_index] for row in tensor)
        left, right = _split_exact_bernstein_line(line)
        left_columns.append(left)
        right_columns.append(right)
    left_tensor = tuple(
        tuple(left_columns[y_index][x_index] for y_index in range(width_y))
        for x_index in range(len(tensor))
    )
    right_tensor = tuple(
        tuple(right_columns[y_index][x_index] for y_index in range(width_y))
        for x_index in range(len(tensor))
    )
    return left_tensor, right_tensor


def _split_box_and_tensor(
    box: RationalBox,
    tensor: BernsteinTensor,
) -> tuple[tuple[RationalBox, BernsteinTensor], ...]:
    x_low, x_high, y_low, y_high = box
    if x_high - x_low >= y_high - y_low:
        midpoint = (x_low + x_high) / 2
        left, right = split_exact_bernstein_tensor(tensor, axis="x")
        return (
            ((x_low, midpoint, y_low, y_high), left),
            ((midpoint, x_high, y_low, y_high), right),
        )
    midpoint = (y_low + y_high) / 2
    left, right = split_exact_bernstein_tensor(tensor, axis="y")
    return (
        ((x_low, x_high, y_low, midpoint), left),
        ((x_low, x_high, midpoint, y_high), right),
    )


def _box_payload(
    *,
    family_name: str,
    box: RationalBox,
    depth: int,
    domain_class: str,
) -> dict[str, Any]:
    return {
        "schema_version": 1,
        "kind": "g10a2_exact_dyadic_leaf_identity",
        "family_name": family_name,
        "box_exact": [_fraction_text(value) for value in box],
        "depth": depth,
        "domain_class": domain_class,
        "split_rule_id": G10A2_SPLIT_RULE_ID,
    }


def _constraint_definition_material(
    *,
    family_contract_sha256: str,
    family_name: str,
    box_sha256: str,
    bernstein_index: tuple[int, int],
    target_degrees: tuple[int, int],
) -> dict[str, Any]:
    return {
        "schema_version": 1,
        "kind": "g10a2_exact_bernstein_coefficient_constraint_definition",
        "family_contract_sha256": family_contract_sha256,
        "family_name": family_name,
        "box_sha256": box_sha256,
        "bernstein_index": list(bernstein_index),
        "target_degrees": list(target_degrees),
        "numeric_margin_float64_hex": G10A_FEEDBACK_NUMERIC_MARGIN.hex(),
        "rho_coefficient_float64_hex": 0.0.hex(),
    }


def _selection_event_material(
    *,
    constraint_definition_id: str,
    candidate_barrier_sha256: str,
    exact_candidate_coefficient: Fraction | str,
    classification: str,
) -> dict[str, Any]:
    coefficient_text = (
        exact_candidate_coefficient
        if isinstance(exact_candidate_coefficient, str)
        else _fraction_text(exact_candidate_coefficient)
    )
    return {
        "schema_version": 1,
        "kind": "g10a2_exact_bernstein_coefficient_selection_event",
        "constraint_definition_id": constraint_definition_id,
        "candidate_barrier_sha256": candidate_barrier_sha256,
        "exact_candidate_coefficient": coefficient_text,
        "classification": classification,
    }


def scan_exact_bernstein_catalog(
    *,
    family_name: str,
    family_contract_sha256: str,
    polynomial: Mapping[Power2D, Fraction],
    root_box: RationalBox,
    classify_domain: Callable[[RationalBox], str],
    depth: int,
    max_new_cuts_per_active_box: int,
    candidate_barrier_sha256: str,
    target_degree_x: int | None = None,
    target_degree_y: int | None = None,
    existing_constraint_definition_ids: frozenset[str] = frozenset(),
    deadline: float | None = None,
    clock: Callable[[], float] = monotonic,
    rss_reader: Callable[[], int | None] | None = None,
    rss_limit_bytes: int = G10A2_MAX_RSS_BYTES,
) -> G10A2ScanResult:
    """Scan one complete fixed-depth catalog and select coefficient cuts.

    Selection is deterministic and exact.  A scan that exceeds ``deadline``
    raises before returning, so a partial traversal can never pass the gate.
    """

    _validate_sha256(family_contract_sha256, name="family_contract_sha256")
    _validate_sha256(candidate_barrier_sha256, name="candidate_barrier_sha256")
    if depth < 0:
        raise ValueError("G10A2 catalog depth must be nonnegative")
    if max_new_cuts_per_active_box < 0:
        raise ValueError("G10A2 cut cap must be nonnegative")
    if deadline is not None and clock() > deadline:
        raise G10A2PreseparatorBudgetExceeded(
            "G10A2 exact preseparator budget expired before traversal"
        )

    root_tensor = exact_bernstein_tensor(
        polynomial,
        root_box,
        target_degree_x=target_degree_x,
        target_degree_y=target_degree_y,
    )
    degrees = (len(root_tensor) - 1, len(root_tensor[0]) - 1)
    stack: list[tuple[RationalBox, int, BernsteinTensor]] = [
        (root_box, 0, root_tensor)
    ]
    selected: list[G10A2ExactCut] = []
    visited_leaf_count = 0
    active_leaf_count = 0
    inspected_coefficient_count = 0
    negative_coefficient_count = 0
    minimum: Fraction | None = None
    peak_rss_bytes: int | None = None
    scanned_leaf_identity_sha256s: list[str] = []
    while stack:
        if deadline is not None and clock() > deadline:
            raise G10A2PreseparatorBudgetExceeded(
                "G10A2 exact preseparator budget expired during traversal"
            )
        if rss_reader is not None:
            rss_bytes = rss_reader()
            if rss_bytes is None:
                raise G10A2PreseparatorBudgetExceeded(
                    "G10A2 RSS measurement unavailable during exact traversal"
                )
            peak_rss_bytes = (
                rss_bytes
                if peak_rss_bytes is None
                else max(peak_rss_bytes, rss_bytes)
            )
            if rss_bytes > rss_limit_bytes:
                raise G10A2PreseparatorBudgetExceeded(
                    "G10A2 RSS cap exceeded during exact traversal"
                )
        box, current_depth, tensor = stack.pop()
        if current_depth < depth:
            children = _split_box_and_tensor(box, tensor)
            stack.append((children[1][0], current_depth + 1, children[1][1]))
            stack.append((children[0][0], current_depth + 1, children[0][1]))
            continue

        visited_leaf_count += 1
        domain_class = str(classify_domain(box))
        if domain_class not in ("inside", "outside", "boundary"):
            raise ValueError("G10A2 domain classifier returned an invalid class")
        box_material = _box_payload(
            family_name=family_name,
            box=box,
            depth=depth,
            domain_class=domain_class,
        )
        box_hash = content_sha256(box_material)
        scanned_leaf_identity_sha256s.append(box_hash)
        if domain_class == "outside":
            continue
        active_leaf_count += 1
        flattened = [
            (value, x_index, y_index)
            for x_index, row in enumerate(tensor)
            for y_index, value in enumerate(row)
        ]
        inspected_coefficient_count += len(flattened)
        leaf_minimum = min(value for value, _, _ in flattened)
        leaf_maximum = max(value for value, _, _ in flattened)
        minimum = leaf_minimum if minimum is None else min(minimum, leaf_minimum)
        negatives = sorted(item for item in flattened if item[0] < 0)
        negative_coefficient_count += len(negatives)
        if max_new_cuts_per_active_box == 0:
            continue
        accepted_on_box = 0
        for value, x_index, y_index in negatives:
            definition_material = _constraint_definition_material(
                family_contract_sha256=family_contract_sha256,
                family_name=family_name,
                box_sha256=box_hash,
                bernstein_index=(x_index, y_index),
                target_degrees=degrees,
            )
            definition_id = content_sha256(definition_material)
            if definition_id in existing_constraint_definition_ids:
                continue
            classification = (
                "proved_candidate_violation"
                if domain_class == "inside" and leaf_maximum < 0
                else "negative_bernstein_lower_bound"
            )
            selection_event_sha256 = content_sha256(
                _selection_event_material(
                    constraint_definition_id=definition_id,
                    candidate_barrier_sha256=candidate_barrier_sha256,
                    exact_candidate_coefficient=value,
                    classification=classification,
                )
            )
            selected.append(
                G10A2ExactCut(
                    family_name=family_name,
                    family_contract_sha256=family_contract_sha256,
                    box_exact=box,
                    box_sha256=box_hash,
                    depth=depth,
                    domain_class=domain_class,  # type: ignore[arg-type]
                    bernstein_index=(x_index, y_index),
                    target_degrees=degrees,
                    exact_candidate_coefficient=value,
                    candidate_barrier_sha256=candidate_barrier_sha256,
                    constraint_definition_id=definition_id,
                    selection_event_sha256=selection_event_sha256,
                    classification=classification,
                )
            )
            accepted_on_box += 1
            if accepted_on_box >= max_new_cuts_per_active_box:
                break

    selected.sort(
        key=lambda cut: (
            cut.exact_candidate_coefficient,
            cut.family_name,
            tuple(cut.box_exact),
            cut.bernstein_index,
        )
    )
    canonical_scanned_hashes = tuple(sorted(scanned_leaf_identity_sha256s))
    scanned_leaf_set_sha256 = g10a2_leaf_set_sha256(
        depth=depth,
        leaf_identity_sha256s=canonical_scanned_hashes,
    )
    return G10A2ScanResult(
        family_name=family_name,
        depth=depth,
        target_degrees=degrees,
        visited_leaf_count=visited_leaf_count,
        active_leaf_count=active_leaf_count,
        inspected_coefficient_count=inspected_coefficient_count,
        negative_coefficient_count=negative_coefficient_count,
        minimum_exact_coefficient=minimum,
        scanned_leaf_identity_sha256s=canonical_scanned_hashes,
        scanned_leaf_set_sha256=scanned_leaf_set_sha256,
        selected_cuts=tuple(selected),
        gate_passed=(negative_coefficient_count == 0),
        peak_rss_bytes=peak_rss_bytes,
    )


def _family_target_degrees(family: _ConstraintFamily) -> tuple[int, int]:
    powers = {
        power
        for polynomial in family.basis
        for power in polynomial
    }
    if not powers:
        raise ValueError("G10A2 family basis must not be empty")
    return max(power[0] for power in powers), max(power[1] for power in powers)


def _family_contract_payload(
    families: Sequence[_ConstraintFamily],
) -> dict[str, Any]:
    return {
        "schema_version": 1,
        "kind": "g10a2_frozen_qsbc_family_contract",
        "synthesis_id": G10A2_SYNTHESIS_ID,
        "formal_model_contract_sha256": content_sha256(
            canonical_rational_model_payload()
        ),
        "controller_content_sha256": G10_CONTROLLER_CONTENT_SHA256,
        "family_count": len(families),
        "families": [
            {
                "name": family.name,
                "root_box_exact": [
                    _fraction_text(value) for value in family.root_box_exact
                ],
                "domain_kind": family.domain_kind,
                "piece_id": family.piece_id,
                "constant_float64_hex": family.constant_value.hex(),
                "target_degrees": list(_family_target_degrees(family)),
            }
            for family in families
        ],
    }


def build_g10a2_geometry_catalog(
    families: Sequence[_ConstraintFamily],
) -> G10A2GeometryCatalog:
    """Build the canonical D3/D4/D5 geometry catalog without a candidate W."""

    if not families:
        raise ValueError("G10A2 geometry catalog requires at least one family")
    if len({family.name for family in families}) != len(families):
        raise ValueError("G10A2 geometry catalog family names must be unique")
    family_contract = _family_contract_payload(families)
    family_contract_hash = content_sha256(family_contract)
    depth_box_counts: dict[str, int] = {}
    depth_payloads: dict[str, list[dict[str, Any]]] = {}
    expected_leaf_set_sha256_by_depth: dict[str, str] = {}
    for depth in (3, 4, 5):
        items: list[dict[str, Any]] = []
        for family in families:
            for box in _dyadic_exact_boxes(family.root_box_exact, depth):
                domain_class = _classify_box(family, box)
                material = _box_payload(
                    family_name=family.name,
                    box=box,
                    depth=depth,
                    domain_class=domain_class,
                )
                material["box_sha256"] = content_sha256(material)
                items.append(material)
        key = str(depth)
        depth_box_counts[key] = len(items)
        depth_payloads[key] = items
        expected_leaf_set_sha256_by_depth[key] = g10a2_leaf_set_sha256(
            depth=depth,
            leaf_identity_sha256s=tuple(
                str(item["box_sha256"]) for item in items
            ),
        )
    root_set_payload = {
        "schema_version": 1,
        "kind": "g10a2_v3_qsbc_root_set",
        "roots": {
            family.name: [
                _fraction_text(value) for value in family.root_box_exact
            ]
            for family in families
        },
    }
    payload: dict[str, Any] = {
        "schema_version": 1,
        "kind": "g10a2_fixed_exact_dyadic_geometry_catalog",
        "catalog_id": G10A2_CATALOG_ID,
        "split_rule_id": G10A2_SPLIT_RULE_ID,
        "depths": [3, 4, 5],
        "family_contract": family_contract,
        "family_contract_sha256": family_contract_hash,
        "v3_root_set": root_set_payload,
        "v3_root_set_sha256": content_sha256(root_set_payload),
        "depth_box_counts": depth_box_counts,
        "expected_leaf_set_sha256_by_depth": (
            expected_leaf_set_sha256_by_depth
        ),
        "boxes_by_depth": depth_payloads,
        "formal_claim_allowed": False,
    }
    catalog_hash = content_sha256(payload)
    payload["catalog_content_sha256"] = catalog_hash
    return G10A2GeometryCatalog(
        payload=payload,
        family_contract_sha256=family_contract_hash,
        content_sha256=catalog_hash,
        depth_box_counts=depth_box_counts,
        expected_leaf_set_sha256_by_depth=(
            expected_leaf_set_sha256_by_depth
        ),
        total_box_count=sum(depth_box_counts.values()),
    )


def build_g10a2_numeric_cut_rows(
    *,
    family: _ConstraintFamily,
    cuts: Sequence[G10A2ExactCut],
    variable_count: int,
    rho_index: int,
) -> tuple[csr_matrix, np.ndarray]:
    """Project exact-selected coefficient cuts into the numeric candidate LP."""

    if (
        rho_index <= 0
        or rho_index >= variable_count
        or len(family.basis) != rho_index
    ):
        raise ValueError("G10A2 numeric cut variable layout mismatch")
    rows: list[np.ndarray] = []
    rhs: list[float] = []
    matrix_cache: dict[RationalBox, np.ndarray] = {}
    for cut in cuts:
        if cut.family_name != family.name:
            raise ValueError("G10A2 cut references a different family")
        if not cut.rho_coefficient_is_zero:
            raise ValueError("G10A2 hard cut must not read rho")
        local = matrix_cache.get(cut.box_exact)
        if local is None:
            local = _polynomial_family_bernstein_matrix(
                family.basis,
                tuple(float(value) for value in cut.box_exact),
            )
            matrix_cache[cut.box_exact] = local
        expected_degrees = _family_target_degrees(family)
        if cut.target_degrees != expected_degrees:
            raise ValueError("G10A2 cut target degree disagrees with family basis")
        flat_index = (
            cut.bernstein_index[0] * (expected_degrees[1] + 1)
            + cut.bernstein_index[1]
        )
        if flat_index < 0 or flat_index >= local.shape[0]:
            raise ValueError("G10A2 Bernstein coefficient index is out of range")
        row = np.zeros(variable_count, dtype=np.float64)
        row[:rho_index] = -local[flat_index]
        row[rho_index] = 0.0
        rows.append(row)
        rhs.append(family.constant_value - G10A_FEEDBACK_NUMERIC_MARGIN)
    if not rows:
        return (
            csr_matrix((0, variable_count), dtype=np.float64),
            np.empty(0, dtype=np.float64),
        )
    matrix = csr_matrix(np.vstack(rows))
    dense_tail = matrix[:, rho_index:].toarray()
    if any(value.hex() != 0.0.hex() for value in dense_tail.flat):
        raise ValueError("G10A2 hard-cut rho/t/z columns are not bitwise zero")
    return matrix, np.asarray(rhs, dtype=np.float64)


def g10a2_publication_gate(
    *,
    preseparator_gate_passed: bool,
    full_v3_call_count: int,
    validation_status: str | None,
    first_slice_ready: bool | None,
    binding_failures: Sequence[str],
    v3_release_gate_passed: bool,
) -> tuple[bool, tuple[str, ...]]:
    """Apply the G10A2-only outer gate around the unchanged V3 release gate."""

    checks = {
        "preseparator_gate_passed": preseparator_gate_passed is True,
        "exactly_one_full_v3_call": (
            full_v3_call_count == G10A2_MAX_FULL_V3_CALLS
        ),
        "V3_status": validation_status == "validated",
        "V3_first_slice_ready": first_slice_ready is True,
        "V3_subject_and_tree_bindings": not binding_failures,
        "unchanged_V3_release_gate": v3_release_gate_passed is True,
    }
    failures = tuple(name for name, passed in checks.items() if not passed)
    return not failures, failures


def _exact_candidate_residuals(
    *,
    barrier: SparsePolynomial2D,
    controller: FrozenController,
    families: Sequence[_ConstraintFamily],
) -> dict[str, Mapping[Power2D, Fraction]]:
    barrier_exact = rational_sparse_polynomial(barrier)
    expected_by_piece = {
        piece.piece_id: exact_expected_next_polynomial_rational(barrier, piece)
        for piece in controller_pieces(controller)
    }
    q_exact = _fraction_from_artifact(G10_Q)
    epsbar_exact = _fraction_from_artifact(G10_EPSBAR)
    residuals: dict[str, Mapping[Power2D, Fraction]] = {}
    for family in families:
        if family.name == "W_nonnegative_on_X":
            residual = barrier_exact
        elif family.name == "q_minus_W_on_X0":
            residual = _rational_add(
                _rational_constant(q_exact),
                _rational_negate(barrier_exact),
            )
        elif family.name.startswith("W_minus_1_on_"):
            residual = _rational_add(
                barrier_exact,
                _rational_constant(-1),
            )
        elif family.name.startswith("drift_") and family.piece_id is not None:
            residual = _rational_add(
                barrier_exact,
                _rational_negate(expected_by_piece[family.piece_id]),
                _rational_constant(-epsbar_exact),
            )
        else:
            raise ValueError("G10A2 encountered an unsupported q-SBC family")
        residuals[family.name] = residual
    expected_names = {family.name for family in families}
    if set(residuals) != expected_names:
        raise ValueError("G10A2 exact residual family set mismatch")
    return residuals


def _geometry_row_key(
    *,
    family_name: str,
    box: RationalBox,
    domain_class: str,
) -> tuple[str, tuple[str, ...], str]:
    return (
        family_name,
        tuple(_fraction_text(value) for value in box),
        domain_class,
    )


def _g9_depth3_geometry_binding(
    *,
    families: Sequence[_ConstraintFamily],
    anchors: Sequence[Any],
    dormant_rows: Sequence[Any],
) -> tuple[str, int]:
    expected: set[tuple[str, tuple[str, ...], str]] = set()
    for family in families:
        for box in _dyadic_exact_boxes(family.root_box_exact, 3):
            domain_class = _classify_box(family, box)
            if domain_class == "outside":
                continue
            midpoint_x = (box[0] + box[1]) / 2
            midpoint_y = (box[2] + box[3]) / 2
            midpoint_box = (
                midpoint_x,
                midpoint_x,
                midpoint_y,
                midpoint_y,
            )
            if _classify_box(family, midpoint_box) == "outside":
                continue
            expected.add(
                _geometry_row_key(
                    family_name=family.name,
                    box=box,
                    domain_class=domain_class,
                )
            )
    actual = {
        _geometry_row_key(
            family_name=row.family_name,
            box=row.box_exact,
            domain_class=row.domain_class,
        )
        for row in (*anchors, *dormant_rows)
    }
    if actual != expected:
        raise ValueError("G10A2 G9/preseparator D3 geometry binding mismatch")
    payload = {
        "schema_version": 1,
        "kind": "g10a2_g9_depth3_eligible_geometry",
        "split_rule_id": G10A2_SPLIT_RULE_ID,
        "items": [
            {
                "family_name": item[0],
                "box_exact": list(item[1]),
                "domain_class": item[2],
            }
            for item in sorted(expected)
        ],
    }
    return content_sha256(payload), len(expected)


def _scan_all_families(
    *,
    barrier: SparsePolynomial2D,
    controller: FrozenController,
    families: Sequence[_ConstraintFamily],
    family_contract_sha256: str,
    depth: int,
    max_new_cuts_per_active_box: int,
    existing_constraint_definition_ids: frozenset[str],
    expected_leaf_set_sha256: str,
    deadline: float,
    clock: Callable[[], float] = monotonic,
    v3_subject_mode: bool = False,
) -> tuple[dict[str, Any], tuple[G10A2ExactCut, ...], bool]:
    candidate_hash = content_sha256(barrier.payload())
    residuals = _exact_candidate_residuals(
        barrier=barrier,
        controller=controller,
        families=families,
    )
    scans: dict[str, dict[str, Any]] = {}
    selected: list[G10A2ExactCut] = []
    negative_count = 0
    visited_leaf_count = 0
    active_leaf_count = 0
    inspected_count = 0
    minimum: Fraction | None = None
    peak_rss_bytes: int | None = None
    scanned_leaf_identity_sha256s: list[str] = []
    for family in families:
        if clock() > deadline:
            raise G10A2PreseparatorBudgetExceeded(
                "G10A2 exact preseparator budget expired between families"
            )
        if v3_subject_mode:
            target_x = None
            target_y = None
        else:
            target_x, target_y = _family_target_degrees(family)
        scan = scan_exact_bernstein_catalog(
            family_name=family.name,
            family_contract_sha256=family_contract_sha256,
            polynomial=residuals[family.name],
            root_box=family.root_box_exact,
            classify_domain=lambda box, item=family: _classify_box(item, box),
            depth=depth,
            max_new_cuts_per_active_box=max_new_cuts_per_active_box,
            candidate_barrier_sha256=candidate_hash,
            target_degree_x=target_x,
            target_degree_y=target_y,
            existing_constraint_definition_ids=(
                existing_constraint_definition_ids
            ),
            deadline=deadline,
            clock=clock,
            rss_reader=_peak_rss_bytes,
            rss_limit_bytes=G10A2_MAX_RSS_BYTES,
        )
        scans[family.name] = scan.payload()
        selected.extend(scan.selected_cuts)
        scanned_leaf_identity_sha256s.extend(
            scan.scanned_leaf_identity_sha256s
        )
        negative_count += scan.negative_coefficient_count
        visited_leaf_count += scan.visited_leaf_count
        active_leaf_count += scan.active_leaf_count
        inspected_count += scan.inspected_coefficient_count
        if scan.minimum_exact_coefficient is not None:
            minimum = (
                scan.minimum_exact_coefficient
                if minimum is None
                else min(minimum, scan.minimum_exact_coefficient)
            )
        if scan.peak_rss_bytes is not None:
            peak_rss_bytes = (
                scan.peak_rss_bytes
                if peak_rss_bytes is None
                else max(peak_rss_bytes, scan.peak_rss_bytes)
            )
    selected.sort(
        key=lambda cut: (
            cut.exact_candidate_coefficient,
            cut.family_name,
            tuple(cut.box_exact),
            cut.bernstein_index,
        )
    )
    expected_visited_leaf_count = len(families) * (2**depth)
    scanned_leaf_set_sha256 = g10a2_leaf_set_sha256(
        depth=depth,
        leaf_identity_sha256s=scanned_leaf_identity_sha256s,
    )
    leaf_set_passed, leaf_set_failures = g10a2_leaf_set_binding_gate(
        expected_leaf_set_sha256=expected_leaf_set_sha256,
        scanned_leaf_set_sha256=scanned_leaf_set_sha256,
        expected_leaf_count=expected_visited_leaf_count,
        scanned_leaf_count=visited_leaf_count,
    )
    if not leaf_set_passed:
        raise ValueError(
            "G10A2 exact scan leaf-set provenance mismatch: "
            f"{leaf_set_failures}"
        )
    gate_passed = negative_count == 0
    payload: dict[str, Any] = {
        "schema_version": 1,
        "kind": "g10a2_exact_bernstein_preseparation_report",
        "synthesis_id": G10A2_SYNTHESIS_ID,
        "candidate_barrier_sha256": candidate_hash,
        "family_contract_sha256": family_contract_sha256,
        "catalog_depth": depth,
        "mode": (
            "V3_subject_exact_degree_gate"
            if v3_subject_mode
            else "fixed_degree_linear_cut_selection"
        ),
        "complete_catalog_traversal_required": True,
        "visited_leaf_count": visited_leaf_count,
        "expected_visited_leaf_count": expected_visited_leaf_count,
        "whole_catalog_traversed": True,
        "expected_leaf_set_sha256": expected_leaf_set_sha256,
        "scanned_leaf_set_sha256": scanned_leaf_set_sha256,
        "leaf_set_binding_passed": True,
        "active_leaf_count": active_leaf_count,
        "inspected_coefficient_count": inspected_count,
        "negative_coefficient_count": negative_count,
        "minimum_exact_coefficient": (
            None if minimum is None else _fraction_text(minimum)
        ),
        "peak_rss_bytes": peak_rss_bytes,
        "selected_cut_count": len(selected),
        "gate_passed": gate_passed,
        "families": scans,
        "formal_claim_allowed": False,
    }
    payload["report_content_sha256"] = content_sha256(payload)
    return payload, tuple(selected), gate_passed


def _build_bound_numeric_cut_rows(
    *,
    families: Sequence[_ConstraintFamily],
    residuals: Mapping[str, Mapping[Power2D, Fraction]],
    cuts: Sequence[G10A2ExactCut],
    candidate_barrier_sha256: str,
    candidate_chebyshev_coefficients: Sequence[float],
    variable_count: int,
    rho_index: int,
    rss_reader: Callable[[], int | None] | None = None,
) -> tuple[csr_matrix, np.ndarray, tuple[dict[str, Any], ...]]:
    family_by_name = {family.name: family for family in families}
    candidate_coefficients = np.asarray(
        candidate_chebyshev_coefficients,
        dtype=np.float64,
    )
    if (
        candidate_coefficients.shape != (rho_index,)
        or not np.isfinite(candidate_coefficients).all()
    ):
        raise ValueError("G10A2 numeric cut replay candidate is invalid")
    blocks: list[csr_matrix] = []
    bounds: list[np.ndarray] = []
    ledger_rows: list[dict[str, Any]] = []
    for family_name in sorted({cut.family_name for cut in cuts}):
        exact_tensor_cache: dict[
            tuple[str, RationalBox, tuple[int, int]], BernsteinTensor
        ] = {}
        family = family_by_name[family_name]
        family_cuts = tuple(cut for cut in cuts if cut.family_name == family_name)
        for cut in family_cuts:
            if cut.candidate_barrier_sha256 != candidate_barrier_sha256:
                raise ValueError("G10A2 cut candidate-subject hash mismatch")
            cache_key = (family_name, cut.box_exact, cut.target_degrees)
            tensor = exact_tensor_cache.get(cache_key)
            if tensor is None:
                tensor = exact_bernstein_tensor(
                    residuals[family_name],
                    cut.box_exact,
                    target_degree_x=cut.target_degrees[0],
                    target_degree_y=cut.target_degrees[1],
                )
                exact_tensor_cache[cache_key] = tensor
                if rss_reader is not None:
                    rss_bytes = rss_reader()
                    if rss_bytes is None or rss_bytes > G10A2_MAX_RSS_BYTES:
                        raise G10A2PreseparatorBudgetExceeded(
                            "G10A2 RSS gate failed during exact cut replay"
                        )
            recomputed = tensor[cut.bernstein_index[0]][cut.bernstein_index[1]]
            if recomputed != cut.exact_candidate_coefficient:
                raise ValueError("G10A2 exact selected coefficient did not replay")
        matrix, rhs = build_g10a2_numeric_cut_rows(
            family=family,
            cuts=family_cuts,
            variable_count=variable_count,
            rho_index=rho_index,
        )
        blocks.append(matrix)
        bounds.append(rhs)
        for index, cut in enumerate(family_cuts):
            dense_row = matrix.getrow(index).toarray().ravel()
            if any(
                value.hex() != 0.0.hex()
                for value in dense_row[rho_index:]
            ):
                raise ValueError(
                    "G10A2 numeric hard-cut rho/t/z is not bitwise zero"
                )
            numeric_payload = {
                "coefficients_float64_hex": [
                    float(value).hex() for value in dense_row
                ],
                "right_hand_side_float64_hex": float(rhs[index]).hex(),
            }
            numeric_candidate_coefficient = float(
                family.constant_value
                - dense_row[:rho_index] @ candidate_coefficients
            )
            exact_candidate_coefficient_float = float(
                cut.exact_candidate_coefficient
            )
            exact_to_float_error = abs(
                numeric_candidate_coefficient
                - exact_candidate_coefficient_float
            )
            projection_scale = float(
                abs(family.constant_value)
                + np.sum(
                    np.abs(
                        dense_row[:rho_index] * candidate_coefficients
                    )
                )
            )
            exact_to_float_tolerance = max(
                1.0e-9 * (1.0 + abs(exact_candidate_coefficient_float)),
                512.0 * np.finfo(np.float64).eps * (1.0 + projection_scale),
            )
            cut_excludes_source_candidate = (
                numeric_candidate_coefficient
                < G10A_FEEDBACK_NUMERIC_MARGIN
                - G10_NUMERIC_FEASIBILITY_TOLERANCE
            )
            if (
                exact_to_float_error > exact_to_float_tolerance
                or not cut_excludes_source_candidate
            ):
                raise ValueError(
                    "G10A2 exact-selected coefficient failed float projection replay"
                )
            row_payload = cut.payload()
            row_payload.update(
                {
                    "exact_candidate_coefficient_recomputed": _fraction_text(
                        cut.exact_candidate_coefficient
                    ),
                    "exact_candidate_coefficient_replay_passed": True,
                    "numeric_candidate_coefficient_float64_hex": (
                        numeric_candidate_coefficient.hex()
                    ),
                    "exact_candidate_coefficient_float64_hex": (
                        exact_candidate_coefficient_float.hex()
                    ),
                    "exact_to_float_projection_error": exact_to_float_error,
                    "exact_to_float_projection_tolerance": (
                        exact_to_float_tolerance
                    ),
                    "exact_to_float_projection_replay_passed": True,
                    "numeric_cut_excludes_source_candidate": True,
                    "constraint_definition_id": (
                        cut.constraint_definition_id
                    ),
                    "numeric_projection_sha256": content_sha256(numeric_payload),
                    "numeric_projection_rho_is_bitwise_zero": True,
                    "numeric_projection_solver_suffix_is_bitwise_zero": True,
                }
            )
            ledger_rows.append(row_payload)
    if not blocks:
        return (
            csr_matrix((0, variable_count), dtype=np.float64),
            np.empty(0, dtype=np.float64),
            (),
        )
    return (
        vstack(blocks, format="csr"),
        np.concatenate(bounds),
        tuple(ledger_rows),
    )


def build_g10a2_cut_ledger_payload(
    *,
    geometry_catalog_sha256: str,
    rows: Sequence[Mapping[str, Any]],
) -> dict[str, Any]:
    _validate_sha256(
        geometry_catalog_sha256,
        name="geometry_catalog_sha256",
    )
    payload: dict[str, Any] = {
        "schema_version": 1,
        "kind": "g10a2_exact_selected_numeric_cut_ledger",
        "synthesis_id": G10A2_SYNTHESIS_ID,
        "geometry_catalog_sha256": geometry_catalog_sha256,
        "row_count": len(rows),
        "rows": [dict(row) for row in rows],
        "numeric_projection_validation_mode": (
            "source_bound_rebuild_during_ledger_creation"
        ),
        "numeric_projection_hash_is_formal_evidence": False,
        "unchanged_V3_reads_this_ledger": False,
        "formal_claim_allowed": False,
    }
    payload["ledger_content_sha256"] = content_sha256(payload)
    return payload


def validate_g10a2_cut_ledger(
    payload: Mapping[str, Any],
) -> tuple[str, ...]:
    """Validate ledger identities and self hash, not the numeric row itself.

    Numeric rows are rebuilt from the bound family source when the ledger is
    created.  Their digest is provenance only; unchanged V3 does not read it.
    """

    failures: list[str] = []
    material = dict(payload)
    declared_hash = str(material.pop("ledger_content_sha256", ""))
    if len(declared_hash) != 64 or content_sha256(material) != declared_hash:
        failures.append("ledger_content_sha256")
    top_level_checks = {
        "kind": payload.get("kind")
        == "g10a2_exact_selected_numeric_cut_ledger",
        "synthesis_id": payload.get("synthesis_id") == G10A2_SYNTHESIS_ID,
        "geometry_catalog_sha256": (
            isinstance(payload.get("geometry_catalog_sha256"), str)
            and len(str(payload.get("geometry_catalog_sha256"))) == 64
            and all(
                character in "0123456789abcdef"
                for character in str(payload.get("geometry_catalog_sha256"))
            )
        ),
        "numeric_projection_validation_mode": payload.get(
            "numeric_projection_validation_mode"
        )
        == "source_bound_rebuild_during_ledger_creation",
        "numeric_projection_hash_is_formal_evidence": payload.get(
            "numeric_projection_hash_is_formal_evidence"
        )
        is False,
        "unchanged_V3_reads_this_ledger": payload.get(
            "unchanged_V3_reads_this_ledger"
        )
        is False,
        "formal_claim_allowed": payload.get("formal_claim_allowed") is False,
    }
    failures.extend(
        name for name, passed in top_level_checks.items() if not passed
    )
    rows = payload.get("rows")
    if not isinstance(rows, Sequence) or isinstance(rows, (str, bytes)):
        return tuple((*failures, "rows_schema"))
    if payload.get("row_count") != len(rows):
        failures.append("row_count")
    for index, raw_row in enumerate(rows):
        prefix = f"row_{index}"
        if not isinstance(raw_row, Mapping):
            failures.append(f"{prefix}_schema")
            continue
        try:
            family_contract_sha256 = str(
                raw_row["family_contract_sha256"]
            )
            family_name = str(raw_row["family_name"])
            box_sha256 = str(raw_row["box_sha256"])
            box_raw = raw_row["box_exact"]
            if (
                not isinstance(box_raw, Sequence)
                or isinstance(box_raw, (str, bytes))
                or len(box_raw) != 4
            ):
                raise ValueError("box schema")
            box_values = tuple(Fraction(str(value)) for value in box_raw)
            if any(
                _fraction_text(value) != str(raw)
                for value, raw in zip(box_values, box_raw)
            ):
                raise ValueError("noncanonical box")
            box_exact: RationalBox = box_values  # type: ignore[assignment]
            domain_class = str(raw_row["domain_class"])
            depth = int(raw_row["depth"])
            expected_box_sha256 = content_sha256(
                _box_payload(
                    family_name=family_name,
                    box=box_exact,
                    depth=depth,
                    domain_class=domain_class,
                )
            )
            if box_sha256 != expected_box_sha256:
                failures.append(f"{prefix}_box_sha256")
            bernstein_index_raw = raw_row["bernstein_index"]
            target_degrees_raw = raw_row["target_degrees"]
            if (
                not isinstance(bernstein_index_raw, Sequence)
                or isinstance(bernstein_index_raw, (str, bytes))
                or len(bernstein_index_raw) != 2
                or not isinstance(target_degrees_raw, Sequence)
                or isinstance(target_degrees_raw, (str, bytes))
                or len(target_degrees_raw) != 2
            ):
                raise ValueError("index schema")
            bernstein_index = (
                int(bernstein_index_raw[0]),
                int(bernstein_index_raw[1]),
            )
            target_degrees = (
                int(target_degrees_raw[0]),
                int(target_degrees_raw[1]),
            )
            definition_id = content_sha256(
                _constraint_definition_material(
                    family_contract_sha256=family_contract_sha256,
                    family_name=family_name,
                    box_sha256=box_sha256,
                    bernstein_index=bernstein_index,
                    target_degrees=target_degrees,
                )
            )
            if raw_row.get("constraint_definition_id") != definition_id:
                failures.append(f"{prefix}_constraint_definition_id")
            coefficient_text = str(raw_row["exact_candidate_coefficient"])
            coefficient = Fraction(coefficient_text)
            if _fraction_text(coefficient) != coefficient_text:
                raise ValueError("noncanonical coefficient")
            event_hash = content_sha256(
                _selection_event_material(
                    constraint_definition_id=definition_id,
                    candidate_barrier_sha256=str(
                        raw_row["candidate_barrier_sha256"]
                    ),
                    exact_candidate_coefficient=coefficient_text,
                    classification=str(raw_row["classification"]),
                )
            )
            if raw_row.get("selection_event_sha256") != event_hash:
                failures.append(f"{prefix}_selection_event_sha256")
            candidate_hash = str(raw_row["candidate_barrier_sha256"])
            if len(candidate_hash) != 64 or any(
                character not in "0123456789abcdef"
                for character in candidate_hash
            ):
                failures.append(f"{prefix}_candidate_barrier_sha256")
            numeric_projection_hash = str(
                raw_row["numeric_projection_sha256"]
            )
            if len(numeric_projection_hash) != 64 or any(
                character not in "0123456789abcdef"
                for character in numeric_projection_hash
            ):
                failures.append(f"{prefix}_numeric_projection_sha256")
            if raw_row.get("rho_coefficient_float64_hex") != 0.0.hex():
                failures.append(f"{prefix}_rho_definition")
            if raw_row.get("formal_claim_allowed") is not False:
                failures.append(f"{prefix}_formal_claim_allowed")
            if raw_row.get("numeric_projection_rho_is_bitwise_zero") is not True:
                failures.append(f"{prefix}_rho_projection")
            if raw_row.get("exact_to_float_projection_replay_passed") is not True:
                failures.append(f"{prefix}_exact_to_float_replay")
            if raw_row.get("numeric_cut_excludes_source_candidate") is not True:
                failures.append(f"{prefix}_cut_excludes_source_candidate")
            if (
                raw_row.get(
                    "numeric_projection_solver_suffix_is_bitwise_zero"
                )
                is not True
            ):
                failures.append(f"{prefix}_solver_suffix_projection")
        except (KeyError, TypeError, ValueError, ZeroDivisionError):
            failures.append(f"{prefix}_schema")
    return tuple(dict.fromkeys(failures))


def _save_cut_ledger(
    path: Path,
    *,
    geometry_catalog_sha256: str,
    rows: Sequence[Mapping[str, Any]],
) -> None:
    payload = build_g10a2_cut_ledger_payload(
        geometry_catalog_sha256=geometry_catalog_sha256,
        rows=rows,
    )
    failures = validate_g10a2_cut_ledger(payload)
    if failures:
        raise ValueError(f"G10A2 cut ledger failed self-validation: {failures}")
    _write_json(path, payload)


def _candidate_payload(
    *,
    candidate_index: int,
    primary_solve_index: int,
    secondary_solve_index: int,
    source_pass_id: str,
    chebyshev_indices: Sequence[Power2D],
    chebyshev_coefficients: Sequence[float],
    barrier: SparsePolynomial2D,
    controller: FrozenController,
    t_primary_optimum: float,
    t_secondary_value: float,
    t_secondary_upper_bound: float,
    l1_secondary_value: float,
    minimum_lp_slack: float,
    maximum_bound_violation: float,
    primary_solver_backend: str,
    primary_solver_status: str,
    secondary_solver_backend: str,
    secondary_solver_status: str,
) -> dict[str, Any]:
    payload: dict[str, Any] = {
        "schema_version": 1,
        "kind": "g10a2_qsbc_numeric_candidate",
        "synthesis_id": G10A2_SYNTHESIS_ID,
        "proof_route_id": G10A2_PROOF_ROUTE_ID,
        "candidate_index": candidate_index,
        "primary_solve_index": primary_solve_index,
        "secondary_solve_index": secondary_solve_index,
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
        "q_float64_hex": G10_Q.hex(),
        "epsbar_float64_hex": G10_EPSBAR.hex(),
        "rho": 0.0,
        "rho_float64_hex": 0.0.hex(),
        "projection_objective": {
            "primary": "min_max_abs_chebyshev_distance_to_G9_seed",
            "secondary": "min_sum_abs_chebyshev_distance_to_G9_seed",
            "t_primary_optimum": t_primary_optimum,
            "t_primary_optimum_float64_hex": t_primary_optimum.hex(),
            "t_secondary_value": t_secondary_value,
            "t_secondary_value_float64_hex": t_secondary_value.hex(),
            "t_secondary_upper_bound": t_secondary_upper_bound,
            "t_secondary_upper_bound_float64_hex": (
                t_secondary_upper_bound.hex()
            ),
            "lexicographic_t_tolerance": (
                G10A2_LEXICOGRAPHIC_T_TOLERANCE
            ),
            "l1_secondary_value": l1_secondary_value,
            "l1_secondary_value_float64_hex": l1_secondary_value.hex(),
        },
        "minimum_active_lp_slack": minimum_lp_slack,
        "maximum_bound_violation": maximum_bound_violation,
        "primary_solver_backend": primary_solver_backend,
        "primary_solver_status": primary_solver_status,
        "secondary_solver_backend": secondary_solver_backend,
        "secondary_solver_status": secondary_solver_status,
        "formal_claim_allowed_before_unique_full_V3": False,
    }
    payload["content_sha256"] = content_sha256(payload)
    return payload


def finalize_g10a2_pending_certificate(
    *,
    pending_path: str | Path,
    certificate_path: str | Path,
) -> Path:
    """Atomically promote the route-owned pending artifact in one directory."""

    pending = Path(pending_path)
    certificate = Path(certificate_path)
    if (
        pending.name != "certificate.pending.json"
        or certificate.name != "certificate.json"
        or pending.parent != certificate.parent
    ):
        raise ValueError("G10A2 certificate promotion path contract mismatch")
    if not pending.is_file():
        raise FileNotFoundError("G10A2 pending certificate is missing")
    if certificate.exists():
        raise FileExistsError("G10A2 final certificate already exists")
    pending.replace(certificate)
    return certificate


def cleanup_g10a2_publication_artifacts(
    destination: str | Path,
) -> tuple[str, ...]:
    """Best-effort cleanup of only G10A2 route-owned publication filenames."""

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
            quarantine = root / f".{target.name}.not_ready_quarantine"
            try:
                quarantine.unlink(missing_ok=True)
                target.replace(quarantine)
            except OSError as quarantine_error:
                failures.append(
                    f"{target.name}:{type(error).__name__}:{error};"
                    f"quarantine:{type(quarantine_error).__name__}:"
                    f"{quarantine_error}"
                )
        if target.exists():
            failures.append(f"{target.name}:still_exists_after_cleanup")
    return tuple(failures)


def _write_g10a2_pending_certificate(
    *,
    destination: Path,
    barrier: SparsePolynomial2D,
    controller: FrozenController,
    rho: float,
    validation: Mapping[str, Any],
    validation_path: Path,
    solver_backend: str,
    cut_row_count: int,
    geometry_catalog_sha256: str,
    v3_root_set_sha256: str,
    preseparator_report_sha256: str,
) -> Path:
    if rho.hex() != 0.0.hex():
        raise ValueError("G10A2 publication requires canonical fixed rho=+0.0")
    publication_fields = {
        "schema_version": validation.get("schema_version") == 2,
        "validator_id": validation.get("validator_id") == G10_VALIDATOR_ID,
        "status": validation.get("status") == "validated",
        "first_slice_ready": validation.get("first_slice_ready") is True,
        "formal_acceptance_tolerance": (
            validation.get("formal_acceptance_tolerance") == 0.0
        ),
    }
    if not all(publication_fields.values()):
        raise ValueError(
            "G10A2 publication called without a schema-v2 validated V3 subject"
        )
    certificate = QSBCertificate(
        certificate_id=(
            f"CBF_ROVER_G10A2_W16_{controller.sha256[:12]}"
        ),
        barrier=barrier,
        q=G10_Q,
        epsbar=G10_EPSBAR,
        controller_content_sha256=controller.sha256,
        schema_version=2,
        solver={
            "method": "exact_Bernstein_preseparated_numeric_LP",
            "synthesis_id": G10A2_SYNTHESIS_ID,
            "proof_route_id": G10A2_PROOF_ROUTE_ID,
            "solver_backend": solver_backend,
            "stage_id": G10A2_STAGE.stage_id,
            "barrier_basis": "total_degree_tensor_chebyshev",
            "barrier_degree": G10A2_STAGE.barrier_degree,
            "controller_degree": controller_max_degree(controller),
            "rho": rho,
            "selected_coefficient_cut_count": cut_row_count,
            "geometry_catalog_sha256": geometry_catalog_sha256,
            "v3_root_set_sha256": v3_root_set_sha256,
            "preseparator_report_sha256": preseparator_report_sha256,
            "preseparator_is_not_formal_evidence": True,
            "full_V3_call_count": 1,
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
    certificate_path = destination / "certificate.pending.json"
    if certificate_path.exists() or (destination / "certificate.json").exists():
        raise FileExistsError("G10A2 publication path is not empty")
    certificate.save(certificate_path)
    return certificate_path


def _numeric_solve_record(
    *,
    solve_index: int,
    candidate_index: int,
    phase: str,
    result: Any,
    matrix: csr_matrix,
    cumulative_lp_time_s: float,
) -> dict[str, Any]:
    return {
        "solve_index": solve_index,
        "candidate_index": candidate_index,
        "phase": phase,
        "matrix_row_count": int(matrix.shape[0]),
        "matrix_column_count": int(matrix.shape[1]),
        "matrix_nonzero_count": int(matrix.nnz),
        "solver_backend": result.backend,
        "solver_backend_status": result.backend_status,
        "solver_success_means_numeric_vector_available": result.success,
        "solver_message": result.message,
        "solver_iterations": result.iterations,
        "solver_time_s": result.solve_time_s,
        "aggregate_lp_time_used_s": cumulative_lp_time_s,
        "aggregate_budget_exceeded": result.aggregate_budget_exceeded,
        "fallback_reason": result.fallback_reason,
        "formal_claim_allowed": False,
    }


def _record_resource_sample(
    *,
    manifest: dict[str, Any],
    label: str,
    matrix: csr_matrix,
    ephemeral_lexicographic_row: bool = False,
) -> None:
    rss_bytes = _peak_rss_bytes()
    failures = g10a2_resource_failures(
        matrix,
        rss_bytes=rss_bytes,
        ephemeral_lexicographic_row=ephemeral_lexicographic_row,
    )
    manifest.setdefault("resource_samples", []).append(
        {
            "label": label,
            "rss_bytes": rss_bytes,
            "rss_limit_bytes": G10A2_MAX_RSS_BYTES,
            "matrix_row_count": int(matrix.shape[0]),
            "matrix_column_count": int(matrix.shape[1]),
            "matrix_nonzero_count": int(matrix.nnz),
            "failures": list(failures),
        }
    )
    if failures:
        raise G10A2PreseparatorBudgetExceeded(
            f"G10A2 hard resource gate failed at {label}: {failures}"
        )


def _check_stage_deadline(
    *,
    stage_deadline: float,
    clock: Callable[[], float] = monotonic,
) -> None:
    if clock() > stage_deadline:
        raise G10A2PreseparatorBudgetExceeded(
            "G10A2 stage wall-time budget expired; current subject discarded"
        )


def _solve_two_phase_projection(
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
    """Solve one fixed min-Linf/min-L1 pair against the current master rows."""

    if master_matrix.shape[0] != master_right_hand_side.size:
        raise ValueError("G10A2 master matrix/right-hand-side mismatch")
    _check_stage_deadline(stage_deadline=stage_deadline, clock=clock)
    _record_resource_sample(
        manifest=manifest,
        label=f"candidate_{candidate_index:02d}_primary",
        matrix=master_matrix,
    )
    remaining_lp_s = (
        G10A2_STAGE.aggregate_lp_time_limit_s - cumulative_lp_time_s
    )
    remaining_wall_s = stage_deadline - clock()
    delegated_primary_s = min(remaining_lp_s, remaining_wall_s)
    if delegated_primary_s <= 0.0:
        raise G10A2PreseparatorBudgetExceeded(
            "G10A2 aggregate LP or stage wall-time budget expired before primary"
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
    _check_stage_deadline(stage_deadline=stage_deadline, clock=clock)
    if (
        primary.values is None
        or not primary.success
        or primary.aggregate_budget_exceeded
        or cumulative_lp_time_s > G10A2_STAGE.aggregate_lp_time_limit_s
    ):
        raise RuntimeError("G10A2 primary projection LP produced no accepted vector")
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
        or primary_values[projection.rho_index] != 0.0
        or primary_minimum_slack < -G10_NUMERIC_FEASIBILITY_TOLERANCE
        or primary_bound_violation > G10_NUMERIC_FEASIBILITY_TOLERANCE
    ):
        raise RuntimeError("G10A2 primary projection vector failed numeric replay")
    phase_records[-1]["minimum_master_slack"] = primary_minimum_slack
    phase_records[-1]["maximum_bound_violation"] = primary_bound_violation
    t_primary = float(primary_values[projection.t_index])
    t_upper = float(
        np.nextafter(t_primary, np.inf)
        + G10A2_LEXICOGRAPHIC_T_TOLERANCE
    )
    lex_row = np.zeros(projection.variable_count, dtype=np.float64)
    lex_row[projection.t_index] = 1.0
    secondary_matrix = vstack(
        (master_matrix, csr_matrix(lex_row.reshape(1, -1))),
        format="csr",
    )
    secondary_rhs = np.concatenate(
        (master_right_hand_side, np.asarray([t_upper], dtype=np.float64))
    )
    _record_resource_sample(
        manifest=manifest,
        label=f"candidate_{candidate_index:02d}_secondary",
        matrix=secondary_matrix,
        ephemeral_lexicographic_row=True,
    )
    remaining_lp_s = (
        G10A2_STAGE.aggregate_lp_time_limit_s - cumulative_lp_time_s
    )
    remaining_wall_s = stage_deadline - clock()
    delegated_secondary_s = min(remaining_lp_s, remaining_wall_s)
    if delegated_secondary_s <= 0.0:
        raise G10A2PreseparatorBudgetExceeded(
            "G10A2 aggregate LP or stage wall-time budget expired before secondary"
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
    _record_resource_sample(
        manifest=manifest,
        label=f"candidate_{candidate_index:02d}_secondary_complete",
        matrix=secondary_matrix,
        ephemeral_lexicographic_row=True,
    )
    _check_stage_deadline(stage_deadline=stage_deadline, clock=clock)
    if (
        secondary.values is None
        or not secondary.success
        or secondary.aggregate_budget_exceeded
        or cumulative_lp_time_s > G10A2_STAGE.aggregate_lp_time_limit_s
    ):
        raise RuntimeError("G10A2 secondary projection LP produced no accepted vector")
    values = np.asarray(secondary.values, dtype=np.float64).copy()
    if not np.isfinite(values).all():
        raise ValueError("G10A2 secondary projection vector is non-finite")
    if values[projection.rho_index] != 0.0:
        raise ValueError("G10A2 fixed rho dummy is not exactly zero")
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
        raise RuntimeError("G10A2 projected candidate failed numeric replay")
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


def synthesize_g10a2_qsbc_schedule(
    *,
    frozen_controller_path: str | Path,
    g9_seed_path: str | Path,
    output_dir: str | Path,
) -> Path:
    """Run the finite G10A2 W16 preseparator and at most one unchanged V3."""

    destination = Path(output_dir).resolve()
    destination.mkdir(parents=True, exist_ok=False)
    manifest_path = destination / "schedule_manifest.json"
    pending_certificate_path = destination / "certificate.pending.json"
    final_certificate_path = destination / "certificate.json"
    stage_started = monotonic()
    stage_deadline = stage_started + G10A2_STAGE.stage_wall_time_limit_s
    manifest: dict[str, Any] = {
        "schema_version": 1,
        "kind": "g10a2_finite_exact_preseparation_qsbc_schedule",
        "synthesis_id": G10A2_SYNTHESIS_ID,
        "proof_route_id": G10A2_PROOF_ROUTE_ID,
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
            "stage_id": G10A2_STAGE.stage_id,
            "barrier_degree": G10A2_STAGE.barrier_degree,
            "max_lp_solves": G10A2_STAGE.max_lp_solves,
            "aggregate_lp_time_limit_s": (
                G10A2_STAGE.aggregate_lp_time_limit_s
            ),
            "aggregate_exact_preseparator_time_limit_s": (
                G10A2_STAGE.aggregate_preseparator_time_limit_s
            ),
            "stage_wall_time_limit_s": G10A2_STAGE.stage_wall_time_limit_s,
            "max_full_V3_calls": G10A2_MAX_FULL_V3_CALLS,
        },
        "hard_resource_caps": {
            "max_rss_bytes": G10A2_MAX_RSS_BYTES,
            "max_selected_cut_rows": G10A2_MAX_HARD_CUT_ROWS,
            "max_master_lp_rows": G10A2_MAX_MASTER_LP_ROWS,
            "max_ephemeral_lp_rows": G10A2_MAX_EPHEMERAL_LP_ROWS,
            "max_csr_nonzeros": G10A2_MAX_CSR_NONZEROS,
            "max_decision_variables": G10A2_MAX_DECISION_VARIABLES,
        },
        "pass_schedule": [
            {
                "pass_id": item.pass_id,
                "depth": item.depth,
                "max_new_cuts_per_active_box": (
                    item.max_new_cuts_per_active_box
                ),
            }
            for item in G10A2_PASS_SCHEDULE
        ],
        "automatic_degree_or_catalog_escalation_allowed": False,
        "LP_phases": [],
        "candidates": [],
        "preseparator_passes": [],
        "full_V3_call_count": 0,
        "resource_samples": [],
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
            raise ValueError("G10A2 requires the bound G9 Chebyshev seed vector")
        if seed.barrier.degree > G10A2_STAGE.barrier_degree:
            raise ValueError("G10A2 G9 seed degree exceeds frozen W16")

        chebyshev_indices, barrier_basis = total_degree_chebyshev_basis(
            G10A2_STAGE.barrier_degree
        )
        coefficient_count = len(chebyshev_indices)
        if coefficient_count != 153:
            raise ValueError("G10A2 W16 coefficient catalog is not the frozen 153")
        families = _constraint_families(
            controller=controller,
            barrier_basis=barrier_basis,
        )
        if len(families) != G10A_FROZEN_QSBC_FAMILY_COUNT:
            raise ValueError("G10A2 q-SBC family count is not the frozen 42")
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
        geometry_catalog = build_g10a2_geometry_catalog(families)
        expected_depth_counts = {
            "3": 42 * 8,
            "4": 42 * 16,
            "5": 42 * 32,
        }
        if dict(geometry_catalog.depth_box_counts) != expected_depth_counts:
            raise ValueError("G10A2 frozen geometry catalog count mismatch")
        geometry_path = destination / "exact_geometry_catalog.json"
        _write_json(geometry_path, geometry_catalog.payload)

        core_store = _ActiveRowStore(
            families=families,
            variable_count=coefficient_count + 1,
            rho_index=coefficient_count,
        )
        _, _, accepted_anchors = core_store.append(anchors)
        if len(accepted_anchors) != G10A_FROZEN_QSBC_FAMILY_COUNT:
            raise ValueError("G10A2 did not bind exactly one G9 anchor per family")
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
            projection.variable_count != G10A2_MAX_DECISION_VARIABLES
            or projection.matrix.shape[0]
            != G10A_FROZEN_QSBC_FAMILY_COUNT + 4 * coefficient_count
        ):
            raise ValueError("G10A2 frozen projection layout mismatch")
        master_matrix = projection.matrix.copy().tocsr()
        master_rhs = projection.right_hand_side.copy()
        _record_resource_sample(
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
                "legacy_declared_report_content_sha256": (
                    seed.legacy_declared_report_content_sha256
                ),
                "current_recomputed_probe_content_sha256": (
                    seed.current_recomputed_probe_content_sha256
                ),
                "barrier_payload_sha256": content_sha256(
                    seed.barrier.payload()
                ),
                "barrier_is_used_as_formal_subject": False,
            },
            "g9_depth3_eligible_geometry_sha256": geometry_binding_hash,
            "g9_depth3_eligible_geometry_count": geometry_binding_count,
            "g9_dormant_rows_used_by_G10A2": 0,
        }
        manifest["geometry_catalog"] = {
            "file": geometry_path.name,
            "file_sha256": file_sha256(geometry_path),
            "content_sha256": geometry_catalog.content_sha256,
            "family_contract_sha256": (
                geometry_catalog.family_contract_sha256
            ),
            "v3_root_set_sha256": geometry_catalog.payload[
                "v3_root_set_sha256"
            ],
            "depth_box_counts": dict(geometry_catalog.depth_box_counts),
            "expected_leaf_set_sha256_by_depth": dict(
                geometry_catalog.expected_leaf_set_sha256_by_depth
            ),
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
            "rho_is_objective_variable": False,
            "rho_bounds": [0.0, 0.0],
        }
        ledger_path = destination / "selected_cut_ledger.json"
        all_ledger_rows: list[Mapping[str, Any]] = []
        _save_cut_ledger(
            ledger_path,
            geometry_catalog_sha256=geometry_catalog.content_sha256,
            rows=all_ledger_rows,
        )
        manifest["status"] = "solving"
        _save_manifest(manifest_path, manifest)

        phase_records = manifest["LP_phases"]
        cumulative_lp_time_s = 0.0
        cumulative_preseparator_time_s = 0.0
        current_seed_values = projection.seed_values.copy()
        existing_definition_ids: set[str] = set()
        final_barrier: SparsePolynomial2D | None = None
        final_coefficients: tuple[float, ...] | None = None
        final_solve: dict[str, Any] | None = None
        final_candidate_directory: Path | None = None
        for candidate_index in range(1, 5):
            source_pass_id = (
                "INITIAL_G9_ANCHORS"
                if candidate_index == 1
                else G10A2_PASS_SCHEDULE[candidate_index - 2].pass_id
            )
            solve = _solve_two_phase_projection(
                projection=projection,
                master_matrix=master_matrix,
                master_right_hand_side=master_rhs,
                seed_values=current_seed_values,
                candidate_index=candidate_index,
                first_solve_index=2 * candidate_index - 1,
                cumulative_lp_time_s=cumulative_lp_time_s,
                stage_deadline=stage_deadline,
                phase_records=phase_records,
                manifest=manifest,
            )
            cumulative_lp_time_s = float(solve["cumulative_lp_time_s"])
            values = np.asarray(solve["values"], dtype=np.float64)
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
                primary_solve_index=2 * candidate_index - 1,
                secondary_solve_index=2 * candidate_index,
                source_pass_id=source_pass_id,
                chebyshev_indices=chebyshev_indices,
                chebyshev_coefficients=coefficients,
                barrier=barrier,
                controller=controller,
                t_primary_optimum=float(solve["t_primary"]),
                t_secondary_value=float(values[projection.t_index]),
                t_secondary_upper_bound=float(solve["t_upper"]),
                l1_secondary_value=float(solve["l1_value"]),
                minimum_lp_slack=float(solve["minimum_slack"]),
                maximum_bound_violation=float(
                    solve["maximum_bound_violation"]
                ),
                primary_solver_backend=solve["primary"].backend,
                primary_solver_status=solve["primary"].backend_status,
                secondary_solver_backend=solve["secondary"].backend,
                secondary_solver_status=solve["secondary"].backend_status,
            )
            candidate_path = candidate_directory / "numeric_candidate.json"
            _write_json(candidate_path, candidate_payload)
            manifest["candidates"].append(
                {
                    "candidate_index": candidate_index,
                    "source_pass_id": source_pass_id,
                    "file": str(candidate_path.relative_to(destination)),
                    "file_sha256": file_sha256(candidate_path),
                    "content_sha256": candidate_payload["content_sha256"],
                    "barrier_payload_sha256": barrier_hash,
                    "master_row_count": int(master_matrix.shape[0]),
                    "master_nonzero_count": int(master_matrix.nnz),
                    "selected_cut_row_count": len(all_ledger_rows),
                }
            )
            _save_manifest(manifest_path, manifest)

            final_barrier = barrier
            final_coefficients = coefficients
            final_solve = solve
            final_candidate_directory = candidate_directory
            if candidate_index == 4:
                break
            pass_spec = G10A2_PASS_SCHEDULE[candidate_index - 1]
            scan_started = monotonic()
            remaining_preseparator_s = (
                G10A2_STAGE.aggregate_preseparator_time_limit_s
                - cumulative_preseparator_time_s
            )
            if remaining_preseparator_s <= 0.0:
                raise G10A2PreseparatorBudgetExceeded(
                    "G10A2 aggregate exact preseparator budget exhausted"
                )
            scan_deadline = min(
                scan_started + remaining_preseparator_s,
                stage_deadline,
            )
            report, selected_cuts, _ = _scan_all_families(
                barrier=barrier,
                controller=controller,
                families=families,
                family_contract_sha256=(
                    geometry_catalog.family_contract_sha256
                ),
                depth=pass_spec.depth,
                max_new_cuts_per_active_box=(
                    pass_spec.max_new_cuts_per_active_box
                ),
                existing_constraint_definition_ids=frozenset(
                    existing_definition_ids
                ),
                expected_leaf_set_sha256=(
                    geometry_catalog.expected_leaf_set_sha256_by_depth[
                        str(pass_spec.depth)
                    ]
                ),
                deadline=scan_deadline,
            )
            scan_elapsed_s = monotonic() - scan_started
            cumulative_preseparator_time_s += scan_elapsed_s
            if (
                cumulative_preseparator_time_s
                > G10A2_STAGE.aggregate_preseparator_time_limit_s
            ):
                raise G10A2PreseparatorBudgetExceeded(
                    "G10A2 aggregate exact preseparator budget exceeded"
                )
            if report["candidate_barrier_sha256"] != barrier_hash:
                raise ValueError("G10A2 pass report candidate binding mismatch")
            report_path = candidate_directory / (
                f"preseparator_{pass_spec.pass_id}.json"
            )
            _write_json(report_path, report)
            residuals = _exact_candidate_residuals(
                barrier=barrier,
                controller=controller,
                families=families,
            )
            cut_matrix, cut_rhs, ledger_rows = _build_bound_numeric_cut_rows(
                families=families,
                residuals=residuals,
                cuts=selected_cuts,
                candidate_barrier_sha256=barrier_hash,
                candidate_chebyshev_coefficients=coefficients,
                variable_count=projection.variable_count,
                rho_index=projection.rho_index,
                rss_reader=_peak_rss_bytes,
            )
            if cut_matrix.shape[0] != len(selected_cuts):
                raise ValueError("G10A2 selected cut row count mismatch")
            selected_ids = {
                cut.constraint_definition_id for cut in selected_cuts
            }
            if (
                len(selected_ids) != len(selected_cuts)
                or selected_ids.intersection(existing_definition_ids)
            ):
                raise ValueError("G10A2 selected duplicate constraint definition")
            existing_definition_ids.update(selected_ids)
            all_ledger_rows.extend(ledger_rows)
            if len(all_ledger_rows) > G10A2_MAX_HARD_CUT_ROWS:
                raise G10A2PreseparatorBudgetExceeded(
                    "G10A2 selected coefficient-cut row cap exceeded"
                )
            master_matrix = vstack(
                (master_matrix, cut_matrix),
                format="csr",
            )
            master_rhs = np.concatenate((master_rhs, cut_rhs))
            _record_resource_sample(
                manifest=manifest,
                label=f"after_{pass_spec.pass_id}",
                matrix=master_matrix,
            )
            _save_cut_ledger(
                ledger_path,
                geometry_catalog_sha256=geometry_catalog.content_sha256,
                rows=all_ledger_rows,
            )
            manifest["preseparator_passes"].append(
                {
                    "pass_id": pass_spec.pass_id,
                    "candidate_index": candidate_index,
                    "candidate_barrier_sha256": barrier_hash,
                    "report": str(report_path.relative_to(destination)),
                    "report_file_sha256": file_sha256(report_path),
                    "report_content_sha256": report[
                        "report_content_sha256"
                    ],
                    "catalog_depth": pass_spec.depth,
                    "scan_elapsed_s": scan_elapsed_s,
                    "aggregate_exact_preseparator_time_used_s": (
                        cumulative_preseparator_time_s
                    ),
                    "selected_new_cut_count": len(selected_cuts),
                    "aggregate_selected_cut_count": len(all_ledger_rows),
                    "whole_catalog_traversed": report[
                        "whole_catalog_traversed"
                    ],
                    "expected_leaf_set_sha256": report[
                        "expected_leaf_set_sha256"
                    ],
                    "scanned_leaf_set_sha256": report[
                        "scanned_leaf_set_sha256"
                    ],
                    "leaf_set_binding_passed": report[
                        "leaf_set_binding_passed"
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
            raise RuntimeError("G10A2 finite candidate schedule produced no subject")
        gate_started = monotonic()
        remaining_preseparator_s = (
            G10A2_STAGE.aggregate_preseparator_time_limit_s
            - cumulative_preseparator_time_s
        )
        if remaining_preseparator_s <= 0.0:
            raise G10A2PreseparatorBudgetExceeded(
                "G10A2 exact budget expired before final D5 gate"
            )
        gate_deadline = min(
            gate_started + remaining_preseparator_s,
            stage_deadline,
        )
        final_report, final_selected, preseparator_gate_passed = (
            _scan_all_families(
                barrier=final_barrier,
                controller=controller,
                families=families,
                family_contract_sha256=(
                    geometry_catalog.family_contract_sha256
                ),
                depth=G10A2_EXACT_GATE_DEPTH,
                max_new_cuts_per_active_box=0,
                existing_constraint_definition_ids=frozenset(),
                expected_leaf_set_sha256=(
                    geometry_catalog.expected_leaf_set_sha256_by_depth["5"]
                ),
                deadline=gate_deadline,
                v3_subject_mode=True,
            )
        )
        gate_elapsed_s = monotonic() - gate_started
        cumulative_preseparator_time_s += gate_elapsed_s
        if (
            cumulative_preseparator_time_s
            > G10A2_STAGE.aggregate_preseparator_time_limit_s
        ):
            raise G10A2PreseparatorBudgetExceeded(
                "G10A2 exact budget exceeded during final D5 gate"
            )
        if final_selected:
            raise ValueError("G10A2 final exact D5 gate must never select cuts")
        final_barrier_hash = content_sha256(final_barrier.payload())
        if final_report["candidate_barrier_sha256"] != final_barrier_hash:
            raise ValueError("G10A2 final D5 subject hash mismatch")
        final_report_path = final_candidate_directory / "final_D5_exact_gate.json"
        _write_json(final_report_path, final_report)
        manifest["final_exact_D5_gate"] = {
            "candidate_barrier_sha256": final_barrier_hash,
            "report": str(final_report_path.relative_to(destination)),
            "report_file_sha256": file_sha256(final_report_path),
            "report_content_sha256": final_report["report_content_sha256"],
            "scan_elapsed_s": gate_elapsed_s,
            "aggregate_exact_preseparator_time_used_s": (
                cumulative_preseparator_time_s
            ),
            "whole_catalog_traversed": final_report[
                "whole_catalog_traversed"
            ],
            "expected_leaf_set_sha256": final_report[
                "expected_leaf_set_sha256"
            ],
            "scanned_leaf_set_sha256": final_report[
                "scanned_leaf_set_sha256"
            ],
            "leaf_set_binding_passed": final_report[
                "leaf_set_binding_passed"
            ],
            "negative_coefficient_count": final_report[
                "negative_coefficient_count"
            ],
            "gate_passed": preseparator_gate_passed,
            "formal_claim_allowed": False,
        }
        if not preseparator_gate_passed:
            manifest["status"] = "not_ready"
            manifest["terminal_reason"] = (
                "final_complete_D5_exact_preseparator_gate_failed_no_V3"
            )
            manifest["aggregate_lp_time_used_s"] = cumulative_lp_time_s
            manifest["aggregate_exact_preseparator_time_used_s"] = (
                cumulative_preseparator_time_s
            )
            manifest["selected_cut_ledger"] = {
                "file": ledger_path.name,
                "file_sha256": file_sha256(ledger_path),
                "row_count": len(all_ledger_rows),
            }
            _save_manifest(manifest_path, manifest)
            return manifest_path

        _check_stage_deadline(stage_deadline=stage_deadline)
        gc.collect()
        _record_resource_sample(
            manifest=manifest,
            label="before_unique_full_V3",
            matrix=master_matrix,
        )
        manifest["full_V3_call_count"] = 1
        _save_manifest(manifest_path, manifest)
        validation_path = _validate_candidate_v3(
            barrier=final_barrier,
            controller=controller,
            output_dir=final_candidate_directory / "independent_validation_v3",
        )
        _record_resource_sample(
            manifest=manifest,
            label="after_unique_full_V3",
            matrix=master_matrix,
        )
        _check_stage_deadline(stage_deadline=stage_deadline)
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
        publication_passed, publication_failures = g10a2_publication_gate(
            preseparator_gate_passed=preseparator_gate_passed,
            full_v3_call_count=manifest["full_V3_call_count"],
            validation_status=validation.get("status"),
            first_slice_ready=validation.get("first_slice_ready"),
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
            "first_slice_ready": validation.get("first_slice_ready"),
            "subject_and_tree_binding_failures": list(binding_failures),
            "unchanged_V3_release_gate_passed": v3_release_passed,
            "unchanged_V3_release_gate_failures": list(
                v3_release_failures
            ),
            "G10A2_publication_gate_passed": publication_passed,
            "G10A2_publication_gate_failures": list(publication_failures),
        }
        manifest["aggregate_lp_time_used_s"] = cumulative_lp_time_s
        manifest["aggregate_exact_preseparator_time_used_s"] = (
            cumulative_preseparator_time_s
        )
        manifest["selected_cut_ledger"] = {
            "file": ledger_path.name,
            "file_sha256": file_sha256(ledger_path),
            "row_count": len(all_ledger_rows),
            "unchanged_V3_reads_this_ledger": False,
        }
        if not publication_passed:
            manifest["status"] = "not_ready"
            manifest["terminal_reason"] = (
                "unique_unchanged_V3_did_not_pass_all_release_gates_no_retry"
            )
            _save_manifest(manifest_path, manifest)
            return manifest_path

        _record_resource_sample(
            manifest=manifest,
            label="before_publication",
            matrix=master_matrix,
        )
        _check_stage_deadline(stage_deadline=stage_deadline)
        written_pending_path = _write_g10a2_pending_certificate(
            destination=destination,
            barrier=final_barrier,
            controller=controller,
            rho=0.0,
            validation=validation,
            validation_path=validation_path,
            solver_backend=final_solve["secondary"].backend,
            cut_row_count=len(all_ledger_rows),
            geometry_catalog_sha256=geometry_catalog.content_sha256,
            v3_root_set_sha256=str(
                geometry_catalog.payload["v3_root_set_sha256"]
            ),
            preseparator_report_sha256=final_report[
                "report_content_sha256"
            ],
        )
        if written_pending_path != pending_certificate_path:
            raise ValueError("G10A2 pending certificate path binding mismatch")
        _record_resource_sample(
            manifest=manifest,
            label="after_pending_certificate_write",
            matrix=master_matrix,
        )
        _check_stage_deadline(stage_deadline=stage_deadline)
        certificate_path = finalize_g10a2_pending_certificate(
            pending_path=pending_certificate_path,
            certificate_path=final_certificate_path,
        )
        _record_resource_sample(
            manifest=manifest,
            label="after_certificate_commit",
            matrix=master_matrix,
        )
        _check_stage_deadline(stage_deadline=stage_deadline)
        manifest["certificate"] = {
            "file": certificate_path.name,
            "file_sha256": file_sha256(certificate_path),
            "atomic_promotion_from": pending_certificate_path.name,
        }
        _record_resource_sample(
            manifest=manifest,
            label="before_ready_manifest",
            matrix=master_matrix,
        )
        _check_stage_deadline(stage_deadline=stage_deadline)
        manifest["status"] = "ready"
        manifest["terminal_reason"] = (
            "unique_unchanged_V3_validated_subject_after_complete_D5_gate"
        )
        _save_manifest(manifest_path, manifest)
        return manifest_path
    except Exception as error:
        cleanup_failures = cleanup_g10a2_publication_artifacts(destination)
        manifest["status"] = "not_ready"
        manifest["terminal_reason"] = "G10A2_execution_error_fail_closed"
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
