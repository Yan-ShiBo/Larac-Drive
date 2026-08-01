"""G11P49 convex piecewise-roof candidate route.

The exact helpers in this module define the finite schedule and the linear
quadrant expectation operator.  Solver assembly is deliberately separate from
formal acceptance: every numeric candidate must be replayed by the independent
V3P validator before a schema-v3 certificate can exist.
"""

from __future__ import annotations

from dataclasses import dataclass
from fractions import Fraction
import gc
import json
from math import comb
from pathlib import Path
from typing import Any, Mapping, Sequence

import numpy as np
from scipy.optimize import linprog
from scipy.sparse import coo_matrix, csr_matrix

from cbf_rover.adapters.training.sac_reference import file_sha256

from .algebra import multiply, negate
from .bernstein_lp import monomial_bernstein_vector
from .certificate import QSBCertificate
from .formal_regions import goal_exterior_guard, initial_disk_guard

from .piecewise_barrier import (
    G11P49_FROZEN_EPSBAR_FLOAT64_HEX,
    G11P49_FROZEN_Q_FLOAT64_HEX,
    G11P49_PROOF_ROUTE_ID,
    G11P49_SYNTHESIS_ID,
    G11P49_VALIDATOR_ID,
    LocalBernsteinRoof2D,
    NOISE_HALF_WIDTH_X_NORMALIZED,
    NOISE_HALF_WIDTH_Y_NORMALIZED,
    P49_CELLS,
    P49_CELL_BY_ID,
    P49_GOAL_BOUNDARY_SAFE_CELLS,
    P49_QUADRANTS,
    P49_SAFE_SOURCE_CELLS,
    P49_UNSAFE_CELLS,
    PiecewiseBernsteinBarrier2D,
    PiecewiseRoofWitness2D,
    RationalPolynomial,
    local_bernstein_basis_rational_polynomials,
    p49_geometry_catalog_sha256,
    p49_quadrant_reach_box,
    p49_reach_target_intersections,
)
from .polynomial import (
    FrozenController,
    NumericPolynomial,
    PolynomialControllerPiece,
    Power2D,
    content_sha256,
    controller_pieces,
    load_frozen_controller,
)
from .rational_certificate_validation import (
    DT,
    NORMALIZATION_SCALE_X,
    NORMALIZATION_SCALE_Y,
    canonical_rational_model_payload,
    rational_sparse_polynomial,
)


G11P49_Q = float.fromhex(G11P49_FROZEN_Q_FLOAT64_HEX)
G11P49_EPSBAR = float.fromhex(G11P49_FROZEN_EPSBAR_FLOAT64_HEX)
G11P49_COEFFICIENT_BOUND = 1000.0
G11P49_MULTIPLIER_COEFFICIENT_BOUND = 100000.0
G11P49_RHO_BOUNDS = (-1.0, 1.0)
G11P49_MAX_RSS_BYTES = 8 * 1024**3
G11P49_MAX_CSR_NONZEROS = 5_000_000
G11P49_NUMERIC_FEASIBILITY_TOLERANCE = 1.0e-9
G11P49_CONTROLLER_CONTENT_SHA256 = (
    "b0cacedfcd4b884961a786b24e84b2f0f33bbc4ac81aef0b1ff1b0dcfe285984"
)
G11P49_CONTROLLER_FILE_SHA256 = (
    "df0dbfd2cc5b3aa0a049b5dbb6e7d0360c6b260801e183e95acfb7a9bccf715d"
)


@dataclass(frozen=True, slots=True)
class G11P49StageSpec:
    """One immutable member of the finite P49 schedule."""

    stage_id: str
    barrier_degree: int
    roof_degree: int
    multiplier_degree: int
    time_limit_s: float
    expected_variable_count: int
    expected_linear_row_count: int

    def __post_init__(self) -> None:
        if self.barrier_degree not in (3, 4):
            raise ValueError("P49 barrier degree must be 3 or 4")
        if self.roof_degree not in (4, 6):
            raise ValueError("P49 roof degree must be 4 or 6")
        if self.multiplier_degree not in (2, 4):
            raise ValueError("P49 multiplier degree must be 2 or 4")
        if self.roof_degree < self.barrier_degree:
            raise ValueError("P49 roof degree must dominate the barrier degree")
        if self.time_limit_s <= 0.0:
            raise ValueError("P49 stage time limit must be positive")
        variables, rows = g11p49_expected_problem_shape(
            barrier_degree=self.barrier_degree,
            roof_degree=self.roof_degree,
            multiplier_degree=self.multiplier_degree,
        )
        if variables != self.expected_variable_count:
            raise ValueError("P49 expected variable count drifted")
        if rows != self.expected_linear_row_count:
            raise ValueError("P49 expected row count drifted")


def g11p49_expected_problem_shape(
    *,
    barrier_degree: int,
    roof_degree: int,
    multiplier_degree: int,
) -> tuple[int, int]:
    """Return the frozen LP column/row counts for one stage."""

    barrier_dofs = (7 * barrier_degree + 1) ** 2
    roof_dofs = 4 * len(P49_SAFE_SOURCE_CELLS) * (roof_degree + 1) ** 2
    shape_multiplier_count = 1 + len(P49_GOAL_BOUNDARY_SAFE_CELLS)
    multiplier_dofs = shape_multiplier_count * (multiplier_degree + 1) ** 2
    variable_count = barrier_dofs + roof_dofs + multiplier_dofs + 1

    # W >= 0 uses the unique global control DOFs.  The unsafe and roof rows
    # intentionally keep their per-cell blocks, even where boundary rows are
    # duplicated; this freezes ordering and makes the formal catalog simple.
    w_nonnegative_rows = barrier_dofs
    unsafe_rows = len(P49_UNSAFE_CELLS) * (barrier_degree + 1) ** 2
    roof_target_intersection_count = 1012
    roof_domination_rows = (
        roof_target_intersection_count * (roof_degree + 1) ** 2
    )

    # C2 has x degree 1 and y degree 2.  Substitution into an R tensor basis of
    # degree r therefore produces a (r, 2r) tensor polynomial.
    drift_rows_per_source = (roof_degree + 1) * (2 * roof_degree + 1)
    drift_rows = len(P49_SAFE_SOURCE_CELLS) * drift_rows_per_source

    initial_degree = max(barrier_degree, multiplier_degree + 2)
    initial_rows = (initial_degree + 1) ** 2
    row_count = (
        w_nonnegative_rows
        + initial_rows
        + unsafe_rows
        + roof_domination_rows
        + drift_rows
    )
    return variable_count, row_count


G11P49_STAGE_SCHEDULE: tuple[G11P49StageSpec, ...] = (
    G11P49StageSpec(
        stage_id="P49_W3_R4_M2",
        barrier_degree=3,
        roof_degree=4,
        multiplier_degree=2,
        time_limit_s=3600.0,
        expected_variable_count=4221,
        expected_linear_row_count=27666,
    ),
    G11P49StageSpec(
        stage_id="P49_W4_R6_M2",
        barrier_degree=4,
        roof_degree=6,
        multiplier_degree=2,
        time_limit_s=7200.0,
        expected_variable_count=8130,
        expected_linear_row_count=54121,
    ),
    G11P49StageSpec(
        stage_id="P49_W4_R6_M4",
        barrier_degree=4,
        roof_degree=6,
        multiplier_degree=4,
        time_limit_s=14400.0,
        expected_variable_count=8194,
        expected_linear_row_count=54145,
    ),
)


def _clean(polynomial: Mapping[Power2D, Fraction]) -> RationalPolynomial:
    return {power: value for power, value in polynomial.items() if value}


def _constant(value: Fraction | int) -> RationalPolynomial:
    exact = Fraction(value)
    return {} if exact == 0 else {(0, 0): exact}


def _add(*polynomials: Mapping[Power2D, Fraction]) -> RationalPolynomial:
    result: RationalPolynomial = {}
    for polynomial in polynomials:
        for power, coefficient in polynomial.items():
            result[power] = result.get(power, Fraction(0)) + coefficient
    return _clean(result)


def _scale(
    polynomial: Mapping[Power2D, Fraction],
    scalar: Fraction | int,
) -> RationalPolynomial:
    exact = Fraction(scalar)
    if exact == 0:
        return {}
    return _clean({power: exact * value for power, value in polynomial.items()})


def _multiply(
    left: Mapping[Power2D, Fraction],
    right: Mapping[Power2D, Fraction],
) -> RationalPolynomial:
    result: RationalPolynomial = {}
    for (left_x, left_y), left_value in left.items():
        for (right_x, right_y), right_value in right.items():
            power = (left_x + right_x, left_y + right_y)
            result[power] = result.get(power, Fraction(0)) + (
                left_value * right_value
            )
    return _clean(result)


def _power(
    polynomial: Mapping[Power2D, Fraction],
    exponent: int,
) -> RationalPolynomial:
    if exponent < 0:
        raise ValueError("rational polynomial exponent must be nonnegative")
    result = _constant(1)
    base = dict(polynomial)
    value = int(exponent)
    while value:
        if value & 1:
            result = _multiply(result, base)
        value //= 2
        if value:
            base = _multiply(base, base)
    return result


def p49_quadrant_moment(
    *,
    order: int,
    sign: int,
    half_width: Fraction,
) -> Fraction:
    """Return an unnormalized triangular moment on one closed half support."""

    if order < 0 or sign not in (-1, 1) or half_width <= 0:
        raise ValueError("invalid P49 quadrant moment request")
    return Fraction(sign**order) * half_width**order / (
        (order + 1) * (order + 2)
    )


def _expected_univariate_power_on_quadrant(
    mean: Mapping[Power2D, Fraction],
    exponent: int,
    *,
    sign: int,
    half_width: Fraction,
) -> RationalPolynomial:
    result: RationalPolynomial = {}
    for noise_order in range(exponent + 1):
        result = _add(
            result,
            _scale(
                _power(mean, exponent - noise_order),
                Fraction(comb(exponent, noise_order))
                * p49_quadrant_moment(
                    order=noise_order,
                    sign=sign,
                    half_width=half_width,
                ),
            ),
        )
    return result


def p49_normalized_deterministic_dynamics(
    piece: PolynomialControllerPiece,
) -> tuple[RationalPolynomial, RationalPolynomial]:
    """Return exact-binary64 C2 means before N1 is added."""

    xi_x: RationalPolynomial = {(1, 0): Fraction(1)}
    xi_y: RationalPolynomial = {(0, 1): Fraction(1)}
    return (
        _add(
            xi_x,
            _scale(
                rational_sparse_polynomial(piece.u_x),
                DT / NORMALIZATION_SCALE_X,
            ),
        ),
        _add(
            xi_y,
            _scale(
                rational_sparse_polynomial(piece.u_y),
                DT / NORMALIZATION_SCALE_Y,
            ),
        ),
    )


def exact_roof_quadrant_expectation(
    roof: LocalBernsteinRoof2D,
    piece: PolynomialControllerPiece,
) -> RationalPolynomial:
    """Integrate one roof on its quadrant with exact triangular half moments.

    The result includes the quadrant probability mass.  In particular a
    constant-one roof maps to the constant ``1/4``; summing the four canonical
    quadrants maps it to one.
    """

    if roof.controller_piece_id != piece.piece_id:
        raise ValueError("roof expectation controller-piece mismatch")
    return exact_polynomial_quadrant_expectation(
        polynomial=roof.rational_polynomial(),
        quadrant=roof.quadrant,
        piece=piece,
    )


def exact_polynomial_quadrant_expectation(
    *,
    polynomial: Mapping[Power2D, Fraction],
    quadrant: tuple[int, int],
    piece: PolynomialControllerPiece,
) -> RationalPolynomial:
    """Apply one unnormalized N1 quadrant expectation to a polynomial."""

    return exact_polynomial_family_quadrant_expectations(
        polynomials=(polynomial,),
        quadrant=quadrant,
        piece=piece,
    )[0]


def exact_polynomial_family_quadrant_expectations(
    *,
    polynomials: Sequence[Mapping[Power2D, Fraction]],
    quadrant: tuple[int, int],
    piece: PolynomialControllerPiece,
) -> tuple[RationalPolynomial, ...]:
    """Apply one exact quadrant operator to a polynomial family efficiently."""

    if quadrant not in ((-1, -1), (-1, 1), (1, -1), (1, 1)):
        raise ValueError("invalid P49 expectation quadrant")
    if not polynomials:
        raise ValueError("P49 expectation family must not be empty")
    mean_x, mean_y = p49_normalized_deterministic_dynamics(piece)
    max_x = max(
        (power[0] for polynomial in polynomials for power in polynomial),
        default=0,
    )
    max_y = max(
        (power[1] for polynomial in polynomials for power in polynomial),
        default=0,
    )
    x_table = tuple(
        _expected_univariate_power_on_quadrant(
            mean_x,
            exponent,
            sign=quadrant[0],
            half_width=NOISE_HALF_WIDTH_X_NORMALIZED,
        )
        for exponent in range(max_x + 1)
    )
    y_table = tuple(
        _expected_univariate_power_on_quadrant(
            mean_y,
            exponent,
            sign=quadrant[1],
            half_width=NOISE_HALF_WIDTH_Y_NORMALIZED,
        )
        for exponent in range(max_y + 1)
    )
    monomial_expectations = {
        (x_power, y_power): _multiply(
            x_table[x_power],
            y_table[y_power],
        )
        for polynomial in polynomials
        for x_power, y_power in polynomial
    }
    return tuple(
        _add(
            *(
                _scale(monomial_expectations[power], coefficient)
                for power, coefficient in polynomial.items()
            )
        )
        for polynomial in polynomials
    )


def exact_source_roof_expectation(
    *,
    roofs: Mapping[tuple[int, int], LocalBernsteinRoof2D],
    piece: PolynomialControllerPiece,
) -> RationalPolynomial:
    """Sum the four unnormalized quadrant roof expectations."""

    if set(roofs) != {(-1, -1), (-1, 1), (1, -1), (1, 1)}:
        raise ValueError("source roof expectation requires exactly four quadrants")
    return _add(
        *(
            exact_roof_quadrant_expectation(roofs[quadrant], piece)
            for quadrant in ((-1, -1), (-1, 1), (1, -1), (1, 1))
        )
    )


@dataclass(frozen=True, slots=True)
class _G11P49VariableLayout:
    barrier_count: int
    roof_starts: Mapping[tuple[str, tuple[int, int]], int]
    roof_count: int
    multiplier_starts: Mapping[str, int]
    multiplier_count: int
    rho_index: int
    variable_count: int


@dataclass(frozen=True, slots=True)
class _G11P49Problem:
    matrix: csr_matrix
    right_hand_side: np.ndarray
    objective: np.ndarray
    bounds: tuple[tuple[float, float], ...]
    layout: _G11P49VariableLayout
    metadata: Mapping[str, Any]


def _build_variable_layout(stage: G11P49StageSpec) -> _G11P49VariableLayout:
    barrier_count = (7 * stage.barrier_degree + 1) ** 2
    cursor = barrier_count
    roof_starts: dict[tuple[str, tuple[int, int]], int] = {}
    roof_block_size = (stage.roof_degree + 1) ** 2
    for source in P49_SAFE_SOURCE_CELLS:
        for quadrant in P49_QUADRANTS:
            roof_starts[(source.cell_id, quadrant)] = cursor
            cursor += roof_block_size
    roof_count = cursor - barrier_count

    multiplier_starts: dict[str, int] = {}
    multiplier_block_size = (stage.multiplier_degree + 1) ** 2
    multiplier_keys = (
        "initial_C_0_3",
        *(f"goal_exterior_{cell.cell_id}" for cell in P49_GOAL_BOUNDARY_SAFE_CELLS),
    )
    for key in multiplier_keys:
        multiplier_starts[key] = cursor
        cursor += multiplier_block_size
    multiplier_count = len(multiplier_keys) * multiplier_block_size
    rho_index = cursor
    variable_count = rho_index + 1
    if variable_count != stage.expected_variable_count:
        raise ValueError("G11P49 variable-layout count drifted")
    return _G11P49VariableLayout(
        barrier_count=barrier_count,
        roof_starts=roof_starts,
        roof_count=roof_count,
        multiplier_starts=multiplier_starts,
        multiplier_count=multiplier_count,
        rho_index=rho_index,
        variable_count=variable_count,
    )


def _numeric(polynomial: Mapping[Power2D, Fraction]) -> NumericPolynomial:
    return {
        power: float(coefficient)
        for power, coefficient in polynomial.items()
        if coefficient
    }


def _scale_numeric(
    polynomial: Mapping[Power2D, float],
    scalar: float,
) -> NumericPolynomial:
    return {
        power: float(scalar) * float(coefficient)
        for power, coefficient in polynomial.items()
        if float(scalar) * float(coefficient) != 0.0
    }


def _constant_numeric(value: float) -> NumericPolynomial:
    return {(0, 0): float(value)}


def _float_box(
    box: Sequence[Fraction],
) -> tuple[float, float, float, float]:
    if len(box) != 4:
        raise ValueError("P49 box must contain four values")
    return (
        float(box[0]),
        float(box[1]),
        float(box[2]),
        float(box[3]),
    )


def _local_basis_numeric(
    *,
    degree: int,
    box: Sequence[Fraction],
) -> tuple[NumericPolynomial, ...]:
    return tuple(
        _numeric(polynomial)
        for polynomial in local_bernstein_basis_rational_polynomials(
            degree=degree,
            box=box,
        )
    )


def _fixed_degree_bernstein_matrix(
    polynomials: Sequence[NumericPolynomial],
    box: tuple[float, float, float, float],
    *,
    target_degree_x: int,
    target_degree_y: int,
) -> np.ndarray:
    union_powers = tuple(
        sorted(
            {
                power
                for polynomial in polynomials
                for power in polynomial
            },
            key=lambda power: (sum(power), power[0], power[1]),
        )
    )
    if not union_powers:
        raise ValueError("P49 Bernstein family must not be empty")
    if max(power[0] for power in union_powers) > target_degree_x or max(
        power[1] for power in union_powers
    ) > target_degree_y:
        raise ValueError("P49 target Bernstein degree is too small")
    transform = np.column_stack(
        [
            monomial_bernstein_vector(
                power,
                box,
                target_degree_x=target_degree_x,
                target_degree_y=target_degree_y,
            )
            for power in union_powers
        ]
    )
    power_index = {power: index for index, power in enumerate(union_powers)}
    coefficients = np.zeros(
        (len(union_powers), len(polynomials)),
        dtype=np.float64,
    )
    for column, polynomial in enumerate(polynomials):
        for power, coefficient in polynomial.items():
            coefficients[power_index[power], column] = float(coefficient)
    return transform @ coefficients


class _P49SparseLPBuilder:
    """Append bounded local dense blocks into a single global CSR matrix."""

    def __init__(
        self,
        *,
        variable_count: int,
        bounds: Sequence[tuple[float, float]],
    ) -> None:
        if variable_count <= 0 or len(bounds) != variable_count:
            raise ValueError("invalid P49 sparse builder dimensions")
        self.variable_count = int(variable_count)
        self.bounds = tuple((float(low), float(high)) for low, high in bounds)
        self.row_parts: list[np.ndarray] = []
        self.column_parts: list[np.ndarray] = []
        self.data_parts: list[np.ndarray] = []
        self.rhs_parts: list[np.ndarray] = []
        self.row_count = 0
        self.estimated_nonzero_count = 0
        self.constraint_counts: dict[str, int] = {}

    def _record_nonzeros(
        self,
        rows: np.ndarray,
        columns: np.ndarray,
        data: np.ndarray,
    ) -> None:
        count = int(data.size)
        if self.estimated_nonzero_count + count > G11P49_MAX_CSR_NONZEROS:
            raise MemoryError("G11P49 frozen CSR nonzero cap exceeded")
        self.row_parts.append(rows.astype(np.int64, copy=False))
        self.column_parts.append(columns.astype(np.int64, copy=False))
        self.data_parts.append(data.astype(np.float64, copy=False))
        self.estimated_nonzero_count += count

    def add_direct_barrier_nonnegative(
        self,
        *,
        barrier_indices: Sequence[int],
        rho_index: int,
    ) -> None:
        count = len(barrier_indices)
        local_rows = np.arange(count, dtype=np.int64) + self.row_count
        rows = np.concatenate((local_rows, local_rows))
        columns = np.concatenate(
            (
                np.asarray(barrier_indices, dtype=np.int64),
                np.full(count, int(rho_index), dtype=np.int64),
            )
        )
        data = np.concatenate(
            (
                np.full(count, -1.0, dtype=np.float64),
                np.ones(count, dtype=np.float64),
            )
        )
        self._record_nonzeros(rows, columns, data)
        self.rhs_parts.append(np.zeros(count, dtype=np.float64))
        self.row_count += count
        self.constraint_counts["W_nonnegative_X"] = count

    def add_positive_family(
        self,
        name: str,
        *,
        indexed_polynomials: Sequence[tuple[int, NumericPolynomial]],
        box: tuple[float, float, float, float],
        target_degree_x: int,
        target_degree_y: int,
        constant_value: float,
    ) -> None:
        variable_indices = tuple(index for index, _ in indexed_polynomials)
        if len(set(variable_indices)) != len(variable_indices):
            raise ValueError("P49 residual contains a duplicate LP variable")
        local = _fixed_degree_bernstein_matrix(
            tuple(polynomial for _, polynomial in indexed_polynomials),
            box,
            target_degree_x=target_degree_x,
            target_degree_y=target_degree_y,
        )
        local_rows, local_columns = np.nonzero(local)
        if local_rows.size:
            self._record_nonzeros(
                local_rows.astype(np.int64) + self.row_count,
                np.asarray(variable_indices, dtype=np.int64)[local_columns],
                -local[local_rows, local_columns],
            )
        self.rhs_parts.append(
            np.full(local.shape[0], float(constant_value), dtype=np.float64)
        )
        self.row_count += int(local.shape[0])
        self.constraint_counts[name] = self.constraint_counts.get(name, 0) + int(
            local.shape[0]
        )

    def finalize(self) -> tuple[csr_matrix, np.ndarray]:
        if self.row_count <= 0 or not self.rhs_parts:
            raise ValueError("G11P49 LP has no constraints")
        rows = np.concatenate(self.row_parts)
        columns = np.concatenate(self.column_parts)
        data = np.concatenate(self.data_parts)
        matrix = coo_matrix(
            (data, (rows, columns)),
            shape=(self.row_count, self.variable_count),
            dtype=np.float64,
        ).tocsr()
        matrix.sum_duplicates()
        matrix.eliminate_zeros()
        if matrix.nnz > G11P49_MAX_CSR_NONZEROS:
            raise MemoryError("G11P49 finalized CSR nonzero cap exceeded")
        return matrix, np.concatenate(self.rhs_parts)


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


def _peak_rss_bytes() -> int | None:
    status_path = Path("/proc/self/status")
    if not status_path.is_file():
        return None
    for line in status_path.read_text(encoding="utf-8").splitlines():
        if line.startswith("VmHWM:"):
            fields = line.split()
            if len(fields) >= 2:
                return int(fields[1]) * 1024
    return None


def _assert_rss_cap() -> int | None:
    peak = _peak_rss_bytes()
    if peak is not None and peak > G11P49_MAX_RSS_BYTES:
        raise MemoryError("G11P49 frozen peak-RSS cap exceeded")
    return peak


def _record_rss_checkpoint(
    manifest: dict[str, Any],
    checkpoint: str,
) -> int | None:
    peak = _peak_rss_bytes()
    within_cap = peak is None or peak <= G11P49_MAX_RSS_BYTES
    checkpoints = manifest.setdefault("resource_checkpoints", [])
    if not isinstance(checkpoints, list):
        raise ValueError("G11P49 resource checkpoint ledger is invalid")
    checkpoints.append(
        {
            "checkpoint": checkpoint,
            "peak_RSS_bytes": peak,
            "max_RSS_bytes": G11P49_MAX_RSS_BYTES,
            "within_cap": within_cap,
        }
    )
    if not within_cap:
        raise MemoryError(
            f"G11P49 frozen peak-RSS cap exceeded at {checkpoint}"
        )
    return peak


def _assert_frozen_controller(
    controller: FrozenController,
    *,
    controller_file_sha256: str,
) -> dict[str, PolynomialControllerPiece]:
    if controller.sha256 != G11P49_CONTROLLER_CONTENT_SHA256:
        raise ValueError("G11P49 frozen controller content hash mismatch")
    if controller_file_sha256 != G11P49_CONTROLLER_FILE_SHA256:
        raise ValueError("G11P49 frozen controller file hash mismatch")
    pieces = controller_pieces(controller)
    expected = (
        ("lower_entry", -1.0, 0.8),
        ("lower_transit", 0.8, 3.2),
        ("goal_capture", 3.2, 4.0),
    )
    actual = tuple(
        (piece.piece_id, piece.x_min_m, piece.x_max_m) for piece in pieces
    )
    if actual != expected:
        raise ValueError("G11P49 frozen controller partition mismatch")
    return {piece.piece_id: piece for piece in pieces}


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
    return max(
        max(float(low) - float(value), float(value) - float(high), 0.0)
        for value, (low, high) in zip(values, bounds)
    )


def _barrier_indices_for_cell(
    *,
    stage: G11P49StageSpec,
    cell_id: str,
) -> tuple[int, ...]:
    cell = P49_CELL_BY_ID[cell_id]
    size = 7 * stage.barrier_degree + 1
    return tuple(
        (cell.x_index * stage.barrier_degree + local_x) * size
        + cell.y_index * stage.barrier_degree
        + local_y
        for local_x in range(stage.barrier_degree + 1)
        for local_y in range(stage.barrier_degree + 1)
    )


def _block_indices(start: int, degree: int) -> tuple[int, ...]:
    return tuple(range(int(start), int(start) + (degree + 1) ** 2))


def _variable_catalog_payload(
    *,
    stage: G11P49StageSpec,
    layout: _G11P49VariableLayout,
) -> dict[str, Any]:
    payload = {
        "schema_version": 1,
        "kind": "g11p49_lp_variable_catalog",
        "stage_id": stage.stage_id,
        "ordering": "zero_based_half_open_x_major_then_y",
        "barrier": {
            "start": 0,
            "count": layout.barrier_count,
            "global_grid_shape": [
                7 * stage.barrier_degree + 1,
                7 * stage.barrier_degree + 1,
            ],
        },
        "roof_blocks": [
            {
                "source_cell_id": source.cell_id,
                "quadrant": list(quadrant),
                "start": layout.roof_starts[(source.cell_id, quadrant)],
                "count": (stage.roof_degree + 1) ** 2,
            }
            for source in P49_SAFE_SOURCE_CELLS
            for quadrant in P49_QUADRANTS
        ],
        "multiplier_blocks": [
            {
                "multiplier_id": key,
                "start": start,
                "count": (stage.multiplier_degree + 1) ** 2,
            }
            for key, start in layout.multiplier_starts.items()
        ],
        "rho_index": layout.rho_index,
        "variable_count": layout.variable_count,
    }
    payload["catalog_content_sha256"] = content_sha256(payload)
    return payload


def _build_g11p49_problem(
    *,
    controller: FrozenController,
    stage: G11P49StageSpec,
) -> _G11P49Problem:
    """Build the frozen G11P49 stage without a global dense matrix."""

    if stage not in G11P49_STAGE_SCHEDULE:
        raise ValueError("G11P49 stage lies outside the frozen schedule")
    pieces = {piece.piece_id: piece for piece in controller_pieces(controller)}
    if set(pieces) != {"lower_entry", "lower_transit", "goal_capture"}:
        raise ValueError("G11P49 requires the frozen C2 piece identities")
    layout = _build_variable_layout(stage)
    bounds = (
        *(
            (-G11P49_COEFFICIENT_BOUND, G11P49_COEFFICIENT_BOUND)
            for _ in range(layout.barrier_count + layout.roof_count)
        ),
        *(
            (0.0, G11P49_MULTIPLIER_COEFFICIENT_BOUND)
            for _ in range(layout.multiplier_count)
        ),
        G11P49_RHO_BOUNDS,
    )
    builder = _P49SparseLPBuilder(
        variable_count=layout.variable_count,
        bounds=bounds,
    )
    builder.add_direct_barrier_nonnegative(
        barrier_indices=range(layout.barrier_count),
        rho_index=layout.rho_index,
    )
    rho_term = (layout.rho_index, _constant_numeric(-1.0))
    barrier_basis_by_cell = {
        cell.cell_id: _local_basis_numeric(
            degree=stage.barrier_degree,
            box=cell.normalized_box,
        )
        for cell in P49_CELLS
    }

    initial_cell = P49_CELL_BY_ID["C_0_3"]
    initial_w_indices = _barrier_indices_for_cell(
        stage=stage,
        cell_id=initial_cell.cell_id,
    )
    initial_multiplier_basis = _local_basis_numeric(
        degree=stage.multiplier_degree,
        box=initial_cell.normalized_box,
    )
    initial_multiplier_indices = _block_indices(
        layout.multiplier_starts["initial_C_0_3"],
        stage.multiplier_degree,
    )
    initial_guard = initial_disk_guard()
    initial_terms = [
        (index, _scale_numeric(polynomial, -1.0))
        for index, polynomial in zip(
            initial_w_indices,
            barrier_basis_by_cell[initial_cell.cell_id],
        )
    ]
    initial_terms.extend(
        (
            index,
            negate(multiply(polynomial, initial_guard)),
        )
        for index, polynomial in zip(
            initial_multiplier_indices,
            initial_multiplier_basis,
        )
    )
    initial_terms.append(rho_term)
    initial_degree = max(
        stage.barrier_degree,
        stage.multiplier_degree + 2,
    )
    builder.add_positive_family(
        "q_minus_W_on_X0",
        indexed_polynomials=initial_terms,
        box=_float_box(initial_cell.normalized_box),
        target_degree_x=initial_degree,
        target_degree_y=initial_degree,
        constant_value=G11P49_Q,
    )

    for cell in P49_UNSAFE_CELLS:
        indices = _barrier_indices_for_cell(
            stage=stage,
            cell_id=cell.cell_id,
        )
        terms = list(zip(indices, barrier_basis_by_cell[cell.cell_id]))
        terms.append(rho_term)
        builder.add_positive_family(
            "W_minus_1_on_Xu",
            indexed_polynomials=terms,
            box=_float_box(cell.normalized_box),
            target_degree_x=stage.barrier_degree,
            target_degree_y=stage.barrier_degree,
            constant_value=-1.0,
        )

    roof_expectation_basis: dict[
        tuple[str, tuple[int, int]], tuple[NumericPolynomial, ...]
    ] = {}
    target_intersection_count = 0
    for source in P49_SAFE_SOURCE_CELLS:
        piece = pieces[source.controller_piece_id]
        for quadrant in P49_QUADRANTS:
            roof_key = (source.cell_id, quadrant)
            reach_box = p49_quadrant_reach_box(source, quadrant)
            roof_basis_rational = local_bernstein_basis_rational_polynomials(
                degree=stage.roof_degree,
                box=reach_box,
            )
            roof_basis = tuple(
                _numeric(polynomial) for polynomial in roof_basis_rational
            )
            roof_expectation_basis[roof_key] = tuple(
                _numeric(polynomial)
                for polynomial in exact_polynomial_family_quadrant_expectations(
                    polynomials=roof_basis_rational,
                    quadrant=quadrant,
                    piece=piece,
                )
            )
            roof_indices = _block_indices(
                layout.roof_starts[roof_key],
                stage.roof_degree,
            )
            roof_terms = tuple(zip(roof_indices, roof_basis))
            for target, intersection in p49_reach_target_intersections(
                source,
                quadrant,
            ):
                target_intersection_count += 1
                target_indices = _barrier_indices_for_cell(
                    stage=stage,
                    cell_id=target.cell_id,
                )
                terms = list(roof_terms)
                terms.extend(
                    (
                        index,
                        _scale_numeric(polynomial, -1.0),
                    )
                    for index, polynomial in zip(
                        target_indices,
                        barrier_basis_by_cell[target.cell_id],
                    )
                )
                terms.append(rho_term)
                builder.add_positive_family(
                    "roof_domination",
                    indexed_polynomials=terms,
                    box=_float_box(intersection),
                    target_degree_x=stage.roof_degree,
                    target_degree_y=stage.roof_degree,
                    constant_value=0.0,
                )
            del roof_basis_rational, roof_basis
            _assert_rss_cap()
    if target_intersection_count != 1012:
        raise ValueError("G11P49 reach-intersection count drifted")

    goal_guard = goal_exterior_guard()
    goal_boundary_ids = {
        cell.cell_id for cell in P49_GOAL_BOUNDARY_SAFE_CELLS
    }
    for source in P49_SAFE_SOURCE_CELLS:
        source_w_indices = _barrier_indices_for_cell(
            stage=stage,
            cell_id=source.cell_id,
        )
        terms = list(
            zip(source_w_indices, barrier_basis_by_cell[source.cell_id])
        )
        for quadrant in P49_QUADRANTS:
            roof_key = (source.cell_id, quadrant)
            roof_indices = _block_indices(
                layout.roof_starts[roof_key],
                stage.roof_degree,
            )
            terms.extend(
                (
                    index,
                    _scale_numeric(polynomial, -1.0),
                )
                for index, polynomial in zip(
                    roof_indices,
                    roof_expectation_basis[roof_key],
                )
            )
        if source.cell_id in goal_boundary_ids:
            multiplier_key = f"goal_exterior_{source.cell_id}"
            multiplier_indices = _block_indices(
                layout.multiplier_starts[multiplier_key],
                stage.multiplier_degree,
            )
            multiplier_basis = _local_basis_numeric(
                degree=stage.multiplier_degree,
                box=source.normalized_box,
            )
            terms.extend(
                (
                    index,
                    negate(multiply(polynomial, goal_guard)),
                )
                for index, polynomial in zip(
                    multiplier_indices,
                    multiplier_basis,
                )
            )
        terms.append(rho_term)
        builder.add_positive_family(
            "integrated_roof_drift",
            indexed_polynomials=terms,
            box=_float_box(source.normalized_box),
            target_degree_x=stage.roof_degree,
            target_degree_y=2 * stage.roof_degree,
            constant_value=-G11P49_EPSBAR,
        )
    del roof_expectation_basis, barrier_basis_by_cell
    gc.collect()

    matrix, right_hand_side = builder.finalize()
    if matrix.shape != (
        stage.expected_linear_row_count,
        stage.expected_variable_count,
    ):
        raise ValueError("G11P49 frozen LP shape drifted")
    if not np.isfinite(matrix.data).all() or not np.isfinite(
        right_hand_side
    ).all():
        raise ValueError("G11P49 LP contains a non-finite numeric coefficient")
    peak_rss = _assert_rss_cap()
    objective = np.zeros(layout.variable_count, dtype=np.float64)
    objective[layout.rho_index] = -1.0
    variable_catalog = _variable_catalog_payload(stage=stage, layout=layout)
    metadata = {
        "matrix_format": "csr",
        "global_dense_matrix_constructed": False,
        "decision_variable_count": int(matrix.shape[1]),
        "total_linear_inequalities": int(matrix.shape[0]),
        "matrix_nonzero_count": int(matrix.nnz),
        "matrix_minimum_nonzero_abs": float(np.min(np.abs(matrix.data))),
        "matrix_maximum_abs": float(np.max(np.abs(matrix.data))),
        "matrix_csr_bytes": int(
            matrix.data.nbytes + matrix.indices.nbytes + matrix.indptr.nbytes
        ),
        "max_CSR_nonzeros": G11P49_MAX_CSR_NONZEROS,
        "peak_RSS_bytes_after_build": peak_rss,
        "max_RSS_bytes": G11P49_MAX_RSS_BYTES,
        "barrier_coefficient_count": layout.barrier_count,
        "roof_polynomial_count": 4 * len(P49_SAFE_SOURCE_CELLS),
        "roof_coefficient_count": layout.roof_count,
        "shape_multiplier_polynomial_count": len(layout.multiplier_starts),
        "shape_multiplier_coefficient_count": layout.multiplier_count,
        "rho_variable_count": 1,
        "target_intersection_count": target_intersection_count,
        "constraint_counts": dict(builder.constraint_counts),
        "variable_catalog": variable_catalog,
    }
    return _G11P49Problem(
        matrix=matrix,
        right_hand_side=right_hand_side,
        objective=objective,
        bounds=builder.bounds,
        layout=layout,
        metadata=metadata,
    )


def _artifacts_from_values(
    *,
    values: np.ndarray,
    controller: FrozenController,
    stage: G11P49StageSpec,
    layout: _G11P49VariableLayout,
) -> tuple[PiecewiseBernsteinBarrier2D, PiecewiseRoofWitness2D]:
    barrier_size = 7 * stage.barrier_degree + 1
    barrier_values = values[: layout.barrier_count].reshape(
        (barrier_size, barrier_size)
    )
    barrier = PiecewiseBernsteinBarrier2D(
        degree=stage.barrier_degree,
        control_grid=tuple(
            tuple(float(value) for value in row) for row in barrier_values
        ),
    )
    roofs: list[LocalBernsteinRoof2D] = []
    roof_size = stage.roof_degree + 1
    for source in P49_SAFE_SOURCE_CELLS:
        for quadrant in P49_QUADRANTS:
            start = layout.roof_starts[(source.cell_id, quadrant)]
            block = values[start : start + roof_size**2].reshape(
                (roof_size, roof_size)
            )
            roofs.append(
                LocalBernsteinRoof2D(
                    source_cell_id=source.cell_id,
                    controller_piece_id=source.controller_piece_id,
                    quadrant=quadrant,
                    normalized_reach_box=p49_quadrant_reach_box(
                        source,
                        quadrant,
                    ),
                    degree=stage.roof_degree,
                    control_grid=tuple(
                        tuple(float(value) for value in row) for row in block
                    ),
                )
            )
    model_hash = content_sha256(canonical_rational_model_payload())
    witness = PiecewiseRoofWitness2D(
        barrier_payload_sha256=barrier.payload_sha256,
        controller_content_sha256=controller.sha256,
        formal_model_contract_sha256=model_hash,
        geometry_catalog_sha256=p49_geometry_catalog_sha256(),
        barrier_degree=stage.barrier_degree,
        roof_degree=stage.roof_degree,
        roofs=tuple(roofs),
    )
    return barrier, witness


def _tree_binding_failures(
    *,
    validation: Mapping[str, Any],
    validation_directory: Path,
) -> list[str]:
    failures: list[str] = []
    root = validation_directory.resolve()
    seen_tree_paths: set[Path] = set()
    groups = (
        "qsbc_constraints",
        "roof_domination_constraints",
        "integrated_drift_constraints",
        "controller_constraints",
    )
    for group in groups:
        constraints = validation.get(group)
        if not isinstance(constraints, dict):
            failures.append(f"{group}_missing")
            continue
        for name, summary in constraints.items():
            if not isinstance(summary, dict):
                failures.append(f"{group}_{name}_not_mapping")
                continue
            if summary.get("status") != "validated":
                failures.append(f"{group}_{name}_not_validated")
            relative = summary.get("tree_file")
            declared_hash = summary.get("tree_file_sha256")
            if not isinstance(relative, str) or not isinstance(
                declared_hash,
                str,
            ):
                failures.append(f"{group}_{name}_tree_binding_missing")
                continue
            tree_path = (root / relative).resolve()
            try:
                tree_path.relative_to(root)
            except ValueError:
                failures.append(f"{group}_{name}_tree_path_escape")
                continue
            if tree_path in seen_tree_paths:
                failures.append(f"{group}_{name}_duplicate_tree_path")
                continue
            seen_tree_paths.add(tree_path)
            if not tree_path.is_file() or file_sha256(tree_path) != declared_hash:
                failures.append(f"{group}_{name}_tree_hash_mismatch")
                continue
            try:
                tree_payload = json.loads(tree_path.read_text(encoding="utf-8"))
            except (OSError, UnicodeError, json.JSONDecodeError):
                failures.append(f"{group}_{name}_tree_not_canonical_json")
                continue
            if not isinstance(tree_payload, dict):
                failures.append(f"{group}_{name}_tree_not_mapping")
                continue
            tree_summary = dict(tree_payload)
            tree_summary.pop("leaves", None)
            report_summary = dict(summary)
            report_summary.pop("tree_file", None)
            report_summary.pop("tree_file_sha256", None)
            if tree_summary != report_summary:
                failures.append(f"{group}_{name}_tree_summary_mismatch")
    return failures


def _expected_v3p_constraint_names(
    *,
    witness: PiecewiseRoofWitness2D,
    controller: FrozenController,
) -> dict[str, set[str]]:
    roofs = {
        (roof.source_cell_id, roof.quadrant): roof for roof in witness.roofs
    }
    return {
        "qsbc_constraints": {
            *(f"W_nonnegative_{cell.cell_id}" for cell in P49_CELLS),
            "q_minus_W_on_X0",
            *(f"W_minus_1_{cell.cell_id}" for cell in P49_UNSAFE_CELLS),
        },
        "roof_domination_constraints": {
            (
                f"{roofs[(source.cell_id, quadrant)].roof_id}"
                f"_dominates_{target.cell_id}"
            )
            for source in P49_SAFE_SOURCE_CELLS
            for quadrant in P49_QUADRANTS
            for target, _ in p49_reach_target_intersections(source, quadrant)
        },
        "integrated_drift_constraints": {
            f"integrated_roof_drift_{source.cell_id}"
            for source in P49_SAFE_SOURCE_CELLS
        },
        "controller_constraints": {
            f"{piece.piece_id}_{suffix}"
            for piece in controller_pieces(controller)
            for suffix in (
                "action_disk",
                "successor_x_lower",
                "successor_x_upper",
                "successor_y_lower",
                "successor_y_upper",
            )
        },
    }


def _v3p_release_gate(
    *,
    validation: Mapping[str, Any],
    validation_directory: Path,
    barrier: PiecewiseBernsteinBarrier2D,
    witness: PiecewiseRoofWitness2D,
    controller: FrozenController,
) -> tuple[bool, tuple[str, ...]]:
    failures: list[str] = []
    expected_scalars = {
        "q_float64_hex": G11P49_Q.hex(),
        "epsbar_float64_hex": G11P49_EPSBAR.hex(),
    }
    expected_bindings = {
        "barrier_payload_sha256": barrier.payload_sha256,
        "roof_witness_content_sha256": witness.sha256,
        "controller_content_sha256": controller.sha256,
        "formal_model_contract_sha256": content_sha256(
            canonical_rational_model_payload()
        ),
        "geometry_catalog_sha256": p49_geometry_catalog_sha256(),
    }
    expected_constraint_names = _expected_v3p_constraint_names(
        witness=witness,
        controller=controller,
    )
    if type(validation.get("schema_version")) is not int or validation.get(
        "schema_version"
    ) != 1:
        failures.append("schema_version")
    if validation.get("kind") != "independent_piecewise_qsbc_validation":
        failures.append("kind")
    if validation.get("validator_id") != G11P49_VALIDATOR_ID:
        failures.append("validator_id")
    if validation.get("proof_route_id") != G11P49_PROOF_ROUTE_ID:
        failures.append("proof_route_id")
    if validation.get("status") != "validated":
        failures.append("status")
    if validation.get("first_slice_ready") is not True:
        failures.append("first_slice_ready")
    if type(validation.get("formal_acceptance_tolerance")) is not float or (
        validation.get("formal_acceptance_tolerance") != 0.0
    ):
        failures.append("formal_acceptance_tolerance")
    for name, expected in expected_scalars.items():
        if validation.get(name) != expected:
            failures.append(name)
    if (
        type(validation.get("q")) is not float
        or float(validation["q"]).hex() != G11P49_Q.hex()
        or validation.get("q_exact") != str(Fraction.from_float(G11P49_Q))
    ):
        failures.append("q_decimal_hex_exact_binding")
    if (
        type(validation.get("epsbar")) is not float
        or float(validation["epsbar"]).hex() != G11P49_EPSBAR.hex()
        or validation.get("epsbar_exact")
        != str(Fraction.from_float(G11P49_EPSBAR))
    ):
        failures.append("epsbar_decimal_hex_exact_binding")
    if validation.get("first_slice_q_max_exact") != "1/10":
        failures.append("first_slice_q_max_exact")
    if validation.get("theorem_scalar_gate") is not True:
        failures.append("theorem_scalar_gate")
    for name, expected in expected_bindings.items():
        if validation.get(name) != expected:
            failures.append(name)
    if validation.get("barrier_degree") != barrier.degree:
        failures.append("barrier_degree")
    if validation.get("roof_degree") != witness.roof_degree:
        failures.append("roof_degree")
    if validation.get("roof_count") != 148:
        failures.append("roof_count")
    if validation.get("constraint_counts") != {
        "qsbc": 62,
        "roof_domination": 1012,
        "integrated_drift": 37,
        "controller": 15,
    }:
        failures.append("constraint_counts")
    for group, expected_names in expected_constraint_names.items():
        constraints = validation.get(group)
        if not isinstance(constraints, dict) or set(constraints) != expected_names:
            failures.append(f"{group}_canonical_names")
    c0_gate = validation.get("c0_gate")
    if not isinstance(c0_gate, dict) or c0_gate.get("status") != "validated" or (
        c0_gate.get("checked_internal_edge_count") != 84
    ):
        failures.append("c0_gate")
    p49_gate = validation.get("p49_catalog_gate")
    if not isinstance(p49_gate, dict) or p49_gate.get("status") != "validated":
        failures.append("p49_catalog_gate")
    arithmetic_gate = validation.get("v3p_arithmetic_gate")
    if (
        not isinstance(arithmetic_gate, dict)
        or arithmetic_gate.get("status") != "validated"
    ):
        failures.append("v3p_arithmetic_gate")
    geometry_gate = validation.get("geometry_gate")
    if (
        not isinstance(geometry_gate, dict)
        or geometry_gate.get("status") != "validated"
    ):
        failures.append("geometry_gate")
    model_payload = validation.get("formal_model_contract")
    if not isinstance(model_payload, dict) or content_sha256(model_payload) != (
        expected_bindings["formal_model_contract_sha256"]
    ):
        failures.append("formal_model_contract_payload")
    report_payload = dict(validation)
    declared_report_hash = report_payload.pop("report_content_sha256", None)
    if not isinstance(declared_report_hash, str) or content_sha256(
        report_payload
    ) != declared_report_hash:
        failures.append("report_content_sha256")
    failures.extend(
        _tree_binding_failures(
            validation=validation,
            validation_directory=validation_directory,
        )
    )
    return not failures, tuple(failures)


def _publication_disk_release_gate(
    *,
    validation_path: Path,
    witness_path: Path,
    pending_certificate_path: Path,
    expected_validation_file_sha256: str,
    expected_validation_content_sha256: str,
    expected_witness_file_sha256: str,
    expected_pending_certificate_file_sha256: str,
    expected_witness: PiecewiseRoofWitness2D,
    expected_certificate: QSBCertificate,
    barrier: PiecewiseBernsteinBarrier2D,
    controller: FrozenController,
) -> tuple[bool, tuple[str, ...]]:
    """Re-read every formal subject from disk immediately before rename."""

    failures: list[str] = []
    try:
        disk_witness = PiecewiseRoofWitness2D.load(witness_path)
    except (OSError, UnicodeError, json.JSONDecodeError, ValueError):
        disk_witness = None
        failures.append("roof_witness_disk_reload")
    if disk_witness is not None:
        if disk_witness != expected_witness:
            failures.append("roof_witness_disk_subject_changed")
        if file_sha256(witness_path) != expected_witness_file_sha256:
            failures.append("roof_witness_file_changed")
        if disk_witness.sha256 != expected_witness.sha256:
            failures.append("roof_witness_content_changed")

    try:
        disk_validation = json.loads(validation_path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError):
        disk_validation = None
        failures.append("validation_report_disk_reload")
    if disk_validation is not None:
        if file_sha256(validation_path) != expected_validation_file_sha256:
            failures.append("validation_report_file_changed")
        if disk_validation.get("report_content_sha256") != (
            expected_validation_content_sha256
        ):
            failures.append("validation_report_content_changed")
        if disk_witness is not None:
            gate_passed, gate_failures = _v3p_release_gate(
                validation=disk_validation,
                validation_directory=validation_path.parent,
                barrier=barrier,
                witness=disk_witness,
                controller=controller,
            )
            if not gate_passed:
                failures.extend(gate_failures)
    try:
        disk_certificate = QSBCertificate.load(
            pending_certificate_path,
            controller=controller,
        )
    except (OSError, UnicodeError, json.JSONDecodeError, ValueError):
        disk_certificate = None
        failures.append("pending_certificate_disk_reload")
    if disk_certificate is not None:
        if file_sha256(pending_certificate_path) != (
            expected_pending_certificate_file_sha256
        ):
            failures.append("pending_certificate_file_changed")
        if disk_certificate != expected_certificate or disk_certificate.sha256 != (
            expected_certificate.sha256
        ):
            failures.append("pending_certificate_subject_changed")
        if disk_certificate.proof_witness.get(
            "roof_witness_content_sha256"
        ) != (disk_witness.sha256 if disk_witness is not None else None):
            failures.append("pending_certificate_witness_binding_changed")
        if disk_certificate.validation.get(
            "validation_report_content_sha256"
        ) != (
            disk_validation.get("report_content_sha256")
            if disk_validation is not None
            else None
        ):
            failures.append("pending_certificate_validation_binding_changed")
    return not failures, tuple(failures)


def _solve_g11p49_stage_impl(
    *,
    frozen_controller_path: str | Path,
    output_dir: str | Path,
    stage: G11P49StageSpec,
) -> Path:
    """Solve one numeric P49 LP and publish only after independent V3P."""

    if stage not in G11P49_STAGE_SCHEDULE:
        raise ValueError("G11P49 stage lies outside the frozen schedule")
    controller_path = Path(frozen_controller_path).resolve()
    controller = load_frozen_controller(controller_path)
    controller_file_hash = file_sha256(controller_path)
    _assert_frozen_controller(
        controller,
        controller_file_sha256=controller_file_hash,
    )
    destination = Path(output_dir).resolve()
    destination.mkdir(parents=True, exist_ok=False)
    manifest_path = destination / "manifest.json"
    manifest: dict[str, Any] = {
        "schema_version": 1,
        "kind": "g11p49_piecewise_roof_lp_synthesis_attempt",
        "synthesis_id": G11P49_SYNTHESIS_ID,
        "proof_route_id": G11P49_PROOF_ROUTE_ID,
        "status": "building",
        "stage": {
            "stage_id": stage.stage_id,
            "barrier_degree": stage.barrier_degree,
            "roof_degree": stage.roof_degree,
            "multiplier_degree": stage.multiplier_degree,
            "time_limit_s": stage.time_limit_s,
            "expected_variable_count": stage.expected_variable_count,
            "expected_linear_row_count": stage.expected_linear_row_count,
        },
        "fixed_scalars": {
            "q": G11P49_Q,
            "q_float64_hex": G11P49_Q.hex(),
            "epsbar": G11P49_EPSBAR,
            "epsbar_float64_hex": G11P49_EPSBAR.hex(),
            "rho_bounds": list(G11P49_RHO_BOUNDS),
        },
        "controller": {
            "file_sha256": controller_file_hash,
            "content_sha256": controller.sha256,
            "frozen_file_sha256": G11P49_CONTROLLER_FILE_SHA256,
            "frozen_content_sha256": G11P49_CONTROLLER_CONTENT_SHA256,
        },
        "geometry_catalog_sha256": p49_geometry_catalog_sha256(),
        "solver": {
            "name": "SciPy HiGHS LP",
            "method": "highs",
            "primal_feasibility_tolerance": 1.0e-9,
            "dual_feasibility_tolerance": 1.0e-9,
            "solver_status_is_not_formal_evidence": True,
        },
        "independent_validation": {
            "required": G11P49_VALIDATOR_ID,
            "reads_solver_status": False,
            "reads_LP_matrix": False,
            "reads_multiplier_values": False,
            "acceptance_tolerance": 0.0,
        },
    }
    _save_manifest(manifest_path, manifest)
    try:
        problem = _build_g11p49_problem(controller=controller, stage=stage)
    except MemoryError as error:
        manifest["status"] = "not_ready_resource_cap_exceeded"
        manifest["resource_failure"] = str(error)
        manifest["peak_RSS_bytes"] = _peak_rss_bytes()
        _save_manifest(manifest_path, manifest)
        return manifest_path

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
        "iterations": int(getattr(result, "nit", 0) or 0),
        "optimality_required_for_formal_claim": False,
    }
    values = _candidate_vector(result, problem.layout.variable_count)
    if values is None:
        manifest["status"] = "not_ready_no_finite_numeric_candidate"
        _save_manifest(manifest_path, manifest)
        return manifest_path

    rho = float(values[problem.layout.rho_index])
    minimum_lp_slack = float(
        np.min(problem.right_hand_side - problem.matrix @ values)
    )
    maximum_bound_violation = _maximum_bound_violation(values, problem.bounds)
    numeric_incumbent_feasible = (
        minimum_lp_slack >= -G11P49_NUMERIC_FEASIBILITY_TOLERANCE
        and maximum_bound_violation <= G11P49_NUMERIC_FEASIBILITY_TOLERANCE
    )
    manifest["numeric_incumbent"] = {
        "rho": rho,
        "rho_float64_hex": rho.hex(),
        "minimum_lp_slack": minimum_lp_slack,
        "maximum_bound_violation": maximum_bound_violation,
        "feasibility_tolerance": G11P49_NUMERIC_FEASIBILITY_TOLERANCE,
        "within_numeric_solver_tolerance": numeric_incumbent_feasible,
        "formal_claim_allowed": False,
    }
    if not rho > 0.0:
        manifest["status"] = "not_ready_nonpositive_rho"
        _save_manifest(manifest_path, manifest)
        return manifest_path
    if not numeric_incumbent_feasible:
        manifest["status"] = "not_ready_numeric_incumbent_outside_tolerance"
        _save_manifest(manifest_path, manifest)
        return manifest_path

    barrier, witness = _artifacts_from_values(
        values=values,
        controller=controller,
        stage=stage,
        layout=problem.layout,
    )
    witness_path = witness.save(destination / "roof_witness.json")
    multiplier_start = problem.layout.barrier_count + problem.layout.roof_count
    multiplier_values = values[multiplier_start : problem.layout.rho_index]
    candidate = {
        "schema_version": 1,
        "kind": "g11p49_piecewise_roof_numeric_candidate",
        "synthesis_id": G11P49_SYNTHESIS_ID,
        "proof_route_id": G11P49_PROOF_ROUTE_ID,
        "stage_id": stage.stage_id,
        "controller_content_sha256": controller.sha256,
        "geometry_catalog_sha256": p49_geometry_catalog_sha256(),
        "variable_catalog_content_sha256": problem_metadata["variable_catalog"][
            "catalog_content_sha256"
        ],
        "barrier": barrier.payload(),
        "barrier_payload_sha256": barrier.payload_sha256,
        "roof_witness": {
            "file": witness_path.name,
            "file_sha256": file_sha256(witness_path),
            "content_sha256": witness.sha256,
        },
        "q": G11P49_Q,
        "q_float64_hex": G11P49_Q.hex(),
        "epsbar": G11P49_EPSBAR,
        "epsbar_float64_hex": G11P49_EPSBAR.hex(),
        "rho": rho,
        "rho_float64_hex": rho.hex(),
        "minimum_lp_slack": minimum_lp_slack,
        "shape_multiplier_summary": {
            "polynomial_count": len(problem.layout.multiplier_starts),
            "coefficient_count": int(multiplier_values.size),
            "minimum": float(np.min(multiplier_values)),
            "maximum": float(np.max(multiplier_values)),
            "values_not_consumed_by_independent_validator": True,
        },
        "formal_claim_allowed_before_independent_validation": False,
    }
    candidate["content_sha256"] = content_sha256(candidate)
    candidate_path = destination / "numeric_candidate.json"
    _write_json(candidate_path, candidate)
    manifest["numeric_candidate"] = {
        "file": candidate_path.name,
        "file_sha256": file_sha256(candidate_path),
        "content_sha256": candidate["content_sha256"],
    }
    manifest["status"] = "positive_rho_pending_v3p"
    _save_manifest(manifest_path, manifest)

    del multiplier_values, values, result, problem
    gc.collect()
    try:
        _record_rss_checkpoint(manifest, "before_v3p")
    except MemoryError as error:
        manifest["status"] = "not_ready_resource_cap_exceeded"
        manifest["resource_failure"] = str(error)
        manifest["peak_RSS_bytes"] = _peak_rss_bytes()
        _save_manifest(manifest_path, manifest)
        return manifest_path
    _save_manifest(manifest_path, manifest)

    from .piecewise_roof_validation import validate_piecewise_qsbc_v3p

    try:
        validation_path = validate_piecewise_qsbc_v3p(
            barrier=barrier,
            roof_witness=witness,
            q=G11P49_Q,
            epsbar=G11P49_EPSBAR,
            controller=controller,
            output_dir=destination / "independent_validation_v3p",
        )
        _record_rss_checkpoint(manifest, "after_v3p")
    except MemoryError as error:
        manifest["status"] = "not_ready_resource_cap_exceeded"
        manifest["resource_failure"] = str(error)
        manifest["peak_RSS_bytes"] = _peak_rss_bytes()
        _save_manifest(manifest_path, manifest)
        return manifest_path
    validation = json.loads(validation_path.read_text(encoding="utf-8"))
    release_gate_passed, release_gate_failures = _v3p_release_gate(
        validation=validation,
        validation_directory=validation_path.parent,
        barrier=barrier,
        witness=witness,
        controller=controller,
    )
    manifest["independent_validation"] = {
        "file": str(validation_path.relative_to(destination)),
        "file_sha256": file_sha256(validation_path),
        "content_sha256": validation.get("report_content_sha256"),
        "validator_id": validation.get("validator_id"),
        "status": validation.get("status"),
        "first_slice_ready": validation.get("first_slice_ready"),
        "acceptance_tolerance": validation.get("formal_acceptance_tolerance"),
        "barrier_payload_sha256": validation.get("barrier_payload_sha256"),
        "roof_witness_content_sha256": validation.get(
            "roof_witness_content_sha256"
        ),
        "release_gate_passed": release_gate_passed,
        "release_gate_failures": list(release_gate_failures),
    }
    if not release_gate_passed:
        manifest["status"] = "not_ready_v3p_release_gate_failed"
        _save_manifest(manifest_path, manifest)
        return manifest_path

    validation_content_hash = str(validation["report_content_sha256"])
    proof_binding = {
        "schema_version": 1,
        "kind": "g11p49_piecewise_roof_witness_binding",
        "proof_route_id": G11P49_PROOF_ROUTE_ID,
        "validator_id": G11P49_VALIDATOR_ID,
        "barrier_degree": barrier.degree,
        "roof_degree": witness.roof_degree,
        "barrier_payload_sha256": barrier.payload_sha256,
        "controller_content_sha256": controller.sha256,
        "geometry_catalog_sha256": p49_geometry_catalog_sha256(),
        "roof_witness_content_sha256": witness.sha256,
        "formal_model_contract_sha256": witness.formal_model_contract_sha256,
        "validation_report_content_sha256": validation_content_hash,
        "roof_witness_file": witness_path.name,
        "validation_report_file": str(validation_path.relative_to(destination)),
    }
    certificate = QSBCertificate(
        certificate_id=(
            f"CBF_ROVER_G11P49_{stage.stage_id}_{controller.sha256[:12]}"
        ),
        barrier=barrier,
        q=G11P49_Q,
        epsbar=G11P49_EPSBAR,
        controller_content_sha256=controller.sha256,
        schema_version=3,
        solver={
            "method": "P49_C0_Bernstein_quadrant_roof_LP",
            "synthesis_id": G11P49_SYNTHESIS_ID,
            "proof_route_id": G11P49_PROOF_ROUTE_ID,
            "solver": "SciPy HiGHS",
            "stage_id": stage.stage_id,
            "barrier_degree": stage.barrier_degree,
            "roof_degree": stage.roof_degree,
            "multiplier_degree": stage.multiplier_degree,
            "rho": rho,
            "minimum_lp_slack": minimum_lp_slack,
            "constraint_count": problem_metadata["total_linear_inequalities"],
            "decision_variable_count": problem_metadata["decision_variable_count"],
            "matrix_nonzero_count": problem_metadata["matrix_nonzero_count"],
        },
        validation={
            "validator_id": G11P49_VALIDATOR_ID,
            "validation_report_sha256": file_sha256(validation_path),
            "validation_report_content_sha256": validation_content_hash,
            "status": "validated",
            "first_slice_ready": True,
            "formal_acceptance_tolerance": 0.0,
            "barrier_payload_sha256": barrier.payload_sha256,
            "roof_witness_content_sha256": witness.sha256,
            "controller_content_sha256": controller.sha256,
            "formal_model_contract_sha256": (
                witness.formal_model_contract_sha256
            ),
            "geometry_catalog_sha256": p49_geometry_catalog_sha256(),
            "q_float64_hex": G11P49_Q.hex(),
            "epsbar_float64_hex": G11P49_EPSBAR.hex(),
            "barrier_degree": barrier.degree,
            "roof_degree": witness.roof_degree,
        },
        proof_witness=proof_binding,
    )
    pending_path = destination / "certificate.pending.json"
    certificate_path = destination / "certificate.json"
    try:
        reconstructed_witness = PiecewiseRoofWitness2D.load(witness_path)
        if reconstructed_witness != witness:
            raise ValueError("G11P49 roof-witness reconstruction mismatch")
        _record_rss_checkpoint(manifest, "before_pending_certificate")
        certificate.save(pending_path)
        reconstructed = QSBCertificate.load(
            pending_path,
            controller=controller,
        )
        if reconstructed.schema_version != 3 or reconstructed.sha256 != (
            certificate.sha256
        ):
            raise ValueError("G11P49 pending certificate reconstruction mismatch")
        _record_rss_checkpoint(manifest, "after_pending_reconstruction")
        pending_file_hash = file_sha256(pending_path)
        publication_gate_passed, publication_gate_failures = (
            _publication_disk_release_gate(
                validation_path=validation_path,
                witness_path=witness_path,
                pending_certificate_path=pending_path,
                expected_validation_file_sha256=certificate.validation[
                    "validation_report_sha256"
                ],
                expected_validation_content_sha256=validation_content_hash,
                expected_witness_file_sha256=candidate["roof_witness"][
                    "file_sha256"
                ],
                expected_pending_certificate_file_sha256=pending_file_hash,
                expected_witness=witness,
                expected_certificate=certificate,
                barrier=barrier,
                controller=controller,
            )
        )
        publication_failures = list(publication_gate_failures)
        manifest["independent_validation"][
            "publication_release_gate_passed"
        ] = publication_gate_passed
        manifest["independent_validation"][
            "publication_release_gate_failures"
        ] = publication_failures
        if not publication_gate_passed:
            manifest["status"] = "not_ready_publication_release_gate_failed"
            pending_path.unlink(missing_ok=True)
            certificate_path.unlink(missing_ok=True)
            _save_manifest(manifest_path, manifest)
            return manifest_path
        _record_rss_checkpoint(manifest, "before_atomic_certificate_rename")
        final_publication_gate_passed, final_publication_gate_failures = (
            _publication_disk_release_gate(
                validation_path=validation_path,
                witness_path=witness_path,
                pending_certificate_path=pending_path,
                expected_validation_file_sha256=certificate.validation[
                    "validation_report_sha256"
                ],
                expected_validation_content_sha256=validation_content_hash,
                expected_witness_file_sha256=candidate["roof_witness"][
                    "file_sha256"
                ],
                expected_pending_certificate_file_sha256=pending_file_hash,
                expected_witness=witness,
                expected_certificate=certificate,
                barrier=barrier,
                controller=controller,
            )
        )
        if not final_publication_gate_passed:
            manifest["independent_validation"][
                "publication_release_gate_passed"
            ] = False
            manifest["independent_validation"][
                "publication_release_gate_failures"
            ] = [
                *publication_failures,
                *final_publication_gate_failures,
            ]
            manifest["status"] = "not_ready_publication_release_gate_failed"
            pending_path.unlink(missing_ok=True)
            certificate_path.unlink(missing_ok=True)
            _save_manifest(manifest_path, manifest)
            return manifest_path
        pending_path.replace(certificate_path)
        _record_rss_checkpoint(manifest, "after_atomic_certificate_rename")
        manifest["certificate"] = {
            "file": certificate_path.name,
            "file_sha256": file_sha256(certificate_path),
            "content_sha256": certificate.sha256,
            "publication": "pending_then_atomic_final",
        }
        _record_rss_checkpoint(manifest, "final_ready_manifest")
        manifest["status"] = "ready"
        _save_manifest(manifest_path, manifest)
    except MemoryError as error:
        pending_path.unlink(missing_ok=True)
        certificate_path.unlink(missing_ok=True)
        manifest["status"] = "not_ready_resource_cap_exceeded"
        manifest["resource_failure"] = str(error)
        manifest["peak_RSS_bytes"] = _peak_rss_bytes()
        _save_manifest(manifest_path, manifest)
        return manifest_path
    except BaseException:
        pending_path.unlink(missing_ok=True)
        certificate_path.unlink(missing_ok=True)
        raise
    return manifest_path


def solve_g11p49_stage(
    *,
    frozen_controller_path: str | Path,
    output_dir: str | Path,
    stage: G11P49StageSpec,
) -> Path:
    """Fail closed on resource exhaustion while preserving program errors."""

    destination = Path(output_dir).resolve()
    manifest_path = destination / "manifest.json"
    try:
        return _solve_g11p49_stage_impl(
            frozen_controller_path=frozen_controller_path,
            output_dir=destination,
            stage=stage,
        )
    except MemoryError as error:
        destination.mkdir(parents=True, exist_ok=True)
        (destination / "certificate.pending.json").unlink(missing_ok=True)
        (destination / "certificate.json").unlink(missing_ok=True)
        try:
            manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        except (OSError, UnicodeError, json.JSONDecodeError):
            manifest = {
                "schema_version": 1,
                "kind": "g11p49_piecewise_roof_lp_synthesis_attempt",
                "synthesis_id": G11P49_SYNTHESIS_ID,
                "proof_route_id": G11P49_PROOF_ROUTE_ID,
                "stage": {"stage_id": stage.stage_id},
            }
        manifest["status"] = "not_ready_resource_cap_exceeded"
        manifest["resource_failure"] = str(error)
        peak = _peak_rss_bytes()
        manifest["peak_RSS_bytes"] = peak
        checkpoints = manifest.setdefault("resource_checkpoints", [])
        if isinstance(checkpoints, list):
            checkpoints.append(
                {
                    "checkpoint": "outer_resource_exception_boundary",
                    "peak_RSS_bytes": peak,
                    "max_RSS_bytes": G11P49_MAX_RSS_BYTES,
                    "within_cap": False,
                }
            )
        _save_manifest(manifest_path, manifest)
        return manifest_path


def synthesize_g11p49_qsbc_schedule(
    *,
    frozen_controller_path: str | Path,
    output_dir: str | Path,
) -> Path:
    """Run the immutable three-stage P49 fallback and stop at first release."""

    controller_path = Path(frozen_controller_path).resolve()
    controller = load_frozen_controller(controller_path)
    controller_file_hash = file_sha256(controller_path)
    _assert_frozen_controller(
        controller,
        controller_file_sha256=controller_file_hash,
    )
    destination = Path(output_dir).resolve()
    destination.mkdir(parents=True, exist_ok=False)
    attempts: list[dict[str, Any]] = []
    selected: dict[str, Any] | None = None
    report_path = destination / "schedule_manifest.json"
    for stage in G11P49_STAGE_SCHEDULE:
        attempt_path = solve_g11p49_stage(
            frozen_controller_path=controller_path,
            output_dir=destination / stage.stage_id,
            stage=stage,
        )
        attempt = json.loads(attempt_path.read_text(encoding="utf-8"))
        row = {
            "stage_id": stage.stage_id,
            "manifest": str(attempt_path.relative_to(destination)),
            "manifest_file_sha256": file_sha256(attempt_path),
            "manifest_content_sha256": attempt["manifest_content_sha256"],
            "status": attempt["status"],
            "rho": attempt.get("numeric_incumbent", {}).get("rho"),
        }
        attempts.append(row)
        if attempt["status"] == "ready":
            selected = row
        schedule_report = {
            "schema_version": 1,
            "kind": "g11p49_piecewise_roof_finite_schedule",
            "synthesis_id": G11P49_SYNTHESIS_ID,
            "proof_route_id": G11P49_PROOF_ROUTE_ID,
            "controller_content_sha256": controller.sha256,
            "geometry_catalog_sha256": p49_geometry_catalog_sha256(),
            "frozen_stage_ids": [
                item.stage_id for item in G11P49_STAGE_SCHEDULE
            ],
            "status": "ready" if selected is not None else "running",
            "selected": selected,
            "attempts": attempts,
            "automatic_escalation_beyond_schedule": False,
        }
        _save_manifest(report_path, schedule_report)
        if selected is not None:
            break
    if selected is None:
        schedule_report["status"] = "not_ready"
        _save_manifest(report_path, schedule_report)
    return report_path
