"""G10A active-set Chebyshev q-SBC candidate generation.

G10A is an independent fallback beside G10 and G10R.  It never treats a
sampled constraint or a HiGHS status as proof.  A G9 barrier is consumed only
to rank a finite catalog of diagnostic point rows.  At fixed checkpoints the
unchanged V3 exact-rational validator is run on the current single global
barrier.  A failed or unresolved V3 leaf may add a hard Bernstein row block on
that leaf's exact rational box.  Publication is possible only through the same
V3 release gate as G10.

The incremental backend prefers public ``highspy`` row addition and its
retained simplex basis.  If importing or calling that API fails, the stage
switches permanently to deterministic sparse SciPy/HiGHS rebuilds.  This is a
candidate-generation implementation detail and has no role in the proof.
"""

from __future__ import annotations

from dataclasses import dataclass, replace
from fractions import Fraction
import gc
import json
from math import isfinite
from pathlib import Path
from time import monotonic
from typing import Any, Literal, Mapping, Sequence

import numpy as np
from scipy.optimize import linprog
from scipy.sparse import csr_matrix, hstack, vstack

from cbf_rover.adapters.training.sac_reference import file_sha256
from cbf_rover.domain.planar import CANONICAL_SYSTEM

from .algebra import add, constant, exact_expected_next_polynomial, negate
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
    G10_RHO_BOUNDS,
    G10_VALIDATOR_ID,
    _assert_frozen_c2,
    _maximum_bound_violation,
    _numeric_to_sparse,
    _save_manifest,
    _v3_release_gate,
    _write_json,
)
from .polynomial import (
    FrozenController,
    NumericPolynomial,
    Power2D,
    SparsePolynomial2D,
    content_sha256,
    controller_max_degree,
    controller_pieces,
    load_frozen_controller,
)
from .rational_certificate_validation import (
    INDEPENDENT_RATIONAL_VALIDATOR_ID,
    INITIAL_DISK,
    RationalBox,
    UNSAFE_PHYSICAL,
    _fraction_text,
    _safe_drift_cells,
    canonical_rational_model_payload,
    classify_full_box,
    classify_goal_exterior_box,
    classify_included_disk_box,
    normalize_physical_box,
)


G10A_SYNTHESIS_ID = "G10A_V3_ACTIVE_BERNSTEIN_CUT_LP_V1"
G10A_PROOF_ROUTE_ID = "G10A_V3_EXACT_BOX_FEEDBACK_V1"
G10A_G9_ROLE = "diagnostic_seed_and_row_ranking_only"
G10A_V3_CHECKPOINT_PERIOD = 4
G10A_G9_CATALOG_DEPTH = 3
G10A_G9_ROWS_PER_OUTER = 12
G10A_FEEDBACK_NUMERIC_MARGIN = 1.0e-8
G10A_FROZEN_QSBC_FAMILY_COUNT = 42
G10A_FROZEN_CONTROLLER_FAMILY_COUNT = 15
G10A_G9_SEED_BUNDLE_KIND = "g10a_g9_seed_bundle"
G10A_G9_DECISION_ID = "DX2_SAMPLED_QSBC_EXPRESSIVITY_LP_V1"
G10A_LEGACY_G9_PROBE_KIND = "w14_chebyshev_dense_positive_margin_probe"
G10A_CONTROLLER_CONSTRAINT_SUFFIXES = (
    "action_disk",
    "successor_x_lower",
    "successor_x_upper",
    "successor_y_lower",
    "successor_y_upper",
)


@dataclass(frozen=True, slots=True)
class G10AStageSpec:
    """One immutable member of the finite G10A schedule."""

    stage_id: str
    barrier_degree: int
    max_outer_iterations: int
    max_v3_calls: int
    time_limit_s: float

    def __post_init__(self) -> None:
        if self.barrier_degree not in (16, 18, 20):
            raise ValueError("G10A barrier degree must be 16, 18, or 20")
        if self.max_outer_iterations <= 0:
            raise ValueError("G10A outer-iteration limit must be positive")
        if self.max_v3_calls <= 0:
            raise ValueError("G10A V3-call limit must be positive")
        if self.max_outer_iterations != (
            G10A_V3_CHECKPOINT_PERIOD * self.max_v3_calls
        ):
            raise ValueError("G10A requires exactly four outer iterations per V3 call")
        if not isfinite(self.time_limit_s) or self.time_limit_s <= 0.0:
            raise ValueError("G10A time limit must be positive and finite")


G10A_STAGE_SCHEDULE = (
    G10AStageSpec("W16_ACTIVE", 16, 12, 3, 3600.0),
    G10AStageSpec("W18_ACTIVE", 18, 16, 4, 7200.0),
    G10AStageSpec("W20_ACTIVE", 20, 20, 5, 10800.0),
)


DomainKind = Literal["full", "initial_disk", "goal_exterior"]
RowKind = Literal["diagnostic_point", "v3_exact_box"]


@dataclass(frozen=True, slots=True)
class _G9Seed:
    path: Path
    file_sha256: str
    report_content_sha256: str
    kind: str
    barrier: SparsePolynomial2D
    q: float | None
    epsbar: float | None
    chebyshev_by_power: Mapping[Power2D, float]
    source_mode: str
    source_probe_file_sha256: str | None = None
    legacy_declared_report_content_sha256: str | None = None
    current_recomputed_probe_content_sha256: str | None = None


@dataclass(frozen=True, slots=True)
class _ConstraintFamily:
    name: str
    root_box_exact: RationalBox
    domain_kind: DomainKind
    basis: tuple[NumericPolynomial, ...]
    constant_value: float
    piece_id: str | None = None


@dataclass(frozen=True, slots=True)
class _ActiveRow:
    family_name: str
    row_kind: RowKind
    box_exact: RationalBox
    source: str
    domain_class: str
    ranking_score: float | None = None
    v3_call_index: int | None = None
    v3_tree_file: str | None = None
    v3_tree_file_sha256: str | None = None
    v3_leaf_depth: int | None = None
    v3_leaf_disposition: str | None = None

    @property
    def key(self) -> tuple[str, RowKind, RationalBox]:
        return self.family_name, self.row_kind, self.box_exact

    def payload(self) -> dict[str, Any]:
        return {
            "family_name": self.family_name,
            "row_kind": self.row_kind,
            "box_exact": [_fraction_text(value) for value in self.box_exact],
            "box_float64_hex": [float(value).hex() for value in self.box_exact],
            "source": self.source,
            "domain_class": self.domain_class,
            "ranking_score": self.ranking_score,
            "v3_call_index": self.v3_call_index,
            "v3_tree_file": self.v3_tree_file,
            "v3_tree_file_sha256": self.v3_tree_file_sha256,
            "v3_leaf_depth": self.v3_leaf_depth,
            "v3_leaf_disposition": self.v3_leaf_disposition,
            "conservative_boundary_hull": (
                self.row_kind == "v3_exact_box" and self.domain_class == "boundary"
            ),
            "formal_claim_allowed": False,
        }


@dataclass(frozen=True, slots=True)
class _NumericSolveResult:
    values: np.ndarray | None
    backend: str
    backend_status: str
    success: bool
    message: str
    iterations: int
    solve_time_s: float
    basis_valid_before: bool | None
    basis_valid_after: bool | None
    fallback_reason: str | None
    aggregate_budget_exceeded: bool = False


def _canonical_content_hash_without_report_hash(
    payload: Mapping[str, Any],
) -> str:
    material = dict(payload)
    material.pop("report_content_sha256", None)
    return content_sha256(material)


def _bind_legacy_probe_bundle(
    *,
    bundle_path: Path,
    bundle: Mapping[str, Any],
) -> tuple[Mapping[str, Any], Mapping[str, Any]]:
    if bundle.get("schema_version") != 1:
        raise ValueError("G10A G9 bundle schema mismatch")
    if bundle.get("kind") != G10A_G9_SEED_BUNDLE_KIND:
        raise ValueError("G10A G9 bundle kind mismatch")
    if bundle.get("decision_id") != G10A_G9_DECISION_ID:
        raise ValueError("G10A G9 bundle decision identity mismatch")
    source_binding = bundle.get("source_probe")
    if not isinstance(source_binding, Mapping):
        raise ValueError("G10A G9 bundle has no source-probe binding")
    source_name = source_binding.get("file")
    if not isinstance(source_name, str) or Path(source_name).name != source_name:
        raise ValueError("G10A G9 source probe must be a sibling file")
    source_path = bundle_path.parent / source_name
    expected_file_hash = source_binding.get("file_sha256")
    if (
        not isinstance(expected_file_hash, str)
        or len(expected_file_hash) != 64
        or not source_path.is_file()
        or file_sha256(source_path) != expected_file_hash
    ):
        raise ValueError("G10A G9 source probe file hash mismatch")
    probe = json.loads(source_path.read_text(encoding="utf-8"))
    if not isinstance(probe, Mapping):
        raise ValueError("G10A legacy G9 probe must be a JSON object")
    if probe.get("schema_version") != 1:
        raise ValueError("G10A legacy G9 probe schema mismatch")
    if probe.get("kind") != G10A_LEGACY_G9_PROBE_KIND:
        raise ValueError("G10A legacy G9 probe kind mismatch")
    if probe.get("degree") != 14:
        raise ValueError("G10A legacy G9 probe degree mismatch")
    if probe.get("basis") != "total_degree_tensor_chebyshev":
        raise ValueError("G10A legacy G9 probe basis mismatch")
    if probe.get("formal_claim_allowed") is not False:
        raise ValueError("G10A legacy G9 probe is not diagnostic-only")
    legacy_declared = str(probe.get("report_content_sha256", ""))
    current_recomputed = _canonical_content_hash_without_report_hash(probe)
    if source_binding.get("legacy_declared_report_content_sha256") != (
        legacy_declared
    ):
        raise ValueError("G10A bundle legacy declared hash binding mismatch")
    if source_binding.get("current_recomputed_content_sha256") != (
        current_recomputed
    ):
        raise ValueError("G10A bundle current recomputed hash binding mismatch")
    if legacy_declared == current_recomputed:
        raise ValueError("G10A legacy mismatch bundle is unnecessary or malformed")
    if source_binding.get("legacy_declared_matches_current") is not False:
        raise ValueError("G10A bundle must explicitly record the legacy hash mismatch")

    for name in ("barrier", "q", "epsbar", "degree", "basis"):
        if bundle.get(name) != probe.get(name):
            raise ValueError(f"G10A bundle/probe {name} mismatch")
    if bundle.get("chebyshev_coefficients") != probe.get(
        "chebyshev_coefficients"
    ):
        raise ValueError("G10A bundle/probe Chebyshev coefficients mismatch")
    return probe, source_binding


def _load_g9_seed(path: str | Path) -> _G9Seed:
    source = Path(path).resolve()
    payload = json.loads(source.read_text(encoding="utf-8"))
    if not isinstance(payload, Mapping):
        raise ValueError("G10A G9 seed must be a JSON object")
    if payload.get("formal_claim_allowed") is not False:
        raise ValueError("G10A seed must explicitly be diagnostic-only")
    kind = str(payload.get("kind", ""))
    decision_id = str(payload.get("decision_id", ""))
    if "probe" not in kind and "sampled" not in kind and "DX2" not in decision_id:
        raise ValueError("G10A seed is not a recognized G9/DX2 diagnostic")

    declared_report_hash = str(payload.get("report_content_sha256", ""))
    if (
        len(declared_report_hash) != 64
        or _canonical_content_hash_without_report_hash(payload)
        != declared_report_hash
    ):
        raise ValueError("G10A G9 report content hash mismatch")
    report_hash = declared_report_hash

    source_mode = "self_contained_current_canonical_probe"
    source_probe_hash: str | None = None
    legacy_declared_hash: str | None = None
    current_recomputed_hash: str | None = None
    if kind == G10A_G9_SEED_BUNDLE_KIND:
        _, source_binding = _bind_legacy_probe_bundle(
            bundle_path=source,
            bundle=payload,
        )
        source_mode = "current_canonical_bundle_binding_legacy_probe"
        source_probe_hash = str(source_binding["file_sha256"])
        legacy_declared_hash = str(
            source_binding["legacy_declared_report_content_sha256"]
        )
        current_recomputed_hash = str(
            source_binding["current_recomputed_content_sha256"]
        )

    barrier_payload = payload.get("barrier", payload.get("diagnostic_barrier"))
    if not isinstance(barrier_payload, Mapping):
        raise ValueError("G10A G9 seed does not contain a diagnostic barrier")
    barrier = SparsePolynomial2D.from_payload(barrier_payload)

    chebyshev_by_power: dict[Power2D, float] = {}
    raw_chebyshev = payload.get("chebyshev_coefficients")
    if isinstance(raw_chebyshev, Sequence) and not isinstance(
        raw_chebyshev, (str, bytes)
    ):
        seed_indices, _ = total_degree_chebyshev_basis(int(payload["degree"]))
        values = tuple(float(value) for value in raw_chebyshev)
        if len(values) != len(seed_indices) or not np.isfinite(values).all():
            raise ValueError("G10A G9 Chebyshev seed vector is invalid")
        chebyshev_by_power = dict(zip(seed_indices, values))

    q_raw = payload.get("q")
    epsbar_raw = payload.get("epsbar")
    q_value = None if q_raw is None else float(q_raw)
    epsbar_value = None if epsbar_raw is None else float(epsbar_raw)
    if q_value is not None and not isfinite(q_value):
        raise ValueError("G10A G9 seed q must be finite")
    if epsbar_value is not None and not isfinite(epsbar_value):
        raise ValueError("G10A G9 seed epsbar must be finite")
    return _G9Seed(
        path=source,
        file_sha256=file_sha256(source),
        report_content_sha256=report_hash,
        kind=kind,
        barrier=barrier,
        q=q_value,
        epsbar=epsbar_value,
        chebyshev_by_power=chebyshev_by_power,
        source_mode=source_mode,
        source_probe_file_sha256=source_probe_hash,
        legacy_declared_report_content_sha256=legacy_declared_hash,
        current_recomputed_probe_content_sha256=current_recomputed_hash,
    )


def _constraint_families(
    *,
    controller: FrozenController,
    barrier_basis: Sequence[NumericPolynomial],
) -> tuple[_ConstraintFamily, ...]:
    basis = tuple(barrier_basis)
    families: list[_ConstraintFamily] = [
        _ConstraintFamily(
            name="W_nonnegative_on_X",
            root_box_exact=(Fraction(-1), Fraction(1), Fraction(-1), Fraction(1)),
            domain_kind="full",
            basis=basis,
            constant_value=0.0,
        )
    ]
    initial_physical_box = (
        INITIAL_DISK.center_x - INITIAL_DISK.radius,
        INITIAL_DISK.center_x + INITIAL_DISK.radius,
        INITIAL_DISK.center_y - INITIAL_DISK.radius,
        INITIAL_DISK.center_y + INITIAL_DISK.radius,
    )
    families.append(
        _ConstraintFamily(
            name="q_minus_W_on_X0",
            root_box_exact=normalize_physical_box(initial_physical_box),
            domain_kind="initial_disk",
            basis=tuple(negate(polynomial) for polynomial in basis),
            constant_value=G10_Q,
        )
    )
    for unsafe_name, unsafe_box in UNSAFE_PHYSICAL:
        families.append(
            _ConstraintFamily(
                name=f"W_minus_1_on_{unsafe_name}",
                root_box_exact=normalize_physical_box(unsafe_box),
                domain_kind="full",
                basis=basis,
                constant_value=-1.0,
            )
        )

    pieces_by_id = {piece.piece_id: piece for piece in controller_pieces(controller)}
    drift_basis_by_piece: dict[str, tuple[NumericPolynomial, ...]] = {}
    for cell_name, cell_box, piece in _safe_drift_cells(controller):
        if piece.piece_id not in pieces_by_id:
            raise ValueError(
                "G10A V3 drift cell references an unknown controller piece"
            )
        if piece.piece_id not in drift_basis_by_piece:
            drift_basis_by_piece[piece.piece_id] = tuple(
                add(
                    polynomial,
                    negate(
                        exact_expected_next_polynomial(
                            _numeric_to_sparse(polynomial),
                            controller,
                            CANONICAL_SYSTEM,
                            piece=piece,
                        )
                    ),
                )
                for polynomial in basis
            )
        families.append(
            _ConstraintFamily(
                name=f"drift_{piece.piece_id}_{cell_name}",
                root_box_exact=cell_box,
                domain_kind="goal_exterior",
                basis=drift_basis_by_piece[piece.piece_id],
                constant_value=-G10_EPSBAR,
                piece_id=piece.piece_id,
            )
        )

    if len(families) != G10A_FROZEN_QSBC_FAMILY_COUNT:
        raise ValueError("G10A frozen V3 q-SBC family count mismatch")
    names = tuple(family.name for family in families)
    if len(set(names)) != len(names):
        raise ValueError("G10A V3 q-SBC family names are not unique")
    return tuple(families)


def _classify_box(family: _ConstraintFamily, box: RationalBox) -> str:
    if family.domain_kind == "full":
        return classify_full_box(box)
    if family.domain_kind == "initial_disk":
        return classify_included_disk_box(box)
    if family.domain_kind == "goal_exterior":
        return classify_goal_exterior_box(box)
    raise ValueError(f"unsupported G10A domain kind: {family.domain_kind}")


def _split_exact_box(box: RationalBox) -> tuple[RationalBox, RationalBox]:
    x_low, x_high, y_low, y_high = box
    width_x = x_high - x_low
    width_y = y_high - y_low
    if width_x >= width_y:
        midpoint = (x_low + x_high) / 2
        return (
            (x_low, midpoint, y_low, y_high),
            (midpoint, x_high, y_low, y_high),
        )
    midpoint = (y_low + y_high) / 2
    return (
        (x_low, x_high, y_low, midpoint),
        (x_low, x_high, midpoint, y_high),
    )


def _dyadic_exact_boxes(root_box: RationalBox, depth: int) -> tuple[RationalBox, ...]:
    if depth < 0:
        raise ValueError("G10A dyadic depth must be nonnegative")
    boxes = (root_box,)
    for _ in range(depth):
        boxes = tuple(child for box in boxes for child in _split_exact_box(box))
    return boxes


def _evaluate_numeric(
    polynomial: Mapping[Power2D, float],
    point: tuple[float, float],
) -> float:
    x_value, y_value = point
    return float(
        sum(
            coefficient * x_value**x_power * y_value**y_power
            for (x_power, y_power), coefficient in polynomial.items()
        )
    )


def _seed_residuals(
    *,
    seed: _G9Seed,
    controller: FrozenController,
    families: Sequence[_ConstraintFamily],
) -> dict[str, NumericPolynomial]:
    barrier = seed.barrier.as_numeric_dict()
    expected_by_piece = {
        piece.piece_id: exact_expected_next_polynomial(
            seed.barrier,
            controller,
            CANONICAL_SYSTEM,
            piece=piece,
        )
        for piece in controller_pieces(controller)
    }
    result: dict[str, NumericPolynomial] = {}
    for family in families:
        if family.name == "W_nonnegative_on_X":
            residual = barrier
        elif family.name == "q_minus_W_on_X0":
            residual = add(constant(G10_Q), negate(barrier))
        elif family.name.startswith("W_minus_1_on_"):
            residual = add(barrier, constant(-1.0))
        elif family.name.startswith("drift_") and family.piece_id is not None:
            residual = add(
                barrier,
                negate(expected_by_piece[family.piece_id]),
                constant(-G10_EPSBAR),
            )
        else:
            raise ValueError(f"unsupported G10A family: {family.name}")
        result[family.name] = residual
    return result


def _g9_ranked_rows(
    *,
    seed: _G9Seed,
    controller: FrozenController,
    families: Sequence[_ConstraintFamily],
) -> tuple[tuple[_ActiveRow, ...], tuple[_ActiveRow, ...]]:
    """Return one anchor per family and a finite diagnostic-only queue.

    Exact rational boxes define deterministic midpoints.  G9 evaluates only the
    midpoint residual used for ordering; these point rows are never proof.
    """

    residuals = _seed_residuals(
        seed=seed,
        controller=controller,
        families=families,
    )
    anchors: list[_ActiveRow] = []
    dormant: list[_ActiveRow] = []
    for family in families:
        candidates: list[_ActiveRow] = []
        for box in _dyadic_exact_boxes(
            family.root_box_exact,
            G10A_G9_CATALOG_DEPTH,
        ):
            domain_class = _classify_box(family, box)
            if domain_class == "outside":
                continue
            midpoint_exact = ((box[0] + box[1]) / 2, (box[2] + box[3]) / 2)
            midpoint_box = (
                midpoint_exact[0],
                midpoint_exact[0],
                midpoint_exact[1],
                midpoint_exact[1],
            )
            if _classify_box(family, midpoint_box) == "outside":
                continue
            score = _evaluate_numeric(
                residuals[family.name],
                (float(midpoint_exact[0]), float(midpoint_exact[1])),
            )
            if not isfinite(score):
                raise ValueError("G10A G9 ranking score is non-finite")
            candidates.append(
                _ActiveRow(
                    family_name=family.name,
                    row_kind="diagnostic_point",
                    box_exact=box,
                    source=G10A_G9_ROLE,
                    domain_class=domain_class,
                    ranking_score=score,
                )
            )
        if not candidates:
            raise ValueError(f"G10A G9 catalog has no active row for {family.name}")
        candidates.sort(
            key=lambda row: (
                float(row.ranking_score),
                tuple(row.box_exact),
            )
        )
        anchors.append(candidates[0])
        dormant.extend(candidates[1:])
    dormant.sort(
        key=lambda row: (
            float(row.ranking_score),
            row.family_name,
            tuple(row.box_exact),
        )
    )
    return tuple(anchors), tuple(dormant)


def _point_family_row(
    family: _ConstraintFamily,
    row: _ActiveRow,
    *,
    variable_count: int,
    rho_index: int,
) -> tuple[csr_matrix, np.ndarray]:
    midpoint = (
        float((row.box_exact[0] + row.box_exact[1]) / 2),
        float((row.box_exact[2] + row.box_exact[3]) / 2),
    )
    coefficients = np.zeros(variable_count, dtype=np.float64)
    coefficients[:rho_index] = -np.asarray(
        [_evaluate_numeric(polynomial, midpoint) for polynomial in family.basis],
        dtype=np.float64,
    )
    coefficients[rho_index] = 1.0
    return (
        csr_matrix(coefficients.reshape(1, -1)),
        np.asarray([family.constant_value], dtype=np.float64),
    )


def _exact_box_family_rows(
    family: _ConstraintFamily,
    row: _ActiveRow,
    *,
    variable_count: int,
    rho_index: int,
) -> tuple[csr_matrix, np.ndarray]:
    box = tuple(float(value) for value in row.box_exact)
    local = _polynomial_family_bernstein_matrix(family.basis, box)
    core = csr_matrix(-local)
    rho_column = csr_matrix((local.shape[0], 1), dtype=np.float64)
    matrix = hstack((core, rho_column), format="csr")
    if matrix.shape[1] != variable_count or rho_index != variable_count - 1:
        raise ValueError("G10A hard-cut variable layout mismatch")
    right_hand_side = np.full(
        local.shape[0],
        family.constant_value - G10A_FEEDBACK_NUMERIC_MARGIN,
        dtype=np.float64,
    )
    return matrix, right_hand_side


class _ActiveRowStore:
    """Accumulate auditable sparse row blocks without a global dense matrix."""

    def __init__(
        self,
        *,
        families: Sequence[_ConstraintFamily],
        variable_count: int,
        rho_index: int,
    ) -> None:
        self.families = {family.name: family for family in families}
        self.variable_count = int(variable_count)
        self.rho_index = int(rho_index)
        self.keys: set[tuple[str, RowKind, RationalBox]] = set()
        self.rows: list[_ActiveRow] = []
        self.matrix = csr_matrix((0, variable_count), dtype=np.float64)
        self.right_hand_side = np.empty(0, dtype=np.float64)

    def append(
        self,
        rows: Sequence[_ActiveRow],
    ) -> tuple[csr_matrix, np.ndarray, tuple[_ActiveRow, ...]]:
        blocks: list[csr_matrix] = []
        rhs_parts: list[np.ndarray] = []
        accepted: list[_ActiveRow] = []
        for row in rows:
            if row.key in self.keys:
                continue
            family = self.families.get(row.family_name)
            if family is None:
                raise ValueError("G10A row references an unknown V3 constraint")
            if not all(
                outer_low <= inner_low <= inner_high <= outer_high
                for inner_low, inner_high, outer_low, outer_high in (
                    (
                        row.box_exact[0],
                        row.box_exact[1],
                        family.root_box_exact[0],
                        family.root_box_exact[1],
                    ),
                    (
                        row.box_exact[2],
                        row.box_exact[3],
                        family.root_box_exact[2],
                        family.root_box_exact[3],
                    ),
                )
            ):
                raise ValueError("G10A active row lies outside its V3 root box")
            if row.row_kind == "diagnostic_point":
                block, rhs = _point_family_row(
                    family,
                    row,
                    variable_count=self.variable_count,
                    rho_index=self.rho_index,
                )
            elif row.row_kind == "v3_exact_box":
                block, rhs = _exact_box_family_rows(
                    family,
                    row,
                    variable_count=self.variable_count,
                    rho_index=self.rho_index,
                )
            else:
                raise ValueError("unsupported G10A row kind")
            blocks.append(block)
            rhs_parts.append(rhs)
            accepted.append(row)
            self.keys.add(row.key)
            self.rows.append(row)
        if not blocks:
            return (
                csr_matrix((0, self.variable_count), dtype=np.float64),
                np.empty(0, dtype=np.float64),
                (),
            )
        new_matrix = vstack(blocks, format="csr")
        new_rhs = np.concatenate(rhs_parts)
        self.matrix = vstack((self.matrix, new_matrix), format="csr")
        self.right_hand_side = np.concatenate((self.right_hand_side, new_rhs))
        return new_matrix, new_rhs, tuple(accepted)

    def payload(self) -> dict[str, Any]:
        return {
            "schema_version": 1,
            "kind": "g10a_active_row_ledger",
            "synthesis_id": G10A_SYNTHESIS_ID,
            "row_block_count": len(self.rows),
            "linear_inequality_count": int(self.matrix.shape[0]),
            "matrix_nonzero_count": int(self.matrix.nnz),
            "global_dense_matrix_constructed": False,
            "rows": [row.payload() for row in self.rows],
            "formal_claim_allowed": False,
        }


def _highs_status_ok(status: Any) -> bool:
    text = str(status)
    if text.endswith("kOk"):
        return True
    try:
        return int(status) == 0
    except (TypeError, ValueError):
        return False


class _IncrementalOrRebuildSolver:
    """Prefer highspy incremental rows, then fail permanently to CSR rebuilds."""

    def __init__(
        self,
        *,
        matrix: csr_matrix,
        right_hand_side: np.ndarray,
        objective: np.ndarray,
        bounds: Sequence[tuple[float, float]],
        seed_values: np.ndarray | None,
    ) -> None:
        self.matrix = matrix.copy().tocsr()
        self.right_hand_side = np.asarray(right_hand_side, dtype=np.float64).copy()
        self.objective = np.asarray(objective, dtype=np.float64).copy()
        self.bounds = tuple((float(low), float(high)) for low, high in bounds)
        self.seed_values = None if seed_values is None else seed_values.copy()
        self._seed_attempted = False
        self._highs: Any | None = None
        self._highspy: Any | None = None
        self.fallback_reason: str | None = None
        self.seed_hint_status: str | None = None
        self.backend_history: list[dict[str, Any]] = []
        try:
            self._initialize_highspy()
        except Exception as error:  # pragma: no cover - server API dependent
            self._disable_highspy(error)

    @property
    def backend(self) -> str:
        return (
            "highspy_incremental_rows_basis"
            if self._highs is not None
            else "scipy_highs_deterministic_sparse_rebuild"
        )

    def _initialize_highspy(self) -> None:
        import highspy  # type: ignore[import-not-found]

        matrix = self.matrix
        lp = highspy.HighsLp()
        lp.num_col_ = int(matrix.shape[1])
        lp.num_row_ = int(matrix.shape[0])
        lp.col_cost_ = self.objective
        lp.col_lower_ = np.asarray([bound[0] for bound in self.bounds])
        lp.col_upper_ = np.asarray([bound[1] for bound in self.bounds])
        lp.row_lower_ = np.full(matrix.shape[0], -highspy.kHighsInf)
        lp.row_upper_ = self.right_hand_side
        lp.a_matrix_.format_ = highspy.MatrixFormat.kRowwise
        lp.a_matrix_.start_ = matrix.indptr.astype(np.int32, copy=False)
        lp.a_matrix_.index_ = matrix.indices.astype(np.int32, copy=False)
        lp.a_matrix_.value_ = matrix.data

        highs = highspy.Highs()
        for name, value in (
            ("output_flag", False),
            ("solver", "simplex"),
            ("presolve", "on"),
            ("primal_feasibility_tolerance", 1.0e-9),
            ("dual_feasibility_tolerance", 1.0e-9),
            ("random_seed", 0),
        ):
            status = highs.setOptionValue(name, value)
            if not _highs_status_ok(status):
                raise RuntimeError(f"highspy rejected option {name}: {status}")
        status = highs.passModel(lp)
        if not _highs_status_ok(status):
            raise RuntimeError(f"highspy passModel failed: {status}")
        self._highspy = highspy
        self._highs = highs

    def _disable_highspy(self, error: Exception) -> None:
        if self.fallback_reason is None:
            self.fallback_reason = f"{type(error).__name__}: {error}"
        self._highs = None
        self._highspy = None

    def append_rows(self, matrix: csr_matrix, right_hand_side: np.ndarray) -> None:
        block = matrix.tocsr()
        rhs = np.asarray(right_hand_side, dtype=np.float64)
        if block.shape[0] != rhs.size or block.shape[1] != self.matrix.shape[1]:
            raise ValueError("G10A incremental row block shape mismatch")
        basis_before: bool | None = None
        basis_after: bool | None = None
        if block.shape[0] and self._highs is not None:
            try:
                basis_before = bool(self._highs.getBasis().valid)
                status = self._highs.addRows(
                    int(block.shape[0]),
                    np.full(block.shape[0], -self._highspy.kHighsInf),
                    rhs,
                    int(block.nnz),
                    # The public addRows API takes one start per new row;
                    # num_new_nz supplies the terminal CSR offset.
                    block.indptr[:-1].astype(np.int32, copy=False),
                    block.indices.astype(np.int32, copy=False),
                    block.data,
                )
                if not _highs_status_ok(status):
                    raise RuntimeError(f"highspy addRows failed: {status}")
                basis_after = bool(self._highs.getBasis().valid)
                # A warm simplex basis is incompatible with presolve reductions.
                status = self._highs.setOptionValue("presolve", "off")
                if not _highs_status_ok(status):
                    raise RuntimeError(f"highspy presolve update failed: {status}")
            except Exception as error:  # pragma: no cover - server API dependent
                self._disable_highspy(error)
        self.matrix = vstack((self.matrix, block), format="csr")
        self.right_hand_side = np.concatenate((self.right_hand_side, rhs))
        self.backend_history.append(
            {
                "event": "append_rows",
                "added_row_count": int(block.shape[0]),
                "added_nonzero_count": int(block.nnz),
                "basis_valid_before": basis_before,
                "basis_valid_after": basis_after,
                "backend_after": self.backend,
                "fallback_reason": self.fallback_reason,
            }
        )

    def solve(self, *, time_limit_s: float) -> _NumericSolveResult:
        if not isfinite(time_limit_s) or time_limit_s <= 0.0:
            raise ValueError("G10A solve time limit must be positive and finite")
        aggregate_started = monotonic()
        aggregate_deadline = aggregate_started + float(time_limit_s)
        failed_highspy_elapsed_s = 0.0
        if self._highs is not None:
            now = monotonic()
            highspy_time_limit_s = aggregate_deadline - now
            if highspy_time_limit_s <= 0.0:
                return self._aggregate_budget_exhausted(
                    elapsed_s=max(0.0, now - aggregate_started),
                    requested_time_limit_s=time_limit_s,
                    phase="before_highspy",
                )
            try:
                result = self._solve_highspy(
                    time_limit_s=highspy_time_limit_s
                )
            except Exception as error:  # pragma: no cover - server API dependent
                self._disable_highspy(error)
                now = monotonic()
                failed_highspy_elapsed_s = max(
                    0.0, now - aggregate_started
                )
                self.backend_history.append(
                    {
                        "event": "backend_exception",
                        "backend": "highspy_incremental_rows_basis",
                        "elapsed_before_fallback_s": failed_highspy_elapsed_s,
                        "fallback_reason": self.fallback_reason,
                    }
                )
            else:
                finished = monotonic()
                return self._finalize_aggregate_solve(
                    result=result,
                    aggregate_elapsed_s=max(
                        0.0, finished - aggregate_started
                    ),
                    requested_time_limit_s=time_limit_s,
                    delegated_time_limit_s=highspy_time_limit_s,
                    failed_highspy_elapsed_s=0.0,
                    budget_exceeded=finished > aggregate_deadline,
                )

        now = monotonic()
        rebuild_time_limit_s = aggregate_deadline - now
        if rebuild_time_limit_s <= 0.0:
            return self._aggregate_budget_exhausted(
                elapsed_s=max(0.0, now - aggregate_started),
                requested_time_limit_s=time_limit_s,
                phase="before_rebuild",
                failed_highspy_elapsed_s=failed_highspy_elapsed_s,
            )
        result = self._solve_rebuild(time_limit_s=rebuild_time_limit_s)
        finished = monotonic()
        return self._finalize_aggregate_solve(
            result=result,
            aggregate_elapsed_s=max(0.0, finished - aggregate_started),
            requested_time_limit_s=time_limit_s,
            delegated_time_limit_s=rebuild_time_limit_s,
            failed_highspy_elapsed_s=failed_highspy_elapsed_s,
            budget_exceeded=finished > aggregate_deadline,
        )

    def _aggregate_budget_exhausted(
        self,
        *,
        elapsed_s: float,
        requested_time_limit_s: float,
        phase: str,
        failed_highspy_elapsed_s: float = 0.0,
    ) -> _NumericSolveResult:
        result = _NumericSolveResult(
            values=None,
            backend=self.backend,
            backend_status="aggregate_lp_time_budget_exhausted",
            success=False,
            message=(
                "G10A aggregate LP wall-time budget exhausted "
                f"{phase}; no candidate accepted"
            ),
            iterations=0,
            solve_time_s=float(elapsed_s),
            basis_valid_before=None,
            basis_valid_after=None,
            fallback_reason=self.fallback_reason,
            aggregate_budget_exceeded=True,
        )
        self.backend_history.append(
            {
                "event": "solve",
                "backend": result.backend,
                "backend_status": result.backend_status,
                "message": result.message,
                "iterations": 0,
                "solve_time_s": result.solve_time_s,
                "requested_aggregate_time_limit_s": float(
                    requested_time_limit_s
                ),
                "failed_highspy_elapsed_s": float(
                    failed_highspy_elapsed_s
                ),
                "fallback_reason": self.fallback_reason,
            }
        )
        return result

    def _finalize_aggregate_solve(
        self,
        *,
        result: _NumericSolveResult,
        aggregate_elapsed_s: float,
        requested_time_limit_s: float,
        delegated_time_limit_s: float,
        failed_highspy_elapsed_s: float,
        budget_exceeded: bool,
    ) -> _NumericSolveResult:
        backend_reported_solve_time_s = result.solve_time_s
        if budget_exceeded:
            result = replace(
                result,
                values=None,
                backend_status="aggregate_lp_time_budget_exhausted",
                success=False,
                message=(
                    "G10A aggregate LP wall-time budget exhausted after "
                    f"{result.backend}; backend candidate discarded; "
                    f"backend_status={result.backend_status}; "
                    f"backend_message={result.message}"
                ),
                solve_time_s=float(aggregate_elapsed_s),
                aggregate_budget_exceeded=True,
            )
        else:
            result = replace(
                result,
                solve_time_s=float(aggregate_elapsed_s),
            )
        if self.backend_history and self.backend_history[-1].get(
            "event"
        ) == "solve":
            history = self.backend_history[-1]
            history["backend_reported_solve_time_s"] = float(
                backend_reported_solve_time_s
            )
            history["solve_time_s"] = result.solve_time_s
            history["requested_aggregate_time_limit_s"] = float(
                requested_time_limit_s
            )
            history["delegated_backend_time_limit_s"] = float(
                delegated_time_limit_s
            )
            history["failed_highspy_elapsed_s"] = float(
                failed_highspy_elapsed_s
            )
            history["aggregate_budget_exceeded"] = bool(budget_exceeded)
            if budget_exceeded:
                history["backend_status_before_aggregate_gate"] = history.get(
                    "backend_status"
                )
                history["backend_message_before_aggregate_gate"] = history.get(
                    "message"
                )
                history["backend_status"] = result.backend_status
                history["message"] = result.message
        return result

    def _solve_highspy(self, *, time_limit_s: float) -> _NumericSolveResult:
        highs = self._highs
        # HiGHS' run clock is cumulative across repeated run() calls.  Its
        # time_limit is therefore an absolute clock threshold, not a fresh
        # duration for this solve.  Add this solve's delegated wall budget to
        # the current public run time so later incremental solves are neither
        # double-charged nor accidentally given a fresh stage budget.
        cumulative_run_time_before_s = float(highs.getRunTime())
        if (
            not isfinite(cumulative_run_time_before_s)
            or cumulative_run_time_before_s < 0.0
        ):
            raise RuntimeError("highspy returned an invalid cumulative run time")
        absolute_time_limit_s = (
            cumulative_run_time_before_s + float(time_limit_s)
        )
        if not isfinite(absolute_time_limit_s):
            raise RuntimeError("highspy absolute time limit is not finite")
        status = highs.setOptionValue("time_limit", absolute_time_limit_s)
        if not _highs_status_ok(status):
            raise RuntimeError(f"highspy rejected time limit: {status}")
        if not self._seed_attempted and self.seed_values is not None:
            self._seed_attempted = True
            if self.seed_values.shape != (self.matrix.shape[1],):
                raise ValueError("G10A highspy seed vector has the wrong shape")
            indices = np.arange(self.matrix.shape[1], dtype=np.int32)
            # Failure to accept a diagnostic seed is not an API failure.  It is
            # only a warm-start hint and is never required for correctness.
            try:
                seed_status = highs.setSolution(
                    len(indices),
                    indices,
                    self.seed_values,
                )
                self.seed_hint_status = str(seed_status)
            except Exception as error:  # pragma: no cover - optional hint only
                self.seed_hint_status = f"{type(error).__name__}: {error}"
        basis_before = bool(highs.getBasis().valid)
        started = monotonic()
        run_status = highs.run()
        elapsed = monotonic() - started
        model_status = highs.getModelStatus()
        solution = highs.getSolution()
        info = highs.getInfo()
        values: np.ndarray | None = None
        if bool(solution.value_valid):
            candidate = np.asarray(solution.col_value, dtype=np.float64)
            if candidate.shape == (self.matrix.shape[1],) and np.isfinite(
                candidate
            ).all():
                values = candidate
        message = highs.modelStatusToString(model_status)
        result = _NumericSolveResult(
            values=values,
            backend="highspy_incremental_rows_basis",
            backend_status=str(model_status),
            success=values is not None,
            message=f"{message}; run_status={run_status}",
            iterations=int(info.simplex_iteration_count),
            solve_time_s=float(elapsed),
            basis_valid_before=basis_before,
            basis_valid_after=bool(highs.getBasis().valid),
            fallback_reason=self.fallback_reason,
        )
        self.backend_history.append(
            {
                "event": "solve",
                "backend": result.backend,
                "backend_status": result.backend_status,
                "message": result.message,
                "iterations": result.iterations,
                "solve_time_s": result.solve_time_s,
                "basis_valid_before": result.basis_valid_before,
                "basis_valid_after": result.basis_valid_after,
                "G9_seed_hint_status": self.seed_hint_status,
                "highs_cumulative_run_time_before_s": (
                    cumulative_run_time_before_s
                ),
                "highs_absolute_time_limit_s": absolute_time_limit_s,
            }
        )
        return result

    def _solve_rebuild(self, *, time_limit_s: float) -> _NumericSolveResult:
        started = monotonic()
        result = linprog(
            self.objective,
            A_ub=self.matrix,
            b_ub=self.right_hand_side,
            bounds=self.bounds,
            method="highs",
            options={
                "presolve": True,
                "time_limit": float(time_limit_s),
                "primal_feasibility_tolerance": 1.0e-9,
                "dual_feasibility_tolerance": 1.0e-9,
            },
        )
        elapsed = monotonic() - started
        values: np.ndarray | None = None
        if getattr(result, "x", None) is not None:
            candidate = np.asarray(result.x, dtype=np.float64)
            if candidate.shape == (self.matrix.shape[1],) and np.isfinite(
                candidate
            ).all():
                values = candidate
        material = _NumericSolveResult(
            values=values,
            backend="scipy_highs_deterministic_sparse_rebuild",
            backend_status=str(int(result.status)),
            success=values is not None,
            message=str(result.message),
            iterations=int(result.nit),
            solve_time_s=float(elapsed),
            basis_valid_before=None,
            basis_valid_after=None,
            fallback_reason=self.fallback_reason,
        )
        self.backend_history.append(
            {
                "event": "solve",
                "backend": material.backend,
                "backend_status": material.backend_status,
                "message": material.message,
                "iterations": material.iterations,
                "solve_time_s": material.solve_time_s,
                "fallback_reason": material.fallback_reason,
            }
        )
        return material


def _parse_exact_fraction(value: Any) -> Fraction:
    text = str(value)
    result = Fraction(text)
    if _fraction_text(result) != text:
        raise ValueError("V3 exact rational text is not canonical")
    return result


def _parse_exact_box(value: Any) -> RationalBox:
    if not isinstance(value, Sequence) or isinstance(value, (str, bytes)):
        raise ValueError("V3 exact box must be a four-element sequence")
    if len(value) != 4:
        raise ValueError("V3 exact box must contain four endpoints")
    box = tuple(_parse_exact_fraction(item) for item in value)
    if not box[0] < box[1] or not box[2] < box[3]:
        raise ValueError("V3 exact box endpoints must be ordered")
    return box  # type: ignore[return-value]


def _v3_feedback_binding_failures(
    *,
    validation: Mapping[str, Any],
    validation_directory: Path,
    barrier: SparsePolynomial2D,
    controller: FrozenController,
    families: Sequence[_ConstraintFamily],
) -> tuple[str, ...]:
    """Cross-bind a V3 report even when its mathematical status is not valid."""

    failures: list[str] = []
    declared_hash = str(validation.get("report_content_sha256", ""))
    report_material = dict(validation)
    report_material.pop("report_content_sha256", None)
    if len(declared_hash) != 64 or content_sha256(report_material) != declared_hash:
        failures.append("report_content_sha256")
    checks = {
        "report_schema_version": validation.get("schema_version") == 2,
        "validator_id": validation.get("validator_id") == G10_VALIDATOR_ID,
        "validator_constant_id": (
            G10_VALIDATOR_ID == INDEPENDENT_RATIONAL_VALIDATOR_ID
        ),
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
        "q_exact": validation.get("q_exact") == str(Fraction.from_float(G10_Q)),
        "epsbar_float64_hex": (
            validation.get("epsbar_float64_hex") == G10_EPSBAR.hex()
        ),
        "epsbar_exact": (
            validation.get("epsbar_exact") == str(Fraction.from_float(G10_EPSBAR))
        ),
        "theorem_scalar_gate": validation.get("theorem_scalar_gate") is True,
        "geometry_contract_status": (
            validation.get("geometry_contract_status") == "validated"
        ),
        "controller_contract_status": (
            validation.get("controller_contract_status") == "validated"
        ),
    }
    failures.extend(name for name, passed in checks.items() if not passed)

    model_payload = validation.get("formal_model_contract")
    expected_model_payload = canonical_rational_model_payload()
    if (
        not isinstance(model_payload, Mapping)
        or dict(model_payload) != expected_model_payload
        or validation.get("formal_model_contract_sha256")
        != content_sha256(expected_model_payload)
    ):
        failures.append("formal_model_contract_sha256")

    constraints = validation.get("constraints")
    expected_names = {family.name for family in families}
    if not isinstance(constraints, Mapping) or set(constraints) != expected_names:
        failures.append("qsbc_constraint_family_set")
    controller_constraints = validation.get("controller_constraints")
    expected_controller_names = {
        f"{piece.piece_id}_{suffix}"
        for piece in controller_pieces(controller)
        for suffix in G10A_CONTROLLER_CONSTRAINT_SUFFIXES
    }
    if (
        controller.sha256 == G10_CONTROLLER_CONTENT_SHA256
        and len(expected_controller_names)
        != G10A_FROZEN_CONTROLLER_FAMILY_COUNT
    ):
        failures.append("frozen_controller_constraint_family_count")
    if (
        not isinstance(controller_constraints, Mapping)
        or set(controller_constraints) != expected_controller_names
    ):
        failures.append("controller_constraint_set")

    independence = validation.get("independence")
    required_independence = {
        "solver_status_read": False,
        "sampled_residual_used": False,
        "float_polynomial_algebra_used_for_status": False,
        "exact_binary64_artifact_coefficients": True,
        "exact_decimal_model_constants": True,
        "controller_S1_and_D1_replayed": True,
    }
    if not isinstance(independence, Mapping) or any(
        independence.get(name) is not value
        for name, value in required_independence.items()
    ):
        failures.append("validator_independence_contract")

    for group_name, group in (
        ("constraints", constraints),
        ("controller_constraints", controller_constraints),
    ):
        if not isinstance(group, Mapping):
            continue
        for name, summary in group.items():
            prefix = f"{group_name}.{name}"
            if not isinstance(summary, Mapping):
                failures.append(f"{prefix}.summary")
                continue
            tree_name = summary.get("tree_file")
            tree_hash = summary.get("tree_file_sha256")
            if not isinstance(tree_name, str) or Path(tree_name).name != tree_name:
                failures.append(f"{prefix}.tree_file")
                continue
            tree_path = validation_directory / tree_name
            if (
                not isinstance(tree_hash, str)
                or len(tree_hash) != 64
                or not tree_path.is_file()
                or file_sha256(tree_path) != tree_hash
            ):
                failures.append(f"{prefix}.tree_file_sha256")
                continue
            try:
                tree = json.loads(tree_path.read_text(encoding="utf-8"))
            except (OSError, json.JSONDecodeError):
                failures.append(f"{prefix}.tree_json")
                continue
            if not isinstance(tree, Mapping) or not isinstance(
                tree.get("leaves"), list
            ):
                failures.append(f"{prefix}.tree_schema")
                continue
            expected_summary = dict(tree)
            expected_summary.pop("leaves", None)
            expected_summary["tree_file"] = tree_name
            expected_summary["tree_file_sha256"] = tree_hash
            if dict(summary) != expected_summary:
                failures.append(f"{prefix}.tree_summary_binding")
    if isinstance(controller_constraints, Mapping) and any(
        not isinstance(item, Mapping) or item.get("status") != "validated"
        for item in controller_constraints.values()
    ):
        failures.append("frozen_controller_v3_gate")

    if isinstance(constraints, Mapping):
        statuses = [
            item.get("status") if isinstance(item, Mapping) else None
            for item in constraints.values()
        ]
        expected_global = (
            "validated"
            if all(status == "validated" for status in statuses)
            else "violated"
            if any(status == "violated" for status in statuses)
            else "unknown"
        )
        if validation.get("global_polynomial_status") != expected_global:
            failures.append("global_polynomial_status")
        if validation.get("status") != expected_global:
            failures.append("combined_formal_status")
        if validation.get("first_slice_ready") != (expected_global == "validated"):
            failures.append("first_slice_ready_consistency")
        family_by_name = {family.name: family for family in families}
        for name, summary in constraints.items():
            if not isinstance(summary, Mapping):
                continue
            expected_root = [
                _fraction_text(value)
                for value in family_by_name[name].root_box_exact
            ]
            if summary.get("root_box_exact") != expected_root:
                failures.append(f"constraints.{name}.root_box_exact")
    return tuple(dict.fromkeys(failures))


def _v3_feedback_rows(
    *,
    validation: Mapping[str, Any],
    validation_directory: Path,
    families: Sequence[_ConstraintFamily],
    v3_call_index: int,
) -> tuple[_ActiveRow, ...]:
    constraints = validation.get("constraints")
    if not isinstance(constraints, Mapping):
        raise ValueError("V3 report has no q-SBC constraint mapping")
    family_by_name = {family.name: family for family in families}
    rows: list[_ActiveRow] = []
    for name in sorted(constraints):
        summary = constraints[name]
        if not isinstance(summary, Mapping):
            raise ValueError("V3 q-SBC constraint summary is invalid")
        status = summary.get("status")
        if status == "validated":
            continue
        if status not in ("violated", "unknown"):
            raise ValueError("V3 q-SBC constraint status is invalid")
        family = family_by_name[name]
        tree_name = str(summary["tree_file"])
        tree_hash = str(summary["tree_file_sha256"])
        tree = json.loads(
            (validation_directory / tree_name).read_text(encoding="utf-8")
        )
        terminal_count = 0
        for leaf in tree["leaves"]:
            disposition = leaf.get("disposition")
            if disposition not in ("proved_violation", "unresolved"):
                continue
            terminal_count += 1
            box = _parse_exact_box(leaf.get("box_exact"))
            if not (
                family.root_box_exact[0] <= box[0] < box[1]
                <= family.root_box_exact[1]
                and family.root_box_exact[2] <= box[2] < box[3]
                <= family.root_box_exact[3]
            ):
                raise ValueError("V3 feedback box lies outside its bound root")
            domain_class = str(leaf.get("domain_class"))
            if domain_class != _classify_box(family, box):
                raise ValueError("V3 feedback domain classification mismatch")
            lower = leaf.get("lower")
            score = None
            if isinstance(lower, Mapping) and "exact" in lower:
                score = float(_parse_exact_fraction(lower["exact"]))
            rows.append(
                _ActiveRow(
                    family_name=name,
                    row_kind="v3_exact_box",
                    box_exact=box,
                    source="V3_failed_or_unknown_exact_Bernstein_leaf",
                    domain_class=domain_class,
                    ranking_score=score,
                    v3_call_index=v3_call_index,
                    v3_tree_file=tree_name,
                    v3_tree_file_sha256=tree_hash,
                    v3_leaf_depth=int(leaf["depth"]),
                    v3_leaf_disposition=str(disposition),
                )
            )
        if terminal_count == 0:
            raise ValueError("nonvalidated V3 q-SBC tree has no terminal feedback leaf")
    rows.sort(
        key=lambda row: (
            float("inf") if row.ranking_score is None else row.ranking_score,
            row.family_name,
            tuple(row.box_exact),
        )
    )
    return tuple(rows)


def _save_active_row_ledger(path: Path, store: _ActiveRowStore) -> None:
    payload = store.payload()
    payload["ledger_content_sha256"] = content_sha256(payload)
    _write_json(path, payload)


def _seed_vector(
    *,
    seed: _G9Seed,
    chebyshev_indices: Sequence[Power2D],
    rho_index: int,
) -> np.ndarray | None:
    if not seed.chebyshev_by_power:
        return None
    values = np.zeros(rho_index + 1, dtype=np.float64)
    for index, power in enumerate(chebyshev_indices):
        values[index] = float(seed.chebyshev_by_power.get(power, 0.0))
    if np.max(np.abs(values[:rho_index])) > G10_COEFFICIENT_BOUND:
        return None
    values[rho_index] = G10_RHO_BOUNDS[0]
    return values


def _candidate_payload(
    *,
    stage: G10AStageSpec,
    outer_iteration: int,
    v3_call_pending: bool,
    chebyshev_indices: Sequence[Power2D],
    chebyshev_coefficients: Sequence[float],
    barrier: SparsePolynomial2D,
    controller: FrozenController,
    rho: float,
    minimum_lp_slack: float,
    maximum_bound_violation: float,
    solve_result: _NumericSolveResult,
    active_store: _ActiveRowStore,
) -> dict[str, Any]:
    payload: dict[str, Any] = {
        "schema_version": 1,
        "kind": "g10a_qsbc_numeric_candidate",
        "synthesis_id": G10A_SYNTHESIS_ID,
        "proof_route_id": G10A_PROOF_ROUTE_ID,
        "stage_id": stage.stage_id,
        "outer_iteration": outer_iteration,
        "v3_call_pending": v3_call_pending,
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
        "q": G10_Q,
        "q_float64_hex": G10_Q.hex(),
        "epsbar": G10_EPSBAR,
        "epsbar_float64_hex": G10_EPSBAR.hex(),
        "rho": rho,
        "rho_float64_hex": rho.hex(),
        "minimum_active_lp_slack": minimum_lp_slack,
        "maximum_bound_violation": maximum_bound_violation,
        "active_linear_inequality_count": int(active_store.matrix.shape[0]),
        "active_row_block_count": len(active_store.rows),
        "solver_backend": solve_result.backend,
        "solver_backend_status": solve_result.backend_status,
        "solver_message": solve_result.message,
        "solver_iterations": solve_result.iterations,
        "solver_basis_valid_before": solve_result.basis_valid_before,
        "solver_basis_valid_after": solve_result.basis_valid_after,
        "solver_fallback_reason": solve_result.fallback_reason,
        "formal_claim_allowed_before_independent_validation": False,
        "validation_subject_sha256": content_sha256(barrier.payload()),
    }
    payload["content_sha256"] = content_sha256(payload)
    return payload


def _validate_candidate_v3(
    *,
    barrier: SparsePolynomial2D,
    controller: FrozenController,
    output_dir: Path,
) -> Path:
    from .rational_certificate_validation import validate_qsbc_values_v3

    return validate_qsbc_values_v3(
        barrier=barrier,
        q=G10_Q,
        epsbar=G10_EPSBAR,
        controller=controller,
        output_dir=output_dir,
    )


def _publish_g10a_certificate(
    *,
    destination: Path,
    stage: G10AStageSpec,
    outer_iteration: int,
    barrier: SparsePolynomial2D,
    controller: FrozenController,
    rho: float,
    minimum_lp_slack: float,
    active_store: _ActiveRowStore,
    validation: Mapping[str, Any],
    validation_path: Path,
    solver_backend: str,
) -> Path:
    certificate = QSBCertificate(
        certificate_id=(
            f"CBF_ROVER_G10A_{stage.stage_id}_O{outer_iteration}_"
            f"{controller.sha256[:12]}"
        ),
        barrier=barrier,
        q=G10_Q,
        epsbar=G10_EPSBAR,
        controller_content_sha256=controller.sha256,
        schema_version=2,
        solver={
            "method": "V3_feedback_active_Bernstein_cut_LP",
            "synthesis_id": G10A_SYNTHESIS_ID,
            "proof_route_id": G10A_PROOF_ROUTE_ID,
            "solver_backend": solver_backend,
            "stage_id": stage.stage_id,
            "outer_iteration": outer_iteration,
            "barrier_basis": "total_degree_tensor_chebyshev",
            "barrier_degree": stage.barrier_degree,
            "controller_degree": controller_max_degree(controller),
            "rho": rho,
            "minimum_active_lp_slack": minimum_lp_slack,
            "active_linear_inequality_count": int(active_store.matrix.shape[0]),
            "active_row_block_count": len(active_store.rows),
            "G9_role": G10A_G9_ROLE,
            "active_rows_are_not_formal_evidence": True,
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
    return certificate_path


def solve_g10a_stage(
    *,
    frozen_controller_path: str | Path,
    g9_seed_path: str | Path,
    output_dir: str | Path,
    stage: G10AStageSpec,
) -> Path:
    """Run one finite G10A active-set stage and fail closed."""

    if stage not in G10A_STAGE_SCHEDULE:
        raise ValueError("G10A stage is outside the frozen finite schedule")
    controller_path = Path(frozen_controller_path).resolve()
    controller = load_frozen_controller(controller_path)
    controller_file_hash = file_sha256(controller_path)
    _assert_frozen_c2(
        controller,
        controller_file_sha256=controller_file_hash,
    )
    seed = _load_g9_seed(g9_seed_path)
    if seed.barrier.degree > stage.barrier_degree:
        raise ValueError("G10A G9 seed degree exceeds the active stage degree")

    destination = Path(output_dir).resolve()
    destination.mkdir(parents=True, exist_ok=False)
    manifest_path = destination / "manifest.json"
    ledger_path = destination / "active_row_ledger.json"
    manifest: dict[str, Any] = {
        "schema_version": 1,
        "kind": "g10a_v3_feedback_active_set_synthesis",
        "synthesis_id": G10A_SYNTHESIS_ID,
        "proof_route_id": G10A_PROOF_ROUTE_ID,
        "status": "building",
        "stage": {
            "stage_id": stage.stage_id,
            "barrier_degree": stage.barrier_degree,
            "max_outer_iterations": stage.max_outer_iterations,
            "max_v3_calls": stage.max_v3_calls,
            "aggregate_lp_time_limit_s": stage.time_limit_s,
            "v3_checkpoint_period": G10A_V3_CHECKPOINT_PERIOD,
        },
        "fixed_scalars": {
            "q": G10_Q,
            "q_float64_hex": G10_Q.hex(),
            "q_exactly_no_greater_than_one_tenth": (
                Fraction.from_float(G10_Q) <= Fraction(1, 10)
            ),
            "epsbar": G10_EPSBAR,
            "epsbar_float64_hex": G10_EPSBAR.hex(),
        },
        "controller": {
            "file_sha256": controller_file_hash,
            "content_sha256": controller.sha256,
            "frozen_file_sha256": G10_CONTROLLER_FILE_SHA256,
            "frozen_content_sha256": G10_CONTROLLER_CONTENT_SHA256,
        },
        "g9_seed": {
            "role": G10A_G9_ROLE,
            "file": seed.path.name,
            "file_sha256": seed.file_sha256,
            "report_content_sha256": seed.report_content_sha256,
            "kind": seed.kind,
            "source_mode": seed.source_mode,
            "source_probe_file_sha256": seed.source_probe_file_sha256,
            "legacy_declared_report_content_sha256": (
                seed.legacy_declared_report_content_sha256
            ),
            "current_recomputed_probe_content_sha256": (
                seed.current_recomputed_probe_content_sha256
            ),
            "legacy_declared_matches_current": (
                None
                if seed.legacy_declared_report_content_sha256 is None
                else False
            ),
            "barrier_payload_sha256": content_sha256(seed.barrier.payload()),
            "barrier_degree": seed.barrier.degree,
            "diagnostic_q": seed.q,
            "diagnostic_epsbar": seed.epsbar,
            "formal_claim_allowed": False,
        },
        "options": {
            "single_global_barrier": True,
            "barrier_basis": "total_degree_tensor_chebyshev",
            "barrier_coefficient_bound": G10_COEFFICIENT_BOUND,
            "rho_bounds": list(G10_RHO_BOUNDS),
            "G9_catalog_depth": G10A_G9_CATALOG_DEPTH,
            "G9_rows_per_outer": G10A_G9_ROWS_PER_OUTER,
            "V3_feedback_numeric_margin": G10A_FEEDBACK_NUMERIC_MARGIN,
            "V3_boundary_leaf_policy": "conservative_full_leaf_box",
            "automatic_degree_escalation_beyond_schedule_allowed": False,
        },
        "solver": {
            "preferred": "highspy_incremental_rows_basis",
            "fallback": "scipy_highs_deterministic_sparse_rebuild",
            "fallback_is_permanent_after_first_API_error": True,
            "solver_status_is_not_formal_evidence": True,
        },
        "independent_validation": {
            "required": G10_VALIDATOR_ID,
            "report_schema_version": 2,
            "acceptance_tolerance": 0.0,
            "reads_solver_status": False,
            "reads_active_LP_rows": False,
            "unknown": "fail_closed_and_feedback_exact_leaf_if_bound",
        },
        "outer_iterations": [],
        "v3_calls": [],
    }
    _save_manifest(manifest_path, manifest)

    chebyshev_indices, barrier_basis = total_degree_chebyshev_basis(
        stage.barrier_degree
    )
    rho_index = len(chebyshev_indices)
    variable_count = rho_index + 1
    objective = np.zeros(variable_count, dtype=np.float64)
    objective[rho_index] = -1.0
    bounds = (
        *((-G10_COEFFICIENT_BOUND, G10_COEFFICIENT_BOUND) for _ in range(rho_index)),
        G10_RHO_BOUNDS,
    )
    families = _constraint_families(
        controller=controller,
        barrier_basis=barrier_basis,
    )
    anchors, dormant_rows = _g9_ranked_rows(
        seed=seed,
        controller=controller,
        families=families,
    )
    catalog_payload: dict[str, Any] = {
        "schema_version": 1,
        "kind": "g10a_G9_ranked_diagnostic_row_catalog",
        "G9_role": G10A_G9_ROLE,
        "seed_report_content_sha256": seed.report_content_sha256,
        "anchor_count": len(anchors),
        "dormant_count": len(dormant_rows),
        "anchors": [row.payload() for row in anchors],
        "dormant": [row.payload() for row in dormant_rows],
        "formal_claim_allowed": False,
    }
    catalog_payload["catalog_content_sha256"] = content_sha256(catalog_payload)
    catalog_path = destination / "g9_ranked_row_catalog.json"
    _write_json(catalog_path, catalog_payload)
    manifest["g9_ranked_catalog"] = {
        "file": catalog_path.name,
        "file_sha256": file_sha256(catalog_path),
        "content_sha256": catalog_payload["catalog_content_sha256"],
        "anchor_count": len(anchors),
        "dormant_count": len(dormant_rows),
    }

    store = _ActiveRowStore(
        families=families,
        variable_count=variable_count,
        rho_index=rho_index,
    )
    _, _, accepted_anchors = store.append(anchors)
    if len(accepted_anchors) != len(families):
        raise ValueError("G10A did not activate exactly one G9 anchor per family")
    _save_active_row_ledger(ledger_path, store)
    manifest["active_problem"] = {
        "decision_variable_count": variable_count,
        "barrier_coefficient_count": rho_index,
        "rho_variable_count": 1,
        "V3_qsbc_family_count": len(families),
        "initial_active_row_block_count": len(store.rows),
        "initial_linear_inequality_count": int(store.matrix.shape[0]),
        "global_dense_matrix_constructed": False,
    }
    manifest["status"] = "solving"
    _save_manifest(manifest_path, manifest)

    solver = _IncrementalOrRebuildSolver(
        matrix=store.matrix,
        right_hand_side=store.right_hand_side,
        objective=objective,
        bounds=bounds,
        seed_values=_seed_vector(
            seed=seed,
            chebyshev_indices=chebyshev_indices,
            rho_index=rho_index,
        ),
    )
    dormant_index = 0
    cumulative_lp_time_s = 0.0
    v3_call_count = 0
    seen_barrier_hashes: set[str] = set()

    for outer_iteration in range(1, stage.max_outer_iterations + 1):
        added_g9_rows: tuple[_ActiveRow, ...] = ()
        if outer_iteration > 1 and dormant_index < len(dormant_rows):
            batch = dormant_rows[
                dormant_index : dormant_index + G10A_G9_ROWS_PER_OUTER
            ]
            dormant_index += len(batch)
            new_matrix, new_rhs, added_g9_rows = store.append(batch)
            solver.append_rows(new_matrix, new_rhs)
            _save_active_row_ledger(ledger_path, store)

        remaining_lp_time_s = stage.time_limit_s - cumulative_lp_time_s
        if remaining_lp_time_s <= 0.0:
            manifest["status"] = "not_ready_aggregate_lp_time_budget_exhausted"
            break
        solve_result = solver.solve(time_limit_s=remaining_lp_time_s)
        cumulative_lp_time_s += solve_result.solve_time_s
        iteration_record: dict[str, Any] = {
            "outer_iteration": outer_iteration,
            "added_G9_diagnostic_row_blocks": len(added_g9_rows),
            "active_row_block_count": len(store.rows),
            "active_linear_inequality_count": int(store.matrix.shape[0]),
            "active_matrix_nonzero_count": int(store.matrix.nnz),
            "solver_backend": solve_result.backend,
            "solver_backend_status": solve_result.backend_status,
            "solver_success_means_numeric_vector_available": solve_result.success,
            "solver_message": solve_result.message,
            "solver_iterations": solve_result.iterations,
            "solver_time_s": solve_result.solve_time_s,
            "aggregate_lp_time_used_s": cumulative_lp_time_s,
            "aggregate_lp_budget_exceeded": (
                solve_result.aggregate_budget_exceeded
                or cumulative_lp_time_s > stage.time_limit_s
            ),
            "basis_valid_before": solve_result.basis_valid_before,
            "basis_valid_after": solve_result.basis_valid_after,
            "fallback_reason": solve_result.fallback_reason,
            "formal_claim_allowed": False,
        }
        manifest["outer_iterations"].append(iteration_record)
        if (
            solve_result.aggregate_budget_exceeded
            or cumulative_lp_time_s > stage.time_limit_s
        ):
            manifest["status"] = (
                "not_ready_aggregate_lp_time_budget_exhausted"
            )
            _save_manifest(manifest_path, manifest)
            break
        if solve_result.values is None:
            manifest["status"] = "not_ready_no_numeric_active_LP_candidate"
            _save_manifest(manifest_path, manifest)
            break

        values = solve_result.values
        rho = float(values[rho_index])
        minimum_lp_slack = float(
            np.min(store.right_hand_side - store.matrix @ values)
        )
        maximum_bound_violation = _maximum_bound_violation(values, bounds)
        numeric_incumbent_feasible = (
            minimum_lp_slack >= -G10_NUMERIC_FEASIBILITY_TOLERANCE
            and maximum_bound_violation <= G10_NUMERIC_FEASIBILITY_TOLERANCE
        )
        chebyshev_coefficients = tuple(float(value) for value in values[:rho_index])
        barrier = chebyshev_coefficients_to_monomial(
            indices=chebyshev_indices,
            coefficients=chebyshev_coefficients,
            basis=barrier_basis,
        )
        barrier_hash = content_sha256(barrier.payload())
        checkpoint = (
            outer_iteration % G10A_V3_CHECKPOINT_PERIOD == 0
            or outer_iteration == stage.max_outer_iterations
        )
        candidate_directory = destination / f"outer_{outer_iteration:02d}"
        candidate_directory.mkdir(parents=False, exist_ok=False)
        candidate = _candidate_payload(
            stage=stage,
            outer_iteration=outer_iteration,
            v3_call_pending=checkpoint,
            chebyshev_indices=chebyshev_indices,
            chebyshev_coefficients=chebyshev_coefficients,
            barrier=barrier,
            controller=controller,
            rho=rho,
            minimum_lp_slack=minimum_lp_slack,
            maximum_bound_violation=maximum_bound_violation,
            solve_result=solve_result,
            active_store=store,
        )
        candidate_path = candidate_directory / "numeric_candidate.json"
        _write_json(candidate_path, candidate)
        iteration_record["numeric_candidate"] = {
            "file": str(candidate_path.relative_to(destination)),
            "file_sha256": file_sha256(candidate_path),
            "content_sha256": candidate["content_sha256"],
            "barrier_payload_sha256": barrier_hash,
            "rho": rho,
            "minimum_active_lp_slack": minimum_lp_slack,
            "maximum_bound_violation": maximum_bound_violation,
            "within_numeric_solver_tolerance": numeric_incumbent_feasible,
        }
        _save_manifest(manifest_path, manifest)
        if not checkpoint:
            continue

        if v3_call_count >= stage.max_v3_calls:
            manifest["status"] = "not_ready_V3_call_budget_exhausted"
            break
        if barrier_hash in seen_barrier_hashes:
            manifest["status"] = "not_ready_repeated_V3_subject_no_progress"
            break
        seen_barrier_hashes.add(barrier_hash)
        v3_call_count += 1
        gc.collect()
        try:
            validation_path = _validate_candidate_v3(
                barrier=barrier,
                controller=controller,
                output_dir=candidate_directory / "independent_validation_v3",
            )
        except Exception as error:
            manifest["status"] = "not_ready_V3_execution_error_fail_closed"
            manifest["V3_execution_error"] = {
                "type": type(error).__name__,
                "message": str(error),
            }
            _save_manifest(manifest_path, manifest)
            break
        validation = json.loads(validation_path.read_text(encoding="utf-8"))
        binding_failures = _v3_feedback_binding_failures(
            validation=validation,
            validation_directory=validation_path.parent,
            barrier=barrier,
            controller=controller,
            families=families,
        )
        v3_record: dict[str, Any] = {
            "v3_call_index": v3_call_count,
            "outer_iteration": outer_iteration,
            "file": str(validation_path.relative_to(destination)),
            "file_sha256": file_sha256(validation_path),
            "report_content_sha256": validation.get("report_content_sha256"),
            "validator_id": validation.get("validator_id"),
            "status": validation.get("status"),
            "global_polynomial_status": validation.get(
                "global_polynomial_status"
            ),
            "first_slice_ready": validation.get("first_slice_ready"),
            "subject_binding_failures": list(binding_failures),
            "formal_claim_allowed_before_release_gate": False,
        }
        manifest["v3_calls"].append(v3_record)
        if binding_failures:
            manifest["status"] = "not_ready_V3_subject_or_tree_binding_failed"
            _save_manifest(manifest_path, manifest)
            break

        if validation.get("status") == "validated":
            release_passed, release_failures = _v3_release_gate(
                validation=validation,
                barrier=barrier,
                controller=controller,
                validation_directory=validation_path.parent,
            )
            v3_record["release_gate_passed"] = release_passed
            v3_record["release_gate_failures"] = list(release_failures)
            if not release_passed:
                manifest["status"] = "not_ready_V3_release_gate_failed"
                _save_manifest(manifest_path, manifest)
                break
            certificate_path = _publish_g10a_certificate(
                destination=destination,
                stage=stage,
                outer_iteration=outer_iteration,
                barrier=barrier,
                controller=controller,
                rho=rho,
                minimum_lp_slack=minimum_lp_slack,
                active_store=store,
                validation=validation,
                validation_path=validation_path,
                solver_backend=solve_result.backend,
            )
            manifest["certificate"] = {
                "file": certificate_path.name,
                "file_sha256": file_sha256(certificate_path),
            }
            manifest["selected_outer_iteration"] = outer_iteration
            manifest["status"] = "ready"
            manifest["solver_backend_history"] = solver.backend_history
            manifest["active_row_ledger"] = {
                "file": ledger_path.name,
                "file_sha256": file_sha256(ledger_path),
            }
            _save_manifest(manifest_path, manifest)
            return manifest_path

        try:
            feedback = _v3_feedback_rows(
                validation=validation,
                validation_directory=validation_path.parent,
                families=families,
                v3_call_index=v3_call_count,
            )
            new_matrix, new_rhs, accepted_feedback = store.append(feedback)
        except Exception as error:
            manifest["status"] = "not_ready_V3_feedback_schema_error_fail_closed"
            manifest["V3_feedback_error"] = {
                "type": type(error).__name__,
                "message": str(error),
            }
            _save_manifest(manifest_path, manifest)
            break
        v3_record["feedback_leaf_count"] = len(feedback)
        v3_record["new_feedback_row_block_count"] = len(accepted_feedback)
        v3_record["feedback_exact_boxes"] = [
            row.payload() for row in accepted_feedback
        ]
        if not accepted_feedback:
            manifest["status"] = "not_ready_V3_feedback_made_no_progress"
            _save_manifest(manifest_path, manifest)
            break
        solver.append_rows(new_matrix, new_rhs)
        _save_active_row_ledger(ledger_path, store)
        _save_manifest(manifest_path, manifest)
    else:
        manifest["status"] = "not_ready_outer_iteration_budget_exhausted"

    if manifest.get("status") == "solving":
        manifest["status"] = "not_ready"
    manifest["terminal_reason"] = manifest["status"]
    manifest["aggregate_lp_time_used_s"] = cumulative_lp_time_s
    manifest["V3_call_count"] = v3_call_count
    manifest["solver_backend_history"] = solver.backend_history
    manifest["active_row_ledger"] = {
        "file": ledger_path.name,
        "file_sha256": file_sha256(ledger_path),
    }
    _save_manifest(manifest_path, manifest)
    return manifest_path


def synthesize_g10a_qsbc_schedule(
    *,
    frozen_controller_path: str | Path,
    g9_seed_path: str | Path,
    output_dir: str | Path,
) -> Path:
    """Run the frozen W16/W18/W20 G10A schedule, stopping at first V3 proof."""

    destination = Path(output_dir).resolve()
    destination.mkdir(parents=True, exist_ok=False)
    manifest_path = destination / "schedule_manifest.json"
    schedule_manifest: dict[str, Any] = {
        "schema_version": 1,
        "kind": "g10a_finite_qsbc_schedule",
        "synthesis_id": G10A_SYNTHESIS_ID,
        "proof_route_id": G10A_PROOF_ROUTE_ID,
        "status": "running",
        "q": G10_Q,
        "q_float64_hex": G10_Q.hex(),
        "epsbar": G10_EPSBAR,
        "epsbar_float64_hex": G10_EPSBAR.hex(),
        "G9_seed_file_sha256": file_sha256(Path(g9_seed_path).resolve()),
        "schedule": [
            {
                "stage_id": stage.stage_id,
                "barrier_degree": stage.barrier_degree,
                "max_outer_iterations": stage.max_outer_iterations,
                "max_V3_calls": stage.max_v3_calls,
                "aggregate_LP_time_limit_s": stage.time_limit_s,
            }
            for stage in G10A_STAGE_SCHEDULE
        ],
        "automatic_escalation_beyond_schedule_allowed": False,
        "attempts": [],
    }
    _save_manifest(manifest_path, schedule_manifest)
    for stage in G10A_STAGE_SCHEDULE:
        stage_directory = destination / stage.stage_id
        try:
            stage_manifest_path = solve_g10a_stage(
                frozen_controller_path=frozen_controller_path,
                g9_seed_path=g9_seed_path,
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
                "G10A_stage_execution_error_fail_closed"
            )
            _save_manifest(manifest_path, schedule_manifest)
            return manifest_path
        stage_manifest = json.loads(stage_manifest_path.read_text(encoding="utf-8"))
        schedule_manifest["attempts"].append(
            {
                "stage_id": stage.stage_id,
                "status": stage_manifest["status"],
                "manifest": str(stage_manifest_path.relative_to(destination)),
                "manifest_file_sha256": file_sha256(stage_manifest_path),
                "outer_iteration_count": len(
                    stage_manifest.get("outer_iterations", ())
                ),
                "V3_call_count": len(stage_manifest.get("v3_calls", ())),
            }
        )
        if stage_manifest["status"] == "ready":
            schedule_manifest["status"] = "ready"
            schedule_manifest["selected_stage"] = stage.stage_id
            certificate_path = stage_directory / "certificate.json"
            schedule_manifest["certificate"] = {
                "file": str(certificate_path.relative_to(destination)),
                "file_sha256": file_sha256(certificate_path),
            }
            _save_manifest(manifest_path, schedule_manifest)
            return manifest_path
        _save_manifest(manifest_path, schedule_manifest)
    schedule_manifest["status"] = "not_ready"
    schedule_manifest["terminal_reason"] = (
        "all_finite_G10A_stages_failed_same_V3_ready_gate"
    )
    _save_manifest(manifest_path, schedule_manifest)
    return manifest_path
