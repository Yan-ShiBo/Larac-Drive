"""G7 high-degree q-SBC synthesis by sufficient Bernstein LP constraints."""

from __future__ import annotations

import json
from math import comb
from pathlib import Path
from typing import Any, Iterable, Mapping, Sequence

import numpy as np
from scipy.optimize import linprog

from cbf_rover.adapters.training.sac_reference import file_sha256
from cbf_rover.domain.planar import CANONICAL_SCENARIO, CANONICAL_SYSTEM

from .algebra import add, exact_expected_next_polynomial, negate
from .certificate import QSBCertificate
from .certificate_validation import validate_qsbc_values
from .formal_regions import (
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


Box = tuple[float, float, float, float]
G7_SYNTHESIS_ID = "G7_BERNSTEIN_PARTITION_HIGHS_DEGREE_14_16_V1"
G7_DEGREE_SCHEDULE = (14, 16)


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


def _axis_monomial_bernstein(
    power: int,
    low: float,
    high: float,
    target_degree: int,
) -> np.ndarray:
    if power < 0 or target_degree < power or low >= high:
        raise ValueError("invalid monomial Bernstein request")
    width = high - low
    result = np.empty(target_degree + 1, dtype=np.float64)
    for bernstein_index in range(target_degree + 1):
        value = 0.0
        for local_power in range(min(power, bernstein_index) + 1):
            value += (
                comb(power, local_power)
                * low ** (power - local_power)
                * width**local_power
                * comb(bernstein_index, local_power)
                / comb(target_degree, local_power)
            )
        result[bernstein_index] = value
    return result


def monomial_bernstein_vector(
    power: Power2D,
    box: Box,
    *,
    target_degree_x: int,
    target_degree_y: int,
) -> np.ndarray:
    x_low, x_high, y_low, y_high = box
    x_values = _axis_monomial_bernstein(
        power[0],
        x_low,
        x_high,
        target_degree_x,
    )
    y_values = _axis_monomial_bernstein(
        power[1],
        y_low,
        y_high,
        target_degree_y,
    )
    return np.outer(x_values, y_values).ravel()


def _polynomial_family_bernstein_matrix(
    polynomials: Sequence[NumericPolynomial],
    box: Box,
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
        raise ValueError("Bernstein polynomial family must not be empty")
    target_degree_x = max(power[0] for power in union_powers)
    target_degree_y = max(power[1] for power in union_powers)
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
    power_index = {
        power: index for index, power in enumerate(union_powers)
    }
    coefficients = np.zeros(
        (len(union_powers), len(polynomials)),
        dtype=np.float64,
    )
    for column, polynomial in enumerate(polynomials):
        for power, coefficient in polynomial.items():
            coefficients[power_index[power], column] = coefficient
    return transform @ coefficients


def _barrier_basis_polynomials(
    powers: Sequence[Power2D],
) -> tuple[NumericPolynomial, ...]:
    return tuple({power: 1.0} for power in powers)


def _drift_basis_polynomials(
    *,
    controller: FrozenController,
    piece: PolynomialControllerPiece,
    powers: Sequence[Power2D],
) -> tuple[NumericPolynomial, ...]:
    return tuple(
        add(
            {power: 1.0},
            negate(
                exact_expected_next_polynomial(
                    SparsePolynomial2D((power,), (1.0,)),
                    controller,
                    CANONICAL_SYSTEM,
                    piece=piece,
                )
            ),
        )
        for power in powers
    )


def _uniform_boxes(parts_per_axis: int) -> tuple[Box, ...]:
    if parts_per_axis < 1:
        raise ValueError("parts_per_axis must be positive")
    breaks = np.linspace(-1.0, 1.0, parts_per_axis + 1)
    return tuple(
        (
            float(x_low),
            float(x_high),
            float(y_low),
            float(y_high),
        )
        for x_low, x_high in zip(breaks[:-1], breaks[1:])
        for y_low, y_high in zip(breaks[:-1], breaks[1:])
    )


def _split_box_both_axes(box: Box) -> tuple[Box, Box, Box, Box]:
    x_low, x_high, y_low, y_high = box
    x_mid = 0.5 * (x_low + x_high)
    y_mid = 0.5 * (y_low + y_high)
    return (
        (x_low, x_mid, y_low, y_mid),
        (x_low, x_mid, y_mid, y_high),
        (x_mid, x_high, y_low, y_mid),
        (x_mid, x_high, y_mid, y_high),
    )


def _physical_box(box: Box) -> Box:
    normalization = CANONICAL_SCENARIO.normalization
    return (
        normalization.center_x_m + normalization.scale_x_m * box[0],
        normalization.center_x_m + normalization.scale_x_m * box[1],
        normalization.center_y_m + normalization.scale_y_m * box[2],
        normalization.center_y_m + normalization.scale_y_m * box[3],
    )


def _goal_box_class(box: Box) -> str:
    x_low, x_high, y_low, y_high = _physical_box(box)
    goal = CANONICAL_SCENARIO.goal
    closest_x = min(max(goal.center_x, x_low), x_high)
    closest_y = min(max(goal.center_y, y_low), y_high)
    minimum = (closest_x - goal.center_x) ** 2 + (
        closest_y - goal.center_y
    ) ** 2
    maximum = max(
        (x_value - goal.center_x) ** 2
        + (y_value - goal.center_y) ** 2
        for x_value in (x_low, x_high)
        for y_value in (y_low, y_high)
    )
    radius_squared = goal.radius**2
    if maximum <= radius_squared:
        return "inside"
    if minimum > radius_squared:
        return "outside"
    return "boundary"


def _goal_exterior_cover(
    box: Box,
    *,
    depth: int,
    maximum_depth: int,
) -> tuple[Box, ...]:
    classification = _goal_box_class(box)
    if classification == "inside":
        return ()
    if classification == "outside":
        return (box,)
    if depth >= maximum_depth:
        normalization = CANONICAL_SCENARIO.normalization
        goal = CANONICAL_SCENARIO.goal
        goal_x = (
            goal.center_x - normalization.center_x_m
        ) / normalization.scale_x_m
        goal_y = (
            goal.center_y - normalization.center_y_m
        ) / normalization.scale_y_m
        if box[0] <= goal_x <= box[1] and box[2] <= goal_y <= box[3]:
            raise RuntimeError(
                "goal boundary depth left the invariant goal center active"
            )
        return (box,)
    x_low, x_high, y_low, y_high = box
    if x_high - x_low >= y_high - y_low:
        midpoint = 0.5 * (x_low + x_high)
        children = (
            (x_low, midpoint, y_low, y_high),
            (midpoint, x_high, y_low, y_high),
        )
    else:
        midpoint = 0.5 * (y_low + y_high)
        children = (
            (x_low, x_high, y_low, midpoint),
            (x_low, x_high, midpoint, y_high),
        )
    return tuple(
        active_box
        for child in children
        for active_box in _goal_exterior_cover(
            child,
            depth=depth + 1,
            maximum_depth=maximum_depth,
        )
    )


def _piece_for_box(
    controller: FrozenController,
    box: Box,
) -> PolynomialControllerPiece:
    physical = _physical_box(box)
    piece = controller_piece_for_x(
        controller,
        0.5 * (physical[0] + physical[1]),
    )
    if piece.x_min_m > physical[0] or piece.x_max_m < physical[1]:
        raise ValueError("Bernstein drift box crosses a controller switch")
    return piece


def _drift_partition(
    controller: FrozenController,
    *,
    split_each_cell: bool,
    goal_boundary_depth: int,
) -> tuple[tuple[PolynomialControllerPiece, Box], ...]:
    result: list[tuple[PolynomialControllerPiece, Box]] = []
    for cell in safe_drift_cells(controller_switch_x_breaks(controller)):
        root = normalized_box(cell)
        initial_boxes = (
            _split_box_both_axes(root) if split_each_cell else (root,)
        )
        for initial_box in initial_boxes:
            for active_box in _goal_exterior_cover(
                initial_box,
                depth=0,
                maximum_depth=goal_boundary_depth,
            ):
                result.append(
                    (
                        _piece_for_box(controller, active_box),
                        active_box,
                    )
                )
    return tuple(result)


def _initial_bounding_box() -> Box:
    initial = CANONICAL_SCENARIO.initial
    normalization = CANONICAL_SCENARIO.normalization
    return (
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


def synthesize_bernstein_qsbc_degree(
    *,
    frozen_controller_path: str | Path,
    barrier_degree: int,
    output_dir: str | Path,
    epsbar: float = 1.0e-6,
    synthesis_margin: float = 2.0e-7,
    time_limit_s: float = 600.0,
    full_box_parts_per_axis: int = 4,
    split_each_drift_cell: bool = True,
    goal_boundary_depth: int = 12,
) -> Path:
    """Solve one sufficient Bernstein LP and independently replay its values."""

    if barrier_degree not in G7_DEGREE_SCHEDULE:
        raise ValueError(
            f"barrier degree must be one of {G7_DEGREE_SCHEDULE}"
        )
    if epsbar <= 0.0 or synthesis_margin <= 0.0:
        raise ValueError("epsbar and synthesis margin must be positive")
    controller = load_frozen_controller(frozen_controller_path)
    destination = Path(output_dir).resolve()
    destination.mkdir(parents=True, exist_ok=False)
    manifest_path = destination / "manifest.json"
    manifest: dict[str, Any] = {
        "schema_version": 1,
        "kind": "g7_bernstein_lp_synthesis_attempt",
        "synthesis_id": G7_SYNTHESIS_ID,
        "status": "building",
        "barrier_degree": barrier_degree,
        "epsbar": epsbar,
        "synthesis_margin": synthesis_margin,
        "controller_content_sha256": controller.sha256,
        "controller_artifact_kind": controller.payload_without_hash()["kind"],
        "options": {
            "time_limit_s": time_limit_s,
            "coefficient_bound": 1000.0,
            "full_box_parts_per_axis": full_box_parts_per_axis,
            "split_each_drift_cell": split_each_drift_cell,
            "goal_boundary_depth": goal_boundary_depth,
            "initial_disk_replaced_by_bounding_box": True,
            "goal_boundary_boxes_are_conservative_supersets": True,
        },
        "solver": {
            "name": "SciPy HiGHS LP",
            "method": "highs",
            "primal_feasibility_tolerance": 1.0e-9,
            "dual_feasibility_tolerance": 1.0e-9,
        },
    }
    _save_manifest(manifest_path, manifest)

    powers = canonical_total_degree_powers(barrier_degree)
    barrier_basis = _barrier_basis_polynomials(powers)
    constraint_matrices: list[np.ndarray] = []
    constraint_bounds: list[np.ndarray] = []
    constraint_counts: dict[str, int] = {}

    def add_positive_family(
        name: str,
        matrix: np.ndarray,
        *,
        q_coefficients: np.ndarray | None = None,
        constant: float,
        required: float,
    ) -> None:
        q_values = (
            np.zeros(matrix.shape[0], dtype=np.float64)
            if q_coefficients is None
            else np.asarray(q_coefficients, dtype=np.float64)
        )
        if q_values.shape != (matrix.shape[0],):
            raise ValueError("q coefficient shape mismatch")
        constraint_matrices.append(
            -np.column_stack((matrix, q_values))
        )
        constraint_bounds.append(
            np.full(
                matrix.shape[0],
                float(constant) - float(required),
                dtype=np.float64,
            )
        )
        constraint_counts[name] = (
            constraint_counts.get(name, 0) + matrix.shape[0]
        )

    for box in _uniform_boxes(full_box_parts_per_axis):
        add_positive_family(
            "W_nonnegative_X",
            _polynomial_family_bernstein_matrix(barrier_basis, box),
            constant=0.0,
            required=synthesis_margin,
        )

    initial_matrix = _polynomial_family_bernstein_matrix(
        barrier_basis,
        _initial_bounding_box(),
    )
    add_positive_family(
        "q_minus_W_X0_bounding_box",
        -initial_matrix,
        q_coefficients=np.ones(initial_matrix.shape[0]),
        constant=0.0,
        required=synthesis_margin,
    )

    for unsafe in CANONICAL_SCENARIO.unsafe_components:
        add_positive_family(
            f"W_minus_1_{unsafe.name}",
            _polynomial_family_bernstein_matrix(
                barrier_basis,
                normalized_box(unsafe),
            ),
            constant=-1.0,
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
    drift_partition = _drift_partition(
        controller,
        split_each_cell=split_each_drift_cell,
        goal_boundary_depth=goal_boundary_depth,
    )
    for piece, box in drift_partition:
        add_positive_family(
            f"drift_{piece.piece_id}",
            _polynomial_family_bernstein_matrix(
                drift_basis_by_piece[piece.piece_id],
                box,
            ),
            constant=-epsbar,
            required=synthesis_margin,
        )

    matrix = np.vstack(constraint_matrices)
    bounds = np.concatenate(constraint_bounds)
    objective = np.zeros(len(powers) + 1, dtype=np.float64)
    objective[-1] = 1.0
    manifest["partition"] = {
        "full_X_box_count": full_box_parts_per_axis**2,
        "unsafe_box_count": len(CANONICAL_SCENARIO.unsafe_components),
        "drift_box_count": len(drift_partition),
        "constraint_counts": constraint_counts,
        "total_linear_inequalities": int(matrix.shape[0]),
        "decision_variable_count": int(matrix.shape[1]),
    }
    manifest["status"] = "solving"
    _save_manifest(manifest_path, manifest)
    result = linprog(
        objective,
        A_ub=matrix,
        b_ub=bounds,
        bounds=[(-1000.0, 1000.0)] * len(powers) + [(0.0, 1.0)],
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

    coefficients = tuple(float(value) for value in result.x[:-1])
    q_value = float(result.x[-1])
    minimum_lp_slack = float(np.min(bounds - matrix @ result.x))
    barrier = SparsePolynomial2D(powers, coefficients)
    candidate = {
        "schema_version": 1,
        "kind": "g7_qsbc_numeric_candidate",
        "synthesis_id": G7_SYNTHESIS_ID,
        "controller_content_sha256": controller.sha256,
        "barrier": barrier.payload(),
        "q": q_value,
        "q_float64_hex": q_value.hex(),
        "epsbar": epsbar,
        "epsbar_float64_hex": epsbar.hex(),
        "minimum_lp_slack": minimum_lp_slack,
        "formal_claim_allowed_before_independent_validation": False,
    }
    candidate["content_sha256"] = content_sha256(candidate)
    candidate_path = destination / "numeric_candidate.json"
    _write_json(candidate_path, candidate)
    manifest.update(
        {
            "status": "optimal_pending_independent_validation",
            "q": q_value,
            "epsbar": epsbar,
            "minimum_lp_slack": minimum_lp_slack,
            "coefficient_max_abs": max(abs(value) for value in coefficients),
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
        certificate_id=(
            f"CBF_ROVER_G7_W{barrier_degree}_{controller.sha256[:12]}"
        ),
        barrier=barrier,
        q=q_value,
        epsbar=epsbar,
        controller_content_sha256=controller.sha256,
        solver={
            "method": "Bernstein_partition_LP",
            "synthesis_id": G7_SYNTHESIS_ID,
            "solver": "SciPy HiGHS",
            "barrier_degree": barrier_degree,
            "controller_degree": controller_max_degree(controller),
            "minimum_lp_slack": minimum_lp_slack,
            "constraint_count": int(matrix.shape[0]),
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


def synthesize_bernstein_qsbc_schedule(
    *,
    frozen_controller_path: str | Path,
    output_dir: str | Path,
    degrees: Iterable[int] = G7_DEGREE_SCHEDULE,
    time_limit_s: float = 600.0,
) -> Path:
    """Run the finite G7 W14/W16 schedule and bind every attempt."""

    schedule = tuple(int(value) for value in degrees)
    if not schedule or any(
        value not in G7_DEGREE_SCHEDULE for value in schedule
    ):
        raise ValueError(f"G7 degrees must be drawn from {G7_DEGREE_SCHEDULE}")
    controller = load_frozen_controller(frozen_controller_path)
    destination = Path(output_dir).resolve()
    destination.mkdir(parents=True, exist_ok=False)
    attempts: list[dict[str, Any]] = []
    selected: dict[str, Any] | None = None
    for degree in schedule:
        attempt_path = synthesize_bernstein_qsbc_degree(
            frozen_controller_path=frozen_controller_path,
            barrier_degree=degree,
            output_dir=destination / f"W_degree_{degree}",
            time_limit_s=time_limit_s,
        )
        attempt = json.loads(attempt_path.read_text(encoding="utf-8"))
        row = {
            "degree": degree,
            "manifest": str(attempt_path.relative_to(destination)),
            "manifest_file_sha256": file_sha256(attempt_path),
            "manifest_content_sha256": attempt[
                "manifest_content_sha256"
            ],
            "status": attempt["status"],
            "q": attempt.get("q"),
            "epsbar": attempt.get("epsbar"),
        }
        attempts.append(row)
        if attempt["status"] == "ready":
            selected = row
            break
    report = {
        "schema_version": 1,
        "kind": "g7_bernstein_lp_degree_schedule",
        "synthesis_id": G7_SYNTHESIS_ID,
        "controller_content_sha256": controller.sha256,
        "schedule": list(schedule),
        "status": "ready" if selected is not None else "not_ready",
        "selected": selected,
        "attempts": attempts,
        "automatic_degree_escalation_beyond_schedule": False,
    }
    report["manifest_content_sha256"] = content_sha256(report)
    report_path = destination / "schedule_manifest.json"
    _write_json(report_path, report)
    return report_path
