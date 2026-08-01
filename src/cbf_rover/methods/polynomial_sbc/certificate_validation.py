"""Solver-independent global replay of all G4 q-SBC inequalities."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any, Callable

from cbf_rover.domain.planar import CANONICAL_SCENARIO, CANONICAL_SYSTEM

from .algebra import (
    add,
    constant,
    exact_expected_next_polynomial,
    negate,
    sparse_to_numeric,
)
from .bernstein import (
    BernsteinValidationOptions,
    Box,
    DomainClass,
    classify_full_box,
    validate_polynomial_nonnegative,
)
from .formal_regions import normalized_box, safe_drift_cells
from .polynomial import (
    FrozenController,
    PolynomialControllerPiece,
    SparsePolynomial2D,
    content_sha256,
    controller_piece_for_x,
    controller_pieces,
    controller_switch_x_breaks,
)
from .rational_certificate_validation import (
    validate_qsbc_values_v3 as validate_qsbc_values_v3,
)


INDEPENDENT_VALIDATOR_ID = "G5_EXACT_FLOAT_BERNSTEIN_REPLAY_V2"
FIRST_SLICE_Q_MAX = 0.10


def _write_json(path: Path, payload: dict[str, Any]) -> None:
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


def _physical_box(box: Box) -> Box:
    normalization = CANONICAL_SCENARIO.normalization
    return (
        normalization.center_x_m + normalization.scale_x_m * box[0],
        normalization.center_x_m + normalization.scale_x_m * box[1],
        normalization.center_y_m + normalization.scale_y_m * box[2],
        normalization.center_y_m + normalization.scale_y_m * box[3],
    )


def _distance_bounds_squared(box: Box, disk: Any) -> tuple[float, float]:
    x_low, x_high, y_low, y_high = box
    closest_x = min(max(disk.center_x, x_low), x_high)
    closest_y = min(max(disk.center_y, y_low), y_high)
    minimum = (closest_x - disk.center_x) ** 2 + (
        closest_y - disk.center_y
    ) ** 2
    maximum = max(
        (x_value - disk.center_x) ** 2 + (y_value - disk.center_y) ** 2
        for x_value in (x_low, x_high)
        for y_value in (y_low, y_high)
    )
    return minimum, maximum


def _include_disk_classifier(disk: Any) -> Callable[[Box], DomainClass]:
    def classify(box: Box) -> DomainClass:
        minimum, maximum = _distance_bounds_squared(_physical_box(box), disk)
        radius_squared = disk.radius**2
        if minimum > radius_squared:
            return "outside"
        if maximum <= radius_squared:
            return "inside"
        return "boundary"

    return classify


def _exclude_goal_classifier(box: Box) -> DomainClass:
    minimum, maximum = _distance_bounds_squared(
        _physical_box(box),
        CANONICAL_SCENARIO.goal,
    )
    radius_squared = CANONICAL_SCENARIO.goal.radius**2
    if maximum <= radius_squared:
        return "outside"
    if minimum > radius_squared:
        return "inside"
    return "boundary"


def _controller_piece_for_cell(
    controller: FrozenController,
    x_min_m: float,
    x_max_m: float,
) -> PolynomialControllerPiece:
    piece = controller_piece_for_x(
        controller,
        0.5 * (float(x_min_m) + float(x_max_m)),
    )
    if piece.x_min_m > x_min_m or piece.x_max_m < x_max_m:
        raise ValueError("drift cell crosses a controller switch")
    return piece


def validate_qsbc_values(
    *,
    barrier: SparsePolynomial2D,
    q: float,
    epsbar: float,
    controller: FrozenController,
    output_dir: str | Path,
    options: BernsteinValidationOptions | None = None,
) -> Path:
    """Validate final numerics without loading a solver task or multiplier."""

    destination = Path(output_dir).resolve()
    destination.mkdir(parents=True, exist_ok=False)
    validation_options = options or BernsteinValidationOptions(
        required_margin=0.0,
        acceptance_tolerance=1.0e-7,
        max_depth=20,
        minimum_width=2.0**-20,
        max_leaves=400_000,
    )
    barrier_polynomial = sparse_to_numeric(barrier)
    constraints: dict[str, dict[str, Any]] = {}
    constraints["W_nonnegative_on_X"] = validate_polynomial_nonnegative(
        barrier_polynomial,
        (-1.0, 1.0, -1.0, 1.0),
        classify_domain=classify_full_box,
        options=validation_options,
    )

    initial = CANONICAL_SCENARIO.initial
    initial_physical_box = (
        initial.center_x - initial.radius,
        initial.center_x + initial.radius,
        initial.center_y - initial.radius,
        initial.center_y + initial.radius,
    )
    normalization = CANONICAL_SCENARIO.normalization
    initial_box = (
        (initial_physical_box[0] - normalization.center_x_m)
        / normalization.scale_x_m,
        (initial_physical_box[1] - normalization.center_x_m)
        / normalization.scale_x_m,
        (initial_physical_box[2] - normalization.center_y_m)
        / normalization.scale_y_m,
        (initial_physical_box[3] - normalization.center_y_m)
        / normalization.scale_y_m,
    )
    constraints["q_minus_W_on_X0"] = validate_polynomial_nonnegative(
        add(constant(float(q)), negate(barrier_polynomial)),
        initial_box,
        classify_domain=_include_disk_classifier(initial),
        options=validation_options,
    )
    for unsafe in CANONICAL_SCENARIO.unsafe_components:
        constraints[f"W_minus_1_on_{unsafe.name}"] = (
            validate_polynomial_nonnegative(
                add(barrier_polynomial, constant(-1.0)),
                normalized_box(unsafe),
                classify_domain=classify_full_box,
                options=validation_options,
            )
        )

    expected_by_piece = {
        piece.piece_id: exact_expected_next_polynomial(
            barrier,
            controller,
            CANONICAL_SYSTEM,
            piece=piece,
        )
        for piece in controller_pieces(controller)
    }
    for cell in safe_drift_cells(controller_switch_x_breaks(controller)):
        piece = _controller_piece_for_cell(
            controller,
            cell.x_min,
            cell.x_max,
        )
        drift_residual = add(
            barrier_polynomial,
            negate(expected_by_piece[piece.piece_id]),
            constant(-float(epsbar)),
        )
        constraints[
            f"drift_{piece.piece_id}_{cell.name}"
        ] = validate_polynomial_nonnegative(
            drift_residual,
            normalized_box(cell),
            classify_domain=_exclude_goal_classifier,
            options=validation_options,
        )

    for name, result in constraints.items():
        _write_json(destination / f"{name}.json", result)
        result["tree_file"] = f"{name}.json"
        result.pop("leaves", None)
    global_status = (
        "validated"
        if all(result["status"] == "validated" for result in constraints.values())
        else (
            "violated"
            if any(
                result["status"] == "violated"
                for result in constraints.values()
            )
            else "unknown"
        )
    )
    theorem_scalar_gate = 0.0 <= float(q) < 1.0 and float(epsbar) > 0.0
    formal_status = (
        "validated"
        if global_status == "validated" and theorem_scalar_gate
        else ("violated" if global_status == "violated" else "unknown")
    )
    report = {
        "schema_version": 1,
        "kind": "independent_qsbc_validation",
        "validator_id": INDEPENDENT_VALIDATOR_ID,
        "status": formal_status,
        "global_polynomial_status": global_status,
        "global_semialgebraic_status": global_status,
        "theorem_scalar_gate": theorem_scalar_gate,
        "q": float(q),
        "epsbar": float(epsbar),
        "first_slice_q_max": FIRST_SLICE_Q_MAX,
        "first_slice_ready": (
            formal_status == "validated" and float(q) <= FIRST_SLICE_Q_MAX
        ),
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
        "shared_switch_boundaries_checked_on_both_closures": (
            len(controller_pieces(controller)) > 1
        ),
        "constraints": constraints,
        "independence": {
            "solver_status_read": False,
            "SOS_multiplier_read": False,
            "Gram_matrix_read": False,
            "sampled_residual_used": False,
            "polynomials_reconstructed_from_final_W_and_controller_pieces": (
                True
            ),
        },
    }
    report["report_content_sha256"] = content_sha256(report)
    report_path = destination / "validation_report.json"
    _write_json(report_path, report)
    return report_path
