"""CV1 global controller gates and exact-rollout diagnostics."""

from __future__ import annotations

import json
from pathlib import Path
import shutil
from typing import Any

from cbf_rover.adapters.exact.environment import ExactPlanarEnvironment
from cbf_rover.domain.planar import (
    CANONICAL_SCENARIO,
    CANONICAL_SYSTEM,
)

from .algebra import action_disk_residual, robust_domain_residuals
from .bernstein import (
    BernsteinValidationOptions,
    Box,
    DomainClass,
    classify_full_box,
    validate_polynomial_nonnegative,
)
from .polynomial import (
    FrozenController,
    FrozenPiecewisePolynomialController,
    PolynomialControllerPiece,
    content_sha256,
    controller_pieces,
    load_frozen_controller,
)


CONTROLLER_REQUIRED_MARGIN = 1.0e-6
ROLLOUT_SEEDS = (
    3101,
    3109,
    3121,
    3137,
    3163,
    3181,
    3203,
    3221,
    3251,
    3271,
)


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


def _normalized_root_box() -> Box:
    return -1.0, 1.0, -1.0, 1.0


def _normalized_piece_box(piece: PolynomialControllerPiece) -> Box:
    normalization = CANONICAL_SCENARIO.normalization
    return (
        (piece.x_min_m - normalization.center_x_m)
        / normalization.scale_x_m,
        (piece.x_max_m - normalization.center_x_m)
        / normalization.scale_x_m,
        -1.0,
        1.0,
    )


def _physical_box(box: Box) -> Box:
    normalization = CANONICAL_SCENARIO.normalization
    return (
        normalization.center_x_m + normalization.scale_x_m * box[0],
        normalization.center_x_m + normalization.scale_x_m * box[1],
        normalization.center_y_m + normalization.scale_y_m * box[2],
        normalization.center_y_m + normalization.scale_y_m * box[3],
    )


def _box_intersects_closed_box(box: Box, region: Any) -> bool:
    x_low, x_high, y_low, y_high = box
    return not (
        x_high < region.x_min
        or x_low > region.x_max
        or y_high < region.y_min
        or y_low > region.y_max
    )


def _box_inside_closed_box(box: Box, region: Any) -> bool:
    x_low, x_high, y_low, y_high = box
    return (
        region.x_min <= x_low
        and x_high <= region.x_max
        and region.y_min <= y_low
        and y_high <= region.y_max
    )


def _box_disk_distance_bounds_squared(box: Box, disk: Any) -> tuple[float, float]:
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


def classify_nonterminal_normalized_box(box: Box) -> DomainClass:
    """Classify boxes against closure(Psi) for CV1."""

    physical = _physical_box(box)
    scenario = CANONICAL_SCENARIO
    for unsafe in scenario.unsafe_components:
        if _box_inside_closed_box(physical, unsafe):
            return "outside"
    _, goal_maximum = _box_disk_distance_bounds_squared(physical, scenario.goal)
    if goal_maximum <= scenario.goal.radius**2:
        return "outside"

    boundary = False
    for unsafe in scenario.unsafe_components:
        if _box_intersects_closed_box(physical, unsafe):
            boundary = True
    goal_minimum, goal_maximum = _box_disk_distance_bounds_squared(
        physical,
        scenario.goal,
    )
    if goal_minimum <= scenario.goal.radius**2 <= goal_maximum:
        boundary = True
    return "boundary" if boundary else "inside"


def _rollout_gate(
    controller: FrozenController,
    *,
    episodes_per_seed: int = 100,
    max_steps: int = 2000,
) -> dict[str, Any]:
    rows: list[dict[str, Any]] = []
    for base_seed in ROLLOUT_SEEDS:
        for episode_index in range(episodes_per_seed):
            episode_seed = base_seed * 10_000 + episode_index
            environment = ExactPlanarEnvironment(
                max_steps=max_steps,
                reset_mode="initial_disk",
            )
            state = environment.reset(seed=episode_seed)
            reason = "timeout"
            numerical_error = False
            steps = 0
            try:
                for steps in range(1, max_steps + 1):
                    action = controller.act(state)
                    state, _, terminated, truncated, info = environment.step(
                        action
                    )
                    if terminated or truncated:
                        reason = str(info["termination_reason"])
                        break
            except (ArithmeticError, ValueError, OverflowError):
                reason = "numerical_error"
                numerical_error = True
            rows.append(
                {
                    "base_seed": base_seed,
                    "episode_index": episode_index,
                    "episode_seed": episode_seed,
                    "steps": steps,
                    "termination_reason": reason,
                    "goal_reached": reason == "goal_reached",
                    "unsafe_entry": reason == "unsafe_entry",
                    "model_contract_breach": reason == "model_contract_breach",
                    "action_contract_breach": reason
                    == "action_contract_breach",
                    "numerical_error": numerical_error,
                    "final_state_m": list(state.as_tuple()),
                }
            )
    count = len(rows)
    goal_count = sum(bool(row["goal_reached"]) for row in rows)
    hard_failure_count = sum(
        bool(row["unsafe_entry"])
        or bool(row["model_contract_breach"])
        or bool(row["action_contract_breach"])
        or bool(row["numerical_error"])
        for row in rows
    )
    goal_rate = goal_count / count
    return {
        "status": (
            "validated"
            if hard_failure_count == 0 and goal_rate >= 0.99
            else "violated"
        ),
        "episode_count": count,
        "goal_count": goal_count,
        "goal_rate": goal_rate,
        "required_goal_rate": 0.99,
        "hard_failure_count": hard_failure_count,
        "required_hard_failure_count": 0,
        "base_seeds": list(ROLLOUT_SEEDS),
        "episodes_per_seed": episodes_per_seed,
        "max_steps": max_steps,
        "diagnostic_only": True,
        "episodes": rows,
    }


def validate_and_freeze_controller(
    *,
    candidate_path: str | Path,
    output_dir: str | Path,
    run_rollouts: bool = True,
    options: BernsteinValidationOptions | None = None,
) -> Path:
    """Run CV1.  Unknown and violations never produce a frozen controller."""

    destination = Path(output_dir).resolve()
    destination.mkdir(parents=True, exist_ok=False)
    controller = load_frozen_controller(candidate_path)
    validation_options = options or BernsteinValidationOptions(
        required_margin=CONTROLLER_REQUIRED_MARGIN,
        acceptance_tolerance=0.0,
        max_depth=18,
        minimum_width=2.0**-18,
        max_leaves=200_000,
    )
    constraints: dict[str, dict[str, Any]] = {}
    pieces = controller_pieces(controller)
    piecewise = isinstance(controller, FrozenPiecewisePolynomialController)
    for piece in pieces:
        prefix = f"{piece.piece_id}_" if piecewise else ""
        root_box = (
            _normalized_piece_box(piece)
            if piecewise
            else _normalized_root_box()
        )
        constraints[f"{prefix}action_disk"] = (
            validate_polynomial_nonnegative(
                action_disk_residual(
                    controller,
                    CANONICAL_SYSTEM,
                    piece=piece,
                ),
                root_box,
                classify_domain=classify_full_box,
                options=validation_options,
            )
        )
        for name, polynomial in robust_domain_residuals(
            controller,
            CANONICAL_SYSTEM,
            CANONICAL_SCENARIO,
            piece=piece,
        ).items():
            constraints[f"{prefix}{name}"] = (
                validate_polynomial_nonnegative(
                    polynomial,
                    root_box,
                    classify_domain=classify_nonterminal_normalized_box,
                    options=validation_options,
                )
            )
    formal_status = (
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
    for name, result in constraints.items():
        _write_json(destination / f"bernstein_{name}.json", result)
        result["tree_file"] = f"bernstein_{name}.json"
        result.pop("leaves", None)

    rollout = (
        _rollout_gate(controller)
        if formal_status == "validated" and run_rollouts
        else {
            "status": "not_run",
            "reason": (
                "formal_controller_gate_not_validated"
                if formal_status != "validated"
                else "explicitly_disabled"
            ),
            "diagnostic_only": True,
        }
    )
    overall_status = (
        "validated"
        if formal_status == "validated"
        and (rollout["status"] == "validated" or not run_rollouts)
        else (
            "unknown"
            if formal_status == "unknown"
            else "violated"
        )
    )
    decision_id = (
        "C2V1_PIECEWISE_BERNSTEIN_AND_1000_ROLLOUTS"
        if piecewise
        else "CV1_BERNSTEIN_GLOBAL_AND_1000_ROLLOUTS"
    )
    report = {
        "schema_version": 1,
        "kind": "controller_validation",
        "decision_id": decision_id,
        "status": overall_status,
        "controller_id": controller.controller_id,
        "controller_kind": (
            "piecewise_polynomial" if piecewise else "global_polynomial"
        ),
        "controller_content_sha256": controller.sha256,
        "formal_status": formal_status,
        "formal_constraints": constraints,
        "rollout_gate": rollout,
        "partition": {
            "piece_count": len(pieces),
            "pieces": [
                {
                    "piece_id": piece.piece_id,
                    "physical_x_interval_m": [
                        piece.x_min_m,
                        piece.x_max_m,
                    ],
                }
                for piece in pieces
            ],
            "shared_boundaries_validated_on_both_closures": piecewise,
        },
        "semantics": {
            "action_post_clipping": False,
            "noise_support_eliminated_exactly_for_D1": True,
            "certificate_feedback_used": False,
            "unknown_is_fail_closed": True,
        },
    }
    if overall_status == "validated":
        frozen_path = destination / "frozen_controller.json"
        shutil.copyfile(Path(candidate_path).resolve(), frozen_path)
        if load_frozen_controller(frozen_path).sha256 != controller.sha256:
            raise RuntimeError("frozen controller copy did not preserve identity")
        report["frozen_controller"] = frozen_path.name
    report["report_content_sha256"] = content_sha256(report)
    report_path = destination / "validation_report.json"
    _write_json(report_path, report)
    return report_path


def replay_controller_validation(
    validation_report: str | Path,
    frozen_controller: str | Path,
    output_dir: str | Path,
) -> Path:
    """Independent replay intentionally ignores the previous leaf trees."""

    previous = json.loads(
        Path(validation_report).read_text(encoding="utf-8")
    )
    controller = load_frozen_controller(frozen_controller)
    if previous["controller_content_sha256"] != controller.sha256:
        raise ValueError("validation report/controller identity mismatch")
    return validate_and_freeze_controller(
        candidate_path=frozen_controller,
        output_dir=output_dir,
        run_rollouts=False,
    )
