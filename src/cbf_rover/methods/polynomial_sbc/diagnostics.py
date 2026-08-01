"""DX1 diagnostics for a failed SAC-to-polynomial PB1 schedule.

This module is deliberately downstream of the frozen PB1 run.  It never
changes checkpoint selection, never emits a controller candidate, and never
feeds a certificate solver.  Its only claim level is diagnostic.
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any, Iterable, Mapping, Sequence

import numpy as np

from cbf_rover.adapters.exact.environment import build_gymnasium_environment
from cbf_rover.adapters.training.sac_reference import (
    file_sha256,
    predict_reference_actions,
    selected_checkpoint_from_manifest,
)
from cbf_rover.domain.planar import CANONICAL_SCENARIO

from .pac import (
    PAC_MAX_ERROR_MPS,
    _candidate_predictions,
    _fit_axis_linf,
    _normalized_points,
)
from .polynomial import (
    canonical_total_degree_powers,
    content_sha256,
)


DX1_DECISION_ID = "DX1_PB1_STRUCTURAL_MISMATCH_DIAGNOSTIC_V1"
DX1_DENSE_DEGREES = (1, 2, 3, 4, 5, 6)
DX1_GRID_SHAPE = (201, 161)
DX1_INITIAL_ROLLOUTS = 50
DX1_UNIFORM_ROLLOUTS = 100
DX1_INITIAL_SEED_BASE = 51_000
DX1_UNIFORM_SEED_BASE = 52_000


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


def _error_summary(values: np.ndarray) -> dict[str, float]:
    errors = np.asarray(values, dtype=np.float64).reshape(-1)
    if not len(errors) or not np.isfinite(errors).all():
        raise ValueError("diagnostic errors must be nonempty and finite")
    return {
        "mean_mps": float(np.mean(errors)),
        "median_mps": float(np.quantile(errors, 0.50)),
        "p90_mps": float(np.quantile(errors, 0.90)),
        "p95_mps": float(np.quantile(errors, 0.95)),
        "p99_mps": float(np.quantile(errors, 0.99)),
        "max_mps": float(np.max(errors)),
        "fraction_above_pb1_gate": float(
            np.mean(errors > PAC_MAX_ERROR_MPS)
        ),
    }


def _safe_grid_mask(x_values: np.ndarray, y_values: np.ndarray) -> np.ndarray:
    x_grid, y_grid = np.meshgrid(
        np.asarray(x_values, dtype=np.float64),
        np.asarray(y_values, dtype=np.float64),
    )
    mask = np.ones_like(x_grid, dtype=bool)
    scenario = CANONICAL_SCENARIO
    for obstacle in scenario.unsafe_components:
        mask &= ~(
            (x_grid >= obstacle.x_min)
            & (x_grid <= obstacle.x_max)
            & (y_grid >= obstacle.y_min)
            & (y_grid <= obstacle.y_max)
        )
    goal_distance_squared = (
        (x_grid - scenario.goal.center_x) ** 2
        + (y_grid - scenario.goal.center_y) ** 2
    )
    mask &= goal_distance_squared > scenario.goal.radius**2
    return mask


def _geometric_zone(x_value: float, y_value: float) -> str:
    if x_value < 0.965:
        longitudinal = "pre_wall"
    elif x_value <= 3.085:
        longitudinal = "wall_span"
    else:
        longitudinal = "post_wall"
    if y_value > 1.385:
        corridor = "upper_corridor"
    elif y_value < -1.385:
        corridor = "lower_corridor"
    else:
        corridor = "interior_band"
    return f"{longitudinal}/{corridor}"


def _load_dataset(
    manifest_path: Path,
    *,
    purpose: str,
) -> tuple[np.ndarray, np.ndarray, dict[str, Any]]:
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    matches = [
        dict(dataset)
        for dataset in manifest["datasets"]
        if dataset["purpose"] == purpose
    ]
    if len(matches) != 1:
        raise ValueError(
            f"{manifest_path} must contain exactly one {purpose} dataset"
        )
    metadata = matches[0]
    dataset_path = manifest_path.parent / str(metadata["file"])
    if file_sha256(dataset_path) != str(metadata["sha256"]):
        raise ValueError(f"dataset hash mismatch: {dataset_path}")
    with np.load(dataset_path, allow_pickle=False) as dataset:
        states = np.asarray(dataset["physical_states_m"], dtype=np.float64)
        actions = np.asarray(
            dataset["reference_actions_mps"],
            dtype=np.float64,
        )
    if (
        states.ndim != 2
        or states.shape[1] != 2
        or actions.shape != states.shape
        or not np.isfinite(states).all()
        or not np.isfinite(actions).all()
    ):
        raise ValueError(f"invalid diagnostic dataset: {dataset_path}")
    if len(states) != int(metadata["sample_count"]):
        raise ValueError(f"dataset sample-count mismatch: {dataset_path}")
    metadata["absolute_path"] = str(dataset_path.resolve())
    return states, actions, metadata


def _dense_fit_diagnostic(
    search_states: np.ndarray,
    search_actions: np.ndarray,
    evaluation_sets: Sequence[tuple[int, np.ndarray, np.ndarray]],
    *,
    degree: int,
    output_dir: Path,
) -> dict[str, Any]:
    support = canonical_total_degree_powers(degree)
    normalized_search = _normalized_points(search_states)
    x_candidate = _fit_axis_linf(
        normalized_search,
        search_actions[:, 0],
        support,
    )
    y_candidate = _fit_axis_linf(
        normalized_search,
        search_actions[:, 1],
        support,
    )
    if x_candidate is None or y_candidate is None:
        raise RuntimeError(f"dense degree-{degree} diagnostic LP failed")

    evaluation_rows: list[dict[str, Any]] = []
    worst_rows: list[dict[str, Any]] = []
    zone_values: dict[str, list[float]] = {}
    maximum_selection_error = 0.0
    for stage, states, actions in evaluation_sets:
        normalized = _normalized_points(states)
        predicted = np.column_stack(
            (
                _candidate_predictions(x_candidate, normalized),
                _candidate_predictions(y_candidate, normalized),
            )
        )
        axis_errors = np.abs(predicted - actions)
        point_errors = np.max(axis_errors, axis=1)
        maximum_selection_error = max(
            maximum_selection_error,
            float(np.max(point_errors)),
        )
        evaluation_rows.append(
            {
                "stage": stage,
                "sample_count": int(len(states)),
                "pointwise_linf": _error_summary(point_errors),
                "u_x": _error_summary(axis_errors[:, 0]),
                "u_y": _error_summary(axis_errors[:, 1]),
            }
        )
        top_indices = np.argsort(point_errors)[-10:][::-1]
        for index in top_indices:
            state = states[index]
            zone = _geometric_zone(float(state[0]), float(state[1]))
            worst_rows.append(
                {
                    "stage": stage,
                    "state_m": [float(state[0]), float(state[1])],
                    "reference_action_mps": [
                        float(actions[index, 0]),
                        float(actions[index, 1]),
                    ],
                    "polynomial_action_mps": [
                        float(predicted[index, 0]),
                        float(predicted[index, 1]),
                    ],
                    "pointwise_linf_mps": float(point_errors[index]),
                    "zone": zone,
                }
            )
        for state, error in zip(states, point_errors):
            zone = _geometric_zone(float(state[0]), float(state[1]))
            zone_values.setdefault(zone, []).append(float(error))

    fit_payload = {
        "schema_version": 1,
        "kind": "dx1_dense_polynomial_fit",
        "claim_level": "diagnostic_only",
        "degree": degree,
        "support": [list(power) for power in support],
        "u_x_coefficients": list(x_candidate.coefficients),
        "u_y_coefficients": list(y_candidate.coefficients),
        "search": {
            "sample_count": int(len(search_states)),
            "u_x_linf_mps": x_candidate.search_linf,
            "u_y_linf_mps": y_candidate.search_linf,
            "u_x_mean_mps": x_candidate.search_mean,
            "u_y_mean_mps": y_candidate.search_mean,
        },
        "selection_evaluations": evaluation_rows,
        "maximum_selection_linf_mps": maximum_selection_error,
        "passes_old_pb1_gate": maximum_selection_error
        <= PAC_MAX_ERROR_MPS,
        "worst_points": sorted(
            worst_rows,
            key=lambda row: float(row["pointwise_linf_mps"]),
            reverse=True,
        )[:20],
        "zone_summaries": {
            zone: _error_summary(np.asarray(values, dtype=np.float64))
            for zone, values in sorted(zone_values.items())
        },
    }
    fit_payload["content_sha256"] = content_sha256(fit_payload)
    artifact = output_dir / f"dense_degree_{degree}.json"
    _write_json(artifact, fit_payload)
    return {
        "degree": degree,
        "artifact": artifact.name,
        "artifact_sha256": file_sha256(artifact),
        "content_sha256": fit_payload["content_sha256"],
        "maximum_selection_linf_mps": maximum_selection_error,
        "passes_old_pb1_gate": fit_payload["passes_old_pb1_gate"],
    }


def _top_neighbor_rows(
    magnitudes: np.ndarray,
    x_positions: np.ndarray,
    y_positions: np.ndarray,
    orientations: Iterable[str],
    *,
    count: int = 20,
) -> list[dict[str, Any]]:
    orientation_values = np.asarray(tuple(orientations), dtype=object)
    if not len(magnitudes):
        return []
    indices = np.argsort(magnitudes)[-count:][::-1]
    return [
        {
            "midpoint_m": [
                float(x_positions[index]),
                float(y_positions[index]),
            ],
            "orientation": str(orientation_values[index]),
            "action_difference_mps": float(magnitudes[index]),
        }
        for index in indices
    ]


def _grid_diagnostic(
    model: Any,
    *,
    x_count: int,
    y_count: int,
) -> dict[str, Any]:
    scenario = CANONICAL_SCENARIO
    x_values = np.linspace(
        scenario.domain.x_min,
        scenario.domain.x_max,
        x_count,
    )
    y_values = np.linspace(
        scenario.domain.y_min,
        scenario.domain.y_max,
        y_count,
    )
    x_grid, y_grid = np.meshgrid(x_values, y_values)
    safe_mask = _safe_grid_mask(x_values, y_values)
    safe_states = np.column_stack((x_grid[safe_mask], y_grid[safe_mask]))
    safe_actions = predict_reference_actions(model, safe_states)

    action_grid = np.full((y_count, x_count, 2), np.nan, dtype=np.float64)
    action_grid[safe_mask] = safe_actions
    horizontal_valid = safe_mask[:, :-1] & safe_mask[:, 1:]
    vertical_valid = safe_mask[:-1, :] & safe_mask[1:, :]
    horizontal_difference = np.linalg.norm(
        action_grid[:, 1:, :] - action_grid[:, :-1, :],
        axis=2,
    )[horizontal_valid]
    vertical_difference = np.linalg.norm(
        action_grid[1:, :, :] - action_grid[:-1, :, :],
        axis=2,
    )[vertical_valid]

    horizontal_x = (
        (x_grid[:, 1:] + x_grid[:, :-1]) / 2.0
    )[horizontal_valid]
    horizontal_y = y_grid[:, :-1][horizontal_valid]
    vertical_x = x_grid[:-1, :][vertical_valid]
    vertical_y = ((y_grid[1:, :] + y_grid[:-1, :]) / 2.0)[vertical_valid]
    neighbor_differences = np.concatenate(
        (horizontal_difference, vertical_difference)
    )
    neighbor_x = np.concatenate((horizontal_x, vertical_x))
    neighbor_y = np.concatenate((horizontal_y, vertical_y))
    orientations = (
        ["x"] * len(horizontal_difference)
        + ["y"] * len(vertical_difference)
    )

    decision_mask = (
        (safe_states[:, 0] >= -0.10)
        & (safe_states[:, 0] <= 0.90)
        & (np.abs(safe_states[:, 1]) <= 0.10)
    )
    decision_actions = safe_actions[decision_mask]
    action_norms = np.linalg.norm(safe_actions, axis=1)

    probe_x = np.linspace(-0.20, 0.95, 231)
    probe_states = np.vstack(
        [
            np.column_stack(
                (probe_x, np.full_like(probe_x, y_value))
            )
            for y_value in (-0.02, 0.0, 0.02)
        ]
    )
    probe_actions = predict_reference_actions(model, probe_states).reshape(
        3,
        len(probe_x),
        2,
    )
    vertical_probe_gap = np.linalg.norm(
        probe_actions[2] - probe_actions[0],
        axis=1,
    )
    return {
        "grid_shape": [x_count, y_count],
        "safe_point_count": int(len(safe_states)),
        "action_norm_mps": _error_summary(action_norms),
        "neighbor_action_difference_mps": _error_summary(
            neighbor_differences
        ),
        "largest_neighbor_changes": _top_neighbor_rows(
            neighbor_differences,
            neighbor_x,
            neighbor_y,
            orientations,
        ),
        "start_decision_band": {
            "sample_count": int(len(decision_actions)),
            "mean_u_y_mps": float(np.mean(decision_actions[:, 1])),
            "fraction_u_y_positive": float(
                np.mean(decision_actions[:, 1] > 0.0)
            ),
            "fraction_u_y_negative": float(
                np.mean(decision_actions[:, 1] < 0.0)
            ),
        },
        "three_line_probe": {
            "x_range_m": [-0.20, 0.95],
            "y_lines_m": [-0.02, 0.0, 0.02],
            "vertical_action_gap_mps": _error_summary(vertical_probe_gap),
            "maximum_gap_x_m": float(
                probe_x[int(np.argmax(vertical_probe_gap))]
            ),
        },
    }


def _rollout_diagnostic(
    model: Any,
    *,
    reset_mode: str,
    episode_count: int,
    seed_base: int,
) -> dict[str, Any]:
    rows: list[dict[str, Any]] = []
    for episode_index in range(episode_count):
        episode_seed = seed_base + episode_index
        environment = build_gymnasium_environment(
            reset_mode=reset_mode,
            seed=episode_seed,
            max_steps=2000,
        )
        observation, _ = environment.reset(seed=episode_seed)
        initial_state = environment.core.state
        minimum_y = initial_state.y_m
        maximum_y = initial_state.y_m
        terminated = False
        truncated = False
        info: dict[str, Any] = {}
        steps = 0
        while not terminated and not truncated:
            raw_action, _ = model.predict(observation, deterministic=True)
            observation, _, terminated, truncated, info = environment.step(
                raw_action
            )
            steps += 1
            minimum_y = min(minimum_y, environment.core.state.y_m)
            maximum_y = max(maximum_y, environment.core.state.y_m)
        reason = str(info.get("termination_reason", "timeout"))
        if maximum_y > 1.385:
            branch = "upper"
        elif minimum_y < -1.385:
            branch = "lower"
        else:
            branch = "neither"
        rows.append(
            {
                "episode_index": episode_index,
                "episode_seed": episode_seed,
                "initial_state_m": list(initial_state.as_tuple()),
                "final_state_m": list(environment.core.state.as_tuple()),
                "steps": steps,
                "termination_reason": reason,
                "route_branch": branch,
                "minimum_y_m": minimum_y,
                "maximum_y_m": maximum_y,
            }
        )
        environment.close()
    reasons = sorted({str(row["termination_reason"]) for row in rows})
    branches = ("upper", "lower", "neither")
    return {
        "reset_mode": reset_mode,
        "episode_count": episode_count,
        "seed_base": seed_base,
        "max_steps": 2000,
        "termination_counts": {
            reason: sum(row["termination_reason"] == reason for row in rows)
            for reason in reasons
        },
        "route_branch_counts": {
            branch: sum(row["route_branch"] == branch for row in rows)
            for branch in branches
        },
        "rows": rows,
        "claim_level": "empirical_diagnostic_only",
    }


def diagnose_pac_failure(
    *,
    selection_manifest: str | Path,
    pac_manifests: Sequence[str | Path],
    output_dir: str | Path,
    device: str = "cpu",
    dense_degrees: Sequence[int] = DX1_DENSE_DEGREES,
    grid_shape: tuple[int, int] = DX1_GRID_SHAPE,
    initial_rollouts: int = DX1_INITIAL_ROLLOUTS,
    uniform_rollouts: int = DX1_UNIFORM_ROLLOUTS,
) -> Path:
    """Run DX1 without producing any controller or certificate input."""

    manifest_paths = [Path(path).resolve() for path in pac_manifests]
    if len(manifest_paths) != 3:
        raise ValueError("DX1 requires exactly three PB1 stage manifests")
    stage_payloads = [
        json.loads(path.read_text(encoding="utf-8"))
        for path in manifest_paths
    ]
    stages = tuple(int(payload["stage"]) for payload in stage_payloads)
    if stages != (1, 2, 3):
        raise ValueError("PB1 manifests must be supplied in stage order 1,2,3")
    if any(payload["status"] != "not_ready" for payload in stage_payloads):
        raise ValueError("DX1 only accepts the three failed PB1 manifests")
    failure_reasons = {
        str(payload.get("failure_reason")) for payload in stage_payloads
    }
    if failure_reasons != {"no_support_pair_passed_selection_gate"}:
        raise ValueError("DX1 requires the PB1 support-selection failure mode")
    selected_hashes = {
        str(payload["selected_checkpoint_sha256"])
        for payload in stage_payloads
    }
    if len(selected_hashes) != 1:
        raise ValueError("PB1 stages do not bind one checkpoint")

    destination = Path(output_dir).resolve()
    destination.mkdir(parents=True, exist_ok=False)
    selection_path = Path(selection_manifest).resolve()
    checkpoint, checkpoint_manifest = selected_checkpoint_from_manifest(
        selection_path
    )
    selected_checkpoint_hash = str(checkpoint_manifest["checkpoint_sha256"])
    if selected_hashes != {selected_checkpoint_hash}:
        raise ValueError("selection/PB1 checkpoint identity mismatch")

    search_states, search_actions, search_metadata = _load_dataset(
        manifest_paths[2],
        purpose="support_search",
    )
    evaluation_sets: list[tuple[int, np.ndarray, np.ndarray]] = []
    evaluation_metadata: list[dict[str, Any]] = []
    for stage, manifest_path in zip(stages, manifest_paths):
        states, actions, metadata = _load_dataset(
            manifest_path,
            purpose="support_selection",
        )
        evaluation_sets.append((stage, states, actions))
        metadata["stage"] = stage
        evaluation_metadata.append(metadata)

    dense_results = [
        _dense_fit_diagnostic(
            search_states,
            search_actions,
            evaluation_sets,
            degree=int(degree),
            output_dir=destination,
        )
        for degree in dense_degrees
    ]

    from stable_baselines3 import SAC

    model = SAC.load(checkpoint, device=device)
    grid_x_count, grid_y_count = grid_shape
    grid = _grid_diagnostic(
        model,
        x_count=int(grid_x_count),
        y_count=int(grid_y_count),
    )
    initial_rollout_result = _rollout_diagnostic(
        model,
        reset_mode="initial_disk",
        episode_count=int(initial_rollouts),
        seed_base=DX1_INITIAL_SEED_BASE,
    )
    uniform_rollout_result = _rollout_diagnostic(
        model,
        reset_mode="uniform_psi",
        episode_count=int(uniform_rollouts),
        seed_base=DX1_UNIFORM_SEED_BASE,
    )
    all_dense_failed = not any(
        bool(row["passes_old_pb1_gate"]) for row in dense_results
    )
    report = {
        "schema_version": 1,
        "kind": "dx1_pb1_structural_mismatch_diagnostic",
        "decision_id": DX1_DECISION_ID,
        "status": "diagnosed",
        "claim_level": "diagnostic_only",
        "selection_manifest": {
            "path": str(selection_path),
            "sha256": file_sha256(selection_path),
        },
        "selected_checkpoint": {
            "path": str(checkpoint),
            "sha256": selected_checkpoint_hash,
            "training_seed": int(checkpoint_manifest["seed"]),
        },
        "pb1_manifests": [
            {
                "stage": stage,
                "path": str(path),
                "sha256": file_sha256(path),
                "status": payload["status"],
                "failure_reason": payload["failure_reason"],
            }
            for stage, path, payload in zip(
                stages,
                manifest_paths,
                stage_payloads,
            )
        ],
        "diagnostic_contract": {
            "dense_degrees": [int(value) for value in dense_degrees],
            "dense_fit_dataset": search_metadata,
            "dense_evaluation_datasets": evaluation_metadata,
            "old_pb1_gate_mps": PAC_MAX_ERROR_MPS,
            "grid_shape": [int(grid_x_count), int(grid_y_count)],
            "initial_rollouts": int(initial_rollouts),
            "uniform_rollouts": int(uniform_rollouts),
            "controller_output_allowed": False,
            "certificate_feedback_allowed": False,
        },
        "dense_polynomial_results": dense_results,
        "all_dense_degrees_failed_old_pb1_gate": all_dense_failed,
        "reference_policy_grid": grid,
        "reference_policy_rollouts": {
            "initial_disk": initial_rollout_result,
            "uniform_psi": uniform_rollout_result,
        },
        "interpretation_boundary": {
            "does_not_change_v1_selection": True,
            "does_not_establish_pac_claim": True,
            "does_not_validate_controller": True,
            "does_not_establish_qsbc": True,
            "not_a_safety_claim": True,
        },
    }
    report["report_content_sha256"] = content_sha256(report)
    report_path = destination / "diagnostic_report.json"
    _write_json(report_path, report)
    return report_path
