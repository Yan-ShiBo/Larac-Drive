"""Bounded repeatability validation for Gazebo Classic road-CBF runs."""

from __future__ import annotations

from dataclasses import dataclass
from math import hypot, isfinite
import json
from pathlib import Path
from typing import Any, Mapping, Sequence

import yaml

from cbf_rover.adapters.exact.road_contract import (
    canonical_content_sha256,
    file_sha256,
)
from cbf_rover.domain.differential_drive import wrap_heading


@dataclass(frozen=True, slots=True)
class GazeboReplayValidationSpec:
    config_path: Path
    config_file_sha256: str
    experiment_id: str
    bound_run_experiment_id: str
    required_run_count: int
    require_distinct_result_directories: bool
    require_valid_manifest_content_hash: bool
    require_valid_referenced_file_hashes: bool
    require_same_config_file_sha256: bool
    require_same_base_differential_config_file_sha256: bool
    require_same_scenario_id: bool
    require_same_seed: bool
    require_same_obstacle_plan_content_sha256: bool
    require_same_world_file_sha256: bool
    require_same_runtime_contract: bool
    require_each_run_provisional_success: bool
    require_each_run_all_gates: bool
    require_each_run_goal_reached: bool
    maximum_transition_count_difference: int
    maximum_simulated_duration_difference_s: float
    maximum_final_center_position_difference_m: float
    maximum_final_heading_difference_rad: float
    require_same_trace_sha256: bool

    def __post_init__(self) -> None:
        if not self.experiment_id.strip() or not self.bound_run_experiment_id.strip():
            raise ValueError("Gazebo replay experiment ids must not be empty")
        if self.required_run_count < 2:
            raise ValueError("Gazebo replay validation requires at least two runs")
        if self.maximum_transition_count_difference < 0:
            raise ValueError("transition-count difference must be nonnegative")
        for field_name in (
            "maximum_simulated_duration_difference_s",
            "maximum_final_center_position_difference_m",
            "maximum_final_heading_difference_rad",
        ):
            value = float(getattr(self, field_name))
            if not isfinite(value) or value < 0.0:
                raise ValueError(f"{field_name} must be finite and nonnegative")


def load_gazebo_replay_validation_spec(
    path: str | Path,
) -> GazeboReplayValidationSpec:
    config_path = Path(path).resolve()
    payload = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    if not isinstance(payload, Mapping):
        raise ValueError("Gazebo replay config must be a mapping")
    if payload.get("kind") != "dynamic_road_gazebo_classic_replay_validation_spec":
        raise ValueError("unexpected Gazebo replay config kind")
    experiment = payload["experiment"]
    inputs = payload["inputs"]
    identity = payload["identity"]
    acceptance = payload["acceptance"]
    trace_policy = payload["trace_policy"]
    return GazeboReplayValidationSpec(
        config_path=config_path,
        config_file_sha256=file_sha256(config_path),
        experiment_id=str(experiment["id"]),
        bound_run_experiment_id=str(experiment["bound_run_experiment_id"]),
        required_run_count=int(inputs["required_run_count"]),
        require_distinct_result_directories=bool(
            inputs["require_distinct_result_directories"]
        ),
        require_valid_manifest_content_hash=bool(
            inputs["require_valid_manifest_content_hash"]
        ),
        require_valid_referenced_file_hashes=bool(
            inputs["require_valid_referenced_file_hashes"]
        ),
        require_same_config_file_sha256=bool(
            identity["require_same_config_file_sha256"]
        ),
        require_same_base_differential_config_file_sha256=bool(
            identity["require_same_base_differential_config_file_sha256"]
        ),
        require_same_scenario_id=bool(identity["require_same_scenario_id"]),
        require_same_seed=bool(identity["require_same_seed"]),
        require_same_obstacle_plan_content_sha256=bool(
            identity["require_same_obstacle_plan_content_sha256"]
        ),
        require_same_world_file_sha256=bool(
            identity["require_same_world_file_sha256"]
        ),
        require_same_runtime_contract=bool(
            identity["require_same_runtime_contract"]
        ),
        require_each_run_provisional_success=bool(
            acceptance["require_each_run_provisional_success"]
        ),
        require_each_run_all_gates=bool(
            acceptance["require_each_run_all_gates"]
        ),
        require_each_run_goal_reached=bool(
            acceptance["require_each_run_goal_reached"]
        ),
        maximum_transition_count_difference=int(
            acceptance["maximum_transition_count_difference"]
        ),
        maximum_simulated_duration_difference_s=float(
            acceptance["maximum_simulated_duration_difference_s"]
        ),
        maximum_final_center_position_difference_m=float(
            acceptance["maximum_final_center_position_difference_m"]
        ),
        maximum_final_heading_difference_rad=float(
            acceptance["maximum_final_heading_difference_rad"]
        ),
        require_same_trace_sha256=bool(
            trace_policy["require_same_trace_sha256"]
        ),
    )


def _read_manifest(path: Path) -> dict[str, Any]:
    payload = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise ValueError(f"run manifest is not a JSON object: {path}")
    if payload.get("kind") != "dynamic_road_gazebo_classic_cbf_run":
        raise ValueError(f"unexpected Gazebo run manifest kind: {path}")
    return payload


def _manifest_content_hash_valid(payload: Mapping[str, Any]) -> bool:
    expected = payload.get("manifest_content_sha256")
    content = dict(payload)
    content.pop("manifest_content_sha256", None)
    return isinstance(expected, str) and canonical_content_sha256(content) == expected


def _referenced_file_hashes_valid(
    manifest_path: Path,
    payload: Mapping[str, Any],
) -> tuple[bool, dict[str, Any]]:
    directory = manifest_path.parent
    references = {
        "obstacle_plan": (
            payload["scenario"]["obstacle_plan_file"],
            payload["scenario"]["obstacle_plan_file_sha256"],
        ),
        "world": (
            payload["scenario"]["world_file"],
            payload["scenario"]["world_file_sha256"],
        ),
        "trace": (
            payload["telemetry"]["trace_file"],
            payload["telemetry"]["trace_file_sha256"],
        ),
    }
    startup = payload.get("startup")
    if isinstance(startup, Mapping):
        references["startup_attempts"] = (
            startup["attempts_file"],
            startup["attempts_file_sha256"],
        )
    facts: dict[str, Any] = {}
    valid = True
    for name, (relative_name, expected_sha256) in references.items():
        candidate = (directory / str(relative_name)).resolve()
        stays_within_run = candidate.parent == directory.resolve()
        exists = candidate.is_file()
        actual_sha256 = file_sha256(candidate) if exists and stays_within_run else None
        matches = exists and stays_within_run and actual_sha256 == expected_sha256
        facts[name] = {
            "path": str(candidate),
            "expected_sha256": str(expected_sha256),
            "actual_sha256": actual_sha256,
            "valid": matches,
        }
        valid = valid and matches
    return valid, facts


def _terminal_state(manifest_path: Path, payload: Mapping[str, Any]) -> dict[str, float]:
    trace_path = manifest_path.parent / str(payload["telemetry"]["trace_file"])
    last_row: Mapping[str, Any] | None = None
    with trace_path.open("r", encoding="utf-8") as stream:
        for line in stream:
            if line.strip():
                item = json.loads(line)
                if not isinstance(item, Mapping):
                    raise ValueError(f"trace row is not a JSON object: {trace_path}")
                last_row = item
    if last_row is None:
        raise ValueError(f"Gazebo trace is empty: {trace_path}")
    state = last_row["state"]
    return {
        "x_m": float(state["x_m"]),
        "y_m": float(state["y_m"]),
        "heading_rad": float(state["heading_rad"]),
    }


def _same(values: Sequence[Any]) -> bool:
    return all(value == values[0] for value in values[1:])


def _spread(values: Sequence[float]) -> float:
    return max(values) - min(values)


def _pairwise_terminal_spreads(
    states: Sequence[Mapping[str, float]],
) -> tuple[float, float]:
    maximum_position = 0.0
    maximum_heading = 0.0
    for index, first in enumerate(states):
        for second in states[index + 1 :]:
            maximum_position = max(
                maximum_position,
                hypot(
                    first["x_m"] - second["x_m"],
                    first["y_m"] - second["y_m"],
                ),
            )
            maximum_heading = max(
                maximum_heading,
                abs(
                    wrap_heading(
                        first["heading_rad"] - second["heading_rad"]
                    )
                ),
            )
    return maximum_position, maximum_heading


def validate_gazebo_replay(
    *,
    spec: GazeboReplayValidationSpec,
    run_manifest_paths: Sequence[str | Path],
    output_dir: str | Path,
) -> Path:
    """Validate two or more isolated Gazebo runs without rewriting them."""

    paths = tuple(Path(path).resolve() for path in run_manifest_paths)
    if len(paths) != spec.required_run_count:
        raise ValueError(
            f"expected {spec.required_run_count} run manifests, got {len(paths)}"
        )
    manifests = tuple(_read_manifest(path) for path in paths)
    integrity = tuple(
        _referenced_file_hashes_valid(path, manifest)
        for path, manifest in zip(paths, manifests, strict=True)
    )
    terminal_states = tuple(
        _terminal_state(path, manifest)
        for path, manifest in zip(paths, manifests, strict=True)
    )
    position_spread, heading_spread = _pairwise_terminal_spreads(
        terminal_states
    )
    transition_counts = tuple(
        int(manifest["telemetry"]["transition_count"])
        for manifest in manifests
    )
    simulated_durations = tuple(
        float(manifest["telemetry"]["gazebo_simulated_duration_s"])
        for manifest in manifests
    )
    run_gates = tuple(
        manifest["acceptance"]["gates"] for manifest in manifests
    )
    trace_hashes = tuple(
        str(manifest["telemetry"]["trace_file_sha256"])
        for manifest in manifests
    )
    runtime_contracts = tuple(manifest["runtime"] for manifest in manifests)
    gates = {
        "bound_run_experiment": all(
            manifest["experiment_id"] == spec.bound_run_experiment_id
            for manifest in manifests
        ),
        "distinct_result_directories": (
            len({path.parent for path in paths}) == len(paths)
            if spec.require_distinct_result_directories
            else True
        ),
        "manifest_content_hashes": (
            all(_manifest_content_hash_valid(manifest) for manifest in manifests)
            if spec.require_valid_manifest_content_hash
            else True
        ),
        "referenced_file_hashes": (
            all(result[0] for result in integrity)
            if spec.require_valid_referenced_file_hashes
            else True
        ),
        "same_config_file_sha256": (
            _same(
                tuple(
                    manifest["contract"]["config_file_sha256"]
                    for manifest in manifests
                )
            )
            if spec.require_same_config_file_sha256
            else True
        ),
        "same_base_differential_config_file_sha256": (
            _same(
                tuple(
                    manifest["contract"][
                        "base_differential_config_file_sha256"
                    ]
                    for manifest in manifests
                )
            )
            if spec.require_same_base_differential_config_file_sha256
            else True
        ),
        "same_scenario_id": (
            _same(
                tuple(
                    manifest["scenario"]["scenario_id"]
                    for manifest in manifests
                )
            )
            if spec.require_same_scenario_id
            else True
        ),
        "same_seed": (
            _same(
                tuple(
                    int(manifest["scenario"]["seed"])
                    for manifest in manifests
                )
            )
            if spec.require_same_seed
            else True
        ),
        "same_obstacle_plan_content_sha256": (
            _same(
                tuple(
                    manifest["scenario"]["obstacle_plan_content_sha256"]
                    for manifest in manifests
                )
            )
            if spec.require_same_obstacle_plan_content_sha256
            else True
        ),
        "same_world_file_sha256": (
            _same(
                tuple(
                    manifest["scenario"]["world_file_sha256"]
                    for manifest in manifests
                )
            )
            if spec.require_same_world_file_sha256
            else True
        ),
        "same_runtime_contract": (
            _same(runtime_contracts)
            if spec.require_same_runtime_contract
            else True
        ),
        "each_run_provisional_success": (
            all(
                manifest["status"] == "provisional_success"
                for manifest in manifests
            )
            if spec.require_each_run_provisional_success
            else True
        ),
        "each_run_all_gates": (
            all(all(bool(value) for value in gates.values()) for gates in run_gates)
            if spec.require_each_run_all_gates
            else True
        ),
        "each_run_goal_reached": (
            all(
                manifest["termination_reason"] == "goal_reached"
                for manifest in manifests
            )
            if spec.require_each_run_goal_reached
            else True
        ),
        "transition_count_spread": (
            int(_spread(tuple(float(value) for value in transition_counts)))
            <= spec.maximum_transition_count_difference
        ),
        "simulated_duration_spread": (
            _spread(simulated_durations)
            <= spec.maximum_simulated_duration_difference_s
        ),
        "final_center_position_spread": (
            position_spread
            <= spec.maximum_final_center_position_difference_m
        ),
        "final_heading_spread": (
            heading_spread <= spec.maximum_final_heading_difference_rad
        ),
        "trace_sha256": (
            _same(trace_hashes) if spec.require_same_trace_sha256 else True
        ),
    }
    records = [
        {
            "run_manifest": str(path),
            "run_manifest_file_sha256": file_sha256(path),
            "run_manifest_content_sha256": manifest[
                "manifest_content_sha256"
            ],
            "status": manifest["status"],
            "termination_reason": manifest["termination_reason"],
            "trace_file_sha256": manifest["telemetry"][
                "trace_file_sha256"
            ],
            "transition_count": int(
                manifest["telemetry"]["transition_count"]
            ),
            "gazebo_simulated_duration_s": float(
                manifest["telemetry"]["gazebo_simulated_duration_s"]
            ),
            "minimum_lifted_certificate_barrier_m": float(
                manifest["telemetry"][
                    "minimum_lifted_certificate_barrier_m"
                ]
            ),
            "minimum_cbf_constraint_residual_mps": float(
                manifest["telemetry"][
                    "minimum_cbf_constraint_residual_mps"
                ]
            ),
            "minimum_measured_contraction_residual_m": float(
                manifest["telemetry"][
                    "minimum_realized_contraction_residual_m"
                ]
            ),
            "maximum_position_tracking_residual_m": float(
                manifest["telemetry"][
                    "maximum_position_tracking_residual_m"
                ]
            ),
            "maximum_absolute_heading_tracking_residual_rad": float(
                manifest["telemetry"][
                    "maximum_absolute_heading_tracking_residual_rad"
                ]
            ),
            "terminal_state": terminal_state,
            "referenced_files": reference_facts,
        }
        for path, manifest, terminal_state, (_, reference_facts) in zip(
            paths,
            manifests,
            terminal_states,
            integrity,
            strict=True,
        )
    ]
    report: dict[str, Any] = {
        "schema_version": 1,
        "kind": "dynamic_road_gazebo_classic_replay_validation",
        "experiment_id": spec.experiment_id,
        "bound_run_experiment_id": spec.bound_run_experiment_id,
        "status": "accepted" if all(gates.values()) else "failed",
        "claim_boundary": {
            "replay_acceptance": (
                "bounded_empirical_repeatability_not_bitwise_determinism"
            ),
            "q_sbc_probability_bound": "not_claimed",
            "ros2_result": "not_claimed",
            "real_vehicle_claim": "forbidden",
        },
        "contract": {
            "config_path": str(spec.config_path),
            "config_file_sha256": spec.config_file_sha256,
            "required_run_count": spec.required_run_count,
            "require_same_trace_sha256": spec.require_same_trace_sha256,
            "maximum_transition_count_difference": (
                spec.maximum_transition_count_difference
            ),
            "maximum_simulated_duration_difference_s": (
                spec.maximum_simulated_duration_difference_s
            ),
            "maximum_final_center_position_difference_m": (
                spec.maximum_final_center_position_difference_m
            ),
            "maximum_final_heading_difference_rad": (
                spec.maximum_final_heading_difference_rad
            ),
        },
        "aggregate": {
            "transition_count_spread": int(
                _spread(tuple(float(value) for value in transition_counts))
            ),
            "simulated_duration_spread_s": _spread(simulated_durations),
            "final_center_position_spread_m": position_spread,
            "final_heading_spread_rad": heading_spread,
            "minimum_lifted_certificate_barrier_m": min(
                record["minimum_lifted_certificate_barrier_m"]
                for record in records
            ),
            "minimum_cbf_constraint_residual_mps": min(
                record["minimum_cbf_constraint_residual_mps"]
                for record in records
            ),
            "minimum_measured_contraction_residual_m": min(
                record["minimum_measured_contraction_residual_m"]
                for record in records
            ),
            "maximum_position_tracking_residual_m": max(
                record["maximum_position_tracking_residual_m"]
                for record in records
            ),
            "maximum_absolute_heading_tracking_residual_rad": max(
                record[
                    "maximum_absolute_heading_tracking_residual_rad"
                ]
                for record in records
            ),
            "byte_identical_trace_replay": _same(trace_hashes),
        },
        "gates": gates,
        "runs": records,
    }
    report["report_content_sha256"] = canonical_content_sha256(report)
    destination = Path(output_dir).resolve()
    destination.mkdir(parents=True, exist_ok=False)
    report_path = destination / "gazebo_replay_validation.json"
    temporary = report_path.with_name(f".{report_path.name}.tmp")
    temporary.write_text(
        json.dumps(
            report,
            indent=2,
            sort_keys=True,
            ensure_ascii=False,
            allow_nan=False,
        )
        + "\n",
        encoding="utf-8",
    )
    temporary.replace(report_path)
    return report_path
