"""Predeclared multi-seed Gazebo Classic road-CBF validation."""

from __future__ import annotations

from collections import Counter
from dataclasses import dataclass
import json
from math import isfinite
from pathlib import Path
from time import perf_counter
from typing import Any, Mapping

import yaml

from cbf_rover.adapters.exact.road_contract import (
    canonical_content_sha256,
    file_sha256,
    generate_seeded_road_scenario,
    obstacle_plan_payload,
)

from .differential_road import simulate_gazebo_differential_road_cbf
from .differential_road_contract import (
    GazeboDifferentialRoadContract,
    load_gazebo_differential_road_contract,
)
from .replay_validation import (
    _manifest_content_hash_valid,
    _referenced_file_hashes_valid,
    _terminal_state,
)


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


def _jsonl_line(payload: Mapping[str, Any]) -> str:
    return (
        json.dumps(
            payload,
            sort_keys=True,
            separators=(",", ":"),
            ensure_ascii=False,
            allow_nan=False,
        )
        + "\n"
    )


def _sha256(value: object, name: str) -> str:
    digest = str(value).lower()
    if len(digest) != 64 or any(
        character not in "0123456789abcdef" for character in digest
    ):
        raise ValueError(f"{name} must be a SHA-256 digest")
    return digest


@dataclass(frozen=True, slots=True)
class GazeboSeedEntry:
    lane_pattern: str
    seed: int

    def __post_init__(self) -> None:
        if self.seed < 0:
            raise ValueError("Gazebo batch seeds must be nonnegative")
        if not self.lane_pattern or any(
            value not in {"0", "1"} for value in self.lane_pattern
        ):
            raise ValueError("lane patterns must be nonempty binary strings")


@dataclass(frozen=True, slots=True)
class GazeboSeedCatalog:
    name: str
    purpose: str
    eligible_for_final_empirical_claim: bool
    source_episode_metrics_sha256: str
    entries: tuple[GazeboSeedEntry, ...]

    def __post_init__(self) -> None:
        if not self.name.strip() or not self.purpose.strip():
            raise ValueError("Gazebo catalog identity must not be empty")
        object.__setattr__(
            self,
            "source_episode_metrics_sha256",
            _sha256(
                self.source_episode_metrics_sha256,
                "source episode metrics hash",
            ),
        )
        if not self.entries:
            raise ValueError("Gazebo seed catalog must not be empty")
        seeds = tuple(entry.seed for entry in self.entries)
        patterns = tuple(entry.lane_pattern for entry in self.entries)
        if len(set(seeds)) != len(seeds):
            raise ValueError("Gazebo seed catalog seeds must be unique")
        if len(set(patterns)) != len(patterns):
            raise ValueError("Gazebo seed catalog lane patterns must be unique")
        if patterns != tuple(sorted(patterns)):
            raise ValueError("Gazebo seed entries must use ascending lane pattern order")

    def overlaps(self, other: "GazeboSeedCatalog") -> bool:
        return bool(
            {entry.seed for entry in self.entries}
            & {entry.seed for entry in other.entries}
        )


@dataclass(frozen=True, slots=True)
class GazeboBatchAcceptanceSpec:
    required_success_rate: float
    maximum_collision_episodes: int
    maximum_road_departure_episodes: int
    maximum_timeout_episodes: int
    minimum_lifted_barrier_m: float
    minimum_cbf_constraint_residual_mps: float
    minimum_measured_contraction_residual_m: float
    maximum_mapping_residual_mps: float
    maximum_position_tracking_residual_m: float
    maximum_heading_tracking_residual_rad: float
    maximum_absolute_roll_rad: float
    maximum_absolute_pitch_rad: float
    maximum_linear_speed_mps: float
    maximum_absolute_angular_speed_radps: float
    minimum_achieved_realtime_factor: float
    maximum_achieved_realtime_factor: float
    required_lane_pattern_count: int
    require_no_post_projection_command_clipping: bool
    require_live_display_each_episode: bool
    require_wall_clock_pacing_each_episode: bool

    def __post_init__(self) -> None:
        if not 0.0 <= self.required_success_rate <= 1.0:
            raise ValueError("required success rate must lie in [0,1]")
        counts = (
            self.maximum_collision_episodes,
            self.maximum_road_departure_episodes,
            self.maximum_timeout_episodes,
            self.required_lane_pattern_count,
        )
        if any(value < 0 for value in counts):
            raise ValueError("Gazebo batch acceptance counts must be nonnegative")
        thresholds = (
            self.minimum_lifted_barrier_m,
            self.minimum_cbf_constraint_residual_mps,
            self.minimum_measured_contraction_residual_m,
            self.maximum_mapping_residual_mps,
            self.maximum_position_tracking_residual_m,
            self.maximum_heading_tracking_residual_rad,
            self.maximum_absolute_roll_rad,
            self.maximum_absolute_pitch_rad,
            self.maximum_linear_speed_mps,
            self.maximum_absolute_angular_speed_radps,
            self.minimum_achieved_realtime_factor,
            self.maximum_achieved_realtime_factor,
        )
        if not all(isfinite(value) for value in thresholds):
            raise ValueError("Gazebo batch acceptance thresholds must be finite")
        if any(
            value < 0.0
            for value in (
                self.maximum_mapping_residual_mps,
                self.maximum_position_tracking_residual_m,
                self.maximum_heading_tracking_residual_rad,
                self.maximum_absolute_roll_rad,
                self.maximum_absolute_pitch_rad,
                self.minimum_achieved_realtime_factor,
            )
        ):
            raise ValueError("Gazebo batch upper bounds must be nonnegative")
        if (
            self.maximum_linear_speed_mps <= 0.0
            or self.maximum_absolute_angular_speed_radps <= 0.0
            or self.maximum_achieved_realtime_factor
            < self.minimum_achieved_realtime_factor
        ):
            raise ValueError("Gazebo batch acceptance bounds are invalid")


@dataclass(frozen=True, slots=True)
class GazeboRoadBatchContract:
    experiment_id: str
    config_path: Path
    config_file_sha256: str
    gazebo_episode_config_path: Path
    source_experiment_id: str
    selection_rule: str
    selection_uses_metrics_values: bool
    catalogs: tuple[GazeboSeedCatalog, ...]
    progress_interval_episodes: int
    hard_timeout_s: float
    acceptance: GazeboBatchAcceptanceSpec
    replay_config_path: Path
    replay_selection_metric: str

    def __post_init__(self) -> None:
        if not self.experiment_id.strip() or not self.source_experiment_id.strip():
            raise ValueError("Gazebo batch experiment identity must not be empty")
        if not self.gazebo_episode_config_path.is_file():
            raise ValueError("bound Gazebo episode config does not exist")
        if not self.replay_config_path.is_file():
            raise ValueError("bound Gazebo replay config does not exist")
        if self.selection_rule != "first_ascending_seed_per_lane_pattern":
            raise ValueError("unexpected Gazebo seed selection rule")
        if self.selection_uses_metrics_values:
            raise ValueError("Gazebo seed selection must not use outcome metrics")
        if not self.catalogs:
            raise ValueError("at least one Gazebo seed catalog is required")
        names = tuple(catalog.name for catalog in self.catalogs)
        if len(set(names)) != len(names):
            raise ValueError("Gazebo seed catalog names must be unique")
        for index, first in enumerate(self.catalogs):
            for second in self.catalogs[index + 1 :]:
                if first.overlaps(second):
                    raise ValueError("Gazebo seed catalogs must not overlap")
        if self.progress_interval_episodes <= 0:
            raise ValueError("Gazebo batch progress interval must be positive")
        if not isfinite(self.hard_timeout_s) or self.hard_timeout_s <= 0.0:
            raise ValueError("Gazebo batch hard timeout must be positive")
        if self.replay_selection_metric not in {
            "maximum_position_tracking_residual_m",
            "minimum_lifted_certificate_barrier_m",
        }:
            raise ValueError("unexpected Gazebo replay selection metric")

    def catalog(self, name: str) -> GazeboSeedCatalog:
        for catalog in self.catalogs:
            if catalog.name == name:
                return catalog
        raise ValueError(f"unknown Gazebo seed catalog: {name}")


def load_gazebo_road_batch_contract(
    path: str | Path,
) -> GazeboRoadBatchContract:
    config_path = Path(path).resolve()
    payload = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    if not isinstance(payload, Mapping):
        raise ValueError("Gazebo batch config must be a mapping")
    if (
        payload.get("kind")
        != "dynamic_road_gazebo_classic_multiseed_experiment"
    ):
        raise ValueError("unexpected Gazebo batch config kind")

    provenance = payload["catalog_provenance"]
    catalogs = tuple(
        GazeboSeedCatalog(
            name=str(name),
            purpose=str(spec["purpose"]),
            eligible_for_final_empirical_claim=bool(
                spec["eligible_for_final_empirical_claim"]
            ),
            source_episode_metrics_sha256=str(
                spec["source_episode_metrics_sha256"]
            ),
            entries=tuple(
                GazeboSeedEntry(
                    lane_pattern=str(entry["lane_pattern"]),
                    seed=int(entry["seed"]),
                )
                for entry in spec["entries"]
            ),
        )
        for name, spec in payload["seed_catalogs"].items()
    )
    execution = payload["execution"]
    expected_execution = {
        "order": "ascending_lane_pattern",
        "parallelism": 1,
        "per_episode_wall_clock_pacing": True,
        "live_display_during_episode": True,
        "render_during_batch": False,
        "silent_retry_allowed": False,
    }
    for name, expected in expected_execution.items():
        if execution.get(name) != expected:
            raise ValueError(f"Gazebo batch execution freezes {name}={expected!r}")
    acceptance = payload["acceptance"]
    replay = payload["independent_replay"]
    return GazeboRoadBatchContract(
        experiment_id=str(payload["experiment"]["id"]),
        config_path=config_path,
        config_file_sha256=file_sha256(config_path),
        gazebo_episode_config_path=(
            config_path.parent
            / str(payload["bindings"]["gazebo_episode_config"])
        ).resolve(),
        source_experiment_id=str(provenance["source_experiment_id"]),
        selection_rule=str(provenance["selection_rule"]),
        selection_uses_metrics_values=bool(
            provenance["selection_uses_metrics_values"]
        ),
        catalogs=catalogs,
        progress_interval_episodes=int(
            execution["progress_interval_episodes"]
        ),
        hard_timeout_s=float(execution["hard_timeout_s"]),
        acceptance=GazeboBatchAcceptanceSpec(
            required_success_rate=float(
                acceptance["required_success_rate"]
            ),
            maximum_collision_episodes=int(
                acceptance["maximum_collision_episodes"]
            ),
            maximum_road_departure_episodes=int(
                acceptance["maximum_road_departure_episodes"]
            ),
            maximum_timeout_episodes=int(
                acceptance["maximum_timeout_episodes"]
            ),
            minimum_lifted_barrier_m=float(
                acceptance["minimum_lifted_certificate_barrier_m"]
            ),
            minimum_cbf_constraint_residual_mps=float(
                acceptance["minimum_cbf_constraint_residual_mps"]
            ),
            minimum_measured_contraction_residual_m=float(
                acceptance["minimum_measured_contraction_residual_m"]
            ),
            maximum_mapping_residual_mps=float(
                acceptance["maximum_mapping_residual_mps"]
            ),
            maximum_position_tracking_residual_m=float(
                acceptance["maximum_position_tracking_residual_m"]
            ),
            maximum_heading_tracking_residual_rad=float(
                acceptance["maximum_heading_tracking_residual_rad"]
            ),
            maximum_absolute_roll_rad=float(
                acceptance["maximum_absolute_roll_rad"]
            ),
            maximum_absolute_pitch_rad=float(
                acceptance["maximum_absolute_pitch_rad"]
            ),
            maximum_linear_speed_mps=float(
                acceptance["maximum_linear_speed_mps"]
            ),
            maximum_absolute_angular_speed_radps=float(
                acceptance["maximum_absolute_angular_speed_radps"]
            ),
            minimum_achieved_realtime_factor=float(
                acceptance["minimum_achieved_realtime_factor"]
            ),
            maximum_achieved_realtime_factor=float(
                acceptance["maximum_achieved_realtime_factor"]
            ),
            required_lane_pattern_count=int(
                acceptance["required_lane_pattern_count"]
            ),
            require_no_post_projection_command_clipping=bool(
                acceptance[
                    "require_no_post_projection_command_clipping"
                ]
            ),
            require_live_display_each_episode=bool(
                acceptance["require_live_display_each_episode"]
            ),
            require_wall_clock_pacing_each_episode=bool(
                acceptance["require_wall_clock_pacing_each_episode"]
            ),
        ),
        replay_config_path=(
            config_path.parent / str(replay["replay_config"])
        ).resolve(),
        replay_selection_metric=str(replay["selection_metric"]),
    )


def _lane_pattern(contract: GazeboDifferentialRoadContract, seed: int) -> str:
    scenario = generate_seeded_road_scenario(contract.base.base, seed=seed)
    return "".join(str(obstacle.lane_index) for obstacle in scenario.obstacles)


def _live_display_file_hash_valid(
    manifest_path: Path,
    manifest: Mapping[str, Any],
) -> bool:
    live = manifest.get("live_display")
    if not isinstance(live, Mapping):
        return False
    candidate = (
        manifest_path.parent
        / str(live["directory"])
        / str(live["manifest_file"])
    ).resolve()
    expected_parent = (
        manifest_path.parent / str(live["directory"])
    ).resolve()
    return (
        candidate.parent == expected_parent
        and candidate.is_file()
        and file_sha256(candidate) == live["manifest_file_sha256"]
    )


def gazebo_batch_episode_record(
    *,
    manifest_path: str | Path,
    expected_entry: GazeboSeedEntry,
    contract: GazeboDifferentialRoadContract,
    relative_manifest_path: str,
) -> dict[str, Any]:
    """Verify and reduce one completed bound Gazebo run."""

    path = Path(manifest_path).resolve()
    manifest = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(manifest, Mapping):
        raise ValueError(f"Gazebo run manifest is not an object: {path}")
    if manifest.get("kind") != "dynamic_road_gazebo_classic_cbf_run":
        raise ValueError(f"unexpected Gazebo run manifest kind: {path}")
    referenced_valid, _ = _referenced_file_hashes_valid(path, manifest)
    terminal = _terminal_state(path, manifest)
    scenario = generate_seeded_road_scenario(
        contract.base.base,
        seed=expected_entry.seed,
    )
    expected_plan = obstacle_plan_payload(
        scenario=scenario,
        seed=expected_entry.seed,
    )
    telemetry = manifest["telemetry"]
    run_gates = manifest["acceptance"]["gates"]
    identity_gates = {
        "manifest_content_hash": _manifest_content_hash_valid(manifest),
        "referenced_file_hashes": referenced_valid,
        "live_display_manifest_hash": _live_display_file_hash_valid(
            path, manifest
        ),
        "experiment_identity": (
            manifest["experiment_id"] == contract.experiment_id
        ),
        "config_identity": (
            manifest["contract"]["config_file_sha256"]
            == contract.config_file_sha256
        ),
        "seed_identity": (
            int(manifest["scenario"]["seed"]) == expected_entry.seed
        ),
        "lane_pattern_identity": (
            _lane_pattern(contract, expected_entry.seed)
            == expected_entry.lane_pattern
        ),
        "obstacle_plan_identity": (
            manifest["scenario"]["obstacle_plan_content_sha256"]
            == expected_plan["content_sha256"]
        ),
        "run_status": manifest["status"] == "provisional_success",
        "termination": manifest["termination_reason"] == "goal_reached",
        "all_single_run_gates": all(
            bool(value) for value in run_gates.values()
        ),
    }
    failure_reasons = [
        f"identity:{name}"
        for name, passed in identity_gates.items()
        if not passed
    ]
    failure_reasons.extend(
        f"single_run:{name}"
        for name, passed in run_gates.items()
        if not bool(passed)
    )
    live_manifest = manifest["live_display"]["manifest"]
    startup = manifest["startup"]
    record: dict[str, Any] = {
        "schema_version": 1,
        "kind": "dynamic_road_gazebo_multiseed_episode",
        "seed": expected_entry.seed,
        "lane_pattern": expected_entry.lane_pattern,
        "status": "passed" if not failure_reasons else "failed",
        "failure_reasons": failure_reasons,
        "termination_reason": str(manifest["termination_reason"]),
        "run_manifest": relative_manifest_path,
        "run_manifest_file_sha256": file_sha256(path),
        "run_manifest_content_sha256": str(
            manifest["manifest_content_sha256"]
        ),
        "trace_file_sha256": str(telemetry["trace_file_sha256"]),
        "obstacle_plan_content_sha256": str(
            manifest["scenario"]["obstacle_plan_content_sha256"]
        ),
        "single_run_gate_count": len(run_gates),
        "single_run_passed_gate_count": sum(
            bool(value) for value in run_gates.values()
        ),
        "transition_count": int(telemetry["transition_count"]),
        "gazebo_simulated_duration_s": float(
            telemetry["gazebo_simulated_duration_s"]
        ),
        "wall_elapsed_s": float(manifest["wall_elapsed_s"]),
        "achieved_realtime_factor": float(
            manifest["wall_clock_pacing"]["achieved_realtime_factor"]
        ),
        "wall_clock_paced": bool(manifest["wall_clock_paced"]),
        "startup_attempt_count": int(startup["attempt_count"]),
        "selected_startup_attempt": int(startup["selected_attempt"]),
        "control_retries_after_initial_state": int(
            startup["control_retries_after_initial_state"]
        ),
        "live_display_update_count": int(live_manifest["update_count"]),
        "live_display_status": str(live_manifest["status"]),
        "live_display_observer_only": bool(
            not live_manifest["semantics"][
                "display_sink_control_feedback"
            ]
            and not live_manifest["semantics"][
                "display_sink_action_modification"
            ]
        ),
        "activated_obstacle_count": int(
            telemetry["activated_obstacle_count"]
        ),
        "collision_count": int(telemetry["collision_count"]),
        "road_departure_count": int(
            telemetry["road_departure_count"]
        ),
        "minimum_lifted_certificate_barrier_m": float(
            telemetry["minimum_lifted_certificate_barrier_m"]
        ),
        "minimum_cbf_constraint_residual_mps": float(
            telemetry["minimum_cbf_constraint_residual_mps"]
        ),
        "minimum_measured_contraction_residual_m": float(
            telemetry["minimum_realized_contraction_residual_m"]
        ),
        "minimum_planned_physical_clearance_m": float(
            telemetry["minimum_planned_physical_clearance_m"]
        ),
        "minimum_lifted_physical_clearance_m": float(
            telemetry["minimum_lifted_physical_clearance_m"]
        ),
        "maximum_mapping_residual_mps": float(
            telemetry["maximum_mapping_residual_mps"]
        ),
        "maximum_position_tracking_residual_m": float(
            telemetry["maximum_position_tracking_residual_m"]
        ),
        "maximum_heading_tracking_residual_rad": float(
            telemetry["maximum_absolute_heading_tracking_residual_rad"]
        ),
        "maximum_absolute_roll_rad": float(
            telemetry["maximum_absolute_roll_rad"]
        ),
        "maximum_absolute_pitch_rad": float(
            telemetry["maximum_absolute_pitch_rad"]
        ),
        "maximum_linear_speed_mps": float(
            telemetry["maximum_linear_speed_mps"]
        ),
        "maximum_absolute_angular_speed_radps": float(
            telemetry["maximum_absolute_angular_speed_radps"]
        ),
        "command_saturation_step_count": int(
            telemetry["command_saturation_step_count"]
        ),
        "final_state": terminal,
    }
    record["record_content_sha256"] = canonical_content_sha256(record)
    return record


def _worst(
    records: list[Mapping[str, Any]],
    metric: str,
    *,
    minimum: bool,
) -> dict[str, Any]:
    selector = min if minimum else max
    record = selector(
        records,
        key=lambda item: (float(item[metric]), int(item["seed"])),
    )
    return {
        "seed": int(record["seed"]),
        "lane_pattern": str(record["lane_pattern"]),
        "value": float(record[metric]),
        "run_manifest": str(record["run_manifest"]),
        "run_manifest_file_sha256": str(
            record["run_manifest_file_sha256"]
        ),
    }


def _aggregate_records(
    records: list[Mapping[str, Any]],
    *,
    acceptance: GazeboBatchAcceptanceSpec,
) -> tuple[dict[str, Any], dict[str, bool]]:
    count = len(records)
    passed = sum(record["status"] == "passed" for record in records)
    success_rate = passed / count
    lane_patterns = Counter(str(record["lane_pattern"]) for record in records)
    failure_reasons = Counter(
        str(reason)
        for record in records
        for reason in record["failure_reasons"]
    )
    collision_episodes = sum(
        int(record["collision_count"]) > 0 for record in records
    )
    departure_episodes = sum(
        int(record["road_departure_count"]) > 0 for record in records
    )
    timeout_episodes = sum(
        record["termination_reason"] == "timeout" for record in records
    )
    minimum_barrier = min(
        float(record["minimum_lifted_certificate_barrier_m"])
        for record in records
    )
    minimum_residual = min(
        float(record["minimum_cbf_constraint_residual_mps"])
        for record in records
    )
    minimum_contraction = min(
        float(record["minimum_measured_contraction_residual_m"])
        for record in records
    )
    maximum_mapping = max(
        float(record["maximum_mapping_residual_mps"])
        for record in records
    )
    maximum_position = max(
        float(record["maximum_position_tracking_residual_m"])
        for record in records
    )
    maximum_heading = max(
        float(record["maximum_heading_tracking_residual_rad"])
        for record in records
    )
    maximum_roll = max(
        float(record["maximum_absolute_roll_rad"]) for record in records
    )
    maximum_pitch = max(
        float(record["maximum_absolute_pitch_rad"]) for record in records
    )
    maximum_linear = max(
        float(record["maximum_linear_speed_mps"]) for record in records
    )
    maximum_angular = max(
        float(record["maximum_absolute_angular_speed_radps"])
        for record in records
    )
    minimum_realtime = min(
        float(record["achieved_realtime_factor"]) for record in records
    )
    maximum_realtime = max(
        float(record["achieved_realtime_factor"]) for record in records
    )
    gates = {
        "success_rate": success_rate >= acceptance.required_success_rate,
        "collision_episodes": (
            collision_episodes <= acceptance.maximum_collision_episodes
        ),
        "road_departure_episodes": (
            departure_episodes
            <= acceptance.maximum_road_departure_episodes
        ),
        "timeout_episodes": (
            timeout_episodes <= acceptance.maximum_timeout_episodes
        ),
        "minimum_lifted_barrier": (
            minimum_barrier >= acceptance.minimum_lifted_barrier_m
        ),
        "minimum_cbf_constraint_residual": (
            minimum_residual
            >= acceptance.minimum_cbf_constraint_residual_mps
        ),
        "minimum_measured_contraction_residual": (
            minimum_contraction
            >= acceptance.minimum_measured_contraction_residual_m
        ),
        "maximum_mapping_residual": (
            maximum_mapping <= acceptance.maximum_mapping_residual_mps
        ),
        "maximum_position_tracking_residual": (
            maximum_position
            <= acceptance.maximum_position_tracking_residual_m
        ),
        "maximum_heading_tracking_residual": (
            maximum_heading
            <= acceptance.maximum_heading_tracking_residual_rad
        ),
        "maximum_roll": maximum_roll <= acceptance.maximum_absolute_roll_rad,
        "maximum_pitch": (
            maximum_pitch <= acceptance.maximum_absolute_pitch_rad
        ),
        "maximum_linear_speed": (
            maximum_linear <= acceptance.maximum_linear_speed_mps
        ),
        "maximum_absolute_angular_speed": (
            maximum_angular
            <= acceptance.maximum_absolute_angular_speed_radps
        ),
        "realtime_factor_lower": (
            minimum_realtime
            >= acceptance.minimum_achieved_realtime_factor
        ),
        "realtime_factor_upper": (
            maximum_realtime
            <= acceptance.maximum_achieved_realtime_factor
        ),
        "lane_pattern_coverage": (
            len(lane_patterns) >= acceptance.required_lane_pattern_count
        ),
        "no_post_projection_command_clipping": all(
            int(record["command_saturation_step_count"]) == 0
            for record in records
        )
        if acceptance.require_no_post_projection_command_clipping
        else True,
        "live_display_each_episode": all(
            record["live_display_status"] == "completed"
            and bool(record["live_display_observer_only"])
            for record in records
        )
        if acceptance.require_live_display_each_episode
        else True,
        "wall_clock_pacing_each_episode": all(
            bool(record["wall_clock_paced"]) for record in records
        )
        if acceptance.require_wall_clock_pacing_each_episode
        else True,
        "no_control_retries_after_initial_state": all(
            int(record["control_retries_after_initial_state"]) == 0
            for record in records
        ),
    }
    metrics = {
        "episode_count": count,
        "passed_episode_count": passed,
        "failed_episode_count": count - passed,
        "success_rate": success_rate,
        "failure_reason_counts": dict(sorted(failure_reasons.items())),
        "lane_pattern_count": len(lane_patterns),
        "lane_pattern_counts": dict(sorted(lane_patterns.items())),
        "collision_episode_count": collision_episodes,
        "road_departure_episode_count": departure_episodes,
        "timeout_episode_count": timeout_episodes,
        "minimum_lifted_certificate_barrier_m": minimum_barrier,
        "minimum_cbf_constraint_residual_mps": minimum_residual,
        "minimum_measured_contraction_residual_m": minimum_contraction,
        "minimum_planned_physical_clearance_m": min(
            float(record["minimum_planned_physical_clearance_m"])
            for record in records
        ),
        "minimum_lifted_physical_clearance_m": min(
            float(record["minimum_lifted_physical_clearance_m"])
            for record in records
        ),
        "maximum_mapping_residual_mps": maximum_mapping,
        "maximum_position_tracking_residual_m": maximum_position,
        "maximum_heading_tracking_residual_rad": maximum_heading,
        "maximum_absolute_roll_rad": maximum_roll,
        "maximum_absolute_pitch_rad": maximum_pitch,
        "maximum_linear_speed_mps": maximum_linear,
        "maximum_absolute_angular_speed_radps": maximum_angular,
        "minimum_achieved_realtime_factor": minimum_realtime,
        "maximum_achieved_realtime_factor": maximum_realtime,
        "total_live_display_updates": sum(
            int(record["live_display_update_count"]) for record in records
        ),
        "total_startup_attempts": sum(
            int(record["startup_attempt_count"]) for record in records
        ),
        "episodes_with_startup_retry": sum(
            int(record["startup_attempt_count"]) > 1 for record in records
        ),
        "maximum_startup_attempt_count": max(
            int(record["startup_attempt_count"]) for record in records
        ),
        "worst_minimum_barrier": _worst(
            records,
            "minimum_lifted_certificate_barrier_m",
            minimum=True,
        ),
        "worst_measured_contraction": _worst(
            records,
            "minimum_measured_contraction_residual_m",
            minimum=True,
        ),
        "largest_position_tracking_residual": _worst(
            records,
            "maximum_position_tracking_residual_m",
            minimum=False,
        ),
        "largest_heading_tracking_residual": _worst(
            records,
            "maximum_heading_tracking_residual_rad",
            minimum=False,
        ),
        "slowest_realtime_factor": _worst(
            records,
            "achieved_realtime_factor",
            minimum=True,
        ),
    }
    return metrics, gates


def run_gazebo_road_batch(
    *,
    batch_config_path: str | Path,
    catalog_name: str,
    bridge_binary: str | Path,
    output_dir: str | Path,
    gzserver_binary: str = "gzserver",
) -> Path:
    """Run one frozen catalog without retrying a control trajectory."""

    batch = load_gazebo_road_batch_contract(batch_config_path)
    catalog = batch.catalog(catalog_name)
    contract = load_gazebo_differential_road_contract(
        batch.gazebo_episode_config_path
    )
    if contract.live_display is None or not contract.live_display.enabled:
        raise ValueError("Gazebo batch requires the bound live display")
    if not contract.live_display.wall_clock_paced:
        raise ValueError("Gazebo batch requires wall-clock-paced episodes")
    if len(catalog.entries) < batch.acceptance.required_lane_pattern_count:
        raise ValueError("Gazebo catalog cannot satisfy lane-pattern coverage")
    for entry in catalog.entries:
        if _lane_pattern(contract, entry.seed) != entry.lane_pattern:
            raise ValueError(
                f"frozen lane pattern does not match seed {entry.seed}"
            )

    destination = Path(output_dir).resolve()
    destination.mkdir(parents=True, exist_ok=False)
    partial_path = destination / "episode_metrics.partial.jsonl"
    metrics_path = destination / "episode_metrics.jsonl"
    progress_path = destination / "progress.json"
    records: list[dict[str, Any]] = []
    start_time = perf_counter()
    _write_json(
        progress_path,
        {
            "schema_version": 1,
            "kind": "dynamic_road_gazebo_multiseed_progress",
            "status": "running",
            "catalog": catalog.name,
            "processed": 0,
            "total": len(catalog.entries),
            "passed": 0,
            "failed": 0,
        },
    )
    try:
        with partial_path.open("x", encoding="utf-8", newline="\n") as stream:
            for index, entry in enumerate(catalog.entries, start=1):
                relative_directory = Path("episodes") / (
                    f"lane_{entry.lane_pattern}_seed_{entry.seed}"
                )
                episode_dir = destination / relative_directory
                manifest_path = simulate_gazebo_differential_road_cbf(
                    contract=contract,
                    bridge_binary=bridge_binary,
                    output_dir=episode_dir,
                    seed=entry.seed,
                    emit_console=False,
                    render_gif=False,
                    gzserver_binary=gzserver_binary,
                )
                record = gazebo_batch_episode_record(
                    manifest_path=manifest_path,
                    expected_entry=entry,
                    contract=contract,
                    relative_manifest_path=str(
                        relative_directory / manifest_path.name
                    ).replace("\\", "/"),
                )
                records.append(record)
                stream.write(_jsonl_line(record))
                stream.flush()
                if (
                    index % batch.progress_interval_episodes == 0
                    or index == len(catalog.entries)
                    or record["status"] == "failed"
                ):
                    progress = {
                        "schema_version": 1,
                        "kind": "dynamic_road_gazebo_multiseed_progress",
                        "status": "running",
                        "catalog": catalog.name,
                        "processed": index,
                        "total": len(catalog.entries),
                        "passed": sum(
                            item["status"] == "passed" for item in records
                        ),
                        "failed": sum(
                            item["status"] == "failed" for item in records
                        ),
                        "last_seed": entry.seed,
                        "last_lane_pattern": entry.lane_pattern,
                        "elapsed_s": perf_counter() - start_time,
                    }
                    _write_json(progress_path, progress)
                    print(
                        json.dumps(
                            progress,
                            sort_keys=True,
                            separators=(",", ":"),
                            allow_nan=False,
                        ),
                        flush=True,
                    )
    except BaseException as error:
        _write_json(
            progress_path,
            {
                "schema_version": 1,
                "kind": "dynamic_road_gazebo_multiseed_progress",
                "status": "crashed",
                "catalog": catalog.name,
                "processed": len(records),
                "total": len(catalog.entries),
                "error_type": type(error).__name__,
                "error": str(error),
            },
        )
        raise

    partial_path.replace(metrics_path)
    metrics_sha256 = file_sha256(metrics_path)
    metrics, gates = _aggregate_records(
        records,
        acceptance=batch.acceptance,
    )
    passed = all(gates.values())
    status = (
        "diagnostic_complete"
        if not catalog.eligible_for_final_empirical_claim
        else ("provisional_success" if passed else "failed")
    )
    replay_selector = metrics["largest_position_tracking_residual"]
    if batch.replay_selection_metric == "minimum_lifted_certificate_barrier_m":
        replay_selector = metrics["worst_minimum_barrier"]
    summary: dict[str, Any] = {
        "schema_version": 1,
        "kind": "dynamic_road_gazebo_multiseed_summary",
        "experiment_id": batch.experiment_id,
        "status": status,
        "claim_boundary": {
            "gazebo_batch": "empirical_finite_catalog_only",
            "seed_selection": "scenario_lane_pattern_only",
            "sampling_probability_bound": "not_claimed",
            "q_sbc_probability_bound": "not_claimed",
            "tracking_error_bound": "empirical_envelope_only",
            "ros2_result": "not_claimed",
            "real_vehicle_claim": "forbidden",
        },
        "batch_config": {
            "path": str(batch.config_path),
            "file_sha256": batch.config_file_sha256,
        },
        "gazebo_episode_config": {
            "path": str(batch.gazebo_episode_config_path),
            "experiment_id": contract.experiment_id,
            "file_sha256": contract.config_file_sha256,
        },
        "catalog_provenance": {
            "source_experiment_id": batch.source_experiment_id,
            "source_episode_metrics_sha256": (
                catalog.source_episode_metrics_sha256
            ),
            "selection_rule": batch.selection_rule,
            "selection_uses_metrics_values": (
                batch.selection_uses_metrics_values
            ),
        },
        "catalog": {
            "name": catalog.name,
            "purpose": catalog.purpose,
            "eligible_for_final_empirical_claim": (
                catalog.eligible_for_final_empirical_claim
            ),
            "count": len(catalog.entries),
            "entries": [
                {
                    "lane_pattern": entry.lane_pattern,
                    "seed": entry.seed,
                }
                for entry in catalog.entries
            ],
        },
        "execution": {
            "order": "ascending_lane_pattern",
            "parallelism": 1,
            "silent_retry_allowed": False,
            "audited_pre_control_startup_attempt_limit": (
                contract.runtime.startup_attempt_limit
            ),
            "retry_after_initial_state_allowed": False,
            "per_episode_wall_clock_pacing": True,
            "live_display_during_episode": True,
            "render_during_batch": False,
            "wall_elapsed_s": perf_counter() - start_time,
        },
        "metrics": metrics,
        "acceptance": {"gates": gates},
        "independent_replay": {
            "status": "not_evaluated",
            "selection_metric": batch.replay_selection_metric,
            "selected_member": replay_selector,
            "replay_config": str(batch.replay_config_path),
            "replay_config_file_sha256": file_sha256(
                batch.replay_config_path
            ),
        },
        "artifacts": {
            "episode_metrics_file": metrics_path.name,
            "episode_metrics_file_sha256": metrics_sha256,
        },
    }
    summary["summary_content_sha256"] = canonical_content_sha256(summary)
    summary_path = destination / "batch_summary.json"
    _write_json(summary_path, summary)
    _write_json(
        progress_path,
        {
            "schema_version": 1,
            "kind": "dynamic_road_gazebo_multiseed_progress",
            "status": "completed",
            "catalog": catalog.name,
            "processed": len(catalog.entries),
            "total": len(catalog.entries),
            "passed": metrics["passed_episode_count"],
            "failed": metrics["failed_episode_count"],
            "summary": summary_path.name,
        },
    )
    return summary_path
