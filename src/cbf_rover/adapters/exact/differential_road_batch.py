"""Predeclared multi-seed stress testing for the differential-road CBF."""

from __future__ import annotations

from collections import Counter
from dataclasses import dataclass
import json
from math import isfinite
from pathlib import Path
from time import perf_counter
from typing import Any, Mapping

import yaml

from cbf_rover.domain.dynamic_road import DynamicRoadScenario

from .differential_road import (
    differential_road_episode_metrics,
    run_differential_road_episode,
)
from .differential_road_contract import (
    DifferentialRoadContract,
    load_differential_road_contract,
)
from .dynamic_road import trace_rows_sha256, write_trace
from .dynamic_road_batch import SeedCatalog
from .road_contract import (
    canonical_content_sha256,
    file_sha256,
    generate_seeded_road_scenario,
    obstacle_plan_payload,
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


@dataclass(frozen=True, slots=True)
class DifferentialBatchAcceptanceSpec:
    required_success_rate: float
    maximum_collision_episodes: int
    maximum_road_departure_episodes: int
    maximum_timeout_episodes: int
    maximum_cbf_infeasible_episodes: int
    maximum_model_contract_breach_episodes: int
    minimum_lifted_barrier_m: float
    minimum_cbf_constraint_residual_mps: float
    minimum_realized_contraction_residual_m: float
    maximum_mapping_residual_mps: float
    maximum_linear_speed_mps: float
    maximum_absolute_angular_speed_radps: float
    required_lane_pattern_count: int
    require_no_post_projection_command_clipping: bool
    require_same_metrics_sha256_replay: bool

    def __post_init__(self) -> None:
        if not 0.0 <= self.required_success_rate <= 1.0:
            raise ValueError("required success rate must lie in [0,1]")
        counts = (
            self.maximum_collision_episodes,
            self.maximum_road_departure_episodes,
            self.maximum_timeout_episodes,
            self.maximum_cbf_infeasible_episodes,
            self.maximum_model_contract_breach_episodes,
            self.required_lane_pattern_count,
        )
        if any(value < 0 for value in counts):
            raise ValueError("batch acceptance counts must be nonnegative")
        thresholds = (
            self.minimum_lifted_barrier_m,
            self.minimum_cbf_constraint_residual_mps,
            self.minimum_realized_contraction_residual_m,
            self.maximum_mapping_residual_mps,
            self.maximum_linear_speed_mps,
            self.maximum_absolute_angular_speed_radps,
        )
        if not all(isfinite(value) for value in thresholds):
            raise ValueError("batch acceptance thresholds must be finite")
        if (
            self.maximum_mapping_residual_mps < 0.0
            or self.maximum_linear_speed_mps <= 0.0
            or self.maximum_absolute_angular_speed_radps <= 0.0
        ):
            raise ValueError("batch upper thresholds are invalid")


@dataclass(frozen=True, slots=True)
class DifferentialRoadBatchContract:
    experiment_id: str
    config_path: Path
    config_file_sha256: str
    differential_config_path: Path
    catalogs: tuple[SeedCatalog, ...]
    progress_interval_episodes: int
    maximum_saved_failure_traces: int
    acceptance: DifferentialBatchAcceptanceSpec
    confidence: float

    def __post_init__(self) -> None:
        if not self.experiment_id.strip():
            raise ValueError("batch experiment id must not be empty")
        if not self.differential_config_path.is_file():
            raise ValueError("bound differential config does not exist")
        if not self.catalogs:
            raise ValueError("at least one seed catalog is required")
        names = [catalog.name for catalog in self.catalogs]
        if len(set(names)) != len(names):
            raise ValueError("seed catalog names must be unique")
        for index, first in enumerate(self.catalogs):
            for second in self.catalogs[index + 1 :]:
                if first.overlaps(second):
                    raise ValueError("seed catalogs must not overlap")
        if self.progress_interval_episodes <= 0:
            raise ValueError("progress interval must be positive")
        if self.maximum_saved_failure_traces < 0:
            raise ValueError("failure trace limit must be nonnegative")
        if not 0.0 < self.confidence < 1.0:
            raise ValueError("confidence must lie in (0,1)")

    def catalog(self, name: str) -> SeedCatalog:
        for catalog in self.catalogs:
            if catalog.name == name:
                return catalog
        raise ValueError(f"unknown seed catalog: {name}")


def load_differential_road_batch_contract(
    path: str | Path,
) -> DifferentialRoadBatchContract:
    config_path = Path(path).resolve()
    payload = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    if not isinstance(payload, Mapping):
        raise ValueError("differential batch config must be a mapping")
    if (
        payload.get("kind")
        != "dynamic_road_differential_multiseed_experiment"
    ):
        raise ValueError("unexpected differential batch config kind")
    catalogs = tuple(
        SeedCatalog(
            name=str(name),
            start_inclusive=int(spec["start_inclusive"]),
            count=int(spec["count"]),
            purpose=str(spec["purpose"]),
            eligible_for_final_empirical_claim=bool(
                spec["eligible_for_final_empirical_claim"]
            ),
        )
        for name, spec in payload["seed_catalogs"].items()
    )
    execution = payload["execution"]
    acceptance = payload["acceptance"]
    interval = payload["empirical_interval"]
    differential_path = (
        config_path.parent
        / str(payload["bindings"]["differential_config"])
    ).resolve()
    return DifferentialRoadBatchContract(
        experiment_id=str(payload["experiment"]["id"]),
        config_path=config_path,
        config_file_sha256=file_sha256(config_path),
        differential_config_path=differential_path,
        catalogs=catalogs,
        progress_interval_episodes=int(
            execution["progress_interval_episodes"]
        ),
        maximum_saved_failure_traces=int(
            execution["maximum_saved_failure_traces"]
        ),
        acceptance=DifferentialBatchAcceptanceSpec(
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
            maximum_cbf_infeasible_episodes=int(
                acceptance["maximum_cbf_infeasible_episodes"]
            ),
            maximum_model_contract_breach_episodes=int(
                acceptance["maximum_model_contract_breach_episodes"]
            ),
            minimum_lifted_barrier_m=float(
                acceptance[
                    "minimum_lifted_certificate_barrier_m"
                ]
            ),
            minimum_cbf_constraint_residual_mps=float(
                acceptance[
                    "minimum_cbf_constraint_residual_mps"
                ]
            ),
            minimum_realized_contraction_residual_m=float(
                acceptance[
                    "minimum_realized_contraction_residual_m"
                ]
            ),
            maximum_mapping_residual_mps=float(
                acceptance["maximum_mapping_residual_mps"]
            ),
            maximum_linear_speed_mps=float(
                acceptance["maximum_linear_speed_mps"]
            ),
            maximum_absolute_angular_speed_radps=float(
                acceptance[
                    "maximum_absolute_angular_speed_radps"
                ]
            ),
            required_lane_pattern_count=int(
                acceptance["required_lane_pattern_count"]
            ),
            require_no_post_projection_command_clipping=bool(
                acceptance[
                    "require_no_post_projection_command_clipping"
                ]
            ),
            require_same_metrics_sha256_replay=bool(
                acceptance["require_same_metrics_sha256_replay"]
            ),
        ),
        confidence=float(interval["confidence"]),
    )


def differential_road_episode_record(
    *,
    seed: int,
    contract: DifferentialRoadContract,
    batch_acceptance: DifferentialBatchAcceptanceSpec,
    scenario: DynamicRoadScenario,
    rows: list[Mapping[str, Any]],
    plan_content_sha256: str,
) -> dict[str, Any]:
    """Reduce one differential trace to deterministic batch facts."""

    metrics = differential_road_episode_metrics(rows)
    termination_reason = str(rows[-1]["termination_reason"])
    residual = metrics["minimum_cbf_constraint_residual_mps"]
    contraction = metrics["minimum_realized_contraction_residual_m"]
    lane_pattern = "".join(
        str(obstacle.lane_index)
        for obstacle in scenario.obstacles
    )
    gates = {
        "goal_reached": termination_reason == "goal_reached",
        "all_obstacles_activated": (
            metrics["activated_obstacle_count"]
            == contract.base.obstacle_sampling.count
        ),
        "collision_free": metrics["collision_count"] == 0,
        "road_departure_free": metrics["road_departure_count"] == 0,
        "lifted_barrier_nonnegative": (
            metrics["minimum_lifted_certificate_barrier_m"]
            >= batch_acceptance.minimum_lifted_barrier_m
        ),
        "constraint_residual_nonnegative": (
            residual is not None
            and residual
            >= batch_acceptance.minimum_cbf_constraint_residual_mps
        ),
        "realized_contraction_nonnegative": (
            contraction is not None
            and contraction
            >= batch_acceptance.minimum_realized_contraction_residual_m
        ),
        "mapping_residual_satisfied": (
            metrics["maximum_mapping_residual_mps"]
            <= batch_acceptance.maximum_mapping_residual_mps
        ),
        "linear_bound_satisfied": (
            metrics["maximum_linear_speed_mps"]
            <= batch_acceptance.maximum_linear_speed_mps
        ),
        "angular_bound_satisfied": (
            metrics["maximum_absolute_angular_speed_radps"]
            <= batch_acceptance.maximum_absolute_angular_speed_radps
        ),
        "no_post_projection_command_clipping": all(
            not row["runtime"][
                "post_projection_command_clipping_applied"
            ]
            for row in rows
        ),
    }
    failure_reasons = [
        name for name, passed in gates.items() if not passed
    ]
    return {
        "schema_version": 1,
        "kind": "differential_road_batch_episode",
        "seed": int(seed),
        "status": "passed" if not failure_reasons else "failed",
        "termination_reason": termination_reason,
        "failure_reasons": failure_reasons,
        "lane_pattern": lane_pattern,
        "obstacle_plan_content_sha256": plan_content_sha256,
        "trace_sha256": trace_rows_sha256(rows),
        "transition_count": metrics["transition_count"],
        "simulated_duration_s": metrics["simulated_duration_s"],
        "activated_obstacle_count": metrics["activated_obstacle_count"],
        "collision_count": metrics["collision_count"],
        "road_departure_count": metrics["road_departure_count"],
        "minimum_lifted_certificate_barrier_m": (
            metrics["minimum_lifted_certificate_barrier_m"]
        ),
        "minimum_active_obstacle_barrier_m": (
            metrics["minimum_active_obstacle_barrier_m"]
        ),
        "minimum_cbf_constraint_residual_mps": residual,
        "minimum_realized_contraction_residual_m": contraction,
        "minimum_planned_physical_clearance_m": (
            metrics["minimum_planned_physical_clearance_m"]
        ),
        "minimum_lifted_physical_clearance_m": (
            metrics["minimum_lifted_physical_clearance_m"]
        ),
        "maximum_mapping_residual_mps": (
            metrics["maximum_mapping_residual_mps"]
        ),
        "maximum_linear_speed_mps": (
            metrics["maximum_linear_speed_mps"]
        ),
        "maximum_absolute_angular_speed_radps": (
            metrics["maximum_absolute_angular_speed_radps"]
        ),
        "filter_intervention_step_count": (
            metrics["filter_intervention_step_count"]
        ),
        "maximum_filter_intervention_mps": (
            metrics["maximum_filter_intervention_mps"]
        ),
        "command_saturation_step_count": (
            metrics["command_saturation_step_count"]
        ),
    }


def _aggregate_records(
    records: list[Mapping[str, Any]],
    *,
    acceptance: DifferentialBatchAcceptanceSpec,
    confidence: float,
    expected_metrics_sha256: str | None,
    metrics_sha256: str,
) -> tuple[dict[str, Any], dict[str, Any]]:
    count = len(records)
    passed = sum(record["status"] == "passed" for record in records)
    success_rate = passed / count
    termination_counts = Counter(
        str(record["termination_reason"]) for record in records
    )
    failure_reason_counts = Counter(
        str(reason)
        for record in records
        for reason in record["failure_reasons"]
    )
    lane_pattern_counts = Counter(
        str(record["lane_pattern"]) for record in records
    )
    collision_episodes = sum(
        int(record["collision_count"]) > 0 for record in records
    )
    road_departure_episodes = sum(
        int(record["road_departure_count"]) > 0 for record in records
    )
    timeout_episodes = termination_counts["timeout"]
    infeasible_episodes = termination_counts["cbf_action_set_infeasible"]
    model_breach_episodes = termination_counts["model_contract_breach"]
    minimum_barrier = min(
        float(record["minimum_lifted_certificate_barrier_m"])
        for record in records
    )
    minimum_residual = min(
        float(record["minimum_cbf_constraint_residual_mps"])
        for record in records
        if record["minimum_cbf_constraint_residual_mps"] is not None
    )
    minimum_contraction = min(
        float(record["minimum_realized_contraction_residual_m"])
        for record in records
        if record["minimum_realized_contraction_residual_m"] is not None
    )
    maximum_mapping_residual = max(
        float(record["maximum_mapping_residual_mps"])
        for record in records
    )
    maximum_linear = max(
        float(record["maximum_linear_speed_mps"]) for record in records
    )
    maximum_angular = max(
        float(record["maximum_absolute_angular_speed_radps"])
        for record in records
    )
    replay_match = (
        None
        if expected_metrics_sha256 is None
        else expected_metrics_sha256 == metrics_sha256
    )
    gates = {
        "success_rate": success_rate >= acceptance.required_success_rate,
        "collision_episodes": (
            collision_episodes <= acceptance.maximum_collision_episodes
        ),
        "road_departure_episodes": (
            road_departure_episodes
            <= acceptance.maximum_road_departure_episodes
        ),
        "timeout_episodes": (
            timeout_episodes <= acceptance.maximum_timeout_episodes
        ),
        "cbf_infeasible_episodes": (
            infeasible_episodes
            <= acceptance.maximum_cbf_infeasible_episodes
        ),
        "model_contract_breach_episodes": (
            model_breach_episodes
            <= acceptance.maximum_model_contract_breach_episodes
        ),
        "minimum_lifted_barrier": (
            minimum_barrier >= acceptance.minimum_lifted_barrier_m
        ),
        "minimum_cbf_constraint_residual": (
            minimum_residual
            >= acceptance.minimum_cbf_constraint_residual_mps
        ),
        "minimum_realized_contraction_residual": (
            minimum_contraction
            >= acceptance.minimum_realized_contraction_residual_m
        ),
        "maximum_mapping_residual": (
            maximum_mapping_residual
            <= acceptance.maximum_mapping_residual_mps
        ),
        "maximum_linear_speed": (
            maximum_linear <= acceptance.maximum_linear_speed_mps
        ),
        "maximum_absolute_angular_speed": (
            maximum_angular
            <= acceptance.maximum_absolute_angular_speed_radps
        ),
        "lane_pattern_coverage": (
            len(lane_pattern_counts)
            >= acceptance.required_lane_pattern_count
        ),
        "no_post_projection_command_clipping": all(
            int(record["command_saturation_step_count"]) == 0
            for record in records
        )
        if acceptance.require_no_post_projection_command_clipping
        else True,
    }
    if (
        acceptance.require_same_metrics_sha256_replay
        and expected_metrics_sha256 is not None
    ):
        gates["same_metrics_sha256_replay"] = bool(replay_match)

    exact_lower_bound = (
        (1.0 - confidence) ** (1.0 / count)
        if passed == count
        else None
    )

    def worst(metric: str, *, minimum: bool) -> dict[str, Any]:
        selector = min if minimum else max
        record = selector(
            records,
            key=lambda item: (float(item[metric]), int(item["seed"])),
        )
        return {
            "seed": int(record["seed"]),
            "value": float(record[metric]),
            "trace_sha256": str(record["trace_sha256"]),
            "lane_pattern": str(record["lane_pattern"]),
        }

    aggregate = {
        "episode_count": count,
        "passed_episode_count": passed,
        "failed_episode_count": count - passed,
        "success_rate": success_rate,
        "one_sided_confidence": confidence,
        "all_success_exact_lower_bound": exact_lower_bound,
        "termination_counts": dict(sorted(termination_counts.items())),
        "failure_reason_counts": dict(sorted(failure_reason_counts.items())),
        "collision_episode_count": collision_episodes,
        "road_departure_episode_count": road_departure_episodes,
        "timeout_episode_count": timeout_episodes,
        "cbf_infeasible_episode_count": infeasible_episodes,
        "model_contract_breach_episode_count": model_breach_episodes,
        "lane_pattern_count": len(lane_pattern_counts),
        "lane_pattern_counts": dict(sorted(lane_pattern_counts.items())),
        "minimum_lifted_certificate_barrier_m": minimum_barrier,
        "minimum_cbf_constraint_residual_mps": minimum_residual,
        "minimum_realized_contraction_residual_m": minimum_contraction,
        "minimum_planned_physical_clearance_m": min(
            float(record["minimum_planned_physical_clearance_m"])
            for record in records
        ),
        "minimum_lifted_physical_clearance_m": min(
            float(record["minimum_lifted_physical_clearance_m"])
            for record in records
        ),
        "maximum_mapping_residual_mps": maximum_mapping_residual,
        "maximum_linear_speed_mps": maximum_linear,
        "maximum_absolute_angular_speed_radps": maximum_angular,
        "episodes_with_filter_intervention": sum(
            int(record["filter_intervention_step_count"]) > 0
            for record in records
        ),
        "total_filter_intervention_steps": sum(
            int(record["filter_intervention_step_count"])
            for record in records
        ),
        "worst_minimum_barrier": worst(
            "minimum_lifted_certificate_barrier_m",
            minimum=True,
        ),
        "worst_physical_clearance": worst(
            "minimum_planned_physical_clearance_m",
            minimum=True,
        ),
        "worst_realized_contraction": worst(
            "minimum_realized_contraction_residual_m",
            minimum=True,
        ),
        "largest_mapping_residual": worst(
            "maximum_mapping_residual_mps",
            minimum=False,
        ),
        "longest_episode": worst(
            "transition_count",
            minimum=False,
        ),
        "largest_filter_intervention": worst(
            "maximum_filter_intervention_mps",
            minimum=False,
        ),
    }
    replay = {
        "status": (
            "not_evaluated"
            if replay_match is None
            else ("match" if replay_match else "mismatch")
        ),
        "expected_metrics_sha256": expected_metrics_sha256,
        "actual_metrics_sha256": metrics_sha256,
    }
    return aggregate, {"gates": gates, "metrics_replay": replay}


def run_differential_road_batch(
    *,
    batch_config_path: str | Path,
    catalog_name: str,
    output_dir: str | Path,
    expected_metrics_sha256: str | None = None,
) -> Path:
    """Run one predeclared nonholonomic seed catalog."""

    batch = load_differential_road_batch_contract(batch_config_path)
    catalog = batch.catalog(catalog_name)
    contract = load_differential_road_contract(
        batch.differential_config_path
    )
    destination = Path(output_dir).resolve()
    destination.mkdir(parents=True, exist_ok=False)
    partial_path = destination / "episode_metrics.partial.jsonl"
    final_metrics_path = destination / "episode_metrics.jsonl"
    progress_path = destination / "progress.json"
    records: list[dict[str, Any]] = []
    saved_failures = 0
    start_time = perf_counter()

    _write_json(
        progress_path,
        {
            "schema_version": 1,
            "kind": "differential_road_batch_progress",
            "status": "running",
            "catalog": catalog.name,
            "processed": 0,
            "total": catalog.count,
            "passed": 0,
            "failed": 0,
        },
    )
    try:
        with partial_path.open(
            "x",
            encoding="utf-8",
            newline="\n",
        ) as stream:
            for index, seed in enumerate(catalog.seeds(), start=1):
                scenario = generate_seeded_road_scenario(
                    contract.base,
                    seed=seed,
                )
                plan = obstacle_plan_payload(
                    scenario=scenario,
                    seed=seed,
                )
                rows, _ = run_differential_road_episode(
                    contract=contract,
                    scenario=scenario,
                    seed=seed,
                    wall_clock=False,
                    emit_console=False,
                )
                record = differential_road_episode_record(
                    seed=seed,
                    contract=contract,
                    batch_acceptance=batch.acceptance,
                    scenario=scenario,
                    rows=rows,
                    plan_content_sha256=str(plan["content_sha256"]),
                )
                records.append(record)
                stream.write(_jsonl_line(record))
                stream.flush()

                if (
                    record["status"] == "failed"
                    and saved_failures
                    < batch.maximum_saved_failure_traces
                ):
                    failure_dir = (
                        destination / "failures" / f"seed_{seed}"
                    )
                    failure_dir.mkdir(parents=True, exist_ok=False)
                    write_trace(failure_dir / "live_trace.jsonl", rows)
                    _write_json(failure_dir / "obstacle_plan.json", plan)
                    _write_json(
                        failure_dir / "episode_record.json",
                        record,
                    )
                    saved_failures += 1

                if (
                    index % batch.progress_interval_episodes == 0
                    or index == catalog.count
                    or record["status"] == "failed"
                ):
                    progress = {
                        "schema_version": 1,
                        "kind": "differential_road_batch_progress",
                        "status": "running",
                        "catalog": catalog.name,
                        "processed": index,
                        "total": catalog.count,
                        "passed": sum(
                            item["status"] == "passed"
                            for item in records
                        ),
                        "failed": sum(
                            item["status"] == "failed"
                            for item in records
                        ),
                        "last_seed": seed,
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
                "kind": "differential_road_batch_progress",
                "status": "crashed",
                "catalog": catalog.name,
                "processed": len(records),
                "total": catalog.count,
                "error_type": type(error).__name__,
                "error": str(error),
            },
        )
        raise

    partial_path.replace(final_metrics_path)
    metrics_sha = file_sha256(final_metrics_path)
    aggregate, acceptance = _aggregate_records(
        records,
        acceptance=batch.acceptance,
        confidence=batch.confidence,
        expected_metrics_sha256=expected_metrics_sha256,
        metrics_sha256=metrics_sha,
    )
    base_passed = all(acceptance["gates"].values())
    replay_status = acceptance["metrics_replay"]["status"]
    if catalog.eligible_for_final_empirical_claim:
        if not base_passed or replay_status == "mismatch":
            status = "failed"
        elif replay_status == "match":
            status = "accepted"
        else:
            status = "provisional_success"
    else:
        status = "diagnostic_complete"
    summary: dict[str, Any] = {
        "schema_version": 1,
        "kind": "differential_road_multiseed_summary",
        "experiment_id": batch.experiment_id,
        "status": status,
        "claim_boundary": {
            "exact_batch": "empirical_only",
            "differential_drive_tracking": (
                "exact_for_declared_Euler_model"
            ),
            "random_geometry_q_sbc": "not_claimed",
            "gazebo_tracking_claim": "not_claimed",
            "real_vehicle_claim": "forbidden",
        },
        "batch_config": {
            "path": str(batch.config_path),
            "file_sha256": batch.config_file_sha256,
        },
        "differential_config": {
            "path": str(batch.differential_config_path),
            "file_sha256": contract.config_file_sha256,
        },
        "base_scenario_config": {
            "path": str(contract.base.config_path),
            "file_sha256": contract.base.config_file_sha256,
        },
        "catalog": {
            "name": catalog.name,
            "purpose": catalog.purpose,
            "start_inclusive": catalog.start_inclusive,
            "stop_exclusive": catalog.stop_exclusive,
            "count": catalog.count,
            "eligible_for_final_empirical_claim": (
                catalog.eligible_for_final_empirical_claim
            ),
        },
        "execution": {
            "order": "ascending_seed",
            "parallelism": 1,
            "wall_elapsed_s": perf_counter() - start_time,
            "saved_failure_trace_count": saved_failures,
        },
        "metrics": aggregate,
        "acceptance": acceptance,
        "artifacts": {
            "episode_metrics_file": final_metrics_path.name,
            "episode_metrics_file_sha256": metrics_sha,
        },
    }
    summary["summary_content_sha256"] = canonical_content_sha256(
        summary
    )
    summary_path = destination / "batch_summary.json"
    _write_json(summary_path, summary)
    _write_json(
        progress_path,
        {
            "schema_version": 1,
            "kind": "differential_road_batch_progress",
            "status": "completed",
            "catalog": catalog.name,
            "processed": catalog.count,
            "total": catalog.count,
            "passed": aggregate["passed_episode_count"],
            "failed": aggregate["failed_episode_count"],
            "summary": summary_path.name,
        },
    )
    return summary_path
