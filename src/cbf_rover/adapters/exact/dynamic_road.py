"""Seeded dynamic-road simulation with a live online distance-CBF filter."""

from __future__ import annotations

import csv
from dataclasses import dataclass
from hashlib import sha256
import json
from math import hypot
from pathlib import Path
from time import perf_counter, sleep
from typing import Any, Iterable, Mapping

import numpy as np

from cbf_rover.domain.dynamic_road import (
    DynamicRoadScenario,
    RoadObstacle,
)
from cbf_rover.domain.planar import PlanarAction, PlanarState
from cbf_rover.methods.online_cbf import (
    InfeasibleProjectionError,
    RoadFilterResult,
    filter_dynamic_road_action,
)
from cbf_rover.methods.online_cbf.road_filter import (
    build_dynamic_road_barriers,
)

from .road_contract import (
    NominalRoadControllerSpec,
    RoadExperimentContract,
    canonical_content_sha256,
    file_sha256,
    generate_seeded_road_scenario,
    load_road_experiment_contract,
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


def write_trace(
    path: Path,
    rows: Iterable[Mapping[str, Any]],
) -> None:
    temporary = path.with_name(f".{path.name}.tmp")
    with temporary.open("w", encoding="utf-8", newline="\n") as stream:
        for row in rows:
            stream.write(_canonical_trace_line(row))
    temporary.replace(path)


_TRAJECTORY_FIELDS = (
    "step_index",
    "simulated_time_s",
    "gazebo_sim_time_s",
    "x_m",
    "y_m",
    "heading_rad",
    "lookahead_x_m",
    "lookahead_y_m",
    "target_lane_index",
    "command_linear_mps",
    "command_angular_radps",
    "filtered_u_x_mps",
    "filtered_u_y_mps",
    "minimum_barrier_m",
    "minimum_active_obstacle_barrier_m",
    "active_obstacle_ids",
    "world_model_active_obstacle_count",
    "solve_status",
    "display_level_q_h_m",
    "termination_reason",
)


def write_trajectory_csv(
    path: str | Path,
    rows: Iterable[Mapping[str, Any]],
) -> Path:
    """Export one full-rate, user-readable trajectory from trace rows."""

    destination = Path(path)
    destination.parent.mkdir(parents=True, exist_ok=True)
    temporary = destination.with_name(f".{destination.name}.tmp")
    with temporary.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=_TRAJECTORY_FIELDS)
        writer.writeheader()
        for row in rows:
            state = row["state"]
            point = state.get(
                "lookahead_point_m",
                [state["x_m"], state["y_m"]],
            )
            action = row.get("action") or {}
            command = action.get("differential_command") or {}
            filtered = (
                action.get("filtered_lookahead")
                or action.get("filtered")
                or {}
            )
            certificate = row["certificate"]
            display_level = certificate.get("display_level") or {}
            solve = certificate.get("online_solve") or {}
            measurement = state.get("gazebo_measurement") or {}
            world_model = row.get("world_model") or {}
            planner = row.get("planner") or {}
            writer.writerow(
                {
                    "step_index": row["step_index"],
                    "simulated_time_s": row["simulated_time_s"],
                    "gazebo_sim_time_s": measurement.get(
                        "gazebo_sim_time_s", ""
                    ),
                    "x_m": state["x_m"],
                    "y_m": state["y_m"],
                    "heading_rad": state.get("heading_rad", ""),
                    "lookahead_x_m": point[0],
                    "lookahead_y_m": point[1],
                    "target_lane_index": planner.get(
                        "target_lane_index", ""
                    ),
                    "command_linear_mps": command.get("linear_mps", ""),
                    "command_angular_radps": command.get(
                        "angular_radps", ""
                    ),
                    "filtered_u_x_mps": filtered.get("u_x_mps", ""),
                    "filtered_u_y_mps": filtered.get("u_y_mps", ""),
                    "minimum_barrier_m": certificate["minimum_barrier_m"],
                    "minimum_active_obstacle_barrier_m": certificate.get(
                        "minimum_active_obstacle_barrier_m", ""
                    ),
                    "active_obstacle_ids": ";".join(
                        row["obstacles"]["active_ids"]
                    ),
                    "world_model_active_obstacle_count": world_model.get(
                        "active_obstacle_count", ""
                    ),
                    "solve_status": solve.get("status", ""),
                    "display_level_q_h_m": display_level.get(
                        "value_m", ""
                    ),
                    "termination_reason": row.get("termination_reason") or "",
                }
            )
    temporary.replace(destination)
    return destination


def _canonical_trace_line(row: Mapping[str, Any]) -> str:
    return (
        json.dumps(
            row,
            sort_keys=True,
            separators=(",", ":"),
            ensure_ascii=False,
            allow_nan=False,
        )
        + "\n"
    )


def trace_rows_sha256(rows: Iterable[Mapping[str, Any]]) -> str:
    """Hash rows exactly as ``live_trace.jsonl`` writes them."""

    digest = sha256()
    for row in rows:
        digest.update(_canonical_trace_line(row).encode("utf-8"))
    return digest.hexdigest()


@dataclass(slots=True)
class LaneSupervisor:
    """Deterministic two-lane nominal controller with lane persistence."""

    contract: RoadExperimentContract
    target_lane_index: int
    controller_spec: NominalRoadControllerSpec

    @classmethod
    def from_contract(
        cls,
        contract: RoadExperimentContract,
        *,
        controller_spec: NominalRoadControllerSpec | None = None,
    ) -> "LaneSupervisor":
        start_y = contract.geometry.start.y_m
        lane_index = min(
            range(2),
            key=lambda index: abs(
                contract.geometry.lane_centers_y_m[index] - start_y
            ),
        )
        return cls(
            contract=contract,
            target_lane_index=lane_index,
            controller_spec=(
                contract.nominal_controller
                if controller_spec is None
                else controller_spec
            ),
        )

    def _lane_obstacle_distance(
        self,
        *,
        lane_index: int,
        state: PlanarState,
        active_obstacles: tuple[RoadObstacle, ...],
    ) -> float:
        pass_margin = self.controller_spec.obstacle_pass_margin_m
        minimum_distance = (
            0.0
            if self.controller_spec.ignore_passed_obstacles_for_lane_selection
            else -pass_margin
        )
        candidates = [
            obstacle.center.x_m - state.x_m
            for obstacle in active_obstacles
            if obstacle.lane_index == lane_index
            and obstacle.center.x_m - state.x_m >= minimum_distance
        ]
        return min(candidates, default=float("inf"))

    def nominal_action(
        self,
        *,
        state: PlanarState,
        active_obstacles: tuple[RoadObstacle, ...],
    ) -> tuple[PlanarAction, dict[str, Any]]:
        spec = self.controller_spec
        lane_distances = tuple(
            self._lane_obstacle_distance(
                lane_index=index,
                state=state,
                active_obstacles=active_obstacles,
            )
            for index in range(2)
        )
        current_distance = lane_distances[self.target_lane_index]
        if current_distance <= spec.lane_switch_lookahead_m:
            other_index = 1 - self.target_lane_index
            if lane_distances[other_index] > current_distance:
                self.target_lane_index = other_index

        target_y = self.contract.geometry.lane_centers_y_m[
            self.target_lane_index
        ]
        remaining_x = max(
            0.0,
            self.contract.geometry.finish_line_x_m - state.x_m,
        )
        forward = spec.forward_speed_mps
        if remaining_x < spec.finish_slowdown_distance_m:
            forward = max(
                0.12,
                forward
                * remaining_x
                / spec.finish_slowdown_distance_m,
            )
        lateral = spec.lateral_gain_per_s * (target_y - state.y_m)
        unscaled_speed = hypot(forward, lateral)
        bounded_by_design = unscaled_speed > spec.max_nominal_speed_mps
        if bounded_by_design:
            scale = spec.max_nominal_speed_mps / unscaled_speed
            forward *= scale
            lateral *= scale
        action = PlanarAction(forward, lateral)
        return action, {
            "controller_id": spec.controller_id,
            "target_lane_index": self.target_lane_index,
            "target_lane_y_m": target_y,
            "lane_obstacle_distances_m": [
                None if value == float("inf") else value
                for value in lane_distances
            ],
            "bounded_action_mapping_applied": bounded_by_design,
            "passed_obstacles_ignored_for_lane_selection": (
                spec.ignore_passed_obstacles_for_lane_selection
            ),
        }


def _active_obstacles(
    scenario: DynamicRoadScenario,
    active_ids: set[str],
) -> tuple[RoadObstacle, ...]:
    return tuple(
        obstacle
        for obstacle in scenario.obstacles
        if obstacle.obstacle_id in active_ids
    )


def _activate_visible_obstacles(
    scenario: DynamicRoadScenario,
    state: PlanarState,
    active_ids: set[str],
) -> tuple[str, ...]:
    newly_active = tuple(
        obstacle.obstacle_id
        for obstacle in scenario.obstacles
        if obstacle.obstacle_id not in active_ids
        and obstacle.is_visible(state)
    )
    active_ids.update(newly_active)
    return newly_active


def _state_payload(
    state: PlanarState,
    *,
    scenario: DynamicRoadScenario,
) -> dict[str, Any]:
    collision_ids = scenario.collision_ids(state)
    geometry = scenario.geometry
    return {
        "x_m": state.x_m,
        "y_m": state.y_m,
        "frame_id": "world",
        "goal_reached": geometry.goal_reached(state),
        "road_departure": geometry.road_departure(state),
        "in_x_domain": geometry.x_min_m <= state.x_m <= geometry.x_max_m,
        "collision_ids": list(collision_ids),
        "minimum_planned_physical_clearance_m": min(
            obstacle.physical_clearance_m(state)
            for obstacle in scenario.obstacles
        ),
    }


def _constraint_payload(
    filter_result: RoadFilterResult | None,
    *,
    state: PlanarState,
    scenario: DynamicRoadScenario,
    active_obstacles: tuple[RoadObstacle, ...],
    contract: RoadExperimentContract,
) -> tuple[list[dict[str, Any]], float, float | None, float | None]:
    if filter_result is None:
        facts = build_dynamic_road_barriers(
            state=state,
            geometry=scenario.geometry,
            active_obstacles=active_obstacles,
            spec=contract.cbf,
        )
        residual_by_name: dict[str, float] = {}
    else:
        facts = filter_result.facts
        residual_by_name = dict(
            filter_result.projection.constraint_residuals_mps
        )
    rows = [
        {
            "barrier_id": fact.barrier_id,
            "barrier_kind": fact.barrier_kind,
            "barrier_m": fact.barrier_m,
            "normal": [fact.normal_x, fact.normal_y],
            "robust_rhs_mps": fact.robust_rhs_mps,
            "constraint_residual_mps": residual_by_name.get(
                fact.barrier_id
            ),
        }
        for fact in facts
    ]
    minimum_barrier = min(float(row["barrier_m"]) for row in rows)
    obstacle_barriers = [
        float(row["barrier_m"])
        for row in rows
        if row["barrier_kind"] == "obstacle"
    ]
    residuals = [
        float(value)
        for value in residual_by_name.values()
    ]
    return (
        rows,
        minimum_barrier,
        min(obstacle_barriers, default=None),
        min(residuals, default=None),
    )


def _frame(
    *,
    step_index: int,
    state: PlanarState,
    scenario: DynamicRoadScenario,
    contract: RoadExperimentContract,
    active_obstacles: tuple[RoadObstacle, ...],
    newly_activated_ids: tuple[str, ...],
    planner_facts: Mapping[str, Any] | None,
    filter_result: RoadFilterResult | None,
    transition_noise_m: tuple[float, float] | None,
    successor: PlanarState | None,
    termination_reason: str | None,
) -> dict[str, Any]:
    constraints, minimum_barrier, minimum_obstacle_barrier, minimum_residual = (
        _constraint_payload(
            filter_result,
            state=state,
            scenario=scenario,
            active_obstacles=active_obstacles,
            contract=contract,
        )
    )
    action_payload: dict[str, Any] | None = None
    if filter_result is not None:
        nominal = filter_result.nominal_action
        filtered = filter_result.filtered_action
        action_payload = {
            "nominal": {
                "u_x_mps": nominal.u_x_mps,
                "u_y_mps": nominal.u_y_mps,
                "speed_mps": hypot(
                    nominal.u_x_mps,
                    nominal.u_y_mps,
                ),
            },
            "filtered": {
                "u_x_mps": filtered.u_x_mps,
                "u_y_mps": filtered.u_y_mps,
                "speed_mps": hypot(
                    filtered.u_x_mps,
                    filtered.u_y_mps,
                ),
            },
            "intervention_norm_mps": (
                filter_result.intervention_norm_mps
            ),
            "projection_source": (
                filter_result.projection.selected_source
            ),
            "active_constraint_names": list(
                filter_result.projection.active_constraint_names
            ),
            "speed_margin_mps": (
                filter_result.projection.speed_margin_mps
            ),
        }
    return {
        "schema_version": 1,
        "kind": "dynamic_road_online_cbf_frame",
        "scenario_id": scenario.scenario_id,
        "step_index": int(step_index),
        "simulated_time_s": step_index * contract.system.dt_s,
        "state": _state_payload(state, scenario=scenario),
        "obstacles": {
            "active_ids": [
                obstacle.obstacle_id for obstacle in active_obstacles
            ],
            "newly_activated_ids": list(newly_activated_ids),
            "active_count": len(active_obstacles),
            "planned_count": len(scenario.obstacles),
        },
        "planner": None if planner_facts is None else dict(planner_facts),
        "action": action_payload,
        "certificate": {
            "certificate_id": "DR1_ROBUST_DISTANCE_CBF_V1",
            "family": "online_robust_distance_cbf",
            "is_q_sbc": False,
            "q_sbc_probability_bound_claimed": False,
            "minimum_barrier_m": minimum_barrier,
            "minimum_active_obstacle_barrier_m": (
                minimum_obstacle_barrier
            ),
            "minimum_constraint_residual_mps": minimum_residual,
            "constraints": constraints,
            "updates_from_measured_state_each_step": True,
        },
        "transition": (
            None
            if successor is None
            else {
                "noise_m": list(transition_noise_m or ()),
                "successor_state_m": [
                    successor.x_m,
                    successor.y_m,
                ],
            }
        ),
        "termination_reason": termination_reason,
        "runtime": {
            "backend": "M1_exact_stochastic_single_integrator",
            "mode": "online_action_filter",
            "certificate_modified_action": (
                filter_result is not None
                and filter_result.intervention_norm_mps > 1.0e-12
            ),
            "post_filter_clipping_applied": False,
        },
    }


def _terminal_reason(
    state: PlanarState,
    scenario: DynamicRoadScenario,
) -> str | None:
    geometry = scenario.geometry
    if not geometry.x_min_m <= state.x_m <= geometry.x_max_m:
        return "model_contract_breach"
    if geometry.road_departure(state):
        return "road_departure"
    if scenario.collision_ids(state):
        return "collision"
    if geometry.goal_reached(state):
        return "goal_reached"
    return None


def run_dynamic_road_episode(
    *,
    contract: RoadExperimentContract,
    scenario: DynamicRoadScenario,
    seed: int,
    wall_clock: bool,
    emit_console: bool,
) -> tuple[list[dict[str, Any]], float]:
    """Execute one deterministic-seed stochastic episode."""

    if seed < 0:
        raise ValueError("seed must be nonnegative")
    noise_seed = np.random.SeedSequence([int(seed), 0xCBF])
    noise_rng = np.random.default_rng(noise_seed)
    planner = LaneSupervisor.from_contract(contract)
    state = scenario.geometry.start
    active_ids: set[str] = set()
    rows: list[dict[str, Any]] = []
    transition_count = 0
    start_time = perf_counter()
    termination_reason: str | None = None

    for step_index in range(contract.max_steps):
        transition_count = step_index
        termination_reason = _terminal_reason(state, scenario)
        if termination_reason is not None:
            break
        newly_active = _activate_visible_obstacles(
            scenario,
            state,
            active_ids,
        )
        active_obstacles = _active_obstacles(scenario, active_ids)
        nominal_action, planner_facts = planner.nominal_action(
            state=state,
            active_obstacles=active_obstacles,
        )
        try:
            filter_result = filter_dynamic_road_action(
                state=state,
                nominal_action=nominal_action,
                geometry=scenario.geometry,
                active_obstacles=active_obstacles,
                spec=contract.cbf,
            )
        except InfeasibleProjectionError:
            termination_reason = "cbf_action_set_infeasible"
            rows.append(
                _frame(
                    step_index=step_index,
                    state=state,
                    scenario=scenario,
                    contract=contract,
                    active_obstacles=active_obstacles,
                    newly_activated_ids=newly_active,
                    planner_facts=planner_facts,
                    filter_result=None,
                    transition_noise_m=None,
                    successor=None,
                    termination_reason=termination_reason,
                )
            )
            break
        action = filter_result.filtered_action
        if not contract.system.action_is_admissible(
            action,
            tolerance=contract.cbf.feasibility_tolerance,
        ):
            raise RuntimeError("online CBF projection violated the S1 disk")
        half_width = contract.system.noise.half_width_m
        noise_array = noise_rng.triangular(
            -half_width,
            0.0,
            half_width,
            size=2,
        )
        noise = (float(noise_array[0]), float(noise_array[1]))
        successor = contract.system.successor(state, action, noise)
        row = _frame(
            step_index=step_index,
            state=state,
            scenario=scenario,
            contract=contract,
            active_obstacles=active_obstacles,
            newly_activated_ids=newly_active,
            planner_facts=planner_facts,
            filter_result=filter_result,
            transition_noise_m=noise,
            successor=successor,
            termination_reason=None,
        )
        rows.append(row)
        if emit_console:
            print(
                json.dumps(
                    row,
                    sort_keys=True,
                    separators=(",", ":"),
                    allow_nan=False,
                ),
                flush=True,
            )
        state = successor
        transition_count = step_index + 1
        if wall_clock:
            target_elapsed = transition_count * contract.system.dt_s
            remaining = target_elapsed - (perf_counter() - start_time)
            if remaining > 0.0:
                sleep(remaining)
    else:
        termination_reason = "timeout"

    if not rows or rows[-1]["termination_reason"] is None:
        if termination_reason is None:
            termination_reason = _terminal_reason(state, scenario) or "timeout"
        newly_active = _activate_visible_obstacles(
            scenario,
            state,
            active_ids,
        )
        active_obstacles = _active_obstacles(scenario, active_ids)
        final_row = _frame(
            step_index=transition_count,
            state=state,
            scenario=scenario,
            contract=contract,
            active_obstacles=active_obstacles,
            newly_activated_ids=newly_active,
            planner_facts={
                "controller_id": (
                    contract.nominal_controller.controller_id
                ),
                "target_lane_index": planner.target_lane_index,
                "target_lane_y_m": (
                    contract.geometry.lane_centers_y_m[
                        planner.target_lane_index
                    ]
                ),
            },
            filter_result=None,
            transition_noise_m=None,
            successor=None,
            termination_reason=termination_reason,
        )
        rows.append(final_row)
        if emit_console:
            print(
                json.dumps(
                    final_row,
                    sort_keys=True,
                    separators=(",", ":"),
                    allow_nan=False,
                ),
                flush=True,
            )
    return rows, perf_counter() - start_time


def dynamic_road_episode_metrics(
    rows: list[Mapping[str, Any]],
) -> dict[str, Any]:
    action_rows = [row for row in rows if row["action"] is not None]
    obstacle_barriers = [
        float(value)
        for row in rows
        if (
            value := row["certificate"][
                "minimum_active_obstacle_barrier_m"
            ]
        )
        is not None
    ]
    residuals = [
        float(value)
        for row in action_rows
        if (
            value := row["certificate"][
                "minimum_constraint_residual_mps"
            ]
        )
        is not None
    ]
    activated_ids = {
        obstacle_id
        for row in rows
        for obstacle_id in row["obstacles"]["active_ids"]
    }
    return {
        "transition_count": len(action_rows),
        "frame_count": len(rows),
        "simulated_duration_s": float(rows[-1]["simulated_time_s"]),
        "activated_obstacle_count": len(activated_ids),
        "activated_obstacle_ids": sorted(activated_ids),
        "collision_count": sum(
            bool(row["state"]["collision_ids"]) for row in rows
        ),
        "road_departure_count": sum(
            bool(row["state"]["road_departure"]) for row in rows
        ),
        "minimum_certificate_barrier_m": min(
            float(row["certificate"]["minimum_barrier_m"])
            for row in rows
        ),
        "minimum_active_obstacle_barrier_m": min(
            obstacle_barriers,
            default=None,
        ),
        "minimum_cbf_constraint_residual_mps": min(
            residuals,
            default=None,
        ),
        "minimum_planned_physical_clearance_m": min(
            float(row["state"]["minimum_planned_physical_clearance_m"])
            for row in rows
        ),
        "maximum_filter_intervention_mps": max(
            (
                float(row["action"]["intervention_norm_mps"])
                for row in action_rows
            ),
            default=0.0,
        ),
        "filter_intervention_step_count": sum(
            float(row["action"]["intervention_norm_mps"]) > 1.0e-12
            for row in action_rows
        ),
        "maximum_filtered_speed_mps": max(
            (
                float(row["action"]["filtered"]["speed_mps"])
                for row in action_rows
            ),
            default=0.0,
        ),
    }


def _acceptance(
    *,
    rows: list[Mapping[str, Any]],
    metrics: Mapping[str, Any],
    contract: RoadExperimentContract,
    trace_sha256: str,
    expected_trace_sha256: str | None,
) -> dict[str, Any]:
    spec = contract.acceptance
    residual = metrics["minimum_cbf_constraint_residual_mps"]
    gates = {
        "goal_reached": rows[-1]["termination_reason"] == "goal_reached",
        "all_obstacles_activated": (
            metrics["activated_obstacle_count"]
            == spec.required_obstacle_activations
        ),
        "collision_count": (
            metrics["collision_count"] <= spec.maximum_collision_count
        ),
        "road_departure_count": (
            metrics["road_departure_count"]
            <= spec.maximum_road_departure_count
        ),
        "certificate_barrier": (
            metrics["minimum_certificate_barrier_m"]
            >= spec.minimum_certificate_barrier_m
        ),
        "cbf_constraint_residual": (
            residual is not None
            and residual >= spec.minimum_constraint_residual_mps
        ),
        "filter_intervention_observed": (
            metrics["filter_intervention_step_count"]
            >= spec.minimum_filter_intervention_step_count
        ),
        "no_post_filter_clipping": all(
            not row["runtime"]["post_filter_clipping_applied"]
            for row in rows
        ),
    }
    replay_match = (
        None
        if expected_trace_sha256 is None
        else trace_sha256 == expected_trace_sha256
    )
    base_passed = all(gates.values())
    if not base_passed or replay_match is False:
        status = "failed"
    elif replay_match is True:
        status = "accepted"
    else:
        status = "provisional_success"
    return {
        "status": status,
        "gates": gates,
        "same_seed_trace_replay": {
            "status": (
                "not_evaluated"
                if replay_match is None
                else ("match" if replay_match else "mismatch")
            ),
            "expected_trace_sha256": expected_trace_sha256,
            "actual_trace_sha256": trace_sha256,
        },
    }


def simulate_dynamic_road_cbf_live(
    *,
    config_path: str | Path,
    output_dir: str | Path,
    seed: int | None = None,
    wall_clock: bool = True,
    emit_console: bool = False,
    render_gif: bool = True,
    display_stride: int | None = None,
    expected_trace_sha256: str | None = None,
) -> Path:
    """Execute, render, hash, and classify one dynamic-road CBF run."""

    contract = load_road_experiment_contract(config_path)
    actual_seed = contract.default_seed if seed is None else int(seed)
    scenario = generate_seeded_road_scenario(
        contract,
        seed=actual_seed,
    )
    destination = Path(output_dir).resolve()
    destination.mkdir(parents=True, exist_ok=False)

    plan = obstacle_plan_payload(scenario=scenario, seed=actual_seed)
    plan_path = destination / "obstacle_plan.json"
    _write_json(plan_path, plan)
    rows, wall_elapsed_s = run_dynamic_road_episode(
        contract=contract,
        scenario=scenario,
        seed=actual_seed,
        wall_clock=wall_clock,
        emit_console=emit_console,
    )
    trace_path = destination / "live_trace.jsonl"
    write_trace(trace_path, rows)
    trace_sha = file_sha256(trace_path)
    metrics = dynamic_road_episode_metrics(rows)
    acceptance = _acceptance(
        rows=rows,
        metrics=metrics,
        contract=contract,
        trace_sha256=trace_sha,
        expected_trace_sha256=expected_trace_sha256,
    )

    animation_path = destination / "dynamic_road_cbf_1x.gif"
    final_frame_path = destination / "dynamic_road_cbf_final.png"
    stride = contract.display_stride if display_stride is None else int(
        display_stride
    )
    if stride <= 0:
        raise ValueError("display_stride must be positive")
    if render_gif:
        from .dynamic_road_render import render_dynamic_road_animation

        render_dynamic_road_animation(
            rows,
            scenario=scenario,
            gif_path=animation_path,
            final_frame_path=final_frame_path,
            dt_s=contract.system.dt_s,
            display_stride=stride,
        )

    manifest: dict[str, Any] = {
        "schema_version": 1,
        "kind": "dynamic_road_online_cbf_run",
        "status": acceptance["status"],
        "termination_reason": rows[-1]["termination_reason"],
        "claim_boundary": {
            "online_distance_cbf": "runtime_evaluated_action_filter",
            "trajectory": "one_seeded_stochastic_sample",
            "random_geometry_q_sbc": "not_claimed",
            "static_q_sbc_inherited": False,
            "gazebo_tracking_claim": "not_applicable",
        },
        "scenario": {
            "scenario_id": scenario.scenario_id,
            "config_path": str(contract.config_path),
            "config_file_sha256": contract.config_file_sha256,
            "obstacle_plan_file": plan_path.name,
            "obstacle_plan_file_sha256": file_sha256(plan_path),
            "obstacle_plan_content_sha256": plan["content_sha256"],
        },
        "seed": actual_seed,
        "wall_clock_paced": bool(wall_clock),
        "wall_elapsed_s": wall_elapsed_s,
        "telemetry": {
            "trace_file": trace_path.name,
            "trace_file_sha256": trace_sha,
            **metrics,
        },
        "acceptance": acceptance,
        "runtime": {
            "backend": "M1_exact_stochastic_single_integrator",
            "certificate_mode": "online_action_filter",
            "post_filter_clipping_applied": False,
        },
        "render": (
            {
                "playback_rate": "1x_simulated_time",
                "display_stride": stride,
                "gif_file": animation_path.name,
                "gif_file_sha256": file_sha256(animation_path),
                "final_frame_file": final_frame_path.name,
                "final_frame_file_sha256": file_sha256(final_frame_path),
            }
            if render_gif
            else None
        ),
    }
    manifest["manifest_content_sha256"] = canonical_content_sha256(
        manifest
    )
    manifest_path = destination / "run_manifest.json"
    _write_json(manifest_path, manifest)
    return manifest_path
