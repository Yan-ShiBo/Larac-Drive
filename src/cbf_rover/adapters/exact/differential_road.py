"""Exact stochastic differential-drive simulation with a lifted online CBF."""

from __future__ import annotations

from math import cos, hypot, sin
import json
from pathlib import Path
from time import perf_counter, sleep
from typing import Any, Mapping

import numpy as np

from cbf_rover.domain.differential_drive import DifferentialDriveState
from cbf_rover.domain.dynamic_road import DynamicRoadScenario, RoadObstacle
from cbf_rover.domain.planar import PlanarAction, PlanarState
from cbf_rover.methods.controllers.lookahead import LookaheadMappingResult
from cbf_rover.methods.online_cbf import (
    DistanceCBFSpec,
    InfeasibleProjectionError,
    RoadFilterResult,
    filter_dynamic_road_action,
)
from cbf_rover.methods.online_cbf.road_filter import (
    build_dynamic_road_barriers,
)

from .differential_road_contract import DifferentialRoadContract
from .dynamic_road import LaneSupervisor, write_trace
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
    center: PlanarState,
    active_ids: set[str],
) -> tuple[str, ...]:
    newly_active = tuple(
        obstacle.obstacle_id
        for obstacle in scenario.obstacles
        if obstacle.obstacle_id not in active_ids
        and obstacle.is_visible(center)
    )
    active_ids.update(newly_active)
    return newly_active


def _terminal_reason(
    state: DifferentialDriveState,
    scenario: DynamicRoadScenario,
) -> str | None:
    center = state.center
    geometry = scenario.geometry
    if not geometry.x_min_m <= center.x_m <= geometry.x_max_m:
        return "model_contract_breach"
    if geometry.road_departure(center):
        return "road_departure"
    if scenario.collision_ids(center):
        return "collision"
    if geometry.goal_reached(center):
        return "goal_reached"
    return None


def _realized_contraction_residuals(
    *,
    current_filter: RoadFilterResult,
    successor: DifferentialDriveState,
    active_obstacles: tuple[RoadObstacle, ...],
    scenario: DynamicRoadScenario,
    contract: DifferentialRoadContract,
    cbf_spec: DistanceCBFSpec | None = None,
) -> dict[str, float]:
    active_cbf = contract.lifted_cbf if cbf_spec is None else cbf_spec
    next_point = successor.lookahead_point(
        contract.mapper.lookahead_distance_m
    )
    next_facts = build_dynamic_road_barriers(
        state=next_point,
        geometry=scenario.geometry,
        active_obstacles=active_obstacles,
        spec=active_cbf,
        additional_inflation_m=contract.additional_body_inflation_m,
    )
    next_barriers = {
        fact.barrier_id: fact.barrier_m for fact in next_facts
    }
    contraction = 1.0 - active_cbf.contraction_kappa
    return {
        fact.barrier_id: (
            next_barriers[fact.barrier_id]
            - contraction * fact.barrier_m
        )
        for fact in current_filter.facts
    }


def _state_payload(
    state: DifferentialDriveState,
    *,
    scenario: DynamicRoadScenario,
    contract: DifferentialRoadContract,
) -> dict[str, Any]:
    center = state.center
    point = state.lookahead_point(contract.mapper.lookahead_distance_m)
    inflation = contract.additional_body_inflation_m
    return {
        "x_m": center.x_m,
        "y_m": center.y_m,
        "heading_rad": state.heading_rad,
        "frame_id": "world",
        "lookahead_point_m": [point.x_m, point.y_m],
        "lookahead_distance_m": contract.mapper.lookahead_distance_m,
        "goal_reached": scenario.geometry.goal_reached(center),
        "road_departure": scenario.geometry.road_departure(center),
        "in_x_domain": (
            scenario.geometry.x_min_m
            <= center.x_m
            <= scenario.geometry.x_max_m
        ),
        "collision_ids": list(scenario.collision_ids(center)),
        "minimum_planned_physical_clearance_m": min(
            obstacle.physical_clearance_m(center)
            for obstacle in scenario.obstacles
        ),
        "minimum_lifted_physical_clearance_m": min(
            obstacle.center_distance_m(point)
            - obstacle.collision_radius_m
            - inflation
            for obstacle in scenario.obstacles
        ),
    }


def _frame(
    *,
    step_index: int,
    state: DifferentialDriveState,
    scenario: DynamicRoadScenario,
    contract: DifferentialRoadContract,
    active_obstacles: tuple[RoadObstacle, ...],
    newly_activated_ids: tuple[str, ...],
    planner_facts: Mapping[str, Any] | None,
    filter_result: RoadFilterResult | None,
    mapping_result: LookaheadMappingResult | None,
    transition_noise_m: tuple[float, float] | None,
    successor: DifferentialDriveState | None,
    realized_contraction_residuals: Mapping[str, float] | None,
    termination_reason: str | None,
    cbf_spec: DistanceCBFSpec | None = None,
) -> dict[str, Any]:
    active_cbf = contract.lifted_cbf if cbf_spec is None else cbf_spec
    point = state.lookahead_point(contract.mapper.lookahead_distance_m)
    facts = (
        build_dynamic_road_barriers(
            state=point,
            geometry=scenario.geometry,
            active_obstacles=active_obstacles,
            spec=active_cbf,
            additional_inflation_m=contract.additional_body_inflation_m,
        )
        if filter_result is None
        else filter_result.facts
    )
    projection_residuals = (
        {}
        if filter_result is None
        else dict(filter_result.projection.constraint_residuals_mps)
    )
    barrier_residuals = [
        projection_residuals[fact.barrier_id]
        for fact in facts
        if fact.barrier_id in projection_residuals
    ]
    reachability_residuals = [
        value
        for name, value in projection_residuals.items()
        if name.startswith("differential_")
    ]
    obstacle_barriers = [
        fact.barrier_m
        for fact in facts
        if fact.barrier_kind == "obstacle"
    ]
    contraction_values = list(
        (realized_contraction_residuals or {}).values()
    )

    action_payload: dict[str, Any] | None = None
    if filter_result is not None and mapping_result is not None:
        nominal = filter_result.nominal_action
        filtered = filter_result.filtered_action
        command = mapping_result.action
        center_velocity = PlanarAction(
            command.linear_mps * cos(state.heading_rad),
            command.linear_mps * sin(state.heading_rad),
        )
        action_payload = {
            "nominal_lookahead": {
                "u_x_mps": nominal.u_x_mps,
                "u_y_mps": nominal.u_y_mps,
                "speed_mps": hypot(
                    nominal.u_x_mps,
                    nominal.u_y_mps,
                ),
            },
            "filtered_lookahead": {
                "u_x_mps": filtered.u_x_mps,
                "u_y_mps": filtered.u_y_mps,
                "speed_mps": hypot(
                    filtered.u_x_mps,
                    filtered.u_y_mps,
                ),
            },
            "differential_command": {
                "linear_mps": command.linear_mps,
                "angular_radps": command.angular_radps,
            },
            "realized_center_velocity": {
                "u_x_mps": center_velocity.u_x_mps,
                "u_y_mps": center_velocity.u_y_mps,
                "speed_mps": abs(command.linear_mps),
            },
            "realized_lookahead_velocity": {
                "u_x_mps": (
                    mapping_result.realized_lookahead_velocity.u_x_mps
                ),
                "u_y_mps": (
                    mapping_result.realized_lookahead_velocity.u_y_mps
                ),
            },
            "mapping_residual_mps": mapping_result.mapping_residual_mps,
            "intervention_norm_mps": (
                filter_result.intervention_norm_mps
            ),
            "projection_source": (
                filter_result.projection.selected_source
            ),
            "active_constraint_names": list(
                filter_result.projection.active_constraint_names
            ),
            "minimum_reachability_residual_mps": min(
                reachability_residuals,
                default=None,
            ),
            "command_saturation_applied": (
                mapping_result.saturation_applied
            ),
        }

    return {
        "schema_version": 1,
        "kind": "dynamic_road_differential_cbf_frame",
        "experiment_id": contract.experiment_id,
        "scenario_id": scenario.scenario_id,
        "step_index": int(step_index),
        "simulated_time_s": step_index * contract.system.dt_s,
        "state": _state_payload(
            state,
            scenario=scenario,
            contract=contract,
        ),
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
            "certificate_id": "DR2_LOOKAHEAD_ROBUST_DISTANCE_CBF_V1",
            "family": "lifted_online_robust_distance_cbf",
            "evaluation_point": "body_center_plus_lookahead",
            "is_q_sbc": False,
            "q_sbc_probability_bound_claimed": False,
            "additional_body_inflation_m": (
                contract.additional_body_inflation_m
            ),
            "position_noise_half_width_m": (
                contract.system.noise.half_width_m
            ),
            "heading_remainder_bound_m": (
                contract.heading_remainder_bound_m
            ),
            "effective_position_support_half_width_m": (
                active_cbf.noise_half_width_m
            ),
            "minimum_barrier_m": min(
                fact.barrier_m for fact in facts
            ),
            "minimum_active_obstacle_barrier_m": min(
                obstacle_barriers,
                default=None,
            ),
            "minimum_constraint_residual_mps": min(
                barrier_residuals,
                default=None,
            ),
            "minimum_realized_contraction_residual_m": min(
                contraction_values,
                default=None,
            ),
            "constraints": [
                {
                    "barrier_id": fact.barrier_id,
                    "barrier_kind": fact.barrier_kind,
                    "barrier_m": fact.barrier_m,
                    "normal": [fact.normal_x, fact.normal_y],
                    "robust_rhs_mps": fact.robust_rhs_mps,
                    "constraint_residual_mps": (
                        projection_residuals.get(fact.barrier_id)
                    ),
                    "realized_contraction_residual_m": (
                        None
                        if realized_contraction_residuals is None
                        else realized_contraction_residuals.get(
                            fact.barrier_id
                        )
                    ),
                }
                for fact in facts
            ],
            "updates_from_measured_state_each_step": True,
        },
        "transition": (
            None
            if successor is None
            else {
                "noise_m": list(transition_noise_m or ()),
                "successor_state": {
                    "x_m": successor.x_m,
                    "y_m": successor.y_m,
                    "heading_rad": successor.heading_rad,
                    "lookahead_point_m": list(
                        successor.lookahead_point(
                            contract.mapper.lookahead_distance_m
                        ).as_tuple()
                    ),
                },
            }
        ),
        "termination_reason": termination_reason,
        "runtime": {
            "backend": "M2_exact_stochastic_differential_drive_euler",
            "mode": "lifted_online_action_filter",
            "certificate_modified_lookahead_velocity": (
                filter_result is not None
                and filter_result.intervention_norm_mps > 1.0e-12
            ),
            "post_projection_command_clipping_applied": False,
        },
    }


def run_differential_road_episode(
    *,
    contract: DifferentialRoadContract,
    scenario: DynamicRoadScenario,
    seed: int,
    wall_clock: bool,
    emit_console: bool,
) -> tuple[list[dict[str, Any]], float]:
    """Run one exact-backend stochastic nonholonomic episode."""

    if seed < 0:
        raise ValueError("seed must be nonnegative")
    noise_rng = np.random.default_rng(
        np.random.SeedSequence([int(seed), 0xCBF])
    )
    planner = LaneSupervisor.from_contract(
        contract.base,
        controller_spec=contract.nominal_controller,
    )
    state = DifferentialDriveState(
        scenario.geometry.start.x_m,
        scenario.geometry.start.y_m,
        contract.initial_heading_rad,
    )
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
            state.center,
            active_ids,
        )
        active_obstacles = _active_obstacles(scenario, active_ids)
        point = state.lookahead_point(
            contract.mapper.lookahead_distance_m
        )
        nominal_action, planner_facts = planner.nominal_action(
            state=point,
            active_obstacles=active_obstacles,
        )
        try:
            filter_result = filter_dynamic_road_action(
                state=point,
                nominal_action=nominal_action,
                geometry=scenario.geometry,
                active_obstacles=active_obstacles,
                spec=contract.lifted_cbf,
                additional_inflation_m=(
                    contract.additional_body_inflation_m
                ),
                additional_constraints=(
                    contract.mapper.reachable_velocity_constraints(
                        heading_rad=state.heading_rad
                    )
                ),
            )
            mapping_result = contract.mapper.command(
                filter_result.filtered_action,
                state=state,
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
                    mapping_result=None,
                    transition_noise_m=None,
                    successor=None,
                    realized_contraction_residuals=None,
                    termination_reason=termination_reason,
                )
            )
            break
        if not contract.system.action_is_admissible(
            mapping_result.action,
            tolerance=contract.lifted_cbf.feasibility_tolerance,
        ):
            raise RuntimeError("projection violated differential bounds")
        half_width = contract.system.noise.half_width_m
        noise_array = noise_rng.triangular(
            -half_width,
            0.0,
            half_width,
            size=2,
        )
        noise = (float(noise_array[0]), float(noise_array[1]))
        successor = contract.system.successor(
            state,
            mapping_result.action,
            noise,
            feasibility_tolerance=(
                contract.lifted_cbf.feasibility_tolerance
            ),
        )
        contraction_residuals = _realized_contraction_residuals(
            current_filter=filter_result,
            successor=successor,
            active_obstacles=active_obstacles,
            scenario=scenario,
            contract=contract,
        )
        row = _frame(
            step_index=step_index,
            state=state,
            scenario=scenario,
            contract=contract,
            active_obstacles=active_obstacles,
            newly_activated_ids=newly_active,
            planner_facts=planner_facts,
            filter_result=filter_result,
            mapping_result=mapping_result,
            transition_noise_m=noise,
            successor=successor,
            realized_contraction_residuals=contraction_residuals,
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
            remaining = (
                transition_count * contract.system.dt_s
                - (perf_counter() - start_time)
            )
            if remaining > 0.0:
                sleep(remaining)
    else:
        termination_reason = "timeout"

    if not rows or rows[-1]["termination_reason"] is None:
        if termination_reason is None:
            termination_reason = _terminal_reason(
                state,
                scenario,
            ) or "timeout"
        newly_active = _activate_visible_obstacles(
            scenario,
            state.center,
            active_ids,
        )
        active_obstacles = _active_obstacles(scenario, active_ids)
        rows.append(
            _frame(
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
                        contract.base.geometry.lane_centers_y_m[
                            planner.target_lane_index
                        ]
                    ),
                    "passed_obstacles_ignored_for_lane_selection": (
                        contract.nominal_controller
                        .ignore_passed_obstacles_for_lane_selection
                    ),
                },
                filter_result=None,
                mapping_result=None,
                transition_noise_m=None,
                successor=None,
                realized_contraction_residuals=None,
                termination_reason=termination_reason,
            )
        )
    return rows, perf_counter() - start_time


def differential_road_episode_metrics(
    rows: list[Mapping[str, Any]],
) -> dict[str, Any]:
    action_rows = [row for row in rows if row["action"] is not None]
    activated_ids = {
        obstacle_id
        for row in rows
        for obstacle_id in row["obstacles"]["active_ids"]
    }

    def values(path: tuple[str, ...]) -> list[float]:
        result: list[float] = []
        for row in action_rows:
            value: Any = row
            for key in path:
                value = value[key]
            if value is not None:
                result.append(float(value))
        return result

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
    residuals = values(
        ("certificate", "minimum_constraint_residual_mps")
    )
    contraction_residuals = values(
        (
            "certificate",
            "minimum_realized_contraction_residual_m",
        )
    )
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
        "minimum_lifted_certificate_barrier_m": min(
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
        "minimum_realized_contraction_residual_m": min(
            contraction_residuals,
            default=None,
        ),
        "minimum_planned_physical_clearance_m": min(
            float(row["state"]["minimum_planned_physical_clearance_m"])
            for row in rows
        ),
        "minimum_lifted_physical_clearance_m": min(
            float(row["state"]["minimum_lifted_physical_clearance_m"])
            for row in rows
        ),
        "maximum_mapping_residual_mps": max(
            values(("action", "mapping_residual_mps")),
            default=0.0,
        ),
        "maximum_linear_speed_mps": max(
            values(
                (
                    "action",
                    "differential_command",
                    "linear_mps",
                )
            ),
            default=0.0,
        ),
        "maximum_absolute_angular_speed_radps": max(
            (
                abs(value)
                for value in values(
                    (
                        "action",
                        "differential_command",
                        "angular_radps",
                    )
                )
            ),
            default=0.0,
        ),
        "maximum_filter_intervention_mps": max(
            values(("action", "intervention_norm_mps")),
            default=0.0,
        ),
        "filter_intervention_step_count": sum(
            float(row["action"]["intervention_norm_mps"]) > 1.0e-12
            for row in action_rows
        ),
        "command_saturation_step_count": sum(
            bool(row["action"]["command_saturation_applied"])
            for row in action_rows
        ),
    }


def _acceptance(
    *,
    rows: list[Mapping[str, Any]],
    metrics: Mapping[str, Any],
    contract: DifferentialRoadContract,
    trace_sha256: str,
    expected_trace_sha256: str | None,
) -> dict[str, Any]:
    spec = contract.acceptance
    residual = metrics["minimum_cbf_constraint_residual_mps"]
    contraction = metrics["minimum_realized_contraction_residual_m"]
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
        "lifted_certificate_barrier": (
            metrics["minimum_lifted_certificate_barrier_m"]
            >= spec.minimum_lifted_barrier_m
        ),
        "cbf_constraint_residual": (
            residual is not None
            and residual >= spec.minimum_constraint_residual_mps
        ),
        "realized_contraction_residual": (
            contraction is not None
            and contraction
            >= spec.minimum_realized_contraction_residual_m
        ),
        "mapping_residual": (
            metrics["maximum_mapping_residual_mps"]
            <= spec.maximum_mapping_residual_mps
        ),
        "linear_command_bound": (
            metrics["maximum_linear_speed_mps"]
            <= contract.system.max_linear_mps
            + contract.lifted_cbf.feasibility_tolerance
        ),
        "angular_command_bound": (
            metrics["maximum_absolute_angular_speed_radps"]
            <= contract.system.max_angular_radps
            + contract.lifted_cbf.feasibility_tolerance
        ),
        "no_post_projection_command_clipping": (
            metrics["command_saturation_step_count"] == 0
            and all(
                not row["runtime"][
                    "post_projection_command_clipping_applied"
                ]
                for row in rows
            )
        ),
    }
    replay_match = (
        None
        if expected_trace_sha256 is None
        else trace_sha256 == expected_trace_sha256
    )
    if not all(gates.values()) or replay_match is False:
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


def simulate_differential_road_cbf(
    *,
    contract: DifferentialRoadContract,
    output_dir: str | Path,
    seed: int | None = None,
    wall_clock: bool = True,
    emit_console: bool = False,
    render_gif: bool = True,
    display_stride: int | None = None,
    expected_trace_sha256: str | None = None,
) -> Path:
    """Execute, render, hash, and classify one differential-road run."""

    actual_seed = contract.default_seed if seed is None else int(seed)
    scenario = generate_seeded_road_scenario(
        contract.base,
        seed=actual_seed,
    )
    destination = Path(output_dir).resolve()
    destination.mkdir(parents=True, exist_ok=False)
    plan = obstacle_plan_payload(scenario=scenario, seed=actual_seed)
    plan_path = destination / "obstacle_plan.json"
    _write_json(plan_path, plan)

    rows, wall_elapsed_s = run_differential_road_episode(
        contract=contract,
        scenario=scenario,
        seed=actual_seed,
        wall_clock=wall_clock,
        emit_console=emit_console,
    )
    trace_path = destination / "live_trace.jsonl"
    write_trace(trace_path, rows)
    trace_sha = file_sha256(trace_path)
    metrics = differential_road_episode_metrics(rows)
    acceptance = _acceptance(
        rows=rows,
        metrics=metrics,
        contract=contract,
        trace_sha256=trace_sha,
        expected_trace_sha256=expected_trace_sha256,
    )

    animation_path = destination / "differential_road_cbf_1x.gif"
    final_frame_path = destination / "differential_road_cbf_final.png"
    stride = (
        contract.display_stride
        if display_stride is None
        else int(display_stride)
    )
    if stride <= 0:
        raise ValueError("display_stride must be positive")
    if render_gif:
        from .differential_road_render import (
            render_differential_road_animation,
        )

        render_differential_road_animation(
            rows,
            scenario=scenario,
            gif_path=animation_path,
            final_frame_path=final_frame_path,
            dt_s=contract.system.dt_s,
            display_stride=stride,
        )

    manifest: dict[str, Any] = {
        "schema_version": 1,
        "kind": "dynamic_road_differential_cbf_run",
        "experiment_id": contract.experiment_id,
        "status": acceptance["status"],
        "termination_reason": rows[-1]["termination_reason"],
        "claim_boundary": {
            "lifted_online_distance_cbf": (
                "runtime_evaluated_action_filter"
            ),
            "differential_drive_tracking": (
                "exact_for_declared_Euler_model"
            ),
            "trajectory": "one_seeded_stochastic_sample",
            "random_geometry_q_sbc": "not_claimed",
            "gazebo_tracking_claim": "not_claimed",
            "real_vehicle_claim": "forbidden",
        },
        "contract": {
            "config_path": str(contract.config_path),
            "config_file_sha256": contract.config_file_sha256,
            "base_scenario_config_path": str(
                contract.base.config_path
            ),
            "base_scenario_config_file_sha256": (
                contract.base.config_file_sha256
            ),
        },
        "scenario": {
            "scenario_id": scenario.scenario_id,
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
            "backend": "M2_exact_stochastic_differential_drive_euler",
            "certificate_mode": "lifted_online_action_filter",
            "lookahead_distance_m": (
                contract.mapper.lookahead_distance_m
            ),
            "heading_remainder_bound_m": (
                contract.heading_remainder_bound_m
            ),
            "effective_position_support_half_width_m": (
                contract.lifted_cbf.noise_half_width_m
            ),
            "post_projection_command_clipping_applied": False,
        },
        "render": (
            {
                "playback_rate": "1x_simulated_time",
                "display_stride": stride,
                "gif_file": animation_path.name,
                "gif_file_sha256": file_sha256(animation_path),
                "final_frame_file": final_frame_path.name,
                "final_frame_file_sha256": file_sha256(
                    final_frame_path
                ),
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
