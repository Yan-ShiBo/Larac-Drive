from __future__ import annotations

from math import cos, isclose, sin
from pathlib import Path

from cbf_rover.adapters.exact.differential_road import (
    differential_road_episode_metrics,
    run_differential_road_episode,
)
from cbf_rover.adapters.exact.differential_road_contract import (
    load_differential_road_contract,
)
from cbf_rover.adapters.exact.road_contract import (
    generate_seeded_road_scenario,
)
from cbf_rover.domain.differential_drive import DifferentialDriveState
from cbf_rover.domain.planar import PlanarAction


REPOSITORY_ROOT = Path(__file__).resolve().parents[3]
CONFIG_V1_PATH = (
    REPOSITORY_ROOT
    / "configs"
    / "experiments"
    / "dynamic_road_differential.yaml"
)
CONFIG_V2_PATH = (
    REPOSITORY_ROOT
    / "configs"
    / "experiments"
    / "dynamic_road_differential_v2.yaml"
)
CONFIG_V3_PATH = (
    REPOSITORY_ROOT
    / "configs"
    / "experiments"
    / "dynamic_road_differential_v3.yaml"
)


def test_lookahead_mapping_round_trip_satisfies_reachable_halfspaces() -> None:
    contract = load_differential_road_contract(CONFIG_V2_PATH)
    state = DifferentialDriveState(1.0, -0.4, 0.31)
    linear = 0.37
    angular = -0.82
    distance = contract.mapper.lookahead_distance_m
    desired = PlanarAction(
        linear * cos(state.heading_rad)
        - distance * angular * sin(state.heading_rad),
        linear * sin(state.heading_rad)
        + distance * angular * cos(state.heading_rad),
    )

    result = contract.mapper.command(desired, state=state)

    assert isclose(result.action.linear_mps, linear, abs_tol=1.0e-15)
    assert isclose(
        result.action.angular_radps,
        angular,
        abs_tol=1.0e-15,
    )
    assert result.mapping_residual_mps <= 1.0e-15
    assert all(
        constraint.residual(desired) >= -1.0e-12
        for constraint in contract.mapper.reachable_velocity_constraints(
            heading_rad=state.heading_rad
        )
    )
    assert not result.saturation_applied


def test_lifted_contract_adds_heading_remainder_to_n1_support() -> None:
    contract = load_differential_road_contract(CONFIG_V2_PATH)

    assert contract.heading_remainder_bound_m > 0.0
    assert isclose(
        contract.lifted_cbf.noise_half_width_m,
        (
            contract.system.noise.half_width_m
            + contract.heading_remainder_bound_m
        ),
        abs_tol=1.0e-15,
    )
    assert (
        contract.additional_body_inflation_m
        == contract.mapper.lookahead_distance_m
    )


def test_v1_failure_remains_reproducible_and_safe() -> None:
    contract = load_differential_road_contract(CONFIG_V1_PATH)
    scenario = generate_seeded_road_scenario(
        contract.base,
        seed=contract.default_seed,
    )

    rows, _ = run_differential_road_episode(
        contract=contract,
        scenario=scenario,
        seed=contract.default_seed,
        wall_clock=False,
        emit_console=False,
    )
    metrics = differential_road_episode_metrics(rows)

    assert rows[-1]["termination_reason"] == "cbf_action_set_infeasible"
    assert metrics["collision_count"] == 0
    assert metrics["road_departure_count"] == 0
    assert metrics["minimum_lifted_certificate_barrier_m"] >= 0.0


def test_v2_discovery_seed_is_deterministic_and_safe() -> None:
    contract = load_differential_road_contract(CONFIG_V2_PATH)
    scenario = generate_seeded_road_scenario(
        contract.base,
        seed=contract.default_seed,
    )

    first_rows, _ = run_differential_road_episode(
        contract=contract,
        scenario=scenario,
        seed=contract.default_seed,
        wall_clock=False,
        emit_console=False,
    )
    replay_rows, _ = run_differential_road_episode(
        contract=contract,
        scenario=scenario,
        seed=contract.default_seed,
        wall_clock=False,
        emit_console=False,
    )
    metrics = differential_road_episode_metrics(first_rows)

    assert first_rows == replay_rows
    assert first_rows[-1]["termination_reason"] == "goal_reached"
    assert metrics["activated_obstacle_count"] == len(scenario.obstacles)
    assert metrics["collision_count"] == 0
    assert metrics["road_departure_count"] == 0
    assert metrics["minimum_lifted_certificate_barrier_m"] >= -1.0e-9
    assert metrics["minimum_cbf_constraint_residual_mps"] >= -1.0e-9
    assert (
        metrics["minimum_realized_contraction_residual_m"]
        >= -1.0e-9
    )
    assert metrics["maximum_mapping_residual_mps"] <= 1.0e-12
    assert metrics["command_saturation_step_count"] == 0


def test_v3_ignores_passed_obstacles_and_resolves_gazebo_deadlock_seeds() -> None:
    contract = load_differential_road_contract(CONFIG_V3_PATH)

    for seed in (7320014, 7320004):
        scenario = generate_seeded_road_scenario(contract.base, seed=seed)
        rows, _ = run_differential_road_episode(
            contract=contract,
            scenario=scenario,
            seed=seed,
            wall_clock=False,
            emit_console=False,
        )
        metrics = differential_road_episode_metrics(rows)

        assert rows[-1]["termination_reason"] == "goal_reached"
        assert metrics["collision_count"] == 0
        assert metrics["road_departure_count"] == 0
        assert metrics["minimum_lifted_certificate_barrier_m"] >= -1.0e-9
        assert metrics["minimum_cbf_constraint_residual_mps"] >= -1.0e-9
        assert metrics["minimum_realized_contraction_residual_m"] >= -1.0e-9
        assert all(
            row["planner"] is None
            or row["planner"][
                "passed_obstacles_ignored_for_lane_selection"
            ]
            for row in rows
        )
