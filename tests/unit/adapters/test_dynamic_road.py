from __future__ import annotations

from pathlib import Path

from cbf_rover.adapters.exact.dynamic_road import (
    run_dynamic_road_episode,
)
from cbf_rover.adapters.exact.road_contract import (
    generate_seeded_road_scenario,
    load_road_experiment_contract,
    obstacle_plan_payload,
)
from cbf_rover.domain.planar import PlanarAction, PlanarState
from cbf_rover.methods.online_cbf import filter_dynamic_road_action


REPOSITORY_ROOT = Path(__file__).resolve().parents[3]
CONFIG_PATH = (
    REPOSITORY_ROOT
    / "configs"
    / "scenarios"
    / "dynamic_road_random_obstacles.yaml"
)


def test_seeded_obstacle_plan_is_deterministic_and_identity_bound() -> None:
    contract = load_road_experiment_contract(CONFIG_PATH)
    first = generate_seeded_road_scenario(contract, seed=1234)
    second = generate_seeded_road_scenario(contract, seed=1234)
    different = generate_seeded_road_scenario(contract, seed=1235)

    first_payload = obstacle_plan_payload(scenario=first, seed=1234)
    second_payload = obstacle_plan_payload(scenario=second, seed=1234)
    different_payload = obstacle_plan_payload(
        scenario=different,
        seed=1235,
    )
    assert first_payload == second_payload
    assert first_payload["content_sha256"] != different_payload[
        "content_sha256"
    ]


def test_obstacle_cbf_constraint_is_robust_to_all_noise_corners() -> None:
    contract = load_road_experiment_contract(CONFIG_PATH)
    scenario = generate_seeded_road_scenario(contract, seed=1234)
    obstacle = scenario.obstacles[0]
    state = PlanarState(
        obstacle.center.x_m - obstacle.certificate_radius_m - 0.20,
        obstacle.center.y_m,
    )
    result = filter_dynamic_road_action(
        state=state,
        nominal_action=PlanarAction(0.42, 0.0),
        geometry=scenario.geometry,
        active_obstacles=(obstacle,),
        spec=contract.cbf,
    )
    current_barrier = obstacle.certificate_barrier_m(state)
    lower_bound = (
        1.0 - contract.cbf.contraction_kappa
    ) * current_barrier
    half_width = contract.system.noise.half_width_m

    for noise_x in (-half_width, half_width):
        for noise_y in (-half_width, half_width):
            successor = contract.system.successor(
                state,
                result.filtered_action,
                (noise_x, noise_y),
            )
            assert (
                obstacle.certificate_barrier_m(successor)
                >= lower_bound - 1.0e-12
            )


def test_dynamic_road_episode_reaches_goal_without_safety_breach() -> None:
    contract = load_road_experiment_contract(CONFIG_PATH)
    scenario = generate_seeded_road_scenario(
        contract,
        seed=contract.default_seed,
    )

    first_rows, _ = run_dynamic_road_episode(
        contract=contract,
        scenario=scenario,
        seed=contract.default_seed,
        wall_clock=False,
        emit_console=False,
    )
    replay_rows, _ = run_dynamic_road_episode(
        contract=contract,
        scenario=scenario,
        seed=contract.default_seed,
        wall_clock=False,
        emit_console=False,
    )

    assert first_rows == replay_rows
    assert first_rows[-1]["termination_reason"] == "goal_reached"
    assert first_rows[-1]["obstacles"]["active_count"] == len(
        scenario.obstacles
    )
    assert all(not row["state"]["collision_ids"] for row in first_rows)
    assert all(not row["state"]["road_departure"] for row in first_rows)
    assert min(
        row["certificate"]["minimum_barrier_m"] for row in first_rows
    ) >= -1.0e-9
    assert min(
        row["certificate"]["minimum_constraint_residual_mps"]
        for row in first_rows
        if row["certificate"]["minimum_constraint_residual_mps"] is not None
    ) >= -1.0e-9
    assert sum(
        row["runtime"]["certificate_modified_action"]
        for row in first_rows
    ) >= 1
    assert all(
        not row["runtime"]["post_filter_clipping_applied"]
        for row in first_rows
    )
