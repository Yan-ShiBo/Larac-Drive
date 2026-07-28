from __future__ import annotations

import pytest

from cbf_rover.experiments import EpisodeMetrics, InterventionMetrics


def test_intervention_metrics_track_rates_magnitudes_and_violations() -> None:
    metrics = InterventionMetrics()

    metrics.record(0.0, feasible=True)
    metrics.record(0.25, feasible=True, certificate_violations=1)
    metrics.record(0.75, feasible=False, certificate_violations=2)

    assert metrics.evaluated_steps == 3
    assert metrics.intervention_count == 2
    assert metrics.intervention_rate == pytest.approx(2 / 3)
    assert metrics.mean_intervention_norm == pytest.approx(0.5)
    assert metrics.max_intervention_norm == pytest.approx(0.75)
    assert metrics.infeasible_count == 1
    assert metrics.certificate_violation_count == 3


def test_empty_intervention_metrics_report_zero_rates() -> None:
    metrics = InterventionMetrics()

    assert metrics.intervention_rate == 0.0
    assert metrics.mean_intervention_norm == 0.0


def test_episode_metrics_keep_scenario_outcome_and_safety_counts_together() -> None:
    metrics = EpisodeMetrics(
        scenario_id="static_obstacles",
        seed=17,
        steps=120,
        success=True,
        collision_count=0,
        total_reward=8.5,
        total_cost=0.25,
        min_obstacle_distance_m=0.42,
        certificate_violation_count=1,
        intervention_count=7,
        mean_intervention_norm=0.08,
        termination_reason="goal_reached",
        extra={"world_id": "static_obstacles_v1"},
    )

    assert metrics.scenario_id == "static_obstacles"
    assert metrics.seed == 17
    assert metrics.collision_count == 0
    assert metrics.certificate_violation_count == 1
    assert metrics.extra == {"world_id": "static_obstacles_v1"}


@pytest.mark.parametrize(
    ("intervention_norm", "certificate_violations"),
    [
        (-0.01, 0),
        (0.0, -1),
    ],
)
def test_intervention_metrics_reject_negative_measurements(
    intervention_norm: float,
    certificate_violations: int,
) -> None:
    metrics = InterventionMetrics()

    with pytest.raises(ValueError, match="non-negative"):
        metrics.record(
            intervention_norm,
            feasible=True,
            certificate_violations=certificate_violations,
        )
