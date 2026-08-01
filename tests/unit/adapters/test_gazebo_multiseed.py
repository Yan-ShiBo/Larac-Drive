from __future__ import annotations

from pathlib import Path

from cbf_rover.adapters.gazebo.differential_road_contract import (
    load_gazebo_differential_road_contract,
)
from cbf_rover.adapters.gazebo.multiseed import (
    _aggregate_records,
    _lane_pattern,
    load_gazebo_road_batch_contract,
)


ROOT = Path(__file__).resolve().parents[3]
BATCH_CONFIG = (
    ROOT
    / "configs/experiments/dynamic_road_gazebo_classic_multiseed_v4.yaml"
)


def test_gazebo_multiseed_catalogs_are_frozen_and_stratified() -> None:
    batch = load_gazebo_road_batch_contract(BATCH_CONFIG)
    episode = load_gazebo_differential_road_contract(
        batch.gazebo_episode_config_path
    )
    discovery = batch.catalog("discovery")
    validation = batch.catalog("validation")
    expected_patterns = {f"{value:05b}" for value in range(32)}

    assert batch.experiment_id == "GZ5_DISCOVERY_CALIBRATED_MULTISEED_V4"
    assert episode.experiment_id == (
        "GZ5_DISCOVERY_CALIBRATED_AUDITED_STARTUP_CBF_LIVE_DISPLAY_V13"
    )
    assert batch.selection_rule == "first_ascending_seed_per_lane_pattern"
    assert not batch.selection_uses_metrics_values
    assert len(discovery.entries) == 32
    assert len(validation.entries) == 32
    assert not discovery.eligible_for_final_empirical_claim
    assert validation.eligible_for_final_empirical_claim
    assert not discovery.overlaps(validation)
    assert discovery.entries[0].seed == 7330037
    assert validation.entries[0].seed == 7331024
    assert {entry.lane_pattern for entry in discovery.entries} == (
        expected_patterns
    )
    assert {entry.lane_pattern for entry in validation.entries} == (
        expected_patterns
    )
    assert all(
        _lane_pattern(episode, entry.seed) == entry.lane_pattern
        for catalog in (discovery, validation)
        for entry in catalog.entries
    )
    assert episode.live_display is not None
    assert episode.live_display.enabled
    assert episode.live_display.wall_clock_paced
    assert episode.runtime.startup_timeout_s == 120.0
    assert episode.runtime.startup_attempt_limit == 3
    assert batch.acceptance.maximum_position_tracking_residual_m == 0.016
    assert batch.acceptance.maximum_heading_tracking_residual_rad == 0.065


def _record(index: int) -> dict[str, object]:
    return {
        "seed": 8_000_000 + index,
        "lane_pattern": f"{index:05b}",
        "status": "passed",
        "failure_reasons": [],
        "termination_reason": "goal_reached",
        "run_manifest": f"episodes/{index}/run_manifest.json",
        "run_manifest_file_sha256": "a" * 64,
        "collision_count": 0,
        "road_departure_count": 0,
        "minimum_lifted_certificate_barrier_m": 0.2 + index * 1.0e-4,
        "minimum_cbf_constraint_residual_mps": -1.0e-16,
        "minimum_measured_contraction_residual_m": 0.05,
        "minimum_planned_physical_clearance_m": 0.4,
        "minimum_lifted_physical_clearance_m": 0.35,
        "maximum_mapping_residual_mps": 0.0,
        "maximum_position_tracking_residual_m": 0.012,
        "maximum_heading_tracking_residual_rad": 0.04,
        "maximum_absolute_roll_rad": 0.001,
        "maximum_absolute_pitch_rad": 0.006,
        "maximum_linear_speed_mps": 0.46,
        "maximum_absolute_angular_speed_radps": 1.5,
        "achieved_realtime_factor": 0.999,
        "live_display_update_count": 200,
        "live_display_status": "completed",
        "live_display_observer_only": True,
        "wall_clock_paced": True,
        "startup_attempt_count": 1,
        "selected_startup_attempt": 1,
        "control_retries_after_initial_state": 0,
        "command_saturation_step_count": 0,
    }


def test_gazebo_multiseed_aggregate_accepts_all_patterns() -> None:
    batch = load_gazebo_road_batch_contract(BATCH_CONFIG)
    records = [_record(index) for index in range(32)]

    metrics, gates = _aggregate_records(
        records,
        acceptance=batch.acceptance,
    )

    assert all(gates.values())
    assert metrics["passed_episode_count"] == 32
    assert metrics["lane_pattern_count"] == 32
    assert metrics["total_live_display_updates"] == 6400
    assert metrics["total_startup_attempts"] == 32
    assert metrics["episodes_with_startup_retry"] == 0
    assert gates["no_control_retries_after_initial_state"]
    assert (
        metrics["largest_position_tracking_residual"]["seed"]
        == 8_000_031
    )


def test_gazebo_multiseed_aggregate_rejects_observer_failure() -> None:
    batch = load_gazebo_road_batch_contract(BATCH_CONFIG)
    records = [_record(index) for index in range(32)]
    records[7]["live_display_observer_only"] = False

    _, gates = _aggregate_records(
        records,
        acceptance=batch.acceptance,
    )

    assert not gates["live_display_each_episode"]
