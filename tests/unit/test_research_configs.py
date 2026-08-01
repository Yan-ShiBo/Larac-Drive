from __future__ import annotations

from pathlib import Path

import yaml


REPOSITORY_ROOT = Path(__file__).resolve().parents[2]


def test_all_versioned_yaml_files_parse() -> None:
    paths = sorted(
        [
            *REPOSITORY_ROOT.glob("configs/**/*.yaml"),
            *REPOSITORY_ROOT.glob("artifacts/**/*.yaml"),
            *REPOSITORY_ROOT.glob("simulation/**/*.yaml"),
        ]
    )
    assert paths
    for path in paths:
        payload = yaml.safe_load(path.read_text(encoding="utf-8"))
        assert payload is not None, path


def test_certificate_schema_union_keeps_global_active_and_p49_candidate() -> None:
    payload = yaml.safe_load(
        (
            REPOSITORY_ROOT / "artifacts" / "certificates" / "schema.yaml"
        ).read_text(encoding="utf-8")
    )
    compatibility = payload["compatibility"]
    assert payload["supported_certificate_schema_versions"] == [1, 2, 3]
    assert compatibility["active_global_certificate_schema_version"] == 2
    assert compatibility["p49_candidate_certificate_schema_version"] == 3

    p49 = payload["certificate_variants"][
        "schema_3_p49_piecewise_candidate"
    ]
    assert not p49["active_global_certificate_schema"]
    assert not p49["published_ready_first_slice_certificate_present"]
    assert (
        p49["barrier"]["partition_id"]
        == "P49_C2_OBSTACLE_ALIGNED_CARTESIAN_V1"
    )
    assert p49["barrier"]["required"] == [
        "schema_version",
        "kind",
        "partition_id",
        "variable_order",
        "basis",
        "degree_per_axis",
        "normalized_x_breaks_exact",
        "normalized_y_breaks_exact",
        "global_control_grid_shape",
        "global_control_dof_count",
        "global_control_grid_decimal",
        "global_control_grid_float64_hex",
        "continuity",
    ]


def test_rover_first_bindings_name_existing_files() -> None:
    path = REPOSITORY_ROOT / "configs" / "experiments" / "rover_first_qsbc.yaml"
    payload = yaml.safe_load(path.read_text(encoding="utf-8"))
    for relative in payload["bindings"].values():
        bound = (path.parent / relative).resolve()
        assert bound.is_file(), bound


def test_scenario_and_system_share_the_declared_time_contract() -> None:
    scenario = yaml.safe_load(
        (
            REPOSITORY_ROOT
            / "configs"
            / "scenarios"
            / "static_obstacles.yaml"
        ).read_text(encoding="utf-8")
    )
    system = yaml.safe_load(
        (
            REPOSITORY_ROOT
            / "configs"
            / "systems"
            / "m1_stochastic_single_integrator.yaml"
        ).read_text(encoding="utf-8")
    )
    assert system["dynamics"]["dt_s"] == 0.05
    assert scenario["episodes"]["max_steps"] == 2000
    assert scenario["episodes"]["horizon_s"] == 100.0
    assert not scenario["episodes"]["max_steps_is_formal_horizon"]


def test_dynamic_road_keeps_online_cbf_separate_from_static_qsbc() -> None:
    payload = yaml.safe_load(
        (
            REPOSITORY_ROOT
            / "configs"
            / "scenarios"
            / "dynamic_road_random_obstacles.yaml"
        ).read_text(encoding="utf-8")
    )

    assert payload["system"]["dt_s"] == 0.05
    assert payload["system"]["max_speed_mps"] == 0.5
    assert not payload["system"]["post_filter_clipping_allowed"]
    assert payload["online_cbf"]["modifies_action"]
    assert not payload["online_cbf"]["post_filter_clipping_allowed"]
    assert not payload["authority"]["static_qsbc_artifacts_are_reused"]
    assert payload["claims"]["q_sbc_probability_bound"] == "not_claimed"
    assert (
        payload["acceptance"]["minimum_filter_intervention_step_count"]
        >= 1
    )


def test_dynamic_road_multiseed_config_binds_existing_scenario() -> None:
    path = (
        REPOSITORY_ROOT
        / "configs"
        / "experiments"
        / "dynamic_road_multiseed.yaml"
    )
    payload = yaml.safe_load(path.read_text(encoding="utf-8"))
    scenario = (
        path.parent / payload["bindings"]["scenario_config"]
    ).resolve()

    assert scenario.is_file()
    assert payload["seed_catalogs"]["discovery"]["count"] == 256
    assert payload["seed_catalogs"]["validation"]["count"] == 1024
    assert payload["acceptance"]["required_lane_pattern_count"] == 32
    assert payload["claims"]["random_geometry_q_sbc"] == "not_claimed"


def test_differential_road_config_is_forward_only_and_not_gazebo() -> None:
    path = (
        REPOSITORY_ROOT
        / "configs"
        / "experiments"
        / "dynamic_road_differential.yaml"
    )
    payload = yaml.safe_load(path.read_text(encoding="utf-8"))
    scenario = (
        path.parent / payload["base_scenario_config"]
    ).resolve()

    assert scenario.is_file()
    assert payload["model"]["dt_s"] == 0.05
    assert not payload["model"]["allow_reverse"]
    assert payload["lookahead_mapping"]["lookahead_distance_m"] == 0.15
    assert (
        payload["lifted_online_cbf"]["additional_body_inflation_m"]
        == payload["lookahead_mapping"]["lookahead_distance_m"]
    )
    assert payload["claims"]["random_geometry_q_sbc"] == "not_claimed"
    assert payload["claims"]["gazebo_tracking_claim"] == "not_claimed"

    v2_path = (
        REPOSITORY_ROOT
        / "configs"
        / "experiments"
        / "dynamic_road_differential_v2.yaml"
    )
    v2_payload = yaml.safe_load(v2_path.read_text(encoding="utf-8"))
    assert (
        v2_payload["nominal_controller"]["lane_switch_lookahead_m"]
        == 2.4
    )
    assert (
        v2_payload["experiment"]["id"]
        == "DR2_LOOKAHEAD_DIFFERENTIAL_CBF_V2"
    )


def test_differential_multiseed_config_binds_v2() -> None:
    path = (
        REPOSITORY_ROOT
        / "configs"
        / "experiments"
        / "dynamic_road_differential_multiseed.yaml"
    )
    payload = yaml.safe_load(path.read_text(encoding="utf-8"))
    differential = (
        path.parent / payload["bindings"]["differential_config"]
    ).resolve()

    assert differential.is_file()
    assert payload["seed_catalogs"]["discovery"]["count"] == 256
    assert payload["seed_catalogs"]["validation"]["count"] == 1024
    assert payload["acceptance"]["required_lane_pattern_count"] == 32
    assert payload["claims"]["random_geometry_q_sbc"] == "not_claimed"
    assert payload["claims"]["gazebo_tracking_claim"] == "not_claimed"


def test_differential_v3_multiseed_uses_new_held_out_catalog() -> None:
    path = (
        REPOSITORY_ROOT
        / "configs"
        / "experiments"
        / "dynamic_road_differential_multiseed_v2.yaml"
    )
    payload = yaml.safe_load(path.read_text(encoding="utf-8"))
    differential = (
        path.parent / payload["bindings"]["differential_config"]
    ).resolve()
    differential_payload = yaml.safe_load(
        differential.read_text(encoding="utf-8")
    )

    assert differential.is_file()
    assert payload["seed_catalogs"]["discovery"]["start_inclusive"] == 7330000
    assert payload["seed_catalogs"]["validation"]["start_inclusive"] == 7331000
    assert payload["seed_catalogs"]["validation"]["count"] == 1024
    assert (
        differential_payload["nominal_controller"][
            "ignore_passed_obstacles_for_lane_selection"
        ]
        is True
    )
