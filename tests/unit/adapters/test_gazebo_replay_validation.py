from __future__ import annotations

import json
from pathlib import Path

from cbf_rover.adapters.exact.road_contract import (
    canonical_content_sha256,
    file_sha256,
    generate_seeded_road_scenario,
    obstacle_plan_payload,
)
from cbf_rover.adapters.gazebo.differential_road_contract import (
    load_gazebo_differential_road_contract,
)
from cbf_rover.adapters.gazebo.replay_validation import (
    load_gazebo_replay_validation_spec,
    validate_gazebo_replay,
)
from cbf_rover.adapters.gazebo.run_render import (
    render_gazebo_differential_road_run,
)


ROOT = Path(__file__).resolve().parents[3]
CONFIG = (
    ROOT
    / "configs/experiments/dynamic_road_gazebo_classic_replay_v2.yaml"
)
RUN_CONFIG = (
    ROOT / "configs/experiments/dynamic_road_gazebo_classic_v6.yaml"
)


def _write_run(
    directory: Path,
    *,
    x_m: float,
    y_m: float,
    heading_rad: float,
    position_residual_m: float,
    trace_tag: str,
    status: str = "provisional_success",
    config_sha256: str = "a" * 64,
    obstacle_plan_content_sha256: str = "c" * 64,
    experiment_id: str = "GZ2_DYNAMIC_ROAD_LIFTED_CBF_CONFIRMATION_V6",
) -> Path:
    directory.mkdir()
    obstacle_plan = directory / "obstacle_plan.json"
    obstacle_plan.write_text('{"seed":7321609}\n', encoding="utf-8")
    world = directory / "dynamic_road_gazebo.world"
    world.write_text("<sdf version='1.6'/>\n", encoding="utf-8")
    trace = directory / "live_trace.jsonl"
    trace.write_text(
        json.dumps(
            {
                "tag": trace_tag,
                "termination_reason": "goal_reached",
                "state": {
                    "x_m": x_m,
                    "y_m": y_m,
                    "heading_rad": heading_rad,
                },
            },
            sort_keys=True,
            separators=(",", ":"),
        )
        + "\n",
        encoding="utf-8",
    )
    startup_attempts = directory / "startup_attempts.json"
    startup_attempts.write_text(
        '{"attempt_count":1,"selected_attempt":1,"status":"ready"}\n',
        encoding="utf-8",
    )
    payload = {
        "schema_version": 1,
        "kind": "dynamic_road_gazebo_classic_cbf_run",
        "experiment_id": experiment_id,
        "status": status,
        "termination_reason": "goal_reached",
        "contract": {
            "config_file_sha256": config_sha256,
            "base_differential_config_file_sha256": "b" * 64,
        },
        "scenario": {
            "scenario_id": "DR1_TWO_LANE_RANDOM_OBSTACLES_V1",
            "seed": 7321609,
            "obstacle_plan_file": obstacle_plan.name,
            "obstacle_plan_file_sha256": file_sha256(obstacle_plan),
            "obstacle_plan_content_sha256": obstacle_plan_content_sha256,
            "world_file": world.name,
            "world_file_sha256": file_sha256(world),
        },
        "runtime": {
            "backend": "Gazebo Classic 11",
            "physics_engine": "ode",
            "physics_step_s": 0.001,
            "control_period_s": 0.05,
            "paused_fixed_step": True,
        },
        "startup": {
            "attempts_file": startup_attempts.name,
            "attempts_file_sha256": file_sha256(startup_attempts),
            "attempt_count": 1,
            "selected_attempt": 1,
            "retry_scope": "before_initial_state_only",
            "control_retries_after_initial_state": 0,
        },
        "telemetry": {
            "trace_file": trace.name,
            "trace_file_sha256": file_sha256(trace),
            "transition_count": 840,
            "gazebo_simulated_duration_s": 42.0,
            "minimum_lifted_certificate_barrier_m": 0.30,
            "minimum_cbf_constraint_residual_mps": 0.0,
            "minimum_realized_contraction_residual_m": 0.05,
            "maximum_position_tracking_residual_m": position_residual_m,
            "maximum_absolute_heading_tracking_residual_rad": 0.076,
        },
        "acceptance": {
            "gates": {
                "goal_reached": status == "provisional_success",
                "collision_count": True,
            }
        },
    }
    payload["manifest_content_sha256"] = canonical_content_sha256(payload)
    manifest = directory / "run_manifest.json"
    manifest.write_text(
        json.dumps(
            payload,
            indent=2,
            sort_keys=True,
            allow_nan=False,
        )
        + "\n",
        encoding="utf-8",
    )
    return manifest


def test_gazebo_replay_accepts_bounded_nonidentical_traces(
    tmp_path: Path,
) -> None:
    spec = load_gazebo_replay_validation_spec(CONFIG)
    first = _write_run(
        tmp_path / "run_a",
        x_m=20.01,
        y_m=0.50,
        heading_rad=0.01,
        position_residual_m=0.021,
        trace_tag="first",
    )
    second = _write_run(
        tmp_path / "run_b",
        x_m=20.02,
        y_m=0.51,
        heading_rad=0.02,
        position_residual_m=0.031,
        trace_tag="second",
    )

    report_path = validate_gazebo_replay(
        spec=spec,
        run_manifest_paths=(first, second),
        output_dir=tmp_path / "validation",
    )
    report = json.loads(report_path.read_text(encoding="utf-8"))

    assert report["status"] == "accepted"
    assert all(report["gates"].values())
    assert not report["aggregate"]["byte_identical_trace_replay"]
    assert report["aggregate"]["final_center_position_spread_m"] < 0.05


def test_gazebo_replay_rejects_a_failed_member(tmp_path: Path) -> None:
    spec = load_gazebo_replay_validation_spec(CONFIG)
    first = _write_run(
        tmp_path / "run_a",
        x_m=20.01,
        y_m=0.50,
        heading_rad=0.01,
        position_residual_m=0.021,
        trace_tag="first",
    )
    second = _write_run(
        tmp_path / "run_b",
        x_m=20.02,
        y_m=0.51,
        heading_rad=0.02,
        position_residual_m=0.041,
        trace_tag="second",
        status="failed",
    )

    report_path = validate_gazebo_replay(
        spec=spec,
        run_manifest_paths=(first, second),
        output_dir=tmp_path / "validation",
    )
    report = json.loads(report_path.read_text(encoding="utf-8"))

    assert report["status"] == "failed"
    assert not report["gates"]["each_run_provisional_success"]
    assert not report["gates"]["each_run_all_gates"]


def test_gazebo_replay_rejects_tampered_startup_evidence(
    tmp_path: Path,
) -> None:
    spec = load_gazebo_replay_validation_spec(CONFIG)
    first = _write_run(
        tmp_path / "run_a",
        x_m=20.01,
        y_m=0.50,
        heading_rad=0.01,
        position_residual_m=0.021,
        trace_tag="first",
    )
    second = _write_run(
        tmp_path / "run_b",
        x_m=20.02,
        y_m=0.51,
        heading_rad=0.02,
        position_residual_m=0.031,
        trace_tag="second",
    )
    (first.parent / "startup_attempts.json").write_text(
        '{"tampered":true}\n',
        encoding="utf-8",
    )

    report_path = validate_gazebo_replay(
        spec=spec,
        run_manifest_paths=(first, second),
        output_dir=tmp_path / "validation",
    )
    report = json.loads(report_path.read_text(encoding="utf-8"))

    assert report["status"] == "failed"
    assert not report["gates"]["referenced_file_hashes"]


def test_gazebo_render_verifies_source_before_rendering(
    tmp_path: Path,
    monkeypatch,
) -> None:
    contract = load_gazebo_differential_road_contract(RUN_CONFIG)
    scenario = generate_seeded_road_scenario(
        contract.base.base,
        seed=contract.default_seed,
    )
    plan = obstacle_plan_payload(
        scenario=scenario,
        seed=contract.default_seed,
    )
    source_manifest = _write_run(
        tmp_path / "run",
        x_m=13.30,
        y_m=0.58,
        heading_rad=0.0,
        position_residual_m=0.021,
        trace_tag="measured-gazebo",
        config_sha256=contract.config_file_sha256,
        obstacle_plan_content_sha256=plan["content_sha256"],
    )
    rendered: dict[str, object] = {}

    def fake_render(rows, **kwargs) -> None:
        rendered["rows"] = rows
        kwargs["gif_path"].write_bytes(b"gif")
        kwargs["final_frame_path"].write_bytes(b"png")

    monkeypatch.setattr(
        "cbf_rover.adapters.gazebo.run_render."
        "render_differential_road_animation",
        fake_render,
    )

    report_path = render_gazebo_differential_road_run(
        contract=contract,
        run_manifest_path=source_manifest,
        output_dir=tmp_path / "render",
    )
    report = json.loads(report_path.read_text(encoding="utf-8"))

    assert report["status"] == "rendered"
    assert report["render"]["playback_rate"] == "1x_simulated_time"
    assert report["source"]["scenario_seed"] == contract.default_seed
    assert rendered["rows"]
