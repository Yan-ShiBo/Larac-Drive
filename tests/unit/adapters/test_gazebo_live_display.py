from __future__ import annotations

import json
from pathlib import Path
import struct
from urllib.request import urlopen

import pytest
import yaml

from cbf_rover.adapters.gazebo.differential_road import (
    _apply_camera_recording_video_acceptance,
    _apply_live_display_acceptance,
)
from cbf_rover.adapters.gazebo.differential_road_contract import (
    load_gazebo_differential_road_contract,
)
from cbf_rover.adapters.gazebo.live_display import GazeboLiveDisplay
from cbf_rover.domain.dynamic_road import (
    DynamicRoadScenario,
    RoadGeometry,
    RoadObstacle,
)
from cbf_rover.domain.planar import PlanarState


ROOT = Path(__file__).resolve().parents[3]
CONFIG = ROOT / "configs/experiments/dynamic_road_gazebo_classic_v7.yaml"
CONFIG_V8 = (
    ROOT / "configs/experiments/dynamic_road_gazebo_classic_v8.yaml"
)
CONFIG_V14 = (
    ROOT / "configs/experiments/dynamic_road_gazebo_classic_v14.yaml"
)
CONFIG_V15 = (
    ROOT / "configs/experiments/dynamic_road_gazebo_classic_v15.yaml"
)


def _bmp(
    width: int,
    height: int,
    *,
    seconds: int = 0,
    nanoseconds: int = 0,
    frame_counter: int = 1,
) -> bytes:
    row_stride = (width * 3 + 3) & ~3
    pixel_bytes = row_stride * height
    metadata = (
        f"\nCBF_ROVER_CAMERA_FRAME_V1 {seconds} {nanoseconds} "
        f"{frame_counter}\n"
    ).encode("ascii")
    file_bytes = 54 + pixel_bytes + len(metadata)
    header = struct.pack("<2sIHHI", b"BM", file_bytes, 0, 0, 54)
    dib = struct.pack(
        "<IIIHHIIIIII",
        40,
        width,
        height,
        1,
        24,
        0,
        pixel_bytes,
        2835,
        2835,
        0,
        0,
    )
    return header + dib + bytes(pixel_bytes) + metadata


def _scenario() -> DynamicRoadScenario:
    geometry = RoadGeometry(
        x_min_m=0.0,
        x_max_m=14.0,
        physical_y_min_m=-1.35,
        physical_y_max_m=1.35,
        lane_centers_y_m=(-0.58, 0.58),
        start=PlanarState(0.5, -0.58),
        finish_line_x_m=13.25,
        rover_radius_m=0.31,
    )
    obstacle = RoadObstacle(
        obstacle_id="obstacle_00",
        center=PlanarState(4.0, -0.58),
        lane_index=0,
        physical_radius_m=0.16,
        safety_margin_m=0.05,
        rover_radius_m=geometry.rover_radius_m,
        appearance_distance_m=3.0,
    )
    return DynamicRoadScenario(
        scenario_id="unit-live-display",
        geometry=geometry,
        obstacles=(obstacle,),
    )


def _row(
    *,
    step_index: int,
    active: bool,
    terminal: bool = False,
    newly_activated: bool = False,
) -> dict[str, object]:
    time_s = step_index * 0.05
    x_m = 0.5 + step_index * 0.02
    active_ids = ["obstacle_00"] if active else []
    constraints: list[dict[str, object]] = [
        {
            "barrier_id": "road:lower",
            "barrier_kind": "road_boundary",
            "barrier_m": 0.4,
            "normal": [0.0, 1.0],
            "robust_rhs_mps": -1.0,
            "constraint_residual_mps": 0.2,
            "realized_contraction_residual_m": 0.01,
        }
    ]
    if active:
        constraints.append(
            {
                "barrier_id": "obstacle:obstacle_00",
                "barrier_kind": "obstacle",
                "barrier_m": 0.32,
                "normal": [-1.0, 0.0],
                "robust_rhs_mps": -0.6,
                "constraint_residual_mps": 0.1,
                "realized_contraction_residual_m": 0.02,
            }
        )
    return {
        "step_index": step_index,
        "simulated_time_s": time_s,
        "termination_reason": "goal_reached" if terminal else None,
        "state": {
            "x_m": x_m,
            "y_m": -0.58,
            "heading_rad": 0.0,
            "lookahead_point_m": [x_m + 0.15, -0.58],
        },
        "obstacles": {
            "active_ids": active_ids,
            "newly_activated_ids": active_ids if newly_activated else [],
            "active_count": len(active_ids),
            "planned_count": 1,
        },
        "certificate": {
            "certificate_id": "unit-cbf",
            "family": "lifted_online_robust_distance_cbf",
            "additional_body_inflation_m": 0.15,
            "minimum_barrier_m": 0.32 if active else 0.40,
            "minimum_active_obstacle_barrier_m": 0.32 if active else None,
            "minimum_constraint_residual_mps": 0.1,
            "minimum_realized_contraction_residual_m": 0.01,
            "constraints": constraints,
        },
        "action": (
            None
            if terminal
            else {
                "differential_command": {
                    "linear_mps": 0.42,
                    "angular_radps": 0.0,
                },
                "intervention_norm_mps": 0.03 if active else 0.0,
            }
        ),
        "runtime": {
            "backend": "Gazebo_Classic_11_ODE_paused_fixed_step",
            "certificate_mode": "measured_state_lifted_online_action_filter",
        },
    }


def test_v7_declares_isolated_observer_only_live_display() -> None:
    payload = yaml.safe_load(CONFIG.read_text(encoding="utf-8"))
    contract = load_gazebo_differential_road_contract(CONFIG)

    display = payload["live_display"]
    assert contract.runtime.master_uri == "http://127.0.0.1:11468"
    assert contract.live_display is not None
    assert contract.live_display.port == 11467
    assert contract.live_display.display_stride == 4
    assert contract.live_display.display_sink_control_feedback is False
    assert display["enabled"] is True
    assert display["bind_host"] == "127.0.0.1"
    assert display["port"] == 11467
    assert display["display_stride"] == 4
    assert display["wall_clock_paced"] is True
    assert display["display_sink_control_feedback"] is False


def test_v8_moves_transport_optimization_to_new_isolated_ports() -> None:
    payload = yaml.safe_load(CONFIG_V8.read_text(encoding="utf-8"))
    contract = load_gazebo_differential_road_contract(CONFIG_V8)

    assert (
        contract.experiment_id
        == "GZ2_DYNAMIC_ROAD_LIFTED_CBF_LIVE_DISPLAY_V8"
    )
    assert contract.phase == "validation"
    assert contract.runtime.master_uri == "http://127.0.0.1:11470"
    assert contract.runtime.control_iterations_per_step == 50
    assert contract.runtime.physics_step_s == 0.001
    assert contract.runtime.solver_iterations == 40
    assert contract.live_display is not None
    assert contract.live_display.port == 11469
    assert contract.live_display.wall_clock_paced is True
    assert (
        payload["claims"]["bridge_step_completion"]
        == "direct_timestamped_pose_iteration_without_world_stats_gate"
    )


def test_live_display_serves_incremental_certificate_frames(
    tmp_path: Path,
) -> None:
    display = GazeboLiveDisplay(
        directory=tmp_path / "live_display",
        scenario=_scenario(),
        display_stride=4,
        port=0,
    )
    url = display.start()
    manifest_path: Path | None = None
    try:
        with urlopen(url, timeout=2.0) as response:
            viewer = response.read().decode("utf-8")
        assert "Rover live obstacle certificate" in viewer
        assert "display is an observer only" in viewer

        display.publish(_row(step_index=0, active=False))
        display.publish(
            _row(
                step_index=1,
                active=True,
                newly_activated=True,
            )
        )
        display.publish(_row(step_index=4, active=True))
        display.publish(_row(step_index=5, active=True, terminal=True))

        with urlopen(f"{url}snapshot.json", timeout=2.0) as response:
            snapshot = json.loads(response.read())
        assert snapshot["status"] == "completed"
        assert snapshot["frame"]["step_index"] == 5
        assert snapshot["frame"]["termination_reason"] == "goal_reached"
        assert snapshot["frame"]["runtime"]["display_sink_control_feedback"] is False
        assert snapshot["scenario"]["obstacles"][0]["active"] is True
        assert snapshot["scenario"]["obstacles"][0]["barrier_m"] == 0.32
        assert [item["time_s"] for item in snapshot["history"]] == [
            0.0,
            0.2,
            0.25,
        ]
    finally:
        manifest_path = display.close()

    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    events = [
        json.loads(line)
        for line in display.events_path.read_text(encoding="utf-8").splitlines()
    ]
    assert manifest["status"] == "completed"
    assert manifest["update_count"] == 3
    assert manifest["maximum_snapshot_bytes"] < 100_000
    assert manifest["semantics"]["display_sink_control_feedback"] is False
    assert len(events) == 3
    assert [event["step_index"] for event in events] == [0, 4, 5]
    assert events[1]["newly_activated_ids"] == ["obstacle_00"]
    assert all(
        event["display_sink_control_feedback"] is False
        for event in events
    )


def test_v14_live_display_copies_camera_without_control_feedback(
    tmp_path: Path,
) -> None:
    contract = load_gazebo_differential_road_contract(CONFIG_V14)
    assert contract.front_camera is not None
    camera_source = tmp_path / "camera_raw" / "front_camera.bmp"
    camera_source.parent.mkdir()
    camera_source.write_bytes(
        _bmp(
            contract.front_camera.image_width,
            contract.front_camera.image_height,
        )
    )
    display = GazeboLiveDisplay(
        directory=tmp_path / "live_display",
        scenario=_scenario(),
        display_stride=4,
        port=0,
        camera_source_path=camera_source,
        camera_spec=contract.front_camera,
        rover_spec=contract.rover,
        state_sample_period_s=contract.base.system.dt_s,
    )
    url = display.start()
    try:
        display.publish(
            _row(step_index=0, active=True, newly_activated=True)
        )
        with urlopen(f"{url}snapshot.json", timeout=2.0) as response:
            snapshot = json.loads(response.read())
        with urlopen(url, timeout=2.0) as response:
            viewer = response.read().decode("utf-8")
        assert snapshot["camera"]["available"] is True
        assert snapshot["camera"]["frame_version"] == 1
        assert snapshot["camera"]["frame_file"] == "front_camera.png"
        assert snapshot["camera"]["observer_only"] is True
        assert snapshot["camera"]["pitch_rad"] == 0.14
        assert snapshot["camera"]["projection"] == (
            "timestamp_aligned_measured_pose_pinhole_ground_plane"
        )
        assert snapshot["camera"]["frame_counter"] == 1
        assert snapshot["camera"]["capture_simulated_time_s"] == 0.0
        assert snapshot["camera"]["alignment"]["aligned_step_index"] == 0
        assert snapshot["camera"]["alignment"]["absolute_time_error_s"] == 0.0
        assert "FRONT RGB / CERTIFICATE PROJECTION" in viewer
        assert "telemetryDetails" in viewer
        with urlopen(f"{url}front_camera.png", timeout=2.0) as response:
            assert response.read(8) == b"\x89PNG\r\n\x1a\n"
    finally:
        display.close()

    manifest = display.manifest_payload()
    assert manifest["camera"]["update_count"] == 1
    assert manifest["camera"]["invalid_frame_count"] == 0
    assert manifest["camera"]["aligned_update_count"] == 1
    assert manifest["camera"]["maximum_alignment_error_s"] == 0.0
    assert manifest["camera"]["final_frame_available"] is True
    assert len(manifest["camera"]["activation_keyframes"]) == 1
    keyframe = manifest["camera"]["activation_keyframes"][0]
    assert keyframe["obstacle_ids"] == ["obstacle_00"]
    assert keyframe["activation_step_index_by_obstacle"] == {
        "obstacle_00": 0
    }
    assert keyframe["camera_alignment"]["active_obstacle_ids"] == [
        "obstacle_00"
    ]
    assert (display.directory / keyframe["frame_file"]).is_file()
    assert (display.directory / keyframe["snapshot_file"]).is_file()
    assert manifest["semantics"]["front_camera_observer_only"] is True


def test_v15_records_unique_camera_frames_before_display_stride(
    tmp_path: Path,
) -> None:
    contract = load_gazebo_differential_road_contract(CONFIG_V15)
    assert contract.front_camera is not None
    assert contract.camera_recording is not None
    camera_source = tmp_path / "camera_raw" / "front_camera.bmp"
    camera_source.parent.mkdir()
    camera_queue = camera_source.parent / "queue"
    camera_queue.mkdir()
    display = GazeboLiveDisplay(
        directory=tmp_path / "live_display",
        scenario=_scenario(),
        display_stride=4,
        port=0,
        camera_source_path=camera_source,
        camera_queue_directory=camera_queue,
        camera_spec=contract.front_camera,
        rover_spec=contract.rover,
        state_sample_period_s=contract.base.system.dt_s,
        camera_recording_spec=contract.camera_recording,
    )
    display.start()
    try:
        for step_index in range(5):
            if step_index % 2 == 0:
                queued_frame = camera_queue / (
                    f"frame_{step_index // 2 + 1:09d}.bmp"
                )
                queued_frame.write_bytes(
                    _bmp(
                        contract.front_camera.image_width,
                        contract.front_camera.image_height,
                        nanoseconds=step_index * 50_000_000,
                        frame_counter=step_index // 2 + 1,
                    )
                )
            display.publish(
                _row(
                    step_index=step_index,
                    active=True,
                    terminal=step_index == 4,
                    newly_activated=step_index == 0,
                )
            )
    finally:
        display.close()

    manifest = display.manifest_payload()
    recording = manifest["camera"]["recording"]
    assert recording is not None
    payload = recording["manifest"]
    assert payload["status"] == "completed"
    assert payload["frame_count"] == 3
    assert payload["first_frame_counter"] == 1
    assert payload["last_frame_counter"] == 3
    assert payload["missing_frame_count"] == 0
    assert payload["maximum_source_frame_gap_s"] == pytest.approx(0.1)
    assert payload["capture_coverage_ratio"] == 1.0
    assert payload["camera_queue_consumed_frame_count"] == 3
    assert payload["camera_queue_remaining_frame_count"] == 0
    assert payload["camera_queue_temporary_file_count"] == 0
    assert all(payload["gates"].values())
    assert not list(camera_queue.iterdir())
    records = [
        json.loads(line)
        for line in display.camera_recording_index_path.read_text(
            encoding="utf-8"
        ).splitlines()
    ]
    assert [record["alignment"]["aligned_step_index"] for record in records] == [
        0,
        2,
        4,
    ]
    assert all(
        (display.camera_recording_directory / record["frame_file"]).is_file()
        for record in records
    )


def test_v7_live_display_acceptance_is_part_of_the_run_gate() -> None:
    contract = load_gazebo_differential_road_contract(CONFIG)
    live_manifest = {
        "status": "completed",
        "display_stride": 4,
        "refresh_interval_s": 0.20,
        "update_count": 3,
        "maximum_snapshot_bytes": 50_000,
        "final_snapshot": {
            "step_index": 5,
            "simulated_time_s": 0.25,
        },
        "semantics": {
            "display_sink_control_feedback": False,
            "display_sink_action_modification": False,
            "online_filter_precedes_display_observation": True,
        },
        "transport": {
            "bind_host": "127.0.0.1",
            "requested_port": 11467,
        },
    }
    result = _apply_live_display_acceptance(
        acceptance={
            "status": "provisional_success",
            "gates": {"physical_baseline": True},
        },
        live_manifest=live_manifest,
        metrics={
            "transition_count": 5,
            "gazebo_simulated_duration_s": 0.25,
        },
        contract=contract,
        wall_clock_paced=True,
    )

    assert result["status"] == "provisional_success"
    assert len(result["gates"]) == 6
    assert all(result["gates"].values())
    assert result["live_display"]["expected_update_count"] == 3

    missing = _apply_live_display_acceptance(
        acceptance={
            "status": "provisional_success",
            "gates": {"physical_baseline": True},
        },
        live_manifest=None,
        metrics={
            "transition_count": 5,
            "gazebo_simulated_duration_s": 0.25,
        },
        contract=contract,
        wall_clock_paced=True,
    )
    assert missing["status"] == "failed"
    assert missing["gates"]["live_display_completed"] is False


def test_v14_camera_alignment_and_keyframes_are_acceptance_gates() -> None:
    contract = load_gazebo_differential_road_contract(CONFIG_V14)
    assert contract.live_display is not None
    keyframes = [
        {
            "obstacle_ids": [f"obstacle_{index:02d}"],
            "frame_file_sha256": "a" * 64,
            "snapshot_file_sha256": "b" * 64,
            "camera_alignment": {
                "absolute_time_error_s": 0.0,
                "active_obstacle_ids": [f"obstacle_{index:02d}"],
            },
        }
        for index in range(1, 6)
    ]
    live_manifest = {
        "status": "completed",
        "display_stride": 4,
        "refresh_interval_s": 0.20,
        "layout": "camera_primary_map_secondary_telemetry_collapsed",
        "camera_certificate_projection": (
            "timestamp_aligned_measured_pose_pinhole_ground_plane"
        ),
        "update_count": 6,
        "maximum_snapshot_bytes": 50_000,
        "final_snapshot": {
            "step_index": 20,
            "simulated_time_s": 1.0,
        },
        "semantics": {
            "display_sink_control_feedback": False,
            "display_sink_action_modification": False,
            "online_filter_precedes_display_observation": True,
            "front_camera_observer_only": True,
        },
        "transport": {
            "bind_host": "127.0.0.1",
            "requested_port": 11471,
        },
        "camera": {
            "configured": True,
            "update_count": 6,
            "aligned_update_count": 6,
            "invalid_frame_count": 0,
            "final_frame_available": True,
            "capture_coverage_ratio": 0.96,
            "maximum_alignment_error_s": 0.025,
            "activation_keyframes": keyframes,
            "pending_activation_keyframe_obstacle_ids": [],
        },
    }
    result = _apply_live_display_acceptance(
        acceptance={
            "status": "provisional_success",
            "gates": {"physical_baseline": True},
        },
        live_manifest=live_manifest,
        metrics={
            "transition_count": 20,
            "gazebo_simulated_duration_s": 1.0,
        },
        contract=contract,
        wall_clock_paced=True,
    )

    assert result["status"] == "provisional_success"
    assert all(result["gates"].values())

    live_manifest["camera"]["maximum_alignment_error_s"] = 0.027
    misaligned = _apply_live_display_acceptance(
        acceptance={
            "status": "provisional_success",
            "gates": {"physical_baseline": True},
        },
        live_manifest=live_manifest,
        metrics={
            "transition_count": 20,
            "gazebo_simulated_duration_s": 1.0,
        },
        contract=contract,
        wall_clock_paced=True,
    )
    assert misaligned["status"] == "failed"
    assert misaligned["gates"]["front_camera_timestamp_alignment"] is False


def test_v15_recording_and_video_are_separate_acceptance_gates() -> None:
    contract = load_gazebo_differential_road_contract(CONFIG_V15)
    live_manifest = {
        "camera": {
            "recording": {
                "manifest": {
                    "status": "completed",
                    "observer_only": True,
                    "frame_count": 3,
                    "capture_coverage_ratio": 0.99,
                    "maximum_source_frame_gap_s": 0.1,
                    "missing_frame_count": 0,
                    "gates": {"source_frame_hash_closure": True},
                }
            }
        }
    }
    video_manifest = {
        "status": "completed",
        "observer_only": True,
        "source_frame_count": 3,
        "rendered_frame_count": 3,
        "video_file": "gazebo_front_camera_certificate_1x.mp4",
        "video_file_sha256": "a" * 64,
        "encoded_duration_s": 0.3,
        "gates": {"video_file_present": True},
    }
    result = _apply_camera_recording_video_acceptance(
        acceptance={
            "status": "provisional_success",
            "gates": {"physical_baseline": True},
        },
        live_manifest=live_manifest,
        video_manifest=video_manifest,
        contract=contract,
    )

    assert result["status"] == "provisional_success"
    assert all(result["gates"].values())

    video_manifest["gates"]["video_file_present"] = False
    failed = _apply_camera_recording_video_acceptance(
        acceptance={
            "status": "provisional_success",
            "gates": {"physical_baseline": True},
        },
        live_manifest=live_manifest,
        video_manifest=video_manifest,
        contract=contract,
    )
    assert failed["status"] == "failed"
    assert failed["gates"]["front_camera_video_gates"] is False
