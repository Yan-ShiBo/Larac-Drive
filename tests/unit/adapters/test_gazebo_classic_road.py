from __future__ import annotations

import csv
from pathlib import Path
from xml.etree import ElementTree

import pytest
from PIL import Image

from cbf_rover.adapters.exact.differential_road_render import (
    render_differential_road_animation,
)
from cbf_rover.adapters.exact.road_contract import (
    file_sha256,
    generate_seeded_road_scenario,
)
from cbf_rover.adapters.exact.dynamic_road import write_trajectory_csv
from cbf_rover.adapters.gazebo.classic_bridge import (
    GazeboClassicSession,
    GazeboClassicState,
    parse_bridge_state,
)
from cbf_rover.adapters.gazebo.differential_road import (
    _gazebo_acceptance,
    _start_gazebo_session_with_attempts,
    _trajectory_telemetry,
    gazebo_differential_road_metrics,
    run_gazebo_differential_road_episode,
)
from cbf_rover.adapters.gazebo.differential_road_contract import (
    load_gazebo_differential_road_contract,
)
from cbf_rover.adapters.gazebo.differential_road_world import (
    render_gazebo_classic_world,
)
from cbf_rover.domain.differential_drive import DifferentialDriveState


ROOT = Path(__file__).resolve().parents[3]
CONFIG = ROOT / "configs/experiments/dynamic_road_gazebo_classic_v6.yaml"
CONFIG_V11 = (
    ROOT / "configs/experiments/dynamic_road_gazebo_classic_v11.yaml"
)
CONFIG_V12 = (
    ROOT / "configs/experiments/dynamic_road_gazebo_classic_v12.yaml"
)
CONFIG_V13 = (
    ROOT / "configs/experiments/dynamic_road_gazebo_classic_v13.yaml"
)
CONFIG_V14 = (
    ROOT / "configs/experiments/dynamic_road_gazebo_classic_v14.yaml"
)
CONFIG_V15 = (
    ROOT / "configs/experiments/dynamic_road_gazebo_classic_v15.yaml"
)
CONFIG_V16 = (
    ROOT / "configs/experiments/dynamic_road_gazebo_classic_v16.yaml"
)


class ExactGazeboStepper:
    def __init__(self, contract) -> None:
        start = contract.base.base.geometry.start
        self._contract = contract
        self._state = DifferentialDriveState(
            start.x_m,
            start.y_m,
            contract.base.initial_heading_rad,
        )
        self.initial_state = GazeboClassicState(
            sim_time_s=0.001,
            iterations=1,
            x_m=self._state.x_m,
            y_m=self._state.y_m,
            z_m=0.0,
            roll_rad=0.0,
            pitch_rad=0.0,
            yaw_rad=self._state.heading_rad,
        )
        self._measurement = self.initial_state

    def step(
        self,
        *,
        linear_mps: float,
        angular_radps: float,
    ) -> GazeboClassicState:
        from cbf_rover.domain.differential_drive import (
            DifferentialDriveAction,
        )

        self._state = self._contract.base.system.successor(
            self._state,
            DifferentialDriveAction(linear_mps, angular_radps),
            (0.0, 0.0),
            feasibility_tolerance=(
                self._contract.base.lifted_cbf.feasibility_tolerance
            ),
        )
        self._measurement = GazeboClassicState(
            sim_time_s=(
                self._measurement.sim_time_s
                + self._contract.runtime.control_period_s
            ),
            iterations=(
                self._measurement.iterations
                + self._contract.runtime.control_iterations_per_step
            ),
            x_m=self._state.x_m,
            y_m=self._state.y_m,
            z_m=0.0,
            roll_rad=0.0,
            pitch_rad=0.0,
            yaw_rad=self._state.heading_rad,
        )
        return self._measurement


def test_gazebo_contract_uses_isolated_fixed_step_runtime() -> None:
    contract = load_gazebo_differential_road_contract(CONFIG)

    assert contract.runtime.required_major_version == 11
    assert contract.phase == "confirmation"
    assert contract.runtime.master_uri == "http://127.0.0.1:11466"
    assert contract.runtime.solver_iterations == 40
    assert contract.runtime.plugin_angular_command_sign == -1.0
    assert (
        contract.runtime.state_plugin
        == "libcbf_rover_gazebo_state_plugin.so"
    )
    assert (
        contract.runtime.bridge_pose_world_timestamp_alignment
        == "required"
    )
    assert contract.runtime.control_period_s == pytest.approx(0.05)
    assert contract.live_display is None
    assert contract.rover.body_planar_half_diagonal_m < 0.31
    assert contract.rover.wheel_axle_x_m == pytest.approx(0.10)
    assert contract.tracking_envelope is not None
    assert (
        contract.tracking_envelope.position_coordinate_half_width_m
        == pytest.approx(0.04)
    )
    assert (
        contract.effective_lifted_cbf.noise_half_width_m
        > contract.base.lifted_cbf.noise_half_width_m
    )


def test_gazebo_session_pins_transport_to_loopback(tmp_path: Path) -> None:
    contract = load_gazebo_differential_road_contract(CONFIG)
    session = GazeboClassicSession(
        runtime=contract.runtime,
        world_path=tmp_path / "world.sdf",
        bridge_binary=tmp_path / "build" / "bridge",
        log_dir=tmp_path / "logs",
    )

    assert session.environment["GAZEBO_MASTER_URI"] == (
        "http://127.0.0.1:11466"
    )
    assert session.environment["GAZEBO_IP"] == "127.0.0.1"
    assert session.environment[
        "CBF_ROVER_GAZEBO_STARTUP_TIMEOUT_S"
    ] == "40.0"


def test_v11_tracking_envelope_is_empirical_and_self_gating() -> None:
    contract = load_gazebo_differential_road_contract(CONFIG_V11)
    envelope = contract.tracking_envelope

    assert envelope is not None
    assert envelope.position_coordinate_half_width_m == pytest.approx(0.016)
    assert envelope.heading_error_bound_rad == pytest.approx(0.055)
    assert contract.acceptance.maximum_position_tracking_residual_m == (
        pytest.approx(envelope.position_coordinate_half_width_m)
    )
    assert contract.acceptance.maximum_heading_tracking_residual_rad == (
        pytest.approx(envelope.heading_error_bound_rad)
    )
    assert contract.effective_lifted_cbf.noise_half_width_m == pytest.approx(
        0.005421809086151014
        + 0.016
        + contract.tracking_lookahead_displacement_bound_m
    )


def test_v12_startup_retries_are_bounded_and_pre_control_only(
    tmp_path: Path,
    monkeypatch,
) -> None:
    contract = load_gazebo_differential_road_contract(CONFIG_V12)
    instances = []

    class FakeSession:
        def __init__(self, **kwargs) -> None:
            self.log_dir = kwargs["log_dir"]
            self.closed = False
            instances.append(self)

        def start(self) -> GazeboClassicState:
            if len(instances) == 1:
                raise TimeoutError("synthetic namespace failure")
            return GazeboClassicState(
                sim_time_s=0.001,
                iterations=1,
                x_m=0.5,
                y_m=-0.58,
                z_m=0.0,
                roll_rad=0.0,
                pitch_rad=0.0,
                yaw_rad=0.0,
            )

        def close(self) -> None:
            self.closed = True

    monkeypatch.setattr(
        "cbf_rover.adapters.gazebo.differential_road."
        "GazeboClassicSession",
        FakeSession,
    )
    monkeypatch.setattr(
        "cbf_rover.adapters.gazebo.differential_road.sleep",
        lambda _: None,
    )

    camera_source = tmp_path / "camera_raw" / "front_camera.bmp"
    camera_queue = camera_source.parent / "queue"
    camera_queue.mkdir(parents=True)
    camera_source.write_bytes(b"failed-attempt latest frame")
    camera_source.with_name("front_camera.bmp.tmp").write_bytes(
        b"failed-attempt latest writer buffer"
    )
    (camera_queue / "frame_000000001.bmp").write_bytes(
        b"failed-attempt complete queued frame"
    )
    (camera_queue / "frame_000000002.bmp.tmp").write_bytes(
        b"failed-attempt queued writer buffer"
    )
    unrelated = camera_queue / "observer_notes.tmp"
    unrelated.write_bytes(b"must remain fail-closed")

    session, records = _start_gazebo_session_with_attempts(
        contract=contract,
        world_path=tmp_path / "world.sdf",
        bridge_path=tmp_path / "bridge",
        destination=tmp_path,
        gzserver_binary="gzserver",
        camera_source_path=camera_source,
        camera_queue_directory=camera_queue,
    )

    assert contract.runtime.startup_attempt_limit == 3
    assert contract.runtime.startup_retry_backoff_s == pytest.approx(1.0)
    assert len(instances) == 2
    assert instances[0].closed
    assert not instances[1].closed
    assert session is instances[1]
    assert [record["status"] for record in records] == [
        "failed_before_initial_state",
        "ready",
    ]
    assert all(not record["control_frames_started"] for record in records)
    residue = records[0]["camera_residue"]
    assert residue["policy"] == (
        "archive_exact_failed_attempt_camera_outputs_before_retry"
    )
    assert residue["archived_file_count"] == 4
    assert residue["archived_temporary_file_count"] == 2
    assert all(
        (tmp_path / item["archive_file"]).is_file()
        for item in residue["files"]
    )
    assert not camera_source.exists()
    assert not camera_source.with_name("front_camera.bmp.tmp").exists()
    assert not list(camera_queue.glob("frame_*.bmp"))
    assert not list(camera_queue.glob("frame_*.bmp.tmp"))
    assert unrelated.is_file()
    session.close()


def test_v13_heading_envelope_is_rounded_from_full_discovery() -> None:
    contract = load_gazebo_differential_road_contract(CONFIG_V13)
    envelope = contract.tracking_envelope

    assert envelope is not None
    assert envelope.position_coordinate_half_width_m == pytest.approx(0.016)
    assert envelope.heading_error_bound_rad == pytest.approx(0.065)
    assert contract.acceptance.maximum_heading_tracking_residual_rad == (
        pytest.approx(envelope.heading_error_bound_rad)
    )
    assert envelope.source_trace_sha256 == (
        "c34e1a76009c193c6e1603ab8b9feb7049b7c76cdc183c40c8fc0ea383091913"
    )


def test_v14_adds_indoor_camera_without_changing_control_contract() -> None:
    v13 = load_gazebo_differential_road_contract(CONFIG_V13)
    v14 = load_gazebo_differential_road_contract(CONFIG_V14)

    assert v14.visual_scene.layout == "indoor_room"
    assert v14.runtime.headless_renderer == "xvfb"
    assert v14.front_camera is not None
    assert v14.front_camera.image_width == 960
    assert v14.front_camera.image_height == 540
    assert v14.front_camera.pose_on_base_link == pytest.approx(
        (0.20, 0.0, 0.14, 0.0, 0.14, 0.0)
    )
    assert v14.acceptance.require_front_camera_frames
    assert v14.front_camera.transport == (
        "atomic_bmp_with_embedded_gazebo_sim_timestamp_observer_only"
    )
    assert v14.live_display is not None
    assert v14.live_display.layout == (
        "camera_primary_map_secondary_telemetry_collapsed"
    )
    assert v14.live_display.camera_certificate_projection == (
        "timestamp_aligned_measured_pose_pinhole_ground_plane"
    )
    assert v14.acceptance.minimum_front_camera_capture_coverage_ratio == (
        pytest.approx(0.95)
    )
    assert v14.acceptance.maximum_front_camera_alignment_error_s == (
        pytest.approx(0.026)
    )
    assert v14.acceptance.require_front_camera_activation_keyframes
    assert v14.base.config_file_sha256 == v13.base.config_file_sha256
    assert v14.effective_lifted_cbf == v13.effective_lifted_cbf


def test_v15_adds_timestamped_video_without_changing_control_contract() -> None:
    v14 = load_gazebo_differential_road_contract(CONFIG_V14)
    v15 = load_gazebo_differential_road_contract(CONFIG_V15)

    assert v15.camera_recording is not None
    assert v15.camera_recording.capture_policy == (
        "every_unique_timestamp_aligned_camera_frame"
    )
    assert v15.camera_recording.archive_format == (
        "png_sequence_with_jsonl_index"
    )
    assert v15.camera_recording.playback_fps == pytest.approx(10.0)
    assert (v15.camera_recording.video_width, v15.camera_recording.video_height) == (
        1920,
        1080,
    )
    assert v15.camera_recording.encoder_backend == "gstreamer_x264"
    assert v15.acceptance.require_front_camera_recording
    assert v15.acceptance.require_front_camera_video
    assert v15.runtime.master_uri == "http://127.0.0.1:11572"
    assert v15.live_display is not None
    assert v15.live_display.port == 11571
    assert v15.base.config_file_sha256 == v14.base.config_file_sha256
    assert v15.effective_lifted_cbf == v14.effective_lifted_cbf


def test_v16_uses_sparse_obstacles_and_early_world_model_preview() -> None:
    contract = load_gazebo_differential_road_contract(CONFIG_V16)
    road = contract.base.base
    sample = road.obstacle_sampling

    assert sample.count == 3
    assert min(
        second - first
        for first, second in zip(
            sample.x_anchors_m,
            sample.x_anchors_m[1:],
        )
    ) >= 3.0
    assert sample.appearance_distance_range_m[0] >= 4.0
    assert contract.base.nominal_controller.lane_switch_lookahead_m >= 3.8
    assert contract.acceptance.required_obstacle_activations == 3
    assert contract.live_display is not None
    assert contract.live_display.preview_level_h_m == pytest.approx(0.30)


def test_v16_rebuilds_world_model_and_starts_avoidance_before_h_equals_q() -> None:
    contract = load_gazebo_differential_road_contract(CONFIG_V16)
    scenario = generate_seeded_road_scenario(
        contract.base.base,
        seed=contract.default_seed,
    )
    rows, _ = run_gazebo_differential_road_episode(
        contract=contract,
        scenario=scenario,
        stepper=ExactGazeboStepper(contract),
        emit_console=False,
    )

    action_rows = [row for row in rows if row["action"] is not None]
    assert action_rows
    scenario_obstacles = {
        obstacle.obstacle_id: obstacle for obstacle in scenario.obstacles
    }
    for row in action_rows:
        world_model = row["world_model"]
        solve = row["certificate"]["online_solve"]
        level = row["certificate"]["display_level"]
        obstacle_constraints = [
            fact
            for fact in row["certificate"]["constraints"]
            if fact["barrier_kind"] == "obstacle"
        ]

        assert world_model["update_index"] == row["step_index"]
        assert world_model["active_obstacle_ids"] == row["obstacles"][
            "active_ids"
        ]
        assert [item["obstacle_id"] for item in world_model["obstacles"]] == (
            row["obstacles"]["active_ids"]
        )
        for item in world_model["obstacles"]:
            obstacle = scenario_obstacles[item["obstacle_id"]]
            safety_radius = (
                obstacle.certificate_radius_m
                + contract.base.additional_body_inflation_m
            )
            assert item["center_m"] == pytest.approx(obstacle.center.as_tuple())
            assert item["safety_boundary_radius_m"] == pytest.approx(
                safety_radius
            )
            assert item["display_contour_radius_m"] == pytest.approx(
                safety_radius + 0.30
            )
        assert world_model["rebuilt_from_measured_pose_each_step"] is True
        assert world_model["gazebo_sim_time_s"] == pytest.approx(
            row["state"]["gazebo_measurement"]["gazebo_sim_time_s"]
        )
        assert solve["step_index"] == row["step_index"]
        assert solve["world_model_update_index"] == world_model["update_index"]
        assert solve["solver"] == (
            "exact_2d_projection_onto_halfspaces_intersect_l2_disk"
        )
        assert solve["status"] == "feasible_exact_projection"
        assert solve["post_solve_clipping_applied"] is False
        assert solve["active_obstacle_constraint_count"] == len(
            obstacle_constraints
        )
        world_obstacles = {
            item["obstacle_id"]: item
            for item in world_model["obstacles"]
        }
        lookahead_x_m, lookahead_y_m = row["state"]["lookahead_point_m"]
        filtered = row["action"]["filtered_lookahead"]
        for fact in obstacle_constraints:
            assert fact["barrier_id"].startswith("obstacle:")
            obstacle_id = fact["barrier_id"].removeprefix("obstacle:")
            item = world_obstacles[obstacle_id]
            center_x_m, center_y_m = item["center_m"]
            expected_barrier_m = (
                (lookahead_x_m - center_x_m) ** 2
                + (lookahead_y_m - center_y_m) ** 2
            ) ** 0.5 - item["safety_boundary_radius_m"]
            expected_residual_mps = (
                fact["normal"][0] * filtered["u_x_mps"]
                + fact["normal"][1] * filtered["u_y_mps"]
                - fact["robust_rhs_mps"]
            )
            assert fact["barrier_m"] == pytest.approx(expected_barrier_m)
            assert fact["constraint_residual_mps"] == pytest.approx(
                expected_residual_mps
            )
        assert level == {
            "symbol": "q_h",
            "value_m": pytest.approx(0.30),
            "level_set": "h_t(x) = q_h",
            "safe_boundary": "h_t(x) = 0",
            "semantics": "runtime_distance_cbf_level_not_qsbc_probability",
            "observer_only": True,
        }
        assert row["certificate"]["q_sbc_probability_bound_claimed"] is False

    initial_lane = 0
    first_switch = next(
        row
        for row in action_rows
        if row["planner"]["target_lane_index"] != initial_lane
    )
    active_barriers = [
        fact["barrier_m"]
        for fact in first_switch["certificate"]["constraints"]
        if fact["barrier_kind"] == "obstacle"
    ]
    assert active_barriers
    assert min(active_barriers) > 0.30
    assert first_switch["action"]["nominal_lookahead"]["u_y_mps"] != 0.0
    assert rows[-1]["termination_reason"] == "goal_reached"


def test_v16_exports_full_rate_trajectory_csv(tmp_path: Path) -> None:
    contract = load_gazebo_differential_road_contract(CONFIG_V16)
    scenario = generate_seeded_road_scenario(
        contract.base.base,
        seed=contract.default_seed,
    )
    rows, _ = run_gazebo_differential_road_episode(
        contract=contract,
        scenario=scenario,
        stepper=ExactGazeboStepper(contract),
        emit_console=False,
    )

    trajectory_path = write_trajectory_csv(
        tmp_path / "trajectory.csv",
        rows,
    )
    with trajectory_path.open(encoding="utf-8", newline="") as stream:
        records = list(csv.DictReader(stream))

    assert len(records) == len(rows)
    assert [int(record["step_index"]) for record in records] == list(
        range(len(rows))
    )
    assert float(records[0]["x_m"]) == pytest.approx(
        rows[0]["state"]["x_m"]
    )
    assert float(records[-1]["simulated_time_s"]) == pytest.approx(
        rows[-1]["simulated_time_s"]
    )
    assert records[-1]["termination_reason"] == "goal_reached"
    action_records = [
        record for record in records if record["command_linear_mps"]
    ]
    assert action_records
    assert all(
        record["solve_status"] == "feasible_exact_projection"
        for record in action_records
    )
    assert records[-1]["solve_status"] == "terminal_state_no_action"
    assert {record["display_level_q_h_m"] for record in records} == {
        "0.3"
    }
    telemetry = _trajectory_telemetry(
        trajectory_path=trajectory_path,
        trace_path=tmp_path / "live_trace.jsonl",
        rows=rows,
    )
    assert telemetry["trajectory_file"] == "trajectory.csv"
    assert telemetry["trajectory_file_sha256"] == file_sha256(
        trajectory_path
    )
    assert telemetry["trajectory_format"] == (
        "csv_full_rate_every_control_frame"
    )
    assert telemetry["trajectory_sample_count"] == len(rows)
    assert telemetry["trajectory_source"] == "live_trace.jsonl"


def test_v16_static_animation_draws_h_equals_q_contour(
    tmp_path: Path,
) -> None:
    contract = load_gazebo_differential_road_contract(CONFIG_V16)
    scenario = generate_seeded_road_scenario(
        contract.base.base,
        seed=contract.default_seed,
    )
    rows, _ = run_gazebo_differential_road_episode(
        contract=contract,
        scenario=scenario,
        stepper=ExactGazeboStepper(contract),
        emit_console=False,
    )
    gif_path = tmp_path / "trajectory.gif"
    final_frame_path = tmp_path / "trajectory.png"

    render_differential_road_animation(
        rows,
        scenario=scenario,
        gif_path=gif_path,
        final_frame_path=final_frame_path,
        dt_s=contract.base.system.dt_s,
        display_stride=len(rows),
        title_backend="GAZEBO TEST",
        title_claim="ONLINE DISTANCE CBF",
    )

    assert gif_path.is_file()
    assert final_frame_path.is_file()
    with Image.open(final_frame_path) as image:
        cyan_pixels = [
            pixel
            for pixel in image.convert("RGB").get_flattened_data()
            if all(
                abs(channel - expected) <= 3
                for channel, expected in zip(pixel, (67, 199, 217))
            )
        ]
    assert cyan_pixels


def test_v14_world_contains_room_and_front_rgb_sensor(tmp_path: Path) -> None:
    contract = load_gazebo_differential_road_contract(CONFIG_V14)
    scenario = generate_seeded_road_scenario(
        contract.base.base,
        seed=contract.default_seed,
    )
    camera_path = tmp_path / "front_camera.bmp"

    world = render_gazebo_classic_world(
        contract=contract,
        scenario=scenario,
        camera_output_path=camera_path,
    )
    root = ElementTree.fromstring(world)
    models = {
        element.attrib["name"]
        for element in root.findall("./world/model")
    }
    sensor = root.find(
        "./world/model[@name='cbf_rover']/link[@name='base_link']"
        "/sensor[@name='front_rgb']"
    )

    assert "indoor_room_shell" in models
    assert sensor is not None
    assert sensor.attrib["type"] == "camera"
    assert sensor.findtext("camera/image/width") == "960"
    assert sensor.findtext("camera/image/height") == "540"
    camera_plugin = sensor.find("plugin")
    assert camera_plugin is not None
    assert camera_plugin.attrib["filename"] == (
        "libcbf_rover_gazebo_camera_plugin.so"
    )
    assert camera_plugin.findtext("output_path") == str(camera_path)


def test_gazebo_world_is_self_contained_and_projects_obstacles() -> None:
    contract = load_gazebo_differential_road_contract(CONFIG)
    scenario = generate_seeded_road_scenario(
        contract.base.base,
        seed=contract.default_seed,
    )

    world = render_gazebo_classic_world(
        contract=contract,
        scenario=scenario,
    )
    root = ElementTree.fromstring(world)
    models = {
        element.attrib["name"]
        for element in root.findall("./world/model")
    }

    assert contract.runtime.model_name in models
    assert "ground" in models
    assert {obstacle.obstacle_id for obstacle in scenario.obstacles} <= models
    assert "model://" not in world
    plugin = root.find(
        f"./world/model[@name='{contract.runtime.model_name}']/plugin"
    )
    assert plugin is not None
    assert plugin.attrib["filename"] == "libDiffDrivePlugin.so"
    state_plugin = root.find(
        f"./world/model[@name='{contract.runtime.model_name}']"
        "/plugin[@name='cbf_rover_timestamped_state']"
    )
    assert state_plugin is not None
    assert (
        state_plugin.attrib["filename"]
        == "libcbf_rover_gazebo_state_plugin.so"
    )


def test_bridge_state_protocol_parser_is_strict() -> None:
    state = parse_bridge_state(
        "STATE 1.25 1250 2.0 -0.5 0.1 0.01 -0.02 0.3"
    )

    assert state.sim_time_s == pytest.approx(1.25)
    assert state.iterations == 1250
    assert state.yaw_rad == pytest.approx(0.3)
    with pytest.raises(ValueError, match="unexpected"):
        parse_bridge_state("STATE 1 2 3")


def test_measured_state_loop_preserves_exact_zero_residual_baseline() -> None:
    contract = load_gazebo_differential_road_contract(CONFIG)
    scenario = generate_seeded_road_scenario(
        contract.base.base,
        seed=contract.default_seed,
    )
    stepper = ExactGazeboStepper(contract)
    observed_rows = []

    rows, _ = run_gazebo_differential_road_episode(
        contract=contract,
        scenario=scenario,
        stepper=stepper,
        emit_console=False,
        frame_observer=observed_rows.append,
    )
    metrics = gazebo_differential_road_metrics(rows)
    acceptance = _gazebo_acceptance(
        rows=rows,
        metrics=metrics,
        contract=contract,
    )

    assert rows[-1]["termination_reason"] == "goal_reached"
    assert metrics["collision_count"] == 0
    assert metrics["road_departure_count"] == 0
    assert metrics["maximum_position_tracking_residual_m"] == pytest.approx(
        0.0
    )
    assert (
        metrics["maximum_absolute_heading_tracking_residual_rad"]
        == pytest.approx(0.0)
    )
    assert observed_rows == rows
    assert observed_rows[-1]["termination_reason"] == "goal_reached"
    assert acceptance["status"] == "provisional_success"
