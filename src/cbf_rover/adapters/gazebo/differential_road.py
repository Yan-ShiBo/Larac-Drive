"""Measured-state dynamic-road online CBF on Gazebo Classic."""

from __future__ import annotations

from math import hypot
import json
from pathlib import Path
from time import perf_counter, sleep
from typing import Any, Callable, Mapping, Protocol

from cbf_rover.adapters.exact.differential_road import (
    _active_obstacles,
    _activate_visible_obstacles,
    _frame,
    _realized_contraction_residuals,
    _terminal_reason,
    differential_road_episode_metrics,
)
from cbf_rover.adapters.exact.differential_road_render import (
    render_differential_road_animation,
)
from cbf_rover.adapters.exact.dynamic_road import LaneSupervisor, write_trace
from cbf_rover.adapters.exact.road_contract import (
    canonical_content_sha256,
    file_sha256,
    generate_seeded_road_scenario,
    obstacle_plan_payload,
)
from cbf_rover.domain.differential_drive import (
    DifferentialDriveState,
    wrap_heading,
)
from cbf_rover.domain.dynamic_road import DynamicRoadScenario
from cbf_rover.methods.online_cbf import (
    InfeasibleProjectionError,
    filter_dynamic_road_action,
)

from .classic_bridge import GazeboClassicSession, GazeboClassicState
from .differential_road_contract import GazeboDifferentialRoadContract
from .differential_road_world import render_gazebo_classic_world
from .live_display import GazeboLiveDisplay


class GazeboStepper(Protocol):
    initial_state: GazeboClassicState | None

    def step(
        self,
        *,
        linear_mps: float,
        angular_radps: float,
    ) -> GazeboClassicState: ...


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


def _remove_camera_writer_temporary_file(camera_path: Path) -> bool:
    """Remove only the camera writer's private buffer after gzserver exits."""

    temporary_path = camera_path.with_name(f"{camera_path.name}.tmp")
    if not temporary_path.exists():
        return False
    if not temporary_path.is_file():
        raise RuntimeError("camera writer temporary path is not a file")
    temporary_path.unlink()
    return True


def _startup_attempts_payload(
    *,
    destination: Path,
    records: list[dict[str, Any]],
    attempt_limit: int,
    selected_attempt: int | None,
    status: str,
) -> dict[str, Any]:
    attempts: list[dict[str, Any]] = []
    for source_record in records:
        record = dict(source_record)
        log_directory = destination / str(record["log_directory"])
        for key, name in (
            ("gzserver_log", "gzserver.log"),
            ("bridge_stderr_log", "bridge.stderr.log"),
        ):
            path = log_directory / name
            record[key] = {
                "file": str(path.relative_to(destination)),
                "file_sha256": file_sha256(path) if path.is_file() else None,
                "size_bytes": path.stat().st_size if path.is_file() else None,
            }
        attempts.append(record)
    payload: dict[str, Any] = {
        "schema_version": 1,
        "kind": "gazebo_classic_startup_attempts",
        "status": status,
        "attempt_limit": attempt_limit,
        "attempt_count": len(records),
        "selected_attempt": selected_attempt,
        "control_frames_before_selection": 0,
        "retry_scope": "before_initial_state_only",
        "attempts": attempts,
    }
    payload["content_sha256"] = canonical_content_sha256(payload)
    return payload


def _write_startup_attempts(
    *,
    destination: Path,
    records: list[dict[str, Any]],
    attempt_limit: int,
    selected_attempt: int | None,
    status: str,
) -> Path:
    path = destination / "startup_attempts.json"
    _write_json(
        path,
        _startup_attempts_payload(
            destination=destination,
            records=records,
            attempt_limit=attempt_limit,
            selected_attempt=selected_attempt,
            status=status,
        ),
    )
    return path


def _start_gazebo_session_with_attempts(
    *,
    contract: GazeboDifferentialRoadContract,
    world_path: Path,
    bridge_path: Path,
    destination: Path,
    gzserver_binary: str,
) -> tuple[GazeboClassicSession, list[dict[str, Any]]]:
    records: list[dict[str, Any]] = []
    for attempt_index in range(1, contract.runtime.startup_attempt_limit + 1):
        log_directory = (
            Path("runtime_attempts") / f"attempt_{attempt_index:03d}"
        )
        session = GazeboClassicSession(
            runtime=contract.runtime,
            world_path=world_path,
            bridge_binary=bridge_path,
            log_dir=destination / log_directory,
            gzserver_binary=gzserver_binary,
        )
        started = perf_counter()
        try:
            initial_state = session.start()
        except BaseException as error:
            session.close()
            record = {
                "attempt": attempt_index,
                "status": "failed_before_initial_state",
                "wall_elapsed_s": perf_counter() - started,
                "log_directory": str(log_directory),
                "error_type": type(error).__name__,
                "error_message": str(error),
                "control_frames_started": False,
            }
            records.append(record)
            _write_startup_attempts(
                destination=destination,
                records=records,
                attempt_limit=contract.runtime.startup_attempt_limit,
                selected_attempt=None,
                status=(
                    "failed"
                    if attempt_index == contract.runtime.startup_attempt_limit
                    else "retrying"
                ),
            )
            print(
                json.dumps(
                    {
                        "event": "gazebo_startup_attempt_failed",
                        "attempt": attempt_index,
                        "attempt_limit": (
                            contract.runtime.startup_attempt_limit
                        ),
                        "error_type": type(error).__name__,
                        "error_message": str(error),
                    },
                    sort_keys=True,
                ),
                flush=True,
            )
            if attempt_index == contract.runtime.startup_attempt_limit:
                raise
            sleep(contract.runtime.startup_retry_backoff_s)
            continue
        records.append(
            {
                "attempt": attempt_index,
                "status": "ready",
                "wall_elapsed_s": perf_counter() - started,
                "log_directory": str(log_directory),
                "initial_sim_time_s": initial_state.sim_time_s,
                "initial_iterations": initial_state.iterations,
                "control_frames_started": False,
            }
        )
        print(
            json.dumps(
                {
                    "event": "gazebo_startup_ready",
                    "attempt": attempt_index,
                    "attempt_limit": contract.runtime.startup_attempt_limit,
                    "initial_sim_time_s": initial_state.sim_time_s,
                    "initial_iterations": initial_state.iterations,
                },
                sort_keys=True,
            ),
            flush=True,
        )
        return session, records
    raise AssertionError("positive startup attempt limit exhausted")


def _planar_state(measurement: GazeboClassicState) -> DifferentialDriveState:
    return DifferentialDriveState(
        measurement.x_m,
        measurement.y_m,
        measurement.yaw_rad,
    )


def _gazebo_measurement_payload(
    measurement: GazeboClassicState,
    *,
    episode_start_time_s: float,
    episode_start_iterations: int,
) -> dict[str, Any]:
    return {
        "gazebo_sim_time_s": measurement.sim_time_s,
        "gazebo_iterations": measurement.iterations,
        "episode_simulated_time_s": (
            measurement.sim_time_s - episode_start_time_s
        ),
        "episode_physics_iterations": (
            measurement.iterations - episode_start_iterations
        ),
        "z_m": measurement.z_m,
        "roll_rad": measurement.roll_rad,
        "pitch_rad": measurement.pitch_rad,
        "yaw_rad": measurement.yaw_rad,
    }


def _patch_gazebo_frame(
    row: dict[str, Any],
    *,
    contract: GazeboDifferentialRoadContract,
    current_measurement: GazeboClassicState,
    episode_start_time_s: float,
    episode_start_iterations: int,
    predicted_successor: DifferentialDriveState | None,
    successor_measurement: GazeboClassicState | None,
) -> dict[str, Any]:
    row["kind"] = "dynamic_road_gazebo_classic_cbf_frame"
    row["experiment_id"] = contract.experiment_id
    row["simulated_time_s"] = (
        current_measurement.sim_time_s - episode_start_time_s
    )
    row["state"]["gazebo_measurement"] = _gazebo_measurement_payload(
        current_measurement,
        episode_start_time_s=episode_start_time_s,
        episode_start_iterations=episode_start_iterations,
    )
    row["runtime"] = {
        "backend": "Gazebo_Classic_11_ODE_paused_fixed_step",
        "certificate_mode": "measured_state_lifted_online_action_filter",
        "certificate_modified_lookahead_velocity": (
            row["action"] is not None
            and float(row["action"]["intervention_norm_mps"]) > 1.0e-12
        ),
        "post_projection_command_clipping_applied": False,
        "ros2_used": False,
        "tracking_error_bound_formally_established": False,
    }
    envelope = contract.tracking_envelope
    row["certificate"]["gazebo_tracking_envelope"] = (
        None
        if envelope is None
        else {
            "source_experiment_id": envelope.source_experiment_id,
            "source_trace_sha256": envelope.source_trace_sha256,
            "position_coordinate_half_width_m": (
                envelope.position_coordinate_half_width_m
            ),
            "heading_error_bound_rad": envelope.heading_error_bound_rad,
            "lookahead_displacement_bound_m": (
                contract.tracking_lookahead_displacement_bound_m
            ),
            "effective_position_support_half_width_m": (
                contract.effective_lifted_cbf.noise_half_width_m
            ),
            "formal_bound": False,
        }
    )
    transition = row.get("transition")
    if (
        transition is not None
        and predicted_successor is not None
        and successor_measurement is not None
    ):
        successor = _planar_state(successor_measurement)
        error_x = successor.x_m - predicted_successor.x_m
        error_y = successor.y_m - predicted_successor.y_m
        error_heading = wrap_heading(
            successor.heading_rad - predicted_successor.heading_rad
        )
        transition.pop("noise_m", None)
        transition["commanded_euler_successor"] = {
            "x_m": predicted_successor.x_m,
            "y_m": predicted_successor.y_m,
            "heading_rad": predicted_successor.heading_rad,
        }
        transition["gazebo_measurement"] = _gazebo_measurement_payload(
            successor_measurement,
            episode_start_time_s=episode_start_time_s,
            episode_start_iterations=episode_start_iterations,
        )
        transition["tracking_residual"] = {
            "x_m": error_x,
            "y_m": error_y,
            "position_norm_m": hypot(error_x, error_y),
            "heading_rad": error_heading,
        }
    return row


def run_gazebo_differential_road_episode(
    *,
    contract: GazeboDifferentialRoadContract,
    scenario: DynamicRoadScenario,
    stepper: GazeboStepper,
    emit_console: bool,
    wall_clock: bool = False,
    frame_observer: Callable[[Mapping[str, Any]], None] | None = None,
) -> tuple[list[dict[str, Any]], float]:
    """Run one measured-state online-CBF episode through Gazebo."""

    differential = contract.base
    effective_cbf = contract.effective_lifted_cbf
    current_measurement = stepper.initial_state
    if current_measurement is None:
        raise RuntimeError("Gazebo stepper has no initial state")
    episode_start_time_s = current_measurement.sim_time_s
    episode_start_iterations = current_measurement.iterations
    state = _planar_state(current_measurement)
    planner = LaneSupervisor.from_contract(
        differential.base,
        controller_spec=differential.nominal_controller,
    )
    active_ids: set[str] = set()
    rows: list[dict[str, Any]] = []
    transition_count = 0
    termination_reason: str | None = None
    wall_start = perf_counter()

    for step_index in range(differential.max_steps):
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
            differential.mapper.lookahead_distance_m
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
                spec=effective_cbf,
                additional_inflation_m=(
                    differential.additional_body_inflation_m
                ),
                additional_constraints=(
                    differential.mapper.reachable_velocity_constraints(
                        heading_rad=state.heading_rad
                    )
                ),
            )
            mapping_result = differential.mapper.command(
                filter_result.filtered_action,
                state=state,
            )
        except InfeasibleProjectionError:
            termination_reason = "cbf_action_set_infeasible"
            row = _frame(
                step_index=step_index,
                state=state,
                scenario=scenario,
                contract=differential,
                active_obstacles=active_obstacles,
                newly_activated_ids=newly_active,
                planner_facts=planner_facts,
                filter_result=None,
                mapping_result=None,
                transition_noise_m=None,
                successor=None,
                realized_contraction_residuals=None,
                termination_reason=termination_reason,
                cbf_spec=effective_cbf,
            )
            row = _patch_gazebo_frame(
                row,
                contract=contract,
                current_measurement=current_measurement,
                episode_start_time_s=episode_start_time_s,
                episode_start_iterations=episode_start_iterations,
                predicted_successor=None,
                successor_measurement=None,
            )
            rows.append(row)
            if frame_observer is not None:
                frame_observer(row)
            break
        if not differential.system.action_is_admissible(
            mapping_result.action,
            tolerance=differential.lifted_cbf.feasibility_tolerance,
        ):
            raise RuntimeError("projection violated differential bounds")

        predicted_successor = differential.system.successor(
            state,
            mapping_result.action,
            (0.0, 0.0),
            feasibility_tolerance=(
                differential.lifted_cbf.feasibility_tolerance
            ),
        )
        successor_measurement = stepper.step(
            linear_mps=mapping_result.action.linear_mps,
            angular_radps=mapping_result.action.angular_radps,
        )
        successor = _planar_state(successor_measurement)
        contraction_residuals = _realized_contraction_residuals(
            current_filter=filter_result,
            successor=successor,
            active_obstacles=active_obstacles,
            scenario=scenario,
            contract=differential,
            cbf_spec=effective_cbf,
        )
        tracking_x = successor.x_m - predicted_successor.x_m
        tracking_y = successor.y_m - predicted_successor.y_m
        row = _frame(
            step_index=step_index,
            state=state,
            scenario=scenario,
            contract=differential,
            active_obstacles=active_obstacles,
            newly_activated_ids=newly_active,
            planner_facts=planner_facts,
            filter_result=filter_result,
            mapping_result=mapping_result,
            transition_noise_m=(tracking_x, tracking_y),
            successor=successor,
            realized_contraction_residuals=contraction_residuals,
            termination_reason=None,
            cbf_spec=effective_cbf,
        )
        row = _patch_gazebo_frame(
            row,
            contract=contract,
            current_measurement=current_measurement,
            episode_start_time_s=episode_start_time_s,
            episode_start_iterations=episode_start_iterations,
            predicted_successor=predicted_successor,
            successor_measurement=successor_measurement,
        )
        rows.append(row)
        if frame_observer is not None:
            frame_observer(row)
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
        current_measurement = successor_measurement
        transition_count = step_index + 1
        if wall_clock:
            remaining = (
                transition_count * differential.system.dt_s
                - (perf_counter() - wall_start)
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
        row = _frame(
            step_index=transition_count,
            state=state,
            scenario=scenario,
            contract=differential,
            active_obstacles=active_obstacles,
            newly_activated_ids=newly_active,
            planner_facts={
                "controller_id": (
                    differential.nominal_controller.controller_id
                ),
                "target_lane_index": planner.target_lane_index,
                "target_lane_y_m": (
                    differential.base.geometry.lane_centers_y_m[
                        planner.target_lane_index
                    ]
                ),
            },
            filter_result=None,
            mapping_result=None,
            transition_noise_m=None,
            successor=None,
            realized_contraction_residuals=None,
            termination_reason=termination_reason,
            cbf_spec=effective_cbf,
        )
        row = _patch_gazebo_frame(
            row,
            contract=contract,
            current_measurement=current_measurement,
            episode_start_time_s=episode_start_time_s,
            episode_start_iterations=episode_start_iterations,
            predicted_successor=None,
            successor_measurement=None,
        )
        rows.append(row)
        if frame_observer is not None:
            frame_observer(row)
    return rows, perf_counter() - wall_start


def gazebo_differential_road_metrics(
    rows: list[Mapping[str, Any]],
) -> dict[str, Any]:
    metrics = differential_road_episode_metrics(rows)
    transitions = [
        row["transition"]
        for row in rows
        if row.get("transition") is not None
        and "tracking_residual" in row["transition"]
    ]
    measurements = [
        row["state"]["gazebo_measurement"]
        for row in rows
    ]
    metrics.update(
        {
            "maximum_position_tracking_residual_m": max(
                (
                    float(item["tracking_residual"]["position_norm_m"])
                    for item in transitions
                ),
                default=0.0,
            ),
            "maximum_absolute_heading_tracking_residual_rad": max(
                (
                    abs(float(item["tracking_residual"]["heading_rad"]))
                    for item in transitions
                ),
                default=0.0,
            ),
            "maximum_absolute_roll_rad": max(
                abs(float(item["roll_rad"])) for item in measurements
            ),
            "maximum_absolute_pitch_rad": max(
                abs(float(item["pitch_rad"])) for item in measurements
            ),
            "gazebo_physics_iteration_count": int(
                measurements[-1]["episode_physics_iterations"]
            ),
            "gazebo_simulated_duration_s": float(
                measurements[-1]["episode_simulated_time_s"]
            ),
        }
    )
    return metrics


def _gazebo_acceptance(
    *,
    rows: list[Mapping[str, Any]],
    metrics: Mapping[str, Any],
    contract: GazeboDifferentialRoadContract,
) -> dict[str, Any]:
    spec = contract.acceptance
    differential = contract.base
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
        "measured_contraction_residual": (
            contraction is not None
            and contraction
            >= spec.minimum_measured_contraction_residual_m
        ),
        "mapping_residual": (
            metrics["maximum_mapping_residual_mps"]
            <= spec.maximum_mapping_residual_mps
        ),
        "linear_command_bound": (
            metrics["maximum_linear_speed_mps"]
            <= differential.system.max_linear_mps
            + differential.lifted_cbf.feasibility_tolerance
        ),
        "angular_command_bound": (
            metrics["maximum_absolute_angular_speed_radps"]
            <= differential.system.max_angular_radps
            + differential.lifted_cbf.feasibility_tolerance
        ),
        "position_tracking_residual": (
            metrics["maximum_position_tracking_residual_m"]
            <= spec.maximum_position_tracking_residual_m
        ),
        "heading_tracking_residual": (
            metrics["maximum_absolute_heading_tracking_residual_rad"]
            <= spec.maximum_heading_tracking_residual_rad
        ),
        "roll_bound": (
            metrics["maximum_absolute_roll_rad"]
            <= spec.maximum_absolute_roll_rad
        ),
        "pitch_bound": (
            metrics["maximum_absolute_pitch_rad"]
            <= spec.maximum_absolute_pitch_rad
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
    return {
        "status": (
            "provisional_success" if all(gates.values()) else "failed"
        ),
        "gates": gates,
        "independent_replay": {
            "status": "not_evaluated",
            "reason": (
                "single-run evidence; use the separate bounded replay validator"
            ),
        },
    }


def _apply_live_display_acceptance(
    *,
    acceptance: Mapping[str, Any],
    live_manifest: Mapping[str, Any] | None,
    metrics: Mapping[str, Any],
    contract: GazeboDifferentialRoadContract,
    wall_clock_paced: bool,
) -> dict[str, Any]:
    display_spec = contract.live_display
    display_required = display_spec is not None and display_spec.enabled
    if live_manifest is None and not display_required:
        return dict(acceptance)

    result = dict(acceptance)
    gates = dict(acceptance["gates"])
    gate_names = (
        "live_display_completed",
        "live_display_expected_updates",
        "live_display_terminal_alignment",
        "live_display_observer_only",
        "live_display_configuration",
    )
    camera_gate_names = (
        "front_camera_frames_present",
        "front_camera_frames_valid",
        "front_camera_observer_only",
        "front_camera_timestamp_alignment",
        "front_camera_temporal_coverage",
        *(
            ("front_camera_activation_keyframes",)
            if contract.acceptance.require_front_camera_activation_keyframes
            else ()
        ),
    ) if contract.acceptance.require_front_camera_frames else ()
    if live_manifest is None:
        gates.update(
            {name: False for name in (*gate_names, *camera_gate_names)}
        )
        result["live_display"] = {
            "required": True,
            "status": "missing",
        }
    else:
        stride = int(live_manifest["display_stride"])
        transition_count = int(metrics["transition_count"])
        expected_updates = transition_count // stride + 1
        if transition_count % stride != 0:
            expected_updates += 1
        final_snapshot = live_manifest["final_snapshot"]
        final_step_index = final_snapshot.get("step_index")
        final_simulated_time_s = final_snapshot.get("simulated_time_s")
        semantics = live_manifest["semantics"]
        transport = live_manifest["transport"]
        configuration_match = True
        if display_spec is not None:
            configuration_match = (
                stride == display_spec.display_stride
                and abs(
                    float(live_manifest["refresh_interval_s"])
                    - display_spec.refresh_interval_s
                )
                <= 1.0e-12
                and transport["bind_host"] == display_spec.bind_host
                and transport["requested_port"] == display_spec.port
                and wall_clock_paced == display_spec.wall_clock_paced
                and (
                    contract.front_camera is None
                    or (
                        live_manifest.get("layout") == display_spec.layout
                        and live_manifest.get(
                            "camera_certificate_projection"
                        )
                        == display_spec.camera_certificate_projection
                    )
                )
            )
        gates.update(
            {
                "live_display_completed": (
                    live_manifest["status"] == "completed"
                ),
                "live_display_expected_updates": (
                    int(live_manifest["update_count"])
                    == expected_updates
                ),
                "live_display_terminal_alignment": (
                    final_step_index is not None
                    and final_simulated_time_s is not None
                    and int(final_step_index) == transition_count
                    and abs(
                        float(final_simulated_time_s)
                        - float(metrics["gazebo_simulated_duration_s"])
                    )
                    <= 1.0e-12
                ),
                "live_display_observer_only": (
                    not semantics["display_sink_control_feedback"]
                    and not semantics["display_sink_action_modification"]
                    and semantics["online_filter_precedes_display_observation"]
                ),
                "live_display_configuration": configuration_match,
            }
        )
        if camera_gate_names:
            camera_manifest = live_manifest.get("camera", {})
            camera_update_count = int(camera_manifest.get("update_count", 0))
            aligned_update_count = int(
                camera_manifest.get("aligned_update_count", 0)
            )
            maximum_alignment_error_s = camera_manifest.get(
                "maximum_alignment_error_s"
            )
            activation_keyframes = list(
                camera_manifest.get("activation_keyframes", [])
            )
            archived_obstacle_ids = [
                str(obstacle_id)
                for keyframe in activation_keyframes
                for obstacle_id in keyframe.get("obstacle_ids", [])
            ]
            gates.update(
                {
                    "front_camera_frames_present": (
                        bool(camera_manifest.get("configured"))
                        and camera_update_count > 0
                        and bool(
                            camera_manifest.get("final_frame_available")
                        )
                    ),
                    "front_camera_frames_valid": (
                        int(
                            camera_manifest.get("invalid_frame_count", 0)
                        )
                        == 0
                    ),
                    "front_camera_observer_only": bool(
                        semantics.get("front_camera_observer_only", False)
                    ),
                    "front_camera_timestamp_alignment": (
                        aligned_update_count == camera_update_count
                        and camera_update_count > 0
                        and maximum_alignment_error_s is not None
                        and float(maximum_alignment_error_s)
                        <= contract.acceptance.maximum_front_camera_alignment_error_s
                    ),
                    "front_camera_temporal_coverage": (
                        float(
                            camera_manifest.get("capture_coverage_ratio", 0.0)
                        )
                        >= contract.acceptance.minimum_front_camera_capture_coverage_ratio
                    ),
                }
            )
            if contract.acceptance.require_front_camera_activation_keyframes:
                required_activations = (
                    contract.acceptance.required_obstacle_activations
                )
                gates["front_camera_activation_keyframes"] = (
                    len(archived_obstacle_ids) == required_activations
                    and len(set(archived_obstacle_ids)) == required_activations
                    and not camera_manifest.get(
                        "pending_activation_keyframe_obstacle_ids", []
                    )
                    and all(
                        keyframe.get("frame_file_sha256")
                        and keyframe.get("snapshot_file_sha256")
                        and keyframe.get("camera_alignment") is not None
                        and set(keyframe.get("obstacle_ids", [])).issubset(
                            set(
                                keyframe.get("camera_alignment", {}).get(
                                    "active_obstacle_ids", []
                                )
                            )
                        )
                        for keyframe in activation_keyframes
                    )
                )
        result["live_display"] = {
            "required": display_required,
            "status": live_manifest["status"],
            "expected_update_count": expected_updates,
            "actual_update_count": int(live_manifest["update_count"]),
            "maximum_snapshot_bytes": int(
                live_manifest["maximum_snapshot_bytes"]
            ),
            "camera": dict(live_manifest.get("camera", {})),
        }
    result["gates"] = gates
    result["status"] = (
        "provisional_success" if all(gates.values()) else "failed"
    )
    return result


def _apply_camera_recording_video_acceptance(
    *,
    acceptance: Mapping[str, Any],
    live_manifest: Mapping[str, Any] | None,
    video_manifest: Mapping[str, Any] | None,
    contract: GazeboDifferentialRoadContract,
) -> dict[str, Any]:
    """Bind continuous camera/video artifacts into the empirical run gates."""

    result = dict(acceptance)
    gates = dict(result["gates"])
    recording_required = contract.acceptance.require_front_camera_recording
    video_required = contract.acceptance.require_front_camera_video
    if not recording_required and not video_required:
        result["gates"] = gates
        result["status"] = (
            "provisional_success" if all(gates.values()) else "failed"
        )
        return result

    recording_payload: Mapping[str, Any] | None = None
    if live_manifest is not None:
        camera_payload = live_manifest.get("camera", {})
        recording_wrapper = camera_payload.get("recording")
        if isinstance(recording_wrapper, Mapping):
            candidate = recording_wrapper.get("manifest")
            if isinstance(candidate, Mapping):
                recording_payload = candidate

    if recording_required:
        recording_spec = contract.camera_recording
        assert recording_spec is not None
        recording_gates = (
            {}
            if recording_payload is None
            else dict(recording_payload.get("gates", {}))
        )
        maximum_gap = (
            None
            if recording_payload is None
            else recording_payload.get("maximum_source_frame_gap_s")
        )
        gates.update(
            {
                "front_camera_recording_completed": (
                    recording_payload is not None
                    and recording_payload.get("status") == "completed"
                ),
                "front_camera_recording_frames_present": (
                    recording_payload is not None
                    and int(recording_payload.get("frame_count", 0)) > 0
                ),
                "front_camera_recording_hash_closure": bool(
                    recording_gates.get("source_frame_hash_closure", False)
                ),
                "front_camera_recording_temporal_coverage": (
                    recording_payload is not None
                    and float(
                        recording_payload.get("capture_coverage_ratio", 0.0)
                    )
                    >= recording_spec.minimum_capture_coverage_ratio
                    - 1.0e-12
                ),
                "front_camera_recording_frame_gap": (
                    maximum_gap is not None
                    and float(maximum_gap)
                    <= recording_spec.maximum_source_frame_gap_s + 1.0e-12
                ),
                "front_camera_recording_observer_only": (
                    recording_payload is not None
                    and bool(recording_payload.get("observer_only", False))
                ),
            }
        )
        result["camera_recording"] = (
            None
            if recording_payload is None
            else {
                "status": recording_payload.get("status"),
                "frame_count": int(recording_payload.get("frame_count", 0)),
                "capture_coverage_ratio": float(
                    recording_payload.get("capture_coverage_ratio", 0.0)
                ),
                "maximum_source_frame_gap_s": maximum_gap,
                "missing_frame_count": int(
                    recording_payload.get("missing_frame_count", 0)
                ),
            }
        )

    if video_required:
        video_gates = (
            {} if video_manifest is None else dict(video_manifest.get("gates", {}))
        )
        recording_count = (
            0
            if recording_payload is None
            else int(recording_payload.get("frame_count", 0))
        )
        gates.update(
            {
                "front_camera_video_completed": (
                    video_manifest is not None
                    and video_manifest.get("status") == "completed"
                ),
                "front_camera_video_gates": bool(video_gates)
                and all(bool(value) for value in video_gates.values()),
                "front_camera_video_source_frame_closure": (
                    video_manifest is not None
                    and int(video_manifest.get("source_frame_count", -1))
                    == recording_count
                ),
                "front_camera_video_observer_only": (
                    video_manifest is not None
                    and bool(video_manifest.get("observer_only", False))
                ),
            }
        )
        result["camera_video"] = (
            None
            if video_manifest is None
            else {
                "status": video_manifest.get("status"),
                "video_file": video_manifest.get("video_file"),
                "video_file_sha256": video_manifest.get(
                    "video_file_sha256"
                ),
                "source_frame_count": video_manifest.get(
                    "source_frame_count"
                ),
                "rendered_frame_count": video_manifest.get(
                    "rendered_frame_count"
                ),
                "encoded_duration_s": video_manifest.get(
                    "encoded_duration_s"
                ),
            }
        )

    result["gates"] = gates
    result["status"] = (
        "provisional_success" if all(gates.values()) else "failed"
    )
    return result


def simulate_gazebo_differential_road_cbf(
    *,
    contract: GazeboDifferentialRoadContract,
    bridge_binary: str | Path,
    output_dir: str | Path,
    seed: int | None = None,
    emit_console: bool = False,
    render_gif: bool = True,
    display_stride: int | None = None,
    gzserver_binary: str = "gzserver",
    live_display: bool | None = None,
    live_display_host: str | None = None,
    live_display_port: int | None = None,
    wall_clock: bool | None = None,
) -> Path:
    """Execute, render, hash, and classify one Gazebo physical run."""

    actual_seed = contract.default_seed if seed is None else int(seed)
    if actual_seed < 0:
        raise ValueError("seed must be nonnegative")
    display_spec = contract.live_display
    actual_live_display = (
        bool(display_spec is not None and display_spec.enabled)
        if live_display is None
        else bool(live_display)
    )
    stride = (
        (
            display_spec.display_stride
            if actual_live_display and display_spec is not None
            else contract.base.display_stride
        )
        if display_stride is None
        else int(display_stride)
    )
    if stride <= 0:
        raise ValueError("display_stride must be positive")
    actual_live_display_host = (
        display_spec.bind_host
        if live_display_host is None and display_spec is not None
        else (
            "127.0.0.1"
            if live_display_host is None
            else live_display_host
        )
    )
    actual_live_display_port = (
        display_spec.port
        if live_display_port is None and display_spec is not None
        else (0 if live_display_port is None else int(live_display_port))
    )
    live_refresh_interval_s = (
        0.20
        if display_spec is None
        else display_spec.refresh_interval_s
    )
    actual_wall_clock = (
        bool(
            actual_live_display
            and display_spec is not None
            and display_spec.wall_clock_paced
        )
        if wall_clock is None
        else bool(wall_clock)
    )
    scenario = generate_seeded_road_scenario(
        contract.base.base,
        seed=actual_seed,
    )
    bridge_path = Path(bridge_binary).resolve()
    state_plugin_path = (
        None
        if contract.runtime.state_plugin is None
        else bridge_path.parent / contract.runtime.state_plugin
    )
    if state_plugin_path is not None and not state_plugin_path.is_file():
        raise FileNotFoundError(state_plugin_path)
    camera_plugin_path = (
        None
        if contract.front_camera is None
        else bridge_path.parent / contract.front_camera.plugin
    )
    if camera_plugin_path is not None and not camera_plugin_path.is_file():
        raise FileNotFoundError(camera_plugin_path)
    destination = Path(output_dir).resolve()
    destination.mkdir(parents=True, exist_ok=False)
    camera_source_path: Path | None = None
    camera_queue_directory: Path | None = None
    if contract.front_camera is not None:
        camera_directory = destination / "camera_raw"
        camera_directory.mkdir(parents=False, exist_ok=False)
        camera_source_path = camera_directory / contract.front_camera.frame_file
        if contract.camera_recording is not None:
            camera_queue_directory = camera_directory / "queue"
            camera_queue_directory.mkdir(parents=False, exist_ok=False)
    plan = obstacle_plan_payload(scenario=scenario, seed=actual_seed)
    plan_path = destination / "obstacle_plan.json"
    _write_json(plan_path, plan)
    world_path = destination / "dynamic_road_gazebo.world"
    world_path.write_text(
        render_gazebo_classic_world(
            contract=contract,
            scenario=scenario,
            camera_output_path=camera_source_path,
            camera_queue_directory=camera_queue_directory,
        ),
        encoding="utf-8",
    )

    session, startup_records = _start_gazebo_session_with_attempts(
        contract=contract,
        world_path=world_path,
        bridge_path=bridge_path,
        destination=destination,
        gzserver_binary=gzserver_binary,
    )
    live_publisher: GazeboLiveDisplay | None = None
    live_display_manifest_path: Path | None = None
    camera_video_manifest_path: Path | None = None
    startup_attempts_path: Path | None = None
    camera_writer_temp_removed_after_shutdown: bool | None = None
    try:
        if actual_live_display:
            live_publisher = GazeboLiveDisplay(
                directory=destination / "live_display",
                scenario=scenario,
                display_stride=stride,
                host=actual_live_display_host,
                port=actual_live_display_port,
                refresh_interval_s=live_refresh_interval_s,
                camera_source_path=camera_source_path,
                camera_queue_directory=camera_queue_directory,
                camera_spec=contract.front_camera,
                rover_spec=(
                    None
                    if contract.front_camera is None
                    else contract.rover
                ),
                state_sample_period_s=(
                    None
                    if contract.front_camera is None
                    else contract.base.system.dt_s
                ),
                camera_recording_spec=contract.camera_recording,
            )
            live_url = live_publisher.start()
            print(f"LIVE_DISPLAY_URL={live_url}", flush=True)
        rows, wall_elapsed_s = run_gazebo_differential_road_episode(
            contract=contract,
            scenario=scenario,
            stepper=session,
            emit_console=emit_console,
            wall_clock=actual_wall_clock,
            frame_observer=(
                None if live_publisher is None else live_publisher.publish
            ),
        )
    except BaseException as error:
        if live_publisher is not None:
            live_publisher.fail(error)
        raise
    finally:
        session.close()
        if live_publisher is not None:
            live_publisher.drain_camera_frames()
        if camera_source_path is not None:
            camera_writer_temp_removed_after_shutdown = (
                _remove_camera_writer_temporary_file(
                    camera_source_path
                )
            )
        if live_publisher is not None:
            live_display_manifest_path = live_publisher.close()
        startup_attempts_path = _write_startup_attempts(
            destination=destination,
            records=startup_records,
            attempt_limit=contract.runtime.startup_attempt_limit,
            selected_attempt=int(startup_records[-1]["attempt"]),
            status="ready",
        )

    assert startup_attempts_path is not None

    trace_path = destination / "live_trace.jsonl"
    write_trace(trace_path, rows)
    trace_sha = file_sha256(trace_path)
    metrics = gazebo_differential_road_metrics(rows)
    acceptance = _gazebo_acceptance(
        rows=rows,
        metrics=metrics,
        contract=contract,
    )
    live_display_manifest = (
        None
        if live_display_manifest_path is None
        else live_publisher.manifest_payload()
    )
    camera_video_manifest: Mapping[str, Any] | None = None
    if contract.camera_recording is not None:
        if live_publisher is None:
            raise RuntimeError("camera recording requires the live display")
        from .camera_video import render_camera_certificate_video

        camera_video_manifest_path = render_camera_certificate_video(
            recording_manifest_path=(
                live_publisher.camera_recording_manifest_path
            ),
            output_dir=destination / "camera_video",
            terminal_reason=rows[-1]["termination_reason"],
            terminal_simulated_time_s=float(rows[-1]["simulated_time_s"]),
        )
        camera_video_manifest = json.loads(
            camera_video_manifest_path.read_text(encoding="utf-8")
        )
    acceptance = _apply_live_display_acceptance(
        acceptance=acceptance,
        live_manifest=live_display_manifest,
        metrics=metrics,
        contract=contract,
        wall_clock_paced=actual_wall_clock,
    )
    acceptance = _apply_camera_recording_video_acceptance(
        acceptance=acceptance,
        live_manifest=live_display_manifest,
        video_manifest=camera_video_manifest,
        contract=contract,
    )
    animation_path = destination / "gazebo_classic_cbf_1x.gif"
    final_frame_path = destination / "gazebo_classic_cbf_final.png"
    if render_gif:
        render_differential_road_animation(
            rows,
            scenario=scenario,
            gif_path=animation_path,
            final_frame_path=final_frame_path,
            dt_s=contract.base.system.dt_s,
            display_stride=stride,
            title_backend="GAZEBO CLASSIC 11 + DIFFERENTIAL DRIVE",
            title_claim="EMPIRICAL / NOT q-SBC",
            footer_backend="GAZEBO CLASSIC 11 ODE / MEASURED-STATE CBF",
        )

    manifest: dict[str, Any] = {
        "schema_version": 1,
        "kind": "dynamic_road_gazebo_classic_cbf_run",
        "experiment_id": contract.experiment_id,
        "status": acceptance["status"],
        "termination_reason": rows[-1]["termination_reason"],
        "claim_boundary": {
            "online_distance_cbf": (
                "evaluated_from_timestamped_pose_iteration_aligned_"
                "measured_gazebo_pose_each_control_step"
            ),
            "differential_drive_tracking": (
                "empirical_gazebo_classic_ode"
            ),
            "tracking_error_bound": "not_formally_established",
            "front_camera": (
                "gazebo_rgb_sensor_observer_only"
                if contract.front_camera is not None
                else "not_configured"
            ),
            "certificate_camera_overlay": (
                "camera_sim_timestamp_nearest_measured_pose_"
                "pinhole_projection_observer_only"
                if contract.front_camera is not None
                else "not_configured"
            ),
            "camera_video": (
                "timestamped_gazebo_rgb_with_observer_only_certificate_overlay"
                if contract.camera_recording is not None
                else "not_configured"
            ),
            "lane_selection": (
                "passed_obstacles_excluded_from_nominal_route_decisions"
                if contract.base.nominal_controller.ignore_passed_obstacles_for_lane_selection
                else "legacy_pass_margin_lane_selection"
            ),
            "q_sbc_probability_bound": "not_claimed",
            "ros2_result": "not_claimed",
            "real_vehicle_claim": "forbidden",
        },
        "contract": {
            "config_path": str(contract.config_path),
            "config_file_sha256": contract.config_file_sha256,
            "base_differential_config_path": str(
                contract.base.config_path
            ),
            "base_differential_config_file_sha256": (
                contract.base.config_file_sha256
            ),
        },
        "scenario": {
            "scenario_id": scenario.scenario_id,
            "seed": actual_seed,
            "obstacle_plan_file": plan_path.name,
            "obstacle_plan_file_sha256": file_sha256(plan_path),
            "obstacle_plan_content_sha256": plan["content_sha256"],
            "world_file": world_path.name,
            "world_file_sha256": file_sha256(world_path),
            "visual_layout": contract.visual_scene.layout,
        },
        "startup": {
            "attempts_file": startup_attempts_path.name,
            "attempts_file_sha256": file_sha256(startup_attempts_path),
            "attempt_count": len(startup_records),
            "selected_attempt": int(startup_records[-1]["attempt"]),
            "retry_scope": "before_initial_state_only",
            "control_retries_after_initial_state": 0,
        },
        "runtime": {
            "backend": "Gazebo Classic 11",
            "physics_engine": contract.runtime.physics_engine,
            "master_uri": contract.runtime.master_uri,
            "gazebo_ip": "127.0.0.1",
            "world_name": contract.runtime.world_name,
            "model_name": contract.runtime.model_name,
            "physics_step_s": contract.runtime.physics_step_s,
            "solver_iterations": contract.runtime.solver_iterations,
            "control_iterations_per_step": (
                contract.runtime.control_iterations_per_step
            ),
            "control_period_s": contract.runtime.control_period_s,
            "startup_timeout_s": contract.runtime.startup_timeout_s,
            "startup_attempt_limit": (
                contract.runtime.startup_attempt_limit
            ),
            "startup_retry_backoff_s": (
                contract.runtime.startup_retry_backoff_s
            ),
            "state_timeout_s": contract.runtime.state_timeout_s,
            "headless_renderer": contract.runtime.headless_renderer,
            "bridge_startup_timeout_environment": (
                "CBF_ROVER_GAZEBO_STARTUP_TIMEOUT_S"
            ),
            "plugin_angular_command_sign": (
                contract.runtime.plugin_angular_command_sign
            ),
            "bridge_protocol": "GZCB1",
            "bridge_binary_path": str(bridge_path),
            "bridge_binary_sha256": file_sha256(bridge_path),
            "state_plugin_path": (
                None
                if state_plugin_path is None
                else str(state_plugin_path)
            ),
            "state_plugin_sha256": (
                None
                if state_plugin_path is None
                else file_sha256(state_plugin_path)
            ),
            "camera_plugin_path": (
                None
                if camera_plugin_path is None
                else str(camera_plugin_path)
            ),
            "camera_plugin_sha256": (
                None
                if camera_plugin_path is None
                else file_sha256(camera_plugin_path)
            ),
            "bridge_pose_world_timestamp_alignment": (
                contract.runtime.bridge_pose_world_timestamp_alignment
            ),
            "bridge_step_completion": (
                "timestamped_pose_iteration_target"
            ),
            "state_plugin_iteration_transport": (
                "pose_message_id_uint32"
            ),
            "world_stats_per_step_gate": False,
            "gzserver_binary": gzserver_binary,
            "base_effective_position_support_half_width_m": (
                contract.base.lifted_cbf.noise_half_width_m
            ),
            "gazebo_effective_position_support_half_width_m": (
                contract.effective_lifted_cbf.noise_half_width_m
            ),
            "tracking_lookahead_displacement_bound_m": (
                contract.tracking_lookahead_displacement_bound_m
            ),
            "paused_fixed_step": True,
            "ros2_used": False,
            "post_projection_command_clipping_applied": False,
            "tracking_envelope": (
                None
                if contract.tracking_envelope is None
                else {
                    "source_experiment_id": (
                        contract.tracking_envelope.source_experiment_id
                    ),
                    "source_trace_sha256": (
                        contract.tracking_envelope.source_trace_sha256
                    ),
                    "position_coordinate_half_width_m": (
                        contract.tracking_envelope
                        .position_coordinate_half_width_m
                    ),
                    "heading_error_bound_rad": (
                        contract.tracking_envelope.heading_error_bound_rad
                    ),
                    "formal_bound": False,
                }
            ),
        },
        "wall_clock_paced": actual_wall_clock,
        "wall_clock_pacing": {
            "enabled": actual_wall_clock,
            "policy": "not_faster_than_one_times_simulated_time",
            "achieved_realtime_factor": (
                float(metrics["gazebo_simulated_duration_s"])
                / wall_elapsed_s
            ),
        },
        "wall_elapsed_s": wall_elapsed_s,
        "telemetry": {
            "trace_file": trace_path.name,
            "trace_file_sha256": trace_sha,
            **metrics,
        },
        "acceptance": acceptance,
        "front_camera": (
            None
            if contract.front_camera is None
            else {
                "sensor_name": contract.front_camera.sensor_name,
                "image_size": [
                    contract.front_camera.image_width,
                    contract.front_camera.image_height,
                ],
                "horizontal_fov_rad": (
                    contract.front_camera.horizontal_fov_rad
                ),
                "update_rate_hz": contract.front_camera.update_rate_hz,
                "transport": contract.front_camera.transport,
                "timestamp_alignment_policy": (
                    "embedded_gazebo_sim_timestamp_to_nearest_measured_state"
                ),
                "pose_on_base_link": list(
                    contract.front_camera.pose_on_base_link
                ),
                "raw_frame_file": (
                    None
                    if camera_source_path is None
                    else str(camera_source_path.relative_to(destination))
                ),
                "raw_frame_file_sha256": (
                    None
                    if camera_source_path is None
                    or not camera_source_path.is_file()
                    else file_sha256(camera_source_path)
                ),
                "display_sink_control_feedback": False,
                "writer_temp_removed_after_gzserver_shutdown": (
                    camera_writer_temp_removed_after_shutdown
                ),
            }
        ),
        "live_display": (
            None
            if live_display_manifest_path is None
            else {
                "directory": live_display_manifest_path.parent.name,
                "manifest_file": live_display_manifest_path.name,
                "manifest_file_sha256": file_sha256(
                    live_display_manifest_path
                ),
                "manifest": live_display_manifest,
            }
        ),
        "camera_video": (
            None
            if camera_video_manifest_path is None
            else {
                "directory": camera_video_manifest_path.parent.name,
                "manifest_file": camera_video_manifest_path.name,
                "manifest_file_sha256": file_sha256(
                    camera_video_manifest_path
                ),
                "manifest": camera_video_manifest,
            }
        ),
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
