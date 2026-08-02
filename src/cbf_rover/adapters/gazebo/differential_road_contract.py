"""Configuration binding for the Gazebo Classic differential-road slice."""

from __future__ import annotations

from dataclasses import dataclass
from math import hypot, isfinite, pi, sin
from pathlib import Path
from typing import Mapping
from urllib.parse import urlparse

import yaml

from cbf_rover.adapters.exact.differential_road_contract import (
    DifferentialRoadContract,
    load_differential_road_contract,
)
from cbf_rover.adapters.exact.road_contract import file_sha256
from cbf_rover.methods.online_cbf import DistanceCBFSpec


def _positive(value: object, name: str) -> float:
    result = float(value)
    if not isfinite(result) or result <= 0.0:
        raise ValueError(f"{name} must be finite and positive")
    return result


@dataclass(frozen=True, slots=True)
class GazeboClassicRuntimeSpec:
    required_major_version: int
    physics_engine: str
    world_name: str
    model_name: str
    master_uri: str
    physics_step_s: float
    solver_iterations: int
    control_iterations_per_step: int
    startup_timeout_s: float
    state_timeout_s: float
    command_plugin: str
    state_plugin: str | None
    plugin_angular_command_sign: float
    bridge_pose_world_timestamp_alignment: str
    startup_attempt_limit: int = 1
    startup_retry_backoff_s: float = 0.0
    headless_renderer: str = "none"

    def __post_init__(self) -> None:
        if self.required_major_version != 11:
            raise ValueError("first Gazebo slice is frozen to major version 11")
        if self.physics_engine != "ode":
            raise ValueError("first Gazebo slice is frozen to ODE")
        if not self.world_name.strip() or not self.model_name.strip():
            raise ValueError("Gazebo world and model names must not be empty")
        if self.control_iterations_per_step <= 0 or self.solver_iterations <= 0:
            raise ValueError(
                "control and solver iteration counts must be positive"
            )
        if self.startup_attempt_limit <= 0:
            raise ValueError("startup attempt limit must be positive")
        if (
            not isfinite(self.startup_retry_backoff_s)
            or self.startup_retry_backoff_s < 0.0
        ):
            raise ValueError(
                "startup retry backoff must be finite and nonnegative"
            )
        if self.headless_renderer not in {"none", "xvfb"}:
            raise ValueError("headless renderer must be none or xvfb")
        for field_name in (
            "physics_step_s",
            "startup_timeout_s",
            "state_timeout_s",
        ):
            object.__setattr__(
                self,
                field_name,
                _positive(getattr(self, field_name), field_name),
            )
        if self.command_plugin != "libDiffDrivePlugin.so":
            raise ValueError("unexpected Gazebo command plugin")
        if self.state_plugin not in {
            None,
            "libcbf_rover_gazebo_state_plugin.so",
        }:
            raise ValueError("unexpected Gazebo state plugin")
        if self.plugin_angular_command_sign not in {-1.0, 1.0}:
            raise ValueError("plugin angular command sign must be -1 or +1")
        if self.bridge_pose_world_timestamp_alignment not in {
            "required",
            "not_required_before_v6",
        }:
            raise ValueError(
                "unexpected bridge pose/world timestamp alignment policy"
            )
        if (
            self.bridge_pose_world_timestamp_alignment == "required"
            and self.state_plugin is None
        ):
            raise ValueError(
                "timestamp-aligned bridge requires the state plugin"
            )
        parsed = urlparse(self.master_uri)
        if (
            parsed.scheme != "http"
            or parsed.hostname not in {"127.0.0.1", "localhost"}
            or parsed.port is None
            or parsed.port == 11345
            or parsed.path not in {"", "/"}
        ):
            raise ValueError(
                "Gazebo master must use a non-default loopback HTTP port"
            )

    @property
    def control_period_s(self) -> float:
        return self.physics_step_s * self.control_iterations_per_step


@dataclass(frozen=True, slots=True)
class GazeboLiveDisplaySpec:
    enabled: bool
    bind_host: str
    port: int
    display_stride: int
    refresh_interval_s: float
    wall_clock_paced: bool
    atomic_snapshot_updates: bool
    display_sink_control_feedback: bool
    obstacle_display_policy: str
    layout: str = "map_primary_telemetry_visible"
    camera_certificate_projection: str = "none"
    preview_level_h_m: float = 0.0

    def __post_init__(self) -> None:
        if self.bind_host not in {"127.0.0.1", "localhost"}:
            raise ValueError("live display must bind to a loopback host")
        if self.port <= 0 or self.port > 65535:
            raise ValueError("live display port must be between 1 and 65535")
        if self.display_stride <= 0:
            raise ValueError("live display stride must be positive")
        object.__setattr__(
            self,
            "refresh_interval_s",
            _positive(
                self.refresh_interval_s,
                "live display refresh interval",
            ),
        )
        if not self.atomic_snapshot_updates:
            raise ValueError("live display requires atomic snapshot updates")
        if self.display_sink_control_feedback:
            raise ValueError("live display must not feed data back to control")
        if self.obstacle_display_policy != "visible_after_controller_activation":
            raise ValueError("unexpected live obstacle display policy")
        if self.layout not in {
            "map_primary_telemetry_visible",
            "camera_primary_map_secondary_telemetry_collapsed",
        }:
            raise ValueError("unexpected live display layout")
        if self.camera_certificate_projection not in {
            "none",
            "timestamp_aligned_measured_pose_pinhole_ground_plane",
        }:
            raise ValueError("unexpected camera certificate projection")
        preview_level = float(self.preview_level_h_m)
        if not isfinite(preview_level) or preview_level < 0.0:
            raise ValueError(
                "live display preview certificate level must be finite and nonnegative"
            )
        object.__setattr__(self, "preview_level_h_m", preview_level)


@dataclass(frozen=True, slots=True)
class GazeboPhysicalRoverSpec:
    base_length_m: float
    base_width_m: float
    base_height_m: float
    base_mass_kg: float
    wheel_radius_m: float
    wheel_width_m: float
    wheel_mass_kg: float
    wheel_separation_m: float
    wheel_axle_x_m: float
    rear_caster_radius_m: float
    rear_caster_x_m: float

    def __post_init__(self) -> None:
        for field_name in (
            "base_length_m",
            "base_width_m",
            "base_height_m",
            "base_mass_kg",
            "wheel_radius_m",
            "wheel_width_m",
            "wheel_mass_kg",
            "wheel_separation_m",
            "rear_caster_radius_m",
        ):
            object.__setattr__(
                self,
                field_name,
                _positive(getattr(self, field_name), field_name),
            )
        if not isfinite(self.wheel_axle_x_m) or not isfinite(
            self.rear_caster_x_m
        ):
            raise ValueError("wheel and caster x coordinates must be finite")
        if self.wheel_axle_x_m <= self.rear_caster_x_m:
            raise ValueError("wheel axle must be ahead of the rear caster")

    @property
    def body_planar_half_diagonal_m(self) -> float:
        return hypot(self.base_length_m / 2.0, self.base_width_m / 2.0)


@dataclass(frozen=True, slots=True)
class GazeboTrackingEnvelopeSpec:
    source_experiment_id: str
    source_trace_sha256: str
    position_coordinate_half_width_m: float
    heading_error_bound_rad: float

    def __post_init__(self) -> None:
        if not self.source_experiment_id.strip():
            raise ValueError("tracking envelope source id must not be empty")
        digest = self.source_trace_sha256.lower()
        if len(digest) != 64 or any(
            character not in "0123456789abcdef" for character in digest
        ):
            raise ValueError("tracking envelope source trace hash is invalid")
        object.__setattr__(self, "source_trace_sha256", digest)
        object.__setattr__(
            self,
            "position_coordinate_half_width_m",
            _positive(
                self.position_coordinate_half_width_m,
                "position_coordinate_half_width_m",
            ),
        )
        object.__setattr__(
            self,
            "heading_error_bound_rad",
            _positive(
                self.heading_error_bound_rad,
                "heading_error_bound_rad",
            ),
        )
        if self.heading_error_bound_rad > pi:
            raise ValueError("heading tracking envelope must not exceed pi")


@dataclass(frozen=True, slots=True)
class GazeboVisualSceneSpec:
    """Visual-only Gazebo setting that must not alter the road contract."""

    layout: str = "outdoor_road"
    room_margin_x_m: float = 2.0
    room_half_width_m: float = 3.5
    wall_height_m: float = 1.4
    wall_thickness_m: float = 0.12

    def __post_init__(self) -> None:
        if self.layout not in {"outdoor_road", "indoor_room"}:
            raise ValueError("visual scene layout must be outdoor_road or indoor_room")
        for field_name in (
            "room_margin_x_m",
            "room_half_width_m",
            "wall_height_m",
            "wall_thickness_m",
        ):
            object.__setattr__(
                self,
                field_name,
                _positive(getattr(self, field_name), field_name),
            )


@dataclass(frozen=True, slots=True)
class GazeboFrontCameraSpec:
    """Front RGB camera and its observer-only filesystem transport."""

    enabled: bool
    sensor_name: str
    plugin: str
    image_width: int
    image_height: int
    horizontal_fov_rad: float
    update_rate_hz: float
    near_clip_m: float
    far_clip_m: float
    pose_on_base_link: tuple[float, float, float, float, float, float]
    frame_file: str
    transport: str

    def __post_init__(self) -> None:
        if not self.enabled:
            raise ValueError("configured front camera must be enabled")
        if not self.sensor_name.strip():
            raise ValueError("front camera sensor name must not be empty")
        if self.plugin != "libcbf_rover_gazebo_camera_plugin.so":
            raise ValueError("unexpected front camera plugin")
        if self.image_width <= 0 or self.image_height <= 0:
            raise ValueError("front camera image dimensions must be positive")
        for field_name in (
            "horizontal_fov_rad",
            "update_rate_hz",
            "near_clip_m",
            "far_clip_m",
        ):
            object.__setattr__(
                self,
                field_name,
                _positive(getattr(self, field_name), field_name),
            )
        if not 0.0 < self.horizontal_fov_rad < pi:
            raise ValueError("front camera horizontal FOV must lie in (0, pi)")
        if self.near_clip_m >= self.far_clip_m:
            raise ValueError("front camera near clip must precede far clip")
        pose = tuple(float(value) for value in self.pose_on_base_link)
        if len(pose) != 6 or not all(isfinite(value) for value in pose):
            raise ValueError("front camera pose must contain six finite values")
        object.__setattr__(self, "pose_on_base_link", pose)
        if pose[0] < 0.0 or pose[2] <= 0.0:
            raise ValueError("front camera must be mounted ahead of and above base")
        if abs(pose[3]) > 1.0e-12 or abs(pose[5]) > 1.0e-12:
            raise ValueError(
                "first camera projection slice requires zero roll and yaw"
            )
        if not 0.0 <= pose[4] <= 0.35:
            raise ValueError("front camera downward pitch must lie in [0, 0.35]")
        if any(separator in self.frame_file for separator in ("/", "\\")):
            raise ValueError("front camera frame file must be a basename")
        if not self.frame_file.lower().endswith(".bmp"):
            raise ValueError("front camera frame file must use BMP")
        if self.transport != (
            "atomic_bmp_with_embedded_gazebo_sim_timestamp_observer_only"
        ):
            raise ValueError("unexpected front camera transport")


@dataclass(frozen=True, slots=True)
class GazeboCameraRecordingSpec:
    """Frozen observer-only recording and video-delivery contract."""

    enabled: bool
    capture_policy: str
    archive_format: str
    playback_fps: float
    video_width: int
    video_height: int
    video_file: str
    encoder_backend: str
    bitrate_kbps: int
    minimum_capture_coverage_ratio: float
    maximum_source_frame_gap_s: float
    minimum_overlay_visible_frame_count: int
    require_all_activated_obstacles_visible: bool

    def __post_init__(self) -> None:
        if not self.enabled:
            raise ValueError("configured camera recording must be enabled")
        if self.capture_policy != "every_unique_timestamp_aligned_camera_frame":
            raise ValueError("unexpected camera recording capture policy")
        if self.archive_format != "png_sequence_with_jsonl_index":
            raise ValueError("unexpected camera recording archive format")
        object.__setattr__(
            self,
            "playback_fps",
            _positive(self.playback_fps, "camera recording playback fps"),
        )
        if self.video_width <= 0 or self.video_height <= 0:
            raise ValueError("camera video dimensions must be positive")
        if self.video_width % 2 or self.video_height % 2:
            raise ValueError("camera video dimensions must be even")
        if self.video_width * 9 != self.video_height * 16:
            raise ValueError("camera video must use a 16:9 frame")
        if any(separator in self.video_file for separator in ("/", "\\")):
            raise ValueError("camera video file must be a basename")
        if not self.video_file.lower().endswith(".mp4"):
            raise ValueError("camera video file must use MP4")
        if self.encoder_backend != "gstreamer_x264":
            raise ValueError("unexpected camera video encoder backend")
        if self.bitrate_kbps <= 0:
            raise ValueError("camera video bitrate must be positive")
        coverage = float(self.minimum_capture_coverage_ratio)
        if not isfinite(coverage) or not 0.0 < coverage <= 1.0:
            raise ValueError(
                "camera recording minimum coverage ratio must lie in (0, 1]"
            )
        object.__setattr__(
            self,
            "maximum_source_frame_gap_s",
            _positive(
                self.maximum_source_frame_gap_s,
                "camera recording maximum source frame gap",
            ),
        )
        if self.minimum_overlay_visible_frame_count <= 0:
            raise ValueError(
                "camera video minimum visible-overlay frame count must be positive"
            )


@dataclass(frozen=True, slots=True)
class GazeboRoadAcceptanceSpec:
    required_obstacle_activations: int
    maximum_collision_count: int
    maximum_road_departure_count: int
    minimum_lifted_barrier_m: float
    minimum_constraint_residual_mps: float
    minimum_measured_contraction_residual_m: float
    maximum_mapping_residual_mps: float
    maximum_position_tracking_residual_m: float
    maximum_heading_tracking_residual_rad: float
    maximum_absolute_roll_rad: float
    maximum_absolute_pitch_rad: float
    require_front_camera_frames: bool = False
    minimum_front_camera_capture_coverage_ratio: float = 0.0
    maximum_front_camera_alignment_error_s: float = 0.0
    require_front_camera_activation_keyframes: bool = False
    require_front_camera_recording: bool = False
    require_front_camera_video: bool = False

    def __post_init__(self) -> None:
        if any(
            value < 0
            for value in (
                self.required_obstacle_activations,
                self.maximum_collision_count,
                self.maximum_road_departure_count,
            )
        ):
            raise ValueError("Gazebo acceptance counts must be nonnegative")
        for field_name in (
            "maximum_mapping_residual_mps",
            "maximum_position_tracking_residual_m",
            "maximum_heading_tracking_residual_rad",
            "maximum_absolute_roll_rad",
            "maximum_absolute_pitch_rad",
        ):
            value = float(getattr(self, field_name))
            if not isfinite(value) or value < 0.0:
                raise ValueError(
                    f"{field_name} must be finite and nonnegative"
                )
        coverage = float(self.minimum_front_camera_capture_coverage_ratio)
        if not isfinite(coverage) or not 0.0 <= coverage <= 1.0:
            raise ValueError(
                "minimum front camera capture coverage ratio must lie in [0, 1]"
            )
        alignment_error = float(self.maximum_front_camera_alignment_error_s)
        if not isfinite(alignment_error) or alignment_error < 0.0:
            raise ValueError(
                "maximum front camera alignment error must be finite and nonnegative"
            )


@dataclass(frozen=True, slots=True)
class GazeboDifferentialRoadContract:
    config_path: Path
    config_file_sha256: str
    experiment_id: str
    phase: str
    default_seed: int
    base: DifferentialRoadContract
    runtime: GazeboClassicRuntimeSpec
    live_display: GazeboLiveDisplaySpec | None
    rover: GazeboPhysicalRoverSpec
    tracking_envelope: GazeboTrackingEnvelopeSpec | None
    visual_scene: GazeboVisualSceneSpec
    front_camera: GazeboFrontCameraSpec | None
    camera_recording: GazeboCameraRecordingSpec | None
    acceptance: GazeboRoadAcceptanceSpec

    def __post_init__(self) -> None:
        if not self.experiment_id.strip():
            raise ValueError("Gazebo experiment id must not be empty")
        if self.phase not in {"discovery", "confirmation", "validation"}:
            raise ValueError("unexpected Gazebo experiment phase")
        if self.default_seed < 0:
            raise ValueError("default seed must be nonnegative")
        if abs(self.runtime.control_period_s - self.base.system.dt_s) > 1.0e-15:
            raise ValueError(
                "Gazebo fixed-step control period must match differential dt"
            )
        if (
            self.rover.body_planar_half_diagonal_m
            > self.base.base.geometry.rover_radius_m
        ):
            raise ValueError(
                "scenario rover disk does not cover the physical body box"
            )
        if self.live_display is not None and self.live_display.enabled:
            master_port = urlparse(self.runtime.master_uri).port
            if self.live_display.port == master_port:
                raise ValueError(
                    "live display and Gazebo master ports must be distinct"
                )
        if self.acceptance.require_front_camera_frames:
            if self.front_camera is None:
                raise ValueError("front camera acceptance requires a camera")
            if self.live_display is None or not self.live_display.enabled:
                raise ValueError(
                    "front camera acceptance requires the live display"
                )
            assert self.live_display is not None
            if self.live_display.layout != (
                "camera_primary_map_secondary_telemetry_collapsed"
            ):
                raise ValueError(
                    "front camera acceptance requires the camera-first layout"
                )
            if self.live_display.camera_certificate_projection != (
                "timestamp_aligned_measured_pose_pinhole_ground_plane"
            ):
                raise ValueError(
                    "front camera acceptance requires timestamp-aligned projection"
                )
            if self.acceptance.minimum_front_camera_capture_coverage_ratio <= 0.0:
                raise ValueError(
                    "front camera acceptance requires positive temporal coverage"
                )
            if self.acceptance.maximum_front_camera_alignment_error_s <= 0.0:
                raise ValueError(
                    "front camera acceptance requires a positive alignment tolerance"
                )
        elif self.acceptance.require_front_camera_activation_keyframes:
            raise ValueError(
                "camera activation keyframes require front camera frame acceptance"
            )
        if self.camera_recording is not None:
            if self.front_camera is None:
                raise ValueError("camera recording requires a front camera")
            if self.live_display is None or not self.live_display.enabled:
                raise ValueError("camera recording requires the live display")
            if self.camera_recording.playback_fps > self.front_camera.update_rate_hz:
                raise ValueError(
                    "camera recording playback fps must not exceed sensor rate"
                )
        if self.acceptance.require_front_camera_recording:
            if self.camera_recording is None or not self.camera_recording.enabled:
                raise ValueError(
                    "camera recording acceptance requires an enabled recording"
                )
        if (
            self.acceptance.require_front_camera_video
            and not self.acceptance.require_front_camera_recording
        ):
            raise ValueError(
                "camera video acceptance requires camera recording acceptance"
            )

    @property
    def tracking_lookahead_displacement_bound_m(self) -> float:
        if self.tracking_envelope is None:
            return 0.0
        return (
            2.0
            * self.base.mapper.lookahead_distance_m
            * sin(self.tracking_envelope.heading_error_bound_rad / 2.0)
        )

    @property
    def effective_lifted_cbf(self) -> DistanceCBFSpec:
        source = self.base.lifted_cbf
        additional_support = 0.0
        if self.tracking_envelope is not None:
            additional_support = (
                self.tracking_envelope.position_coordinate_half_width_m
                + self.tracking_lookahead_displacement_bound_m
            )
        return DistanceCBFSpec(
            dt_s=source.dt_s,
            max_speed_mps=source.max_speed_mps,
            noise_half_width_m=(
                source.noise_half_width_m + additional_support
            ),
            contraction_kappa=source.contraction_kappa,
            feasibility_tolerance=source.feasibility_tolerance,
        )


def load_gazebo_differential_road_contract(
    path: str | Path,
) -> GazeboDifferentialRoadContract:
    config_path = Path(path).resolve()
    payload = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    if not isinstance(payload, Mapping):
        raise ValueError("Gazebo differential-road config must be a mapping")
    if payload.get("kind") != "dynamic_road_gazebo_classic_cbf_spec":
        raise ValueError("unexpected Gazebo differential-road config kind")

    experiment = payload["experiment"]
    runtime_payload = payload["gazebo_classic"]
    live_display_payload = payload.get("live_display")
    rover_payload = payload["physical_rover"]
    tracking_payload = payload.get("tracking_envelope")
    visual_scene_payload = payload.get("visual_scene", {})
    front_camera_payload = payload.get("front_camera")
    camera_recording_payload = payload.get("camera_recording")
    acceptance_payload = payload["acceptance"]
    base_path = (
        config_path.parent / str(payload["base_differential_config"])
    ).resolve()
    base = load_differential_road_contract(base_path)

    base_size = tuple(float(value) for value in rover_payload["base_size_m"])
    if len(base_size) != 3:
        raise ValueError("physical rover base_size_m must have three values")
    runtime = GazeboClassicRuntimeSpec(
        required_major_version=int(
            runtime_payload["required_major_version"]
        ),
        physics_engine=str(runtime_payload["physics_engine"]),
        world_name=str(runtime_payload["world_name"]),
        model_name=str(runtime_payload["model_name"]),
        master_uri=str(runtime_payload["master_uri"]),
        physics_step_s=float(runtime_payload["physics_step_s"]),
        solver_iterations=int(runtime_payload["solver_iterations"]),
        control_iterations_per_step=int(
            runtime_payload["control_iterations_per_step"]
        ),
        startup_timeout_s=float(runtime_payload["startup_timeout_s"]),
        state_timeout_s=float(runtime_payload["state_timeout_s"]),
        command_plugin=str(runtime_payload["command_plugin"]),
        state_plugin=(
            None
            if runtime_payload.get("state_plugin") is None
            else str(runtime_payload["state_plugin"])
        ),
        plugin_angular_command_sign=float(
            runtime_payload["plugin_angular_command_sign"]
        ),
        bridge_pose_world_timestamp_alignment=str(
            runtime_payload.get(
                "bridge_pose_world_timestamp_alignment",
                "not_required_before_v6",
            )
        ),
        startup_attempt_limit=int(
            runtime_payload.get("startup_attempt_limit", 1)
        ),
        startup_retry_backoff_s=float(
            runtime_payload.get("startup_retry_backoff_s", 0.0)
        ),
        headless_renderer=str(
            runtime_payload.get("headless_renderer", "none")
        ),
    )
    live_display = (
        None
        if live_display_payload is None
        else GazeboLiveDisplaySpec(
            enabled=bool(live_display_payload["enabled"]),
            bind_host=str(live_display_payload["bind_host"]),
            port=int(live_display_payload["port"]),
            display_stride=int(live_display_payload["display_stride"]),
            refresh_interval_s=float(
                live_display_payload["refresh_interval_s"]
            ),
            wall_clock_paced=bool(
                live_display_payload["wall_clock_paced"]
            ),
            atomic_snapshot_updates=bool(
                live_display_payload["atomic_snapshot_updates"]
            ),
            display_sink_control_feedback=bool(
                live_display_payload["display_sink_control_feedback"]
            ),
            obstacle_display_policy=str(
                live_display_payload["obstacle_display_policy"]
            ),
            layout=str(
                live_display_payload.get(
                    "layout", "map_primary_telemetry_visible"
                )
            ),
            camera_certificate_projection=str(
                live_display_payload.get(
                    "camera_certificate_projection", "none"
                )
            ),
            preview_level_h_m=float(
                live_display_payload.get("preview_level_h_m", 0.0)
            ),
        )
    )
    rover = GazeboPhysicalRoverSpec(
        base_length_m=base_size[0],
        base_width_m=base_size[1],
        base_height_m=base_size[2],
        base_mass_kg=float(rover_payload["base_mass_kg"]),
        wheel_radius_m=float(rover_payload["wheel_radius_m"]),
        wheel_width_m=float(rover_payload["wheel_width_m"]),
        wheel_mass_kg=float(rover_payload["wheel_mass_kg"]),
        wheel_separation_m=float(rover_payload["wheel_separation_m"]),
        wheel_axle_x_m=float(rover_payload["wheel_axle_x_m"]),
        rear_caster_radius_m=float(
            rover_payload["rear_caster_radius_m"]
        ),
        rear_caster_x_m=float(rover_payload["rear_caster_x_m"]),
    )
    tracking_envelope = (
        None
        if tracking_payload is None
        else GazeboTrackingEnvelopeSpec(
            source_experiment_id=str(
                tracking_payload["source_experiment_id"]
            ),
            source_trace_sha256=str(
                tracking_payload["source_trace_sha256"]
            ),
            position_coordinate_half_width_m=float(
                tracking_payload["position_coordinate_half_width_m"]
            ),
            heading_error_bound_rad=float(
                tracking_payload["heading_error_bound_rad"]
            ),
        )
    )
    visual_scene = GazeboVisualSceneSpec(
        layout=str(visual_scene_payload.get("layout", "outdoor_road")),
        room_margin_x_m=float(
            visual_scene_payload.get("room_margin_x_m", 2.0)
        ),
        room_half_width_m=float(
            visual_scene_payload.get("room_half_width_m", 3.5)
        ),
        wall_height_m=float(
            visual_scene_payload.get("wall_height_m", 1.4)
        ),
        wall_thickness_m=float(
            visual_scene_payload.get("wall_thickness_m", 0.12)
        ),
    )
    front_camera = (
        None
        if front_camera_payload is None
        else GazeboFrontCameraSpec(
            enabled=bool(front_camera_payload["enabled"]),
            sensor_name=str(front_camera_payload["sensor_name"]),
            plugin=str(front_camera_payload["plugin"]),
            image_width=int(front_camera_payload["image_width"]),
            image_height=int(front_camera_payload["image_height"]),
            horizontal_fov_rad=float(
                front_camera_payload["horizontal_fov_rad"]
            ),
            update_rate_hz=float(front_camera_payload["update_rate_hz"]),
            near_clip_m=float(front_camera_payload["near_clip_m"]),
            far_clip_m=float(front_camera_payload["far_clip_m"]),
            pose_on_base_link=tuple(
                float(value)
                for value in front_camera_payload["pose_on_base_link"]
            ),
            frame_file=str(front_camera_payload["frame_file"]),
            transport=str(front_camera_payload["transport"]),
        )
    )
    camera_recording = (
        None
        if camera_recording_payload is None
        else GazeboCameraRecordingSpec(
            enabled=bool(camera_recording_payload["enabled"]),
            capture_policy=str(camera_recording_payload["capture_policy"]),
            archive_format=str(camera_recording_payload["archive_format"]),
            playback_fps=float(camera_recording_payload["playback_fps"]),
            video_width=int(camera_recording_payload["video_width"]),
            video_height=int(camera_recording_payload["video_height"]),
            video_file=str(camera_recording_payload["video_file"]),
            encoder_backend=str(camera_recording_payload["encoder_backend"]),
            bitrate_kbps=int(camera_recording_payload["bitrate_kbps"]),
            minimum_capture_coverage_ratio=float(
                camera_recording_payload["minimum_capture_coverage_ratio"]
            ),
            maximum_source_frame_gap_s=float(
                camera_recording_payload["maximum_source_frame_gap_s"]
            ),
            minimum_overlay_visible_frame_count=int(
                camera_recording_payload[
                    "minimum_overlay_visible_frame_count"
                ]
            ),
            require_all_activated_obstacles_visible=bool(
                camera_recording_payload[
                    "require_all_activated_obstacles_visible"
                ]
            ),
        )
    )
    acceptance = GazeboRoadAcceptanceSpec(
        required_obstacle_activations=int(
            acceptance_payload["required_obstacle_activations"]
        ),
        maximum_collision_count=int(
            acceptance_payload["maximum_collision_count"]
        ),
        maximum_road_departure_count=int(
            acceptance_payload["maximum_road_departure_count"]
        ),
        minimum_lifted_barrier_m=float(
            acceptance_payload["minimum_lifted_certificate_barrier_m"]
        ),
        minimum_constraint_residual_mps=float(
            acceptance_payload["minimum_cbf_constraint_residual_mps"]
        ),
        minimum_measured_contraction_residual_m=float(
            acceptance_payload[
                "minimum_measured_contraction_residual_m"
            ]
        ),
        maximum_mapping_residual_mps=float(
            acceptance_payload["maximum_mapping_residual_mps"]
        ),
        maximum_position_tracking_residual_m=float(
            acceptance_payload[
                "maximum_position_tracking_residual_m"
            ]
        ),
        maximum_heading_tracking_residual_rad=float(
            acceptance_payload[
                "maximum_heading_tracking_residual_rad"
            ]
        ),
        maximum_absolute_roll_rad=float(
            acceptance_payload["maximum_absolute_roll_rad"]
        ),
        maximum_absolute_pitch_rad=float(
            acceptance_payload["maximum_absolute_pitch_rad"]
        ),
        require_front_camera_frames=bool(
            acceptance_payload.get("require_front_camera_frames", False)
        ),
        minimum_front_camera_capture_coverage_ratio=float(
            acceptance_payload.get(
                "minimum_front_camera_capture_coverage_ratio", 0.0
            )
        ),
        maximum_front_camera_alignment_error_s=float(
            acceptance_payload.get(
                "maximum_front_camera_alignment_error_s", 0.0
            )
        ),
        require_front_camera_activation_keyframes=bool(
            acceptance_payload.get(
                "require_front_camera_activation_keyframes", False
            )
        ),
        require_front_camera_recording=bool(
            acceptance_payload.get("require_front_camera_recording", False)
        ),
        require_front_camera_video=bool(
            acceptance_payload.get("require_front_camera_video", False)
        ),
    )
    return GazeboDifferentialRoadContract(
        config_path=config_path,
        config_file_sha256=file_sha256(config_path),
        experiment_id=str(experiment["id"]),
        phase=str(experiment["phase"]),
        default_seed=int(experiment["default_seed"]),
        base=base,
        runtime=runtime,
        live_display=live_display,
        rover=rover,
        tracking_envelope=tracking_envelope,
        visual_scene=visual_scene,
        front_camera=front_camera,
        camera_recording=camera_recording,
        acceptance=acceptance,
    )
