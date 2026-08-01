"""Configuration binding and seeded obstacle plans for the dynamic road."""

from __future__ import annotations

from dataclasses import dataclass
from hashlib import sha256
import json
from pathlib import Path
from typing import Any, Mapping, Sequence

import numpy as np
import yaml

from cbf_rover.domain.dynamic_road import (
    DynamicRoadScenario,
    RoadGeometry,
    RoadObstacle,
)
from cbf_rover.domain.planar import (
    PlanarState,
    StochasticSingleIntegrator2D,
    SymmetricTriangularNoise2D,
)
from cbf_rover.methods.online_cbf import DistanceCBFSpec


def file_sha256(path: str | Path) -> str:
    digest = sha256()
    with Path(path).open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def canonical_content_sha256(payload: Mapping[str, Any]) -> str:
    encoded = json.dumps(
        payload,
        sort_keys=True,
        separators=(",", ":"),
        ensure_ascii=False,
        allow_nan=False,
    ).encode("utf-8")
    return sha256(encoded).hexdigest()


def _float_pair(value: Any, name: str) -> tuple[float, float]:
    if not isinstance(value, Sequence) or isinstance(value, (str, bytes)):
        raise ValueError(f"{name} must contain two numbers")
    values = tuple(float(item) for item in value)
    if len(values) != 2:
        raise ValueError(f"{name} must contain two numbers")
    return values[0], values[1]


@dataclass(frozen=True, slots=True)
class ObstacleSamplingSpec:
    count: int
    x_anchors_m: tuple[float, ...]
    x_jitter_m: float
    physical_radius_range_m: tuple[float, float]
    appearance_distance_range_m: tuple[float, float]
    safety_margin_m: float

    def __post_init__(self) -> None:
        if self.count <= 0 or self.count != len(self.x_anchors_m):
            raise ValueError("obstacle count must match x anchors")
        if tuple(sorted(self.x_anchors_m)) != self.x_anchors_m:
            raise ValueError("obstacle x anchors must be ordered")
        if self.x_jitter_m < 0.0:
            raise ValueError("obstacle x jitter must be nonnegative")
        radius_low, radius_high = self.physical_radius_range_m
        reveal_low, reveal_high = self.appearance_distance_range_m
        if not 0.0 < radius_low <= radius_high:
            raise ValueError("obstacle radius range is invalid")
        if not 0.0 < reveal_low <= reveal_high:
            raise ValueError("appearance distance range is invalid")
        if self.safety_margin_m < 0.0:
            raise ValueError("obstacle safety margin must be nonnegative")
        if self.count > 1:
            minimum_anchor_gap = min(
                second - first
                for first, second in zip(
                    self.x_anchors_m,
                    self.x_anchors_m[1:],
                )
            )
            if minimum_anchor_gap <= 2.0 * self.x_jitter_m:
                raise ValueError("x jitter could reorder obstacle anchors")


@dataclass(frozen=True, slots=True)
class NominalRoadControllerSpec:
    controller_id: str
    forward_speed_mps: float
    max_nominal_speed_mps: float
    lateral_gain_per_s: float
    lane_switch_lookahead_m: float
    obstacle_pass_margin_m: float
    finish_slowdown_distance_m: float
    ignore_passed_obstacles_for_lane_selection: bool = False

    def __post_init__(self) -> None:
        if not self.controller_id.strip():
            raise ValueError("nominal controller id must not be empty")
        values = (
            self.forward_speed_mps,
            self.max_nominal_speed_mps,
            self.lateral_gain_per_s,
            self.lane_switch_lookahead_m,
            self.obstacle_pass_margin_m,
            self.finish_slowdown_distance_m,
        )
        if any(value <= 0.0 for value in values):
            raise ValueError("nominal controller constants must be positive")
        if self.forward_speed_mps > self.max_nominal_speed_mps:
            raise ValueError("forward speed exceeds nominal action radius")


@dataclass(frozen=True, slots=True)
class RoadAcceptanceSpec:
    required_obstacle_activations: int
    maximum_collision_count: int
    maximum_road_departure_count: int
    minimum_certificate_barrier_m: float
    minimum_constraint_residual_mps: float
    minimum_filter_intervention_step_count: int

    def __post_init__(self) -> None:
        counts = (
            self.required_obstacle_activations,
            self.maximum_collision_count,
            self.maximum_road_departure_count,
            self.minimum_filter_intervention_step_count,
        )
        if any(count < 0 for count in counts):
            raise ValueError("acceptance counts must be nonnegative")


@dataclass(frozen=True, slots=True)
class RoadExperimentContract:
    config_path: Path
    config_file_sha256: str
    scenario_id: str
    default_seed: int
    geometry: RoadGeometry
    system: StochasticSingleIntegrator2D
    obstacle_sampling: ObstacleSamplingSpec
    nominal_controller: NominalRoadControllerSpec
    cbf: DistanceCBFSpec
    max_steps: int
    display_stride: int
    acceptance: RoadAcceptanceSpec

    def __post_init__(self) -> None:
        if self.max_steps <= 0 or self.display_stride <= 0:
            raise ValueError("episode steps and display stride must be positive")
        if self.nominal_controller.max_nominal_speed_mps > (
            self.system.max_speed_mps
        ):
            raise ValueError("nominal action radius exceeds S1 speed disk")
        if self.cbf.dt_s != self.system.dt_s:
            raise ValueError("CBF and system time steps must match")
        if self.cbf.max_speed_mps != self.system.max_speed_mps:
            raise ValueError("CBF and system action radii must match")
        if (
            self.cbf.noise_half_width_m
            != self.system.noise.half_width_m
        ):
            raise ValueError("CBF and system noise supports must match")
        earliest_first_activation = (
            self.obstacle_sampling.x_anchors_m[0]
            - self.obstacle_sampling.x_jitter_m
            - self.obstacle_sampling.appearance_distance_range_m[1]
        )
        if earliest_first_activation <= self.geometry.start.x_m:
            raise ValueError(
                "every seeded obstacle must appear after the run starts"
            )
        latest_final_certificate_edge = (
            self.obstacle_sampling.x_anchors_m[-1]
            + self.obstacle_sampling.x_jitter_m
            + self.obstacle_sampling.physical_radius_range_m[1]
            + self.geometry.rover_radius_m
            + self.obstacle_sampling.safety_margin_m
        )
        if latest_final_certificate_edge >= self.geometry.finish_line_x_m:
            raise ValueError(
                "finish line must lie beyond every certificate obstacle"
            )


def load_road_experiment_contract(
    path: str | Path,
) -> RoadExperimentContract:
    config_path = Path(path).resolve()
    payload = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    if not isinstance(payload, Mapping):
        raise ValueError("dynamic-road config must be a mapping")
    if payload.get("kind") != "dynamic_road_online_cbf_spec":
        raise ValueError("unexpected dynamic-road config kind")

    scenario_payload = payload["scenario"]
    system_payload = payload["system"]
    noise_payload = system_payload["noise"]
    road_payload = payload["road"]
    obstacle_payload = payload["random_obstacles"]
    nominal_payload = payload["nominal_controller"]
    cbf_payload = payload["online_cbf"]
    episode_payload = payload["episode"]
    acceptance_payload = payload["acceptance"]

    x_min, x_max = _float_pair(
        road_payload["x_bounds_m"],
        "road.x_bounds_m",
    )
    y_min, y_max = _float_pair(
        road_payload["physical_y_bounds_m"],
        "road.physical_y_bounds_m",
    )
    lane_low, lane_high = _float_pair(
        road_payload["lane_centers_y_m"],
        "road.lane_centers_y_m",
    )
    start_x, start_y = _float_pair(
        road_payload["start_m"],
        "road.start_m",
    )
    geometry = RoadGeometry(
        x_min_m=x_min,
        x_max_m=x_max,
        physical_y_min_m=y_min,
        physical_y_max_m=y_max,
        lane_centers_y_m=(lane_low, lane_high),
        start=PlanarState(start_x, start_y),
        finish_line_x_m=float(road_payload["finish_line_x_m"]),
        rover_radius_m=float(
            road_payload["rover_circumscribed_radius_m"]
        ),
    )
    system = StochasticSingleIntegrator2D(
        dt_s=float(system_payload["dt_s"]),
        max_speed_mps=float(system_payload["max_speed_mps"]),
        noise=SymmetricTriangularNoise2D(
            half_width_m=float(noise_payload["half_width_m"]),
            distribution_id=str(noise_payload["id"]),
        ),
    )
    radius_range = _float_pair(
        obstacle_payload["physical_radius_range_m"],
        "random_obstacles.physical_radius_range_m",
    )
    reveal_range = _float_pair(
        obstacle_payload["appearance_distance_range_m"],
        "random_obstacles.appearance_distance_range_m",
    )
    obstacle_sampling = ObstacleSamplingSpec(
        count=int(obstacle_payload["count"]),
        x_anchors_m=tuple(
            float(value) for value in obstacle_payload["x_anchor_m"]
        ),
        x_jitter_m=float(obstacle_payload["x_jitter_m"]),
        physical_radius_range_m=radius_range,
        appearance_distance_range_m=reveal_range,
        safety_margin_m=float(
            obstacle_payload["certificate_safety_margin_m"]
        ),
    )
    nominal = NominalRoadControllerSpec(
        controller_id=str(nominal_payload["id"]),
        forward_speed_mps=float(nominal_payload["forward_speed_mps"]),
        max_nominal_speed_mps=float(
            nominal_payload["max_nominal_speed_mps"]
        ),
        lateral_gain_per_s=float(
            nominal_payload["lateral_gain_per_s"]
        ),
        lane_switch_lookahead_m=float(
            nominal_payload["lane_switch_lookahead_m"]
        ),
        obstacle_pass_margin_m=float(
            nominal_payload["obstacle_pass_margin_m"]
        ),
        finish_slowdown_distance_m=float(
            nominal_payload["finish_slowdown_distance_m"]
        ),
        ignore_passed_obstacles_for_lane_selection=bool(
            nominal_payload.get(
                "ignore_passed_obstacles_for_lane_selection",
                False,
            )
        ),
    )
    cbf = DistanceCBFSpec(
        dt_s=system.dt_s,
        max_speed_mps=system.max_speed_mps,
        noise_half_width_m=system.noise.half_width_m,
        contraction_kappa=float(
            cbf_payload["discrete_contraction_kappa"]
        ),
        feasibility_tolerance=float(
            cbf_payload["feasibility_tolerance"]
        ),
    )
    acceptance = RoadAcceptanceSpec(
        required_obstacle_activations=int(
            acceptance_payload["required_obstacle_activations"]
        ),
        maximum_collision_count=int(
            acceptance_payload["maximum_collision_count"]
        ),
        maximum_road_departure_count=int(
            acceptance_payload["maximum_road_departure_count"]
        ),
        minimum_certificate_barrier_m=float(
            acceptance_payload["minimum_certificate_barrier_m"]
        ),
        minimum_constraint_residual_mps=float(
            acceptance_payload[
                "minimum_cbf_constraint_residual_mps"
            ]
        ),
        minimum_filter_intervention_step_count=int(
            acceptance_payload[
                "minimum_filter_intervention_step_count"
            ]
        ),
    )
    return RoadExperimentContract(
        config_path=config_path,
        config_file_sha256=file_sha256(config_path),
        scenario_id=str(scenario_payload["id"]),
        default_seed=int(obstacle_payload["default_seed"]),
        geometry=geometry,
        system=system,
        obstacle_sampling=obstacle_sampling,
        nominal_controller=nominal,
        cbf=cbf,
        max_steps=int(episode_payload["max_steps"]),
        display_stride=int(episode_payload["display_stride"]),
        acceptance=acceptance,
    )


def generate_seeded_road_scenario(
    contract: RoadExperimentContract,
    *,
    seed: int,
) -> DynamicRoadScenario:
    """Generate the complete hidden obstacle plan from one recorded seed."""

    rng = np.random.default_rng(int(seed))
    sample = contract.obstacle_sampling
    obstacles: list[RoadObstacle] = []
    for index, anchor_x in enumerate(sample.x_anchors_m, start=1):
        center_x = anchor_x + float(
            rng.uniform(-sample.x_jitter_m, sample.x_jitter_m)
        )
        lane_index = int(rng.integers(0, 2))
        center_y = contract.geometry.lane_centers_y_m[lane_index]
        physical_radius = float(
            rng.uniform(*sample.physical_radius_range_m)
        )
        appearance_distance = float(
            rng.uniform(*sample.appearance_distance_range_m)
        )
        obstacles.append(
            RoadObstacle(
                obstacle_id=f"obstacle_{index:02d}",
                center=PlanarState(center_x, center_y),
                lane_index=lane_index,
                physical_radius_m=physical_radius,
                safety_margin_m=sample.safety_margin_m,
                rover_radius_m=contract.geometry.rover_radius_m,
                appearance_distance_m=appearance_distance,
            )
        )
    return DynamicRoadScenario(
        scenario_id=contract.scenario_id,
        geometry=contract.geometry,
        obstacles=tuple(obstacles),
    )


def obstacle_plan_payload(
    *,
    scenario: DynamicRoadScenario,
    seed: int,
) -> dict[str, Any]:
    payload: dict[str, Any] = {
        "schema_version": 1,
        "kind": "seeded_dynamic_road_obstacle_plan",
        "scenario_id": scenario.scenario_id,
        "seed": int(seed),
        "random_generator": "numpy.default_rng.PCG64",
        "obstacles": [
            {
                "obstacle_id": obstacle.obstacle_id,
                "center_m": [
                    obstacle.center.x_m,
                    obstacle.center.y_m,
                ],
                "lane_index": obstacle.lane_index,
                "physical_radius_m": obstacle.physical_radius_m,
                "rover_collision_radius_m": obstacle.collision_radius_m,
                "certificate_radius_m": obstacle.certificate_radius_m,
                "certificate_safety_margin_m": obstacle.safety_margin_m,
                "appearance_distance_m": obstacle.appearance_distance_m,
                "activation_x_m": obstacle.activation_x_m,
            }
            for obstacle in scenario.obstacles
        ],
    }
    payload["content_sha256"] = canonical_content_sha256(payload)
    return payload
