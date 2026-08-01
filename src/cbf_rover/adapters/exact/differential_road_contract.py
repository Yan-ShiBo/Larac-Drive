"""Configuration binding for the exact differential-road experiment."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Mapping

import yaml

from cbf_rover.domain.differential_drive import (
    StochasticDifferentialDrive2D,
)
from cbf_rover.methods.controllers.lookahead import (
    LookaheadDifferentialMapper,
)
from cbf_rover.methods.online_cbf import DistanceCBFSpec

from .road_contract import (
    NominalRoadControllerSpec,
    RoadExperimentContract,
    file_sha256,
    load_road_experiment_contract,
)


@dataclass(frozen=True, slots=True)
class DifferentialRoadAcceptanceSpec:
    required_obstacle_activations: int
    maximum_collision_count: int
    maximum_road_departure_count: int
    minimum_lifted_barrier_m: float
    minimum_constraint_residual_mps: float
    minimum_realized_contraction_residual_m: float
    maximum_mapping_residual_mps: float

    def __post_init__(self) -> None:
        counts = (
            self.required_obstacle_activations,
            self.maximum_collision_count,
            self.maximum_road_departure_count,
        )
        if any(value < 0 for value in counts):
            raise ValueError("acceptance counts must be nonnegative")
        if self.maximum_mapping_residual_mps < 0.0:
            raise ValueError("mapping residual gate must be nonnegative")


@dataclass(frozen=True, slots=True)
class DifferentialRoadContract:
    config_path: Path
    config_file_sha256: str
    experiment_id: str
    phase: str
    default_seed: int
    base: RoadExperimentContract
    system: StochasticDifferentialDrive2D
    initial_heading_rad: float
    nominal_controller: NominalRoadControllerSpec
    mapper: LookaheadDifferentialMapper
    lifted_cbf: DistanceCBFSpec
    additional_body_inflation_m: float
    heading_remainder_bound_m: float
    max_steps: int
    display_stride: int
    acceptance: DifferentialRoadAcceptanceSpec

    def __post_init__(self) -> None:
        if not self.experiment_id.strip():
            raise ValueError("experiment_id must not be empty")
        if self.phase != "discovery":
            raise ValueError("first differential contract must be discovery")
        if self.default_seed < 0:
            raise ValueError("default seed must be nonnegative")
        if self.max_steps <= 0 or self.display_stride <= 0:
            raise ValueError("episode bounds must be positive")
        if self.system.dt_s != self.base.system.dt_s:
            raise ValueError("base and differential time steps must match")
        if self.system.noise != self.base.system.noise:
            raise ValueError("differential position noise must inherit N1")
        if (
            self.nominal_controller.max_nominal_speed_mps
            > self.base.system.max_speed_mps
        ):
            raise ValueError("nominal controller exceeds planar speed disk")
        if (
            self.mapper.lookahead_distance_m
            != self.additional_body_inflation_m
        ):
            raise ValueError("body inflation must equal lookahead distance")
        if self.mapper.max_linear_mps != self.system.max_linear_mps:
            raise ValueError("mapper and system linear bounds must match")
        if self.mapper.max_angular_radps != self.system.max_angular_radps:
            raise ValueError("mapper and system angular bounds must match")
        if self.mapper.allow_reverse != self.system.allow_reverse:
            raise ValueError("mapper and system reverse contracts must match")
        expected_noise_support = (
            self.system.noise.half_width_m
            + self.heading_remainder_bound_m
        )
        if abs(
            self.lifted_cbf.noise_half_width_m
            - expected_noise_support
        ) > 1.0e-15:
            raise ValueError("lifted CBF disturbance support is inconsistent")


def load_differential_road_contract(
    path: str | Path,
) -> DifferentialRoadContract:
    config_path = Path(path).resolve()
    payload = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    if not isinstance(payload, Mapping):
        raise ValueError("differential-road config must be a mapping")
    if payload.get("kind") != "dynamic_road_differential_cbf_spec":
        raise ValueError("unexpected differential-road config kind")

    experiment_payload = payload["experiment"]
    model_payload = payload["model"]
    mapping_payload = payload["lookahead_mapping"]
    lifted_payload = payload["lifted_online_cbf"]
    episode_payload = payload["episode"]
    acceptance_payload = payload["acceptance"]
    base_path = (
        config_path.parent / str(payload["base_scenario_config"])
    ).resolve()
    base = load_road_experiment_contract(base_path)
    nominal_payload = payload.get("nominal_controller")
    if nominal_payload is None:
        nominal = base.nominal_controller
    else:
        nominal = NominalRoadControllerSpec(
            controller_id=str(nominal_payload["id"]),
            forward_speed_mps=float(
                nominal_payload["forward_speed_mps"]
            ),
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

    system = StochasticDifferentialDrive2D(
        dt_s=float(model_payload["dt_s"]),
        max_linear_mps=float(model_payload["max_linear_mps"]),
        max_angular_radps=float(model_payload["max_angular_radps"]),
        allow_reverse=bool(model_payload["allow_reverse"]),
        noise=base.system.noise,
        model_id=str(model_payload["id"]),
    )
    mapper = LookaheadDifferentialMapper(
        lookahead_distance_m=float(
            mapping_payload["lookahead_distance_m"]
        ),
        max_linear_mps=system.max_linear_mps,
        max_angular_radps=system.max_angular_radps,
        allow_reverse=system.allow_reverse,
        feasibility_tolerance=base.cbf.feasibility_tolerance,
        mapper_id=str(mapping_payload["id"]),
    )
    remainder_bound = mapper.discrete_heading_remainder_bound_m(
        dt_s=system.dt_s
    )
    lifted_cbf = DistanceCBFSpec(
        dt_s=system.dt_s,
        max_speed_mps=base.system.max_speed_mps,
        noise_half_width_m=(
            system.noise.half_width_m + remainder_bound
        ),
        contraction_kappa=base.cbf.contraction_kappa,
        feasibility_tolerance=base.cbf.feasibility_tolerance,
    )
    acceptance = DifferentialRoadAcceptanceSpec(
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
            acceptance_payload[
                "minimum_lifted_certificate_barrier_m"
            ]
        ),
        minimum_constraint_residual_mps=float(
            acceptance_payload[
                "minimum_cbf_constraint_residual_mps"
            ]
        ),
        minimum_realized_contraction_residual_m=float(
            acceptance_payload[
                "minimum_realized_contraction_residual_m"
            ]
        ),
        maximum_mapping_residual_mps=float(
            acceptance_payload["maximum_mapping_residual_mps"]
        ),
    )
    return DifferentialRoadContract(
        config_path=config_path,
        config_file_sha256=file_sha256(config_path),
        experiment_id=str(experiment_payload["id"]),
        phase=str(experiment_payload["phase"]),
        default_seed=int(experiment_payload["default_seed"]),
        base=base,
        system=system,
        initial_heading_rad=float(model_payload["initial_heading_rad"]),
        nominal_controller=nominal,
        mapper=mapper,
        lifted_cbf=lifted_cbf,
        additional_body_inflation_m=float(
            lifted_payload["additional_body_inflation_m"]
        ),
        heading_remainder_bound_m=remainder_bound,
        max_steps=int(episode_payload["max_steps"]),
        display_stride=int(episode_payload["display_stride"]),
        acceptance=acceptance,
    )
