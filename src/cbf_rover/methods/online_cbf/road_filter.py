"""Robust one-step distance-CBF constraints for the dynamic road."""

from __future__ import annotations

from dataclasses import dataclass
from math import hypot, isfinite
from typing import Iterable

from cbf_rover.domain.dynamic_road import RoadGeometry, RoadObstacle
from cbf_rover.domain.planar import PlanarAction, PlanarState

from .projection import (
    LinearHalfspace2D,
    ProjectionResult,
    project_action_to_halfspaces_and_disk,
)


@dataclass(frozen=True, slots=True)
class DistanceCBFSpec:
    """Runtime constants for the robust distance-CBF inequalities."""

    dt_s: float
    max_speed_mps: float
    noise_half_width_m: float
    contraction_kappa: float
    feasibility_tolerance: float = 1.0e-10

    def __post_init__(self) -> None:
        values = (
            self.dt_s,
            self.max_speed_mps,
            self.noise_half_width_m,
            self.contraction_kappa,
            self.feasibility_tolerance,
        )
        if not all(isfinite(float(value)) for value in values):
            raise ValueError("CBF spec values must be finite")
        if self.dt_s <= 0.0 or self.max_speed_mps <= 0.0:
            raise ValueError("CBF dt and max speed must be positive")
        if self.noise_half_width_m < 0.0:
            raise ValueError("noise half-width must be nonnegative")
        if not 0.0 < self.contraction_kappa <= 1.0:
            raise ValueError("contraction kappa must lie in (0,1]")
        if self.feasibility_tolerance < 0.0:
            raise ValueError("feasibility tolerance must be nonnegative")

    def robust_rhs(
        self,
        *,
        barrier_m: float,
        normal_x: float,
        normal_y: float,
    ) -> float:
        return (
            -(self.contraction_kappa / self.dt_s) * barrier_m
            + (self.noise_half_width_m / self.dt_s)
            * (abs(normal_x) + abs(normal_y))
        )


@dataclass(frozen=True, slots=True)
class BarrierConstraintFact:
    """One evaluated barrier and its matching action inequality."""

    barrier_id: str
    barrier_kind: str
    barrier_m: float
    normal_x: float
    normal_y: float
    robust_rhs_mps: float
    constraint: LinearHalfspace2D


@dataclass(frozen=True, slots=True)
class RoadFilterResult:
    """Filtered action plus the exact barriers used to obtain it."""

    nominal_action: PlanarAction
    facts: tuple[BarrierConstraintFact, ...]
    projection: ProjectionResult

    @property
    def filtered_action(self) -> PlanarAction:
        return self.projection.action

    @property
    def intervention_norm_mps(self) -> float:
        return hypot(
            self.filtered_action.u_x_mps - self.nominal_action.u_x_mps,
            self.filtered_action.u_y_mps - self.nominal_action.u_y_mps,
        )


def _fact(
    *,
    barrier_id: str,
    barrier_kind: str,
    barrier_m: float,
    normal_x: float,
    normal_y: float,
    spec: DistanceCBFSpec,
) -> BarrierConstraintFact:
    rhs = spec.robust_rhs(
        barrier_m=barrier_m,
        normal_x=normal_x,
        normal_y=normal_y,
    )
    constraint = LinearHalfspace2D(
        name=barrier_id,
        a_x=normal_x,
        a_y=normal_y,
        b=rhs,
    )
    return BarrierConstraintFact(
        barrier_id=barrier_id,
        barrier_kind=barrier_kind,
        barrier_m=barrier_m,
        normal_x=normal_x,
        normal_y=normal_y,
        robust_rhs_mps=rhs,
        constraint=constraint,
    )


def build_dynamic_road_barriers(
    *,
    state: PlanarState,
    geometry: RoadGeometry,
    active_obstacles: Iterable[RoadObstacle],
    spec: DistanceCBFSpec,
    additional_inflation_m: float = 0.0,
) -> tuple[BarrierConstraintFact, ...]:
    """Evaluate road-edge and visible-obstacle barriers at one state."""

    inflation = float(additional_inflation_m)
    if not isfinite(inflation) or inflation < 0.0:
        raise ValueError("additional inflation must be finite and nonnegative")
    lower_bound = geometry.center_y_min_m + inflation
    upper_bound = geometry.center_y_max_m - inflation
    if lower_bound >= upper_bound:
        raise ValueError("additional inflation eliminates the road-safe strip")
    facts = [
        _fact(
            barrier_id="road_lower_edge",
            barrier_kind="road_edge",
            barrier_m=state.y_m - lower_bound,
            normal_x=0.0,
            normal_y=1.0,
            spec=spec,
        ),
        _fact(
            barrier_id="road_upper_edge",
            barrier_kind="road_edge",
            barrier_m=upper_bound - state.y_m,
            normal_x=0.0,
            normal_y=-1.0,
            spec=spec,
        ),
    ]
    for obstacle in active_obstacles:
        delta_x = state.x_m - obstacle.center.x_m
        delta_y = state.y_m - obstacle.center.y_m
        distance = hypot(delta_x, delta_y)
        if distance <= 1.0e-12:
            raise ValueError(
                f"state coincides with obstacle center {obstacle.obstacle_id}"
            )
        facts.append(
            _fact(
                barrier_id=f"obstacle:{obstacle.obstacle_id}",
                barrier_kind="obstacle",
                barrier_m=(
                    distance
                    - obstacle.certificate_radius_m
                    - inflation
                ),
                normal_x=delta_x / distance,
                normal_y=delta_y / distance,
                spec=spec,
            )
        )
    return tuple(facts)


def filter_dynamic_road_action(
    *,
    state: PlanarState,
    nominal_action: PlanarAction,
    geometry: RoadGeometry,
    active_obstacles: Iterable[RoadObstacle],
    spec: DistanceCBFSpec,
    additional_inflation_m: float = 0.0,
    additional_constraints: Iterable[LinearHalfspace2D] = (),
) -> RoadFilterResult:
    """Return the minimum-change action satisfying all current CBFs."""

    facts = build_dynamic_road_barriers(
        state=state,
        geometry=geometry,
        active_obstacles=active_obstacles,
        spec=spec,
        additional_inflation_m=additional_inflation_m,
    )
    command_constraints = tuple(additional_constraints)
    projection = project_action_to_halfspaces_and_disk(
        nominal_action,
        (
            *(fact.constraint for fact in facts),
            *command_constraints,
        ),
        max_speed_mps=spec.max_speed_mps,
        tolerance=spec.feasibility_tolerance,
    )
    return RoadFilterResult(
        nominal_action=nominal_action,
        facts=facts,
        projection=projection,
    )
