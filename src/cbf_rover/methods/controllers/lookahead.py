"""Convex lookahead-point mapping for differential-drive commands."""

from __future__ import annotations

from dataclasses import dataclass
from math import cos, hypot, isfinite, sin

from cbf_rover.domain.differential_drive import (
    DifferentialDriveAction,
    DifferentialDriveState,
)
from cbf_rover.domain.planar import PlanarAction
from cbf_rover.methods.online_cbf.projection import LinearHalfspace2D


@dataclass(frozen=True, slots=True)
class LookaheadMappingResult:
    """Exact command mapping and its round-trip audit facts."""

    action: DifferentialDriveAction
    realized_lookahead_velocity: PlanarAction
    mapping_residual_mps: float
    saturation_applied: bool = False


@dataclass(frozen=True, slots=True)
class LookaheadDifferentialMapper:
    """Map a reachable lookahead velocity to bounded ``[v, omega]``."""

    lookahead_distance_m: float = 0.15
    max_linear_mps: float = 0.5
    max_angular_radps: float = 1.5
    allow_reverse: bool = False
    feasibility_tolerance: float = 1.0e-10
    mapper_id: str = "D2_LOOKAHEAD_DIFFERENTIAL_MAPPER_V1"

    def __post_init__(self) -> None:
        values = (
            self.lookahead_distance_m,
            self.max_linear_mps,
            self.max_angular_radps,
            self.feasibility_tolerance,
        )
        if not all(isfinite(float(value)) for value in values):
            raise ValueError("lookahead mapper constants must be finite")
        if (
            self.lookahead_distance_m <= 0.0
            or self.max_linear_mps <= 0.0
            or self.max_angular_radps <= 0.0
        ):
            raise ValueError("lookahead and command bounds must be positive")
        if self.feasibility_tolerance < 0.0:
            raise ValueError("feasibility tolerance must be nonnegative")
        if not self.mapper_id.strip():
            raise ValueError("mapper_id must not be empty")

    def reachable_velocity_constraints(
        self,
        *,
        heading_rad: float,
    ) -> tuple[LinearHalfspace2D, ...]:
        """Return the world-velocity halfspaces induced by command bounds."""

        heading = float(heading_rad)
        if not isfinite(heading):
            raise ValueError("heading must be finite")
        forward_x = cos(heading)
        forward_y = sin(heading)
        lateral_x = -forward_y
        lateral_y = forward_x
        minimum_linear = (
            -self.max_linear_mps if self.allow_reverse else 0.0
        )
        angular_velocity_bound = (
            self.lookahead_distance_m * self.max_angular_radps
        )
        return (
            LinearHalfspace2D(
                name="differential_linear_min",
                a_x=forward_x,
                a_y=forward_y,
                b=minimum_linear,
            ),
            LinearHalfspace2D(
                name="differential_linear_max",
                a_x=-forward_x,
                a_y=-forward_y,
                b=-self.max_linear_mps,
            ),
            LinearHalfspace2D(
                name="differential_angular_min",
                a_x=lateral_x,
                a_y=lateral_y,
                b=-angular_velocity_bound,
            ),
            LinearHalfspace2D(
                name="differential_angular_max",
                a_x=-lateral_x,
                a_y=-lateral_y,
                b=-angular_velocity_bound,
            ),
        )

    def command(
        self,
        desired_lookahead_velocity: PlanarAction,
        *,
        state: DifferentialDriveState,
    ) -> LookaheadMappingResult:
        """Invert the lookahead Jacobian without command clipping."""

        heading = state.heading_rad
        forward_x = cos(heading)
        forward_y = sin(heading)
        lateral_x = -forward_y
        lateral_y = forward_x
        linear = (
            forward_x * desired_lookahead_velocity.u_x_mps
            + forward_y * desired_lookahead_velocity.u_y_mps
        )
        angular = (
            lateral_x * desired_lookahead_velocity.u_x_mps
            + lateral_y * desired_lookahead_velocity.u_y_mps
        ) / self.lookahead_distance_m
        action = DifferentialDriveAction(linear, angular)
        lower_linear = (
            -self.max_linear_mps if self.allow_reverse else 0.0
        )
        tolerance = self.feasibility_tolerance
        if not (
            lower_linear - tolerance
            <= action.linear_mps
            <= self.max_linear_mps + tolerance
            and abs(action.angular_radps)
            <= self.max_angular_radps + tolerance
        ):
            raise ValueError(
                "lookahead velocity lies outside the reachable command set"
            )
        realized = PlanarAction(
            forward_x * linear
            + lateral_x * self.lookahead_distance_m * angular,
            forward_y * linear
            + lateral_y * self.lookahead_distance_m * angular,
        )
        residual = hypot(
            realized.u_x_mps - desired_lookahead_velocity.u_x_mps,
            realized.u_y_mps - desired_lookahead_velocity.u_y_mps,
        )
        return LookaheadMappingResult(
            action=action,
            realized_lookahead_velocity=realized,
            mapping_residual_mps=residual,
        )

    def discrete_heading_remainder_bound_m(self, *, dt_s: float) -> float:
        """Bound the Euler lookahead-point linearization remainder per step."""

        dt = float(dt_s)
        if not isfinite(dt) or dt <= 0.0:
            raise ValueError("dt_s must be finite and positive")
        delta = dt * self.max_angular_radps
        return self.lookahead_distance_m * hypot(
            cos(delta) - 1.0,
            sin(delta) - delta,
        )
