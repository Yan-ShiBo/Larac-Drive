"""Pure nonholonomic state and dynamics for differential-drive experiments."""

from __future__ import annotations

from dataclasses import dataclass
from math import cos, isfinite, pi, sin
from typing import Iterable

from .planar import PlanarState, SymmetricTriangularNoise2D


def _finite(value: float, name: str) -> float:
    result = float(value)
    if not isfinite(result):
        raise ValueError(f"{name} must be finite")
    return result


def wrap_heading(angle_rad: float) -> float:
    """Map a finite angle to the half-open interval ``[-pi, pi)``."""

    value = _finite(angle_rad, "angle_rad")
    return (value + pi) % (2.0 * pi) - pi


@dataclass(frozen=True, slots=True)
class DifferentialDriveState:
    """World position and yaw of a planar differential-drive body."""

    x_m: float
    y_m: float
    heading_rad: float

    def __post_init__(self) -> None:
        object.__setattr__(self, "x_m", _finite(self.x_m, "x_m"))
        object.__setattr__(self, "y_m", _finite(self.y_m, "y_m"))
        object.__setattr__(
            self,
            "heading_rad",
            wrap_heading(self.heading_rad),
        )

    @property
    def center(self) -> PlanarState:
        return PlanarState(self.x_m, self.y_m)

    def lookahead_point(self, distance_m: float) -> PlanarState:
        distance = _finite(distance_m, "distance_m")
        if distance <= 0.0:
            raise ValueError("lookahead distance must be positive")
        return PlanarState(
            self.x_m + distance * cos(self.heading_rad),
            self.y_m + distance * sin(self.heading_rad),
        )


@dataclass(frozen=True, slots=True)
class DifferentialDriveAction:
    """Linear and angular velocity command ``[v, omega]``."""

    linear_mps: float
    angular_radps: float

    def __post_init__(self) -> None:
        object.__setattr__(
            self,
            "linear_mps",
            _finite(self.linear_mps, "linear_mps"),
        )
        object.__setattr__(
            self,
            "angular_radps",
            _finite(self.angular_radps, "angular_radps"),
        )


@dataclass(frozen=True, slots=True)
class StochasticDifferentialDrive2D:
    """Euler unicycle transition with bounded additive position noise."""

    dt_s: float = 0.05
    max_linear_mps: float = 0.5
    max_angular_radps: float = 1.5
    allow_reverse: bool = False
    noise: SymmetricTriangularNoise2D = SymmetricTriangularNoise2D()
    model_id: str = "M2_STOCHASTIC_DIFFERENTIAL_DRIVE_EULER"

    def __post_init__(self) -> None:
        for field_name in (
            "dt_s",
            "max_linear_mps",
            "max_angular_radps",
        ):
            object.__setattr__(
                self,
                field_name,
                _finite(getattr(self, field_name), field_name),
            )
        if (
            self.dt_s <= 0.0
            or self.max_linear_mps <= 0.0
            or self.max_angular_radps <= 0.0
        ):
            raise ValueError("time step and command bounds must be positive")
        if not self.model_id.strip():
            raise ValueError("model_id must not be empty")

    def action_is_admissible(
        self,
        action: DifferentialDriveAction,
        *,
        tolerance: float = 0.0,
    ) -> bool:
        tolerance_value = _finite(tolerance, "tolerance")
        if tolerance_value < 0.0:
            raise ValueError("tolerance must be nonnegative")
        lower_linear = (
            -self.max_linear_mps if self.allow_reverse else 0.0
        )
        return (
            lower_linear - tolerance_value
            <= action.linear_mps
            <= self.max_linear_mps + tolerance_value
            and abs(action.angular_radps)
            <= self.max_angular_radps + tolerance_value
        )

    def successor(
        self,
        state: DifferentialDriveState,
        action: DifferentialDriveAction,
        noise_m: Iterable[float],
        *,
        feasibility_tolerance: float = 0.0,
    ) -> DifferentialDriveState:
        if not self.action_is_admissible(
            action,
            tolerance=feasibility_tolerance,
        ):
            raise ValueError("differential-drive action is inadmissible")
        values = tuple(float(value) for value in noise_m)
        if len(values) != 2 or not all(isfinite(value) for value in values):
            raise ValueError("noise_m must contain two finite coordinates")
        if any(abs(value) > self.noise.half_width_m for value in values):
            raise ValueError("noise sample lies outside the declared support")
        heading = state.heading_rad
        return DifferentialDriveState(
            x_m=(
                state.x_m
                + self.dt_s * action.linear_mps * cos(heading)
                + values[0]
            ),
            y_m=(
                state.y_m
                + self.dt_s * action.linear_mps * sin(heading)
                + values[1]
            ),
            heading_rad=(
                heading + self.dt_s * action.angular_radps
            ),
        )
