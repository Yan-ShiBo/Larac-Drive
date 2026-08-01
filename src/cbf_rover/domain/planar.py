"""Pure mathematical objects for the first Rover q-SBC slice.

This module deliberately has no NumPy, Torch, ROS, Gazebo, solver, or file-system
dependency.  It is the authoritative executable form of the M1/S1/P1/D1/N1
research contract.
"""

from __future__ import annotations

from dataclasses import dataclass
from math import isfinite
from typing import Iterable


def _finite(value: float, name: str) -> float:
    result = float(value)
    if not isfinite(result):
        raise ValueError(f"{name} must be finite")
    return result


@dataclass(frozen=True, slots=True)
class PlanarState:
    """Position state ``[x_m, y_m]`` in the world frame."""

    x_m: float
    y_m: float

    def __post_init__(self) -> None:
        object.__setattr__(self, "x_m", _finite(self.x_m, "x_m"))
        object.__setattr__(self, "y_m", _finite(self.y_m, "y_m"))

    def as_tuple(self) -> tuple[float, float]:
        return self.x_m, self.y_m


@dataclass(frozen=True, slots=True)
class PlanarAction:
    """World-frame velocity request ``[u_x_mps, u_y_mps]``."""

    u_x_mps: float
    u_y_mps: float

    def __post_init__(self) -> None:
        object.__setattr__(self, "u_x_mps", _finite(self.u_x_mps, "u_x_mps"))
        object.__setattr__(self, "u_y_mps", _finite(self.u_y_mps, "u_y_mps"))

    @property
    def squared_speed_m2ps2(self) -> float:
        return self.u_x_mps**2 + self.u_y_mps**2

    def as_tuple(self) -> tuple[float, float]:
        return self.u_x_mps, self.u_y_mps


@dataclass(frozen=True, slots=True)
class ClosedBox2D:
    """Closed axis-aligned rectangle."""

    name: str
    x_min: float
    x_max: float
    y_min: float
    y_max: float

    def __post_init__(self) -> None:
        for field_name in ("x_min", "x_max", "y_min", "y_max"):
            object.__setattr__(
                self,
                field_name,
                _finite(getattr(self, field_name), field_name),
            )
        if self.x_min >= self.x_max or self.y_min >= self.y_max:
            raise ValueError(f"{self.name} must have strictly ordered bounds")
        if not self.name.strip():
            raise ValueError("box name must not be empty")

    def contains(self, state: PlanarState) -> bool:
        return (
            self.x_min <= state.x_m <= self.x_max
            and self.y_min <= state.y_m <= self.y_max
        )

    def normalized_bounds(
        self,
        normalization: "AffineNormalization2D",
    ) -> tuple[tuple[float, float], tuple[float, float]]:
        low = normalization.normalize(PlanarState(self.x_min, self.y_min))
        high = normalization.normalize(PlanarState(self.x_max, self.y_max))
        return (low.x_m, high.x_m), (low.y_m, high.y_m)


@dataclass(frozen=True, slots=True)
class ClosedDisk2D:
    """Closed Euclidean disk."""

    name: str
    center_x: float
    center_y: float
    radius: float

    def __post_init__(self) -> None:
        for field_name in ("center_x", "center_y", "radius"):
            object.__setattr__(
                self,
                field_name,
                _finite(getattr(self, field_name), field_name),
            )
        if self.radius <= 0.0:
            raise ValueError(f"{self.name} radius must be positive")
        if not self.name.strip():
            raise ValueError("disk name must not be empty")

    def squared_distance(self, state: PlanarState) -> float:
        return (state.x_m - self.center_x) ** 2 + (
            state.y_m - self.center_y
        ) ** 2

    def contains(self, state: PlanarState) -> bool:
        return self.squared_distance(state) <= self.radius**2


@dataclass(frozen=True, slots=True)
class AffineNormalization2D:
    """State normalization used by PB1 and all polynomial artifacts."""

    center_x_m: float = 1.5
    scale_x_m: float = 2.5
    center_y_m: float = 0.0
    scale_y_m: float = 2.0

    def __post_init__(self) -> None:
        for field_name in ("center_x_m", "scale_x_m", "center_y_m", "scale_y_m"):
            object.__setattr__(
                self,
                field_name,
                _finite(getattr(self, field_name), field_name),
            )
        if self.scale_x_m <= 0.0 or self.scale_y_m <= 0.0:
            raise ValueError("normalization scales must be positive")

    def normalize(self, state: PlanarState) -> PlanarState:
        return PlanarState(
            (state.x_m - self.center_x_m) / self.scale_x_m,
            (state.y_m - self.center_y_m) / self.scale_y_m,
        )

    def denormalize(self, normalized: PlanarState) -> PlanarState:
        return PlanarState(
            self.center_x_m + self.scale_x_m * normalized.x_m,
            self.center_y_m + self.scale_y_m * normalized.y_m,
        )


@dataclass(frozen=True, slots=True)
class SymmetricTriangularNoise2D:
    """N1: independent symmetric triangular noise on both axes and time."""

    half_width_m: float = 0.005
    distribution_id: str = "N1_IID_SYMMETRIC_TRIANGULAR_5MM"

    def __post_init__(self) -> None:
        object.__setattr__(
            self,
            "half_width_m",
            _finite(self.half_width_m, "half_width_m"),
        )
        if self.half_width_m <= 0.0:
            raise ValueError("noise half-width must be positive")

    def moment(self, order: int) -> float:
        if isinstance(order, bool) or not isinstance(order, int) or order < 0:
            raise ValueError("moment order must be a nonnegative integer")
        if order == 0:
            return 1.0
        if order % 2:
            return 0.0
        return (
            2.0
            * self.half_width_m**order
            / ((order + 1) * (order + 2))
        )

    def mixed_moment(self, x_order: int, y_order: int) -> float:
        return self.moment(x_order) * self.moment(y_order)


@dataclass(frozen=True, slots=True)
class StaticObstacleScenario:
    """Frozen P1/D1 geometry for the first mathematical slice."""

    domain: ClosedBox2D
    initial: ClosedDisk2D
    goal: ClosedDisk2D
    unsafe_components: tuple[ClosedBox2D, ...]
    normalization: AffineNormalization2D

    def __post_init__(self) -> None:
        if not self.unsafe_components:
            raise ValueError("at least one unsafe component is required")

    def is_unsafe(self, state: PlanarState) -> bool:
        return any(region.contains(state) for region in self.unsafe_components)

    def is_goal(self, state: PlanarState) -> bool:
        return self.goal.contains(state)

    def is_nonterminal_safe(self, state: PlanarState) -> bool:
        return (
            self.domain.contains(state)
            and not self.is_goal(state)
            and not self.is_unsafe(state)
        )

    def terminal_reason(self, state: PlanarState) -> str | None:
        if not self.domain.contains(state):
            return "model_contract_breach"
        if self.is_unsafe(state):
            return "unsafe_entry"
        if self.is_goal(state):
            return "goal_reached"
        return None


@dataclass(frozen=True, slots=True)
class StochasticSingleIntegrator2D:
    """M1/S1 state transition without action clipping or state projection."""

    dt_s: float = 0.05
    max_speed_mps: float = 0.5
    noise: SymmetricTriangularNoise2D = SymmetricTriangularNoise2D()

    def __post_init__(self) -> None:
        object.__setattr__(self, "dt_s", _finite(self.dt_s, "dt_s"))
        object.__setattr__(
            self,
            "max_speed_mps",
            _finite(self.max_speed_mps, "max_speed_mps"),
        )
        if self.dt_s <= 0.0 or self.max_speed_mps <= 0.0:
            raise ValueError("dt and max speed must be positive")

    def action_is_admissible(
        self,
        action: PlanarAction,
        *,
        tolerance: float = 0.0,
    ) -> bool:
        tolerance_value = _finite(tolerance, "tolerance")
        if tolerance_value < 0.0:
            raise ValueError("tolerance must be nonnegative")
        return (
            action.squared_speed_m2ps2
            <= self.max_speed_mps**2 + tolerance_value
        )

    def deterministic_successor(
        self,
        state: PlanarState,
        action: PlanarAction,
    ) -> PlanarState:
        return PlanarState(
            state.x_m + self.dt_s * action.u_x_mps,
            state.y_m + self.dt_s * action.u_y_mps,
        )

    def successor(
        self,
        state: PlanarState,
        action: PlanarAction,
        noise_m: Iterable[float],
    ) -> PlanarState:
        values = tuple(float(value) for value in noise_m)
        if len(values) != 2 or not all(isfinite(value) for value in values):
            raise ValueError("noise_m must contain two finite coordinates")
        if any(abs(value) > self.noise.half_width_m for value in values):
            raise ValueError("noise sample lies outside the declared N1 support")
        deterministic = self.deterministic_successor(state, action)
        return PlanarState(
            deterministic.x_m + values[0],
            deterministic.y_m + values[1],
        )


def canonical_static_obstacle_scenario() -> StaticObstacleScenario:
    """Return the exact P1/D1 scenario; callers cannot mutate it."""

    return StaticObstacleScenario(
        domain=ClosedBox2D("X", -1.0, 4.0, -2.0, 2.0),
        initial=ClosedDisk2D("X0", 0.0, 0.0, 0.10),
        goal=ClosedDisk2D("Xg", 3.5, 0.0, 0.25),
        unsafe_components=(
            ClosedBox2D("Xu1", 0.965, 2.035, -0.535, 0.535),
            ClosedBox2D("Xu2", 1.915, 3.085, 0.215, 1.385),
            ClosedBox2D("Xu3", 1.915, 3.085, -1.385, -0.215),
        ),
        normalization=AffineNormalization2D(),
    )


CANONICAL_SYSTEM = StochasticSingleIntegrator2D()
CANONICAL_SCENARIO = canonical_static_obstacle_scenario()
