"""Pure geometry for the seeded dynamic-road simulation."""

from __future__ import annotations

from dataclasses import dataclass
from math import hypot, isfinite

from .planar import PlanarState


def _finite(value: float, name: str) -> float:
    result = float(value)
    if not isfinite(result):
        raise ValueError(f"{name} must be finite")
    return result


@dataclass(frozen=True, slots=True)
class RoadGeometry:
    """Two-lane road geometry expressed in the world frame."""

    x_min_m: float
    x_max_m: float
    physical_y_min_m: float
    physical_y_max_m: float
    lane_centers_y_m: tuple[float, float]
    start: PlanarState
    finish_line_x_m: float
    rover_radius_m: float

    def __post_init__(self) -> None:
        for field_name in (
            "x_min_m",
            "x_max_m",
            "physical_y_min_m",
            "physical_y_max_m",
            "finish_line_x_m",
            "rover_radius_m",
        ):
            object.__setattr__(
                self,
                field_name,
                _finite(getattr(self, field_name), field_name),
            )
        lanes = tuple(
            _finite(value, "lane_center_y_m")
            for value in self.lane_centers_y_m
        )
        object.__setattr__(self, "lane_centers_y_m", lanes)
        if self.x_min_m >= self.x_max_m:
            raise ValueError("road x bounds must be strictly ordered")
        if self.physical_y_min_m >= self.physical_y_max_m:
            raise ValueError("road y bounds must be strictly ordered")
        if len(lanes) != 2 or lanes[0] >= lanes[1]:
            raise ValueError("exactly two ordered lane centers are required")
        if self.rover_radius_m <= 0.0:
            raise ValueError("rover radius must be positive")
        if not self.x_min_m <= self.start.x_m < self.finish_line_x_m:
            raise ValueError("road start x must precede the finish line")
        if not self.finish_line_x_m < self.x_max_m:
            raise ValueError("finish line must lie strictly inside the road")
        if not self.center_y_min_m < self.center_y_max_m:
            raise ValueError("rover does not fit between the physical edges")
        if not self.center_contains(self.start):
            raise ValueError("road start must lie in the center-safe strip")
        if any(
            not self.center_y_min_m < lane < self.center_y_max_m
            for lane in lanes
        ):
            raise ValueError("lane centers must lie in the center-safe strip")

    @property
    def center_y_min_m(self) -> float:
        return self.physical_y_min_m + self.rover_radius_m

    @property
    def center_y_max_m(self) -> float:
        return self.physical_y_max_m - self.rover_radius_m

    def center_contains(self, state: PlanarState) -> bool:
        return (
            self.x_min_m <= state.x_m <= self.x_max_m
            and self.center_y_min_m <= state.y_m <= self.center_y_max_m
        )

    def road_departure(self, state: PlanarState) -> bool:
        return not (
            self.center_y_min_m <= state.y_m <= self.center_y_max_m
        )

    def goal_reached(self, state: PlanarState) -> bool:
        return state.x_m >= self.finish_line_x_m


@dataclass(frozen=True, slots=True)
class RoadObstacle:
    """One circular obstacle and its conservative certificate radius."""

    obstacle_id: str
    center: PlanarState
    lane_index: int
    physical_radius_m: float
    safety_margin_m: float
    rover_radius_m: float
    appearance_distance_m: float

    def __post_init__(self) -> None:
        if not self.obstacle_id.strip():
            raise ValueError("obstacle_id must not be empty")
        if self.lane_index not in (0, 1):
            raise ValueError("lane_index must be zero or one")
        for field_name in (
            "physical_radius_m",
            "safety_margin_m",
            "rover_radius_m",
            "appearance_distance_m",
        ):
            object.__setattr__(
                self,
                field_name,
                _finite(getattr(self, field_name), field_name),
            )
        if self.physical_radius_m <= 0.0:
            raise ValueError("physical obstacle radius must be positive")
        if self.safety_margin_m < 0.0:
            raise ValueError("certificate safety margin must be nonnegative")
        if self.rover_radius_m <= 0.0:
            raise ValueError("rover radius must be positive")
        if self.appearance_distance_m <= 0.0:
            raise ValueError("appearance distance must be positive")

    @property
    def collision_radius_m(self) -> float:
        return self.physical_radius_m + self.rover_radius_m

    @property
    def certificate_radius_m(self) -> float:
        return self.collision_radius_m + self.safety_margin_m

    @property
    def activation_x_m(self) -> float:
        return self.center.x_m - self.appearance_distance_m

    def is_visible(self, state: PlanarState) -> bool:
        return state.x_m >= self.activation_x_m

    def center_distance_m(self, state: PlanarState) -> float:
        return hypot(
            state.x_m - self.center.x_m,
            state.y_m - self.center.y_m,
        )

    def physical_clearance_m(self, state: PlanarState) -> float:
        return self.center_distance_m(state) - self.collision_radius_m

    def certificate_barrier_m(self, state: PlanarState) -> float:
        return self.center_distance_m(state) - self.certificate_radius_m


@dataclass(frozen=True, slots=True)
class DynamicRoadScenario:
    """Frozen geometry plus one seeded obstacle plan."""

    scenario_id: str
    geometry: RoadGeometry
    obstacles: tuple[RoadObstacle, ...]

    def __post_init__(self) -> None:
        if not self.scenario_id.strip():
            raise ValueError("scenario_id must not be empty")
        if not self.obstacles:
            raise ValueError("at least one road obstacle is required")
        identities = [obstacle.obstacle_id for obstacle in self.obstacles]
        if len(set(identities)) != len(identities):
            raise ValueError("obstacle identities must be unique")
        x_values = [obstacle.center.x_m for obstacle in self.obstacles]
        if x_values != sorted(x_values):
            raise ValueError("road obstacles must be ordered by x")
        for obstacle in self.obstacles:
            lane_y = self.geometry.lane_centers_y_m[obstacle.lane_index]
            if abs(obstacle.center.y_m - lane_y) > 1.0e-12:
                raise ValueError("obstacle center must match its lane center")

    def active_obstacles(
        self,
        state: PlanarState,
    ) -> tuple[RoadObstacle, ...]:
        return tuple(
            obstacle
            for obstacle in self.obstacles
            if obstacle.is_visible(state)
        )

    def collision_ids(self, state: PlanarState) -> tuple[str, ...]:
        return tuple(
            obstacle.obstacle_id
            for obstacle in self.obstacles
            if obstacle.physical_clearance_m(state) < 0.0
        )
