"""Exact deterministic projection for a two-dimensional CBF action set."""

from __future__ import annotations

from dataclasses import dataclass
from math import hypot, isfinite, sqrt
from typing import Iterable, Sequence

from cbf_rover.domain.planar import PlanarAction


@dataclass(frozen=True, slots=True)
class LinearHalfspace2D:
    """Closed halfspace ``a_x*u_x + a_y*u_y >= b``."""

    name: str
    a_x: float
    a_y: float
    b: float

    def __post_init__(self) -> None:
        if not self.name.strip():
            raise ValueError("constraint name must not be empty")
        values = (self.a_x, self.a_y, self.b)
        if not all(isfinite(float(value)) for value in values):
            raise ValueError("constraint coefficients must be finite")
        if hypot(self.a_x, self.a_y) <= 0.0:
            raise ValueError("constraint normal must be nonzero")

    @property
    def squared_normal(self) -> float:
        return self.a_x**2 + self.a_y**2

    def residual(self, action: PlanarAction) -> float:
        return (
            self.a_x * action.u_x_mps
            + self.a_y * action.u_y_mps
            - self.b
        )


@dataclass(frozen=True, slots=True)
class ProjectionResult:
    """The unique minimum-distance feasible action and audit facts."""

    action: PlanarAction
    objective_m2ps2: float
    selected_source: str
    constraint_residuals_mps: tuple[tuple[str, float], ...]
    active_constraint_names: tuple[str, ...]
    speed_margin_mps: float


class InfeasibleProjectionError(RuntimeError):
    """Raised when the declared halfspaces and speed disk do not intersect."""


def _candidate_is_feasible(
    action: PlanarAction,
    constraints: Sequence[LinearHalfspace2D],
    *,
    max_speed_mps: float,
    tolerance: float,
) -> bool:
    if action.squared_speed_m2ps2 > max_speed_mps**2 + tolerance:
        return False
    return all(
        constraint.residual(action) >= -tolerance
        for constraint in constraints
    )


def _line_circle_intersections(
    constraint: LinearHalfspace2D,
    radius: float,
    tolerance: float,
) -> tuple[PlanarAction, ...]:
    normal_squared = constraint.squared_normal
    base_x = constraint.b * constraint.a_x / normal_squared
    base_y = constraint.b * constraint.a_y / normal_squared
    base_squared = base_x**2 + base_y**2
    radius_squared = radius**2
    if base_squared > radius_squared + tolerance:
        return ()
    tangent_distance = sqrt(max(0.0, radius_squared - base_squared))
    normal_length = sqrt(normal_squared)
    tangent_x = -constraint.a_y / normal_length
    tangent_y = constraint.a_x / normal_length
    if tangent_distance <= tolerance:
        return (PlanarAction(base_x, base_y),)
    return (
        PlanarAction(
            base_x + tangent_distance * tangent_x,
            base_y + tangent_distance * tangent_y,
        ),
        PlanarAction(
            base_x - tangent_distance * tangent_x,
            base_y - tangent_distance * tangent_y,
        ),
    )


def project_action_to_halfspaces_and_disk(
    nominal_action: PlanarAction,
    constraints: Iterable[LinearHalfspace2D],
    *,
    max_speed_mps: float,
    tolerance: float = 1.0e-10,
) -> ProjectionResult:
    """Project onto ``{u: A u >= b, ||u||_2 <= max_speed}``.

    In two dimensions an optimum is the nominal point or lies on one or two
    active boundaries. Enumerating line projections, line intersections, the
    radial speed-circle projection, and line-circle intersections therefore
    covers the full convex feasible set without an iterative solver.
    """

    radius = float(max_speed_mps)
    tolerance_value = float(tolerance)
    if not isfinite(radius) or radius <= 0.0:
        raise ValueError("max_speed_mps must be finite and positive")
    if not isfinite(tolerance_value) or tolerance_value < 0.0:
        raise ValueError("tolerance must be finite and nonnegative")
    ordered = tuple(constraints)
    names = [constraint.name for constraint in ordered]
    if len(set(names)) != len(names):
        raise ValueError("constraint names must be unique")

    candidates: list[tuple[str, PlanarAction]] = []

    def add(source: str, x_value: float, y_value: float) -> None:
        if not isfinite(x_value) or not isfinite(y_value):
            return
        action = PlanarAction(x_value, y_value)
        if _candidate_is_feasible(
            action,
            ordered,
            max_speed_mps=radius,
            tolerance=tolerance_value,
        ):
            candidates.append((source, action))

    add(
        "nominal",
        nominal_action.u_x_mps,
        nominal_action.u_y_mps,
    )
    nominal_speed = hypot(
        nominal_action.u_x_mps,
        nominal_action.u_y_mps,
    )
    if nominal_speed > 0.0:
        add(
            "speed_circle_radial",
            radius * nominal_action.u_x_mps / nominal_speed,
            radius * nominal_action.u_y_mps / nominal_speed,
        )

    for constraint in ordered:
        signed_gap = (
            constraint.b
            - constraint.a_x * nominal_action.u_x_mps
            - constraint.a_y * nominal_action.u_y_mps
        )
        multiplier = signed_gap / constraint.squared_normal
        add(
            f"line:{constraint.name}",
            nominal_action.u_x_mps + multiplier * constraint.a_x,
            nominal_action.u_y_mps + multiplier * constraint.a_y,
        )
        for intersection_index, intersection in enumerate(
            _line_circle_intersections(
                constraint,
                radius,
                tolerance_value,
            )
        ):
            add(
                f"line_circle:{constraint.name}:{intersection_index}",
                intersection.u_x_mps,
                intersection.u_y_mps,
            )

    for first_index, first in enumerate(ordered):
        for second in ordered[first_index + 1 :]:
            determinant = first.a_x * second.a_y - first.a_y * second.a_x
            scale = sqrt(first.squared_normal * second.squared_normal)
            if abs(determinant) <= tolerance_value * max(1.0, scale):
                continue
            x_value = (
                first.b * second.a_y - first.a_y * second.b
            ) / determinant
            y_value = (
                first.a_x * second.b - first.b * second.a_x
            ) / determinant
            add(
                f"pair:{first.name}:{second.name}",
                x_value,
                y_value,
            )

    if not candidates:
        raise InfeasibleProjectionError(
            "online CBF halfspaces do not intersect the action disk"
        )

    def ranking(
        candidate: tuple[str, PlanarAction],
    ) -> tuple[float, str, float, float]:
        source, action = candidate
        objective = (
            action.u_x_mps - nominal_action.u_x_mps
        ) ** 2 + (
            action.u_y_mps - nominal_action.u_y_mps
        ) ** 2
        return objective, source, action.u_x_mps, action.u_y_mps

    source, selected = min(candidates, key=ranking)
    objective = ranking((source, selected))[0]
    residuals = tuple(
        (constraint.name, constraint.residual(selected))
        for constraint in ordered
    )
    active_tolerance = max(1.0e-8, 10.0 * tolerance_value)
    active = tuple(
        name for name, residual in residuals if abs(residual) <= active_tolerance
    )
    return ProjectionResult(
        action=selected,
        objective_m2ps2=objective,
        selected_source=source,
        constraint_residuals_mps=residuals,
        active_constraint_names=active,
        speed_margin_mps=radius
        - hypot(selected.u_x_mps, selected.u_y_mps),
    )
