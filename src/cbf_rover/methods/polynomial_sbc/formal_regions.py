"""Deterministic semialgebraic decomposition shared by synthesis and replay."""

from __future__ import annotations

from typing import Any, Iterable

from cbf_rover.domain.planar import CANONICAL_SCENARIO, ClosedBox2D, PlanarState

from .algebra import (
    X_VARIABLE,
    Y_VARIABLE,
    add,
    constant,
    multiply,
    power,
    scale,
    subtract,
)
from .polynomial import NumericPolynomial


PHYSICAL_X_BREAKS = (-1.0, 0.965, 1.915, 2.035, 3.085, 4.0)
PHYSICAL_Y_BREAKS = (-2.0, -1.385, -0.535, -0.215, 0.215, 0.535, 1.385, 2.0)


def normalized_interval(
    low: float,
    high: float,
    *,
    center: float,
    scale_value: float,
) -> tuple[float, float]:
    return (low - center) / scale_value, (high - center) / scale_value


def normalized_box(region: ClosedBox2D) -> tuple[float, float, float, float]:
    normalization = CANONICAL_SCENARIO.normalization
    x_low, x_high = normalized_interval(
        region.x_min,
        region.x_max,
        center=normalization.center_x_m,
        scale_value=normalization.scale_x_m,
    )
    y_low, y_high = normalized_interval(
        region.y_min,
        region.y_max,
        center=normalization.center_y_m,
        scale_value=normalization.scale_y_m,
    )
    return x_low, x_high, y_low, y_high


def safe_drift_cells(
    additional_x_breaks: Iterable[float] = (),
) -> tuple[ClosedBox2D, ...]:
    """Exact grid cover of ``X minus Xu``; goal exclusion stays a guard."""

    scenario = CANONICAL_SCENARIO
    x_breaks = tuple(
        sorted(
            {
                *PHYSICAL_X_BREAKS,
                *(
                    float(value)
                    for value in additional_x_breaks
                    if scenario.domain.x_min
                    < float(value)
                    < scenario.domain.x_max
                ),
            }
        )
    )
    cells: list[ClosedBox2D] = []
    for x_index, (x_low, x_high) in enumerate(
        zip(x_breaks[:-1], x_breaks[1:])
    ):
        for y_index, (y_low, y_high) in enumerate(
            zip(PHYSICAL_Y_BREAKS[:-1], PHYSICAL_Y_BREAKS[1:])
        ):
            center_state = PlanarState(
                0.5 * (x_low + x_high),
                0.5 * (y_low + y_high),
            )
            if scenario.is_unsafe(center_state):
                continue
            cells.append(
                ClosedBox2D(
                    f"D_{x_index}_{y_index}",
                    x_low,
                    x_high,
                    y_low,
                    y_high,
                )
            )
    return tuple(cells)


def rectangle_guards(
    box: tuple[float, float, float, float],
) -> tuple[NumericPolynomial, NumericPolynomial]:
    x_low, x_high, y_low, y_high = box
    x_guard = multiply(
        subtract(X_VARIABLE, constant(x_low)),
        subtract(constant(x_high), X_VARIABLE),
    )
    y_guard = multiply(
        subtract(Y_VARIABLE, constant(y_low)),
        subtract(constant(y_high), Y_VARIABLE),
    )
    return x_guard, y_guard


def physical_squared_distance_polynomial(
    center_x_m: float,
    center_y_m: float,
) -> NumericPolynomial:
    normalization = CANONICAL_SCENARIO.normalization
    physical_dx = add(
        constant(normalization.center_x_m - center_x_m),
        scale(X_VARIABLE, normalization.scale_x_m),
    )
    physical_dy = add(
        constant(normalization.center_y_m - center_y_m),
        scale(Y_VARIABLE, normalization.scale_y_m),
    )
    return add(power(physical_dx, 2), power(physical_dy, 2))


def initial_disk_guard() -> NumericPolynomial:
    initial = CANONICAL_SCENARIO.initial
    return subtract(
        constant(initial.radius**2),
        physical_squared_distance_polynomial(
            initial.center_x,
            initial.center_y,
        ),
    )


def goal_exterior_guard() -> NumericPolynomial:
    goal = CANONICAL_SCENARIO.goal
    return subtract(
        physical_squared_distance_polynomial(goal.center_x, goal.center_y),
        constant(goal.radius**2),
    )


def describe_cells() -> list[dict[str, Any]]:
    return [
        {
            "name": cell.name,
            "physical_box": [
                cell.x_min,
                cell.x_max,
                cell.y_min,
                cell.y_max,
            ],
            "normalized_box": list(normalized_box(cell)),
        }
        for cell in safe_drift_cells()
    ]
