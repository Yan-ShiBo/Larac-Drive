"""Independent exact-float Bernstein branch-and-bound validation.

The Bernstein conversion is performed with :class:`fractions.Fraction` created
from the exact binary64 inputs.  Bounds are then rounded outwards, so replay
does not inherit a solver's sampled points, SOS multipliers, or feasibility
status.
"""

from __future__ import annotations

from dataclasses import dataclass
from fractions import Fraction
from math import comb, nextafter
from typing import Any, Callable, Literal, Mapping

from .polynomial import NumericPolynomial, Power2D


Box = tuple[float, float, float, float]
DomainClass = Literal["outside", "boundary", "inside"]
DomainClassifier = Callable[[Box], DomainClass]


def _fraction(value: float) -> Fraction:
    return Fraction.from_float(float(value))


def _outward_lower(value: Fraction) -> float:
    converted = float(value)
    if Fraction.from_float(converted) > value:
        converted = nextafter(converted, float("-inf"))
    return nextafter(converted, float("-inf"))


def _outward_upper(value: Fraction) -> float:
    converted = float(value)
    if Fraction.from_float(converted) < value:
        converted = nextafter(converted, float("inf"))
    return nextafter(converted, float("inf"))


def exact_bernstein_bounds(
    polynomial: Mapping[Power2D, float],
    box: Box,
) -> tuple[float, float]:
    """Return an enclosing range over one closed axis-aligned box."""

    x_low, x_high, y_low, y_high = box
    if not x_low < x_high or not y_low < y_high:
        raise ValueError("Bernstein box must have positive width")
    max_x = max((power[0] for power in polynomial), default=0)
    max_y = max((power[1] for power in polynomial), default=0)
    x0 = _fraction(x_low)
    hx = _fraction(x_high) - x0
    y0 = _fraction(y_low)
    hy = _fraction(y_high) - y0
    local: dict[Power2D, Fraction] = {}
    for (x_power, y_power), raw_coefficient in polynomial.items():
        coefficient = _fraction(raw_coefficient)
        for local_x in range(x_power + 1):
            x_factor = (
                comb(x_power, local_x)
                * x0 ** (x_power - local_x)
                * hx**local_x
            )
            for local_y in range(y_power + 1):
                factor = (
                    coefficient
                    * x_factor
                    * comb(y_power, local_y)
                    * y0 ** (y_power - local_y)
                    * hy**local_y
                )
                power = (local_x, local_y)
                local[power] = local.get(power, Fraction(0)) + factor
    coefficients: list[Fraction] = []
    for bernstein_x in range(max_x + 1):
        for bernstein_y in range(max_y + 1):
            value = Fraction(0)
            for local_x in range(bernstein_x + 1):
                x_ratio = Fraction(
                    comb(bernstein_x, local_x),
                    comb(max_x, local_x),
                )
                for local_y in range(bernstein_y + 1):
                    value += (
                        local.get((local_x, local_y), Fraction(0))
                        * x_ratio
                        * Fraction(
                            comb(bernstein_y, local_y),
                            comb(max_y, local_y),
                        )
                    )
            coefficients.append(value)
    return (
        _outward_lower(min(coefficients)),
        _outward_upper(max(coefficients)),
    )


@dataclass(frozen=True, slots=True)
class BernsteinValidationOptions:
    required_margin: float = 0.0
    acceptance_tolerance: float = 0.0
    max_depth: int = 18
    minimum_width: float = 2.0**-18
    max_leaves: int = 200_000


def validate_polynomial_nonnegative(
    polynomial: NumericPolynomial,
    root_box: Box,
    *,
    classify_domain: DomainClassifier,
    options: BernsteinValidationOptions,
) -> dict[str, Any]:
    """Fail-closed global validation with an auditable leaf tree."""

    if options.max_depth < 0 or options.minimum_width <= 0.0:
        raise ValueError("invalid branch-and-bound options")
    threshold = options.required_margin - options.acceptance_tolerance
    stack: list[tuple[Box, int]] = [(root_box, 0)]
    leaves: list[dict[str, Any]] = []
    global_lower = float("inf")
    global_upper = float("-inf")
    status = "validated"
    failure_reason: str | None = None
    while stack:
        box, depth = stack.pop()
        domain_class = classify_domain(box)
        if domain_class == "outside":
            leaves.append(
                {
                    "box": list(box),
                    "depth": depth,
                    "domain_class": domain_class,
                    "disposition": "pruned_outside_domain",
                }
            )
            continue
        lower, upper = exact_bernstein_bounds(polynomial, box)
        global_lower = min(global_lower, lower)
        global_upper = max(global_upper, upper)
        if lower >= threshold:
            leaves.append(
                {
                    "box": list(box),
                    "depth": depth,
                    "domain_class": domain_class,
                    "lower": lower,
                    "upper": upper,
                    "disposition": "accepted_by_lower_bound",
                }
            )
            continue
        if domain_class == "inside" and upper < threshold:
            status = "violated"
            failure_reason = "strict_negative_upper_bound_on_active_box"
            leaves.append(
                {
                    "box": list(box),
                    "depth": depth,
                    "domain_class": domain_class,
                    "lower": lower,
                    "upper": upper,
                    "disposition": "proved_violation",
                }
            )
            break
        x_low, x_high, y_low, y_high = box
        width_x = x_high - x_low
        width_y = y_high - y_low
        if (
            depth >= options.max_depth
            or max(width_x, width_y) <= options.minimum_width
        ):
            status = "unknown"
            failure_reason = "subdivision_budget_exhausted"
            leaves.append(
                {
                    "box": list(box),
                    "depth": depth,
                    "domain_class": domain_class,
                    "lower": lower,
                    "upper": upper,
                    "disposition": "unresolved",
                }
            )
            break
        if len(leaves) + len(stack) + 2 > options.max_leaves:
            status = "unknown"
            failure_reason = "leaf_budget_exhausted"
            leaves.append(
                {
                    "box": list(box),
                    "depth": depth,
                    "domain_class": domain_class,
                    "lower": lower,
                    "upper": upper,
                    "disposition": "unresolved",
                }
            )
            break
        if width_x >= width_y:
            midpoint = x_low + 0.5 * width_x
            children = (
                (x_low, midpoint, y_low, y_high),
                (midpoint, x_high, y_low, y_high),
            )
        else:
            midpoint = y_low + 0.5 * width_y
            children = (
                (x_low, x_high, y_low, midpoint),
                (x_low, x_high, midpoint, y_high),
            )
        stack.append((children[1], depth + 1))
        stack.append((children[0], depth + 1))
    if not leaves:
        status = "unknown"
        failure_reason = "empty_active_domain"
    return {
        "status": status,
        "failure_reason": failure_reason,
        "required_margin": options.required_margin,
        "acceptance_tolerance": options.acceptance_tolerance,
        "root_box": list(root_box),
        "global_visited_lower_bound": (
            None if global_lower == float("inf") else global_lower
        ),
        "global_visited_upper_bound": (
            None if global_upper == float("-inf") else global_upper
        ),
        "leaf_count": len(leaves),
        "leaves": leaves,
    }


def classify_full_box(_: Box) -> DomainClass:
    return "inside"
