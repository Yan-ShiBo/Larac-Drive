"""Independent exact-rational V3P replay for the G11P49 proof route.

The validator consumes only the final binary64 barrier/roof artifacts and the
frozen controller/model contracts.  It never reads the LP matrix, solver
status, objective, numeric slacks, or shape multipliers.
"""

from __future__ import annotations

from dataclasses import dataclass, replace
from fractions import Fraction
import hashlib
import json
from math import comb, isfinite
from pathlib import Path
from typing import Any, Mapping, Sequence

from .piecewise_barrier import (
    G11P49_PROOF_ROUTE_ID,
    G11P49_REACH_CATALOG_ID,
    G11P49_VALIDATOR_ID,
    P49_PARTITION_ID,
    LocalBernsteinRoof2D,
    PiecewiseBernsteinBarrier2D,
    PiecewiseRoofWitness2D,
)
from .polynomial import (
    FrozenController,
    Power2D,
    content_sha256,
    controller_pieces,
)
from .rational_certificate_validation import (
    CONTROLLER_REQUIRED_MARGIN,
    DOMAIN_PHYSICAL,
    DT,
    FIRST_SLICE_Q_MAX,
    GOAL_DISK,
    INITIAL_DISK,
    MAX_SPEED,
    NOISE_HALF_WIDTH,
    NORMALIZATION_CENTER_X,
    NORMALIZATION_CENTER_Y,
    NORMALIZATION_SCALE_X,
    NORMALIZATION_SCALE_Y,
    RationalBernsteinValidationOptions,
    UNSAFE_PHYSICAL,
    _action_disk_residual,
    _assert_canonical_runtime_contract,
    _piece_normalized_box,
    _robust_domain_residuals,
    canonical_rational_model_payload,
    classify_full_box,
    classify_goal_exterior_box,
    classify_included_disk_box,
    exact_geometry_gate,
    first_slice_q_gate,
    normalize_physical_box,
    rational_sparse_polynomial,
    validate_rational_polynomial_nonnegative,
)


G11P49_V3P_GEOMETRY_CATALOG_SHA256 = (
    "ef545affb1c35d965be5d4b9d9e5ab39854e4d8069c296b924d45e049a88924a"
)
_V3P_PHYSICAL_X_BREAKS = tuple(
    Fraction(value)
    for value in (
        "-1",
        "0.8",
        "0.965",
        "1.915",
        "2.035",
        "3.085",
        "3.2",
        "4",
    )
)
_V3P_PHYSICAL_Y_BREAKS = tuple(
    Fraction(value)
    for value in (
        "-2",
        "-1.385",
        "-0.535",
        "-0.215",
        "0.215",
        "0.535",
        "1.385",
        "2",
    )
)
_V3P_EXPECTED_NORMALIZED_X_BREAKS = (
    Fraction(-1),
    Fraction(-7, 25),
    Fraction(-107, 500),
    Fraction(83, 500),
    Fraction(107, 500),
    Fraction(317, 500),
    Fraction(17, 25),
    Fraction(1),
)
_V3P_EXPECTED_NORMALIZED_Y_BREAKS = (
    Fraction(-1),
    Fraction(-277, 400),
    Fraction(-107, 400),
    Fraction(-43, 400),
    Fraction(43, 400),
    Fraction(107, 400),
    Fraction(277, 400),
    Fraction(1),
)
_V3P_QUADRANTS = ((-1, -1), (-1, 1), (1, -1), (1, 1))

_EXPECTED_P49_COUNTS = {
    "cell_count": 49,
    "unsafe_cell_count": 12,
    "safe_source_cell_count": 37,
    "goal_boundary_safe_cell_count": 3,
    "roof_count": 148,
    "target_intersection_count": 1012,
    "internal_conforming_edge_count": 84,
}
_EXPECTED_UNSAFE_CELL_BINDINGS = {
    "C_2_2": ("Xu1",),
    "C_2_3": ("Xu1",),
    "C_2_4": ("Xu1",),
    "C_3_1": ("Xu3",),
    "C_3_2": ("Xu1", "Xu3"),
    "C_3_3": ("Xu1",),
    "C_3_4": ("Xu1", "Xu2"),
    "C_3_5": ("Xu2",),
    "C_4_1": ("Xu3",),
    "C_4_2": ("Xu3",),
    "C_4_4": ("Xu2",),
    "C_4_5": ("Xu2",),
}
RationalPolynomial = dict[Power2D, Fraction]
RationalBox = tuple[Fraction, Fraction, Fraction, Fraction]


def _fraction_text(value: Fraction) -> str:
    if value.denominator == 1:
        return str(value.numerator)
    return f"{value.numerator}/{value.denominator}"


def _fraction_from_artifact(value: float) -> Fraction:
    return Fraction.from_float(float(value))


def _clean(polynomial: Mapping[Power2D, Fraction]) -> RationalPolynomial:
    return {power: value for power, value in polynomial.items() if value}


def _constant(value: Fraction | int) -> RationalPolynomial:
    exact = Fraction(value)
    return {} if exact == 0 else {(0, 0): exact}


def _add(*polynomials: Mapping[Power2D, Fraction]) -> RationalPolynomial:
    result: RationalPolynomial = {}
    for polynomial in polynomials:
        for power, coefficient in polynomial.items():
            result[power] = result.get(power, Fraction(0)) + coefficient
    return _clean(result)


def _negate(polynomial: Mapping[Power2D, Fraction]) -> RationalPolynomial:
    return {power: -value for power, value in polynomial.items() if value}


def _scale(
    polynomial: Mapping[Power2D, Fraction],
    scalar: Fraction | int,
) -> RationalPolynomial:
    exact = Fraction(scalar)
    return _clean({power: exact * value for power, value in polynomial.items()})


def _multiply(
    left: Mapping[Power2D, Fraction],
    right: Mapping[Power2D, Fraction],
) -> RationalPolynomial:
    result: RationalPolynomial = {}
    for (left_x, left_y), left_value in left.items():
        for (right_x, right_y), right_value in right.items():
            power = (left_x + right_x, left_y + right_y)
            result[power] = result.get(power, Fraction(0)) + (
                left_value * right_value
            )
    return _clean(result)


def _power(
    polynomial: Mapping[Power2D, Fraction],
    exponent: int,
) -> RationalPolynomial:
    if exponent < 0:
        raise ValueError("V3P polynomial exponent must be nonnegative")
    result = _constant(1)
    base = dict(polynomial)
    value = int(exponent)
    while value:
        if value & 1:
            result = _multiply(result, base)
        value //= 2
        if value:
            base = _multiply(base, base)
    return result


def _box_payload(box: Sequence[Fraction]) -> list[str]:
    return [_fraction_text(value) for value in box]


@dataclass(frozen=True, slots=True)
class _V3PCell:
    x_index: int
    y_index: int
    physical_box: RationalBox
    normalized_box: RationalBox
    unsafe_component_ids: tuple[str, ...]
    goal_boundary: bool
    controller_piece_id: str

    @property
    def cell_id(self) -> str:
        return f"C_{self.x_index}_{self.y_index}"

    @property
    def unsafe(self) -> bool:
        return bool(self.unsafe_component_ids)

    @property
    def safe_source(self) -> bool:
        return not self.unsafe

    def payload(self) -> dict[str, Any]:
        return {
            "cell_id": self.cell_id,
            "grid_index": [self.x_index, self.y_index],
            "physical_box_exact": _box_payload(self.physical_box),
            "normalized_box_exact": _box_payload(self.normalized_box),
            "unsafe_component_ids": list(self.unsafe_component_ids),
            "safe_source": self.safe_source,
            "goal_boundary": self.goal_boundary,
            "controller_piece_id": self.controller_piece_id,
        }


def _v3p_normalized_breaks() -> tuple[
    tuple[Fraction, ...], tuple[Fraction, ...]
]:
    x_breaks = tuple(
        (value - NORMALIZATION_CENTER_X) / NORMALIZATION_SCALE_X
        for value in _V3P_PHYSICAL_X_BREAKS
    )
    y_breaks = tuple(
        (value - NORMALIZATION_CENTER_Y) / NORMALIZATION_SCALE_Y
        for value in _V3P_PHYSICAL_Y_BREAKS
    )
    if x_breaks != _V3P_EXPECTED_NORMALIZED_X_BREAKS:
        raise ValueError("V3P independently derived x normalization drifted")
    if y_breaks != _V3P_EXPECTED_NORMALIZED_Y_BREAKS:
        raise ValueError("V3P independently derived y normalization drifted")
    if (x_breaks[0], x_breaks[-1], y_breaks[0], y_breaks[-1]) != (
        Fraction(-1),
        Fraction(1),
        Fraction(-1),
        Fraction(1),
    ):
        raise ValueError("V3P normalized P49 domain is not D1")
    return x_breaks, y_breaks


def _v3p_reach_padding() -> tuple[Fraction, Fraction, Fraction, Fraction]:
    padding = (
        DT * MAX_SPEED / NORMALIZATION_SCALE_X,
        DT * MAX_SPEED / NORMALIZATION_SCALE_Y,
        NOISE_HALF_WIDTH / NORMALIZATION_SCALE_X,
        NOISE_HALF_WIDTH / NORMALIZATION_SCALE_Y,
    )
    if padding != (
        Fraction(1, 100),
        Fraction(1, 80),
        Fraction(1, 500),
        Fraction(1, 400),
    ):
        raise ValueError("V3P independently derived action/noise padding drifted")
    return padding


def _positive_area_overlap(left: RationalBox, right: RationalBox) -> bool:
    return (
        max(left[0], right[0]) < min(left[1], right[1])
        and max(left[2], right[2]) < min(left[3], right[3])
    )


def _box_subset(left: RationalBox, right: RationalBox) -> bool:
    return (
        right[0] <= left[0] <= left[1] <= right[1]
        and right[2] <= left[2] <= left[3] <= right[3]
    )


def _v3p_controller_piece_id(physical_box: RationalBox) -> str:
    if physical_box[0] == Fraction(-1) and physical_box[1] == Fraction(4, 5):
        return "lower_entry"
    if Fraction(4, 5) <= physical_box[0] < physical_box[1] <= Fraction(16, 5):
        return "lower_transit"
    if physical_box[0] == Fraction(16, 5) and physical_box[1] == Fraction(4):
        return "goal_capture"
    raise ValueError("V3P cell crosses a frozen C2 controller switch")


def _v3p_goal_boundary(box: RationalBox) -> bool:
    x_low, x_high, y_low, y_high = box
    closest_x = min(max(GOAL_DISK.center_x, x_low), x_high)
    closest_y = min(max(GOAL_DISK.center_y, y_low), y_high)
    minimum = (closest_x - GOAL_DISK.center_x) ** 2 + (
        closest_y - GOAL_DISK.center_y
    ) ** 2
    maximum = max(
        (x_value - GOAL_DISK.center_x) ** 2
        + (y_value - GOAL_DISK.center_y) ** 2
        for x_value in (x_low, x_high)
        for y_value in (y_low, y_high)
    )
    return minimum <= GOAL_DISK.radius**2 < maximum


def _v3p_build_cells() -> tuple[_V3PCell, ...]:
    x_faces = set(_V3P_PHYSICAL_X_BREAKS)
    y_faces = set(_V3P_PHYSICAL_Y_BREAKS)
    for _, obstacle in UNSAFE_PHYSICAL:
        if obstacle[0] not in x_faces or obstacle[1] not in x_faces:
            raise ValueError("V3P obstacle x face is not P49 aligned")
        if obstacle[2] not in y_faces or obstacle[3] not in y_faces:
            raise ValueError("V3P obstacle y face is not P49 aligned")

    normalized_x, normalized_y = _v3p_normalized_breaks()
    cells: list[_V3PCell] = []
    for x_index in range(7):
        for y_index in range(7):
            physical_box: RationalBox = (
                _V3P_PHYSICAL_X_BREAKS[x_index],
                _V3P_PHYSICAL_X_BREAKS[x_index + 1],
                _V3P_PHYSICAL_Y_BREAKS[y_index],
                _V3P_PHYSICAL_Y_BREAKS[y_index + 1],
            )
            normalized_box: RationalBox = (
                normalized_x[x_index],
                normalized_x[x_index + 1],
                normalized_y[y_index],
                normalized_y[y_index + 1],
            )
            unsafe_ids: list[str] = []
            for obstacle_id, obstacle in UNSAFE_PHYSICAL:
                overlaps = _positive_area_overlap(physical_box, obstacle)
                contained = _box_subset(physical_box, obstacle)
                if overlaps != contained:
                    raise ValueError(
                        "V3P P49 cell is neither obstacle-contained nor "
                        "interior-disjoint"
                    )
                if contained:
                    unsafe_ids.append(obstacle_id)
            cells.append(
                _V3PCell(
                    x_index=x_index,
                    y_index=y_index,
                    physical_box=physical_box,
                    normalized_box=normalized_box,
                    unsafe_component_ids=tuple(unsafe_ids),
                    goal_boundary=_v3p_goal_boundary(physical_box),
                    controller_piece_id=_v3p_controller_piece_id(physical_box),
                )
            )
    if len(cells) != 49:
        raise ValueError("V3P independent P49 cell count drifted")
    return tuple(cells)


def _v3p_quadrant_id(quadrant: tuple[int, int]) -> str:
    if quadrant not in _V3P_QUADRANTS:
        raise ValueError("V3P quadrant signs must both be -1 or +1")
    return "".join("p" if sign > 0 else "m" for sign in quadrant)


def _v3p_reach_box(
    cell: _V3PCell,
    quadrant: tuple[int, int],
) -> RationalBox:
    if not cell.safe_source or quadrant not in _V3P_QUADRANTS:
        raise ValueError("invalid V3P reach-box request")
    action_x, action_y, noise_x, noise_y = _v3p_reach_padding()
    x_low, x_high, y_low, y_high = cell.normalized_box
    return (
        max(Fraction(-1), x_low - action_x - (noise_x if quadrant[0] < 0 else 0)),
        min(Fraction(1), x_high + action_x + (noise_x if quadrant[0] > 0 else 0)),
        max(Fraction(-1), y_low - action_y - (noise_y if quadrant[1] < 0 else 0)),
        min(Fraction(1), y_high + action_y + (noise_y if quadrant[1] > 0 else 0)),
    )


def _v3p_positive_intersection(
    left: RationalBox,
    right: RationalBox,
) -> RationalBox | None:
    result: RationalBox = (
        max(left[0], right[0]),
        min(left[1], right[1]),
        max(left[2], right[2]),
        min(left[3], right[3]),
    )
    if result[0] >= result[1] or result[2] >= result[3]:
        return None
    return result


def _v3p_reach_target_intersections(
    cell: _V3PCell,
    quadrant: tuple[int, int],
    cells: Sequence[_V3PCell],
) -> tuple[tuple[_V3PCell, RationalBox], ...]:
    reach_box = _v3p_reach_box(cell, quadrant)
    intersections = tuple(
        (target, intersection)
        for target in cells
        if (
            intersection := _v3p_positive_intersection(
                reach_box,
                target.normalized_box,
            )
        )
        is not None
    )
    covered_area = sum(
        (box[1] - box[0]) * (box[3] - box[2])
        for _, box in intersections
    )
    reach_area = (reach_box[1] - reach_box[0]) * (
        reach_box[3] - reach_box[2]
    )
    if covered_area != reach_area:
        raise ValueError("V3P target closures do not cover the reach box")
    return intersections


def v3p_geometry_catalog() -> dict[str, Any]:
    cells = _v3p_build_cells()
    safe = tuple(cell for cell in cells if cell.safe_source)
    unsafe = tuple(cell for cell in cells if cell.unsafe)
    goal_boundary = tuple(cell for cell in safe if cell.goal_boundary)
    obligations: list[dict[str, Any]] = []
    quadrant_counts = {quadrant: 0 for quadrant in _V3P_QUADRANTS}
    for source in safe:
        for quadrant in _V3P_QUADRANTS:
            intersections = _v3p_reach_target_intersections(
                source,
                quadrant,
                cells,
            )
            quadrant_counts[quadrant] += len(intersections)
            obligations.append(
                {
                    "source_cell_id": source.cell_id,
                    "controller_piece_id": source.controller_piece_id,
                    "quadrant": list(quadrant),
                    "quadrant_id": _v3p_quadrant_id(quadrant),
                    "normalized_reach_box_exact": _box_payload(
                        _v3p_reach_box(source, quadrant)
                    ),
                    "target_intersections": [
                        {
                            "target_cell_id": target.cell_id,
                            "normalized_box_exact": _box_payload(intersection),
                        }
                        for target, intersection in intersections
                    ],
                }
            )
    if quadrant_counts != {quadrant: 253 for quadrant in _V3P_QUADRANTS}:
        raise ValueError("V3P per-quadrant target count drifted")
    payload: dict[str, Any] = {
        "schema_version": 1,
        "kind": "g11p49_exact_geometry_catalog",
        "partition_id": P49_PARTITION_ID,
        "reach_catalog_id": G11P49_REACH_CATALOG_ID,
        "physical_x_breaks_exact": _box_payload(_V3P_PHYSICAL_X_BREAKS),
        "physical_y_breaks_exact": _box_payload(_V3P_PHYSICAL_Y_BREAKS),
        "normalized_x_breaks_exact": _box_payload(
            _V3P_EXPECTED_NORMALIZED_X_BREAKS
        ),
        "normalized_y_breaks_exact": _box_payload(
            _V3P_EXPECTED_NORMALIZED_Y_BREAKS
        ),
        "quadrant_order": [list(value) for value in _V3P_QUADRANTS],
        "reach_padding": {
            "action_shift_x_normalized": "1/100",
            "action_shift_y_normalized": "1/80",
            "noise_half_width_x_normalized": "1/500",
            "noise_half_width_y_normalized": "1/400",
            "derived_from_action_disk_not_controller_range": True,
        },
        "counts": {
            "cell_count": len(cells),
            "unsafe_cell_count": len(unsafe),
            "safe_source_cell_count": len(safe),
            "goal_boundary_safe_cell_count": len(goal_boundary),
            "roof_count": len(obligations),
            "target_intersection_count": sum(quadrant_counts.values()),
            "internal_conforming_edge_count": 84,
        },
        "cells": [cell.payload() for cell in cells],
        "reach_obligations": obligations,
    }
    payload["catalog_content_sha256"] = content_sha256(payload)
    if payload["catalog_content_sha256"] != (
        G11P49_V3P_GEOMETRY_CATALOG_SHA256
    ):
        raise ValueError("V3P independent geometry catalog hash drifted")
    return payload


def v3p_geometry_catalog_sha256() -> str:
    return str(v3p_geometry_catalog()["catalog_content_sha256"])


def _v3p_expand_local_bernstein_grid(
    *,
    degree: int,
    control_grid: Sequence[Sequence[float]],
    box: RationalBox,
) -> RationalPolynomial:
    if degree < 0 or len(control_grid) != degree + 1 or any(
        len(row) != degree + 1 for row in control_grid
    ):
        raise ValueError("V3P local Bernstein grid shape mismatch")
    if box[0] >= box[1] or box[2] >= box[3]:
        raise ValueError("V3P local Bernstein box is empty")
    exact_grid = tuple(
        tuple(Fraction.from_float(float(value)) for value in row)
        for row in control_grid
    )
    if any(not isfinite(float(value)) for row in control_grid for value in row):
        raise ValueError("V3P local Bernstein grid contains non-finite values")

    local_power: RationalPolynomial = {}
    for x_index in range(degree + 1):
        for y_index in range(degree + 1):
            control = exact_grid[x_index][y_index]
            if not control:
                continue
            for x_power in range(x_index, degree + 1):
                x_coefficient = (
                    comb(degree, x_index)
                    * comb(degree - x_index, x_power - x_index)
                    * (-1) ** (x_power - x_index)
                )
                for y_power in range(y_index, degree + 1):
                    y_coefficient = (
                        comb(degree, y_index)
                        * comb(degree - y_index, y_power - y_index)
                        * (-1) ** (y_power - y_index)
                    )
                    power = (x_power, y_power)
                    local_power[power] = local_power.get(
                        power,
                        Fraction(0),
                    ) + control * x_coefficient * y_coefficient

    width_x = box[1] - box[0]
    width_y = box[3] - box[2]
    result: RationalPolynomial = {}
    for (local_x, local_y), coefficient in _clean(local_power).items():
        for global_x in range(local_x + 1):
            x_coefficient = (
                Fraction(comb(local_x, global_x))
                * (-box[0]) ** (local_x - global_x)
                / width_x**local_x
            )
            for global_y in range(local_y + 1):
                y_coefficient = (
                    Fraction(comb(local_y, global_y))
                    * (-box[2]) ** (local_y - global_y)
                    / width_y**local_y
                )
                power = (global_x, global_y)
                result[power] = result.get(power, Fraction(0)) + (
                    coefficient * x_coefficient * y_coefficient
                )
    return _clean(result)


def _v3p_barrier_cell_control_grid(
    barrier: PiecewiseBernsteinBarrier2D,
    cell: _V3PCell,
) -> tuple[tuple[float, ...], ...]:
    degree = barrier.degree
    size = 7 * degree + 1
    if len(barrier.control_grid) != size or any(
        len(row) != size for row in barrier.control_grid
    ):
        raise ValueError("V3P raw barrier control-grid shape mismatch")
    x_start = cell.x_index * degree
    y_start = cell.y_index * degree
    return tuple(
        tuple(
            float(barrier.control_grid[x_start + local_x][y_start + local_y])
            for local_y in range(degree + 1)
        )
        for local_x in range(degree + 1)
    )


def _v3p_barrier_polynomials(
    barrier: PiecewiseBernsteinBarrier2D,
    cells: Sequence[_V3PCell],
) -> dict[str, RationalPolynomial]:
    return {
        cell.cell_id: _v3p_expand_local_bernstein_grid(
            degree=barrier.degree,
            control_grid=_v3p_barrier_cell_control_grid(barrier, cell),
            box=cell.normalized_box,
        )
        for cell in cells
    }


def _v3p_validate_roofs(
    *,
    witness: PiecewiseRoofWitness2D,
    safe_cells: Sequence[_V3PCell],
) -> dict[tuple[str, tuple[int, int]], LocalBernsteinRoof2D]:
    expected_keys = tuple(
        (cell.cell_id, quadrant)
        for cell in safe_cells
        for quadrant in _V3P_QUADRANTS
    )
    actual_keys = tuple(
        (str(roof.source_cell_id), tuple(roof.quadrant))
        for roof in witness.roofs
    )
    if actual_keys != expected_keys:
        raise ValueError("V3P roof ordering/key catalog mismatch")
    by_id = {cell.cell_id: cell for cell in safe_cells}
    roofs: dict[tuple[str, tuple[int, int]], LocalBernsteinRoof2D] = {}
    for roof in witness.roofs:
        key = (str(roof.source_cell_id), tuple(roof.quadrant))
        source = by_id.get(key[0])
        if source is None or key[1] not in _V3P_QUADRANTS:
            raise ValueError("V3P roof source/quadrant mismatch")
        if roof.controller_piece_id != source.controller_piece_id:
            raise ValueError("V3P roof controller-piece mismatch")
        if tuple(roof.normalized_reach_box) != _v3p_reach_box(source, key[1]):
            raise ValueError("V3P roof reach box mismatch")
        if roof.degree != witness.roof_degree:
            raise ValueError("V3P roof degree mismatch")
        roofs[key] = roof
    return roofs


def _v3p_roof_polynomial(
    roof: LocalBernsteinRoof2D,
    source: _V3PCell,
) -> RationalPolynomial:
    return _v3p_expand_local_bernstein_grid(
        degree=roof.degree,
        control_grid=roof.control_grid,
        box=_v3p_reach_box(source, tuple(roof.quadrant)),
    )


def _v3p_half_moment(
    *,
    order: int,
    sign: int,
    half_width: Fraction,
) -> Fraction:
    if order < 0 or sign not in (-1, 1) or half_width <= 0:
        raise ValueError("invalid V3P half-moment request")
    return Fraction(sign**order) * half_width**order / (
        (order + 1) * (order + 2)
    )


def _v3p_arithmetic_gate() -> dict[str, Any]:
    checks: dict[str, bool] = {
        "quadrant_order": _V3P_QUADRANTS
        == ((-1, -1), (-1, 1), (1, -1), (1, 1)),
        "normalization_breaks": _v3p_normalized_breaks()
        == (
            _V3P_EXPECTED_NORMALIZED_X_BREAKS,
            _V3P_EXPECTED_NORMALIZED_Y_BREAKS,
        ),
        "derived_reach_padding": _v3p_reach_padding()
        == (
            Fraction(1, 100),
            Fraction(1, 80),
            Fraction(1, 500),
            Fraction(1, 400),
        ),
    }
    for axis, half_width in (
        ("x", Fraction(1, 500)),
        ("y", Fraction(1, 400)),
    ):
        checks[f"{axis}_half_mass"] = all(
            _v3p_half_moment(order=0, sign=sign, half_width=half_width)
            == Fraction(1, 2)
            for sign in (-1, 1)
        )
        checks[f"{axis}_moments_n0_to_n6"] = all(
            (
                _v3p_half_moment(
                    order=order,
                    sign=-1,
                    half_width=half_width,
                )
                + _v3p_half_moment(
                    order=order,
                    sign=1,
                    half_width=half_width,
                )
            )
            == (
                Fraction(0)
                if order % 2
                else 2
                * half_width**order
                / ((order + 1) * (order + 2))
            )
            for order in range(7)
        )
    checks["constant_quadrant_probability"] = all(
        _v3p_half_moment(
            order=0,
            sign=quadrant[0],
            half_width=Fraction(1, 500),
        )
        * _v3p_half_moment(
            order=0,
            sign=quadrant[1],
            half_width=Fraction(1, 400),
        )
        == Fraction(1, 4)
        for quadrant in _V3P_QUADRANTS
    )
    return {
        "status": "validated" if all(checks.values()) else "violated",
        "arithmetic": "independent_exact_fraction_self_check",
        "checks": checks,
    }


def _v3p_shifted_power_expectation(
    *,
    mean: Mapping[Power2D, Fraction],
    exponent: int,
    sign: int,
    half_width: Fraction,
) -> RationalPolynomial:
    return _add(
        *(
            _scale(
                _power(mean, exponent - noise_order),
                comb(exponent, noise_order)
                * _v3p_half_moment(
                    order=noise_order,
                    sign=sign,
                    half_width=half_width,
                ),
            )
            for noise_order in range(exponent + 1)
        )
    )


def _v3p_quadrant_roof_expectation(
    *,
    polynomial: Mapping[Power2D, Fraction],
    quadrant: tuple[int, int],
    piece: Any,
) -> RationalPolynomial:
    if quadrant not in _V3P_QUADRANTS:
        raise ValueError("invalid V3P expectation quadrant")
    mean_x = _add(
        {(1, 0): Fraction(1)},
        _scale(rational_sparse_polynomial(piece.u_x), DT / NORMALIZATION_SCALE_X),
    )
    mean_y = _add(
        {(0, 1): Fraction(1)},
        _scale(rational_sparse_polynomial(piece.u_y), DT / NORMALIZATION_SCALE_Y),
    )
    max_x = max((power[0] for power in polynomial), default=0)
    max_y = max((power[1] for power in polynomial), default=0)
    x_table = tuple(
        _v3p_shifted_power_expectation(
            mean=mean_x,
            exponent=order,
            sign=quadrant[0],
            half_width=NOISE_HALF_WIDTH / NORMALIZATION_SCALE_X,
        )
        for order in range(max_x + 1)
    )
    y_table = tuple(
        _v3p_shifted_power_expectation(
            mean=mean_y,
            exponent=order,
            sign=quadrant[1],
            half_width=NOISE_HALF_WIDTH / NORMALIZATION_SCALE_Y,
        )
        for order in range(max_y + 1)
    )
    return _add(
        *(
            _scale(
                _multiply(x_table[x_power], y_table[y_power]),
                coefficient,
            )
            for (x_power, y_power), coefficient in polynomial.items()
        )
    )


def _v3p_source_roof_expectation(
    *,
    roofs: Mapping[tuple[int, int], LocalBernsteinRoof2D],
    source: _V3PCell,
    piece: Any,
) -> RationalPolynomial:
    if set(roofs) != set(_V3P_QUADRANTS):
        raise ValueError("V3P source expectation requires four quadrants")
    return _add(
        *(
            _v3p_quadrant_roof_expectation(
                polynomial=_v3p_roof_polynomial(roofs[quadrant], source),
                quadrant=quadrant,
                piece=piece,
            )
            for quadrant in _V3P_QUADRANTS
        )
    )


def _write_json(path: Path, payload: Mapping[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.tmp")
    temporary.write_text(
        json.dumps(
            payload,
            indent=2,
            sort_keys=True,
            ensure_ascii=False,
            allow_nan=False,
        )
        + "\n",
        encoding="utf-8",
    )
    temporary.replace(path)


def _file_sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _constraint_status(constraints: Mapping[str, Mapping[str, Any]]) -> str:
    if all(result["status"] == "validated" for result in constraints.values()):
        return "validated"
    if any(result["status"] == "violated" for result in constraints.values()):
        return "violated"
    return "unknown"


def _record_constraint(
    *,
    destination: Path,
    group: str,
    name: str,
    polynomial: Mapping[Power2D, Fraction],
    root_box: tuple[Fraction, Fraction, Fraction, Fraction],
    classify_domain: Any,
    options: RationalBernsteinValidationOptions,
    summaries: dict[str, dict[str, Any]],
) -> None:
    result = validate_rational_polynomial_nonnegative(
        polynomial,
        root_box,
        classify_domain=classify_domain,
        options=options,
    )
    tree_relative = Path("trees") / group / f"{name}.json"
    tree_path = destination / tree_relative
    _write_json(tree_path, result)
    summary = dict(result)
    summary["tree_file"] = tree_relative.as_posix()
    summary["tree_file_sha256"] = _file_sha256(tree_path)
    summary.pop("leaves", None)
    summaries[name] = summary


def _controller_piece_map(controller: FrozenController) -> dict[str, Any]:
    pieces = controller_pieces(controller)
    expected = (
        ("lower_entry", Fraction(-1), Fraction(4, 5)),
        ("lower_transit", Fraction(4, 5), Fraction(16, 5)),
        ("goal_capture", Fraction(16, 5), Fraction(4)),
    )
    actual = tuple(
        (
            piece.piece_id,
            Fraction(str(float(piece.x_min_m))),
            Fraction(str(float(piece.x_max_m))),
        )
        for piece in pieces
    )
    if actual != expected:
        raise ValueError("V3P requires the frozen three-piece C2 partition")
    return {piece.piece_id: piece for piece in pieces}


def _c0_gate(
    barrier: PiecewiseBernsteinBarrier2D,
    cells: Sequence[_V3PCell] | None = None,
) -> dict[str, Any]:
    local_cells = tuple(cells) if cells is not None else _v3p_build_cells()
    by_index = {
        (cell.x_index, cell.y_index): cell for cell in local_cells
    }
    checked_edges = 0
    mismatches: list[str] = []
    for x_index in range(6):
        for y_index in range(7):
            left = _v3p_barrier_cell_control_grid(
                barrier,
                by_index[(x_index, y_index)],
            )
            right = _v3p_barrier_cell_control_grid(
                barrier,
                by_index[(x_index + 1, y_index)],
            )
            checked_edges += 1
            if tuple(Fraction.from_float(value) for value in left[-1]) != tuple(
                Fraction.from_float(value) for value in right[0]
            ):
                mismatches.append(f"vertical_C_{x_index}_{y_index}")
    for x_index in range(7):
        for y_index in range(6):
            lower = _v3p_barrier_cell_control_grid(
                barrier,
                by_index[(x_index, y_index)],
            )
            upper = _v3p_barrier_cell_control_grid(
                barrier,
                by_index[(x_index, y_index + 1)],
            )
            checked_edges += 1
            lower_edge = tuple(
                Fraction.from_float(row[-1]) for row in lower
            )
            upper_edge = tuple(Fraction.from_float(row[0]) for row in upper)
            if lower_edge != upper_edge:
                mismatches.append(f"horizontal_C_{x_index}_{y_index}")
    return {
        "status": (
            "validated" if checked_edges == 84 and not mismatches else "violated"
        ),
        "arithmetic": "exact_binary64_rational_equality",
        "checked_internal_edge_count": checked_edges,
        "expected_internal_edge_count": 84,
        "mismatched_edge_ids": mismatches,
        "construction": "shared_conforming_edge_control_dofs",
    }


def _p49_catalog_gate(catalog: Mapping[str, Any]) -> dict[str, Any]:
    expected = v3p_geometry_catalog()
    material = dict(catalog)
    declared_hash = material.pop("catalog_content_sha256", None)
    cells = catalog.get("cells")
    obligations = catalog.get("reach_obligations")
    cell_payloads = cells if isinstance(cells, list) else []
    obligation_payloads = obligations if isinstance(obligations, list) else []
    unsafe_bindings = {
        str(cell.get("cell_id")): tuple(cell.get("unsafe_component_ids", []))
        for cell in cell_payloads
        if isinstance(cell, dict) and cell.get("unsafe_component_ids")
    }
    goal_boundary_ids = [
        str(cell.get("cell_id"))
        for cell in cell_payloads
        if isinstance(cell, dict)
        and cell.get("safe_source") is True
        and cell.get("goal_boundary") is True
    ]
    controller_counts: dict[str, int] = {}
    for cell in cell_payloads:
        if not isinstance(cell, dict) or cell.get("safe_source") is not True:
            continue
        piece_id = str(cell.get("controller_piece_id"))
        controller_counts[piece_id] = controller_counts.get(piece_id, 0) + 1
    quadrant_counts: dict[tuple[int, int], int] = {}
    roof_keys: list[tuple[str, tuple[int, int]]] = []
    positive_intersections = True
    for obligation in obligation_payloads:
        if not isinstance(obligation, dict):
            positive_intersections = False
            continue
        quadrant_payload = obligation.get("quadrant")
        if not isinstance(quadrant_payload, list) or len(quadrant_payload) != 2:
            positive_intersections = False
            continue
        quadrant = (int(quadrant_payload[0]), int(quadrant_payload[1]))
        roof_keys.append((str(obligation.get("source_cell_id")), quadrant))
        intersections = obligation.get("target_intersections")
        if not isinstance(intersections, list):
            positive_intersections = False
            continue
        quadrant_counts[quadrant] = quadrant_counts.get(quadrant, 0) + len(
            intersections
        )
        for intersection in intersections:
            if not isinstance(intersection, dict):
                positive_intersections = False
                continue
            box = intersection.get("normalized_box_exact")
            if not isinstance(box, list) or len(box) != 4:
                positive_intersections = False
                continue
            exact_box = tuple(Fraction(str(value)) for value in box)
            if exact_box[0] >= exact_box[1] or exact_box[2] >= exact_box[3]:
                positive_intersections = False
    reach_padding = catalog.get("reach_padding")
    checks = {
        "schema_and_kind": (
            catalog.get("schema_version") == 1
            and catalog.get("kind") == "g11p49_exact_geometry_catalog"
        ),
        "content_hash_self_consistent": (
            declared_hash == content_sha256(material)
            and declared_hash == G11P49_V3P_GEOMETRY_CATALOG_SHA256
        ),
        "full_independent_payload_match": dict(catalog) == expected,
        "counts": catalog.get("counts") == _EXPECTED_P49_COUNTS,
        "exact_breaks": all(
            catalog.get(name) == expected.get(name)
            for name in (
                "physical_x_breaks_exact",
                "physical_y_breaks_exact",
                "normalized_x_breaks_exact",
                "normalized_y_breaks_exact",
            )
        ),
        "unsafe_cell_bindings": unsafe_bindings
        == _EXPECTED_UNSAFE_CELL_BINDINGS,
        "goal_boundary_cells": goal_boundary_ids == ["C_6_2", "C_6_3", "C_6_4"],
        "controller_source_counts": controller_counts
        == {"lower_entry": 7, "lower_transit": 23, "goal_capture": 7},
        "canonical_roof_keys": roof_keys
        == [
            (cell.cell_id, quadrant)
            for cell in _v3p_build_cells()
            if cell.safe_source
            for quadrant in _V3P_QUADRANTS
        ],
        "quadrant_intersection_counts": quadrant_counts
        == {quadrant: 253 for quadrant in _V3P_QUADRANTS},
        "positive_area_intersections": positive_intersections,
        "reach_padding": isinstance(reach_padding, dict)
        and reach_padding
        == {
            "action_shift_x_normalized": "1/100",
            "action_shift_y_normalized": "1/80",
            "noise_half_width_x_normalized": "1/500",
            "noise_half_width_y_normalized": "1/400",
            "derived_from_action_disk_not_controller_range": True,
        },
    }
    return {
        "status": "validated" if all(checks.values()) else "violated",
        "catalog_content_sha256": declared_hash,
        "counts": dict(catalog.get("counts", {})),
        "expected_counts": dict(_EXPECTED_P49_COUNTS),
        "arithmetic": "exact_rational_rebuild_and_frozen_content_hash",
        "checks": checks,
    }


def validate_piecewise_qsbc_v3p(
    *,
    barrier: PiecewiseBernsteinBarrier2D,
    roof_witness: PiecewiseRoofWitness2D,
    q: float,
    epsbar: float,
    controller: FrozenController,
    output_dir: str | Path,
    options: RationalBernsteinValidationOptions | None = None,
) -> Path:
    """Replay G4 through the frozen P49 roof sufficient theorem."""

    _assert_canonical_runtime_contract(controller)
    pieces = _controller_piece_map(controller)
    validation_options = options or RationalBernsteinValidationOptions()
    if validation_options.required_margin != 0:
        raise ValueError("V3P G4 replay requires zero formal margin")

    model_payload = canonical_rational_model_payload()
    model_hash = content_sha256(model_payload)
    geometry_hash = v3p_geometry_catalog_sha256()
    barrier_hash = barrier.payload_sha256
    if barrier.partition_id != P49_PARTITION_ID or barrier.schema_version != 1:
        raise ValueError("V3P raw barrier partition/schema mismatch")
    if barrier.degree not in (3, 4):
        raise ValueError("V3P barrier degree is outside the frozen schedule")
    if roof_witness.barrier_payload_sha256 != barrier_hash:
        raise ValueError("V3P barrier/witness hash mismatch")
    if roof_witness.controller_content_sha256 != controller.sha256:
        raise ValueError("V3P controller/witness hash mismatch")
    if roof_witness.formal_model_contract_sha256 != model_hash:
        raise ValueError("V3P model/witness hash mismatch")
    if roof_witness.geometry_catalog_sha256 != geometry_hash:
        raise ValueError("V3P geometry/witness hash mismatch")
    if roof_witness.barrier_degree != barrier.degree:
        raise ValueError("V3P barrier/witness degree mismatch")

    cells = _v3p_build_cells()
    cell_by_id = {cell.cell_id: cell for cell in cells}
    safe_cells = tuple(cell for cell in cells if cell.safe_source)
    unsafe_cells = tuple(cell for cell in cells if cell.unsafe)
    goal_boundary_cells = tuple(
        cell for cell in safe_cells if cell.goal_boundary
    )
    catalog = v3p_geometry_catalog()
    p49_catalog_gate = _p49_catalog_gate(catalog)
    c0_gate = _c0_gate(barrier, cells)
    arithmetic_gate = _v3p_arithmetic_gate()
    canonical_geometry_gate = exact_geometry_gate()

    destination = Path(output_dir).resolve()
    destination.mkdir(parents=True, exist_ok=False)
    q_exact = _fraction_from_artifact(q)
    epsbar_exact = _fraction_from_artifact(epsbar)
    barrier_polynomials = _v3p_barrier_polynomials(barrier, cells)

    qsbc_constraints: dict[str, dict[str, Any]] = {}
    for cell in cells:
        _record_constraint(
            destination=destination,
            group="qsbc",
            name=f"W_nonnegative_{cell.cell_id}",
            polynomial=barrier_polynomials[cell.cell_id],
            root_box=cell.normalized_box,
            classify_domain=classify_full_box,
            options=validation_options,
            summaries=qsbc_constraints,
        )

    initial_physical_box = (
        INITIAL_DISK.center_x - INITIAL_DISK.radius,
        INITIAL_DISK.center_x + INITIAL_DISK.radius,
        INITIAL_DISK.center_y - INITIAL_DISK.radius,
        INITIAL_DISK.center_y + INITIAL_DISK.radius,
    )
    initial_cell = cell_by_id["C_0_3"]
    _record_constraint(
        destination=destination,
        group="qsbc",
        name="q_minus_W_on_X0",
        polynomial=_add(
            _constant(q_exact),
            _negate(barrier_polynomials[initial_cell.cell_id]),
        ),
        root_box=normalize_physical_box(initial_physical_box),
        classify_domain=classify_included_disk_box,
        options=validation_options,
        summaries=qsbc_constraints,
    )

    for cell in unsafe_cells:
        _record_constraint(
            destination=destination,
            group="qsbc",
            name=f"W_minus_1_{cell.cell_id}",
            polynomial=_add(
                barrier_polynomials[cell.cell_id],
                _constant(-1),
            ),
            root_box=cell.normalized_box,
            classify_domain=classify_full_box,
            options=validation_options,
            summaries=qsbc_constraints,
        )

    roofs = _v3p_validate_roofs(
        witness=roof_witness,
        safe_cells=safe_cells,
    )
    roof_polynomials = {
        (source.cell_id, quadrant): _v3p_roof_polynomial(
            roofs[(source.cell_id, quadrant)],
            source,
        )
        for source in safe_cells
        for quadrant in _V3P_QUADRANTS
    }
    roof_constraints: dict[str, dict[str, Any]] = {}
    for source in safe_cells:
        for quadrant in _V3P_QUADRANTS:
            roof_polynomial = roof_polynomials[(source.cell_id, quadrant)]
            for target, intersection in _v3p_reach_target_intersections(
                source,
                quadrant,
                cells,
            ):
                name = (
                    f"R_{source.cell_id}_{_v3p_quadrant_id(quadrant)}"
                    f"_dominates_{target.cell_id}"
                )
                _record_constraint(
                    destination=destination,
                    group="roof",
                    name=name,
                    polynomial=_add(
                        roof_polynomial,
                        _negate(barrier_polynomials[target.cell_id]),
                    ),
                    root_box=intersection,
                    classify_domain=classify_full_box,
                    options=validation_options,
                    summaries=roof_constraints,
                )

    drift_constraints: dict[str, dict[str, Any]] = {}
    goal_boundary_ids = {cell.cell_id for cell in goal_boundary_cells}
    for source in safe_cells:
        source_roofs = {
            quadrant: roofs[(source.cell_id, quadrant)]
            for quadrant in _V3P_QUADRANTS
        }
        expectation = _v3p_source_roof_expectation(
            roofs=source_roofs,
            source=source,
            piece=pieces[source.controller_piece_id],
        )
        residual = _add(
            barrier_polynomials[source.cell_id],
            _negate(expectation),
            _constant(-epsbar_exact),
        )
        _record_constraint(
            destination=destination,
            group="drift",
            name=f"integrated_roof_drift_{source.cell_id}",
            polynomial=residual,
            root_box=source.normalized_box,
            classify_domain=(
                classify_goal_exterior_box
                if source.cell_id in goal_boundary_ids
                else classify_full_box
            ),
            options=validation_options,
            summaries=drift_constraints,
        )

    controller_options = replace(
        validation_options,
        required_margin=CONTROLLER_REQUIRED_MARGIN,
    )
    controller_constraints: dict[str, dict[str, Any]] = {}
    for piece in controller_pieces(controller):
        piece_box = _piece_normalized_box(piece)
        _record_constraint(
            destination=destination,
            group="controller",
            name=f"{piece.piece_id}_action_disk",
            polynomial=_action_disk_residual(piece),
            root_box=piece_box,
            classify_domain=classify_full_box,
            options=controller_options,
            summaries=controller_constraints,
        )
        for name, residual in _robust_domain_residuals(piece).items():
            _record_constraint(
                destination=destination,
                group="controller",
                name=f"{piece.piece_id}_{name}",
                polynomial=residual,
                root_box=piece_box,
                classify_domain=classify_full_box,
                options=controller_options,
                summaries=controller_constraints,
            )

    qsbc_status = _constraint_status(qsbc_constraints)
    roof_status = _constraint_status(roof_constraints)
    drift_status = _constraint_status(drift_constraints)
    controller_status = _constraint_status(controller_constraints)
    geometry_status = (
        "validated"
        if canonical_geometry_gate["status"] == "validated"
        and p49_catalog_gate["status"] == "validated"
        and c0_gate["status"] == "validated"
        and arithmetic_gate["status"] == "validated"
        else "violated"
    )
    theorem_scalar_gate = (
        Fraction(0) <= q_exact < Fraction(1) and epsbar_exact > 0
    )
    combined_statuses = (
        qsbc_status,
        roof_status,
        drift_status,
        controller_status,
        geometry_status,
    )
    formal_status = (
        "validated"
        if all(status == "validated" for status in combined_statuses)
        and theorem_scalar_gate
        else (
            "violated"
            if "violated" in combined_statuses or not theorem_scalar_gate
            else "unknown"
        )
    )

    report = {
        "schema_version": 1,
        "kind": "independent_piecewise_qsbc_validation",
        "validator_id": G11P49_VALIDATOR_ID,
        "proof_route_id": G11P49_PROOF_ROUTE_ID,
        "status": formal_status,
        "qsbc_status": qsbc_status,
        "roof_domination_status": roof_status,
        "integrated_drift_status": drift_status,
        "controller_contract_status": controller_status,
        "geometry_contract_status": geometry_status,
        "formal_acceptance_tolerance": 0.0,
        "theorem_scalar_gate": theorem_scalar_gate,
        "q": float(q),
        "q_float64_hex": float(q).hex(),
        "q_exact": _fraction_text(q_exact),
        "epsbar": float(epsbar),
        "epsbar_float64_hex": float(epsbar).hex(),
        "epsbar_exact": _fraction_text(epsbar_exact),
        "first_slice_q_max_exact": _fraction_text(FIRST_SLICE_Q_MAX),
        "first_slice_ready": formal_status == "validated" and first_slice_q_gate(q),
        "barrier_payload_sha256": barrier_hash,
        "barrier_degree": barrier.degree,
        "barrier_global_control_dof_count": (7 * barrier.degree + 1) ** 2,
        "roof_witness_content_sha256": roof_witness.sha256,
        "roof_degree": roof_witness.roof_degree,
        "roof_count": len(roof_witness.roofs),
        "controller_content_sha256": controller.sha256,
        "formal_model_contract": model_payload,
        "formal_model_contract_sha256": model_hash,
        "geometry_catalog_sha256": geometry_hash,
        "geometry_gate": canonical_geometry_gate,
        "p49_catalog_gate": p49_catalog_gate,
        "c0_gate": c0_gate,
        "v3p_arithmetic_gate": arithmetic_gate,
        "constraint_counts": {
            "qsbc": len(qsbc_constraints),
            "roof_domination": len(roof_constraints),
            "integrated_drift": len(drift_constraints),
            "controller": len(controller_constraints),
        },
        "qsbc_constraints": qsbc_constraints,
        "roof_domination_constraints": roof_constraints,
        "integrated_drift_constraints": drift_constraints,
        "controller_constraints": controller_constraints,
        "independence": {
            "solver_status_read": False,
            "LP_matrix_read": False,
            "LP_multiplier_read": False,
            "numeric_slack_used_for_status": False,
            "sampled_residual_used": False,
            "float_polynomial_algebra_used_for_status": False,
            "exact_binary64_artifact_coefficients": True,
            "exact_decimal_model_constants": True,
            "P49_catalog_rebuilt_without_generator_helpers": True,
            "barrier_cell_slicing_rebuilt_from_raw_global_grid": True,
            "roof_polynomials_rebuilt_from_raw_local_grids": True,
            "quadrant_expectation_rebuilt_without_synthesis_helpers": True,
            "controller_S1_and_D1_replayed": True,
        },
        "domain_physical_exact": [_fraction_text(value) for value in DOMAIN_PHYSICAL],
    }
    report["report_content_sha256"] = content_sha256(report)
    report_path = destination / "validation_report.json"
    _write_json(report_path, report)
    return report_path
