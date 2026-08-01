"""P49 conforming piecewise-Bernstein barrier and roof artifacts.

This module contains no solver and makes no formal claim.  It freezes the
geometry and serialization used by the G11P49 candidate generator and by the
independent V3P validator.  A single global control net is shared by all 49
cells; edge coefficients are therefore identical by construction and the
resulting barrier is C0 on the whole compact domain.
"""

from __future__ import annotations

from dataclasses import dataclass
from fractions import Fraction
import json
from math import comb, isfinite
from pathlib import Path
from typing import Any, Mapping, Sequence

from .polynomial import Power2D, content_sha256


P49_PARTITION_ID = "P49_C2_OBSTACLE_ALIGNED_CARTESIAN_V1"
G11P49_REACH_CATALOG_ID = "G11P49_ACTION_DISK_QUADRANT_REACH_CATALOG_V1"
G11P49_PROOF_ROUTE_ID = "G11P49_C0_PIECEWISE_ROOF_SUFFICIENT_G4_V1"
G11P49_SYNTHESIS_ID = "G11P49_C0_BERNSTEIN_ROOF_LP_V1"
G11P49_VALIDATOR_ID = "G5_EXACT_RATIONAL_PIECEWISE_ROOF_REPLAY_V3P_V1"
G11P49_FROZEN_Q_FLOAT64_HEX = "0x1.9999999999999p-4"
G11P49_FROZEN_EPSBAR_FLOAT64_HEX = "0x1.0c6f7a0b5ed8dp-20"
G11P49_GEOMETRY_CATALOG_SHA256 = (
    "ef545affb1c35d965be5d4b9d9e5ab39854e4d8069c296b924d45e049a88924a"
)

P49_PHYSICAL_X_BREAKS: tuple[Fraction, ...] = tuple(
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
P49_PHYSICAL_Y_BREAKS: tuple[Fraction, ...] = tuple(
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
P49_NORMALIZED_X_BREAKS: tuple[Fraction, ...] = (
    Fraction(-1),
    Fraction(-7, 25),
    Fraction(-107, 500),
    Fraction(83, 500),
    Fraction(107, 500),
    Fraction(317, 500),
    Fraction(17, 25),
    Fraction(1),
)
P49_NORMALIZED_Y_BREAKS: tuple[Fraction, ...] = (
    Fraction(-1),
    Fraction(-277, 400),
    Fraction(-107, 400),
    Fraction(-43, 400),
    Fraction(43, 400),
    Fraction(107, 400),
    Fraction(277, 400),
    Fraction(1),
)

P49_QUADRANTS: tuple[tuple[int, int], ...] = (
    (-1, -1),
    (-1, 1),
    (1, -1),
    (1, 1),
)

_DOMAIN_NORMALIZED = (
    Fraction(-1),
    Fraction(1),
    Fraction(-1),
    Fraction(1),
)
_ACTION_SHIFT_X_NORMALIZED = Fraction(1, 100)
_ACTION_SHIFT_Y_NORMALIZED = Fraction(1, 80)
NOISE_HALF_WIDTH_X_NORMALIZED = Fraction(1, 500)
NOISE_HALF_WIDTH_Y_NORMALIZED = Fraction(1, 400)

_UNSAFE_PHYSICAL: tuple[tuple[str, tuple[Fraction, ...]], ...] = (
    (
        "Xu1",
        (
            Fraction("0.965"),
            Fraction("2.035"),
            Fraction("-0.535"),
            Fraction("0.535"),
        ),
    ),
    (
        "Xu2",
        (
            Fraction("1.915"),
            Fraction("3.085"),
            Fraction("0.215"),
            Fraction("1.385"),
        ),
    ),
    (
        "Xu3",
        (
            Fraction("1.915"),
            Fraction("3.085"),
            Fraction("-1.385"),
            Fraction("-0.215"),
        ),
    ),
)
_GOAL_CENTER_PHYSICAL = (Fraction("3.5"), Fraction(0))
_GOAL_RADIUS_PHYSICAL = Fraction("0.25")

RationalPolynomial = dict[Power2D, Fraction]


def _fraction_text(value: Fraction) -> str:
    if value.denominator == 1:
        return str(value.numerator)
    return f"{value.numerator}/{value.denominator}"


def _fraction_from_text(value: Any) -> Fraction:
    if not isinstance(value, str):
        raise ValueError("exact rational values must be serialized as strings")
    return Fraction(value)


def _box_payload(box: Sequence[Fraction]) -> list[str]:
    return [_fraction_text(value) for value in box]


def _validate_sha256(value: str, *, name: str) -> str:
    digest = str(value).lower()
    if len(digest) != 64 or any(
        character not in "0123456789abcdef" for character in digest
    ):
        raise ValueError(f"{name} is not a lowercase SHA-256 digest")
    return digest


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


def _point_inside_box(
    point: tuple[Fraction, Fraction],
    box: Sequence[Fraction],
) -> bool:
    return box[0] <= point[0] <= box[1] and box[2] <= point[1] <= box[3]


def _box_distance_bounds_squared(
    box: Sequence[Fraction],
    point: tuple[Fraction, Fraction],
) -> tuple[Fraction, Fraction]:
    x_low, x_high, y_low, y_high = box
    x_value, y_value = point
    minimum_x = (
        x_low - x_value
        if x_value < x_low
        else x_value - x_high
        if x_value > x_high
        else Fraction(0)
    )
    minimum_y = (
        y_low - y_value
        if y_value < y_low
        else y_value - y_high
        if y_value > y_high
        else Fraction(0)
    )
    maximum_x = max(abs(x_low - x_value), abs(x_high - x_value))
    maximum_y = max(abs(y_low - y_value), abs(y_high - y_value))
    return (
        minimum_x**2 + minimum_y**2,
        maximum_x**2 + maximum_y**2,
    )


def _controller_piece_id_for_x_index(x_index: int) -> str:
    if x_index == 0:
        return "lower_entry"
    if 1 <= x_index <= 5:
        return "lower_transit"
    if x_index == 6:
        return "goal_capture"
    raise ValueError("P49 x index lies outside the frozen partition")


@dataclass(frozen=True, slots=True)
class P49Cell:
    """One closed cell in the frozen conforming P49 partition."""

    x_index: int
    y_index: int
    physical_box: tuple[Fraction, Fraction, Fraction, Fraction]
    normalized_box: tuple[Fraction, Fraction, Fraction, Fraction]
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


def _build_p49_cells() -> tuple[P49Cell, ...]:
    cells: list[P49Cell] = []
    for x_index in range(7):
        for y_index in range(7):
            physical_box = (
                P49_PHYSICAL_X_BREAKS[x_index],
                P49_PHYSICAL_X_BREAKS[x_index + 1],
                P49_PHYSICAL_Y_BREAKS[y_index],
                P49_PHYSICAL_Y_BREAKS[y_index + 1],
            )
            normalized_box = (
                P49_NORMALIZED_X_BREAKS[x_index],
                P49_NORMALIZED_X_BREAKS[x_index + 1],
                P49_NORMALIZED_Y_BREAKS[y_index],
                P49_NORMALIZED_Y_BREAKS[y_index + 1],
            )
            center = (
                (physical_box[0] + physical_box[1]) / 2,
                (physical_box[2] + physical_box[3]) / 2,
            )
            unsafe_ids = tuple(
                unsafe_id
                for unsafe_id, unsafe_box in _UNSAFE_PHYSICAL
                if _point_inside_box(center, unsafe_box)
            )
            minimum_goal_distance, maximum_goal_distance = (
                _box_distance_bounds_squared(
                    physical_box,
                    _GOAL_CENTER_PHYSICAL,
                )
            )
            goal_radius_squared = _GOAL_RADIUS_PHYSICAL**2
            goal_boundary = (
                minimum_goal_distance <= goal_radius_squared
                and maximum_goal_distance > goal_radius_squared
            )
            cells.append(
                P49Cell(
                    x_index=x_index,
                    y_index=y_index,
                    physical_box=physical_box,
                    normalized_box=normalized_box,
                    unsafe_component_ids=unsafe_ids,
                    goal_boundary=goal_boundary,
                    controller_piece_id=_controller_piece_id_for_x_index(
                        x_index
                    ),
                )
            )
    return tuple(cells)


P49_CELLS = _build_p49_cells()
P49_CELL_BY_ID = {cell.cell_id: cell for cell in P49_CELLS}
P49_SAFE_SOURCE_CELLS = tuple(cell for cell in P49_CELLS if cell.safe_source)
P49_UNSAFE_CELLS = tuple(cell for cell in P49_CELLS if cell.unsafe)
P49_GOAL_BOUNDARY_SAFE_CELLS = tuple(
    cell for cell in P49_SAFE_SOURCE_CELLS if cell.goal_boundary
)


def p49_quadrant_id(quadrant: tuple[int, int]) -> str:
    if quadrant not in P49_QUADRANTS:
        raise ValueError("P49 quadrant signs must both be -1 or +1")
    return "".join("p" if sign > 0 else "m" for sign in quadrant)


def p49_quadrant_reach_box(
    cell: P49Cell,
    quadrant: tuple[int, int],
) -> tuple[Fraction, Fraction, Fraction, Fraction]:
    """Return the exact action-disk/N1 reach superset, clipped to D1."""

    if not cell.safe_source:
        raise ValueError("P49 reach boxes are defined only for safe source cells")
    if quadrant not in P49_QUADRANTS:
        raise ValueError("invalid P49 noise quadrant")
    x_low, x_high, y_low, y_high = cell.normalized_box
    x_noise_low = (
        -NOISE_HALF_WIDTH_X_NORMALIZED if quadrant[0] < 0 else Fraction(0)
    )
    x_noise_high = (
        NOISE_HALF_WIDTH_X_NORMALIZED if quadrant[0] > 0 else Fraction(0)
    )
    y_noise_low = (
        -NOISE_HALF_WIDTH_Y_NORMALIZED if quadrant[1] < 0 else Fraction(0)
    )
    y_noise_high = (
        NOISE_HALF_WIDTH_Y_NORMALIZED if quadrant[1] > 0 else Fraction(0)
    )
    return (
        max(
            _DOMAIN_NORMALIZED[0],
            x_low - _ACTION_SHIFT_X_NORMALIZED + x_noise_low,
        ),
        min(
            _DOMAIN_NORMALIZED[1],
            x_high + _ACTION_SHIFT_X_NORMALIZED + x_noise_high,
        ),
        max(
            _DOMAIN_NORMALIZED[2],
            y_low - _ACTION_SHIFT_Y_NORMALIZED + y_noise_low,
        ),
        min(
            _DOMAIN_NORMALIZED[3],
            y_high + _ACTION_SHIFT_Y_NORMALIZED + y_noise_high,
        ),
    )


def _positive_area_intersection(
    left: Sequence[Fraction],
    right: Sequence[Fraction],
) -> tuple[Fraction, Fraction, Fraction, Fraction] | None:
    intersection = (
        max(left[0], right[0]),
        min(left[1], right[1]),
        max(left[2], right[2]),
        min(left[3], right[3]),
    )
    if intersection[0] >= intersection[1] or intersection[2] >= intersection[3]:
        return None
    return intersection


def p49_reach_target_intersections(
    cell: P49Cell,
    quadrant: tuple[int, int],
) -> tuple[
    tuple[P49Cell, tuple[Fraction, Fraction, Fraction, Fraction]], ...
]:
    reach_box = p49_quadrant_reach_box(cell, quadrant)
    result: list[
        tuple[P49Cell, tuple[Fraction, Fraction, Fraction, Fraction]]
    ] = []
    for target in P49_CELLS:
        intersection = _positive_area_intersection(
            reach_box,
            target.normalized_box,
        )
        if intersection is not None:
            result.append((target, intersection))
    return tuple(result)


def p49_geometry_catalog() -> dict[str, Any]:
    obligations: list[dict[str, Any]] = []
    target_intersection_count = 0
    for source in P49_SAFE_SOURCE_CELLS:
        for quadrant in P49_QUADRANTS:
            intersections = p49_reach_target_intersections(source, quadrant)
            target_intersection_count += len(intersections)
            obligations.append(
                {
                    "source_cell_id": source.cell_id,
                    "controller_piece_id": source.controller_piece_id,
                    "quadrant": list(quadrant),
                    "quadrant_id": p49_quadrant_id(quadrant),
                    "normalized_reach_box_exact": _box_payload(
                        p49_quadrant_reach_box(source, quadrant)
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
    payload: dict[str, Any] = {
        "schema_version": 1,
        "kind": "g11p49_exact_geometry_catalog",
        "partition_id": P49_PARTITION_ID,
        "reach_catalog_id": G11P49_REACH_CATALOG_ID,
        "physical_x_breaks_exact": _box_payload(P49_PHYSICAL_X_BREAKS),
        "physical_y_breaks_exact": _box_payload(P49_PHYSICAL_Y_BREAKS),
        "normalized_x_breaks_exact": _box_payload(P49_NORMALIZED_X_BREAKS),
        "normalized_y_breaks_exact": _box_payload(P49_NORMALIZED_Y_BREAKS),
        "quadrant_order": [list(value) for value in P49_QUADRANTS],
        "reach_padding": {
            "action_shift_x_normalized": _fraction_text(
                _ACTION_SHIFT_X_NORMALIZED
            ),
            "action_shift_y_normalized": _fraction_text(
                _ACTION_SHIFT_Y_NORMALIZED
            ),
            "noise_half_width_x_normalized": _fraction_text(
                NOISE_HALF_WIDTH_X_NORMALIZED
            ),
            "noise_half_width_y_normalized": _fraction_text(
                NOISE_HALF_WIDTH_Y_NORMALIZED
            ),
            "derived_from_action_disk_not_controller_range": True,
        },
        "counts": {
            "cell_count": len(P49_CELLS),
            "unsafe_cell_count": len(P49_UNSAFE_CELLS),
            "safe_source_cell_count": len(P49_SAFE_SOURCE_CELLS),
            "goal_boundary_safe_cell_count": len(
                P49_GOAL_BOUNDARY_SAFE_CELLS
            ),
            "roof_count": len(obligations),
            "target_intersection_count": target_intersection_count,
            "internal_conforming_edge_count": 84,
        },
        "cells": [cell.payload() for cell in P49_CELLS],
        "reach_obligations": obligations,
    }
    payload["catalog_content_sha256"] = content_sha256(payload)
    return payload


def p49_geometry_catalog_sha256() -> str:
    digest = str(p49_geometry_catalog()["catalog_content_sha256"])
    if digest != G11P49_GEOMETRY_CATALOG_SHA256:
        raise ValueError("P49 frozen geometry-catalog content hash drifted")
    return digest


def _bernstein_values(degree: int, coordinate: float) -> tuple[float, ...]:
    return tuple(
        comb(degree, index)
        * coordinate**index
        * (1.0 - coordinate) ** (degree - index)
        for index in range(degree + 1)
    )


def _axis_bernstein_rational_polynomial(
    *,
    degree: int,
    index: int,
    low: Fraction,
    high: Fraction,
) -> dict[int, Fraction]:
    """Expand one local Bernstein basis function in the global coordinate."""

    if degree < 0 or not 0 <= index <= degree or low >= high:
        raise ValueError("invalid local Bernstein basis request")
    width = high - low
    result: dict[int, Fraction] = {}
    # C(d,i) * (z-low)^i * (high-z)^(d-i) / width^d.
    for left_power in range(index + 1):
        left_coefficient = (
            Fraction(comb(index, left_power))
            * (-low) ** (index - left_power)
        )
        for right_power in range(degree - index + 1):
            right_coefficient = (
                Fraction(comb(degree - index, right_power))
                * high ** (degree - index - right_power)
                * (-1) ** right_power
            )
            power = left_power + right_power
            result[power] = result.get(power, Fraction(0)) + (
                Fraction(comb(degree, index))
                * left_coefficient
                * right_coefficient
                / width**degree
            )
    return {power: value for power, value in result.items() if value}


def local_bernstein_grid_rational_polynomial(
    *,
    degree: int,
    control_grid: Sequence[Sequence[float]],
    box: Sequence[Fraction],
) -> RationalPolynomial:
    """Expand a binary64 local tensor-Bernstein grid exactly."""

    if len(control_grid) != degree + 1 or any(
        len(row) != degree + 1 for row in control_grid
    ):
        raise ValueError("local Bernstein control-grid shape mismatch")
    basis = local_bernstein_basis_rational_polynomials(
        degree=degree,
        box=box,
    )
    result: RationalPolynomial = {}
    for basis_polynomial, coefficient in zip(
        basis,
        (value for row in control_grid for value in row),
    ):
        exact_coefficient = Fraction.from_float(float(coefficient))
        if exact_coefficient == 0:
            continue
        for power, value in basis_polynomial.items():
            result[power] = result.get(power, Fraction(0)) + (
                exact_coefficient * value
            )
    return {power: value for power, value in result.items() if value}


def local_bernstein_basis_rational_polynomials(
    *,
    degree: int,
    box: Sequence[Fraction],
) -> tuple[RationalPolynomial, ...]:
    """Return the canonical x-major local tensor-Bernstein basis exactly."""

    if degree < 0 or len(box) != 4 or box[0] >= box[1] or box[2] >= box[3]:
        raise ValueError("invalid local Bernstein basis box")
    x_basis = tuple(
        _axis_bernstein_rational_polynomial(
            degree=degree,
            index=index,
            low=box[0],
            high=box[1],
        )
        for index in range(degree + 1)
    )
    y_basis = tuple(
        _axis_bernstein_rational_polynomial(
            degree=degree,
            index=index,
            low=box[2],
            high=box[3],
        )
        for index in range(degree + 1)
    )
    return tuple(
        {
            (x_power, y_power): x_value * y_value
            for x_power, x_value in x_polynomial.items()
            for y_power, y_value in y_polynomial.items()
            if x_value * y_value
        }
        for x_polynomial in x_basis
        for y_polynomial in y_basis
    )


def _validated_control_grid(
    values: Sequence[Sequence[float]],
    *,
    expected_size: int,
    name: str,
) -> tuple[tuple[float, ...], ...]:
    if len(values) != expected_size or any(
        len(row) != expected_size for row in values
    ):
        raise ValueError(f"{name} control-grid shape mismatch")
    result = tuple(tuple(float(value) for value in row) for row in values)
    if any(not isfinite(value) for row in result for value in row):
        raise ValueError(f"{name} control grid contains a non-finite value")
    return result


@dataclass(frozen=True, slots=True)
class PiecewiseBernsteinBarrier2D:
    """C0 P49 barrier represented by one shared global control net."""

    degree: int
    control_grid: tuple[tuple[float, ...], ...]
    partition_id: str = P49_PARTITION_ID
    schema_version: int = 1

    def __post_init__(self) -> None:
        degree = int(self.degree)
        if degree < 1:
            raise ValueError("piecewise barrier degree must be positive")
        if self.schema_version != 1:
            raise ValueError("unsupported piecewise barrier schema")
        if self.partition_id != P49_PARTITION_ID:
            raise ValueError("piecewise barrier partition identity mismatch")
        size = 7 * degree + 1
        grid = _validated_control_grid(
            self.control_grid,
            expected_size=size,
            name="piecewise barrier",
        )
        object.__setattr__(self, "degree", degree)
        object.__setattr__(self, "control_grid", grid)

    @property
    def control_grid_shape(self) -> tuple[int, int]:
        size = 7 * self.degree + 1
        return size, size

    @property
    def dof_count(self) -> int:
        size = 7 * self.degree + 1
        return size**2

    def cell_control_grid(
        self,
        x_index: int,
        y_index: int,
    ) -> tuple[tuple[float, ...], ...]:
        if not 0 <= x_index < 7 or not 0 <= y_index < 7:
            raise ValueError("piecewise barrier cell index is outside P49")
        x_start = x_index * self.degree
        y_start = y_index * self.degree
        return tuple(
            tuple(
                self.control_grid[x_start + local_x][y_start + local_y]
                for local_y in range(self.degree + 1)
            )
            for local_x in range(self.degree + 1)
        )

    def cell_rational_polynomial(self, cell_id: str) -> RationalPolynomial:
        cell = P49_CELL_BY_ID.get(str(cell_id))
        if cell is None:
            raise ValueError("unknown P49 barrier cell")
        return local_bernstein_grid_rational_polynomial(
            degree=self.degree,
            control_grid=self.cell_control_grid(cell.x_index, cell.y_index),
            box=cell.normalized_box,
        )

    def evaluate(self, xi_x: float, xi_y: float) -> float:
        x_value = float(xi_x)
        y_value = float(xi_y)
        if not isfinite(x_value) or not isfinite(y_value):
            raise ValueError("piecewise barrier state must be finite")
        x_breaks = tuple(float(value) for value in P49_NORMALIZED_X_BREAKS)
        y_breaks = tuple(float(value) for value in P49_NORMALIZED_Y_BREAKS)
        x_index = next(
            (
                index
                for index in range(7)
                if x_breaks[index] <= x_value <= x_breaks[index + 1]
            ),
            None,
        )
        y_index = next(
            (
                index
                for index in range(7)
                if y_breaks[index] <= y_value <= y_breaks[index + 1]
            ),
            None,
        )
        if x_index is None or y_index is None:
            raise ValueError("piecewise barrier state lies outside D1")
        local_x = (x_value - x_breaks[x_index]) / (
            x_breaks[x_index + 1] - x_breaks[x_index]
        )
        local_y = (y_value - y_breaks[y_index]) / (
            y_breaks[y_index + 1] - y_breaks[y_index]
        )
        x_values = _bernstein_values(self.degree, local_x)
        y_values = _bernstein_values(self.degree, local_y)
        controls = self.cell_control_grid(x_index, y_index)
        return sum(
            controls[x_local][y_local]
            * x_values[x_local]
            * y_values[y_local]
            for x_local in range(self.degree + 1)
            for y_local in range(self.degree + 1)
        )

    def payload(self) -> dict[str, Any]:
        return {
            "schema_version": self.schema_version,
            "kind": "piecewise_tensor_bernstein_barrier_2d",
            "partition_id": self.partition_id,
            "variable_order": ["xi_x", "xi_y"],
            "basis": "local_tensor_bernstein",
            "degree_per_axis": self.degree,
            "normalized_x_breaks_exact": _box_payload(
                P49_NORMALIZED_X_BREAKS
            ),
            "normalized_y_breaks_exact": _box_payload(
                P49_NORMALIZED_Y_BREAKS
            ),
            "global_control_grid_shape": list(self.control_grid_shape),
            "global_control_dof_count": self.dof_count,
            "global_control_grid_decimal": [
                list(row) for row in self.control_grid
            ],
            "global_control_grid_float64_hex": [
                [value.hex() for value in row] for row in self.control_grid
            ],
            "continuity": {
                "class": "C0",
                "construction": "shared_conforming_edge_control_dofs",
                "internal_edge_count": 84,
                "separate_float_equality_rows_required": False,
            },
        }

    @property
    def payload_sha256(self) -> str:
        return content_sha256(self.payload())

    @classmethod
    def from_payload(
        cls,
        payload: Mapping[str, Any],
    ) -> "PiecewiseBernsteinBarrier2D":
        if payload.get("kind") != "piecewise_tensor_bernstein_barrier_2d":
            raise ValueError("piecewise barrier kind mismatch")
        if payload.get("variable_order") != ["xi_x", "xi_y"]:
            raise ValueError("piecewise barrier variable order mismatch")
        if payload.get("basis") != "local_tensor_bernstein":
            raise ValueError("piecewise barrier basis mismatch")
        if payload.get("normalized_x_breaks_exact") != _box_payload(
            P49_NORMALIZED_X_BREAKS
        ) or payload.get("normalized_y_breaks_exact") != _box_payload(
            P49_NORMALIZED_Y_BREAKS
        ):
            raise ValueError("piecewise barrier exact partition mismatch")
        decimal = tuple(
            tuple(float(value) for value in row)
            for row in payload["global_control_grid_decimal"]
        )
        encoded = tuple(
            tuple(str(value) for value in row)
            for row in payload["global_control_grid_float64_hex"]
        )
        if tuple(tuple(value.hex() for value in row) for row in decimal) != encoded:
            raise ValueError("piecewise barrier decimal/hex grid mismatch")
        result = cls(
            schema_version=int(payload["schema_version"]),
            partition_id=str(payload["partition_id"]),
            degree=int(payload["degree_per_axis"]),
            control_grid=decimal,
        )
        if list(result.control_grid_shape) != payload.get(
            "global_control_grid_shape"
        ):
            raise ValueError("piecewise barrier grid-shape metadata mismatch")
        if result.dof_count != int(payload["global_control_dof_count"]):
            raise ValueError("piecewise barrier DOF metadata mismatch")
        continuity = payload.get("continuity", {})
        if (
            continuity.get("class") != "C0"
            or continuity.get("construction")
            != "shared_conforming_edge_control_dofs"
            or int(continuity.get("internal_edge_count", -1)) != 84
            or continuity.get("separate_float_equality_rows_required") is not False
        ):
            raise ValueError("piecewise barrier continuity metadata mismatch")
        return result


@dataclass(frozen=True, slots=True)
class LocalBernsteinRoof2D:
    """One polynomial upper roof for one source cell and N1 quadrant."""

    source_cell_id: str
    controller_piece_id: str
    quadrant: tuple[int, int]
    normalized_reach_box: tuple[Fraction, Fraction, Fraction, Fraction]
    degree: int
    control_grid: tuple[tuple[float, ...], ...]

    def __post_init__(self) -> None:
        source = P49_CELL_BY_ID.get(str(self.source_cell_id))
        if source is None or not source.safe_source:
            raise ValueError("roof source must be one of the 37 safe P49 cells")
        if self.controller_piece_id != source.controller_piece_id:
            raise ValueError("roof/controller-piece binding mismatch")
        if self.quadrant not in P49_QUADRANTS:
            raise ValueError("roof quadrant binding mismatch")
        expected_box = p49_quadrant_reach_box(source, self.quadrant)
        if tuple(self.normalized_reach_box) != expected_box:
            raise ValueError("roof reach-box binding mismatch")
        degree = int(self.degree)
        if degree < 1:
            raise ValueError("roof degree must be positive")
        grid = _validated_control_grid(
            self.control_grid,
            expected_size=degree + 1,
            name="roof",
        )
        object.__setattr__(self, "degree", degree)
        object.__setattr__(self, "control_grid", grid)
        object.__setattr__(self, "normalized_reach_box", expected_box)

    @property
    def roof_id(self) -> str:
        return f"R_{self.source_cell_id}_{p49_quadrant_id(self.quadrant)}"

    def rational_polynomial(self) -> RationalPolynomial:
        return local_bernstein_grid_rational_polynomial(
            degree=self.degree,
            control_grid=self.control_grid,
            box=self.normalized_reach_box,
        )

    def payload(self) -> dict[str, Any]:
        return {
            "roof_id": self.roof_id,
            "source_cell_id": self.source_cell_id,
            "controller_piece_id": self.controller_piece_id,
            "quadrant": list(self.quadrant),
            "quadrant_id": p49_quadrant_id(self.quadrant),
            "normalized_reach_box_exact": _box_payload(
                self.normalized_reach_box
            ),
            "basis": "local_tensor_bernstein",
            "degree_per_axis": self.degree,
            "control_grid_decimal": [list(row) for row in self.control_grid],
            "control_grid_float64_hex": [
                [value.hex() for value in row] for row in self.control_grid
            ],
        }

    @classmethod
    def from_payload(cls, payload: Mapping[str, Any]) -> "LocalBernsteinRoof2D":
        if payload.get("basis") != "local_tensor_bernstein":
            raise ValueError("roof basis mismatch")
        quadrant = tuple(int(value) for value in payload["quadrant"])
        if len(quadrant) != 2:
            raise ValueError("roof quadrant shape mismatch")
        decimal = tuple(
            tuple(float(value) for value in row)
            for row in payload["control_grid_decimal"]
        )
        encoded = tuple(
            tuple(str(value) for value in row)
            for row in payload["control_grid_float64_hex"]
        )
        if tuple(tuple(value.hex() for value in row) for row in decimal) != encoded:
            raise ValueError("roof decimal/hex grid mismatch")
        result = cls(
            source_cell_id=str(payload["source_cell_id"]),
            controller_piece_id=str(payload["controller_piece_id"]),
            quadrant=(quadrant[0], quadrant[1]),
            normalized_reach_box=tuple(
                _fraction_from_text(value)
                for value in payload["normalized_reach_box_exact"]
            ),
            degree=int(payload["degree_per_axis"]),
            control_grid=decimal,
        )
        if result.roof_id != payload.get("roof_id") or p49_quadrant_id(
            result.quadrant
        ) != payload.get("quadrant_id"):
            raise ValueError("roof identity metadata mismatch")
        return result


@dataclass(frozen=True, slots=True)
class PiecewiseRoofWitness2D:
    """Hash-bound collection of all 148 auxiliary roof polynomials."""

    barrier_payload_sha256: str
    controller_content_sha256: str
    formal_model_contract_sha256: str
    geometry_catalog_sha256: str
    barrier_degree: int
    roof_degree: int
    roofs: tuple[LocalBernsteinRoof2D, ...]
    schema_version: int = 1

    def __post_init__(self) -> None:
        if self.schema_version != 1:
            raise ValueError("unsupported P49 roof-witness schema")
        for field_name in (
            "barrier_payload_sha256",
            "controller_content_sha256",
            "formal_model_contract_sha256",
            "geometry_catalog_sha256",
        ):
            object.__setattr__(
                self,
                field_name,
                _validate_sha256(getattr(self, field_name), name=field_name),
            )
        if self.geometry_catalog_sha256 != p49_geometry_catalog_sha256():
            raise ValueError("roof witness geometry-catalog hash mismatch")
        degree_pair = (int(self.barrier_degree), int(self.roof_degree))
        if degree_pair not in ((3, 4), (4, 6)):
            raise ValueError("roof witness degree pair is outside the frozen schedule")
        expected_keys = tuple(
            (cell.cell_id, quadrant)
            for cell in P49_SAFE_SOURCE_CELLS
            for quadrant in P49_QUADRANTS
        )
        actual_keys = tuple(
            (roof.source_cell_id, roof.quadrant) for roof in self.roofs
        )
        if actual_keys != expected_keys:
            raise ValueError("roof witness does not contain the canonical 148 roofs")
        if any(roof.degree != int(self.roof_degree) for roof in self.roofs):
            raise ValueError("roof witness degree metadata mismatch")
        object.__setattr__(self, "barrier_degree", int(self.barrier_degree))
        object.__setattr__(self, "roof_degree", int(self.roof_degree))

    def payload_without_hash(self) -> dict[str, Any]:
        return {
            "schema_version": self.schema_version,
            "kind": "g11p49_piecewise_roof_witness",
            "proof_route_id": G11P49_PROOF_ROUTE_ID,
            "partition_id": P49_PARTITION_ID,
            "reach_catalog_id": G11P49_REACH_CATALOG_ID,
            "barrier_payload_sha256": self.barrier_payload_sha256,
            "controller_content_sha256": self.controller_content_sha256,
            "formal_model_contract_sha256": self.formal_model_contract_sha256,
            "geometry_catalog_sha256": self.geometry_catalog_sha256,
            "barrier_degree": self.barrier_degree,
            "roof_degree": self.roof_degree,
            "roof_count": len(self.roofs),
            "roofs": [roof.payload() for roof in self.roofs],
        }

    @property
    def sha256(self) -> str:
        return content_sha256(self.payload_without_hash())

    def payload(self) -> dict[str, Any]:
        payload = self.payload_without_hash()
        payload["content_sha256"] = self.sha256
        return payload

    def save(self, path: str | Path) -> Path:
        destination = Path(path)
        _write_json(destination, self.payload())
        return destination

    @classmethod
    def load(cls, path: str | Path) -> "PiecewiseRoofWitness2D":
        payload = json.loads(Path(path).read_text(encoding="utf-8"))
        declared_hash = str(payload.pop("content_sha256", "")).lower()
        if content_sha256(payload) != declared_hash:
            raise ValueError("roof witness content hash mismatch")
        if payload.get("kind") != "g11p49_piecewise_roof_witness":
            raise ValueError("roof witness kind mismatch")
        if payload.get("proof_route_id") != G11P49_PROOF_ROUTE_ID:
            raise ValueError("roof witness proof-route mismatch")
        if payload.get("partition_id") != P49_PARTITION_ID or payload.get(
            "reach_catalog_id"
        ) != G11P49_REACH_CATALOG_ID:
            raise ValueError("roof witness geometry identity mismatch")
        result = cls(
            schema_version=int(payload["schema_version"]),
            barrier_payload_sha256=str(payload["barrier_payload_sha256"]),
            controller_content_sha256=str(
                payload["controller_content_sha256"]
            ),
            formal_model_contract_sha256=str(
                payload["formal_model_contract_sha256"]
            ),
            geometry_catalog_sha256=str(payload["geometry_catalog_sha256"]),
            barrier_degree=int(payload["barrier_degree"]),
            roof_degree=int(payload["roof_degree"]),
            roofs=tuple(
                LocalBernsteinRoof2D.from_payload(item)
                for item in payload["roofs"]
            ),
        )
        if int(payload["roof_count"]) != len(result.roofs):
            raise ValueError("roof witness count metadata mismatch")
        if result.sha256 != declared_hash:
            raise ValueError("reconstructed roof witness hash mismatch")
        return result
