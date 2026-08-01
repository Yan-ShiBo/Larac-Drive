from __future__ import annotations

from math import isclose

from cbf_rover.methods.polynomial_sbc.algebra import evaluate
from cbf_rover.methods.polynomial_sbc.sos import (
    _box_affine_map,
    _normalize_guard,
    _numeric_affine_pullback,
    _unit_disk_guard,
    _unit_rectangle_guards,
)


def test_affine_pullback_preserves_polynomial_values() -> None:
    polynomial = {
        (0, 0): 0.3,
        (1, 0): -0.7,
        (0, 2): 1.1,
        (2, 1): -0.4,
    }
    affine = {
        "x_offset": -0.31,
        "x_scale": 0.07,
        "y_offset": 0.42,
        "y_scale": 0.13,
    }
    transformed = _numeric_affine_pullback(polynomial, **affine)
    local_point = (0.6, -0.8)
    global_point = (
        affine["x_offset"] + affine["x_scale"] * local_point[0],
        affine["y_offset"] + affine["y_scale"] * local_point[1],
    )
    assert isclose(
        evaluate(transformed, local_point),
        evaluate(polynomial, global_point),
        rel_tol=1.0e-14,
        abs_tol=1.0e-14,
    )


def test_box_map_and_unit_guards_have_fixed_numerical_scale() -> None:
    actual = _box_affine_map((-0.6, -0.56, -0.2, 0.4))
    expected = (-0.58, 0.02, 0.1, 0.3)
    assert all(
        isclose(actual_value, expected_value)
        for actual_value, expected_value in zip(actual, expected)
    )
    x_guard, y_guard = _unit_rectangle_guards()
    assert evaluate(x_guard, (-1.0, 0.0)) == 0.0
    assert evaluate(y_guard, (0.0, 1.0)) == 0.0
    assert isclose(
        evaluate(_unit_disk_guard(), (0.6, 0.8)),
        0.0,
        abs_tol=1.0e-15,
    )


def test_positive_guard_normalization_preserves_sign() -> None:
    guard = {(0, 0): -12.0, (1, 0): 4.0, (0, 2): 20.0}
    normalized = _normalize_guard(guard)
    for point in ((-1.0, -1.0), (0.0, 0.0), (0.3, 0.9)):
        original = evaluate(guard, point)
        scaled = evaluate(normalized, point)
        assert isclose(scaled, original / 20.0)
