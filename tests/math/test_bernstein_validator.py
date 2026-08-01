from __future__ import annotations

from cbf_rover.methods.polynomial_sbc.bernstein import (
    BernsteinValidationOptions,
    classify_full_box,
    exact_bernstein_bounds,
    validate_polynomial_nonnegative,
)


def test_exact_float_bernstein_bounds_enclose_quadratic_range() -> None:
    polynomial = {(0, 0): 1.0, (2, 0): -1.0, (0, 2): -1.0}
    lower, upper = exact_bernstein_bounds(
        polynomial,
        (-0.5, 0.5, -0.5, 0.5),
    )
    assert lower <= 0.5
    assert upper >= 1.0


def test_branch_and_bound_validates_positive_polynomial() -> None:
    result = validate_polynomial_nonnegative(
        {(0, 0): 0.1, (2, 0): 1.0, (0, 2): 1.0},
        (-1.0, 1.0, -1.0, 1.0),
        classify_domain=classify_full_box,
        options=BernsteinValidationOptions(max_depth=8),
    )
    assert result["status"] == "validated"


def test_branch_and_bound_proves_a_violation() -> None:
    result = validate_polynomial_nonnegative(
        {(0, 0): -0.1},
        (-1.0, 1.0, -1.0, 1.0),
        classify_domain=classify_full_box,
        options=BernsteinValidationOptions(max_depth=1),
    )
    assert result["status"] == "violated"
