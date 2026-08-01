from __future__ import annotations

import numpy as np

from cbf_rover.methods.polynomial_sbc.bernstein import (
    exact_bernstein_bounds,
)
from cbf_rover.methods.polynomial_sbc.bernstein_lp import (
    _goal_exterior_cover,
    _polynomial_family_bernstein_matrix,
)


def test_float_bernstein_matrix_matches_exact_numeric_bounds() -> None:
    polynomial = {
        (0, 0): 0.3,
        (1, 0): -0.4,
        (0, 2): 0.7,
        (2, 1): -0.2,
    }
    box = (-0.7, 0.2, -0.4, 0.9)
    coefficients = _polynomial_family_bernstein_matrix(
        (polynomial,),
        box,
    )[:, 0]
    exact_lower, exact_upper = exact_bernstein_bounds(polynomial, box)
    assert np.min(coefficients) >= exact_lower
    assert np.max(coefficients) <= exact_upper
    assert np.isclose(np.min(coefficients), exact_lower)
    assert np.isclose(np.max(coefficients), exact_upper)


def test_goal_exterior_cover_skips_the_goal_center() -> None:
    cover = _goal_exterior_cover(
        (0.6, 1.0, -0.4, 0.4),
        depth=0,
        maximum_depth=12,
    )
    assert cover
    assert all(
        not (
            box[0] <= 0.8 <= box[1]
            and box[2] <= 0.0 <= box[3]
        )
        for box in cover
    )
