from __future__ import annotations

import numpy as np

from cbf_rover.methods.polynomial_sbc.pac import (
    _fit_axis_linf,
    _fit_joint_linf,
)


def test_highs_axis_linf_recovers_a_polynomial() -> None:
    rng = np.random.default_rng(5)
    points = rng.uniform(-1.0, 1.0, size=(100, 2))
    support = ((0, 0), (1, 0), (0, 1))
    targets = 0.1 + 0.2 * points[:, 0] - 0.3 * points[:, 1]
    result = _fit_axis_linf(points, targets, support)
    assert result is not None
    assert result.search_linf <= 1.0e-9
    np.testing.assert_allclose(result.coefficients, (0.1, 0.2, -0.3))


def test_highs_joint_linf_uses_one_shared_radius() -> None:
    rng = np.random.default_rng(7)
    points = rng.uniform(-1.0, 1.0, size=(100, 2))
    support_x = ((0, 0), (1, 0))
    support_y = ((0, 0), (0, 1))
    targets = np.column_stack(
        (
            0.2 + 0.1 * points[:, 0],
            -0.1 + 0.3 * points[:, 1],
        )
    )
    coefficients_x, coefficients_y, radius, _ = _fit_joint_linf(
        points,
        targets,
        support_x,
        support_y,
    )
    assert radius <= 1.0e-9
    np.testing.assert_allclose(coefficients_x, (0.2, 0.1))
    np.testing.assert_allclose(coefficients_y, (-0.1, 0.3))
