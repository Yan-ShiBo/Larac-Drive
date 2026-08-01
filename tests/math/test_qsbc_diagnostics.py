from __future__ import annotations

import numpy as np

from cbf_rover.methods.polynomial_sbc.qsbc_diagnostics import (
    _design_matrix,
    _nonterminal_mask,
)


def test_diagnostic_design_matrix_uses_declared_power_order() -> None:
    points = np.asarray([[2.0, 3.0], [-1.0, 0.5]])
    matrix = _design_matrix(points, ((0, 0), (1, 0), (0, 2), (1, 1)))
    assert np.array_equal(
        matrix,
        np.asarray(
            [
                [1.0, 2.0, 9.0, 6.0],
                [1.0, -1.0, 0.25, -0.5],
            ]
        ),
    )


def test_nonterminal_mask_excludes_goal_and_each_unsafe_component() -> None:
    points = np.asarray(
        [
            [0.0, 0.0],
            [3.5, 0.0],
            [1.5, 0.0],
            [2.5, 0.8],
            [2.5, -0.8],
            [3.5, 1.0],
        ]
    )
    assert _nonterminal_mask(points).tolist() == [
        True,
        False,
        False,
        False,
        False,
        True,
    ]
