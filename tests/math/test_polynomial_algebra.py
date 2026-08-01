from __future__ import annotations

from math import isclose

from cbf_rover.domain.planar import CANONICAL_SYSTEM
from cbf_rover.methods.polynomial_sbc.algebra import (
    evaluate,
    exact_expected_next_polynomial,
)
from cbf_rover.methods.polynomial_sbc.polynomial import (
    FrozenPiecewisePolynomialController,
    FrozenPolynomialController,
    PolynomialControllerPiece,
    SparsePolynomial2D,
    canonical_total_degree_powers,
)


def _zero_controller() -> FrozenPolynomialController:
    zero = SparsePolynomial2D(((0, 0),), (0.0,))
    return FrozenPolynomialController(
        controller_id="test_zero",
        u_x=zero,
        u_y=zero,
        upstream_checkpoint_sha256="0" * 64,
    )


def test_pb1_catalog_has_explicit_canonical_order() -> None:
    assert canonical_total_degree_powers(2) == (
        (0, 0),
        (0, 1),
        (1, 0),
        (0, 2),
        (1, 1),
        (2, 0),
    )


def test_n1_expectation_uses_exact_normalized_moments() -> None:
    barrier = SparsePolynomial2D(((2, 0), (0, 2)), (1.0, 1.0))
    expected = exact_expected_next_polynomial(
        barrier,
        _zero_controller(),
        CANONICAL_SYSTEM,
    )
    point = (0.2, -0.3)
    expected_value = (
        point[0] ** 2
        + point[1] ** 2
        + (0.005 / 2.5) ** 2 / 6.0
        + (0.005 / 2.0) ** 2 / 6.0
    )
    assert isclose(evaluate(expected, point), expected_value, rel_tol=1.0e-14)


def test_polynomial_evaluation_matches_manual_value() -> None:
    polynomial = SparsePolynomial2D(
        ((0, 0), (1, 0), (0, 2), (2, 1)),
        (1.0, 2.0, -3.0, 4.0),
    )
    assert polynomial.degree == 3
    assert polynomial.evaluate(0.5, -0.25) == (
        1.0 + 2.0 * 0.5 - 3.0 * (-0.25) ** 2 + 4.0 * 0.5**2 * -0.25
    )


def test_piecewise_expectation_requires_and_uses_the_explicit_piece() -> None:
    zero = SparsePolynomial2D(((0, 0),), (0.0,))
    left = PolynomialControllerPiece(
        "left",
        -1.0,
        0.0,
        SparsePolynomial2D(((0, 0),), (0.1,)),
        zero,
    )
    right = PolynomialControllerPiece(
        "right",
        0.0,
        4.0,
        SparsePolynomial2D(((0, 0),), (0.2,)),
        zero,
    )
    controller = FrozenPiecewisePolynomialController(
        controller_id="piecewise_expectation",
        pieces=(left, right),
        construction_source_sha256="1" * 64,
    )
    barrier = SparsePolynomial2D(((1, 0),), (1.0,))
    left_expected = exact_expected_next_polynomial(
        barrier,
        controller,
        CANONICAL_SYSTEM,
        piece=left,
    )
    right_expected = exact_expected_next_polynomial(
        barrier,
        controller,
        CANONICAL_SYSTEM,
        piece=right,
    )
    assert isclose(evaluate(left_expected, (0.0, 0.0)), 0.002)
    assert isclose(evaluate(right_expected, (0.0, 0.0)), 0.004)
