from __future__ import annotations

from fractions import Fraction

from cbf_rover.methods.polynomial_sbc.polynomial import (
    FrozenPolynomialController,
    SparsePolynomial2D,
    controller_pieces,
)
from cbf_rover.methods.polynomial_sbc.rational_certificate_validation import (
    NOISE_HALF_WIDTH,
    NORMALIZATION_SCALE_X,
    NORMALIZATION_SCALE_Y,
    RationalBernsteinValidationOptions,
    barrier_payload_sha256,
    classify_full_box,
    classify_goal_exterior_box,
    classify_included_disk_box,
    exact_geometry_gate,
    exact_expected_next_polynomial_rational,
    first_slice_q_gate,
    normalize_physical_box,
    validate_rational_polynomial_nonnegative,
)


def _zero_controller() -> FrozenPolynomialController:
    zero = SparsePolynomial2D(((0, 0),), (0.0,))
    return FrozenPolynomialController(
        controller_id="v3_zero_controller",
        u_x=zero,
        u_y=zero,
        upstream_checkpoint_sha256="0" * 64,
    )


def test_v3_does_not_validate_a_negative_five_e_minus_eight_constant() -> None:
    result = validate_rational_polynomial_nonnegative(
        {(0, 0): Fraction(-1, 20_000_000)},
        (Fraction(-1), Fraction(1), Fraction(-1), Fraction(1)),
        classify_domain=classify_full_box,
        options=RationalBernsteinValidationOptions(max_depth=0),
    )

    assert result["status"] == "violated"
    assert result["acceptance_tolerance"] == 0.0


def test_v3_tangent_disk_boxes_are_never_pruned_as_outside() -> None:
    initial_tangent = normalize_physical_box(
        (
            Fraction("0.1"),
            Fraction("0.2"),
            Fraction("0.0"),
            Fraction("0.1"),
        )
    )
    goal_tangent = normalize_physical_box(
        (
            Fraction("3.75"),
            Fraction("3.8"),
            Fraction("0.0"),
            Fraction("0.1"),
        )
    )

    assert classify_included_disk_box(initial_tangent) == "boundary"
    assert classify_goal_exterior_box(goal_tangent) == "boundary"


def test_v3_exact_n1_expectation_for_zero_controller_quadratic() -> None:
    barrier = SparsePolynomial2D(
        ((2, 0), (0, 2)),
        (1.0, 1.0),
    )
    piece = controller_pieces(_zero_controller())[0]
    expected = exact_expected_next_polynomial_rational(barrier, piece)
    expected_constant = (
        (NOISE_HALF_WIDTH / NORMALIZATION_SCALE_X) ** 2 / 6
        + (NOISE_HALF_WIDTH / NORMALIZATION_SCALE_Y) ** 2 / 6
    )

    assert expected[(2, 0)] == 1
    assert expected[(0, 2)] == 1
    assert expected[(0, 0)] == expected_constant


def test_v3_binary64_point_one_does_not_pass_exact_one_tenth_gate() -> None:
    assert Fraction.from_float(0.1) > Fraction(1, 10)
    assert not first_slice_q_gate(0.1)


def test_v3_barrier_hash_binds_binary64_coefficients() -> None:
    left = SparsePolynomial2D(((0, 0), (1, 0)), (1.0, 2.0))
    right = SparsePolynomial2D(((0, 0), (1, 0)), (1.0, 2.0 + 2.0**-50))

    assert barrier_payload_sha256(left) != barrier_payload_sha256(right)


def test_v3_canonical_geometry_gate_is_exactly_validated() -> None:
    result = exact_geometry_gate()

    assert result["status"] == "validated"
    assert all(result["checks"].values())
