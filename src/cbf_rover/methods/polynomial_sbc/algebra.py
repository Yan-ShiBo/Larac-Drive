"""Small deterministic polynomial algebra used by synthesis and replay."""

from __future__ import annotations

from math import comb, isfinite
from typing import Iterable, Mapping

from cbf_rover.domain.planar import (
    StaticObstacleScenario,
    StochasticSingleIntegrator2D,
)

from .polynomial import (
    FrozenController,
    NumericPolynomial,
    PolynomialControllerPiece,
    Power2D,
    SparsePolynomial2D,
    controller_pieces,
)


ZERO_POWER: Power2D = (0, 0)
X_VARIABLE: NumericPolynomial = {(1, 0): 1.0}
Y_VARIABLE: NumericPolynomial = {(0, 1): 1.0}


def clean(polynomial: Mapping[Power2D, float], tolerance: float = 0.0) -> NumericPolynomial:
    if tolerance < 0.0:
        raise ValueError("tolerance must be nonnegative")
    result: NumericPolynomial = {}
    for raw_power, raw_coefficient in polynomial.items():
        power = tuple(int(value) for value in raw_power)
        if len(power) != 2 or any(value < 0 for value in power):
            raise ValueError("invalid bivariate polynomial exponent")
        coefficient = float(raw_coefficient)
        if not isfinite(coefficient):
            raise ValueError("polynomial coefficient must be finite")
        if abs(coefficient) > tolerance:
            result[power] = result.get(power, 0.0) + coefficient
    return {
        power: coefficient
        for power, coefficient in result.items()
        if abs(coefficient) > tolerance
    }


def constant(value: float) -> NumericPolynomial:
    coefficient = float(value)
    if not isfinite(coefficient):
        raise ValueError("constant must be finite")
    return {} if coefficient == 0.0 else {ZERO_POWER: coefficient}


def add(*polynomials: Mapping[Power2D, float]) -> NumericPolynomial:
    result: NumericPolynomial = {}
    for polynomial in polynomials:
        for power, coefficient in polynomial.items():
            result[power] = result.get(power, 0.0) + float(coefficient)
    return clean(result)


def scale(polynomial: Mapping[Power2D, float], factor: float) -> NumericPolynomial:
    factor_value = float(factor)
    if not isfinite(factor_value):
        raise ValueError("scale must be finite")
    return clean(
        {power: factor_value * float(coefficient) for power, coefficient in polynomial.items()}
    )


def negate(polynomial: Mapping[Power2D, float]) -> NumericPolynomial:
    return scale(polynomial, -1.0)


def subtract(
    left: Mapping[Power2D, float],
    right: Mapping[Power2D, float],
) -> NumericPolynomial:
    return add(left, negate(right))


def multiply(
    left: Mapping[Power2D, float],
    right: Mapping[Power2D, float],
) -> NumericPolynomial:
    result: NumericPolynomial = {}
    for (left_x, left_y), left_coefficient in left.items():
        for (right_x, right_y), right_coefficient in right.items():
            power = (left_x + right_x, left_y + right_y)
            result[power] = (
                result.get(power, 0.0)
                + float(left_coefficient) * float(right_coefficient)
            )
    return clean(result)


def power(polynomial: Mapping[Power2D, float], exponent: int) -> NumericPolynomial:
    if isinstance(exponent, bool) or not isinstance(exponent, int) or exponent < 0:
        raise ValueError("exponent must be a nonnegative integer")
    result = constant(1.0)
    factor = clean(polynomial)
    remaining = exponent
    while remaining:
        if remaining & 1:
            result = multiply(result, factor)
        remaining >>= 1
        if remaining:
            factor = multiply(factor, factor)
    return result


def degree(polynomial: Mapping[Power2D, float]) -> int:
    return max((sum(power_value) for power_value in polynomial), default=0)


def evaluate(
    polynomial: Mapping[Power2D, float],
    point: tuple[float, float],
) -> float:
    x_value, y_value = (float(value) for value in point)
    return float(
        sum(
            float(coefficient) * x_value**x_power * y_value**y_power
            for (x_power, y_power), coefficient in polynomial.items()
        )
    )


def sparse_to_numeric(polynomial: SparsePolynomial2D) -> NumericPolynomial:
    return polynomial.as_numeric_dict()


def _selected_piece(
    controller: FrozenController,
    piece: PolynomialControllerPiece | None,
) -> PolynomialControllerPiece:
    if piece is not None:
        if piece not in controller_pieces(controller):
            raise ValueError("controller piece does not belong to controller")
        return piece
    pieces = controller_pieces(controller)
    if len(pieces) != 1:
        raise ValueError("piecewise controller requires an explicit piece")
    return pieces[0]


def normalized_deterministic_dynamics(
    controller: FrozenController,
    system: StochasticSingleIntegrator2D,
    *,
    piece: PolynomialControllerPiece | None = None,
) -> tuple[NumericPolynomial, NumericPolynomial]:
    """Return the noise-free normalized successor map."""

    normalization = controller.normalization
    selected = _selected_piece(controller, piece)
    mu_x = add(
        X_VARIABLE,
        scale(
            sparse_to_numeric(selected.u_x),
            system.dt_s / normalization.scale_x_m,
        ),
    )
    mu_y = add(
        Y_VARIABLE,
        scale(
            sparse_to_numeric(selected.u_y),
            system.dt_s / normalization.scale_y_m,
        ),
    )
    return mu_x, mu_y


def _expected_univariate_power(
    mean: Mapping[Power2D, float],
    exponent: int,
    *,
    physical_noise_half_width: float,
    coordinate_scale: float,
) -> NumericPolynomial:
    result: NumericPolynomial = {}
    normalized_half_width = physical_noise_half_width / coordinate_scale
    for noise_order in range(exponent + 1):
        if noise_order % 2:
            continue
        if noise_order == 0:
            moment = 1.0
        else:
            moment = (
                2.0
                * normalized_half_width**noise_order
                / ((noise_order + 1) * (noise_order + 2))
            )
        result = add(
            result,
            scale(
                power(mean, exponent - noise_order),
                comb(exponent, noise_order) * moment,
            ),
        )
    return result


def exact_expected_next_polynomial(
    barrier: SparsePolynomial2D,
    controller: FrozenController,
    system: StochasticSingleIntegrator2D,
    *,
    piece: PolynomialControllerPiece | None = None,
) -> NumericPolynomial:
    """Compute ``E[W(xi[k+1])]`` with the exact N1 moment family."""

    mu_x, mu_y = normalized_deterministic_dynamics(
        controller,
        system,
        piece=piece,
    )
    max_x = max(power_value[0] for power_value in barrier.powers)
    max_y = max(power_value[1] for power_value in barrier.powers)
    x_table = [
        _expected_univariate_power(
            mu_x,
            exponent,
            physical_noise_half_width=system.noise.half_width_m,
            coordinate_scale=controller.normalization.scale_x_m,
        )
        for exponent in range(max_x + 1)
    ]
    y_table = [
        _expected_univariate_power(
            mu_y,
            exponent,
            physical_noise_half_width=system.noise.half_width_m,
            coordinate_scale=controller.normalization.scale_y_m,
        )
        for exponent in range(max_y + 1)
    ]
    result: NumericPolynomial = {}
    for (x_power, y_power), coefficient in zip(
        barrier.powers,
        barrier.coefficients,
    ):
        result = add(
            result,
            scale(
                multiply(x_table[x_power], y_table[y_power]),
                coefficient,
            ),
        )
    return result


def action_disk_residual(
    controller: FrozenController,
    system: StochasticSingleIntegrator2D,
    *,
    piece: PolynomialControllerPiece | None = None,
) -> NumericPolynomial:
    selected = _selected_piece(controller, piece)
    u_x = sparse_to_numeric(selected.u_x)
    u_y = sparse_to_numeric(selected.u_y)
    return subtract(
        constant(system.max_speed_mps**2),
        add(multiply(u_x, u_x), multiply(u_y, u_y)),
    )


def robust_domain_residuals(
    controller: FrozenController,
    system: StochasticSingleIntegrator2D,
    scenario: StaticObstacleScenario,
    *,
    piece: PolynomialControllerPiece | None = None,
) -> dict[str, NumericPolynomial]:
    """D1 margins after analytically eliminating all bounded N1 samples."""

    normalization = controller.normalization
    selected = _selected_piece(controller, piece)
    physical_x = add(
        constant(normalization.center_x_m),
        scale(X_VARIABLE, normalization.scale_x_m),
    )
    physical_y = add(
        constant(normalization.center_y_m),
        scale(Y_VARIABLE, normalization.scale_y_m),
    )
    deterministic_x = add(
        physical_x,
        scale(sparse_to_numeric(selected.u_x), system.dt_s),
    )
    deterministic_y = add(
        physical_y,
        scale(sparse_to_numeric(selected.u_y), system.dt_s),
    )
    noise = system.noise.half_width_m
    return {
        "successor_x_lower": subtract(
            deterministic_x,
            constant(scenario.domain.x_min + noise),
        ),
        "successor_x_upper": subtract(
            constant(scenario.domain.x_max - noise),
            deterministic_x,
        ),
        "successor_y_lower": subtract(
            deterministic_y,
            constant(scenario.domain.y_min + noise),
        ),
        "successor_y_upper": subtract(
            constant(scenario.domain.y_max - noise),
            deterministic_y,
        ),
    }


def iter_coefficients(
    polynomial: Mapping[Power2D, float],
) -> Iterable[tuple[Power2D, float]]:
    return sorted(polynomial.items(), key=lambda item: (sum(item[0]), item[0][0]))
