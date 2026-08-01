"""Deterministic total-degree Chebyshev basis construction for G10.

The synthesis variables multiply ``T_i(xi_x) T_j(xi_y)`` with ``i+j <= d``.
The final certificate remains a canonical monomial :class:`SparsePolynomial2D`
so the independent validator does not need to understand the solver basis.
"""

from __future__ import annotations

from math import isfinite
from typing import Sequence

from .algebra import (
    X_VARIABLE,
    Y_VARIABLE,
    add,
    constant,
    multiply,
    scale,
)
from .polynomial import (
    NumericPolynomial,
    Power2D,
    SparsePolynomial2D,
    canonical_total_degree_powers,
)


def _axis_chebyshev_polynomials(
    degree: int,
    variable: NumericPolynomial,
) -> tuple[NumericPolynomial, ...]:
    if isinstance(degree, bool) or not isinstance(degree, int) or degree < 0:
        raise ValueError("degree must be a nonnegative integer")
    result: list[NumericPolynomial] = [constant(1.0)]
    if degree == 0:
        return tuple(result)
    result.append(dict(variable))
    for order in range(2, degree + 1):
        result.append(
            add(
                scale(multiply(variable, result[order - 1]), 2.0),
                scale(result[order - 2], -1.0),
            )
        )
    return tuple(result)


def total_degree_chebyshev_basis(
    degree: int,
) -> tuple[tuple[Power2D, ...], tuple[NumericPolynomial, ...]]:
    """Return ordered Chebyshev indices and their monomial expansions."""

    indices = canonical_total_degree_powers(degree)
    x_basis = _axis_chebyshev_polynomials(degree, X_VARIABLE)
    y_basis = _axis_chebyshev_polynomials(degree, Y_VARIABLE)
    return indices, tuple(
        multiply(x_basis[x_order], y_basis[y_order])
        for x_order, y_order in indices
    )


def chebyshev_coefficients_to_monomial(
    *,
    indices: Sequence[Power2D],
    coefficients: Sequence[float],
    basis: Sequence[NumericPolynomial] | None = None,
) -> SparsePolynomial2D:
    """Convert final binary64 Chebyshev coefficients to a monomial artifact."""

    normalized_indices = tuple(
        (int(index[0]), int(index[1])) for index in indices
    )
    values = tuple(float(value) for value in coefficients)
    if len(normalized_indices) != len(values) or not normalized_indices:
        raise ValueError("Chebyshev indices and coefficients must be nonempty and aligned")
    if any(value < 0 for index in normalized_indices for value in index):
        raise ValueError("Chebyshev indices must be nonnegative")
    if not all(isfinite(value) for value in values):
        raise ValueError("Chebyshev coefficients must be finite")
    if basis is None:
        degree = max(sum(index) for index in normalized_indices)
        canonical_indices, expanded_basis = total_degree_chebyshev_basis(degree)
        if normalized_indices != canonical_indices:
            raise ValueError("indices must use the canonical total-degree catalog")
    else:
        expanded_basis = tuple(basis)
        if len(expanded_basis) != len(normalized_indices):
            raise ValueError("explicit Chebyshev basis is not aligned")

    polynomial: NumericPolynomial = {}
    for coefficient, basis_polynomial in zip(values, expanded_basis):
        polynomial = add(polynomial, scale(basis_polynomial, coefficient))
    if not polynomial:
        polynomial = constant(0.0)
        if not polynomial:
            polynomial = {(0, 0): 0.0}
    powers = tuple(
        sorted(
            polynomial,
            key=lambda power: (sum(power), power[0], power[1]),
        )
    )
    return SparsePolynomial2D(
        powers=powers,
        coefficients=tuple(polynomial[power] for power in powers),
    )
