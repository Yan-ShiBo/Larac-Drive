"""Exact-rational replay for the canonical Rover q-SBC contract.

This module is intentionally independent of every solver representation.  The
controller and barrier coefficients are interpreted as their exact binary64
values, while the confirmed M1/P1/D1/N1 constants use their exact decimal
contract values.  Polynomial construction, N1 moments, semialgebraic domain
classification, and Bernstein bounds remain :class:`fractions.Fraction`
objects until a human-readable report is emitted.
"""

from __future__ import annotations

from dataclasses import dataclass, replace
from fractions import Fraction
import hashlib
import json
from math import comb
from pathlib import Path
from typing import Any, Callable, Literal, Mapping

from cbf_rover.domain.planar import CANONICAL_SCENARIO, CANONICAL_SYSTEM

from .polynomial import (
    FrozenController,
    PolynomialControllerPiece,
    Power2D,
    SparsePolynomial2D,
    content_sha256,
    controller_pieces,
)


INDEPENDENT_RATIONAL_VALIDATOR_ID = (
    "G5_EXACT_RATIONAL_BERNSTEIN_REPLAY_V3"
)
FIRST_SLICE_Q_MAX = Fraction(1, 10)
CONTROLLER_REQUIRED_MARGIN = Fraction(1, 1_000_000)

RationalPolynomial = dict[Power2D, Fraction]
RationalBox = tuple[Fraction, Fraction, Fraction, Fraction]
DomainClass = Literal["outside", "boundary", "inside"]
DomainClassifier = Callable[[RationalBox], DomainClass]


def _decimal(value: str) -> Fraction:
    return Fraction(value)


DT = _decimal("0.05")
MAX_SPEED = _decimal("0.5")
NOISE_HALF_WIDTH = _decimal("0.005")
NORMALIZATION_CENTER_X = _decimal("1.5")
NORMALIZATION_SCALE_X = _decimal("2.5")
NORMALIZATION_CENTER_Y = Fraction(0)
NORMALIZATION_SCALE_Y = _decimal("2.0")


@dataclass(frozen=True, slots=True)
class RationalDisk:
    """A closed physical disk with exact-rational parameters."""

    name: str
    center_x: Fraction
    center_y: Fraction
    radius: Fraction


DOMAIN_PHYSICAL: RationalBox = (
    _decimal("-1.0"),
    _decimal("4.0"),
    _decimal("-2.0"),
    _decimal("2.0"),
)
INITIAL_DISK = RationalDisk(
    "X0",
    Fraction(0),
    Fraction(0),
    _decimal("0.10"),
)
GOAL_DISK = RationalDisk(
    "Xg",
    _decimal("3.5"),
    Fraction(0),
    _decimal("0.25"),
)
UNSAFE_PHYSICAL: tuple[tuple[str, RationalBox], ...] = (
    (
        "Xu1",
        (
            _decimal("0.965"),
            _decimal("2.035"),
            _decimal("-0.535"),
            _decimal("0.535"),
        ),
    ),
    (
        "Xu2",
        (
            _decimal("1.915"),
            _decimal("3.085"),
            _decimal("0.215"),
            _decimal("1.385"),
        ),
    ),
    (
        "Xu3",
        (
            _decimal("1.915"),
            _decimal("3.085"),
            _decimal("-1.385"),
            _decimal("-0.215"),
        ),
    ),
)
BASE_PHYSICAL_X_BREAKS = tuple(
    _decimal(value)
    for value in ("-1.0", "0.965", "1.915", "2.035", "3.085", "4.0")
)
BASE_PHYSICAL_Y_BREAKS = tuple(
    _decimal(value)
    for value in (
        "-2.0",
        "-1.385",
        "-0.535",
        "-0.215",
        "0.215",
        "0.535",
        "1.385",
        "2.0",
    )
)


def canonical_rational_model_payload() -> dict[str, Any]:
    """Return the canonical exact-decimal M1/P1/D1/N1 identity."""

    return {
        "schema_version": 1,
        "kind": "formal_model_contract",
        "contract_id": "M1_S1_P1_D1_N1_EXACT_RATIONAL_V1",
        "state_order": ["x_m", "y_m"],
        "action_order": ["u_x_mps", "u_y_mps"],
        "frame_id": "world",
        "dt": "1/20",
        "maximum_speed": "1/2",
        "noise": {
            "distribution_id": "N1_IID_SYMMETRIC_TRIANGULAR_5MM",
            "axes_independent": True,
            "time_iid": True,
            "half_width": "1/200",
        },
        "domain": ["-1", "4", "-2", "2"],
        "initial_disk": ["0", "0", "1/10"],
        "goal_disk": ["7/2", "0", "1/4"],
        "unsafe_boxes": [
            {
                "name": name,
                "bounds": [_fraction_text(value) for value in box],
            }
            for name, box in UNSAFE_PHYSICAL
        ],
        "normalization": {
            "xi_x": ["3/2", "5/2"],
            "xi_y": ["0", "2"],
        },
        "numeric_semantics": {
            "model_constants": "exact_confirmed_decimal_rationals",
            "artifact_coefficients": "exact_binary64_rationals",
        },
    }


def canonical_rational_model_sha256() -> str:
    return content_sha256(canonical_rational_model_payload())


def _fraction_text(value: Fraction) -> str:
    if value.denominator == 1:
        return str(value.numerator)
    return f"{value.numerator}/{value.denominator}"


def _fraction_from_artifact(value: float) -> Fraction:
    """Preserve the exact binary64 identity stored by an artifact."""

    return Fraction.from_float(float(value))


def _fraction_from_contract_float(value: float) -> Fraction:
    """Recover the human-confirmed decimal represented by a runtime float."""

    return Fraction(str(float(value)))


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


def _file_sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _clean(polynomial: Mapping[Power2D, Fraction]) -> RationalPolynomial:
    return {
        (int(power[0]), int(power[1])): Fraction(coefficient)
        for power, coefficient in polynomial.items()
        if coefficient != 0
    }


def _constant(value: Fraction | int) -> RationalPolynomial:
    coefficient = Fraction(value)
    return {} if coefficient == 0 else {(0, 0): coefficient}


def _add(*polynomials: Mapping[Power2D, Fraction]) -> RationalPolynomial:
    result: RationalPolynomial = {}
    for polynomial in polynomials:
        for power, coefficient in polynomial.items():
            result[power] = result.get(power, Fraction(0)) + coefficient
    return _clean(result)


def _scale(
    polynomial: Mapping[Power2D, Fraction],
    factor: Fraction | int,
) -> RationalPolynomial:
    multiplier = Fraction(factor)
    return _clean(
        {power: coefficient * multiplier for power, coefficient in polynomial.items()}
    )


def _negate(polynomial: Mapping[Power2D, Fraction]) -> RationalPolynomial:
    return _scale(polynomial, -1)


def _multiply(
    left: Mapping[Power2D, Fraction],
    right: Mapping[Power2D, Fraction],
) -> RationalPolynomial:
    result: RationalPolynomial = {}
    for (left_x, left_y), left_coefficient in left.items():
        for (right_x, right_y), right_coefficient in right.items():
            power = (left_x + right_x, left_y + right_y)
            result[power] = (
                result.get(power, Fraction(0))
                + left_coefficient * right_coefficient
            )
    return _clean(result)


def _power(
    polynomial: Mapping[Power2D, Fraction],
    exponent: int,
) -> RationalPolynomial:
    if exponent < 0:
        raise ValueError("polynomial exponent must be nonnegative")
    result = _constant(1)
    factor = _clean(polynomial)
    remaining = exponent
    while remaining:
        if remaining & 1:
            result = _multiply(result, factor)
        remaining >>= 1
        if remaining:
            factor = _multiply(factor, factor)
    return result


def rational_sparse_polynomial(
    polynomial: SparsePolynomial2D,
) -> RationalPolynomial:
    """Convert final binary64 coefficients to exact rationals."""

    return _clean(
        {
            power: _fraction_from_artifact(coefficient)
            for power, coefficient in zip(
                polynomial.powers,
                polynomial.coefficients,
            )
        }
    )


def _assert_canonical_runtime_contract(controller: FrozenController) -> None:
    """Fail closed if executable constants drift from the V3 contract."""

    system_values = (
        _fraction_from_contract_float(CANONICAL_SYSTEM.dt_s),
        _fraction_from_contract_float(CANONICAL_SYSTEM.max_speed_mps),
        _fraction_from_contract_float(CANONICAL_SYSTEM.noise.half_width_m),
    )
    if system_values != (DT, MAX_SPEED, NOISE_HALF_WIDTH):
        raise ValueError("runtime M1/S1/N1 constants disagree with V3")

    scenario = CANONICAL_SCENARIO
    domain = tuple(
        _fraction_from_contract_float(value)
        for value in (
            scenario.domain.x_min,
            scenario.domain.x_max,
            scenario.domain.y_min,
            scenario.domain.y_max,
        )
    )
    if domain != DOMAIN_PHYSICAL:
        raise ValueError("runtime D1 domain disagrees with V3")
    disks = (
        tuple(
            _fraction_from_contract_float(value)
            for value in (
                scenario.initial.center_x,
                scenario.initial.center_y,
                scenario.initial.radius,
            )
        ),
        tuple(
            _fraction_from_contract_float(value)
            for value in (
                scenario.goal.center_x,
                scenario.goal.center_y,
                scenario.goal.radius,
            )
        ),
    )
    if disks != (
        (INITIAL_DISK.center_x, INITIAL_DISK.center_y, INITIAL_DISK.radius),
        (GOAL_DISK.center_x, GOAL_DISK.center_y, GOAL_DISK.radius),
    ):
        raise ValueError("runtime P1 disks disagree with V3")
    unsafe = tuple(
        (
            region.name,
            tuple(
                _fraction_from_contract_float(value)
                for value in (
                    region.x_min,
                    region.x_max,
                    region.y_min,
                    region.y_max,
                )
            ),
        )
        for region in scenario.unsafe_components
    )
    if unsafe != UNSAFE_PHYSICAL:
        raise ValueError("runtime P1 unsafe boxes disagree with V3")

    normalization = controller.normalization
    controller_normalization = tuple(
        _fraction_from_contract_float(value)
        for value in (
            normalization.center_x_m,
            normalization.scale_x_m,
            normalization.center_y_m,
            normalization.scale_y_m,
        )
    )
    if controller_normalization != (
        NORMALIZATION_CENTER_X,
        NORMALIZATION_SCALE_X,
        NORMALIZATION_CENTER_Y,
        NORMALIZATION_SCALE_Y,
    ):
        raise ValueError("controller normalization disagrees with V3")

    semantics = controller.payload_without_hash().get("semantics", {})
    if semantics.get("runtime_mode") != "monitor_only":
        raise ValueError("controller runtime mode is not monitor_only")
    if semantics.get("post_controller_clipping_allowed") is not False:
        raise ValueError("controller permits post-controller clipping")

    pieces = controller_pieces(controller)
    intervals = tuple(
        (
            _fraction_from_contract_float(piece.x_min_m),
            _fraction_from_contract_float(piece.x_max_m),
        )
        for piece in pieces
    )
    if intervals[0][0] != DOMAIN_PHYSICAL[0] or intervals[-1][1] != (
        DOMAIN_PHYSICAL[1]
    ):
        raise ValueError("controller partition does not cover D1")
    if any(left[1] != right[0] for left, right in zip(intervals, intervals[1:])):
        raise ValueError("controller partition is not exactly adjacent")


def _normalized_deterministic_dynamics(
    piece: PolynomialControllerPiece,
) -> tuple[RationalPolynomial, RationalPolynomial]:
    xi_x: RationalPolynomial = {(1, 0): Fraction(1)}
    xi_y: RationalPolynomial = {(0, 1): Fraction(1)}
    return (
        _add(
            xi_x,
            _scale(
                rational_sparse_polynomial(piece.u_x),
                DT / NORMALIZATION_SCALE_X,
            ),
        ),
        _add(
            xi_y,
            _scale(
                rational_sparse_polynomial(piece.u_y),
                DT / NORMALIZATION_SCALE_Y,
            ),
        ),
    )


def _expected_univariate_power(
    mean: Mapping[Power2D, Fraction],
    exponent: int,
    *,
    normalized_half_width: Fraction,
) -> RationalPolynomial:
    result: RationalPolynomial = {}
    for noise_order in range(exponent + 1):
        if noise_order % 2:
            continue
        moment = (
            Fraction(1)
            if noise_order == 0
            else (
                2
                * normalized_half_width**noise_order
                / ((noise_order + 1) * (noise_order + 2))
            )
        )
        result = _add(
            result,
            _scale(
                _power(mean, exponent - noise_order),
                comb(exponent, noise_order) * moment,
            ),
        )
    return result


def exact_expected_next_polynomial_rational(
    barrier: SparsePolynomial2D,
    piece: PolynomialControllerPiece,
) -> RationalPolynomial:
    """Construct the exact N1 expectation for one frozen controller piece."""

    mu_x, mu_y = _normalized_deterministic_dynamics(piece)
    max_x = max((power[0] for power in barrier.powers), default=0)
    max_y = max((power[1] for power in barrier.powers), default=0)
    x_table = tuple(
        _expected_univariate_power(
            mu_x,
            exponent,
            normalized_half_width=(
                NOISE_HALF_WIDTH / NORMALIZATION_SCALE_X
            ),
        )
        for exponent in range(max_x + 1)
    )
    y_table = tuple(
        _expected_univariate_power(
            mu_y,
            exponent,
            normalized_half_width=(
                NOISE_HALF_WIDTH / NORMALIZATION_SCALE_Y
            ),
        )
        for exponent in range(max_y + 1)
    )
    result: RationalPolynomial = {}
    for (x_power, y_power), coefficient in rational_sparse_polynomial(
        barrier
    ).items():
        result = _add(
            result,
            _scale(
                _multiply(x_table[x_power], y_table[y_power]),
                coefficient,
            ),
        )
    return result


def normalize_physical_box(box: RationalBox) -> RationalBox:
    return (
        (box[0] - NORMALIZATION_CENTER_X) / NORMALIZATION_SCALE_X,
        (box[1] - NORMALIZATION_CENTER_X) / NORMALIZATION_SCALE_X,
        (box[2] - NORMALIZATION_CENTER_Y) / NORMALIZATION_SCALE_Y,
        (box[3] - NORMALIZATION_CENTER_Y) / NORMALIZATION_SCALE_Y,
    )


def _physical_box(box: RationalBox) -> RationalBox:
    return (
        NORMALIZATION_CENTER_X + NORMALIZATION_SCALE_X * box[0],
        NORMALIZATION_CENTER_X + NORMALIZATION_SCALE_X * box[1],
        NORMALIZATION_CENTER_Y + NORMALIZATION_SCALE_Y * box[2],
        NORMALIZATION_CENTER_Y + NORMALIZATION_SCALE_Y * box[3],
    )


def _distance_bounds_squared(
    normalized_box: RationalBox,
    disk: RationalDisk,
) -> tuple[Fraction, Fraction]:
    x_low, x_high, y_low, y_high = _physical_box(normalized_box)
    closest_x = min(max(disk.center_x, x_low), x_high)
    closest_y = min(max(disk.center_y, y_low), y_high)
    minimum = (closest_x - disk.center_x) ** 2 + (
        closest_y - disk.center_y
    ) ** 2
    maximum = max(
        (x_value - disk.center_x) ** 2
        + (y_value - disk.center_y) ** 2
        for x_value in (x_low, x_high)
        for y_value in (y_low, y_high)
    )
    return minimum, maximum


def classify_included_disk_box(
    box: RationalBox,
    disk: RationalDisk = INITIAL_DISK,
) -> DomainClass:
    """Classify an exact box when the active domain is the disk itself."""

    minimum, maximum = _distance_bounds_squared(box, disk)
    radius_squared = disk.radius**2
    if minimum > radius_squared:
        return "outside"
    if maximum <= radius_squared:
        return "inside"
    return "boundary"


def classify_goal_exterior_box(box: RationalBox) -> DomainClass:
    """Classify an exact box when the active domain excludes the goal disk."""

    minimum, maximum = _distance_bounds_squared(box, GOAL_DISK)
    radius_squared = GOAL_DISK.radius**2
    if maximum <= radius_squared:
        return "outside"
    if minimum > radius_squared:
        return "inside"
    return "boundary"


def classify_full_box(_: RationalBox) -> DomainClass:
    return "inside"


def _box_inside_box(inner: RationalBox, outer: RationalBox) -> bool:
    return (
        outer[0] <= inner[0]
        and inner[1] <= outer[1]
        and outer[2] <= inner[2]
        and inner[3] <= outer[3]
    )


def _disk_physical_box(disk: RationalDisk) -> RationalBox:
    return (
        disk.center_x - disk.radius,
        disk.center_x + disk.radius,
        disk.center_y - disk.radius,
        disk.center_y + disk.radius,
    )


def _disk_intersects_box(disk: RationalDisk, box: RationalBox) -> bool:
    minimum, _ = _distance_bounds_squared(normalize_physical_box(box), disk)
    return minimum <= disk.radius**2


def _disks_intersect(left: RationalDisk, right: RationalDisk) -> bool:
    center_distance = (left.center_x - right.center_x) ** 2 + (
        left.center_y - right.center_y
    ) ** 2
    return center_distance <= (left.radius + right.radius) ** 2


def exact_geometry_gate() -> dict[str, Any]:
    """Prove the canonical set-containment and terminal-disjointness gates."""

    subset_checks = {
        "X0_subset_X": _box_inside_box(
            _disk_physical_box(INITIAL_DISK),
            DOMAIN_PHYSICAL,
        ),
        "Xg_subset_X": _box_inside_box(
            _disk_physical_box(GOAL_DISK),
            DOMAIN_PHYSICAL,
        ),
        **{
            f"{name}_subset_X": _box_inside_box(box, DOMAIN_PHYSICAL)
            for name, box in UNSAFE_PHYSICAL
        },
    }
    disjointness_checks = {
        "X0_disjoint_Xg": not _disks_intersect(INITIAL_DISK, GOAL_DISK),
        **{
            f"X0_disjoint_{name}": not _disk_intersects_box(
                INITIAL_DISK,
                box,
            )
            for name, box in UNSAFE_PHYSICAL
        },
        **{
            f"Xg_disjoint_{name}": not _disk_intersects_box(
                GOAL_DISK,
                box,
            )
            for name, box in UNSAFE_PHYSICAL
        },
    }
    all_checks = {**subset_checks, **disjointness_checks}
    return {
        "status": (
            "validated" if all(all_checks.values()) else "violated"
        ),
        "arithmetic": "exact_rational",
        "checks": all_checks,
    }


def first_slice_q_gate(q: float) -> bool:
    """Apply the exact decimal 1/10 gate to a binary64 artifact value."""

    return _fraction_from_artifact(q) <= FIRST_SLICE_Q_MAX


def barrier_payload_sha256(barrier: SparsePolynomial2D) -> str:
    """Return the canonical hash that binds a V3 report to one exact W artifact."""

    return content_sha256(barrier.payload())


def exact_rational_bernstein_bounds(
    polynomial: Mapping[Power2D, Fraction],
    box: RationalBox,
) -> tuple[Fraction, Fraction]:
    """Return exact tensor-Bernstein bounds without float conversion."""

    x_low, x_high, y_low, y_high = box
    if not x_low < x_high or not y_low < y_high:
        raise ValueError("Bernstein box must have positive width")
    max_x = max((power[0] for power in polynomial), default=0)
    max_y = max((power[1] for power in polynomial), default=0)
    width_x = x_high - x_low
    width_y = y_high - y_low
    local: RationalPolynomial = {}
    for (x_power, y_power), coefficient in polynomial.items():
        for local_x in range(x_power + 1):
            x_factor = (
                comb(x_power, local_x)
                * x_low ** (x_power - local_x)
                * width_x**local_x
            )
            for local_y in range(y_power + 1):
                power = (local_x, local_y)
                factor = (
                    coefficient
                    * x_factor
                    * comb(y_power, local_y)
                    * y_low ** (y_power - local_y)
                    * width_y**local_y
                )
                local[power] = local.get(power, Fraction(0)) + factor
    coefficients: list[Fraction] = []
    for bernstein_x in range(max_x + 1):
        for bernstein_y in range(max_y + 1):
            value = Fraction(0)
            for local_x in range(bernstein_x + 1):
                x_ratio = Fraction(
                    comb(bernstein_x, local_x),
                    comb(max_x, local_x),
                )
                for local_y in range(bernstein_y + 1):
                    value += (
                        local.get((local_x, local_y), Fraction(0))
                        * x_ratio
                        * Fraction(
                            comb(bernstein_y, local_y),
                            comb(max_y, local_y),
                        )
                    )
            coefficients.append(value)
    return min(coefficients), max(coefficients)


@dataclass(frozen=True, slots=True)
class RationalBernsteinValidationOptions:
    """Formal V3 options; no acceptance tolerance exists by design."""

    required_margin: Fraction = Fraction(0)
    max_depth: int = 20
    minimum_width: Fraction = Fraction(1, 2**20)
    max_leaves: int = 400_000


def _bound_payload(value: Fraction) -> dict[str, Any]:
    return {
        "exact": _fraction_text(value),
        "float": float(value),
    }


def validate_rational_polynomial_nonnegative(
    polynomial: Mapping[Power2D, Fraction],
    root_box: RationalBox,
    *,
    classify_domain: DomainClassifier,
    options: RationalBernsteinValidationOptions,
) -> dict[str, Any]:
    """Prove a rational polynomial nonnegative with zero formal tolerance."""

    if options.max_depth < 0 or options.minimum_width <= 0:
        raise ValueError("invalid rational branch-and-bound options")
    stack: list[tuple[RationalBox, int]] = [(root_box, 0)]
    leaves: list[dict[str, Any]] = []
    global_lower: Fraction | None = None
    global_upper: Fraction | None = None
    status = "validated"
    failure_reason: str | None = None
    while stack:
        box, depth = stack.pop()
        domain_class = classify_domain(box)
        if domain_class == "outside":
            leaves.append(
                {
                    "box_exact": [_fraction_text(value) for value in box],
                    "depth": depth,
                    "domain_class": domain_class,
                    "disposition": "pruned_outside_domain",
                }
            )
            continue
        lower, upper = exact_rational_bernstein_bounds(polynomial, box)
        global_lower = lower if global_lower is None else min(global_lower, lower)
        global_upper = upper if global_upper is None else max(global_upper, upper)
        if lower >= options.required_margin:
            leaves.append(
                {
                    "box_exact": [_fraction_text(value) for value in box],
                    "depth": depth,
                    "domain_class": domain_class,
                    "lower": _bound_payload(lower),
                    "upper": _bound_payload(upper),
                    "disposition": "accepted_by_exact_lower_bound",
                }
            )
            continue
        if domain_class == "inside" and upper < options.required_margin:
            status = "violated"
            failure_reason = "strict_negative_exact_upper_bound_on_active_box"
            leaves.append(
                {
                    "box_exact": [_fraction_text(value) for value in box],
                    "depth": depth,
                    "domain_class": domain_class,
                    "lower": _bound_payload(lower),
                    "upper": _bound_payload(upper),
                    "disposition": "proved_violation",
                }
            )
            break
        x_low, x_high, y_low, y_high = box
        width_x = x_high - x_low
        width_y = y_high - y_low
        if depth >= options.max_depth or max(width_x, width_y) <= (
            options.minimum_width
        ):
            status = "unknown"
            failure_reason = "subdivision_budget_exhausted"
            leaves.append(
                {
                    "box_exact": [_fraction_text(value) for value in box],
                    "depth": depth,
                    "domain_class": domain_class,
                    "lower": _bound_payload(lower),
                    "upper": _bound_payload(upper),
                    "disposition": "unresolved",
                }
            )
            break
        if len(leaves) + len(stack) + 2 > options.max_leaves:
            status = "unknown"
            failure_reason = "leaf_budget_exhausted"
            leaves.append(
                {
                    "box_exact": [_fraction_text(value) for value in box],
                    "depth": depth,
                    "domain_class": domain_class,
                    "lower": _bound_payload(lower),
                    "upper": _bound_payload(upper),
                    "disposition": "unresolved",
                }
            )
            break
        if width_x >= width_y:
            midpoint = (x_low + x_high) / 2
            children = (
                (x_low, midpoint, y_low, y_high),
                (midpoint, x_high, y_low, y_high),
            )
        else:
            midpoint = (y_low + y_high) / 2
            children = (
                (x_low, x_high, y_low, midpoint),
                (x_low, x_high, midpoint, y_high),
            )
        stack.append((children[1], depth + 1))
        stack.append((children[0], depth + 1))
    if not leaves:
        status = "unknown"
        failure_reason = "empty_active_domain"
    return {
        "status": status,
        "failure_reason": failure_reason,
        "required_margin": _bound_payload(options.required_margin),
        "acceptance_tolerance": 0.0,
        "root_box_exact": [_fraction_text(value) for value in root_box],
        "global_visited_lower_bound": (
            None if global_lower is None else _bound_payload(global_lower)
        ),
        "global_visited_upper_bound": (
            None if global_upper is None else _bound_payload(global_upper)
        ),
        "leaf_count": len(leaves),
        "leaves": leaves,
    }


def _point_inside_box(point: tuple[Fraction, Fraction], box: RationalBox) -> bool:
    return box[0] <= point[0] <= box[1] and box[2] <= point[1] <= box[3]


def _safe_drift_cells(
    controller: FrozenController,
) -> tuple[tuple[str, RationalBox, PolynomialControllerPiece], ...]:
    pieces = controller_pieces(controller)
    switch_breaks = {
        _fraction_from_contract_float(piece.x_max_m)
        for piece in pieces[:-1]
    }
    x_breaks = tuple(sorted({*BASE_PHYSICAL_X_BREAKS, *switch_breaks}))
    result: list[tuple[str, RationalBox, PolynomialControllerPiece]] = []
    for x_index, (x_low, x_high) in enumerate(zip(x_breaks[:-1], x_breaks[1:])):
        for y_index, (y_low, y_high) in enumerate(
            zip(BASE_PHYSICAL_Y_BREAKS[:-1], BASE_PHYSICAL_Y_BREAKS[1:])
        ):
            midpoint = ((x_low + x_high) / 2, (y_low + y_high) / 2)
            if any(_point_inside_box(midpoint, box) for _, box in UNSAFE_PHYSICAL):
                continue
            physical_box = (x_low, x_high, y_low, y_high)
            candidates = [
                piece
                for piece in pieces
                if _fraction_from_contract_float(piece.x_min_m) <= midpoint[0]
                <= _fraction_from_contract_float(piece.x_max_m)
            ]
            if not candidates:
                raise ValueError("drift cell is outside the controller partition")
            piece = candidates[0]
            piece_low = _fraction_from_contract_float(piece.x_min_m)
            piece_high = _fraction_from_contract_float(piece.x_max_m)
            if piece_low > x_low or piece_high < x_high:
                raise ValueError("drift cell crosses a controller switch")
            result.append(
                (
                    f"D_{x_index}_{y_index}",
                    normalize_physical_box(physical_box),
                    piece,
                )
            )
    return tuple(result)


def _action_disk_residual(
    piece: PolynomialControllerPiece,
) -> RationalPolynomial:
    u_x = rational_sparse_polynomial(piece.u_x)
    u_y = rational_sparse_polynomial(piece.u_y)
    return _add(
        _constant(MAX_SPEED**2),
        _negate(_add(_multiply(u_x, u_x), _multiply(u_y, u_y))),
    )


def _robust_domain_residuals(
    piece: PolynomialControllerPiece,
) -> dict[str, RationalPolynomial]:
    physical_x = {
        (0, 0): NORMALIZATION_CENTER_X,
        (1, 0): NORMALIZATION_SCALE_X,
    }
    physical_y = {
        (0, 0): NORMALIZATION_CENTER_Y,
        (0, 1): NORMALIZATION_SCALE_Y,
    }
    deterministic_x = _add(
        physical_x,
        _scale(rational_sparse_polynomial(piece.u_x), DT),
    )
    deterministic_y = _add(
        physical_y,
        _scale(rational_sparse_polynomial(piece.u_y), DT),
    )
    return {
        "successor_x_lower": _add(
            deterministic_x,
            _constant(-(DOMAIN_PHYSICAL[0] + NOISE_HALF_WIDTH)),
        ),
        "successor_x_upper": _add(
            _constant(DOMAIN_PHYSICAL[1] - NOISE_HALF_WIDTH),
            _negate(deterministic_x),
        ),
        "successor_y_lower": _add(
            deterministic_y,
            _constant(-(DOMAIN_PHYSICAL[2] + NOISE_HALF_WIDTH)),
        ),
        "successor_y_upper": _add(
            _constant(DOMAIN_PHYSICAL[3] - NOISE_HALF_WIDTH),
            _negate(deterministic_y),
        ),
    }


def _piece_normalized_box(piece: PolynomialControllerPiece) -> RationalBox:
    return normalize_physical_box(
        (
            _fraction_from_contract_float(piece.x_min_m),
            _fraction_from_contract_float(piece.x_max_m),
            DOMAIN_PHYSICAL[2],
            DOMAIN_PHYSICAL[3],
        )
    )


def _constraint_status(
    constraints: Mapping[str, Mapping[str, Any]],
) -> str:
    if all(result["status"] == "validated" for result in constraints.values()):
        return "validated"
    if any(result["status"] == "violated" for result in constraints.values()):
        return "violated"
    return "unknown"


def validate_qsbc_values_v3(
    *,
    barrier: SparsePolynomial2D,
    q: float,
    epsbar: float,
    controller: FrozenController,
    output_dir: str | Path,
    options: RationalBernsteinValidationOptions | None = None,
) -> Path:
    """Replay G4, S1, and D1 with exact rational arithmetic."""

    _assert_canonical_runtime_contract(controller)
    validation_options = options or RationalBernsteinValidationOptions()
    if validation_options.required_margin != 0:
        raise ValueError("V3 G4 replay requires a zero formal margin threshold")
    destination = Path(output_dir).resolve()
    destination.mkdir(parents=True, exist_ok=False)

    barrier_polynomial = rational_sparse_polynomial(barrier)
    barrier_hash = barrier_payload_sha256(barrier)
    q_exact = _fraction_from_artifact(q)
    epsbar_exact = _fraction_from_artifact(epsbar)
    constraints: dict[str, dict[str, Any]] = {}
    constraints["W_nonnegative_on_X"] = validate_rational_polynomial_nonnegative(
        barrier_polynomial,
        (Fraction(-1), Fraction(1), Fraction(-1), Fraction(1)),
        classify_domain=classify_full_box,
        options=validation_options,
    )

    initial_physical_box = (
        INITIAL_DISK.center_x - INITIAL_DISK.radius,
        INITIAL_DISK.center_x + INITIAL_DISK.radius,
        INITIAL_DISK.center_y - INITIAL_DISK.radius,
        INITIAL_DISK.center_y + INITIAL_DISK.radius,
    )
    constraints["q_minus_W_on_X0"] = validate_rational_polynomial_nonnegative(
        _add(_constant(q_exact), _negate(barrier_polynomial)),
        normalize_physical_box(initial_physical_box),
        classify_domain=classify_included_disk_box,
        options=validation_options,
    )
    for unsafe_name, unsafe_box in UNSAFE_PHYSICAL:
        constraints[f"W_minus_1_on_{unsafe_name}"] = (
            validate_rational_polynomial_nonnegative(
                _add(barrier_polynomial, _constant(-1)),
                normalize_physical_box(unsafe_box),
                classify_domain=classify_full_box,
                options=validation_options,
            )
        )

    expected_by_piece = {
        piece.piece_id: exact_expected_next_polynomial_rational(barrier, piece)
        for piece in controller_pieces(controller)
    }
    for cell_name, cell_box, piece in _safe_drift_cells(controller):
        residual = _add(
            barrier_polynomial,
            _negate(expected_by_piece[piece.piece_id]),
            _constant(-epsbar_exact),
        )
        constraints[f"drift_{piece.piece_id}_{cell_name}"] = (
            validate_rational_polynomial_nonnegative(
                residual,
                cell_box,
                classify_domain=classify_goal_exterior_box,
                options=validation_options,
            )
        )

    controller_options = replace(
        validation_options,
        required_margin=CONTROLLER_REQUIRED_MARGIN,
    )
    controller_constraints: dict[str, dict[str, Any]] = {}
    for piece in controller_pieces(controller):
        piece_box = _piece_normalized_box(piece)
        controller_constraints[f"{piece.piece_id}_action_disk"] = (
            validate_rational_polynomial_nonnegative(
                _action_disk_residual(piece),
                piece_box,
                classify_domain=classify_full_box,
                options=controller_options,
            )
        )
        for name, residual in _robust_domain_residuals(piece).items():
            controller_constraints[f"{piece.piece_id}_{name}"] = (
                validate_rational_polynomial_nonnegative(
                    residual,
                    piece_box,
                    classify_domain=classify_full_box,
                    options=controller_options,
                )
            )

    for group, group_constraints in (
        ("qsbc", constraints),
        ("controller", controller_constraints),
    ):
        for name, result in group_constraints.items():
            tree_name = f"{group}_{name}.json"
            tree_path = destination / tree_name
            _write_json(tree_path, result)
            result["tree_file"] = tree_name
            result["tree_file_sha256"] = _file_sha256(tree_path)
            result.pop("leaves", None)

    polynomial_status = _constraint_status(constraints)
    controller_status = _constraint_status(controller_constraints)
    geometry_gate = exact_geometry_gate()
    geometry_status = str(geometry_gate["status"])
    theorem_scalar_gate = (
        Fraction(0) <= q_exact < Fraction(1) and epsbar_exact > 0
    )
    combined_statuses = (
        polynomial_status,
        controller_status,
        geometry_status,
    )
    formal_status = (
        "validated"
        if all(status == "validated" for status in combined_statuses)
        and theorem_scalar_gate
        else (
            "violated"
            if "violated" in combined_statuses
            or not theorem_scalar_gate
            else "unknown"
        )
    )
    model_payload = canonical_rational_model_payload()
    report = {
        "schema_version": 2,
        "kind": "independent_qsbc_validation",
        "validator_id": INDEPENDENT_RATIONAL_VALIDATOR_ID,
        "status": formal_status,
        "global_polynomial_status": polynomial_status,
        "controller_contract_status": controller_status,
        "geometry_contract_status": geometry_status,
        "geometry_gate": geometry_gate,
        "theorem_scalar_gate": theorem_scalar_gate,
        "formal_acceptance_tolerance": 0.0,
        "q": float(q),
        "q_float64_hex": float(q).hex(),
        "q_exact": _fraction_text(q_exact),
        "epsbar": float(epsbar),
        "epsbar_float64_hex": float(epsbar).hex(),
        "epsbar_exact": _fraction_text(epsbar_exact),
        "barrier_payload_sha256": barrier_hash,
        "barrier_degree": barrier.degree,
        "barrier_term_count": len(barrier.powers),
        "first_slice_q_max": float(FIRST_SLICE_Q_MAX),
        "first_slice_ready": (
            formal_status == "validated" and first_slice_q_gate(q)
        ),
        "controller_content_sha256": controller.sha256,
        "formal_model_contract": model_payload,
        "formal_model_contract_sha256": content_sha256(model_payload),
        "constraints": constraints,
        "controller_constraints": controller_constraints,
        "independence": {
            "solver_status_read": False,
            "SOS_multiplier_read": False,
            "Gram_matrix_read": False,
            "sampled_residual_used": False,
            "float_polynomial_algebra_used_for_status": False,
            "exact_binary64_artifact_coefficients": True,
            "exact_decimal_model_constants": True,
            "exact_disk_classification": True,
            "controller_S1_and_D1_replayed": True,
        },
    }
    report["report_content_sha256"] = content_sha256(report)
    report_path = destination / "validation_report.json"
    _write_json(report_path, report)
    return report_path
