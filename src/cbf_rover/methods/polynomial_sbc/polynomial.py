"""Canonical sparse bivariate polynomials and frozen controller artifacts."""

from __future__ import annotations

from dataclasses import dataclass, field
import hashlib
import json
from math import isfinite
from pathlib import Path
from typing import Any, Iterable, Mapping, Sequence, TypeAlias

from cbf_rover.domain.planar import AffineNormalization2D, PlanarAction, PlanarState


Power2D = tuple[int, int]
NumericPolynomial = dict[Power2D, float]


def canonical_total_degree_powers(degree: int) -> tuple[Power2D, ...]:
    """PB1 monomial order: total degree, then x exponent."""

    if isinstance(degree, bool) or not isinstance(degree, int) or degree < 0:
        raise ValueError("degree must be a nonnegative integer")
    return tuple(
        (x_power, total - x_power)
        for total in range(degree + 1)
        for x_power in range(total + 1)
    )


def _canonical_json(payload: Mapping[str, Any]) -> bytes:
    return json.dumps(
        payload,
        sort_keys=True,
        separators=(",", ":"),
        ensure_ascii=False,
        allow_nan=False,
    ).encode("utf-8")


def content_sha256(payload: Mapping[str, Any]) -> str:
    return hashlib.sha256(_canonical_json(payload)).hexdigest()


@dataclass(frozen=True, slots=True)
class SparsePolynomial2D:
    """A polynomial in normalized variables ``(xi_x, xi_y)``."""

    powers: tuple[Power2D, ...]
    coefficients: tuple[float, ...]

    def __post_init__(self) -> None:
        normalized_powers: list[Power2D] = []
        for raw_power in self.powers:
            if len(raw_power) != 2:
                raise ValueError("every power must have two exponents")
            power = tuple(int(value) for value in raw_power)
            if any(value < 0 for value in power):
                raise ValueError("polynomial exponents must be nonnegative")
            if power != raw_power:
                raise ValueError("polynomial exponents must be exact integers")
            normalized_powers.append(power)
        coefficients = tuple(float(value) for value in self.coefficients)
        if len(normalized_powers) != len(coefficients):
            raise ValueError("powers and coefficients must have equal length")
        if not normalized_powers:
            raise ValueError("a polynomial must contain at least one term")
        if len(set(normalized_powers)) != len(normalized_powers):
            raise ValueError("polynomial powers must be unique")
        if not all(isfinite(value) for value in coefficients):
            raise ValueError("polynomial coefficients must be finite")
        object.__setattr__(self, "powers", tuple(normalized_powers))
        object.__setattr__(self, "coefficients", coefficients)

    @property
    def degree(self) -> int:
        return max(sum(power) for power in self.powers)

    def evaluate(self, xi_x: float, xi_y: float) -> float:
        x_value = float(xi_x)
        y_value = float(xi_y)
        if not isfinite(x_value) or not isfinite(y_value):
            raise ValueError("polynomial input must be finite")
        return float(
            sum(
                coefficient * x_value**x_power * y_value**y_power
                for (x_power, y_power), coefficient in zip(
                    self.powers,
                    self.coefficients,
                )
            )
        )

    def evaluate_many(self, points: Any) -> Any:
        """Evaluate an ``(n,2)`` array while keeping NumPy out of the domain layer."""

        import numpy as np

        values = np.asarray(points, dtype=np.float64)
        if values.ndim != 2 or values.shape[1] != 2:
            raise ValueError("points must have shape (n, 2)")
        if not np.isfinite(values).all():
            raise ValueError("points must be finite")
        result = np.zeros(values.shape[0], dtype=np.float64)
        for (x_power, y_power), coefficient in zip(
            self.powers,
            self.coefficients,
        ):
            result += (
                coefficient
                * values[:, 0] ** x_power
                * values[:, 1] ** y_power
            )
        return result

    def as_numeric_dict(self, *, zero_tolerance: float = 0.0) -> NumericPolynomial:
        if zero_tolerance < 0.0:
            raise ValueError("zero_tolerance must be nonnegative")
        return {
            power: coefficient
            for power, coefficient in zip(self.powers, self.coefficients)
            if abs(coefficient) > zero_tolerance
        }

    def payload(self) -> dict[str, Any]:
        return {
            "variable_order": ["xi_x", "xi_y"],
            "powers": [list(power) for power in self.powers],
            "coefficients_decimal": list(self.coefficients),
            "coefficients_float64_hex": [
                value.hex() for value in self.coefficients
            ],
            "degree": self.degree,
            "term_count": len(self.powers),
        }

    @classmethod
    def from_payload(cls, payload: Mapping[str, Any]) -> "SparsePolynomial2D":
        if payload.get("variable_order") != ["xi_x", "xi_y"]:
            raise ValueError("polynomial variable order mismatch")
        decimal = tuple(float(value) for value in payload["coefficients_decimal"])
        encoded = tuple(str(value) for value in payload["coefficients_float64_hex"])
        if tuple(value.hex() for value in decimal) != encoded:
            raise ValueError("decimal and float64-hex coefficients disagree")
        result = cls(
            powers=tuple(
                tuple(int(value) for value in power)
                for power in payload["powers"]
            ),
            coefficients=decimal,
        )
        if int(payload["degree"]) != result.degree:
            raise ValueError("polynomial degree metadata mismatch")
        if int(payload["term_count"]) != len(result.powers):
            raise ValueError("polynomial term-count metadata mismatch")
        return result


@dataclass(frozen=True, slots=True)
class FrozenPolynomialController:
    """C1/PB1 deterministic controller consumed read-only by q-SBC."""

    controller_id: str
    u_x: SparsePolynomial2D
    u_y: SparsePolynomial2D
    normalization: AffineNormalization2D = AffineNormalization2D()
    upstream_checkpoint_sha256: str = ""
    construction: Mapping[str, Any] = field(default_factory=dict)
    schema_version: int = 1

    def __post_init__(self) -> None:
        if self.schema_version != 1:
            raise ValueError("global polynomial controller schema must be 1")
        if not self.controller_id.strip():
            raise ValueError("controller_id must not be empty")
        checkpoint_hash = self.upstream_checkpoint_sha256.lower()
        if len(checkpoint_hash) != 64 or any(
            character not in "0123456789abcdef" for character in checkpoint_hash
        ):
            raise ValueError("upstream checkpoint hash must be a SHA-256 hex digest")
        object.__setattr__(self, "upstream_checkpoint_sha256", checkpoint_hash)

    @property
    def name(self) -> str:
        return self.controller_id

    def act(
        self,
        observation: PlanarState,
        context: Mapping[str, Any] | None = None,
    ) -> PlanarAction:
        del context
        xi = self.normalization.normalize(observation)
        return PlanarAction(
            self.u_x.evaluate(xi.x_m, xi.y_m),
            self.u_y.evaluate(xi.x_m, xi.y_m),
        )

    def payload_without_hash(self) -> dict[str, Any]:
        return {
            "schema_version": self.schema_version,
            "kind": "frozen_polynomial_controller",
            "controller_id": self.controller_id,
            "semantics": {
                "state_order": ["x_m", "y_m"],
                "action_order": ["u_x_mps", "u_y_mps"],
                "frame_id": "world",
                "runtime_mode": "monitor_only",
                "post_controller_clipping_allowed": False,
            },
            "normalization": {
                "xi_x": {
                    "center_m": self.normalization.center_x_m,
                    "scale_m": self.normalization.scale_x_m,
                },
                "xi_y": {
                    "center_m": self.normalization.center_y_m,
                    "scale_m": self.normalization.scale_y_m,
                },
            },
            "u_x": self.u_x.payload(),
            "u_y": self.u_y.payload(),
            "upstream_checkpoint_sha256": self.upstream_checkpoint_sha256,
            "construction": dict(self.construction),
        }

    @property
    def sha256(self) -> str:
        return content_sha256(self.payload_without_hash())

    def payload(self) -> dict[str, Any]:
        result = self.payload_without_hash()
        result["content_sha256"] = self.sha256
        return result

    def save(self, path: str | Path) -> Path:
        destination = Path(path)
        destination.parent.mkdir(parents=True, exist_ok=True)
        temporary = destination.with_name(f".{destination.name}.tmp")
        temporary.write_bytes(_canonical_json(self.payload()) + b"\n")
        temporary.replace(destination)
        return destination

    @classmethod
    def load(cls, path: str | Path) -> "FrozenPolynomialController":
        payload = json.loads(Path(path).read_text(encoding="utf-8"))
        declared_hash = str(payload.pop("content_sha256", "")).lower()
        actual_hash = content_sha256(payload)
        if declared_hash != actual_hash:
            raise ValueError("controller content hash mismatch")
        if payload.get("kind") != "frozen_polynomial_controller":
            raise ValueError("global polynomial controller kind mismatch")
        normalization_payload = payload["normalization"]
        result = cls(
            schema_version=int(payload["schema_version"]),
            controller_id=str(payload["controller_id"]),
            normalization=AffineNormalization2D(
                center_x_m=float(normalization_payload["xi_x"]["center_m"]),
                scale_x_m=float(normalization_payload["xi_x"]["scale_m"]),
                center_y_m=float(normalization_payload["xi_y"]["center_m"]),
                scale_y_m=float(normalization_payload["xi_y"]["scale_m"]),
            ),
            u_x=SparsePolynomial2D.from_payload(payload["u_x"]),
            u_y=SparsePolynomial2D.from_payload(payload["u_y"]),
            upstream_checkpoint_sha256=str(payload["upstream_checkpoint_sha256"]),
            construction=dict(payload.get("construction", {})),
        )
        if result.sha256 != declared_hash:
            raise ValueError("reconstructed controller hash mismatch")
        return result


@dataclass(frozen=True, slots=True)
class PolynomialControllerPiece:
    """One closed x-interval and its normalized polynomial action."""

    piece_id: str
    x_min_m: float
    x_max_m: float
    u_x: SparsePolynomial2D
    u_y: SparsePolynomial2D

    def __post_init__(self) -> None:
        if not self.piece_id.strip():
            raise ValueError("piece_id must not be empty")
        x_min = float(self.x_min_m)
        x_max = float(self.x_max_m)
        if not isfinite(x_min) or not isfinite(x_max) or x_min >= x_max:
            raise ValueError("piece x interval must be finite and ordered")
        object.__setattr__(self, "x_min_m", x_min)
        object.__setattr__(self, "x_max_m", x_max)

    def contains_x(self, x_m: float) -> bool:
        value = float(x_m)
        return self.x_min_m <= value <= self.x_max_m

    @property
    def degree(self) -> int:
        return max(self.u_x.degree, self.u_y.degree)

    def payload(self) -> dict[str, Any]:
        return {
            "piece_id": self.piece_id,
            "physical_x_interval_m": [self.x_min_m, self.x_max_m],
            "u_x": self.u_x.payload(),
            "u_y": self.u_y.payload(),
        }

    @classmethod
    def from_payload(
        cls,
        payload: Mapping[str, Any],
    ) -> "PolynomialControllerPiece":
        interval = payload["physical_x_interval_m"]
        if not isinstance(interval, list) or len(interval) != 2:
            raise ValueError("piece interval must contain two values")
        return cls(
            piece_id=str(payload["piece_id"]),
            x_min_m=float(interval[0]),
            x_max_m=float(interval[1]),
            u_x=SparsePolynomial2D.from_payload(payload["u_x"]),
            u_y=SparsePolynomial2D.from_payload(payload["u_y"]),
        )


@dataclass(frozen=True, slots=True)
class FrozenPiecewisePolynomialController:
    """Frozen semialgebraic controller with ordered closed x pieces."""

    controller_id: str
    pieces: tuple[PolynomialControllerPiece, ...]
    construction_source_sha256: str
    normalization: AffineNormalization2D = AffineNormalization2D()
    construction: Mapping[str, Any] = field(default_factory=dict)
    schema_version: int = 2

    def __post_init__(self) -> None:
        if self.schema_version != 2:
            raise ValueError("piecewise polynomial controller schema must be 2")
        if not self.controller_id.strip():
            raise ValueError("controller_id must not be empty")
        pieces = tuple(self.pieces)
        if not pieces:
            raise ValueError("piecewise controller requires at least one piece")
        if len({piece.piece_id for piece in pieces}) != len(pieces):
            raise ValueError("piece ids must be unique")
        for left, right in zip(pieces[:-1], pieces[1:]):
            if left.x_min_m >= right.x_min_m:
                raise ValueError("pieces must be ordered by x_min_m")
            if left.x_max_m != right.x_min_m:
                raise ValueError("piece intervals must be exactly adjacent")
        source_hash = self.construction_source_sha256.lower()
        if len(source_hash) != 64 or any(
            character not in "0123456789abcdef" for character in source_hash
        ):
            raise ValueError(
                "construction_source_sha256 must be a SHA-256 digest"
            )
        object.__setattr__(self, "pieces", pieces)
        object.__setattr__(self, "construction_source_sha256", source_hash)

    @property
    def name(self) -> str:
        return self.controller_id

    @property
    def max_degree(self) -> int:
        return max(piece.degree for piece in self.pieces)

    def piece_for_x(self, x_m: float) -> PolynomialControllerPiece:
        """Use ordered first-match semantics at shared closed boundaries."""

        for piece in self.pieces:
            if piece.contains_x(x_m):
                return piece
        raise ValueError("state x lies outside the controller partition")

    def act(
        self,
        observation: PlanarState,
        context: Mapping[str, Any] | None = None,
    ) -> PlanarAction:
        del context
        piece = self.piece_for_x(observation.x_m)
        xi = self.normalization.normalize(observation)
        return PlanarAction(
            piece.u_x.evaluate(xi.x_m, xi.y_m),
            piece.u_y.evaluate(xi.x_m, xi.y_m),
        )

    def payload_without_hash(self) -> dict[str, Any]:
        return {
            "schema_version": self.schema_version,
            "kind": "frozen_piecewise_polynomial_controller",
            "controller_id": self.controller_id,
            "semantics": {
                "state_order": ["x_m", "y_m"],
                "action_order": ["u_x_mps", "u_y_mps"],
                "frame_id": "world",
                "runtime_mode": "monitor_only",
                "post_controller_clipping_allowed": False,
                "partition": "ordered_closed_x_intervals_first_match",
                "formal_boundary_rule": "validate_both_adjacent_closures",
            },
            "normalization": {
                "xi_x": {
                    "center_m": self.normalization.center_x_m,
                    "scale_m": self.normalization.scale_x_m,
                },
                "xi_y": {
                    "center_m": self.normalization.center_y_m,
                    "scale_m": self.normalization.scale_y_m,
                },
            },
            "pieces": [piece.payload() for piece in self.pieces],
            "construction_source_sha256": self.construction_source_sha256,
            "construction": dict(self.construction),
        }

    @property
    def sha256(self) -> str:
        return content_sha256(self.payload_without_hash())

    def payload(self) -> dict[str, Any]:
        payload = self.payload_without_hash()
        payload["content_sha256"] = self.sha256
        return payload

    def save(self, path: str | Path) -> Path:
        destination = Path(path)
        destination.parent.mkdir(parents=True, exist_ok=True)
        temporary = destination.with_name(f".{destination.name}.tmp")
        temporary.write_bytes(_canonical_json(self.payload()) + b"\n")
        temporary.replace(destination)
        return destination

    @classmethod
    def load(
        cls,
        path: str | Path,
    ) -> "FrozenPiecewisePolynomialController":
        payload = json.loads(Path(path).read_text(encoding="utf-8"))
        declared_hash = str(payload.pop("content_sha256", "")).lower()
        actual_hash = content_sha256(payload)
        if declared_hash != actual_hash:
            raise ValueError("controller content hash mismatch")
        if payload.get("kind") != "frozen_piecewise_polynomial_controller":
            raise ValueError("piecewise polynomial controller kind mismatch")
        normalization_payload = payload["normalization"]
        result = cls(
            schema_version=int(payload["schema_version"]),
            controller_id=str(payload["controller_id"]),
            normalization=AffineNormalization2D(
                center_x_m=float(
                    normalization_payload["xi_x"]["center_m"]
                ),
                scale_x_m=float(
                    normalization_payload["xi_x"]["scale_m"]
                ),
                center_y_m=float(
                    normalization_payload["xi_y"]["center_m"]
                ),
                scale_y_m=float(
                    normalization_payload["xi_y"]["scale_m"]
                ),
            ),
            pieces=tuple(
                PolynomialControllerPiece.from_payload(piece)
                for piece in payload["pieces"]
            ),
            construction_source_sha256=str(
                payload["construction_source_sha256"]
            ),
            construction=dict(payload.get("construction", {})),
        )
        if result.sha256 != declared_hash:
            raise ValueError("reconstructed controller hash mismatch")
        return result


FrozenController: TypeAlias = (
    FrozenPolynomialController | FrozenPiecewisePolynomialController
)


def load_frozen_controller(path: str | Path) -> FrozenController:
    """Dispatch a hash-checked frozen controller by its artifact kind."""

    source = Path(path)
    payload = json.loads(source.read_text(encoding="utf-8"))
    kind = str(payload.get("kind", ""))
    if kind == "frozen_polynomial_controller":
        return FrozenPolynomialController.load(source)
    if kind == "frozen_piecewise_polynomial_controller":
        return FrozenPiecewisePolynomialController.load(source)
    raise ValueError(f"unsupported frozen controller kind: {kind!r}")


def controller_pieces(
    controller: FrozenController,
) -> tuple[PolynomialControllerPiece, ...]:
    """Expose a uniform piece view for synthesis and independent replay."""

    if isinstance(controller, FrozenPiecewisePolynomialController):
        return controller.pieces
    return (
        PolynomialControllerPiece(
            piece_id="global",
            x_min_m=-1.0,
            x_max_m=4.0,
            u_x=controller.u_x,
            u_y=controller.u_y,
        ),
    )


def controller_switch_x_breaks(
    controller: FrozenController,
) -> tuple[float, ...]:
    pieces = controller_pieces(controller)
    return tuple(piece.x_max_m for piece in pieces[:-1])


def controller_piece_for_x(
    controller: FrozenController,
    x_m: float,
) -> PolynomialControllerPiece:
    for piece in controller_pieces(controller):
        if piece.contains_x(x_m):
            return piece
    raise ValueError("x lies outside the frozen controller partition")


def controller_max_degree(controller: FrozenController) -> int:
    return max(piece.degree for piece in controller_pieces(controller))


def polynomial_from_dense_coefficients(
    powers: Sequence[Power2D],
    coefficients: Iterable[float],
    *,
    drop_tolerance: float = 0.0,
) -> SparsePolynomial2D:
    if drop_tolerance < 0.0:
        raise ValueError("drop_tolerance must be nonnegative")
    retained = [
        (tuple(power), float(coefficient))
        for power, coefficient in zip(powers, coefficients)
        if abs(float(coefficient)) > drop_tolerance
    ]
    if not retained:
        retained = [((0, 0), 0.0)]
    return SparsePolynomial2D(
        powers=tuple(power for power, _ in retained),
        coefficients=tuple(coefficient for _, coefficient in retained),
    )
