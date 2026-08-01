"""G4 infinite-horizon q-SBC certificate artifact and pure evaluator."""

from __future__ import annotations

from dataclasses import dataclass, field
from fractions import Fraction
import json
from math import inf, isfinite, nextafter
from pathlib import Path
from typing import Any, Mapping

from cbf_rover.domain.planar import (
    CANONICAL_SCENARIO,
    PlanarState,
)

from .piecewise_barrier import (
    G11P49_FROZEN_EPSBAR_FLOAT64_HEX,
    G11P49_FROZEN_Q_FLOAT64_HEX,
    G11P49_PROOF_ROUTE_ID,
    G11P49_VALIDATOR_ID,
    PiecewiseBernsteinBarrier2D,
    p49_geometry_catalog_sha256,
)
from .polynomial import (
    FrozenController,
    SparsePolynomial2D,
    content_sha256,
)


THEOREM_ID = "G4_QSBC_INFINITE_HORIZON_REACH_AVOID_V1"


def _outward_lower_float(value: Fraction) -> float:
    rounded = float(value)
    if Fraction.from_float(rounded) > value:
        return nextafter(rounded, -inf)
    return rounded


def _outward_upper_float(value: Fraction) -> float:
    rounded = float(value)
    if Fraction.from_float(rounded) < value:
        return nextafter(rounded, inf)
    return rounded


@dataclass(frozen=True, slots=True)
class QSBCertificate:
    """Frozen W/q/epsbar result bound to one frozen controller."""

    certificate_id: str
    barrier: SparsePolynomial2D | PiecewiseBernsteinBarrier2D
    q: float
    epsbar: float
    controller_content_sha256: str
    solver: Mapping[str, Any] = field(default_factory=dict)
    validation: Mapping[str, Any] = field(default_factory=dict)
    proof_witness: Mapping[str, Any] = field(default_factory=dict)
    theorem_id: str = THEOREM_ID
    schema_version: int = 1

    def __post_init__(self) -> None:
        q_value = float(self.q)
        epsbar_value = float(self.epsbar)
        if not isfinite(q_value) or not 0.0 <= q_value <= 1.0:
            raise ValueError("q must be finite and lie in [0,1]")
        if not isfinite(epsbar_value) or epsbar_value <= 0.0:
            raise ValueError("epsbar must be finite and strictly positive")
        controller_hash = self.controller_content_sha256.lower()
        if len(controller_hash) != 64 or any(
            character not in "0123456789abcdef" for character in controller_hash
        ):
            raise ValueError("controller_content_sha256 is invalid")
        if self.theorem_id != THEOREM_ID:
            raise ValueError("unsupported q-SBC theorem identity")
        if self.schema_version not in (1, 2, 3):
            raise ValueError("unsupported q-SBC certificate schema version")
        if self.schema_version in (1, 2):
            if not isinstance(self.barrier, SparsePolynomial2D):
                raise ValueError("schema-1/2 certificates require a global barrier")
            if self.proof_witness:
                raise ValueError("schema-1/2 certificates cannot bind a P49 witness")
        else:
            if not isinstance(self.barrier, PiecewiseBernsteinBarrier2D):
                raise ValueError("schema-3 certificates require a P49 barrier")
            if Fraction.from_float(q_value) > Fraction(1, 10):
                raise ValueError(
                    "schema-3 first-slice q exceeds exact one-tenth bound"
                )
            if q_value.hex() != G11P49_FROZEN_Q_FLOAT64_HEX:
                raise ValueError("schema-3 q is not the frozen G11P49 value")
            if epsbar_value.hex() != G11P49_FROZEN_EPSBAR_FLOAT64_HEX:
                raise ValueError(
                    "schema-3 epsbar is not the frozen G11P49 value"
                )
            witness = dict(self.proof_witness)
            if (
                type(witness.get("schema_version")) is not int
                or witness.get("schema_version") != 1
                or witness.get("kind")
                != "g11p49_piecewise_roof_witness_binding"
            ):
                raise ValueError("schema-3 proof-witness binding kind mismatch")
            if witness.get("proof_route_id") != G11P49_PROOF_ROUTE_ID:
                raise ValueError("schema-3 proof-route identity mismatch")
            if witness.get("validator_id") != G11P49_VALIDATOR_ID:
                raise ValueError("schema-3 validator identity mismatch")
            expected_hashes = {
                "barrier_payload_sha256": self.barrier.payload_sha256,
                "controller_content_sha256": controller_hash,
                "geometry_catalog_sha256": p49_geometry_catalog_sha256(),
            }
            for name, expected in expected_hashes.items():
                if str(witness.get(name, "")).lower() != expected:
                    raise ValueError(f"schema-3 {name} binding mismatch")
            for name in (
                "roof_witness_content_sha256",
                "formal_model_contract_sha256",
                "validation_report_content_sha256",
            ):
                digest = str(witness.get(name, "")).lower()
                if len(digest) != 64 or any(
                    character not in "0123456789abcdef" for character in digest
                ):
                    raise ValueError(f"schema-3 {name} is invalid")
            if witness.get("barrier_degree") != self.barrier.degree:
                raise ValueError("schema-3 barrier-degree binding mismatch")
            roof_degree = witness.get("roof_degree")
            if type(roof_degree) is not int or roof_degree not in (4, 6):
                raise ValueError("schema-3 roof-degree binding mismatch")
            if (self.barrier.degree, roof_degree) not in ((3, 4), (4, 6)):
                raise ValueError(
                    "schema-3 degree pair is outside the frozen G11P49 schedule"
                )

            validation = dict(self.validation)
            expected_validation_bindings = {
                "validator_id": G11P49_VALIDATOR_ID,
                "status": "validated",
                "barrier_payload_sha256": self.barrier.payload_sha256,
                "roof_witness_content_sha256": witness[
                    "roof_witness_content_sha256"
                ],
                "controller_content_sha256": controller_hash,
                "formal_model_contract_sha256": witness[
                    "formal_model_contract_sha256"
                ],
                "geometry_catalog_sha256": witness[
                    "geometry_catalog_sha256"
                ],
                "validation_report_content_sha256": witness[
                    "validation_report_content_sha256"
                ],
                "q_float64_hex": q_value.hex(),
                "epsbar_float64_hex": epsbar_value.hex(),
                "barrier_degree": self.barrier.degree,
                "roof_degree": roof_degree,
            }
            for name, expected in expected_validation_bindings.items():
                if validation.get(name) != expected:
                    raise ValueError(f"schema-3 validation {name} mismatch")
            if validation.get("first_slice_ready") is not True:
                raise ValueError("schema-3 validation is not first-slice ready")
            if (
                type(validation.get("formal_acceptance_tolerance")) is not float
                or validation.get("formal_acceptance_tolerance") != 0.0
            ):
                raise ValueError(
                    "schema-3 validation requires zero formal tolerance"
                )
            validation_file_hash = str(
                validation.get("validation_report_sha256", "")
            ).lower()
            if len(validation_file_hash) != 64 or any(
                character not in "0123456789abcdef"
                for character in validation_file_hash
            ):
                raise ValueError(
                    "schema-3 validation report file hash is invalid"
                )
            object.__setattr__(self, "proof_witness", witness)
            object.__setattr__(self, "validation", validation)
        object.__setattr__(self, "q", q_value)
        object.__setattr__(self, "epsbar", epsbar_value)
        object.__setattr__(self, "controller_content_sha256", controller_hash)

    def evaluate(self, state: PlanarState) -> float:
        xi = CANONICAL_SCENARIO.normalization.normalize(state)
        return self.barrier.evaluate(xi.x_m, xi.y_m)

    @property
    def reach_avoid_lower_bound(self) -> float:
        exact = Fraction(1) - Fraction.from_float(self.q)
        return _outward_lower_float(exact)

    @property
    def expected_stopping_time_upper_steps(self) -> float:
        exact = Fraction.from_float(self.q) / Fraction.from_float(self.epsbar)
        return _outward_upper_float(exact)

    def payload_without_hash(self) -> dict[str, Any]:
        if self.schema_version == 1:
            theorem_conclusion = {
                "event": "reach_Xg_before_Xu",
                "time_horizon": "infinite",
                "unsafe_before_goal_probability_upper_bound": self.q,
                "goal_before_unsafe_probability_lower_bound": 1.0 - self.q,
                "expected_stopping_time_upper_steps": self.q / self.epsbar,
                "q_is_not_an_instantaneous_probability": True,
                "max_steps_2000_is_diagnostic_only": True,
            }
        else:
            q_exact = Fraction.from_float(self.q)
            epsbar_exact = Fraction.from_float(self.epsbar)
            reach_avoid_exact = Fraction(1) - q_exact
            stopping_time_exact = q_exact / epsbar_exact
            theorem_conclusion = {
                "event": "reach_Xg_before_Xu",
                "time_horizon": "infinite",
                "unsafe_before_goal_probability_upper_bound": self.q,
                "unsafe_before_goal_probability_upper_bound_exact": str(q_exact),
                "goal_before_unsafe_probability_lower_bound": (
                    _outward_lower_float(reach_avoid_exact)
                ),
                "goal_before_unsafe_probability_lower_bound_exact": str(
                    reach_avoid_exact
                ),
                "expected_stopping_time_upper_steps": _outward_upper_float(
                    stopping_time_exact
                ),
                "expected_stopping_time_upper_steps_exact": str(
                    stopping_time_exact
                ),
                "float_bounds_round_outward": True,
                "q_is_not_an_instantaneous_probability": True,
                "max_steps_2000_is_diagnostic_only": True,
            }
        payload = {
            "schema_version": self.schema_version,
            "kind": "q_sbc_certificate",
            "certificate_id": self.certificate_id,
            "theorem_id": self.theorem_id,
            "barrier": self.barrier.payload(),
            "q": self.q,
            "q_float64_hex": self.q.hex(),
            "epsbar": self.epsbar,
            "epsbar_float64_hex": self.epsbar.hex(),
            "controller_content_sha256": self.controller_content_sha256,
            "theorem_conclusion": theorem_conclusion,
            "solver": dict(self.solver),
            "validation": dict(self.validation),
        }
        if self.schema_version == 3:
            payload["proof_witness"] = dict(self.proof_witness)
        return payload

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
        temporary.write_text(
            json.dumps(
                self.payload(),
                indent=2,
                sort_keys=True,
                ensure_ascii=False,
                allow_nan=False,
            )
            + "\n",
            encoding="utf-8",
        )
        temporary.replace(destination)
        return destination

    @classmethod
    def load(
        cls,
        path: str | Path,
        *,
        controller: FrozenController | None = None,
    ) -> "QSBCertificate":
        payload = json.loads(Path(path).read_text(encoding="utf-8"))
        declared_hash = str(payload.pop("content_sha256", "")).lower()
        if content_sha256(payload) != declared_hash:
            raise ValueError("certificate content hash mismatch")
        if float(payload["q"]).hex() != str(payload["q_float64_hex"]):
            raise ValueError("q decimal/hex mismatch")
        if float(payload["epsbar"]).hex() != str(
            payload["epsbar_float64_hex"]
        ):
            raise ValueError("epsbar decimal/hex mismatch")
        schema_version = int(payload["schema_version"])
        if schema_version in (1, 2):
            barrier = SparsePolynomial2D.from_payload(payload["barrier"])
        elif schema_version == 3:
            barrier = PiecewiseBernsteinBarrier2D.from_payload(payload["barrier"])
        else:
            raise ValueError("unsupported q-SBC certificate schema version")
        result = cls(
            schema_version=schema_version,
            certificate_id=str(payload["certificate_id"]),
            theorem_id=str(payload["theorem_id"]),
            barrier=barrier,
            q=float(payload["q"]),
            epsbar=float(payload["epsbar"]),
            controller_content_sha256=str(
                payload["controller_content_sha256"]
            ),
            solver=dict(payload.get("solver", {})),
            validation=dict(payload.get("validation", {})),
            proof_witness=(
                dict(payload.get("proof_witness", {}))
                if schema_version == 3
                else {}
            ),
        )
        if result.sha256 != declared_hash:
            raise ValueError("reconstructed certificate hash mismatch")
        if controller is not None and controller.sha256 != (
            result.controller_content_sha256
        ):
            raise ValueError("certificate/controller identity mismatch")
        return result
