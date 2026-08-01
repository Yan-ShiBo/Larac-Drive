from __future__ import annotations

from cbf_rover.application.certificate_runtime import (
    build_qsbc_runtime_facts,
    runtime_facts_json,
)
from cbf_rover.domain.planar import PlanarAction, PlanarState
from cbf_rover.methods.polynomial_sbc.certificate import QSBCertificate
from cbf_rover.methods.polynomial_sbc.polynomial import SparsePolynomial2D


def _certificate(*, validation_status: str = "validated") -> QSBCertificate:
    return QSBCertificate(
        certificate_id="test-certificate",
        barrier=SparsePolynomial2D(
            powers=((0, 0), (1, 0)),
            coefficients=(0.2, 0.1),
        ),
        q=0.1,
        epsbar=1.0e-6,
        controller_content_sha256="a" * 64,
        validation={
            "status": validation_status,
            "first_slice_ready": True,
        },
    )


def test_live_facts_are_monitor_only_and_identity_bound() -> None:
    certificate = _certificate()
    state = PlanarState(1.5, 0.0)
    payload = build_qsbc_runtime_facts(
        certificate=certificate,
        state=state,
        nominal_action=PlanarAction(0.2, -0.1),
        stamp_s=12.5,
        yaw_rad=0.25,
        command_linear_mps=0.18,
        command_angular_radps=-0.4,
        tracker_saturation_applied=False,
        geometry_terminal_reason=None,
    )

    assert payload["mode"] == "monitor_only"
    assert payload["runtime_claim_level"] == "empirical"
    assert payload["certificate"]["content_sha256"] == certificate.sha256
    assert payload["certificate"]["barrier_value_W"] == 0.2
    assert payload["certificate"]["q_is_not_an_instantaneous_probability"]
    assert not payload["control"]["certificate_modified_action"]
    assert '"barrier_value_W":0.2' in runtime_facts_json(payload)
