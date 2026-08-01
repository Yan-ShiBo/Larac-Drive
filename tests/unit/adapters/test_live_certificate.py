from __future__ import annotations

from cbf_rover.adapters.exact.live_certificate import (
    run_exact_certificate_episode,
)
from cbf_rover.methods.polynomial_sbc.certificate import QSBCertificate
from cbf_rover.methods.polynomial_sbc.polynomial import (
    FrozenPolynomialController,
    SparsePolynomial2D,
)


def _constant(value: float) -> SparsePolynomial2D:
    return SparsePolynomial2D(((0, 0),), (value,))


def test_exact_live_frames_are_identity_bound_and_monitor_only() -> None:
    controller = FrozenPolynomialController(
        controller_id="unit-controller",
        u_x=_constant(0.1),
        u_y=_constant(0.0),
        upstream_checkpoint_sha256="b" * 64,
    )
    certificate = QSBCertificate(
        certificate_id="unit-certificate",
        barrier=_constant(0.2),
        q=0.5,
        epsbar=1.0e-6,
        controller_content_sha256=controller.sha256,
        validation={"status": "validated", "first_slice_ready": False},
    )

    rows, _ = run_exact_certificate_episode(
        controller=controller,
        certificate=certificate,
        seed=9,
        wall_clock=False,
        emit_console=False,
        max_steps=2,
    )

    assert len(rows) == 3
    assert all(row["runtime"]["mode"] == "monitor_only" for row in rows)
    assert all(
        not row["runtime"]["certificate_modified_action"] for row in rows
    )
    assert all(
        row["certificate"]["content_sha256"] == certificate.sha256
        for row in rows
    )
