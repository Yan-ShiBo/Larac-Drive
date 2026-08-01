from __future__ import annotations

import json
from fractions import Fraction

import numpy as np
import pytest
from scipy.sparse import csr_matrix

from cbf_rover.adapters.training.sac_reference import file_sha256
from cbf_rover.entrypoints.research import build_parser
from cbf_rover.methods.polynomial_sbc.algebra import evaluate
from cbf_rover.methods.polynomial_sbc import g10_sparse_multiplier_lp as g10
from cbf_rover.methods.polynomial_sbc.chebyshev import (
    chebyshev_coefficients_to_monomial,
    total_degree_chebyshev_basis,
)
from cbf_rover.methods.polynomial_sbc.certificate import QSBCertificate
from cbf_rover.methods.polynomial_sbc.g10_sparse_multiplier_lp import (
    G10_EPSBAR,
    G10_Q,
    G10_STAGE_SCHEDULE,
    G10_CONTROLLER_CONTENT_SHA256,
    G10_CONTROLLER_FILE_SHA256,
    G10StageSpec,
    _SparseLPBuilder,
    _build_g10_problem,
    _local_multiplier_material,
    _maximum_bound_violation,
    _v3_release_gate,
)
from cbf_rover.methods.polynomial_sbc.polynomial import (
    FrozenPolynomialController,
    SparsePolynomial2D,
    content_sha256,
)


def test_g10_fixed_q_is_strictly_below_exact_one_tenth() -> None:
    assert G10_Q.hex() == "0x1.9999999999999p-4"
    assert Fraction.from_float(G10_Q) < Fraction(1, 10)
    assert G10_EPSBAR.hex() == "0x1.0c6f7a0b5ed8dp-20"


def test_g10_schedule_is_finite_and_frozen() -> None:
    assert [
        (
            stage.stage_id,
            stage.barrier_degree,
            stage.multiplier_degree,
            stage.time_limit_s,
        )
        for stage in G10_STAGE_SCHEDULE
    ] == [
        ("W14_M2", 14, 2, 900.0),
        ("W16_M2", 16, 2, 3600.0),
        ("W16_M4", 16, 4, 7200.0),
    ]


def test_g10_frozen_controller_hashes_are_explicit() -> None:
    assert G10_CONTROLLER_CONTENT_SHA256 == (
        "b0cacedfcd4b884961a786b24e84b2f0f33bbc4ac81aef0b1ff1b0dcfe285984"
    )
    assert G10_CONTROLLER_FILE_SHA256 == (
        "df0dbfd2cc5b3aa0a049b5dbb6e7d0360c6b260801e183e95acfb7a9bccf715d"
    )


def test_g10_rejects_direct_stage_outside_frozen_schedule() -> None:
    altered = G10StageSpec("W14_M2", 14, 2, 901.0)

    with pytest.raises(ValueError, match="outside the frozen finite schedule"):
        _build_g10_problem(controller=None, stage=altered)  # type: ignore[arg-type]


def test_g10_cli_exposes_only_controller_and_output_for_frozen_schedule() -> None:
    args = build_parser().parse_args(
        [
            "solve-qsbc-g10",
            "--controller",
            "controller.json",
            "--output-dir",
            "results",
        ]
    )

    assert args.command == "solve-qsbc-g10"
    assert args.controller == "controller.json"
    assert args.output_dir == "results"


def test_total_degree_chebyshev_conversion_is_canonical() -> None:
    indices, basis = total_degree_chebyshev_basis(2)
    coefficients = tuple(1.0 if index == (2, 0) else 0.0 for index in indices)
    polynomial = chebyshev_coefficients_to_monomial(
        indices=indices,
        coefficients=coefficients,
        basis=basis,
    )

    assert polynomial.as_numeric_dict() == {
        (0, 0): -1.0,
        (2, 0): 2.0,
    }


def test_local_tensor_bernstein_multiplier_is_nonnegative_partition_of_unity() -> None:
    basis, guard_x, guard_y = _local_multiplier_material(
        (-0.7, 0.3, -0.4, 0.8),
        2,
    )

    assert len(basis) == 9
    for point in ((-0.7, -0.4), (-0.2, 0.1), (0.3, 0.8)):
        values = [evaluate(polynomial, point) for polynomial in basis]
        assert min(values) >= -1.0e-15
        assert np.isclose(sum(values), 1.0)
        assert evaluate(guard_x, point) >= -1.0e-15
        assert evaluate(guard_y, point) >= -1.0e-15


def test_sparse_builder_finalizes_directly_to_csr() -> None:
    builder = _SparseLPBuilder(
        core_bounds=((-1.0, 1.0),),
        multiplier_upper_bound=10.0,
    )
    builder.add_positive_family(
        "linear",
        indexed_polynomials=((0, {(1, 0): 1.0}),),
        box=(-1.0, 1.0, -1.0, 1.0),
        constant_value=0.0,
    )

    matrix, right_hand_side, bounds = builder.finalize()

    assert isinstance(matrix, csr_matrix)
    assert matrix.shape == (2, 1)
    assert right_hand_side.shape == (2,)
    assert bounds == ((-1.0, 1.0),)
    assert matrix.toarray().tolist() == [[1.0], [-1.0]]


def test_g10_numeric_bound_violation_is_explicit() -> None:
    values = np.asarray([1.0 + 2.0e-8, -0.25], dtype=np.float64)

    assert _maximum_bound_violation(values, ((-1.0, 1.0), (-0.5, 0.5))) == (
        pytest.approx(2.0e-8)
    )


def test_g10_v3_release_gate_binds_report_tree_controller_and_barrier(
    tmp_path,
) -> None:
    tree_path = tmp_path / "proof_tree.json"
    tree_path.write_text("{}\n", encoding="utf-8")
    zero = SparsePolynomial2D(((0, 0),), (0.0,))
    controller = FrozenPolynomialController(
        controller_id="release_gate_test",
        u_x=zero,
        u_y=zero,
        upstream_checkpoint_sha256="0" * 64,
    )
    barrier = SparsePolynomial2D(((0, 0), (1, 0)), (0.25, 0.5))
    validation = {
        "schema_version": 2,
        "validator_id": g10.G10_VALIDATOR_ID,
        "status": "validated",
        "first_slice_ready": True,
        "formal_acceptance_tolerance": 0.0,
        "controller_content_sha256": controller.sha256,
        "barrier_payload_sha256": content_sha256(barrier.payload()),
        "barrier_degree": barrier.degree,
        "barrier_term_count": len(barrier.powers),
        "q_float64_hex": G10_Q.hex(),
        "q_exact": str(Fraction.from_float(G10_Q)),
        "epsbar_float64_hex": G10_EPSBAR.hex(),
        "epsbar_exact": str(Fraction.from_float(G10_EPSBAR)),
        "theorem_scalar_gate": True,
        "constraints": {
            "test": {
                "tree_file": tree_path.name,
                "tree_file_sha256": file_sha256(tree_path),
            }
        },
        "controller_constraints": {},
    }
    validation["report_content_sha256"] = content_sha256(validation)

    passed, failures = _v3_release_gate(
        validation=validation,
        barrier=barrier,
        controller=controller,
        validation_directory=tmp_path,
    )
    assert passed
    assert failures == ()

    changed = SparsePolynomial2D(((0, 0), (1, 0)), (0.25, 0.75))
    passed, failures = _v3_release_gate(
        validation=validation,
        barrier=changed,
        controller=controller,
        validation_directory=tmp_path,
    )
    assert not passed
    assert "barrier_payload_sha256" in failures


def test_schema_v2_certificate_rounds_derived_bounds_outward() -> None:
    barrier = SparsePolynomial2D(((0, 0),), (0.0,))
    certificate = QSBCertificate(
        certificate_id="outward_rounding_test",
        barrier=barrier,
        q=G10_Q,
        epsbar=G10_EPSBAR,
        controller_content_sha256="0" * 64,
        schema_version=2,
    )
    conclusion = certificate.payload()["theorem_conclusion"]
    exact_lower = Fraction(1) - Fraction.from_float(G10_Q)
    exact_upper = Fraction.from_float(G10_Q) / Fraction.from_float(G10_EPSBAR)

    assert Fraction.from_float(
        conclusion["goal_before_unsafe_probability_lower_bound"]
    ) <= exact_lower
    assert Fraction.from_float(
        conclusion["expected_stopping_time_upper_steps"]
    ) >= exact_upper
    assert conclusion["goal_before_unsafe_probability_lower_bound_exact"] == str(
        exact_lower
    )
    assert conclusion["expected_stopping_time_upper_steps_exact"] == str(
        exact_upper
    )


def test_g10_schedule_execution_error_fails_closed(
    tmp_path,
    monkeypatch,
) -> None:
    def fail_stage(**_kwargs):
        raise RuntimeError("declared test failure")

    monkeypatch.setattr(g10, "solve_g10_stage", fail_stage)
    manifest_path = g10.synthesize_g10_qsbc_schedule(
        frozen_controller_path=tmp_path / "unused_controller.json",
        output_dir=tmp_path / "schedule",
    )
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))

    assert manifest["status"] == "not_ready"
    assert manifest["terminal_reason"] == "G10_stage_execution_error_fail_closed"
    assert manifest["attempts"][0]["formal_claim_allowed"] is False
