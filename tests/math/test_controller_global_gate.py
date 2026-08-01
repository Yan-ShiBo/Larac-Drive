from __future__ import annotations

import json

from cbf_rover.methods.polynomial_sbc.controller_validation import (
    validate_and_freeze_controller,
)
from cbf_rover.methods.polynomial_sbc.polynomial import (
    FrozenPiecewisePolynomialController,
    FrozenPolynomialController,
    PolynomialControllerPiece,
    SparsePolynomial2D,
)


def test_cv1_validates_a_known_inward_linear_controller_without_rollouts(
    tmp_path,
) -> None:
    controller = FrozenPolynomialController(
        controller_id="known_inward_linear_test",
        u_x=SparsePolynomial2D(((1, 0),), (-0.2,)),
        u_y=SparsePolynomial2D(((0, 1),), (-0.2,)),
        upstream_checkpoint_sha256="b" * 64,
    )
    candidate = controller.save(tmp_path / "candidate.json")
    report_path = validate_and_freeze_controller(
        candidate_path=candidate,
        output_dir=tmp_path / "validation",
        run_rollouts=False,
    )
    report = json.loads(report_path.read_text(encoding="utf-8"))
    assert report["formal_status"] == "validated"
    assert report["status"] == "validated"
    assert (report_path.parent / "frozen_controller.json").is_file()


def test_c2v1_validates_piece_closures_without_rollouts(tmp_path) -> None:
    controller = FrozenPiecewisePolynomialController(
        controller_id="known_inward_piecewise_test",
        pieces=(
            PolynomialControllerPiece(
                "left",
                -1.0,
                0.8,
                SparsePolynomial2D(((1, 0),), (-0.2,)),
                SparsePolynomial2D(((0, 1),), (-0.2,)),
            ),
            PolynomialControllerPiece(
                "middle",
                0.8,
                3.2,
                SparsePolynomial2D(((1, 0),), (-0.2,)),
                SparsePolynomial2D(((0, 1),), (-0.2,)),
            ),
            PolynomialControllerPiece(
                "right",
                3.2,
                4.0,
                SparsePolynomial2D(((1, 0),), (-0.2,)),
                SparsePolynomial2D(((0, 1),), (-0.2,)),
            ),
        ),
        construction_source_sha256="f" * 64,
    )
    candidate = controller.save(tmp_path / "candidate_piecewise.json")
    report_path = validate_and_freeze_controller(
        candidate_path=candidate,
        output_dir=tmp_path / "piecewise_validation",
        run_rollouts=False,
    )
    report = json.loads(report_path.read_text(encoding="utf-8"))
    assert report["formal_status"] == "validated"
    assert report["status"] == "validated"
    assert report["partition"][
        "shared_boundaries_validated_on_both_closures"
    ]
    assert len(report["formal_constraints"]) == 15
