from __future__ import annotations

import json
from fractions import Fraction

from cbf_rover.entrypoints.research import build_parser
from cbf_rover.methods.polynomial_sbc.algebra import constant
from cbf_rover.methods.polynomial_sbc.g10_sparse_multiplier_lp import (
    G10_STAGE_SCHEDULE,
    _SparseLPBuilder,
)
from cbf_rover.methods.polynomial_sbc import g10r_reduced_multiplier_lp as g10r
from cbf_rover.methods.polynomial_sbc.g10r_reduced_multiplier_lp import (
    G10R_REDUNDANCY_PROOF_ID,
    G10R_REMOVED_BOX_GUARD_MULTIPLIER_COUNT,
    G10R_RETAINED_SHAPE_MULTIPLIER_COUNT,
    G10R_SYNTHESIS_ID,
    _add_reduced_obligation,
    _box_guard_product_weight,
    _frozen_decision_variable_counts,
)


def test_g10r_is_an_independent_cli_route() -> None:
    parser = build_parser()
    legacy = parser.parse_args(
        [
            "solve-qsbc-g10",
            "--controller",
            "controller.json",
            "--output-dir",
            "legacy",
        ]
    )
    reduced = parser.parse_args(
        [
            "solve-qsbc-g10r",
            "--controller",
            "controller.json",
            "--output-dir",
            "reduced",
        ]
    )

    assert legacy.command == "solve-qsbc-g10"
    assert reduced.command == "solve-qsbc-g10r"
    assert reduced.controller == "controller.json"
    assert reduced.output_dir == "reduced"
    assert G10R_SYNTHESIS_ID == (
        "G10R_CHEBYSHEV_SHAPE_MULTIPLIER_SPARSE_LP_V1"
    )


def test_same_box_guard_product_has_strictly_positive_exact_weights() -> None:
    for degree in (2, 4):
        weights = tuple(
            _box_guard_product_weight(degree, index)
            for index in range(degree + 1)
        )
        assert all(isinstance(weight, Fraction) for weight in weights)
        assert all(weight > 0 for weight in weights)

    assert _box_guard_product_weight(2, 0) == Fraction(1, 1)
    assert _box_guard_product_weight(2, 1) == Fraction(4, 3)
    assert _box_guard_product_weight(2, 2) == Fraction(1, 1)
    assert G10R_REDUNDANCY_PROOF_ID == (
        "G10R_SAME_BOX_GUARD_PROJECTION_EQUIVALENCE_V1"
    )


def test_g10r_frozen_variable_counts_remove_only_rectangle_multipliers() -> None:
    assert G10R_REMOVED_BOX_GUARD_MULTIPLIER_COUNT == 444
    assert G10R_RETAINED_SHAPE_MULTIPLIER_COUNT == 29
    assert [
        _frozen_decision_variable_counts(stage)
        for stage in G10_STAGE_SCHEDULE
    ] == [
        (4378, 382),
        (4411, 415),
        (11979, 879),
    ]


def test_g10r_obligation_allocates_no_same_box_multiplier() -> None:
    builder = _SparseLPBuilder(
        core_bounds=((-1.0, 1.0), (-1.0, 1.0)),
        multiplier_upper_bound=10.0,
    )
    _add_reduced_obligation(
        builder=builder,
        name="box_only",
        box=(-1.0, 1.0, -1.0, 1.0),
        core_terms=((0, {(1, 0): 1.0}),),
        constant_value=0.0,
        rho_index=1,
        multiplier_degree=2,
    )

    assert builder.multiplier_polynomial_count == 0
    assert builder.multiplier_coefficient_count == 0


def test_g10r_obligation_retains_one_shape_multiplier() -> None:
    builder = _SparseLPBuilder(
        core_bounds=((-1.0, 1.0), (-1.0, 1.0)),
        multiplier_upper_bound=10.0,
    )
    _add_reduced_obligation(
        builder=builder,
        name="curved_shape",
        box=(-1.0, 1.0, -1.0, 1.0),
        core_terms=((0, {(1, 0): 1.0}),),
        constant_value=0.0,
        rho_index=1,
        multiplier_degree=2,
        shape_guard=constant(1.0),
    )

    assert builder.multiplier_polynomial_count == 1
    assert builder.multiplier_coefficient_count == 9


def test_g10r_schedule_execution_error_fails_closed(
    tmp_path,
    monkeypatch,
) -> None:
    def fail_stage(**_kwargs):
        raise RuntimeError("declared G10R test failure")

    monkeypatch.setattr(g10r, "solve_g10r_stage", fail_stage)
    manifest_path = g10r.synthesize_g10r_qsbc_schedule(
        frozen_controller_path=tmp_path / "unused_controller.json",
        output_dir=tmp_path / "schedule",
    )
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))

    assert manifest["status"] == "not_ready"
    assert manifest["terminal_reason"] == (
        "G10R_stage_execution_error_fail_closed"
    )
    assert manifest["attempts"][0]["formal_claim_allowed"] is False
