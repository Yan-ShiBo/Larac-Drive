from __future__ import annotations

import json
from fractions import Fraction

from cbf_rover.entrypoints.research import build_parser
from cbf_rover.methods.polynomial_sbc import g10e_extended_reduced_lp as g10e
from cbf_rover.methods.polynomial_sbc.g10e_extended_reduced_lp import (
    G10E_FROZEN_INEQUALITY_COUNTS,
    G10E_REDUNDANCY_PROOF_ID,
    G10E_STAGE_SCHEDULE,
    G10E_SYNTHESIS_ID,
    G10EStageSpec,
    _frozen_g10e_decision_variable_counts,
)
from cbf_rover.methods.polynomial_sbc.g10r_reduced_multiplier_lp import (
    G10R_REMOVED_BOX_GUARD_MULTIPLIER_COUNT,
    G10R_RETAINED_SHAPE_MULTIPLIER_COUNT,
    _box_guard_product_weight,
)


def test_g10e_is_an_independent_cli_route() -> None:
    parser = build_parser()
    legacy = parser.parse_args(
        [
            "solve-qsbc-g10r",
            "--controller",
            "controller.json",
            "--output-dir",
            "legacy",
        ]
    )
    extended = parser.parse_args(
        [
            "solve-qsbc-g10e",
            "--controller",
            "controller.json",
            "--output-dir",
            "extended",
        ]
    )

    assert legacy.command == "solve-qsbc-g10r"
    assert extended.command == "solve-qsbc-g10e"
    assert extended.controller == "controller.json"
    assert extended.output_dir == "extended"
    assert G10E_SYNTHESIS_ID == (
        "G10E_EXTENDED_CHEBYSHEV_SHAPE_MULTIPLIER_LP_V1"
    )


def test_g10e_schedule_is_exactly_w18_then_w20() -> None:
    assert G10E_STAGE_SCHEDULE == (
        G10EStageSpec("W18_M4", 18, 4, 7200.0),
        G10EStageSpec("W20_M4", 20, 4, 10_800.0),
    )
    assert all(
        stage.barrier_degree >= stage.multiplier_degree + 2
        for stage in G10E_STAGE_SCHEDULE
    )


def test_g10e_reuses_exact_same_box_redundancy_identity() -> None:
    weights = tuple(_box_guard_product_weight(4, index) for index in range(5))

    assert all(isinstance(weight, Fraction) for weight in weights)
    assert all(weight > 0 for weight in weights)
    assert G10E_REDUNDANCY_PROOF_ID == (
        "G10E_SAME_BOX_GUARD_PROJECTION_EQUIVALENCE_W18_W20_V1"
    )
    assert G10R_REMOVED_BOX_GUARD_MULTIPLIER_COUNT == 444
    assert G10R_RETAINED_SHAPE_MULTIPLIER_COUNT == 29


def test_g10e_frozen_variable_counts_retain_only_shape_multipliers() -> None:
    assert [
        _frozen_g10e_decision_variable_counts(stage)
        for stage in G10E_STAGE_SCHEDULE
    ] == [
        (12016, 916),
        (12057, 957),
    ]
    assert G10E_FROZEN_INEQUALITY_COUNTS == {
        "W18_M4": 121_182,
        "W20_M4": 148_302,
    }


def test_g10e_stage_validation_rejects_schedule_expansion() -> None:
    try:
        G10EStageSpec("W22_M4", 22, 4, 10_800.0)
    except ValueError as error:
        assert str(error) == "G10E barrier degree must be 18 or 20"
    else:
        raise AssertionError("G10E must reject barrier degree 22")


def test_g10e_schedule_execution_error_fails_closed(
    tmp_path,
    monkeypatch,
) -> None:
    def fail_stage(**_kwargs):
        raise RuntimeError("declared G10E test failure")

    monkeypatch.setattr(g10e, "solve_g10e_stage", fail_stage)
    manifest_path = g10e.synthesize_g10e_qsbc_schedule(
        frozen_controller_path=tmp_path / "unused_controller.json",
        output_dir=tmp_path / "schedule",
    )
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))

    assert manifest["status"] == "not_ready"
    assert manifest["terminal_reason"] == (
        "G10E_stage_execution_error_fail_closed"
    )
    assert manifest["attempts"][0]["formal_claim_allowed"] is False
