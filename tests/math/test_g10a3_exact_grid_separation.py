from __future__ import annotations

from copy import deepcopy
from fractions import Fraction
import hashlib
import json
from pathlib import Path
from types import SimpleNamespace

import pytest
import yaml
from scipy.sparse import csr_matrix

from cbf_rover.entrypoints.research import build_parser
from cbf_rover.methods.polynomial_sbc.g10a_active_set_lp import (
    _ConstraintFamily,
)
from cbf_rover.methods.polynomial_sbc.polynomial import content_sha256


def test_g10a3_cli_is_independent_and_requires_the_bound_seed() -> None:
    args = build_parser().parse_args(
        [
            "solve-qsbc-g10a3",
            "--controller",
            "controller.json",
            "--g9-seed",
            "g10a_g9_seed_bundle.json",
            "--output-dir",
            "results",
        ]
    )

    assert args.command == "solve-qsbc-g10a3"
    assert args.controller == "controller.json"
    assert args.g9_seed == "g10a_g9_seed_bundle.json"
    assert args.output_dir == "results"


def _full_family(
    *,
    name: str = "W_nonnegative_on_X",
    root: tuple[Fraction, Fraction, Fraction, Fraction] = (
        Fraction(-1),
        Fraction(1),
        Fraction(-1),
        Fraction(1),
    ),
) -> _ConstraintFamily:
    return _ConstraintFamily(
        name=name,
        root_box_exact=root,
        domain_kind="full",
        basis=({(0, 0): 1.0},),
        constant_value=0.0,
    )


def _solver_candidate(
    coefficients: tuple[float, ...],
    *,
    variable_count: int,
    rho_index: int,
) -> tuple[tuple[float, ...], str]:
    from cbf_rover.methods.polynomial_sbc.g10a3_exact_grid_separation import (
        _g10a3_candidate_values_sha256,
    )

    assert len(coefficients) == rho_index
    suffix = tuple(
        0.0 if index == rho_index else float(index - rho_index)
        for index in range(rho_index, variable_count)
    )
    values = coefficients + suffix
    return values, _g10a3_candidate_values_sha256(values)


def test_g10a3_schedule_is_finite_w16_only_and_has_one_full_v3() -> None:
    from cbf_rover.methods.polynomial_sbc.g10a3_exact_grid_separation import (
        G10A3_CATALOG_GENERATION_POINT_YIELDS,
        G10A3_GRID_PASS_SCHEDULE,
        G10A3_MAX_CANDIDATE_SCAN_POINT_VISITS,
        G10A3_MAX_FULL_V3_CALLS,
        G10A3_MAX_HARD_POINT_ROWS,
        G10A3_MAX_TOTAL_GRID_POINT_YIELDS,
        G10A3_STAGE,
    )

    assert G10A3_STAGE.stage_id == "W16_EXACT_GRID_MULTIPOINT"
    assert G10A3_STAGE.barrier_degree == 16
    assert G10A3_STAGE.max_lp_solves == 8
    assert [
        (item.pass_id, item.nodes_per_axis, item.strata_per_axis, item.max_new_points_per_family)
        for item in G10A3_GRID_PASS_SCHEDULE
    ] == [
        ("G17_S8", 17, 4, 8),
        ("G33_S16", 33, 8, 16),
        ("G65_S32", 65, 8, 32),
    ]
    assert G10A3_MAX_FULL_V3_CALLS == 1
    assert G10A3_MAX_HARD_POINT_ROWS == 42 * (8 + 16 + 32)
    assert G10A3_CATALOG_GENERATION_POINT_YIELDS == 42 * (
        17**2 + 33**2 + 65**2
    )
    assert G10A3_MAX_CANDIDATE_SCAN_POINT_VISITS == 42 * (
        17**2 + 33**2 + 65**2 + 65**2
    )
    assert G10A3_MAX_TOTAL_GRID_POINT_YIELDS == 42 * (
        2 * 17**2 + 2 * 33**2 + 3 * 65**2
    )


def test_g10a3_catalog_is_exact_nested_and_hash_binds_the_root() -> None:
    from cbf_rover.methods.polynomial_sbc.g10a3_exact_grid_separation import (
        build_g10a3_grid_catalog,
        iter_g10a3_grid_points,
    )

    first = build_g10a3_grid_catalog((_full_family(),))
    second = build_g10a3_grid_catalog((_full_family(),))

    assert first.content_sha256 == second.content_sha256
    assert first.point_counts_by_grid == {"17": 289, "33": 1089, "65": 4225}
    assert first.catalog_generation_point_yields == 17**2 + 33**2 + 65**2
    points_g33 = tuple(
        iter_g10a3_grid_points(
            _full_family(),
            nodes_per_axis=33,
            strata_per_axis=8,
        )
    )
    assert points_g33[0].point_exact == (
        Fraction(-1),
        Fraction(-1),
    )
    assert points_g33[-1].point_exact == (Fraction(1), Fraction(1))
    assert any(
        point.point_exact == (Fraction(15, 16), Fraction(15, 16))
        for point in points_g33
    )
    changed = build_g10a3_grid_catalog(
        (
            _full_family(
                root=(
                    Fraction(-1),
                    Fraction(1),
                    Fraction(-1),
                    Fraction(3, 4),
                )
            ),
        )
    )
    assert changed.content_sha256 != first.content_sha256

    changed_basis_family = _ConstraintFamily(
        name="W_nonnegative_on_X",
        root_box_exact=(
            Fraction(-1),
            Fraction(1),
            Fraction(-1),
            Fraction(1),
        ),
        domain_kind="full",
        basis=({(0, 0): 2.0},),
        constant_value=0.0,
    )
    changed_basis = build_g10a3_grid_catalog((changed_basis_family,))
    assert changed_basis.family_contract_sha256 != (
        first.family_contract_sha256
    )
    assert changed_basis.content_sha256 != first.content_sha256


def test_g10a3_scaled_exact_grid_evaluator_matches_fraction_reference() -> None:
    from cbf_rover.methods.polynomial_sbc.g10a3_exact_grid_separation import (
        _evaluate_exact_polynomial,
        _scaled_exact_grid_polynomial_values,
    )

    polynomial = {
        (0, 0): Fraction(1, 10),
        (1, 0): Fraction(-7, 16),
        (0, 2): Fraction(5, 9),
        (3, 1): Fraction(-11, 32),
    }
    x_values = (Fraction(-1, 3), Fraction(0), Fraction(2, 5))
    y_values = (Fraction(-2, 7), Fraction(1, 4))
    scaled = _scaled_exact_grid_polynomial_values(
        polynomial,
        x_values=x_values,
        y_values=y_values,
    )

    for i, x_value in enumerate(x_values):
        for j, y_value in enumerate(y_values):
            assert scaled.fraction_at(i, j) == _evaluate_exact_polynomial(
                polynomial,
                (x_value, y_value),
            )


def test_g10a3_exact_scan_selects_spatially_distinct_worst_points() -> None:
    from cbf_rover.methods.polynomial_sbc.g10a3_exact_grid_separation import (
        G10A3_GRID_PASS_SCHEDULE,
        build_g10a3_grid_catalog,
        scan_g10a3_exact_grid,
    )

    family = _ConstraintFamily(
        name="W_nonnegative_on_X",
        root_box_exact=(Fraction(-1), Fraction(1), Fraction(-1), Fraction(1)),
        domain_kind="full",
        basis=({(1, 0): 1.0}, {(0, 1): 1.0}),
        constant_value=0.0,
    )
    catalog = build_g10a3_grid_catalog((family,))
    candidate_values, candidate_values_sha256 = _solver_candidate(
        (1.0, 1.0),
        variable_count=6,
        rho_index=2,
    )
    result = scan_g10a3_exact_grid(
        families=(family,),
        exact_coefficient_space_residuals={
            family.name: {(1, 0): Fraction(1), (0, 1): Fraction(1)}
        },
        exact_artifact_residuals={
            family.name: {(1, 0): Fraction(1), (0, 1): Fraction(1)}
        },
        candidate_values=candidate_values,
        candidate_values_sha256=candidate_values_sha256,
        variable_count=6,
        rho_index=2,
        candidate_barrier_sha256="1" * 64,
        family_contract_sha256=catalog.family_contract_sha256,
        pass_spec=G10A3_GRID_PASS_SCHEDULE[0],
        expected_point_set_sha256=catalog.expected_point_set_sha256_by_grid["17"],
        existing_constraint_definition_ids=frozenset(),
    )

    assert result.whole_catalog_traversed is True
    assert result.point_set_binding_passed is True
    assert result.visited_point_count == 17**2
    assert result.active_point_count == 17**2
    assert len(result.selected_cuts) == 8
    assert result.selected_cuts[0].point_exact == (Fraction(-1), Fraction(-1))
    assert len(
        {
            (cut.stratum_i, cut.stratum_j)
            for cut in result.selected_cuts
        }
    ) == 8
    assert len(
        {cut.constraint_definition_id for cut in result.selected_cuts}
    ) == 8
    assert all(
        cut.exact_coefficient_space_candidate_residual < 0
        for cut in result.selected_cuts
    )
    assert result.formal_claim_allowed is False


def test_g10a3_separates_coefficient_selection_from_artifact_gate() -> None:
    from cbf_rover.methods.polynomial_sbc.g10a3_exact_grid_separation import (
        G10A3_GRID_PASS_SCHEDULE,
        build_g10a3_grid_catalog,
        build_g10a3_numeric_point_cut_rows,
        scan_g10a3_exact_grid,
    )

    family = _full_family()
    catalog = build_g10a3_grid_catalog((family,))
    selection_values, selection_values_sha256 = _solver_candidate(
        (-1.0,),
        variable_count=4,
        rho_index=1,
    )
    selection_scan = scan_g10a3_exact_grid(
        families=(family,),
        exact_coefficient_space_residuals={
            family.name: {(0, 0): Fraction(-1)}
        },
        exact_artifact_residuals={
            family.name: {(0, 0): Fraction(-3, 4)}
        },
        candidate_values=selection_values,
        candidate_values_sha256=selection_values_sha256,
        variable_count=4,
        rho_index=1,
        candidate_barrier_sha256="5" * 64,
        family_contract_sha256=catalog.family_contract_sha256,
        pass_spec=G10A3_GRID_PASS_SCHEDULE[0],
        expected_point_set_sha256=catalog.expected_point_set_sha256_by_grid["17"],
        existing_constraint_definition_ids=frozenset(),
    )
    cut = selection_scan.selected_cuts[0]
    assert cut.exact_coefficient_space_candidate_residual == -1
    assert cut.exact_artifact_candidate_residual == Fraction(-3, 4)
    assert cut.exact_artifact_minus_coefficient_space_residual == Fraction(1, 4)
    _, _, rows = build_g10a3_numeric_point_cut_rows(
        families=(family,),
        cuts=(cut,),
        candidate_barrier_sha256="5" * 64,
        candidate_values=selection_values,
        candidate_values_sha256=selection_values_sha256,
        exact_artifact_residuals={
            family.name: {(0, 0): Fraction(-3, 4)}
        },
        variable_count=4,
        rho_index=1,
    )
    assert rows[0]["candidate_row_replay_residual"] == -1.0
    assert rows[0]["candidate_artifact_replay_residual"] == -0.75
    assert rows[0]["artifact_minus_coefficient_space_residual"] == 0.25

    artifact_values, artifact_values_sha256 = _solver_candidate(
        (1.0,),
        variable_count=4,
        rho_index=1,
    )
    artifact_gate = scan_g10a3_exact_grid(
        families=(family,),
        exact_coefficient_space_residuals={
            family.name: {(0, 0): Fraction(1)}
        },
        exact_artifact_residuals={
            family.name: {(0, 0): Fraction(-1, 2)}
        },
        candidate_values=artifact_values,
        candidate_values_sha256=artifact_values_sha256,
        variable_count=4,
        rho_index=1,
        candidate_barrier_sha256="6" * 64,
        family_contract_sha256=catalog.family_contract_sha256,
        pass_spec=G10A3_GRID_PASS_SCHEDULE[0],
        expected_point_set_sha256=catalog.expected_point_set_sha256_by_grid["17"],
        existing_constraint_definition_ids=frozenset(),
        select_points=False,
    )
    assert artifact_gate.coefficient_space_negative_point_count == 0
    assert artifact_gate.negative_point_count == 17**2
    assert artifact_gate.gate_passed is False


def test_g10a3_point_rows_have_original_direction_and_zero_solver_suffix() -> None:
    from cbf_rover.methods.polynomial_sbc.g10a3_exact_grid_separation import (
        G10A3_GRID_PASS_SCHEDULE,
        build_g10a3_grid_catalog,
        build_g10a3_numeric_point_cut_rows,
        scan_g10a3_exact_grid,
    )

    family = _ConstraintFamily(
        name="W_nonnegative_on_X",
        root_box_exact=(Fraction(-1), Fraction(1), Fraction(-1), Fraction(1)),
        domain_kind="full",
        basis=({(0, 0): 1.0}, {(1, 0): 1.0}),
        constant_value=0.0,
    )
    catalog = build_g10a3_grid_catalog((family,))
    candidate_values, candidate_values_sha256 = _solver_candidate(
        (0.0, 1.0),
        variable_count=6,
        rho_index=2,
    )
    scan = scan_g10a3_exact_grid(
        families=(family,),
        exact_coefficient_space_residuals={
            family.name: {(1, 0): Fraction(1)}
        },
        exact_artifact_residuals={family.name: {(1, 0): Fraction(1)}},
        candidate_values=candidate_values,
        candidate_values_sha256=candidate_values_sha256,
        variable_count=6,
        rho_index=2,
        candidate_barrier_sha256="2" * 64,
        family_contract_sha256=catalog.family_contract_sha256,
        pass_spec=G10A3_GRID_PASS_SCHEDULE[0],
        expected_point_set_sha256=catalog.expected_point_set_sha256_by_grid["17"],
        existing_constraint_definition_ids=frozenset(),
    )
    first = scan.selected_cuts[0]
    matrix, rhs, ledger_rows = build_g10a3_numeric_point_cut_rows(
        families=(family,),
        cuts=(first,),
        candidate_barrier_sha256="2" * 64,
        candidate_values=candidate_values,
        candidate_values_sha256=candidate_values_sha256,
        exact_artifact_residuals={family.name: {(1, 0): Fraction(1)}},
        variable_count=6,
        rho_index=2,
    )

    assert matrix.shape == (1, 6)
    assert matrix.toarray()[0].tolist() == [-1.0, 1.0, 0.0, 0.0, 0.0, 0.0]
    assert rhs.tolist() == [-1.0e-8]
    assert ledger_rows[0]["constraint_definition_id"] == first.constraint_definition_id
    assert ledger_rows[0]["rho_t_z_columns_bitwise_zero"] is True
    assert ledger_rows[0]["candidate_row_replay_residual"] < 0
    assert ledger_rows[0]["formal_claim_allowed"] is False
    actual_violation = float((matrix @ candidate_values - rhs)[0])
    assert actual_violation == ledger_rows[0]["candidate_numeric_violation"]
    assert actual_violation > 1.0e-9

    changed_values = (-0.0, -1.0, 0.0, 1.0, 2.0, 3.0)
    with pytest.raises(ValueError, match="decision-vector hash mismatch"):
        build_g10a3_numeric_point_cut_rows(
            families=(family,),
            cuts=(first,),
            candidate_barrier_sha256="2" * 64,
            candidate_values=changed_values,
            candidate_values_sha256=candidate_values_sha256,
            exact_artifact_residuals={family.name: {(1, 0): Fraction(1)}},
            variable_count=6,
            rho_index=2,
        )


@pytest.mark.parametrize(
    ("family_name", "constant_value", "candidate_coefficient", "expected_residual"),
    (
        ("W_minus_1_on_X0", -1.0, 0.0, -1.0),
        ("drift_lower_entry_D_0_0", 0.25, -0.5, -0.25),
    ),
)
def test_g10a3_candidate_replay_includes_nonzero_family_constant(
    family_name: str,
    constant_value: float,
    candidate_coefficient: float,
    expected_residual: float,
) -> None:
    from cbf_rover.methods.polynomial_sbc.g10a3_exact_grid_separation import (
        G10A3_GRID_PASS_SCHEDULE,
        build_g10a3_grid_catalog,
        build_g10a3_numeric_point_cut_rows,
        scan_g10a3_exact_grid,
    )

    family = _ConstraintFamily(
        name=family_name,
        root_box_exact=(Fraction(-1), Fraction(1), Fraction(-1), Fraction(1)),
        domain_kind="full",
        basis=({(0, 0): 1.0},),
        constant_value=constant_value,
    )
    catalog = build_g10a3_grid_catalog((family,))
    candidate_values, candidate_values_sha256 = _solver_candidate(
        (candidate_coefficient,),
        variable_count=4,
        rho_index=1,
    )
    scan = scan_g10a3_exact_grid(
        families=(family,),
        exact_coefficient_space_residuals={
            family.name: {(0, 0): Fraction.from_float(expected_residual)}
        },
        exact_artifact_residuals={
            family.name: {(0, 0): Fraction.from_float(expected_residual)}
        },
        candidate_values=candidate_values,
        candidate_values_sha256=candidate_values_sha256,
        variable_count=4,
        rho_index=1,
        candidate_barrier_sha256="4" * 64,
        family_contract_sha256=catalog.family_contract_sha256,
        pass_spec=G10A3_GRID_PASS_SCHEDULE[0],
        expected_point_set_sha256=catalog.expected_point_set_sha256_by_grid["17"],
        existing_constraint_definition_ids=frozenset(),
    )

    _, _, rows = build_g10a3_numeric_point_cut_rows(
        families=(family,),
        cuts=(scan.selected_cuts[0],),
        candidate_barrier_sha256="4" * 64,
        candidate_values=candidate_values,
        candidate_values_sha256=candidate_values_sha256,
        exact_artifact_residuals={
            family.name: {(0, 0): Fraction.from_float(expected_residual)}
        },
        variable_count=4,
        rho_index=1,
    )

    assert rows[0]["candidate_row_replay_residual"] == expected_residual
    assert Fraction(rows[0]["candidate_exact_exclusion_margin"]) == (
        Fraction.from_float(1.0e-8) - Fraction.from_float(expected_residual)
    )
    assert rows[0]["candidate_numeric_violation"] == pytest.approx(
        1.0e-8 - expected_residual
    )
    assert rows[0]["candidate_numeric_violation_float64_hex"] == rows[0][
        "candidate_numeric_violation"
    ].hex()


def test_g10a3_rejects_exact_cut_that_float_row_does_not_exclude() -> None:
    from cbf_rover.methods.polynomial_sbc.g10a3_exact_grid_separation import (
        G10A3_GRID_PASS_SCHEDULE,
        _g10a3_numeric_point_row_replay,
        build_g10a3_grid_catalog,
        scan_g10a3_exact_grid,
    )

    family = _ConstraintFamily(
        name="W_nonnegative_on_X",
        root_box_exact=(Fraction(-1), Fraction(1), Fraction(-1), Fraction(1)),
        domain_kind="full",
        basis=(
            {(0, 0): 1.0e16},
            {(0, 0): -2.0e-8},
            {(0, 0): -1.0e16},
        ),
        constant_value=1.0e-8,
    )
    exact_residual = -Fraction.from_float(1.0e-8)
    catalog = build_g10a3_grid_catalog((family,))
    candidate_values, candidate_values_sha256 = _solver_candidate(
        (1.0, 1.0, 1.0),
        variable_count=7,
        rho_index=3,
    )
    scan = scan_g10a3_exact_grid(
        families=(family,),
        exact_coefficient_space_residuals={
            family.name: {(0, 0): exact_residual}
        },
        exact_artifact_residuals={
            family.name: {(0, 0): exact_residual}
        },
        candidate_values=candidate_values,
        candidate_values_sha256=candidate_values_sha256,
        variable_count=7,
        rho_index=3,
        candidate_barrier_sha256="7" * 64,
        family_contract_sha256=catalog.family_contract_sha256,
        pass_spec=G10A3_GRID_PASS_SCHEDULE[0],
        expected_point_set_sha256=catalog.expected_point_set_sha256_by_grid["17"],
        existing_constraint_definition_ids=frozenset(),
    )

    assert scan.selected_cuts == ()
    _, rhs, lhs, numeric_violation = _g10a3_numeric_point_row_replay(
        exact_row=(
            Fraction.from_float(1.0e16),
            Fraction.from_float(-2.0e-8),
            Fraction.from_float(-1.0e16),
        ),
        constant_exact=Fraction.from_float(1.0e-8),
        candidate_values=candidate_values,
        variable_count=7,
        rho_index=3,
    )
    assert exact_residual < 0
    assert lhs == rhs == 0.0
    assert numeric_violation <= 1.0e-9


def test_g10a3_resource_and_publication_gates_fail_closed() -> None:
    from cbf_rover.methods.polynomial_sbc.g10a3_exact_grid_separation import (
        G10A3_MAX_MASTER_LP_ROWS,
        g10a3_publication_gate,
        g10a3_resource_failures,
    )

    missing_rss = g10a3_resource_failures(
        csr_matrix((1, 1)),
        rss_bytes=None,
    )
    assert "RSS_measurement_available" in missing_rss
    too_many_rows = g10a3_resource_failures(
        csr_matrix((G10A3_MAX_MASTER_LP_ROWS + 1, 1)),
        rss_bytes=1,
    )
    assert "linear_row_cap" in too_many_rows

    passed, failures = g10a3_publication_gate(
        final_grid_gate_passed=True,
        complete_grid_visit_count=42 * 65**2,
        expected_grid_visit_count=42 * 65**2,
        point_set_binding_passed=True,
        cut_ledger_passed=True,
        resource_gate_passed=True,
        full_v3_call_count=1,
        validation_status="validated",
        global_polynomial_status="validated",
        first_slice_ready=True,
        controller_contract_status="validated",
        geometry_contract_status="validated",
        theorem_scalar_gate=True,
        formal_acceptance_tolerance=0.0,
        binding_failures=(),
        v3_release_gate_passed=True,
    )
    assert passed is True
    assert failures == ()

    rejected, rejected_failures = g10a3_publication_gate(
        final_grid_gate_passed=True,
        complete_grid_visit_count=42 * 65**2 - 1,
        expected_grid_visit_count=42 * 65**2,
        point_set_binding_passed=True,
        cut_ledger_passed=True,
        resource_gate_passed=True,
        full_v3_call_count=1,
        validation_status="validated",
        global_polynomial_status="validated",
        first_slice_ready=True,
        controller_contract_status="validated",
        geometry_contract_status="validated",
        theorem_scalar_gate=True,
        formal_acceptance_tolerance=0.0,
        binding_failures=(),
        v3_release_gate_passed=True,
    )
    assert rejected is False
    assert "complete_final_grid_visit_count" in rejected_failures


def test_g10a3_ephemeral_lexicographic_row_uses_g10a3_caps_and_label(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    import cbf_rover.methods.polynomial_sbc.g10a3_exact_grid_separation as g10a3

    monkeypatch.setattr(g10a3, "G10A3_MAX_CSR_NONZEROS", 0)
    monkeypatch.setattr(g10a3, "_peak_rss_bytes", lambda: 1)
    manifest: dict[str, object] = {}
    projection = SimpleNamespace(variable_count=4, t_index=2)

    with pytest.raises(
        g10a3.G10A3GridBudgetExceeded,
        match="g10a3_candidate_01_secondary_ephemeral_lexicographic",
    ):
        g10a3._solve_g10a3_two_phase_projection(
            projection=projection,
            master_matrix=csr_matrix((1, 4)),
            master_right_hand_side=g10a3.np.zeros(1),
            seed_values=g10a3.np.zeros(4),
            candidate_index=1,
            first_solve_index=1,
            cumulative_lp_time_s=0.0,
            stage_deadline=10.0,
            phase_records=[],
            manifest=manifest,
            clock=lambda: 0.0,
        )

    samples = manifest["g10a3_resource_samples"]
    assert isinstance(samples, list)
    assert [sample["label"] for sample in samples] == [
        "g10a3_candidate_01_primary",
        "g10a3_candidate_01_secondary_ephemeral_lexicographic",
    ]
    assert samples[-1]["matrix_row_count"] == 2
    assert samples[-1]["failures"] == ["CSR_nonzero_cap"]


def test_g10a3_cut_ledger_self_validates_and_detects_point_tampering() -> None:
    from cbf_rover.methods.polynomial_sbc.g10a3_exact_grid_separation import (
        G10A3_GRID_PASS_SCHEDULE,
        G10A3CandidateReplaySubject,
        _point_cut_selection_material,
        build_g10a3_cut_ledger_payload,
        build_g10a3_grid_catalog,
        build_g10a3_numeric_point_cut_rows,
        scan_g10a3_exact_grid,
        validate_g10a3_cut_ledger,
    )

    family = _full_family()
    catalog = build_g10a3_grid_catalog((family,))
    candidate_values, candidate_values_sha256 = _solver_candidate(
        (-1.0,),
        variable_count=4,
        rho_index=1,
    )
    scan = scan_g10a3_exact_grid(
        families=(family,),
        exact_coefficient_space_residuals={
            family.name: {(0, 0): Fraction(-1)}
        },
        exact_artifact_residuals={family.name: {(0, 0): Fraction(-1)}},
        candidate_values=candidate_values,
        candidate_values_sha256=candidate_values_sha256,
        variable_count=4,
        rho_index=1,
        candidate_barrier_sha256="3" * 64,
        family_contract_sha256=catalog.family_contract_sha256,
        pass_spec=G10A3_GRID_PASS_SCHEDULE[0],
        expected_point_set_sha256=catalog.expected_point_set_sha256_by_grid["17"],
        existing_constraint_definition_ids=frozenset(),
    )
    _, _, rows = build_g10a3_numeric_point_cut_rows(
        families=(family,),
        cuts=(scan.selected_cuts[0],),
        candidate_barrier_sha256="3" * 64,
        candidate_values=candidate_values,
        candidate_values_sha256=candidate_values_sha256,
        exact_artifact_residuals={family.name: {(0, 0): Fraction(-1)}},
        variable_count=4,
        rho_index=1,
    )
    payload = build_g10a3_cut_ledger_payload(
        grid_catalog_sha256=catalog.content_sha256,
        rows=rows,
    )
    replay_subjects = {
        candidate_values_sha256: G10A3CandidateReplaySubject(
            candidate_barrier_sha256="3" * 64,
            candidate_values=candidate_values,
            candidate_values_sha256=candidate_values_sha256,
            exact_artifact_residuals={
                family.name: {(0, 0): Fraction(-1)}
            },
        )
    }
    assert validate_g10a3_cut_ledger(
        payload,
        expected_grid_catalog_sha256=catalog.content_sha256,
        families=(family,),
        candidate_replay_subjects=replay_subjects,
    ) == ()
    assert rows[0]["candidate_values_sha256"] == candidate_values_sha256
    assert any(value != 0.0 for value in candidate_values[2:])

    changed_suffix = (*candidate_values[:-1], candidate_values[-1] + 1.0)
    changed_subjects = {
        candidate_values_sha256: G10A3CandidateReplaySubject(
            candidate_barrier_sha256="3" * 64,
            candidate_values=changed_suffix,
            candidate_values_sha256=candidate_values_sha256,
            exact_artifact_residuals={
                family.name: {(0, 0): Fraction(-1)}
            },
        )
    }
    assert "row_0_candidate_replay_subject" in validate_g10a3_cut_ledger(
        payload,
        expected_grid_catalog_sha256=catalog.content_sha256,
        families=(family,),
        candidate_replay_subjects=changed_subjects,
    )

    assert "grid_catalog_sha256" in validate_g10a3_cut_ledger(
        payload,
        expected_grid_catalog_sha256="f" * 64,
        families=(family,),
        candidate_replay_subjects=replay_subjects,
    )

    tampered = deepcopy(payload)
    tampered["rows"][0]["point_exact"][0] = "0"
    material = dict(tampered)
    material.pop("content_sha256")
    tampered["content_sha256"] = content_sha256(material)
    assert "row_0_point_sha256" in validate_g10a3_cut_ledger(
        tampered,
        expected_grid_catalog_sha256=catalog.content_sha256,
        families=(family,),
        candidate_replay_subjects=replay_subjects,
    )

    prefix_rows = deepcopy(list(rows))
    prefix_rows[0]["numeric_row_float64_hex"][0] = 7.0.hex()
    prefix_payload = build_g10a3_cut_ledger_payload(
        grid_catalog_sha256=catalog.content_sha256,
        rows=prefix_rows,
    )
    assert "row_0_numeric_row_prefix" in validate_g10a3_cut_ledger(
        prefix_payload,
        expected_grid_catalog_sha256=catalog.content_sha256,
        families=(family,),
        candidate_replay_subjects=replay_subjects,
    )

    rhs_rows = deepcopy(list(rows))
    rhs_rows[0]["right_hand_side_float64_hex"] = 9.0.hex()
    rhs_payload = build_g10a3_cut_ledger_payload(
        grid_catalog_sha256=catalog.content_sha256,
        rows=rhs_rows,
    )
    assert "row_0_right_hand_side" in validate_g10a3_cut_ledger(
        rhs_payload,
        expected_grid_catalog_sha256=catalog.content_sha256,
        families=(family,),
        candidate_replay_subjects=replay_subjects,
    )

    dual_rows = deepcopy(list(rows))
    dual_row = dual_rows[0]
    dual_row["exact_artifact_candidate_residual"] = "-1/2"
    dual_row["exact_artifact_minus_coefficient_space_residual"] = "1/2"
    dual_row["candidate_artifact_replay_residual"] = -0.5
    dual_row["artifact_minus_coefficient_space_residual"] = 0.5
    dual_row["selection_event_sha256"] = content_sha256(
        _point_cut_selection_material(
            definition_id=dual_row["constraint_definition_id"],
            candidate_barrier_sha256=dual_row["candidate_barrier_sha256"],
            candidate_values_sha256=dual_row["candidate_values_sha256"],
            exact_coefficient_space_candidate_residual=Fraction(-1),
            exact_artifact_candidate_residual=Fraction(-1, 2),
            exact_artifact_minus_coefficient_space_residual=Fraction(1, 2),
            candidate_numeric_violation_float64_hex=dual_row[
                "candidate_numeric_violation_float64_hex"
            ],
            pass_id=dual_row["pass_id"],
            point_sha256=dual_row["point_sha256"],
        )
    )
    dual_payload = build_g10a3_cut_ledger_payload(
        grid_catalog_sha256=catalog.content_sha256,
        rows=dual_rows,
    )
    assert "row_0_artifact_subject_replay" in validate_g10a3_cut_ledger(
        dual_payload,
        expected_grid_catalog_sha256=catalog.content_sha256,
        families=(family,),
        candidate_replay_subjects=replay_subjects,
    )


def test_g10a3_disk_release_gate_rejects_every_formal_subject_tamper(
    tmp_path: Path,
) -> None:
    import cbf_rover.methods.polynomial_sbc.g10a3_exact_grid_separation as g10a3
    from cbf_rover.methods.polynomial_sbc.g10a_active_set_lp import (
        G10A_CONTROLLER_CONSTRAINT_SUFFIXES,
    )
    from cbf_rover.methods.polynomial_sbc.polynomial import (
        FrozenPiecewisePolynomialController,
        PolynomialControllerPiece,
        SparsePolynomial2D,
        controller_pieces,
    )
    from cbf_rover.methods.polynomial_sbc.rational_certificate_validation import (
        canonical_rational_model_payload,
    )

    zero = SparsePolynomial2D(((0, 0),), (0.0,))
    controller = FrozenPiecewisePolynomialController(
        controller_id="G10A3_DISK_GATE_TEST_CONTROLLER",
        pieces=(
            PolynomialControllerPiece("only", -1.0, 4.0, zero, zero),
        ),
        construction_source_sha256="a" * 64,
    )
    barrier = SparsePolynomial2D(((0, 0),), (0.25,))
    family = _full_family()
    families = (family,)
    catalog = g10a3.build_g10a3_grid_catalog(families)
    catalog_path = tmp_path / "exact_grid_catalog.json"
    g10a3._write_json(catalog_path, catalog.payload)

    candidate_barrier = SparsePolynomial2D(((0, 0),), (-1.0,))
    candidate_barrier_sha256 = content_sha256(candidate_barrier.payload())
    candidate_values, candidate_values_sha256 = _solver_candidate(
        (-1.0,),
        variable_count=g10a3.G10A3_MAX_DECISION_VARIABLES,
        rho_index=1,
    )
    candidate_directory = tmp_path / "candidate_01"
    candidate_directory.mkdir()
    candidate_path = candidate_directory / "numeric_candidate.json"
    candidate_payload: dict[str, object] = {
        "schema_version": 1,
        "kind": "g10a3_qsbc_numeric_candidate",
        "synthesis_id": g10a3.G10A3_SYNTHESIS_ID,
        "proof_route_id": g10a3.G10A3_PROOF_ROUTE_ID,
        "candidate_index": 1,
        "source_pass_id": "TEST",
        "controller_content_sha256": controller.sha256,
        "chebyshev_barrier": {
            "basis": "total_degree_tensor_chebyshev",
            "indices": [[0, 0]],
            "coefficients_decimal": [-1.0],
            "coefficients_float64_hex": [(-1.0).hex()],
        },
        "barrier": candidate_barrier.payload(),
        "barrier_payload_sha256": candidate_barrier_sha256,
        "full_solver_candidate": {
            "decision_layout": "[Chebyshev_c,rho,t,L1_z]",
            "decision_variable_count": len(candidate_values),
            "values_decimal": list(candidate_values),
            "values_float64_hex": [
                value.hex() for value in candidate_values
            ],
            "values_sha256": candidate_values_sha256,
        },
    }
    candidate_payload["content_sha256"] = content_sha256(candidate_payload)
    g10a3._write_json(candidate_path, candidate_payload)
    candidate_artifact = g10a3.G10A3CandidateDiskArtifact(
        path=candidate_path,
        file_sha256=g10a3.file_sha256(candidate_path),
        content_sha256=str(candidate_payload["content_sha256"]),
        candidate_barrier_sha256=candidate_barrier_sha256,
        candidate_values_sha256=candidate_values_sha256,
    )
    candidate_scan = g10a3.scan_g10a3_exact_grid(
        families=families,
        exact_coefficient_space_residuals={
            family.name: {(0, 0): Fraction(-1)}
        },
        exact_artifact_residuals={
            family.name: {(0, 0): Fraction(-1)}
        },
        candidate_values=candidate_values,
        candidate_values_sha256=candidate_values_sha256,
        variable_count=g10a3.G10A3_MAX_DECISION_VARIABLES,
        rho_index=1,
        candidate_barrier_sha256=candidate_barrier_sha256,
        family_contract_sha256=catalog.family_contract_sha256,
        pass_spec=g10a3.G10A3_GRID_PASS_SCHEDULE[0],
        expected_point_set_sha256=(
            catalog.expected_point_set_sha256_by_grid["17"]
        ),
        existing_constraint_definition_ids=frozenset(),
    )
    _, _, candidate_rows = g10a3.build_g10a3_numeric_point_cut_rows(
        families=families,
        cuts=(candidate_scan.selected_cuts[0],),
        candidate_barrier_sha256=candidate_barrier_sha256,
        candidate_values=candidate_values,
        candidate_values_sha256=candidate_values_sha256,
        exact_artifact_residuals={
            family.name: {(0, 0): Fraction(-1)}
        },
        variable_count=g10a3.G10A3_MAX_DECISION_VARIABLES,
        rho_index=1,
    )
    ledger = g10a3.build_g10a3_cut_ledger_payload(
        grid_catalog_sha256=catalog.content_sha256,
        rows=candidate_rows,
    )
    ledger_path = tmp_path / "selected_point_cut_ledger.json"
    g10a3._write_json(ledger_path, ledger)
    trusted_ledger_file_sha256 = g10a3.file_sha256(ledger_path)

    final_scan = g10a3.G10A3GridScanResult(
        pass_id="G65_FINAL_GATE",
        candidate_barrier_sha256=content_sha256(barrier.payload()),
        visited_point_count=65**2,
        active_point_count=65**2,
        margin_violating_point_count=0,
        coefficient_space_negative_point_count=0,
        negative_point_count=0,
        minimum_exact_coefficient_space_residual=Fraction(1, 4),
        minimum_exact_residual=Fraction(1, 4),
        scanned_point_set_sha256=(
            catalog.expected_point_set_sha256_by_grid["65"]
        ),
        expected_point_set_sha256=(
            catalog.expected_point_set_sha256_by_grid["65"]
        ),
        point_set_binding_passed=True,
        whole_catalog_traversed=True,
        selected_cuts=(),
        gate_passed=True,
    )
    final_grid = g10a3._grid_scan_payload(final_scan)
    final_grid_path = tmp_path / "final_G65_exact_grid.json"
    g10a3._write_json(final_grid_path, final_grid)

    validation_directory = tmp_path / "independent_validation_v3"
    validation_directory.mkdir()

    def tree_summary(
        name: str,
        *,
        root_box_exact: list[str] | None = None,
    ) -> dict[str, object]:
        tree: dict[str, object] = {
            "status": "validated",
            "leaves": [],
        }
        if root_box_exact is not None:
            tree["root_box_exact"] = root_box_exact
        tree_path = validation_directory / f"{name}.json"
        tree_path.write_text(
            json.dumps(tree, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        summary = dict(tree)
        summary.pop("leaves")
        summary["tree_file"] = tree_path.name
        summary["tree_file_sha256"] = hashlib.sha256(
            tree_path.read_bytes()
        ).hexdigest()
        return summary

    constraints = {
        family.name: tree_summary(
            "qsbc_tree",
            root_box_exact=[str(value) for value in family.root_box_exact],
        )
    }
    controller_constraints = {
        name: tree_summary(f"controller_tree_{index:02d}")
        for index, name in enumerate(
            sorted(
                f"{piece.piece_id}_{suffix}"
                for piece in controller_pieces(controller)
                for suffix in G10A_CONTROLLER_CONSTRAINT_SUFFIXES
            )
        )
    }
    model_payload = canonical_rational_model_payload()
    validation: dict[str, object] = {
        "schema_version": 2,
        "validator_id": g10a3.G10_VALIDATOR_ID,
        "status": "validated",
        "global_polynomial_status": "validated",
        "first_slice_ready": True,
        "formal_acceptance_tolerance": 0.0,
        "controller_content_sha256": controller.sha256,
        "barrier_payload_sha256": content_sha256(barrier.payload()),
        "barrier_degree": barrier.degree,
        "barrier_term_count": len(barrier.powers),
        "q_float64_hex": g10a3.G10_Q.hex(),
        "q_exact": str(Fraction.from_float(g10a3.G10_Q)),
        "epsbar_float64_hex": g10a3.G10_EPSBAR.hex(),
        "epsbar_exact": str(Fraction.from_float(g10a3.G10_EPSBAR)),
        "theorem_scalar_gate": True,
        "geometry_contract_status": "validated",
        "controller_contract_status": "validated",
        "formal_model_contract": model_payload,
        "formal_model_contract_sha256": content_sha256(model_payload),
        "independence": {
            "solver_status_read": False,
            "sampled_residual_used": False,
            "float_polynomial_algebra_used_for_status": False,
            "exact_binary64_artifact_coefficients": True,
            "exact_decimal_model_constants": True,
            "controller_S1_and_D1_replayed": True,
        },
        "constraints": constraints,
        "controller_constraints": controller_constraints,
    }
    validation["report_content_sha256"] = content_sha256(validation)
    validation_path = validation_directory / "validation_report.json"
    g10a3._write_json(validation_path, validation)

    pending_path, certificate = g10a3._write_g10a3_pending_certificate(
        destination=tmp_path,
        barrier=barrier,
        controller=controller,
        validation=validation,
        validation_path=validation_path,
        solver_backend="test_backend",
        selected_point_row_count=1,
        selected_cut_ledger_file_sha256=trusted_ledger_file_sha256,
        selected_cut_ledger_content_sha256=str(ledger["content_sha256"]),
        grid_catalog_sha256=catalog.content_sha256,
        v3_root_set_sha256=str(catalog.payload["v3_root_set_sha256"]),
        final_grid_report_sha256=str(final_grid["report_content_sha256"]),
    )
    assert certificate.solver["selected_cut_ledger_file_sha256"] == (
        trusted_ledger_file_sha256
    )
    assert certificate.solver["selected_cut_ledger_content_sha256"] == (
        ledger["content_sha256"]
    )
    gate_arguments = {
        "validation_path": validation_path,
        "ledger_path": ledger_path,
        "grid_catalog_path": catalog_path,
        "final_grid_path": final_grid_path,
        "pending_certificate_path": pending_path,
        "expected_validation_file_sha256": g10a3.file_sha256(
            validation_path
        ),
        "expected_validation_content_sha256": validation[
            "report_content_sha256"
        ],
        "expected_ledger_payload": ledger,
        "expected_ledger_file_sha256": trusted_ledger_file_sha256,
        "expected_ledger_content_sha256": ledger["content_sha256"],
        "expected_ledger_row_count": len(candidate_rows),
        "expected_grid_catalog_file_sha256": g10a3.file_sha256(
            catalog_path
        ),
        "expected_final_grid_file_sha256": g10a3.file_sha256(
            final_grid_path
        ),
        "expected_final_grid_content_sha256": final_grid[
            "report_content_sha256"
        ],
        "expected_pending_certificate_file_sha256": g10a3.file_sha256(
            pending_path
        ),
        "expected_certificate": certificate,
        "expected_grid_catalog": catalog,
        "expected_final_scan": final_scan,
        "families": families,
        "expected_candidate_artifacts": {
            (
                candidate_barrier_sha256,
                candidate_values_sha256,
            ): candidate_artifact
        },
        "barrier": barrier,
        "controller": controller,
        "full_v3_call_count": 1,
        "resource_gate_passed": True,
    }

    empty_ledger = g10a3.build_g10a3_cut_ledger_payload(
        grid_catalog_sha256=catalog.content_sha256,
        rows=(),
    )
    assert empty_ledger["row_count"] == 0
    assert g10a3.validate_g10a3_cut_ledger(
        empty_ledger,
        expected_grid_catalog_sha256=catalog.content_sha256,
        families=families,
        candidate_replay_subjects={},
    ) == ()
    g10a3._write_json(ledger_path, empty_ledger)
    passed, failures = g10a3._g10a3_publication_disk_release_gate(
        **gate_arguments
    )
    assert not passed
    assert "cut_ledger_subject_changed" in failures
    assert "cut_ledger_row_count_changed" in failures
    g10a3._write_json(ledger_path, ledger)

    replacement_certificate = g10a3.QSBCertificate(
        certificate_id="G10A3_SELF_HASHED_FIRST_LOAD_REPLACEMENT",
        barrier=SparsePolynomial2D(((0, 0),), (0.5,)),
        q=certificate.q,
        epsbar=certificate.epsbar,
        controller_content_sha256=controller.sha256,
        solver=dict(certificate.solver),
        validation=dict(certificate.validation),
        schema_version=2,
    )
    replacement_certificate.save(pending_path)
    passed, failures = g10a3._g10a3_publication_disk_release_gate(
        **gate_arguments
    )
    assert not passed
    assert "pending_certificate_trusted_barrier_object" in failures

    certificate.save(pending_path)
    gate_arguments["expected_pending_certificate_file_sha256"] = (
        g10a3.file_sha256(pending_path)
    )
    passed, failures = g10a3._g10a3_publication_disk_release_gate(
        **gate_arguments
    )
    assert passed
    assert failures == ()

    first_tree_path = validation_directory / str(
        constraints[family.name]["tree_file"]
    )
    original_bytes = {
        path: path.read_bytes()
        for path in (
            pending_path,
            validation_path,
            first_tree_path,
            candidate_path,
            ledger_path,
            catalog_path,
            final_grid_path,
        )
    }
    tampered_pending = json.loads(
        original_bytes[pending_path].decode("utf-8")
    )
    tampered_pending["solver"]["selected_cut_ledger_file_sha256"] = (
        "f" * 64
    )
    tampered_pending.pop("content_sha256")
    tampered_pending["content_sha256"] = content_sha256(tampered_pending)
    tampered_pending_bytes = (
        json.dumps(tampered_pending, indent=2, sort_keys=True) + "\n"
    ).encode("utf-8")
    cases = (
        (
            pending_path,
            tampered_pending_bytes,
            "pending_certificate_ledger_file_binding",
        ),
        (validation_path, b"{}\n", "validation_report_file_changed"),
        (first_tree_path, b"{}\n", "tree_file_sha256"),
        (candidate_path, b"{}\n", "candidate_"),
        (ledger_path, b"{}\n", "cut_ledger"),
        (catalog_path, b"{}\n", "grid_catalog"),
        (final_grid_path, b"{}\n", "final_grid"),
    )
    for path_to_tamper, replacement, expected_failure in cases:
        path_to_tamper.write_bytes(replacement)
        passed, failures = g10a3._g10a3_publication_disk_release_gate(
            **gate_arguments
        )
        assert not passed
        assert any(expected_failure in failure for failure in failures)
        path_to_tamper.write_bytes(original_bytes[path_to_tamper])

    final_certificate_path = g10a3.finalize_g10a3_pending_certificate(
        pending_path=pending_path,
        certificate_path=tmp_path / "certificate.json",
        expected_certificate=certificate,
        controller=controller,
    )
    assert g10a3.QSBCertificate.load(
        final_certificate_path,
        controller=controller,
    ) == certificate


def test_g10a3_nested_grids_do_not_reselect_the_same_exact_point() -> None:
    from cbf_rover.methods.polynomial_sbc.g10a3_exact_grid_separation import (
        G10A3_GRID_PASS_SCHEDULE,
        G10A3GridPassSpec,
        build_g10a3_grid_catalog,
        scan_g10a3_exact_grid,
    )

    family = _full_family()
    catalog = build_g10a3_grid_catalog((family,))
    candidate_values, candidate_values_sha256 = _solver_candidate(
        (-1.0,),
        variable_count=4,
        rho_index=1,
    )
    first = scan_g10a3_exact_grid(
        families=(family,),
        exact_coefficient_space_residuals={
            family.name: {(0, 0): Fraction(-1)}
        },
        exact_artifact_residuals={family.name: {(0, 0): Fraction(-1)}},
        candidate_values=candidate_values,
        candidate_values_sha256=candidate_values_sha256,
        variable_count=4,
        rho_index=1,
        candidate_barrier_sha256="4" * 64,
        family_contract_sha256=catalog.family_contract_sha256,
        pass_spec=G10A3_GRID_PASS_SCHEDULE[0],
        expected_point_set_sha256=catalog.expected_point_set_sha256_by_grid["17"],
        existing_constraint_definition_ids=frozenset(),
    )
    second = scan_g10a3_exact_grid(
        families=(family,),
        exact_coefficient_space_residuals={
            family.name: {(0, 0): Fraction(-1)}
        },
        exact_artifact_residuals={family.name: {(0, 0): Fraction(-1)}},
        candidate_values=candidate_values,
        candidate_values_sha256=candidate_values_sha256,
        variable_count=4,
        rho_index=1,
        candidate_barrier_sha256="5" * 64,
        family_contract_sha256=catalog.family_contract_sha256,
        pass_spec=G10A3_GRID_PASS_SCHEDULE[1],
        expected_point_set_sha256=catalog.expected_point_set_sha256_by_grid["33"],
        existing_constraint_definition_ids=frozenset(
            cut.constraint_definition_id for cut in first.selected_cuts
        ),
    )

    first_points = {cut.point_exact for cut in first.selected_cuts}
    second_points = {cut.point_exact for cut in second.selected_cuts}
    assert len(second.selected_cuts) == 16
    assert first_points.isdisjoint(second_points)

    all_strata = G10A3GridPassSpec("G17_ALL_STRATA", 17, 4, 16)
    stratum_winners = scan_g10a3_exact_grid(
        families=(family,),
        exact_coefficient_space_residuals={
            family.name: {(0, 0): Fraction(-1)}
        },
        exact_artifact_residuals={family.name: {(0, 0): Fraction(-1)}},
        candidate_values=candidate_values,
        candidate_values_sha256=candidate_values_sha256,
        variable_count=4,
        rho_index=1,
        candidate_barrier_sha256="6" * 64,
        family_contract_sha256=catalog.family_contract_sha256,
        pass_spec=all_strata,
        expected_point_set_sha256=catalog.expected_point_set_sha256_by_grid["17"],
        existing_constraint_definition_ids=frozenset(),
    )
    replacements = scan_g10a3_exact_grid(
        families=(family,),
        exact_coefficient_space_residuals={
            family.name: {(0, 0): Fraction(-1)}
        },
        exact_artifact_residuals={family.name: {(0, 0): Fraction(-1)}},
        candidate_values=candidate_values,
        candidate_values_sha256=candidate_values_sha256,
        variable_count=4,
        rho_index=1,
        candidate_barrier_sha256="7" * 64,
        family_contract_sha256=catalog.family_contract_sha256,
        pass_spec=all_strata,
        expected_point_set_sha256=catalog.expected_point_set_sha256_by_grid["17"],
        existing_constraint_definition_ids=frozenset(
            cut.constraint_definition_id for cut in stratum_winners.selected_cuts
        ),
    )
    assert len(replacements.selected_cuts) == 16
    assert {
        cut.point_exact for cut in stratum_winners.selected_cuts
    }.isdisjoint({cut.point_exact for cut in replacements.selected_cuts})


def test_g10a3_schedule_fails_closed_without_publishing_on_missing_input(
    tmp_path,
) -> None:
    from cbf_rover.methods.polynomial_sbc.g10a3_exact_grid_separation import (
        synthesize_g10a3_qsbc_schedule,
    )

    output = tmp_path / "g10a3"
    manifest_path = synthesize_g10a3_qsbc_schedule(
        frozen_controller_path=tmp_path / "missing-controller.json",
        g9_seed_path=tmp_path / "missing-seed.json",
        output_dir=output,
    )
    payload = json.loads(manifest_path.read_text(encoding="utf-8"))

    assert payload["status"] == "not_ready"
    assert payload["terminal_reason"] == "G10A3_execution_error_fail_closed"
    assert payload["execution_error"]["type"] == "FileNotFoundError"
    assert payload["hard_resource_caps"][
        "catalog_generation_point_yields"
    ] == 235326
    assert payload["hard_resource_caps"][
        "max_candidate_scan_point_visits"
    ] == 412776
    assert payload["hard_resource_caps"][
        "max_total_grid_point_yields"
    ] == 648102
    assert "max_grid_point_visits" not in payload["hard_resource_caps"]
    assert not (output / "certificate.pending.json").exists()
    assert not (output / "certificate.json").exists()


def test_g10a3_yaml_freezes_the_same_finite_caps_as_the_module() -> None:
    from cbf_rover.methods.polynomial_sbc.g10a3_exact_grid_separation import (
        G10A3_CATALOG_GENERATION_POINT_YIELDS,
        G10A3_MAX_CANDIDATE_SCAN_POINT_VISITS,
        G10A3_MAX_HARD_POINT_ROWS,
        G10A3_MAX_MASTER_LP_ROWS,
        G10A3_MAX_TOTAL_GRID_POINT_YIELDS,
    )

    root = Path(__file__).resolve().parents[2]
    payload = yaml.safe_load(
        (root / "configs/certificates/g10a3_qsbc.yaml").read_text(
            encoding="utf-8"
        )
    )

    assert payload["synthesis"]["barrier_degree"] == 16
    assert payload["synthesis"]["fixed_LP_solve_count"] == 8
    assert payload["resource_caps"]["catalog_generation_point_yields"] == (
        G10A3_CATALOG_GENERATION_POINT_YIELDS
    )
    assert payload["resource_caps"]["max_candidate_scan_point_visits"] == (
        G10A3_MAX_CANDIDATE_SCAN_POINT_VISITS
    )
    assert payload["resource_caps"]["max_total_grid_point_yields"] == (
        G10A3_MAX_TOTAL_GRID_POINT_YIELDS
    )
    assert payload["resource_caps"]["max_selected_point_rows"] == (
        G10A3_MAX_HARD_POINT_ROWS
    )
    assert payload["resource_caps"]["max_master_rows"] == (
        G10A3_MAX_MASTER_LP_ROWS
    )
    assert payload["independent_validation"]["max_full_V3_calls"] == 1
    assert payload["acceptance"]["exact_grid_is_formal_evidence"] is False
    assert payload["exact_grid_preseparation"]["residual_spaces"][
        "preseparation_selection_uses"
    ] == "coefficient_space"
    assert payload["exact_grid_preseparation"]["residual_spaces"][
        "final_G65_gate_uses"
    ] == "artifact_space_only"
    assert payload["exact_grid_preseparation"]["selection"][
        "candidate_row_numeric_replay"
    ] == "final_float64_row_at_full_candidate_minus_float64_RHS"
    assert payload["synthesis"]["projection_objective"][
        "primary_and_ephemeral_secondary_rows_use_G10A3_resource_caps"
    ] is True
    assert payload["exact_grid_preseparation"]["selection"][
        "full_candidate_vector_dimension"
    ] == 308
    assert payload["exact_grid_preseparation"]["provenance"][
        "family_contract_binds_normalized_full_basis_coefficients"
    ] is True
    assert payload["acceptance"]["publication_transaction"][
        "final_certificate_reloaded_after_atomic_rename"
    ] is True
