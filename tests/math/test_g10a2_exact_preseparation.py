from __future__ import annotations

from copy import deepcopy
from fractions import Fraction
from pathlib import Path

import numpy as np
import pytest
from scipy.sparse import csr_matrix

from cbf_rover.entrypoints.research import build_parser
from cbf_rover.methods.polynomial_sbc.g10a2_exact_preseparation import (
    G10A2_EXACT_GATE_DEPTH,
    G10A2_MAX_MASTER_LP_ROWS,
    G10A2_MAX_FULL_V3_CALLS,
    G10A2_PASS_SCHEDULE,
    G10A2_PROOF_ROUTE_ID,
    G10A2_STAGE,
    G10A2PreseparatorBudgetExceeded,
    _build_bound_numeric_cut_rows,
    build_g10a2_cut_ledger_payload,
    build_g10a2_geometry_catalog,
    build_g10a2_numeric_cut_rows,
    build_g10a2_projection_problem,
    cleanup_g10a2_publication_artifacts,
    exact_bernstein_tensor,
    finalize_g10a2_pending_certificate,
    g10a2_publication_gate,
    g10a2_leaf_set_binding_gate,
    g10a2_leaf_set_sha256,
    g10a2_resource_failures,
    scan_exact_bernstein_catalog,
    split_exact_bernstein_tensor,
    validate_g10a2_cut_ledger,
)
from cbf_rover.methods.polynomial_sbc.g10a_active_set_lp import (
    G10A_FEEDBACK_NUMERIC_MARGIN,
    G10A_PROOF_ROUTE_ID,
    _ConstraintFamily,
)
from cbf_rover.methods.polynomial_sbc.polynomial import content_sha256
from cbf_rover.methods.polynomial_sbc.rational_certificate_validation import (
    exact_rational_bernstein_bounds,
)


def _full_family() -> _ConstraintFamily:
    return _ConstraintFamily(
        name="worked_linear_family",
        root_box_exact=(
            Fraction(-1),
            Fraction(1),
            Fraction(-1),
            Fraction(1),
        ),
        domain_kind="full",
        basis=({(0, 0): 1.0}, {(1, 0): 1.0}),
        constant_value=0.0,
    )


def test_g10a2_cli_is_independent_and_requires_the_bound_seed() -> None:
    args = build_parser().parse_args(
        [
            "solve-qsbc-g10a2",
            "--controller",
            "controller.json",
            "--g9-seed",
            "g10a_g9_seed_bundle.json",
            "--output-dir",
            "results",
        ]
    )

    assert args.command == "solve-qsbc-g10a2"
    assert args.controller == "controller.json"
    assert args.g9_seed == "g10a_g9_seed_bundle.json"
    assert args.output_dir == "results"


def test_g10a2_schedule_is_finite_w16_only_and_has_one_full_v3() -> None:
    assert (
        G10A2_STAGE.stage_id,
        G10A2_STAGE.barrier_degree,
        G10A2_STAGE.max_lp_solves,
    ) == ("W16_EXACT_PRESEPARATION", 16, 8)
    assert [
        (item.pass_id, item.depth, item.max_new_cuts_per_active_box)
        for item in G10A2_PASS_SCHEDULE
    ] == [
        ("D3_ONE_PER_BOX", 3, 1),
        ("D4_TWO_PER_BOX", 4, 2),
        ("D5_FOUR_PER_BOX", 5, 4),
    ]
    assert G10A2_EXACT_GATE_DEPTH == 5
    assert G10A2_MAX_FULL_V3_CALLS == 1
    assert G10A2_PROOF_ROUTE_ID != G10A_PROOF_ROUTE_ID


def test_exact_bernstein_tensor_and_midpoint_split_match_worked_values() -> None:
    tensor = exact_bernstein_tensor(
        {(1, 0): Fraction(1)},
        (
            Fraction(-1),
            Fraction(1),
            Fraction(-1),
            Fraction(1),
        ),
        target_degree_x=1,
        target_degree_y=0,
    )

    assert tensor == ((Fraction(-1),), (Fraction(1),))
    left, right = split_exact_bernstein_tensor(tensor, axis="x")
    assert left == ((Fraction(-1),), (Fraction(0),))
    assert right == ((Fraction(0),), (Fraction(1),))


def test_exact_tensor_matches_existing_exact_bounds_on_multiple_inputs() -> None:
    cases = (
        (
            {
                (0, 0): Fraction(3, 7),
                (1, 0): Fraction(-2, 5),
                (0, 2): Fraction(4, 9),
                (2, 1): Fraction(-1, 6),
            },
            (Fraction(-1), Fraction(1), Fraction(-2), Fraction(3)),
        ),
        (
            {
                (0, 0): Fraction(-5, 11),
                (3, 0): Fraction(7, 13),
                (1, 2): Fraction(2, 17),
            },
            (
                Fraction(-3, 4),
                Fraction(5, 8),
                Fraction(-7, 9),
                Fraction(4, 5),
            ),
        ),
    )
    for polynomial, box in cases:
        tensor = exact_bernstein_tensor(polynomial, box)
        values = tuple(value for row in tensor for value in row)
        assert (min(values), max(values)) == exact_rational_bernstein_bounds(
            polynomial,
            box,
        )


def test_exact_tensor_y_split_and_degree_elevation_are_indexed_correctly() -> None:
    y_tensor = exact_bernstein_tensor(
        {(0, 1): Fraction(1)},
        (Fraction(0), Fraction(1), Fraction(0), Fraction(1)),
    )
    lower, upper = split_exact_bernstein_tensor(y_tensor, axis="y")
    assert lower == ((Fraction(0), Fraction(1, 2)),)
    assert upper == ((Fraction(1, 2), Fraction(1)),)

    elevated = exact_bernstein_tensor(
        {(1, 0): Fraction(1)},
        (Fraction(0), Fraction(1), Fraction(0), Fraction(1)),
        target_degree_x=2,
        target_degree_y=0,
    )
    assert elevated == (
        (Fraction(0),),
        (Fraction(1, 2),),
        (Fraction(1),),
    )


def test_exact_catalog_scan_selects_only_a_negative_coefficient() -> None:
    scan = scan_exact_bernstein_catalog(
        family_name="worked_linear_family",
        family_contract_sha256="1" * 64,
        polynomial={(1, 0): Fraction(1)},
        root_box=(
            Fraction(-1),
            Fraction(1),
            Fraction(-1),
            Fraction(1),
        ),
        classify_domain=lambda _box: "inside",
        depth=1,
        max_new_cuts_per_active_box=1,
        candidate_barrier_sha256="2" * 64,
        target_degree_x=1,
        target_degree_y=0,
    )

    assert scan.gate_passed is False
    assert scan.visited_leaf_count == 2
    assert scan.active_leaf_count == 2
    assert scan.negative_coefficient_count == 1
    assert scan.minimum_exact_coefficient == Fraction(-1)
    assert len(scan.selected_cuts) == 1
    cut = scan.selected_cuts[0]
    assert cut.box_exact == (
        Fraction(-1),
        Fraction(0),
        Fraction(-1),
        Fraction(1),
    )
    assert cut.bernstein_index == (0, 0)
    assert cut.exact_candidate_coefficient == Fraction(-1)
    assert cut.rho_coefficient_is_zero
    assert not cut.formal_claim_allowed


def test_selection_event_binds_candidate_but_definition_does_not() -> None:
    arguments = {
        "family_name": "worked_linear_family",
        "family_contract_sha256": "7" * 64,
        "polynomial": {(1, 0): Fraction(1)},
        "root_box": (
            Fraction(-1),
            Fraction(1),
            Fraction(-1),
            Fraction(1),
        ),
        "classify_domain": lambda _box: "inside",
        "depth": 1,
        "max_new_cuts_per_active_box": 1,
        "target_degree_x": 1,
        "target_degree_y": 0,
    }
    first = scan_exact_bernstein_catalog(
        **arguments,
        candidate_barrier_sha256="8" * 64,
    ).selected_cuts[0]
    second = scan_exact_bernstein_catalog(
        **arguments,
        candidate_barrier_sha256="9" * 64,
    ).selected_cuts[0]

    assert first.constraint_definition_id == second.constraint_definition_id
    assert first.selection_event_sha256 != second.selection_event_sha256


def test_catalog_scans_boundary_and_skips_outside_without_skipping_leaves() -> None:
    scan = scan_exact_bernstein_catalog(
        family_name="worked_linear_family",
        family_contract_sha256="a" * 64,
        polynomial={(1, 0): Fraction(-1)},
        root_box=(
            Fraction(-1),
            Fraction(1),
            Fraction(-1),
            Fraction(1),
        ),
        classify_domain=(
            lambda box: "outside" if box[1] <= 0 else "boundary"
        ),
        depth=1,
        max_new_cuts_per_active_box=1,
        candidate_barrier_sha256="b" * 64,
        target_degree_x=1,
        target_degree_y=0,
    )

    assert scan.visited_leaf_count == 2
    assert scan.active_leaf_count == 1
    assert scan.inspected_coefficient_count == 2
    assert scan.negative_coefficient_count == 1
    assert scan.selected_cuts[0].domain_class == "boundary"


def test_final_d5_mode_scans_every_leaf_even_when_every_coefficient_passes() -> None:
    scan = scan_exact_bernstein_catalog(
        family_name="worked_linear_family",
        family_contract_sha256="c" * 64,
        polynomial={(0, 0): Fraction(0)},
        root_box=(
            Fraction(-1),
            Fraction(1),
            Fraction(-1),
            Fraction(1),
        ),
        classify_domain=lambda _box: "inside",
        depth=5,
        max_new_cuts_per_active_box=0,
        candidate_barrier_sha256="d" * 64,
    )

    assert scan.visited_leaf_count == 32
    assert scan.active_leaf_count == 32
    assert scan.inspected_coefficient_count == 32
    assert scan.negative_coefficient_count == 0
    assert scan.selected_cuts == ()
    assert scan.gate_passed


def test_g10a2_numeric_cut_has_original_residual_direction_and_zero_rho() -> None:
    family = _full_family()
    scan = scan_exact_bernstein_catalog(
        family_name=family.name,
        family_contract_sha256="3" * 64,
        polynomial={(1, 0): Fraction(1)},
        root_box=family.root_box_exact,
        classify_domain=lambda _box: "inside",
        depth=1,
        max_new_cuts_per_active_box=1,
        candidate_barrier_sha256="4" * 64,
        target_degree_x=1,
        target_degree_y=0,
    )

    matrix, rhs = build_g10a2_numeric_cut_rows(
        family=family,
        cuts=scan.selected_cuts,
        variable_count=6,
        rho_index=2,
    )

    assert matrix.shape == (1, 6)
    assert matrix.toarray()[0].tolist() == [-1.0, 1.0, 0.0, 0.0, 0.0, 0.0]
    assert rhs.tolist() == [-G10A_FEEDBACK_NUMERIC_MARGIN]
    assert np.count_nonzero(matrix.toarray()[:, 2:]) == 0


def test_projection_objective_cannot_recreate_rho_or_pure_l1_degeneracy() -> None:
    anchors = csr_matrix(np.asarray([[1.0, -2.0, 1.0]]))
    problem = build_g10a2_projection_problem(
        seed_coefficients=(0.002, -0.003),
        anchor_core_matrix=anchors,
        anchor_right_hand_side=np.asarray([1.0]),
    )

    assert problem.variable_count == 6
    assert (problem.rho_index, problem.t_index, problem.z_start_index) == (2, 3, 4)
    assert problem.bounds[problem.rho_index] == (0.0, 0.0)
    assert problem.primary_objective.tolist() == [0.0, 0.0, 0.0, 1.0, 0.0, 0.0]
    assert problem.secondary_objective.tolist() == [0.0, 0.0, 0.0, 0.0, 1.0, 1.0]
    assert np.count_nonzero(problem.matrix.toarray()[0, 3:]) == 0
    assert problem.regularizer_row_count == 8
    assert np.allclose(problem.seed_values, [0.002, -0.003, 0.0, 0.0, 0.0, 0.0])


def test_g10a2_geometry_catalog_is_canonical_and_finite() -> None:
    first = build_g10a2_geometry_catalog((_full_family(),))
    second = build_g10a2_geometry_catalog((_full_family(),))

    assert first.content_sha256 == second.content_sha256
    assert first.family_contract_sha256 == second.family_contract_sha256
    assert first.depth_box_counts == {"3": 8, "4": 16, "5": 32}
    assert first.total_box_count == 56
    assert not first.formal_claim_allowed


def test_leaf_set_hash_distinguishes_equal_count_different_sets() -> None:
    first = g10a2_leaf_set_sha256(
        depth=3,
        leaf_identity_sha256s=("1" * 64, "2" * 64),
    )
    second = g10a2_leaf_set_sha256(
        depth=3,
        leaf_identity_sha256s=("1" * 64, "3" * 64),
    )

    assert first != second


def test_runtime_scan_matches_catalog_leaf_set_provenance() -> None:
    catalog = build_g10a2_geometry_catalog((_full_family(),))
    scan = scan_exact_bernstein_catalog(
        family_name="worked_linear_family",
        family_contract_sha256=catalog.family_contract_sha256,
        polynomial={(0, 0): Fraction(0)},
        root_box=_full_family().root_box_exact,
        classify_domain=lambda _box: "inside",
        depth=3,
        max_new_cuts_per_active_box=0,
        candidate_barrier_sha256="4" * 64,
    )
    expected = catalog.expected_leaf_set_sha256_by_depth["3"]
    passed, failures = g10a2_leaf_set_binding_gate(
        expected_leaf_set_sha256=expected,
        scanned_leaf_set_sha256=scan.scanned_leaf_set_sha256,
        expected_leaf_count=8,
        scanned_leaf_count=scan.visited_leaf_count,
    )

    assert passed
    assert failures == ()


def test_frozen_42_family_catalog_binds_the_v3_root_set() -> None:
    families = tuple(
        _ConstraintFamily(
            name=f"family_{index:02d}",
            root_box_exact=(
                Fraction(-1),
                Fraction(1),
                Fraction(-1),
                Fraction(1),
            ),
            domain_kind="full",
            basis=({(0, 0): 1.0},),
            constant_value=0.0,
        )
        for index in range(42)
    )
    catalog = build_g10a2_geometry_catalog(families)
    changed_families = (*families[:-1], _full_family())
    changed = build_g10a2_geometry_catalog(changed_families)

    assert catalog.payload["family_contract"]["family_count"] == 42
    assert catalog.depth_box_counts == {
        "3": 42 * 8,
        "4": 42 * 16,
        "5": 42 * 32,
    }
    assert len(catalog.payload["v3_root_set"]["roots"]) == 42
    assert catalog.payload["v3_root_set_sha256"] != changed.payload[
        "v3_root_set_sha256"
    ]


def test_cut_ledger_replays_exact_to_float_and_detects_subject_tamper() -> None:
    family = _full_family()
    candidate_hash = "e" * 64
    scan = scan_exact_bernstein_catalog(
        family_name=family.name,
        family_contract_sha256="f" * 64,
        polynomial={(1, 0): Fraction(1)},
        root_box=family.root_box_exact,
        classify_domain=lambda _box: "inside",
        depth=1,
        max_new_cuts_per_active_box=1,
        candidate_barrier_sha256=candidate_hash,
        target_degree_x=1,
        target_degree_y=0,
    )
    matrix, _, rows = _build_bound_numeric_cut_rows(
        families=(family,),
        residuals={family.name: {(1, 0): Fraction(1)}},
        cuts=scan.selected_cuts,
        candidate_barrier_sha256=candidate_hash,
        candidate_chebyshev_coefficients=(0.0, 1.0),
        variable_count=6,
        rho_index=2,
    )
    payload = build_g10a2_cut_ledger_payload(
        geometry_catalog_sha256="0" * 64,
        rows=rows,
    )

    assert matrix.toarray()[0, 2:].tolist() == [0.0, 0.0, 0.0, 0.0]
    assert rows[0]["exact_to_float_projection_replay_passed"] is True
    assert validate_g10a2_cut_ledger(payload) == ()

    tampered = deepcopy(payload)
    tampered["rows"][0]["candidate_barrier_sha256"] = "1" * 64
    material = dict(tampered)
    material.pop("ledger_content_sha256")
    tampered["ledger_content_sha256"] = content_sha256(material)
    failures = validate_g10a2_cut_ledger(tampered)
    assert "row_0_selection_event_sha256" in failures

    tampered_box = deepcopy(payload)
    tampered_box["rows"][0]["box_exact"][0] = "-2"
    material = dict(tampered_box)
    material.pop("ledger_content_sha256")
    tampered_box["ledger_content_sha256"] = content_sha256(material)
    assert "row_0_box_sha256" in validate_g10a2_cut_ledger(tampered_box)


def test_resource_caps_fail_closed_for_missing_rss_or_excess_rows() -> None:
    missing_rss = g10a2_resource_failures(
        csr_matrix((1, 1)),
        rss_bytes=None,
    )
    too_many_rows = g10a2_resource_failures(
        csr_matrix((G10A2_MAX_MASTER_LP_ROWS + 1, 1)),
        rss_bytes=0,
    )

    assert "RSS_measurement_available" in missing_rss
    assert "RSS_cap" in missing_rss
    assert "linear_row_cap" in too_many_rows


def test_g10a2_scan_fails_closed_when_the_fixed_budget_expires() -> None:
    clock_values = iter((0.0, 2.0))

    with pytest.raises(G10A2PreseparatorBudgetExceeded):
        scan_exact_bernstein_catalog(
            family_name="worked_linear_family",
            family_contract_sha256="5" * 64,
            polynomial={(1, 0): Fraction(1)},
            root_box=(
                Fraction(-1),
                Fraction(1),
                Fraction(-1),
                Fraction(1),
            ),
            classify_domain=lambda _box: "inside",
            depth=1,
            max_new_cuts_per_active_box=1,
            candidate_barrier_sha256="6" * 64,
            target_degree_x=1,
            target_degree_y=0,
            deadline=1.0,
            clock=lambda: next(clock_values),
        )


def test_preseparator_can_never_publish_without_the_unique_full_v3() -> None:
    passed, failures = g10a2_publication_gate(
        preseparator_gate_passed=True,
        full_v3_call_count=0,
        validation_status="validated",
        first_slice_ready=True,
        binding_failures=(),
        v3_release_gate_passed=True,
    )
    assert not passed
    assert "exactly_one_full_v3_call" in failures

    passed, failures = g10a2_publication_gate(
        preseparator_gate_passed=False,
        full_v3_call_count=1,
        validation_status="validated",
        first_slice_ready=True,
        binding_failures=(),
        v3_release_gate_passed=True,
    )
    assert not passed
    assert "preseparator_gate_passed" in failures

    passed, failures = g10a2_publication_gate(
        preseparator_gate_passed=True,
        full_v3_call_count=1,
        validation_status="validated",
        first_slice_ready=True,
        binding_failures=(),
        v3_release_gate_passed=True,
    )
    assert passed
    assert failures == ()


def test_pending_certificate_becomes_formal_only_by_atomic_finalize(
    tmp_path: Path,
) -> None:
    pending = tmp_path / "certificate.pending.json"
    final = tmp_path / "certificate.json"
    pending.write_text('{"schema_version": 2}\n', encoding="utf-8")

    result = finalize_g10a2_pending_certificate(
        pending_path=pending,
        certificate_path=final,
    )

    assert result == final
    assert not pending.exists()
    assert final.read_text(encoding="utf-8") == '{"schema_version": 2}\n'


def test_fail_closed_cleanup_removes_every_route_owned_certificate_file(
    tmp_path: Path,
) -> None:
    pending = tmp_path / "certificate.pending.json"
    final = tmp_path / "certificate.json"
    internal_tmp = tmp_path / ".certificate.pending.json.tmp"
    for path in (pending, final, internal_tmp):
        path.write_text("candidate", encoding="utf-8")

    failures = cleanup_g10a2_publication_artifacts(tmp_path)

    assert failures == ()
    assert not pending.exists()
    assert not final.exists()
    assert not internal_tmp.exists()
