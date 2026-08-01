import ast
from fractions import Fraction
import hashlib
import json
from math import inf, nextafter
from pathlib import Path

import pytest

from cbf_rover.domain.planar import PlanarState
from cbf_rover.entrypoints.research import build_parser, main
from cbf_rover.methods.polynomial_sbc.certificate import QSBCertificate
from cbf_rover.methods.polynomial_sbc.g11p49_piecewise_roof_lp import (
    G11P49_EPSBAR,
    G11P49_Q,
    G11P49_STAGE_SCHEDULE,
    _build_variable_layout,
    _expected_v3p_constraint_names,
    _publication_disk_release_gate,
    _record_rss_checkpoint,
    _v3p_release_gate,
    _variable_catalog_payload,
    exact_roof_quadrant_expectation,
    g11p49_expected_problem_shape,
    p49_quadrant_moment,
)
from cbf_rover.methods.polynomial_sbc.piecewise_barrier import (
    G11P49_GEOMETRY_CATALOG_SHA256,
    G11P49_PROOF_ROUTE_ID,
    G11P49_REACH_CATALOG_ID,
    G11P49_VALIDATOR_ID,
    LocalBernsteinRoof2D,
    P49_CELLS,
    P49_CELL_BY_ID,
    P49_GOAL_BOUNDARY_SAFE_CELLS,
    P49_NORMALIZED_X_BREAKS,
    P49_NORMALIZED_Y_BREAKS,
    P49_PARTITION_ID,
    P49_QUADRANTS,
    P49_SAFE_SOURCE_CELLS,
    P49_UNSAFE_CELLS,
    PiecewiseBernsteinBarrier2D,
    PiecewiseRoofWitness2D,
    local_bernstein_basis_rational_polynomials,
    local_bernstein_grid_rational_polynomial,
    p49_geometry_catalog,
    p49_geometry_catalog_sha256,
    p49_quadrant_reach_box,
    p49_reach_target_intersections,
)
from cbf_rover.methods.polynomial_sbc.piecewise_roof_validation import (
    _p49_catalog_gate,
    _v3p_arithmetic_gate,
    _v3p_barrier_polynomials,
    _v3p_barrier_cell_control_grid,
    _v3p_build_cells,
    _v3p_expand_local_bernstein_grid,
    _v3p_half_moment,
    _v3p_quadrant_roof_expectation,
    _v3p_source_roof_expectation,
    _v3p_validate_roofs,
    validate_piecewise_qsbc_v3p,
    v3p_geometry_catalog,
    v3p_geometry_catalog_sha256,
)
from cbf_rover.methods.polynomial_sbc.polynomial import (
    FrozenPiecewisePolynomialController,
    PolynomialControllerPiece,
    SparsePolynomial2D,
    content_sha256,
    controller_pieces,
)
from cbf_rover.methods.polynomial_sbc.rational_certificate_validation import (
    RationalBernsteinValidationOptions,
    canonical_rational_model_payload,
    classify_full_box,
    exact_expected_next_polynomial_rational,
    validate_rational_polynomial_nonnegative,
)


def _zero_polynomial() -> SparsePolynomial2D:
    return SparsePolynomial2D(((0, 0),), (0.0,))


def _test_controller() -> FrozenPiecewisePolynomialController:
    entry_x = SparsePolynomial2D(((0, 0), (1, 0)), (-0.05, -0.25))
    entry_y = SparsePolynomial2D(
        ((0, 0), (0, 1), (0, 2)),
        (-0.28, -0.11, 0.28),
    )
    transit_x = SparsePolynomial2D(((0, 0), (1, 0)), (0.18, -0.25))
    transit_y = SparsePolynomial2D(
        ((0, 0), (0, 1), (0, 2)),
        (-0.26, -0.11, 0.26),
    )
    capture_x = SparsePolynomial2D(((0, 0), (1, 0)), (0.42, -0.525))
    capture_y = SparsePolynomial2D(((0, 1),), (-0.2,))
    return FrozenPiecewisePolynomialController(
        controller_id="G11P49_TEST_CONTROLLER",
        pieces=(
            PolynomialControllerPiece(
                "lower_entry",
                -1.0,
                0.8,
                entry_x,
                entry_y,
            ),
            PolynomialControllerPiece(
                "lower_transit",
                0.8,
                3.2,
                transit_x,
                transit_y,
            ),
            PolynomialControllerPiece(
                "goal_capture",
                3.2,
                4.0,
                capture_x,
                capture_y,
            ),
        ),
        construction_source_sha256="a" * 64,
    )


def _constant_barrier(value: float = 0.25) -> PiecewiseBernsteinBarrier2D:
    degree = 3
    size = 7 * degree + 1
    return PiecewiseBernsteinBarrier2D(
        degree=degree,
        control_grid=tuple(
            tuple(value for _ in range(size)) for _ in range(size)
        ),
    )


def test_g11p49_cli_routes_are_explicit_and_independent() -> None:
    parser = build_parser()
    solve = parser.parse_args(
        [
            "solve-qsbc-g11p49",
            "--controller",
            "controller.json",
            "--output-dir",
            "out",
        ]
    )
    replay = parser.parse_args(
        [
            "replay-certificate-v3p",
            "--certificate",
            "certificate.json",
            "--roof-witness",
            "roof_witness.json",
            "--controller",
            "controller.json",
            "--output-dir",
            "replay",
        ]
    )

    assert solve.command == "solve-qsbc-g11p49"
    assert replay.command == "replay-certificate-v3p"
    assert replay.roof_witness == "roof_witness.json"


def _constant_roof(
    *,
    source_cell_id: str,
    quadrant: tuple[int, int],
    value: float,
    degree: int = 1,
) -> LocalBernsteinRoof2D:
    source = P49_CELL_BY_ID[source_cell_id]
    return LocalBernsteinRoof2D(
        source_cell_id=source.cell_id,
        controller_piece_id=source.controller_piece_id,
        quadrant=quadrant,
        normalized_reach_box=p49_quadrant_reach_box(source, quadrant),
        degree=degree,
        control_grid=tuple(
            tuple(value for _ in range(degree + 1))
            for _ in range(degree + 1)
        ),
    )


def _constant_witness(
    *,
    barrier: PiecewiseBernsteinBarrier2D,
    controller: FrozenPiecewisePolynomialController,
    value: float,
) -> PiecewiseRoofWitness2D:
    return PiecewiseRoofWitness2D(
        barrier_payload_sha256=barrier.payload_sha256,
        controller_content_sha256=controller.sha256,
        formal_model_contract_sha256=content_sha256(
            canonical_rational_model_payload()
        ),
        geometry_catalog_sha256=p49_geometry_catalog_sha256(),
        barrier_degree=barrier.degree,
        roof_degree=4,
        roofs=tuple(
            _constant_roof(
                source_cell_id=cell.cell_id,
                quadrant=quadrant,
                value=value,
                degree=4,
            )
            for cell in P49_SAFE_SOURCE_CELLS
            for quadrant in P49_QUADRANTS
        ),
    )


def _schema3_bindings(
    *,
    barrier: PiecewiseBernsteinBarrier2D,
    witness: PiecewiseRoofWitness2D,
    controller: FrozenPiecewisePolynomialController,
) -> tuple[dict, dict]:
    report_content_hash = "e" * 64
    proof = {
        "schema_version": 1,
        "kind": "g11p49_piecewise_roof_witness_binding",
        "proof_route_id": G11P49_PROOF_ROUTE_ID,
        "validator_id": G11P49_VALIDATOR_ID,
        "barrier_degree": barrier.degree,
        "roof_degree": witness.roof_degree,
        "barrier_payload_sha256": barrier.payload_sha256,
        "controller_content_sha256": controller.sha256,
        "geometry_catalog_sha256": p49_geometry_catalog_sha256(),
        "roof_witness_content_sha256": witness.sha256,
        "formal_model_contract_sha256": witness.formal_model_contract_sha256,
        "validation_report_content_sha256": report_content_hash,
    }
    validation = {
        "validator_id": G11P49_VALIDATOR_ID,
        "validation_report_sha256": "f" * 64,
        "validation_report_content_sha256": report_content_hash,
        "status": "validated",
        "first_slice_ready": True,
        "formal_acceptance_tolerance": 0.0,
        "barrier_payload_sha256": barrier.payload_sha256,
        "roof_witness_content_sha256": witness.sha256,
        "controller_content_sha256": controller.sha256,
        "formal_model_contract_sha256": witness.formal_model_contract_sha256,
        "geometry_catalog_sha256": p49_geometry_catalog_sha256(),
        "q_float64_hex": G11P49_Q.hex(),
        "epsbar_float64_hex": G11P49_EPSBAR.hex(),
        "barrier_degree": barrier.degree,
        "roof_degree": witness.roof_degree,
    }
    return proof, validation


def _evaluate_rational(polynomial: dict, x_value: Fraction, y_value: Fraction):
    return sum(
        coefficient * x_value**power[0] * y_value**power[1]
        for power, coefficient in polynomial.items()
    )


def test_p49_exact_partition_and_reach_catalog_counts_are_frozen() -> None:
    catalog = p49_geometry_catalog()

    assert P49_PARTITION_ID == "P49_C2_OBSTACLE_ALIGNED_CARTESIAN_V1"
    assert G11P49_REACH_CATALOG_ID == (
        "G11P49_ACTION_DISK_QUADRANT_REACH_CATALOG_V1"
    )
    assert len(P49_CELLS) == 49
    assert len(P49_UNSAFE_CELLS) == 12
    assert len(P49_SAFE_SOURCE_CELLS) == 37
    assert len(P49_GOAL_BOUNDARY_SAFE_CELLS) == 3
    assert len(catalog["reach_obligations"]) == 148
    assert catalog["counts"] == {
        "cell_count": 49,
        "unsafe_cell_count": 12,
        "safe_source_cell_count": 37,
        "goal_boundary_safe_cell_count": 3,
        "roof_count": 148,
        "target_intersection_count": 1012,
        "internal_conforming_edge_count": 84,
    }
    assert catalog["catalog_content_sha256"] == p49_geometry_catalog_sha256()
    assert p49_geometry_catalog_sha256() == (
        "ef545affb1c35d965be5d4b9d9e5ab39854e4d8069c296b924d45e049a88924a"
    )
    assert G11P49_GEOMETRY_CATALOG_SHA256 == p49_geometry_catalog_sha256()

    for quadrant in P49_QUADRANTS:
        count = sum(
            len(p49_reach_target_intersections(cell, quadrant))
            for cell in P49_SAFE_SOURCE_CELLS
        )
        assert count == 253

    catalog["counts"]["cell_count"] = 0
    assert p49_geometry_catalog()["counts"]["cell_count"] == 49

    rebuilt = p49_geometry_catalog()
    assert _p49_catalog_gate(rebuilt)["status"] == "validated"
    rebuilt["reach_padding"]["action_shift_x_normalized"] = "1/99"
    assert _p49_catalog_gate(rebuilt)["status"] == "violated"


def test_p49_normalized_breaks_are_the_exact_affine_images() -> None:
    assert P49_NORMALIZED_X_BREAKS == (
        Fraction(-1),
        Fraction(-7, 25),
        Fraction(-107, 500),
        Fraction(83, 500),
        Fraction(107, 500),
        Fraction(317, 500),
        Fraction(17, 25),
        Fraction(1),
    )
    assert P49_NORMALIZED_Y_BREAKS == (
        Fraction(-1),
        Fraction(-277, 400),
        Fraction(-107, 400),
        Fraction(-43, 400),
        Fraction(43, 400),
        Fraction(107, 400),
        Fraction(277, 400),
        Fraction(1),
    )


def test_v3p_independently_rebuilds_cells_switches_and_worked_reaches() -> None:
    cells = _v3p_build_cells()
    by_id = {cell.cell_id: cell for cell in cells}
    unsafe = {
        cell.cell_id: cell.unsafe_component_ids
        for cell in cells
        if cell.unsafe
    }
    assert unsafe == {
        "C_2_2": ("Xu1",),
        "C_2_3": ("Xu1",),
        "C_2_4": ("Xu1",),
        "C_3_1": ("Xu3",),
        "C_3_2": ("Xu1", "Xu3"),
        "C_3_3": ("Xu1",),
        "C_3_4": ("Xu1", "Xu2"),
        "C_3_5": ("Xu2",),
        "C_4_1": ("Xu3",),
        "C_4_2": ("Xu3",),
        "C_4_4": ("Xu2",),
        "C_4_5": ("Xu2",),
    }
    assert [
        cell.cell_id
        for cell in cells
        if cell.safe_source and cell.goal_boundary
    ] == ["C_6_2", "C_6_3", "C_6_4"]
    assert by_id["C_0_0"].physical_box[1] == Fraction(4, 5)
    assert by_id["C_1_0"].physical_box[0] == Fraction(4, 5)
    assert by_id["C_0_0"].controller_piece_id == "lower_entry"
    assert by_id["C_1_0"].controller_piece_id == "lower_transit"
    assert by_id["C_5_0"].physical_box[1] == Fraction(16, 5)
    assert by_id["C_6_0"].physical_box[0] == Fraction(16, 5)
    assert by_id["C_5_0"].controller_piece_id == "lower_transit"
    assert by_id["C_6_0"].controller_piece_id == "goal_capture"

    catalog = v3p_geometry_catalog()
    assert catalog == p49_geometry_catalog()
    assert v3p_geometry_catalog_sha256() == G11P49_GEOMETRY_CATALOG_SHA256
    obligations = {
        (row["source_cell_id"], tuple(row["quadrant"])): row
        for row in catalog["reach_obligations"]
    }
    worked = {
        ("C_0_0", (-1, -1)): (
            ["-1", "-27/100", "-1", "-17/25"],
            ["C_0_0", "C_0_1", "C_1_0", "C_1_1"],
        ),
        ("C_6_6", (1, 1)): (
            ["67/100", "1", "17/25", "1"],
            ["C_5_5", "C_5_6", "C_6_5", "C_6_6"],
        ),
        ("C_1_3", (1, -1)): (
            ["-29/100", "-101/500", "-49/400", "3/25"],
            [
                "C_0_2",
                "C_0_3",
                "C_0_4",
                "C_1_2",
                "C_1_3",
                "C_1_4",
                "C_2_2",
                "C_2_3",
                "C_2_4",
            ],
        ),
    }
    for key, (expected_box, expected_targets) in worked.items():
        obligation = obligations[key]
        assert obligation["normalized_reach_box_exact"] == expected_box
        assert [
            target["target_cell_id"]
            for target in obligation["target_intersections"]
        ] == expected_targets


def test_v3p_local_bernstein_expansion_is_x_major_and_nontransposed() -> None:
    unit_box = (Fraction(0), Fraction(1), Fraction(0), Fraction(1))
    constant = _v3p_expand_local_bernstein_grid(
        degree=2,
        control_grid=tuple(tuple(3.0 for _ in range(3)) for _ in range(3)),
        box=unit_box,
    )
    assert constant == {(0, 0): Fraction(3)}

    asymmetric = _v3p_expand_local_bernstein_grid(
        degree=1,
        control_grid=((1.0, 2.0), (4.0, 8.0)),
        box=unit_box,
    )
    assert _evaluate_rational(asymmetric, Fraction(0), Fraction(0)) == 1
    assert _evaluate_rational(asymmetric, Fraction(0), Fraction(1)) == 2
    assert _evaluate_rational(asymmetric, Fraction(1), Fraction(0)) == 4
    assert _evaluate_rational(asymmetric, Fraction(1), Fraction(1)) == 8

    single_basis = _v3p_expand_local_bernstein_grid(
        degree=2,
        control_grid=((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 0.0)),
        box=(Fraction(-1), Fraction(2), Fraction(-2), Fraction(1)),
    )
    assert _evaluate_rational(single_basis, Fraction(1, 2), Fraction(-1, 2)) == (
        Fraction(1, 4)
    )

    barrier = PiecewiseBernsteinBarrier2D(
        degree=3,
        control_grid=tuple(
            tuple(float(100 * x + y) for y in range(22)) for x in range(22)
        ),
    )
    cells = _v3p_build_cells()
    by_id = {cell.cell_id: cell for cell in cells}
    left = _v3p_barrier_cell_control_grid(barrier, by_id["C_0_0"])
    right = _v3p_barrier_cell_control_grid(barrier, by_id["C_1_0"])
    lower = _v3p_barrier_cell_control_grid(barrier, by_id["C_0_0"])
    upper = _v3p_barrier_cell_control_grid(barrier, by_id["C_0_1"])
    assert left[-1] == right[0]
    assert tuple(row[-1] for row in lower) == tuple(row[0] for row in upper)


def test_piecewise_barrier_global_grid_has_frozen_dof_counts() -> None:
    for degree, expected in ((3, 484), (4, 841)):
        size = 7 * degree + 1
        barrier = PiecewiseBernsteinBarrier2D(
            degree=degree,
            control_grid=tuple(tuple(2.5 for _ in range(size)) for _ in range(size)),
        )

        assert barrier.dof_count == expected
        assert barrier.evaluate(-1.0, -1.0) == pytest.approx(2.5)
        assert barrier.evaluate(1.0, 1.0) == pytest.approx(2.5)
        assert barrier.evaluate(0.0, 0.0) == pytest.approx(2.5)


def test_shared_control_net_is_exactly_c0_on_every_internal_edge() -> None:
    degree = 3
    size = 7 * degree + 1
    barrier = PiecewiseBernsteinBarrier2D(
        degree=degree,
        control_grid=tuple(
            tuple(float(100 * x_index + y_index) for y_index in range(size))
            for x_index in range(size)
        ),
    )

    for x_index in range(6):
        for y_index in range(7):
            left = barrier.cell_control_grid(x_index, y_index)
            right = barrier.cell_control_grid(x_index + 1, y_index)
            assert left[-1] == right[0]
    for x_index in range(7):
        for y_index in range(6):
            lower = barrier.cell_control_grid(x_index, y_index)
            upper = barrier.cell_control_grid(x_index, y_index + 1)
            assert tuple(row[-1] for row in lower) == tuple(
                row[0] for row in upper
            )


def test_exact_local_bernstein_expansion_matches_affine_worked_example() -> None:
    polynomial = local_bernstein_grid_rational_polynomial(
        degree=1,
        control_grid=((0.0, 1.0), (1.0, 2.0)),
        box=(Fraction(0), Fraction(1), Fraction(0), Fraction(1)),
    )

    assert polynomial == {(1, 0): Fraction(1), (0, 1): Fraction(1)}
    assert _evaluate_rational(polynomial, Fraction(1, 3), Fraction(1, 5)) == (
        Fraction(8, 15)
    )

    basis = local_bernstein_basis_rational_polynomials(
        degree=2,
        box=(Fraction(-1), Fraction(2), Fraction(-2), Fraction(1)),
    )
    assert len(basis) == 9
    summed: dict[tuple[int, int], Fraction] = {}
    for basis_polynomial in basis:
        for power, coefficient in basis_polynomial.items():
            summed[power] = summed.get(power, Fraction(0)) + coefficient
    assert {power: value for power, value in summed.items() if value} == {
        (0, 0): Fraction(1)
    }


def test_quadrant_moments_have_half_mass_and_reconstruct_symmetry() -> None:
    half_width = Fraction(1, 500)

    assert p49_quadrant_moment(
        order=0,
        sign=-1,
        half_width=half_width,
    ) == Fraction(1, 2)
    assert p49_quadrant_moment(
        order=0,
        sign=1,
        half_width=half_width,
    ) == Fraction(1, 2)
    for order in range(1, 7):
        negative = p49_quadrant_moment(
            order=order,
            sign=-1,
            half_width=half_width,
        )
        positive = p49_quadrant_moment(
            order=order,
            sign=1,
            half_width=half_width,
        )
        if order % 2:
            assert negative + positive == 0
        else:
            assert negative + positive == (
                2
                * half_width**order
                / ((order + 1) * (order + 2))
            )


def test_v3p_half_moments_and_constant_quadrant_probability_are_exact() -> None:
    assert _v3p_arithmetic_gate()["status"] == "validated"
    for half_width in (Fraction(1, 500), Fraction(1, 400)):
        for order in range(7):
            negative = _v3p_half_moment(
                order=order,
                sign=-1,
                half_width=half_width,
            )
            positive = _v3p_half_moment(
                order=order,
                sign=1,
                half_width=half_width,
            )
            assert (negative, positive) == (
                Fraction((-1) ** order) * half_width**order
                / ((order + 1) * (order + 2)),
                half_width**order / ((order + 1) * (order + 2)),
            )

    piece = controller_pieces(_test_controller())[0]
    for quadrant in P49_QUADRANTS:
        assert _v3p_quadrant_roof_expectation(
            polynomial={(0, 0): Fraction(1)},
            quadrant=quadrant,
            piece=piece,
        ) == {(0, 0): Fraction(1, 4)}


def test_v3p_four_quadrants_match_full_symmetric_n1_for_all_degree6_monomials() -> None:
    for piece in controller_pieces(_test_controller()):
        for x_power in range(7):
            for y_power in range(7):
                monomial = {(x_power, y_power): Fraction(1)}
                quadrant_sum: dict[tuple[int, int], Fraction] = {}
                for quadrant in P49_QUADRANTS:
                    expectation = _v3p_quadrant_roof_expectation(
                        polynomial=monomial,
                        quadrant=quadrant,
                        piece=piece,
                    )
                    for power, coefficient in expectation.items():
                        quadrant_sum[power] = quadrant_sum.get(
                            power,
                            Fraction(0),
                        ) + coefficient
                quadrant_sum = {
                    power: coefficient
                    for power, coefficient in quadrant_sum.items()
                    if coefficient
                }
                expected = exact_expected_next_polynomial_rational(
                    SparsePolynomial2D(((x_power, y_power),), (1.0,)),
                    piece,
                )
                assert quadrant_sum == expected


def test_constant_roof_has_exact_quadrant_probability_one_quarter() -> None:
    roof = _constant_roof(
        source_cell_id="C_0_0",
        quadrant=(-1, -1),
        value=1.0,
    )
    piece = PolynomialControllerPiece(
        piece_id="lower_entry",
        x_min_m=-1.0,
        x_max_m=0.8,
        u_x=_zero_polynomial(),
        u_y=_zero_polynomial(),
    )

    assert exact_roof_quadrant_expectation(roof, piece) == {
        (0, 0): Fraction(1, 4)
    }


def test_finite_schedule_problem_shapes_are_exact() -> None:
    assert [stage.stage_id for stage in G11P49_STAGE_SCHEDULE] == [
        "P49_W3_R4_M2",
        "P49_W4_R6_M2",
        "P49_W4_R6_M4",
    ]
    assert [stage.expected_variable_count for stage in G11P49_STAGE_SCHEDULE] == [
        4221,
        8130,
        8194,
    ]
    assert [
        stage.expected_linear_row_count for stage in G11P49_STAGE_SCHEDULE
    ] == [27666, 54121, 54145]
    assert g11p49_expected_problem_shape(
        barrier_degree=4,
        roof_degree=6,
        multiplier_degree=4,
    ) == (8194, 54145)

    expected_layouts = (
        (484, 3700, 36, 4220, 4221),
        (841, 7252, 36, 8129, 8130),
        (841, 7252, 100, 8193, 8194),
    )
    for stage, expected in zip(G11P49_STAGE_SCHEDULE, expected_layouts):
        layout = _build_variable_layout(stage)
        assert (
            layout.barrier_count,
            layout.roof_count,
            layout.multiplier_count,
            layout.rho_index,
            layout.variable_count,
        ) == expected
        catalog = _variable_catalog_payload(stage=stage, layout=layout)
        assert catalog["variable_count"] == stage.expected_variable_count
        assert len(catalog["roof_blocks"]) == 148
        assert len(catalog["multiplier_blocks"]) == 4


def test_g11p49_resource_checkpoint_is_fail_closed(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    import cbf_rover.methods.polynomial_sbc.g11p49_piecewise_roof_lp as lp

    manifest: dict[str, object] = {}
    monkeypatch.setattr(
        lp,
        "_peak_rss_bytes",
        lambda: lp.G11P49_MAX_RSS_BYTES + 1,
    )
    with pytest.raises(MemoryError, match="test_checkpoint"):
        _record_rss_checkpoint(manifest, "test_checkpoint")
    assert manifest["resource_checkpoints"] == [
        {
            "checkpoint": "test_checkpoint",
            "peak_RSS_bytes": lp.G11P49_MAX_RSS_BYTES + 1,
            "max_RSS_bytes": lp.G11P49_MAX_RSS_BYTES,
            "within_cap": False,
        }
    ]


def test_g11p49_rechecks_full_disk_gate_immediately_before_atomic_publish() -> None:
    import cbf_rover.methods.polynomial_sbc.g11p49_piecewise_roof_lp as lp

    source = Path(lp.__file__).read_text(encoding="utf-8")
    module = ast.parse(source)
    solve = next(
        node
        for node in module.body
        if isinstance(node, ast.FunctionDef)
        and node.name == "_solve_g11p49_stage_impl"
    )
    publication_calls = [
        node
        for node in ast.walk(solve)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Name)
        and node.func.id == "_publication_disk_release_gate"
    ]
    assert len(publication_calls) == 2

    publication_try = next(
        node
        for node in ast.walk(solve)
        if isinstance(node, ast.Try)
        and any(
            isinstance(statement, ast.Assign)
            and any(
                isinstance(target, ast.Tuple)
                and any(
                    isinstance(element, ast.Name)
                    and element.id == "final_publication_gate_passed"
                    for element in target.elts
                )
                for target in statement.targets
            )
            for statement in node.body
        )
    )
    final_gate_index = next(
        index
        for index, statement in enumerate(publication_try.body)
        if isinstance(statement, ast.Assign)
        and any(
            isinstance(target, ast.Tuple)
            and any(
                isinstance(element, ast.Name)
                and element.id == "final_publication_gate_passed"
                for element in target.elts
            )
            for target in statement.targets
        )
    )
    failure_branch = publication_try.body[final_gate_index + 1]
    atomic_replace = publication_try.body[final_gate_index + 2]
    assert isinstance(failure_branch, ast.If)
    assert isinstance(failure_branch.test, ast.UnaryOp)
    assert isinstance(failure_branch.test.op, ast.Not)
    assert isinstance(failure_branch.test.operand, ast.Name)
    assert failure_branch.test.operand.id == "final_publication_gate_passed"
    assert isinstance(atomic_replace, ast.Expr)
    assert isinstance(atomic_replace.value, ast.Call)
    assert isinstance(atomic_replace.value.func, ast.Attribute)
    assert atomic_replace.value.func.attr == "replace"
    assert isinstance(atomic_replace.value.func.value, ast.Name)
    assert atomic_replace.value.func.value.id == "pending_path"


def test_g11p49_ready_status_is_written_once_after_final_rss_checkpoint() -> None:
    import cbf_rover.methods.polynomial_sbc.g11p49_piecewise_roof_lp as lp

    source = Path(lp.__file__).read_text(encoding="utf-8")
    module = ast.parse(source)
    solve = next(
        node
        for node in module.body
        if isinstance(node, ast.FunctionDef)
        and node.name == "_solve_g11p49_stage_impl"
    )
    publication_try = next(
        node
        for node in ast.walk(solve)
        if isinstance(node, ast.Try)
        and any(
            isinstance(statement, ast.Assign)
            and any(
                isinstance(target, ast.Subscript)
                and isinstance(target.slice, ast.Constant)
                and target.slice.value == "status"
                and isinstance(statement.value, ast.Constant)
                and statement.value.value == "ready"
                for target in statement.targets
            )
            for statement in node.body
        )
    )
    ready_index = next(
        index
        for index, statement in enumerate(publication_try.body)
        if isinstance(statement, ast.Assign)
        and isinstance(statement.value, ast.Constant)
        and statement.value.value == "ready"
    )
    final_checkpoint = publication_try.body[ready_index - 1]
    manifest_write = publication_try.body[ready_index + 1]
    assert isinstance(final_checkpoint, ast.Expr)
    assert isinstance(final_checkpoint.value, ast.Call)
    assert isinstance(final_checkpoint.value.func, ast.Name)
    assert final_checkpoint.value.func.id == "_record_rss_checkpoint"
    assert isinstance(final_checkpoint.value.args[1], ast.Constant)
    assert final_checkpoint.value.args[1].value == "final_ready_manifest"
    assert isinstance(manifest_write, ast.Expr)
    assert isinstance(manifest_write.value, ast.Call)
    assert isinstance(manifest_write.value.func, ast.Name)
    assert manifest_write.value.func.id == "_save_manifest"
    assert not any(
        isinstance(node, ast.Call)
        and isinstance(node.func, ast.Name)
        and node.func.id == "_save_manifest"
        for statement in publication_try.body[ready_index + 2 :]
        for node in ast.walk(statement)
    )


def test_v3p_has_an_ast_enforced_n_version_import_boundary() -> None:
    import cbf_rover.methods.polynomial_sbc.piecewise_roof_validation as v3p

    source = Path(v3p.__file__).read_text(encoding="utf-8")
    tree = ast.parse(source)
    forbidden_names = {
        "exact_source_roof_expectation",
        "p49_geometry_catalog",
        "p49_geometry_catalog_sha256",
        "p49_reach_target_intersections",
        "local_bernstein_grid_rational_polynomial",
        "local_bernstein_basis_rational_polynomials",
        "cell_rational_polynomial",
        "rational_polynomial",
    }
    allowed_piecewise_imports = {
        "G11P49_PROOF_ROUTE_ID",
        "G11P49_REACH_CATALOG_ID",
        "G11P49_VALIDATOR_ID",
        "P49_PARTITION_ID",
        "LocalBernsteinRoof2D",
        "PiecewiseBernsteinBarrier2D",
        "PiecewiseRoofWitness2D",
    }
    for node in ast.walk(tree):
        if isinstance(node, ast.ImportFrom):
            assert not (node.module or "").endswith("g11p49_piecewise_roof_lp")
            if (node.module or "").endswith("piecewise_barrier"):
                assert {alias.name for alias in node.names} == (
                    allowed_piecewise_imports
                )
        if isinstance(node, ast.Attribute):
            assert node.attr not in forbidden_names
        if isinstance(node, ast.Name):
            assert node.id not in forbidden_names


def test_v3p_local_math_ignores_monkeypatched_generator_helpers(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    import cbf_rover.methods.polynomial_sbc.g11p49_piecewise_roof_lp as lp
    import cbf_rover.methods.polynomial_sbc.piecewise_barrier as generator

    barrier = _constant_barrier()
    controller = _test_controller()
    witness = _constant_witness(
        barrier=barrier,
        controller=controller,
        value=1.0,
    )
    cells = _v3p_build_cells()
    safe_cells = tuple(cell for cell in cells if cell.safe_source)
    roofs = _v3p_validate_roofs(witness=witness, safe_cells=safe_cells)
    source = safe_cells[0]
    source_roofs = {
        quadrant: roofs[(source.cell_id, quadrant)]
        for quadrant in P49_QUADRANTS
    }
    expected_catalog = v3p_geometry_catalog()
    expected_barriers = _v3p_barrier_polynomials(barrier, cells)
    expected_expectation = _v3p_source_roof_expectation(
        roofs=source_roofs,
        source=source,
        piece=controller_pieces(controller)[0],
    )

    def forbidden(*args: object, **kwargs: object) -> object:
        del args, kwargs
        raise AssertionError("generator semantic helper was called by V3P")

    for name in (
        "p49_geometry_catalog",
        "p49_geometry_catalog_sha256",
        "p49_reach_target_intersections",
        "p49_quadrant_reach_box",
        "local_bernstein_grid_rational_polynomial",
        "local_bernstein_basis_rational_polynomials",
    ):
        monkeypatch.setattr(generator, name, forbidden)
    monkeypatch.setattr(
        generator.PiecewiseBernsteinBarrier2D,
        "cell_rational_polynomial",
        forbidden,
    )
    monkeypatch.setattr(
        generator.LocalBernsteinRoof2D,
        "rational_polynomial",
        forbidden,
    )
    monkeypatch.setattr(lp, "exact_source_roof_expectation", forbidden)

    assert v3p_geometry_catalog() == expected_catalog
    assert _v3p_barrier_polynomials(barrier, cells) == expected_barriers
    assert _v3p_source_roof_expectation(
        roofs=source_roofs,
        source=source,
        piece=controller_pieces(controller)[0],
    ) == expected_expectation


def test_v3p_geometry_and_arithmetic_fault_injection_is_fail_closed(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    import cbf_rover.methods.polynomial_sbc.piecewise_roof_validation as v3p

    canonical = v3p_geometry_catalog()
    tampered_catalogs = []
    for mutation in (
        "delete_intersection",
        "change_break",
        "change_padding",
        "swap_quadrants",
        "change_switch_assignment",
    ):
        payload = json.loads(json.dumps(canonical))
        if mutation == "delete_intersection":
            payload["reach_obligations"][0]["target_intersections"].pop()
            payload["counts"]["target_intersection_count"] -= 1
        elif mutation == "change_break":
            payload["normalized_x_breaks_exact"][1] = "-1/4"
        elif mutation == "change_padding":
            payload["reach_padding"]["action_shift_x_normalized"] = "1/99"
        elif mutation == "swap_quadrants":
            payload["quadrant_order"][0], payload["quadrant_order"][1] = (
                payload["quadrant_order"][1],
                payload["quadrant_order"][0],
            )
        else:
            payload["cells"][0]["controller_piece_id"] = "lower_transit"
        payload.pop("catalog_content_sha256")
        payload["catalog_content_sha256"] = content_sha256(payload)
        tampered_catalogs.append(payload)
    assert all(
        _p49_catalog_gate(payload)["status"] == "violated"
        for payload in tampered_catalogs
    )

    original_moment = v3p._v3p_half_moment
    with monkeypatch.context() as scoped:
        scoped.setattr(
            v3p,
            "_v3p_half_moment",
            lambda *, order, sign, half_width: (
                Fraction(1)
                if order == 0
                else original_moment(
                    order=order,
                    sign=sign,
                    half_width=half_width,
                )
            ),
        )
        assert v3p._v3p_arithmetic_gate()["status"] == "violated"
    with monkeypatch.context() as scoped:
        scoped.setattr(v3p, "NORMALIZATION_SCALE_X", Fraction(3))
        with pytest.raises(ValueError, match="normalization drifted"):
            v3p.v3p_geometry_catalog()
    with monkeypatch.context() as scoped:
        scoped.setattr(v3p, "MAX_SPEED", Fraction(3, 5))
        with pytest.raises(ValueError, match="padding drifted"):
            v3p.v3p_geometry_catalog()
    with monkeypatch.context() as scoped:
        scoped.setattr(
            v3p,
            "_V3P_QUADRANTS",
            ((-1, 1), (-1, -1), (1, -1), (1, 1)),
        )
        with pytest.raises(ValueError, match="catalog hash drifted"):
            v3p.v3p_geometry_catalog()
    with monkeypatch.context() as scoped:
        scoped.setattr(
            v3p,
            "_v3p_controller_piece_id",
            lambda physical_box: "lower_entry",
        )
        with pytest.raises(ValueError, match="catalog hash drifted"):
            v3p.v3p_geometry_catalog()

    unknown = validate_rational_polynomial_nonnegative(
        {(1, 0): Fraction(1)},
        (Fraction(-1), Fraction(1), Fraction(-1), Fraction(1)),
        classify_domain=classify_full_box,
        options=RationalBernsteinValidationOptions(max_depth=0),
    )
    assert unknown["status"] == "unknown"
    assert unknown["status"] != "validated"


def test_v3p_checks_a_target_intersection_that_a_tampered_catalog_deletes(
    tmp_path: Path,
) -> None:
    degree = 3
    size = 7 * degree + 1
    grid = [[0.0 for _ in range(size)] for _ in range(size)]
    grid[degree + 1][degree + 1] = 1.0
    barrier = PiecewiseBernsteinBarrier2D(
        degree=degree,
        control_grid=tuple(tuple(row) for row in grid),
    )
    controller = _test_controller()
    selected_key = ("C_0_0", (-1, -1))
    witness = PiecewiseRoofWitness2D(
        barrier_payload_sha256=barrier.payload_sha256,
        controller_content_sha256=controller.sha256,
        formal_model_contract_sha256=content_sha256(
            canonical_rational_model_payload()
        ),
        geometry_catalog_sha256=p49_geometry_catalog_sha256(),
        barrier_degree=degree,
        roof_degree=4,
        roofs=tuple(
            _constant_roof(
                source_cell_id=cell.cell_id,
                quadrant=quadrant,
                value=(0.0 if (cell.cell_id, quadrant) == selected_key else 2.0),
                degree=4,
            )
            for cell in P49_SAFE_SOURCE_CELLS
            for quadrant in P49_QUADRANTS
        ),
    )
    report_path = validate_piecewise_qsbc_v3p(
        barrier=barrier,
        roof_witness=witness,
        q=G11P49_Q,
        epsbar=G11P49_EPSBAR,
        controller=controller,
        output_dir=tmp_path / "negative_intersection_replay",
    )
    report = json.loads(report_path.read_text(encoding="utf-8"))
    selected = report["roof_domination_constraints"]
    deleted_by_tampered_catalog = selected[
        "R_C_0_0_mm_dominates_C_1_1"
    ]
    assert deleted_by_tampered_catalog["status"] != "validated"
    assert deleted_by_tampered_catalog["failure_reason"] is not None
    assert all(
        selected[f"R_C_0_0_mm_dominates_{target}"]["status"] == "validated"
        for target in ("C_0_0", "C_0_1", "C_1_0")
    )
    assert report["status"] != "validated"
    assert report["first_slice_ready"] is False


def test_roof_witness_round_trip_binds_all_148_roofs(tmp_path: Path) -> None:
    roofs = tuple(
        _constant_roof(
            source_cell_id=cell.cell_id,
            quadrant=quadrant,
            value=0.0,
            degree=4,
        )
        for cell in P49_SAFE_SOURCE_CELLS
        for quadrant in P49_QUADRANTS
    )
    witness = PiecewiseRoofWitness2D(
        barrier_payload_sha256="a" * 64,
        controller_content_sha256="b" * 64,
        formal_model_contract_sha256="c" * 64,
        geometry_catalog_sha256=p49_geometry_catalog_sha256(),
        barrier_degree=3,
        roof_degree=4,
        roofs=roofs,
    )
    path = witness.save(tmp_path / "roof_witness.json")
    loaded = PiecewiseRoofWitness2D.load(path)

    assert loaded == witness
    assert loaded.sha256 == witness.sha256

    payload = json.loads(path.read_text(encoding="utf-8"))
    payload["roofs"][0]["control_grid_decimal"][0][0] = 1.0
    path.write_text(json.dumps(payload), encoding="utf-8")
    with pytest.raises(ValueError, match="content hash mismatch"):
        PiecewiseRoofWitness2D.load(path)

    with pytest.raises(ValueError, match="degree pair"):
        PiecewiseRoofWitness2D(
            barrier_payload_sha256="a" * 64,
            controller_content_sha256="b" * 64,
            formal_model_contract_sha256="c" * 64,
            geometry_catalog_sha256=p49_geometry_catalog_sha256(),
            barrier_degree=3,
            roof_degree=3,
            roofs=tuple(
                _constant_roof(
                    source_cell_id=cell.cell_id,
                    quadrant=quadrant,
                    value=0.0,
                    degree=3,
                )
                for cell in P49_SAFE_SOURCE_CELLS
                for quadrant in P49_QUADRANTS
            ),
        )


def test_v3p_release_and_second_publication_gates_reject_disk_tampering(
    tmp_path: Path,
) -> None:
    barrier = _constant_barrier()
    controller = _test_controller()
    witness = _constant_witness(
        barrier=barrier,
        controller=controller,
        value=0.0,
    )
    validation_directory = tmp_path / "validation"
    validation_directory.mkdir()
    groups: dict[str, dict[str, dict[str, object]]] = {}
    expected_names = _expected_v3p_constraint_names(
        witness=witness,
        controller=controller,
    )
    for group, names in expected_names.items():
        summaries: dict[str, dict[str, object]] = {}
        for name in sorted(names):
            relative = Path("trees") / group / f"{name}.json"
            tree_path = validation_directory / relative
            tree_path.parent.mkdir(parents=True, exist_ok=True)
            tree_path.write_text(
                json.dumps({"status": "validated"}, sort_keys=True) + "\n",
                encoding="utf-8",
            )
            summaries[name] = {
                "status": "validated",
                "tree_file": relative.as_posix(),
                "tree_file_sha256": hashlib.sha256(
                    tree_path.read_bytes()
                ).hexdigest(),
            }
        groups[group] = summaries

    validation = {
        "schema_version": 1,
        "kind": "independent_piecewise_qsbc_validation",
        "validator_id": G11P49_VALIDATOR_ID,
        "proof_route_id": G11P49_PROOF_ROUTE_ID,
        "status": "validated",
        "first_slice_ready": True,
        "formal_acceptance_tolerance": 0.0,
        "theorem_scalar_gate": True,
        "q": G11P49_Q,
        "q_float64_hex": G11P49_Q.hex(),
        "q_exact": str(Fraction.from_float(G11P49_Q)),
        "epsbar": G11P49_EPSBAR,
        "epsbar_float64_hex": G11P49_EPSBAR.hex(),
        "epsbar_exact": str(Fraction.from_float(G11P49_EPSBAR)),
        "first_slice_q_max_exact": "1/10",
        "barrier_payload_sha256": barrier.payload_sha256,
        "roof_witness_content_sha256": witness.sha256,
        "controller_content_sha256": controller.sha256,
        "formal_model_contract_sha256": witness.formal_model_contract_sha256,
        "geometry_catalog_sha256": p49_geometry_catalog_sha256(),
        "barrier_degree": barrier.degree,
        "roof_degree": witness.roof_degree,
        "roof_count": 148,
        "constraint_counts": {
            "qsbc": 62,
            "roof_domination": 1012,
            "integrated_drift": 37,
            "controller": 15,
        },
        "c0_gate": {
            "status": "validated",
            "checked_internal_edge_count": 84,
        },
        "p49_catalog_gate": {"status": "validated"},
        "geometry_gate": {"status": "validated"},
        "v3p_arithmetic_gate": {"status": "validated"},
        "formal_model_contract": canonical_rational_model_payload(),
        **groups,
    }
    validation["report_content_sha256"] = content_sha256(validation)
    passed, failures = _v3p_release_gate(
        validation=validation,
        validation_directory=validation_directory,
        barrier=barrier,
        witness=witness,
        controller=controller,
    )
    assert passed
    assert failures == ()

    first_tree = next((validation_directory / "trees").rglob("*.json"))
    first_tree.write_text('{"status":"violated"}\n', encoding="utf-8")
    passed, failures = _v3p_release_gate(
        validation=validation,
        validation_directory=validation_directory,
        barrier=barrier,
        witness=witness,
        controller=controller,
    )
    assert not passed
    assert any(failure.endswith("tree_hash_mismatch") for failure in failures)

    first_summary = next(
        summary
        for summaries in groups.values()
        for summary in summaries.values()
        if (validation_directory / str(summary["tree_file"])) == first_tree
    )
    first_tree.write_text(
        json.dumps({"status": "validated"}, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    first_summary["tree_file_sha256"] = hashlib.sha256(
        first_tree.read_bytes()
    ).hexdigest()
    tampered_report = json.loads(json.dumps(validation))
    removed_name = next(iter(tampered_report["qsbc_constraints"]))
    del tampered_report["qsbc_constraints"][removed_name]
    tampered_report.pop("report_content_sha256")
    tampered_report["report_content_sha256"] = content_sha256(tampered_report)
    passed, failures = _v3p_release_gate(
        validation=tampered_report,
        validation_directory=validation_directory,
        barrier=barrier,
        witness=witness,
        controller=controller,
    )
    assert not passed
    assert "qsbc_constraints_canonical_names" in failures

    validation_path = validation_directory / "validation_report.json"
    validation_path.write_text(
        json.dumps(validation, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    witness_path = witness.save(tmp_path / "roof_witness.json")
    proof_witness, certificate_validation = _schema3_bindings(
        barrier=barrier,
        witness=witness,
        controller=controller,
    )
    proof_witness["validation_report_content_sha256"] = validation[
        "report_content_sha256"
    ]
    certificate_validation["validation_report_content_sha256"] = validation[
        "report_content_sha256"
    ]
    certificate_validation["validation_report_sha256"] = hashlib.sha256(
        validation_path.read_bytes()
    ).hexdigest()
    certificate = QSBCertificate(
        certificate_id="G11P49_TOCTOU_TEST",
        barrier=barrier,
        q=G11P49_Q,
        epsbar=G11P49_EPSBAR,
        controller_content_sha256=controller.sha256,
        validation=certificate_validation,
        proof_witness=proof_witness,
        schema_version=3,
    )
    pending_path = certificate.save(tmp_path / "certificate.pending.json")
    disk_gate_arguments = {
        "validation_path": validation_path,
        "witness_path": witness_path,
        "pending_certificate_path": pending_path,
        "expected_validation_file_sha256": hashlib.sha256(
            validation_path.read_bytes()
        ).hexdigest(),
        "expected_validation_content_sha256": validation[
            "report_content_sha256"
        ],
        "expected_witness_file_sha256": hashlib.sha256(
            witness_path.read_bytes()
        ).hexdigest(),
        "expected_pending_certificate_file_sha256": hashlib.sha256(
            pending_path.read_bytes()
        ).hexdigest(),
        "expected_witness": witness,
        "expected_certificate": certificate,
        "barrier": barrier,
        "controller": controller,
    }
    # Model the first, earlier publication gate succeeding on frozen subjects.
    passed, failures = _publication_disk_release_gate(**disk_gate_arguments)
    assert passed
    assert failures == ()

    # Each mutation occurs after that first gate.  The immediately-pre-rename
    # full gate must re-read and reject every formal disk subject.
    final_path = tmp_path / "certificate.json"
    original_tree = first_tree.read_bytes()
    original_report = validation_path.read_bytes()
    original_witness = witness_path.read_bytes()
    original_pending = pending_path.read_bytes()
    for path_to_tamper, replacement, expected_failure in (
        (first_tree, b'{"status":"violated"}\n', "tree_hash_mismatch"),
        (validation_path, b"{}\n", "validation_report_file_changed"),
        (witness_path, b"{}\n", "roof_witness_disk_reload"),
        (pending_path, b"{}\n", "pending_certificate_disk_reload"),
    ):
        path_to_tamper.write_bytes(replacement)
        passed, failures = _publication_disk_release_gate(
            **disk_gate_arguments
        )
        assert not passed
        assert any(expected_failure in failure for failure in failures)
        assert not final_path.exists()
        if path_to_tamper == first_tree:
            first_tree.write_bytes(original_tree)
        elif path_to_tamper == validation_path:
            validation_path.write_bytes(original_report)
        elif path_to_tamper == witness_path:
            witness_path.write_bytes(original_witness)
        else:
            pending_path.write_bytes(original_pending)


def test_schema3_certificate_dispatches_only_to_piecewise_barrier(
    tmp_path: Path,
) -> None:
    barrier = _constant_barrier()
    controller = _test_controller()
    witness = _constant_witness(
        barrier=barrier,
        controller=controller,
        value=0.0,
    )
    proof_witness, validation = _schema3_bindings(
        barrier=barrier,
        witness=witness,
        controller=controller,
    )
    certificate = QSBCertificate(
        certificate_id="G11P49_TEST",
        barrier=barrier,
        q=G11P49_Q,
        epsbar=G11P49_EPSBAR,
        controller_content_sha256=controller.sha256,
        validation=validation,
        proof_witness=proof_witness,
        schema_version=3,
    )
    path = certificate.save(tmp_path / "certificate.json")
    loaded = QSBCertificate.load(path)

    assert loaded == certificate
    assert loaded.evaluate(PlanarState(0.0, 0.0)) == pytest.approx(0.25)
    assert loaded.payload_without_hash()["proof_witness"] == proof_witness

    for changed_q, changed_epsbar, message in (
        (
            nextafter(0.1, inf),
            G11P49_EPSBAR,
            "exceeds exact one-tenth",
        ),
        (
            nextafter(G11P49_Q, -inf),
            G11P49_EPSBAR,
            "q is not the frozen",
        ),
        (
            G11P49_Q,
            nextafter(G11P49_EPSBAR, inf),
            "epsbar is not the frozen",
        ),
    ):
        changed_validation = dict(validation)
        changed_validation["q_float64_hex"] = changed_q.hex()
        changed_validation["epsbar_float64_hex"] = changed_epsbar.hex()
        with pytest.raises(ValueError, match=message):
            QSBCertificate(
                certificate_id="G11P49_BAD_SCALAR",
                barrier=barrier,
                q=changed_q,
                epsbar=changed_epsbar,
                controller_content_sha256=controller.sha256,
                validation=changed_validation,
                proof_witness=proof_witness,
                schema_version=3,
            )

    legacy = QSBCertificate(
        certificate_id="LEGACY_SCALARS_UNCHANGED",
        barrier=_zero_polynomial(),
        q=0.9,
        epsbar=0.125,
        controller_content_sha256=controller.sha256,
        schema_version=2,
    )
    assert legacy.q == 0.9
    assert legacy.epsbar == 0.125

    mismatched_degree_proof = dict(proof_witness)
    mismatched_degree_validation = dict(validation)
    mismatched_degree_proof["roof_degree"] = 6
    mismatched_degree_validation["roof_degree"] = 6
    with pytest.raises(ValueError, match="degree pair is outside the frozen"):
        QSBCertificate(
            certificate_id="G11P49_BAD_DEGREE_PAIR",
            barrier=barrier,
            q=G11P49_Q,
            epsbar=G11P49_EPSBAR,
            controller_content_sha256=controller.sha256,
            validation=mismatched_degree_validation,
            proof_witness=mismatched_degree_proof,
            schema_version=3,
        )

    for target, replacement, message in (
        (
            ("proof_witness", "geometry_catalog_sha256"),
            "0" * 64,
            "geometry_catalog_sha256 binding mismatch",
        ),
        (
            ("validation", "roof_witness_content_sha256"),
            "1" * 64,
            "validation roof_witness_content_sha256 mismatch",
        ),
        (
            ("validation", "formal_acceptance_tolerance"),
            1.0e-12,
            "requires zero formal tolerance",
        ),
    ):
        payload = certificate.payload()
        payload[target[0]][target[1]] = replacement
        payload.pop("content_sha256")
        payload["content_sha256"] = content_sha256(payload)
        path.write_text(json.dumps(payload), encoding="utf-8")
        with pytest.raises(ValueError, match=message):
            QSBCertificate.load(path)

    with pytest.raises(ValueError, match="schema-1/2 certificates require"):
        QSBCertificate(
            certificate_id="BAD_GLOBAL_DISPATCH",
            barrier=barrier,
            q=0.1,
            epsbar=0.000001,
            controller_content_sha256="b" * 64,
            schema_version=2,
        )
    with pytest.raises(ValueError, match="schema-3 certificates require"):
        QSBCertificate(
            certificate_id="BAD_PIECEWISE_DISPATCH",
            barrier=_zero_polynomial(),
            q=0.1,
            epsbar=0.000001,
            controller_content_sha256="b" * 64,
            proof_witness=proof_witness,
            schema_version=3,
        )


def test_v3p_cli_rejects_schema_and_external_witness_substitution(
    tmp_path: Path,
) -> None:
    barrier = _constant_barrier()
    controller = _test_controller()
    witness = _constant_witness(
        barrier=barrier,
        controller=controller,
        value=0.0,
    )
    substituted_witness = _constant_witness(
        barrier=barrier,
        controller=controller,
        value=1.0,
    )
    proof_witness, validation = _schema3_bindings(
        barrier=barrier,
        witness=witness,
        controller=controller,
    )
    certificate = QSBCertificate(
        certificate_id="G11P49_CLI_TEST",
        barrier=barrier,
        q=G11P49_Q,
        epsbar=G11P49_EPSBAR,
        controller_content_sha256=controller.sha256,
        validation=validation,
        proof_witness=proof_witness,
        schema_version=3,
    )
    controller_path = controller.save(tmp_path / "controller.json")
    certificate_path = certificate.save(tmp_path / "certificate.json")
    substituted_path = substituted_witness.save(
        tmp_path / "substituted_witness.json"
    )

    for command, message in (
        ("replay-certificate", "schema-3 certificates require"),
        ("replay-certificate-v3", "schema-3 certificates require V3P"),
    ):
        with pytest.raises(ValueError, match=message):
            main(
                [
                    command,
                    "--certificate",
                    str(certificate_path),
                    "--controller",
                    str(controller_path),
                    "--output-dir",
                    str(tmp_path / f"{command}_output"),
                ]
            )

    v3p_output = tmp_path / "v3p_substitution_output"
    with pytest.raises(
        ValueError,
        match="external witness roof_witness_content_sha256 mismatch",
    ):
        main(
            [
                "replay-certificate-v3p",
                "--certificate",
                str(certificate_path),
                "--roof-witness",
                str(substituted_path),
                "--controller",
                str(controller_path),
                "--output-dir",
                str(v3p_output),
            ]
        )
    assert not v3p_output.exists()

    global_certificate = QSBCertificate(
        certificate_id="GLOBAL_CLI_TEST",
        barrier=_zero_polynomial(),
        q=G11P49_Q,
        epsbar=G11P49_EPSBAR,
        controller_content_sha256=controller.sha256,
        schema_version=2,
    )
    global_path = global_certificate.save(tmp_path / "global_certificate.json")
    with pytest.raises(ValueError, match="requires a schema-3 certificate"):
        main(
            [
                "replay-certificate-v3p",
                "--certificate",
                str(global_path),
                "--roof-witness",
                str(tmp_path / "not_read.json"),
                "--controller",
                str(controller_path),
                "--output-dir",
                str(tmp_path / "global_v3p_output"),
            ]
        )
