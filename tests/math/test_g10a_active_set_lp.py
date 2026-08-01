from __future__ import annotations

from fractions import Fraction
import json

import numpy as np
import pytest
from scipy.sparse import csr_matrix

from cbf_rover.adapters.training.sac_reference import file_sha256
from cbf_rover.entrypoints.research import build_parser
from cbf_rover.methods.polynomial_sbc import g10a_active_set_lp as g10a
from cbf_rover.methods.polynomial_sbc.g10a_active_set_lp import (
    G10A_STAGE_SCHEDULE,
    G10A_FEEDBACK_NUMERIC_MARGIN,
    _ActiveRow,
    _ActiveRowStore,
    _ConstraintFamily,
    _IncrementalOrRebuildSolver,
    _NumericSolveResult,
    _load_g9_seed,
    _parse_exact_box,
    _v3_feedback_binding_failures,
    _v3_feedback_rows,
)
from cbf_rover.methods.polynomial_sbc.polynomial import (
    FrozenPolynomialController,
    SparsePolynomial2D,
    content_sha256,
)


def _full_family() -> _ConstraintFamily:
    return _ConstraintFamily(
        name="W_nonnegative_on_X",
        root_box_exact=(Fraction(-1), Fraction(1), Fraction(-1), Fraction(1)),
        domain_kind="full",
        basis=({(0, 0): 1.0}, {(1, 0): 1.0}),
        constant_value=0.0,
    )


def test_g10a_schedule_is_finite_and_frozen() -> None:
    assert [
        (
            stage.stage_id,
            stage.barrier_degree,
            stage.max_outer_iterations,
            stage.max_v3_calls,
            stage.time_limit_s,
        )
        for stage in G10A_STAGE_SCHEDULE
    ] == [
        ("W16_ACTIVE", 16, 12, 3, 3600.0),
        ("W18_ACTIVE", 18, 16, 4, 7200.0),
        ("W20_ACTIVE", 20, 20, 5, 10800.0),
    ]


def test_g10a_is_an_independent_cli_route_with_explicit_g9_seed() -> None:
    args = build_parser().parse_args(
        [
            "solve-qsbc-g10a",
            "--controller",
            "controller.json",
            "--g9-seed",
            "g9.json",
            "--output-dir",
            "results",
        ]
    )

    assert args.command == "solve-qsbc-g10a"
    assert args.controller == "controller.json"
    assert args.g9_seed == "g9.json"
    assert args.output_dir == "results"


def test_g10a_loads_only_hash_bound_diagnostic_seed(tmp_path) -> None:
    barrier = SparsePolynomial2D(((0, 0), (1, 0)), (0.25, -0.5))
    payload = {
        "schema_version": 1,
        "kind": "w1_chebyshev_test_probe",
        "formal_claim_allowed": False,
        "degree": 1,
        "q": 0.2,
        "epsbar": 1.0e-6,
        "barrier": barrier.payload(),
        "chebyshev_coefficients": [0.25, 0.0, -0.5],
    }
    payload["report_content_sha256"] = content_sha256(payload)
    seed_path = tmp_path / "probe.json"
    seed_path.write_text(json.dumps(payload), encoding="utf-8")

    seed = _load_g9_seed(seed_path)

    assert seed.barrier == barrier
    assert seed.report_content_sha256 == payload["report_content_sha256"]
    assert seed.chebyshev_by_power[(1, 0)] == -0.5

    payload["formal_claim_allowed"] = True
    seed_path.write_text(json.dumps(payload), encoding="utf-8")
    with pytest.raises(ValueError, match="diagnostic-only"):
        _load_g9_seed(seed_path)


def test_g10a_canonical_bundle_binds_real_legacy_w14_schema(tmp_path) -> None:
    barrier = SparsePolynomial2D(((0, 0), (14, 0)), (0.25, -0.5))
    legacy_probe = {
        "schema_version": 1,
        "kind": "w14_chebyshev_dense_positive_margin_probe",
        "formal_claim_allowed": False,
        "degree": 14,
        "basis": "total_degree_tensor_chebyshev",
        "q": 0.2145613783418696,
        "epsbar": 1.0e-6,
        "barrier": barrier.payload(),
        "chebyshev_coefficients": [0.0] * 120,
        # This reproduces the server artifact class: a legacy declared hash
        # that is stale under the current canonical content-hash function.
        "report_content_sha256": (
            "7b2abeb9b9138393b39ad3fdd2936c915521be68b353186b297d455bc99f8657"
        ),
    }
    probe_path = tmp_path / "legacy_w14_probe.json"
    probe_path.write_text(json.dumps(legacy_probe), encoding="utf-8")
    recomputed_probe_hash = content_sha256(
        {
            key: value
            for key, value in legacy_probe.items()
            if key != "report_content_sha256"
        }
    )
    assert recomputed_probe_hash != legacy_probe["report_content_sha256"]

    bundle = {
        "schema_version": 1,
        "kind": "g10a_g9_seed_bundle",
        "decision_id": "DX2_SAMPLED_QSBC_EXPRESSIVITY_LP_V1",
        "formal_claim_allowed": False,
        "source_probe": {
            "file": probe_path.name,
            "file_sha256": file_sha256(probe_path),
            "legacy_declared_report_content_sha256": legacy_probe[
                "report_content_sha256"
            ],
            "current_recomputed_content_sha256": recomputed_probe_hash,
            "legacy_declared_matches_current": False,
        },
        "degree": legacy_probe["degree"],
        "basis": legacy_probe["basis"],
        "q": legacy_probe["q"],
        "epsbar": legacy_probe["epsbar"],
        "barrier": legacy_probe["barrier"],
        "chebyshev_coefficients": legacy_probe["chebyshev_coefficients"],
    }
    bundle["report_content_sha256"] = content_sha256(bundle)
    bundle_path = tmp_path / "g10a_g9_seed_bundle.json"
    bundle_path.write_text(json.dumps(bundle), encoding="utf-8")

    seed = _load_g9_seed(bundle_path)

    assert seed.barrier == barrier
    assert seed.source_mode == (
        "current_canonical_bundle_binding_legacy_probe"
    )
    assert seed.source_probe_file_sha256 == file_sha256(probe_path)
    assert seed.legacy_declared_report_content_sha256 == (
        "7b2abeb9b9138393b39ad3fdd2936c915521be68b353186b297d455bc99f8657"
    )
    assert (
        seed.current_recomputed_probe_content_sha256
        == recomputed_probe_hash
    )
    with pytest.raises(ValueError, match="report content hash mismatch"):
        _load_g9_seed(probe_path)

    probe_path.write_text("{}\n", encoding="utf-8")
    with pytest.raises(ValueError, match="source probe file hash mismatch"):
        _load_g9_seed(bundle_path)


def test_g10a_point_rows_use_rho_but_v3_box_rows_are_hard() -> None:
    family = _full_family()
    store = _ActiveRowStore(
        families=(family,),
        variable_count=3,
        rho_index=2,
    )
    point = _ActiveRow(
        family_name=family.name,
        row_kind="diagnostic_point",
        box_exact=(Fraction(-1), Fraction(0), Fraction(-1), Fraction(0)),
        source=g10a.G10A_G9_ROLE,
        domain_class="inside",
        ranking_score=-1.0,
    )
    point_matrix, _, accepted = store.append((point,))

    assert len(accepted) == 1
    assert point_matrix.shape == (1, 3)
    assert point_matrix.toarray()[0, 2] == 1.0

    hard = _ActiveRow(
        family_name=family.name,
        row_kind="v3_exact_box",
        box_exact=(Fraction(-1), Fraction(0), Fraction(-1), Fraction(0)),
        source="V3_failed_or_unknown_exact_Bernstein_leaf",
        domain_class="inside",
        v3_call_index=1,
        v3_leaf_depth=2,
        v3_leaf_disposition="unresolved",
    )
    hard_matrix, hard_rhs, accepted = store.append((hard,))

    assert len(accepted) == 1
    assert hard_matrix.shape[1] == 3
    assert np.count_nonzero(hard_matrix.toarray()[:, 2]) == 0
    assert np.all(hard_rhs == -G10A_FEEDBACK_NUMERIC_MARGIN)


def test_g10a_incremental_backend_or_deterministic_rebuild_adds_rows() -> None:
    solver = _IncrementalOrRebuildSolver(
        matrix=csr_matrix([[-1.0]]),
        right_hand_side=np.asarray([0.0]),
        objective=np.asarray([-1.0]),
        bounds=((-1.0, 1.0),),
        seed_values=None,
    )
    first = solver.solve(time_limit_s=10.0)
    assert first.values is not None
    assert first.values[0] == pytest.approx(1.0)

    solver.append_rows(csr_matrix([[1.0]]), np.asarray([0.5]))
    second = solver.solve(time_limit_s=10.0)

    assert second.values is not None
    assert second.values[0] == pytest.approx(0.5)
    assert solver.matrix.shape == (2, 1)
    assert solver.backend in {
        "highspy_incremental_rows_basis",
        "scipy_highs_deterministic_sparse_rebuild",
    }


def _fake_numeric_result(*, backend: str, elapsed_s: float) -> _NumericSolveResult:
    return _NumericSolveResult(
        values=np.asarray([0.5]),
        backend=backend,
        backend_status="test_optimal",
        success=True,
        message="test numeric candidate",
        iterations=1,
        solve_time_s=elapsed_s,
        basis_valid_before=None,
        basis_valid_after=None,
        fallback_reason=None,
    )


def test_g10a_failed_highspy_time_is_subtracted_before_rebuild(
    monkeypatch,
) -> None:
    solver = object.__new__(_IncrementalOrRebuildSolver)
    solver._highs = object()
    solver._highspy = object()
    solver.fallback_reason = None
    solver.backend_history = []
    delegated: dict[str, float] = {}

    def fail_highspy(_self, *, time_limit_s):
        delegated["highspy"] = time_limit_s
        raise RuntimeError("synthetic highspy API failure")

    def solve_rebuild(self, *, time_limit_s):
        delegated["rebuild"] = time_limit_s
        result = _fake_numeric_result(
            backend="scipy_highs_deterministic_sparse_rebuild",
            elapsed_s=1.5,
        )
        self.backend_history.append(
            {
                "event": "solve",
                "backend": result.backend,
                "backend_status": result.backend_status,
                "message": result.message,
                "iterations": result.iterations,
                "solve_time_s": result.solve_time_s,
            }
        )
        return result

    clock = iter((0.0, 0.0, 3.0, 3.5, 5.0))
    monkeypatch.setattr(g10a, "monotonic", lambda: next(clock))
    monkeypatch.setattr(
        _IncrementalOrRebuildSolver, "_solve_highspy", fail_highspy
    )
    monkeypatch.setattr(
        _IncrementalOrRebuildSolver, "_solve_rebuild", solve_rebuild
    )

    result = solver.solve(time_limit_s=10.0)

    assert delegated["highspy"] == pytest.approx(10.0)
    assert delegated["rebuild"] == pytest.approx(6.5)
    assert result.values is not None
    assert result.solve_time_s == pytest.approx(5.0)
    assert result.aggregate_budget_exceeded is False
    assert solver.backend_history[-1]["failed_highspy_elapsed_s"] == 3.0


def test_g10a_failed_highspy_cannot_restart_an_exhausted_budget(
    monkeypatch,
) -> None:
    solver = object.__new__(_IncrementalOrRebuildSolver)
    solver._highs = object()
    solver._highspy = object()
    solver.fallback_reason = None
    solver.backend_history = []
    rebuild_called = False

    def fail_highspy(_self, *, time_limit_s):
        assert time_limit_s == pytest.approx(10.0)
        raise RuntimeError("synthetic highspy timeout failure")

    def forbidden_rebuild(_self, *, time_limit_s):
        nonlocal rebuild_called
        rebuild_called = True
        raise AssertionError(f"unexpected rebuild budget {time_limit_s}")

    clock = iter((0.0, 0.0, 11.0, 11.0))
    monkeypatch.setattr(g10a, "monotonic", lambda: next(clock))
    monkeypatch.setattr(
        _IncrementalOrRebuildSolver, "_solve_highspy", fail_highspy
    )
    monkeypatch.setattr(
        _IncrementalOrRebuildSolver, "_solve_rebuild", forbidden_rebuild
    )

    result = solver.solve(time_limit_s=10.0)

    assert rebuild_called is False
    assert result.values is None
    assert result.backend_status == "aggregate_lp_time_budget_exhausted"
    assert result.aggregate_budget_exceeded is True
    assert result.solve_time_s == pytest.approx(11.0)


def test_g10a_discards_backend_candidate_returned_after_wall_budget(
    monkeypatch,
) -> None:
    solver = object.__new__(_IncrementalOrRebuildSolver)
    solver._highs = object()
    solver._highspy = object()
    solver.fallback_reason = None
    solver.backend_history = []

    def late_highspy(self, *, time_limit_s):
        result = _fake_numeric_result(
            backend="highspy_incremental_rows_basis",
            elapsed_s=time_limit_s,
        )
        self.backend_history.append(
            {
                "event": "solve",
                "backend": result.backend,
                "backend_status": result.backend_status,
                "message": result.message,
                "iterations": result.iterations,
                "solve_time_s": result.solve_time_s,
            }
        )
        return result

    clock = iter((0.0, 0.0, 10.5))
    monkeypatch.setattr(g10a, "monotonic", lambda: next(clock))
    monkeypatch.setattr(
        _IncrementalOrRebuildSolver, "_solve_highspy", late_highspy
    )

    result = solver.solve(time_limit_s=10.0)

    assert result.values is None
    assert result.aggregate_budget_exceeded is True
    assert result.backend_status == "aggregate_lp_time_budget_exhausted"


def test_g10a_highspy_add_rows_uses_one_start_per_new_row() -> None:
    class Basis:
        valid = True

    class FakeHighs:
        def __init__(self):
            self.starts = None

        def getBasis(self):
            return Basis()

        def addRows(
            self,
            row_count,
            _lower,
            _upper,
            nonzero_count,
            starts,
            _indices,
            _values,
        ):
            assert row_count == 2
            assert nonzero_count == 3
            self.starts = starts.copy()
            return 0

        def setOptionValue(self, name, value):
            assert (name, value) == ("presolve", "off")
            return 0

    class FakeHighspy:
        kHighsInf = 1.0e30

    solver = object.__new__(_IncrementalOrRebuildSolver)
    solver.matrix = csr_matrix((0, 2), dtype=np.float64)
    solver.right_hand_side = np.empty(0, dtype=np.float64)
    solver._highs = FakeHighs()
    solver._highspy = FakeHighspy()
    solver.fallback_reason = None
    solver.backend_history = []
    block = csr_matrix([[1.0, 0.0], [2.0, 3.0]])

    solver.append_rows(block, np.asarray([1.0, 2.0]))

    assert solver._highs is not None
    assert solver._highs.starts.tolist() == [0, 1]


def test_g10a_highspy_time_limit_accounts_for_cumulative_run_clock(
    monkeypatch,
) -> None:
    class Basis:
        valid = True

    class Solution:
        value_valid = True
        col_value = [0.5]

    class Info:
        simplex_iteration_count = 2

    class FakeHighs:
        def __init__(self):
            self.calls = []

        def getRunTime(self):
            self.calls.append("getRunTime")
            return 12.0

        def setOptionValue(self, name, value):
            self.calls.append((name, value))
            return 0

        def getBasis(self):
            return Basis()

        def run(self):
            self.calls.append("run")
            return 0

        def getModelStatus(self):
            return "test_optimal"

        def getSolution(self):
            return Solution()

        def getInfo(self):
            return Info()

        def modelStatusToString(self, status):
            return str(status)

    solver = object.__new__(_IncrementalOrRebuildSolver)
    solver.matrix = csr_matrix((0, 1), dtype=np.float64)
    solver.seed_values = None
    solver._seed_attempted = False
    solver._highs = FakeHighs()
    solver.fallback_reason = None
    solver.seed_hint_status = None
    solver.backend_history = []
    clock = iter((4.0, 5.0))
    monkeypatch.setattr(g10a, "monotonic", lambda: next(clock))

    result = solver._solve_highspy(time_limit_s=7.0)

    assert result.values is not None
    assert result.solve_time_s == pytest.approx(1.0)
    assert solver._highs.calls[:2] == [
        "getRunTime",
        ("time_limit", 19.0),
    ]


def test_g10a_v3_feedback_consumes_only_terminal_exact_boxes(tmp_path) -> None:
    family = _full_family()
    tree = {
        "status": "unknown",
        "failure_reason": "subdivision_budget_exhausted",
        "required_margin": {"exact": "0", "float": 0.0},
        "acceptance_tolerance": 0.0,
        "root_box_exact": ["-1", "1", "-1", "1"],
        "global_visited_lower_bound": {"exact": "-1/8", "float": -0.125},
        "global_visited_upper_bound": {"exact": "1/4", "float": 0.25},
        "leaf_count": 2,
        "leaves": [
            {
                "box_exact": ["-1", "0", "-1", "0"],
                "depth": 2,
                "domain_class": "inside",
                "lower": {"exact": "1/8", "float": 0.125},
                "upper": {"exact": "1/4", "float": 0.25},
                "disposition": "accepted_by_exact_lower_bound",
            },
            {
                "box_exact": ["0", "1", "0", "1"],
                "depth": 2,
                "domain_class": "inside",
                "lower": {"exact": "-1/8", "float": -0.125},
                "upper": {"exact": "1/8", "float": 0.125},
                "disposition": "unresolved",
            },
        ],
    }
    tree_path = tmp_path / "qsbc_W_nonnegative_on_X.json"
    tree_path.write_text(json.dumps(tree), encoding="utf-8")
    summary = dict(tree)
    summary.pop("leaves")
    summary["tree_file"] = tree_path.name
    summary["tree_file_sha256"] = file_sha256(tree_path)
    validation = {"constraints": {family.name: summary}}

    rows = _v3_feedback_rows(
        validation=validation,
        validation_directory=tmp_path,
        families=(family,),
        v3_call_index=1,
    )

    assert len(rows) == 1
    assert rows[0].row_kind == "v3_exact_box"
    assert rows[0].box_exact == (
        Fraction(0),
        Fraction(1),
        Fraction(0),
        Fraction(1),
    )
    assert rows[0].v3_leaf_disposition == "unresolved"


def test_g10a_feedback_gate_binds_report_tree_and_subject(tmp_path) -> None:
    family = _full_family()
    barrier = SparsePolynomial2D(((0, 0),), (0.0,))
    controller = FrozenPolynomialController(
        controller_id="g10a_binding_test",
        u_x=barrier,
        u_y=barrier,
        upstream_checkpoint_sha256="0" * 64,
    )
    qsbc_tree = {
        "status": "unknown",
        "failure_reason": "subdivision_budget_exhausted",
        "required_margin": {"exact": "0", "float": 0.0},
        "acceptance_tolerance": 0.0,
        "root_box_exact": ["-1", "1", "-1", "1"],
        "global_visited_lower_bound": {"exact": "-1", "float": -1.0},
        "global_visited_upper_bound": {"exact": "1", "float": 1.0},
        "leaf_count": 1,
        "leaves": [
            {
                "box_exact": ["-1", "1", "-1", "1"],
                "depth": 0,
                "domain_class": "inside",
                "lower": {"exact": "-1", "float": -1.0},
                "upper": {"exact": "1", "float": 1.0},
                "disposition": "unresolved",
            }
        ],
    }
    controller_tree = {
        "status": "validated",
        "failure_reason": None,
        "required_margin": {"exact": "1/1000000", "float": 1.0e-6},
        "acceptance_tolerance": 0.0,
        "root_box_exact": ["-1", "1", "-1", "1"],
        "global_visited_lower_bound": {"exact": "1", "float": 1.0},
        "global_visited_upper_bound": {"exact": "1", "float": 1.0},
        "leaf_count": 1,
        "leaves": [
            {
                "box_exact": ["-1", "1", "-1", "1"],
                "depth": 0,
                "domain_class": "inside",
                "lower": {"exact": "1", "float": 1.0},
                "upper": {"exact": "1", "float": 1.0},
                "disposition": "accepted_by_exact_lower_bound",
            }
        ],
    }

    def write_tree(name, tree):
        path = tmp_path / name
        path.write_text(json.dumps(tree), encoding="utf-8")
        summary = dict(tree)
        summary.pop("leaves")
        summary["tree_file"] = name
        summary["tree_file_sha256"] = file_sha256(path)
        return path, summary

    qsbc_path, qsbc_summary = write_tree("qsbc.json", qsbc_tree)
    _, controller_summary = write_tree("controller.json", controller_tree)
    model = g10a.canonical_rational_model_payload()
    controller_constraint_names = {
        f"global_{suffix}"
        for suffix in g10a.G10A_CONTROLLER_CONSTRAINT_SUFFIXES
    }
    validation = {
        "schema_version": 2,
        "validator_id": g10a.G10_VALIDATOR_ID,
        "status": "unknown",
        "global_polynomial_status": "unknown",
        "controller_contract_status": "validated",
        "geometry_contract_status": "validated",
        "formal_acceptance_tolerance": 0.0,
        "q_float64_hex": g10a.G10_Q.hex(),
        "q_exact": str(Fraction.from_float(g10a.G10_Q)),
        "epsbar_float64_hex": g10a.G10_EPSBAR.hex(),
        "epsbar_exact": str(Fraction.from_float(g10a.G10_EPSBAR)),
        "barrier_payload_sha256": content_sha256(barrier.payload()),
        "barrier_degree": barrier.degree,
        "barrier_term_count": len(barrier.powers),
        "first_slice_ready": False,
        "controller_content_sha256": controller.sha256,
        "formal_model_contract": model,
        "formal_model_contract_sha256": content_sha256(model),
        "theorem_scalar_gate": True,
        "constraints": {family.name: qsbc_summary},
        "controller_constraints": {
            name: dict(controller_summary)
            for name in controller_constraint_names
        },
        "independence": {
            "solver_status_read": False,
            "sampled_residual_used": False,
            "float_polynomial_algebra_used_for_status": False,
            "exact_binary64_artifact_coefficients": True,
            "exact_decimal_model_constants": True,
            "controller_S1_and_D1_replayed": True,
        },
    }
    validation["report_content_sha256"] = content_sha256(validation)

    assert _v3_feedback_binding_failures(
        validation=validation,
        validation_directory=tmp_path,
        barrier=barrier,
        controller=controller,
        families=(family,),
    ) == ()

    missing_controller_family = dict(validation)
    missing_controller_family["controller_constraints"] = dict(
        validation["controller_constraints"]
    )
    missing_controller_family["controller_constraints"].pop(
        next(iter(controller_constraint_names))
    )
    missing_controller_family.pop("report_content_sha256")
    missing_controller_family["report_content_sha256"] = content_sha256(
        missing_controller_family
    )
    assert "controller_constraint_set" in _v3_feedback_binding_failures(
        validation=missing_controller_family,
        validation_directory=tmp_path,
        barrier=barrier,
        controller=controller,
        families=(family,),
    )

    qsbc_path.write_text("{}\n", encoding="utf-8")
    failures = _v3_feedback_binding_failures(
        validation=validation,
        validation_directory=tmp_path,
        barrier=barrier,
        controller=controller,
        families=(family,),
    )
    assert f"constraints.{family.name}.tree_file_sha256" in failures


def test_g10a_rejects_noncanonical_or_degenerate_exact_boxes() -> None:
    with pytest.raises(ValueError, match="not canonical"):
        _parse_exact_box(["-2/2", "1", "-1", "1"])
    with pytest.raises(ValueError, match="ordered"):
        _parse_exact_box(["0", "0", "-1", "1"])


def test_g10a_schedule_execution_error_fails_closed(tmp_path, monkeypatch) -> None:
    def fail_stage(**_kwargs):
        raise RuntimeError("declared G10A test failure")

    monkeypatch.setattr(g10a, "solve_g10a_stage", fail_stage)
    seed_path = tmp_path / "seed.json"
    seed_path.write_text("{}\n", encoding="utf-8")
    manifest_path = g10a.synthesize_g10a_qsbc_schedule(
        frozen_controller_path=tmp_path / "unused_controller.json",
        g9_seed_path=seed_path,
        output_dir=tmp_path / "schedule",
    )
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))

    assert manifest["status"] == "not_ready"
    assert manifest["terminal_reason"] == (
        "G10A_stage_execution_error_fail_closed"
    )
    assert manifest["attempts"][0]["formal_claim_allowed"] is False
