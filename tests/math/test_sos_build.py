from __future__ import annotations

import pytest

from cbf_rover.methods.polynomial_sbc.polynomial import (
    FrozenPiecewisePolynomialController,
    FrozenPolynomialController,
    PolynomialControllerPiece,
    SparsePolynomial2D,
)


@pytest.mark.research_slow
@pytest.mark.server_integration
def test_mosek_fusion_builds_the_full_degree_four_sos_task(tmp_path) -> None:
    from mosek.fusion import Model

    from cbf_rover.methods.polynomial_sbc.sos import (
        SOSSynthesisOptions,
        _build_symbolic_problem,
    )

    controller = FrozenPolynomialController(
        controller_id="sos_build_fixture",
        u_x=SparsePolynomial2D(((1, 0),), (-0.2,)),
        u_y=SparsePolynomial2D(((0, 1),), (-0.2,)),
        upstream_checkpoint_sha256="c" * 64,
    )
    with Model("cbf_rover_sos_build_test") as model:
        _, _, _, powers, grams = _build_symbolic_problem(
            model,
            controller=controller,
            barrier_degree=4,
            options=SOSSynthesisOptions(),
        )
        assert len(powers) == 15
        assert len(grams) > 20
        model.writeTask(str(tmp_path / "build_test.ptf"))


@pytest.mark.research_slow
@pytest.mark.server_integration
def test_mosek_license_and_solver_parameters_reach_optimal_status() -> None:
    from mosek.fusion import Domain, Model, ObjectiveSense

    with Model("cbf_rover_mosek_license_test") as model:
        value = model.variable("value", 1, Domain.greaterThan(1.0))
        model.objective(ObjectiveSense.Minimize, value.index(0))
        model.setSolverParam("optimizerMaxTime", 30.0)
        model.setSolverParam("numThreads", 2)
        model.setSolverParam("intpntCoTolRelGap", 1.0e-8)
        model.solve()
        assert str(model.getPrimalSolutionStatus()).casefold().endswith(
            "optimal"
        )
        assert value.level()[0] == pytest.approx(1.0, abs=1.0e-7)


@pytest.mark.research_slow
@pytest.mark.server_integration
def test_mosek_fusion_builds_piecewise_drift_cells(tmp_path) -> None:
    from mosek.fusion import Model

    from cbf_rover.methods.polynomial_sbc.sos import (
        SOSSynthesisOptions,
        _build_symbolic_problem,
    )

    inward_x = SparsePolynomial2D(((1, 0),), (-0.2,))
    inward_y = SparsePolynomial2D(((0, 1),), (-0.2,))
    controller = FrozenPiecewisePolynomialController(
        controller_id="piecewise_sos_build_fixture",
        pieces=(
            PolynomialControllerPiece(
                "left",
                -1.0,
                0.8,
                inward_x,
                inward_y,
            ),
            PolynomialControllerPiece(
                "middle",
                0.8,
                3.2,
                inward_x,
                inward_y,
            ),
            PolynomialControllerPiece(
                "right",
                3.2,
                4.0,
                inward_x,
                inward_y,
            ),
        ),
        construction_source_sha256="2" * 64,
    )
    with Model("cbf_rover_piecewise_sos_build_test") as model:
        _, _, _, powers, grams = _build_symbolic_problem(
            model,
            controller=controller,
            barrier_degree=4,
            options=SOSSynthesisOptions(),
        )
        assert len(powers) == 15
        assert len(grams) > 20
        model.writeTask(str(tmp_path / "piecewise_build_test.ptf"))
