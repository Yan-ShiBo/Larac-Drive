from __future__ import annotations

import json

import pytest

from cbf_rover.methods.polynomial_sbc.polynomial import (
    FrozenPiecewisePolynomialController,
    FrozenPolynomialController,
    PolynomialControllerPiece,
    SparsePolynomial2D,
    load_frozen_controller,
)


def _controller() -> FrozenPolynomialController:
    return FrozenPolynomialController(
        controller_id="artifact_round_trip",
        u_x=SparsePolynomial2D(((0, 0), (1, 0)), (0.1, -0.2)),
        u_y=SparsePolynomial2D(((0, 0), (0, 1)), (-0.1, 0.2)),
        upstream_checkpoint_sha256="a" * 64,
        construction={"stage": 1},
    )


def test_controller_round_trip_preserves_semantic_hash(tmp_path) -> None:
    controller = _controller()
    path = controller.save(tmp_path / "controller.json")
    loaded = FrozenPolynomialController.load(path)
    assert loaded == controller
    assert loaded.sha256 == controller.sha256


def test_controller_loader_fails_closed_on_coefficient_edit(tmp_path) -> None:
    path = _controller().save(tmp_path / "controller.json")
    payload = json.loads(path.read_text(encoding="utf-8"))
    payload["u_x"]["coefficients_decimal"][0] += 0.001
    path.write_text(json.dumps(payload), encoding="utf-8")
    with pytest.raises(ValueError):
        FrozenPolynomialController.load(path)


def _piecewise_controller() -> FrozenPiecewisePolynomialController:
    zero = SparsePolynomial2D(((0, 0),), (0.0,))
    return FrozenPiecewisePolynomialController(
        controller_id="piecewise_artifact_round_trip",
        pieces=(
            PolynomialControllerPiece(
                "left",
                -1.0,
                0.8,
                SparsePolynomial2D(((0, 0),), (0.1,)),
                zero,
            ),
            PolynomialControllerPiece(
                "right",
                0.8,
                4.0,
                SparsePolynomial2D(((0, 0),), (0.2,)),
                zero,
            ),
        ),
        construction_source_sha256="d" * 64,
    )


def test_piecewise_round_trip_and_generic_loader_preserve_identity(
    tmp_path,
) -> None:
    controller = _piecewise_controller()
    path = controller.save(tmp_path / "piecewise.json")
    loaded = load_frozen_controller(path)
    assert loaded == controller
    assert loaded.sha256 == controller.sha256


def test_piecewise_runtime_uses_ordered_first_match_at_shared_boundary() -> None:
    from cbf_rover.domain.planar import PlanarState

    controller = _piecewise_controller()
    assert controller.act(PlanarState(0.8, 0.0)).u_x_mps == 0.1
    assert controller.act(PlanarState(0.800001, 0.0)).u_x_mps == 0.2
    with pytest.raises(ValueError):
        controller.act(PlanarState(4.1, 0.0))


def test_piecewise_partition_rejects_a_gap() -> None:
    zero = SparsePolynomial2D(((0, 0),), (0.0,))
    with pytest.raises(ValueError):
        FrozenPiecewisePolynomialController(
            controller_id="gap",
            pieces=(
                PolynomialControllerPiece("left", -1.0, 0.0, zero, zero),
                PolynomialControllerPiece("right", 0.1, 4.0, zero, zero),
            ),
            construction_source_sha256="e" * 64,
        )
