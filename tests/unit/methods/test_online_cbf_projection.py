from __future__ import annotations

import pytest

from cbf_rover.domain.planar import PlanarAction
from cbf_rover.methods.online_cbf import (
    InfeasibleProjectionError,
    LinearHalfspace2D,
    project_action_to_halfspaces_and_disk,
)


def test_projection_preserves_a_feasible_nominal_action() -> None:
    nominal = PlanarAction(0.2, -0.1)
    result = project_action_to_halfspaces_and_disk(
        nominal,
        (LinearHalfspace2D("forward", 1.0, 0.0, 0.1),),
        max_speed_mps=0.5,
    )

    assert result.action == nominal
    assert result.objective_m2ps2 == 0.0
    assert dict(result.constraint_residuals_mps)["forward"] == pytest.approx(
        0.1
    )


def test_projection_finds_pairwise_boundary_inside_speed_disk() -> None:
    result = project_action_to_halfspaces_and_disk(
        PlanarAction(0.0, 0.0),
        (
            LinearHalfspace2D("x", 1.0, 0.0, 0.4),
            LinearHalfspace2D("y", 0.0, 1.0, 0.3),
        ),
        max_speed_mps=0.5,
    )

    assert result.action.u_x_mps == pytest.approx(0.4)
    assert result.action.u_y_mps == pytest.approx(0.3)
    assert result.action.squared_speed_m2ps2 == pytest.approx(0.25)
    assert set(result.active_constraint_names) == {"x", "y"}


def test_projection_rejects_an_empty_halfspace_disk_intersection() -> None:
    with pytest.raises(InfeasibleProjectionError):
        project_action_to_halfspaces_and_disk(
            PlanarAction(0.0, 0.0),
            (LinearHalfspace2D("impossible", 1.0, 0.0, 0.6),),
            max_speed_mps=0.5,
        )
