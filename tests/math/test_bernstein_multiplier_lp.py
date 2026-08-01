from __future__ import annotations

from cbf_rover.methods.polynomial_sbc.bernstein_multiplier_lp import (
    _uniform_refinement,
    refine_goal_boundary_box,
)


def test_uniform_refinement_splits_both_axes() -> None:
    boxes = _uniform_refinement((-1.0, 1.0, -1.0, 1.0), 2)

    assert len(boxes) == 16
    assert boxes[0] == (-1.0, -0.5, -1.0, -0.5)
    assert boxes[-1] == (0.5, 1.0, 0.5, 1.0)


def test_goal_boundary_refinement_keeps_exact_classes() -> None:
    boxes = refine_goal_boundary_box(
        (0.68, 1.0, -0.2675, 0.2675),
        depth=0,
        maximum_depth=2,
    )

    assert len(boxes) > 1
    assert {classification for _, classification in boxes} <= {
        "inside",
        "outside",
        "boundary",
    }
    assert any(classification == "boundary" for _, classification in boxes)
