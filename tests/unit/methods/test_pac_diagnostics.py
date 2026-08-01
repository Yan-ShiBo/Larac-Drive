from __future__ import annotations

import numpy as np

from cbf_rover.methods.polynomial_sbc.diagnostics import (
    _error_summary,
    _geometric_zone,
    _safe_grid_mask,
)


def test_error_summary_reports_old_pb1_gate_fraction() -> None:
    summary = _error_summary(np.asarray([0.01, 0.05, 0.06]))

    assert summary["max_mps"] == 0.06
    assert summary["fraction_above_pb1_gate"] == 1.0 / 3.0


def test_safe_grid_mask_excludes_obstacles_and_goal() -> None:
    x_values = np.asarray([0.0, 1.0, 3.5])
    y_values = np.asarray([0.0])

    mask = _safe_grid_mask(x_values, y_values)

    assert mask.tolist() == [[True, False, False]]


def test_geometric_zone_tracks_wall_and_route_corridor() -> None:
    assert _geometric_zone(0.5, 0.0) == "pre_wall/interior_band"
    assert _geometric_zone(2.5, 1.6) == "wall_span/upper_corridor"
    assert _geometric_zone(3.5, -1.6) == "post_wall/lower_corridor"
