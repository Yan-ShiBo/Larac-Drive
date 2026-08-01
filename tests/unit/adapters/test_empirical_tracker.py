from __future__ import annotations

from math import isclose, pi

from cbf_rover.adapters.gazebo.tracker import (
    EmpiricalDifferentialTracker,
    wrap_angle,
)
from cbf_rover.domain.planar import PlanarAction


def test_tracker_preserves_forward_velocity_when_heading_matches() -> None:
    command = EmpiricalDifferentialTracker().command(
        PlanarAction(0.3, 0.0),
        heading_rad=0.0,
    )
    assert isclose(command.linear_mps, 0.3)
    assert command.angular_radps == 0.0
    assert not command.saturation_applied
    assert command.claim_level == "empirical"


def test_tracker_saturation_is_explicit_and_empirical() -> None:
    command = EmpiricalDifferentialTracker().command(
        PlanarAction(0.0, 0.5),
        heading_rad=-pi / 2.0,
    )
    assert abs(command.angular_radps) == 1.5
    assert command.saturation_applied


def test_wrap_angle_uses_half_open_principal_interval() -> None:
    assert -pi <= wrap_angle(7.0) < pi
