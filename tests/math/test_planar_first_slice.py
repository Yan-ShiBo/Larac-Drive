from __future__ import annotations

from math import isclose

import pytest

from cbf_rover.domain.planar import (
    CANONICAL_SCENARIO,
    CANONICAL_SYSTEM,
    PlanarAction,
    PlanarState,
)


def test_p1_geometry_is_exactly_the_confirmed_conservative_projection() -> None:
    scenario = CANONICAL_SCENARIO
    assert (
        scenario.domain.x_min,
        scenario.domain.x_max,
        scenario.domain.y_min,
        scenario.domain.y_max,
    ) == (-1.0, 4.0, -2.0, 2.0)
    assert [
        (box.x_min, box.x_max, box.y_min, box.y_max)
        for box in scenario.unsafe_components
    ] == [
        (0.965, 2.035, -0.535, 0.535),
        (1.915, 3.085, 0.215, 1.385),
        (1.915, 3.085, -1.385, -0.215),
    ]
    assert scenario.initial.contains(PlanarState(0.1, 0.0))
    assert scenario.goal.contains(PlanarState(3.5, 0.25))


def test_n1_exact_moment_family() -> None:
    noise = CANONICAL_SYSTEM.noise
    assert noise.moment(0) == 1.0
    assert noise.moment(1) == 0.0
    assert noise.moment(3) == 0.0
    assert isclose(noise.moment(2), 0.005**2 / 6.0)
    assert isclose(noise.moment(4), 2.0 * 0.005**4 / (5.0 * 6.0))
    assert isclose(noise.mixed_moment(2, 2), noise.moment(2) ** 2)
    with pytest.raises(ValueError):
        noise.moment(-1)


def test_m1_transition_has_no_hidden_action_or_state_clipping() -> None:
    system = CANONICAL_SYSTEM
    state = PlanarState(1.0, -0.5)
    action = PlanarAction(0.3, -0.4)
    successor = system.successor(state, action, (0.005, -0.005))
    assert successor.x_m == pytest.approx(1.02)
    assert successor.y_m == pytest.approx(-0.525)
    assert system.action_is_admissible(action)
    assert not system.action_is_admissible(PlanarAction(0.5, 0.01))
    with pytest.raises(ValueError):
        system.successor(state, action, (0.0050001, 0.0))


def test_d1_outside_successor_is_a_contract_breach() -> None:
    scenario = CANONICAL_SCENARIO
    assert (
        scenario.terminal_reason(PlanarState(-1.0000001, 0.0))
        == "model_contract_breach"
    )
