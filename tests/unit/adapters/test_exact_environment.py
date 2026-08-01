from __future__ import annotations

import numpy as np

from cbf_rover.adapters.exact.environment import (
    ExactPlanarEnvironment,
    ReferenceActionCodec,
    sample_uniform_psi,
)
from cbf_rover.domain.planar import (
    CANONICAL_SCENARIO,
    PlanarAction,
    PlanarState,
)


def test_reference_codec_maps_the_square_inside_conservative_disk() -> None:
    codec = ReferenceActionCodec()
    for raw_action in (
        (-1.0, -1.0),
        (-1.0, 1.0),
        (1.0, -1.0),
        (1.0, 1.0),
        (0.2, -0.8),
        (0.0, 0.0),
    ):
        action = codec.decode(raw_action)
        assert action.squared_speed_m2ps2 <= 0.42**2 + 1.0e-15


def test_uniform_psi_rejection_sampling_is_seed_reproducible() -> None:
    left = sample_uniform_psi(np.random.default_rng(123), 100)
    right = sample_uniform_psi(np.random.default_rng(123), 100)
    np.testing.assert_array_equal(left, right)
    assert all(
        CANONICAL_SCENARIO.is_nonterminal_safe(PlanarState(row[0], row[1]))
        for row in left
    )


def test_exact_environment_fails_closed_on_action_violation() -> None:
    environment = ExactPlanarEnvironment(reset_mode="initial_disk")
    state = environment.reset(seed=7)
    next_state, _, terminated, truncated, info = environment.step(
        PlanarAction(0.5, 0.5)
    )
    assert next_state == state
    assert terminated
    assert not truncated
    assert info["termination_reason"] == "action_contract_breach"
