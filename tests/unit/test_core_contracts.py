from __future__ import annotations

from collections.abc import Callable
from math import inf

import pytest

from cbf_rover.core import (
    Action,
    CertificateEvaluation,
    ClaimKind,
    FilterResult,
    Observation,
    RoverState,
    SafetyClaim,
)


def test_action_has_finite_values_and_a_stable_distance_contract() -> None:
    reference = Action.from_sequence((0.2, -0.1), labels=("linear", "angular"))
    filtered = Action.from_sequence((0.2, 0.2), labels=("linear", "angular"))

    assert reference.values == (0.2, -0.1)
    assert reference.labels == ("linear", "angular")
    assert reference.distance_to(filtered) == pytest.approx(0.3)


@pytest.mark.parametrize(
    ("values", "labels"),
    [
        ((), ()),
        ((inf,), ()),
        ((0.0, 1.0), ("linear",)),
    ],
)
def test_action_rejects_invalid_vectors(
    values: tuple[float, ...],
    labels: tuple[str, ...],
) -> None:
    with pytest.raises(ValueError):
        Action(values=values, labels=labels)


def test_transport_independent_state_rejects_invalid_timing() -> None:
    observation = Observation(payload={"depth": "opaque"}, timestamp_s=1.25)

    with pytest.raises(ValueError, match="step_index"):
        RoverState(observation=observation, step_index=-1, elapsed_s=0.0)
    with pytest.raises(ValueError, match="elapsed_s"):
        RoverState(observation=observation, step_index=0, elapsed_s=-0.01)


def test_certificate_and_filter_results_preserve_explicit_empirical_claim() -> None:
    claim = SafetyClaim(
        kind=ClaimKind.EMPIRICAL,
        statement="Evaluated only in the configured simulation scenario.",
        assumptions=("server scenario manifest is unchanged",),
    )
    evaluation = CertificateEvaluation(
        name="learned_cbf_like_score",
        value=0.4,
        satisfied=True,
        claim=claim,
    )
    result = FilterResult(
        reference_action=(0.5, 0.2),
        filtered_action=(0.4, 0.1),
        intervened=True,
        intervention_norm=0.2,
        feasible=True,
        certificates=(evaluation,),
    )

    assert result.certificates == (evaluation,)
    assert result.certificates[0].claim == claim
    assert result.intervention_norm == pytest.approx(0.2)
    assert result.feasible is True


@pytest.mark.parametrize(
    ("factory", "message"),
    [
        (
            lambda: CertificateEvaluation(
                name="certificate",
                value=0.0,
                satisfied=False,
                violation=-0.1,
            ),
            "non-negative",
        ),
        (
            lambda: FilterResult(
                reference_action=(0.0, 0.0),
                filtered_action=(0.0, 0.0),
                intervened=False,
                intervention_norm=-0.1,
                feasible=True,
            ),
            "non-negative",
        ),
        (
            lambda: SafetyClaim(kind=ClaimKind.DIAGNOSTIC, statement="  "),
            "must not be empty",
        ),
    ],
)
def test_core_contracts_reject_invalid_safety_records(
    factory: Callable[[], object],
    message: str,
) -> None:
    with pytest.raises(ValueError, match=message):
        factory()
