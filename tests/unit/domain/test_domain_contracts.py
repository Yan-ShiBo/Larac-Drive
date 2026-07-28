from __future__ import annotations

from collections.abc import Callable
from math import inf

import pytest

from cbf_rover.domain import (
    ActionModificationResult,
    ActionVector,
    CertificateEvaluation,
    ClaimKind,
    SafetyClaim,
    StateVector,
)


def test_action_vector_preserves_semantics_and_distance() -> None:
    reference = ActionVector.from_sequence(
        (0.2, -0.1),
        labels=("linear", "angular"),
        units=("m/s", "rad/s"),
        frame_id="base_link",
    )
    modified = ActionVector.from_sequence(
        (0.2, 0.2),
        labels=("linear", "angular"),
        units=("m/s", "rad/s"),
        frame_id="base_link",
    )

    assert reference.values == (0.2, -0.1)
    assert reference.labels == ("linear", "angular")
    assert reference.units == ("m/s", "rad/s")
    assert reference.frame_id == "base_link"
    assert reference.distance_to(modified) == pytest.approx(0.3)

    incompatible = ActionVector.from_sequence(
        (0.2, 0.2),
        labels=("linear", "angular"),
        units=("normalized", "normalized"),
        frame_id="policy",
    )
    with pytest.raises(ValueError, match="identical labels, units, and frame_id"):
        reference.distance_to(incompatible)


@pytest.mark.parametrize(
    ("values", "labels", "units"),
    [
        ((), (), ()),
        ((inf,), (), ()),
        ((0.0, 1.0), ("linear",), ()),
        ((0.0, 1.0), (), ("m/s",)),
    ],
)
def test_action_vector_rejects_invalid_coordinates_or_semantics(
    values: tuple[float, ...],
    labels: tuple[str, ...],
    units: tuple[str, ...],
) -> None:
    with pytest.raises(ValueError):
        ActionVector(
            values=values,
            labels=labels,
            units=units,
            frame_id="base_link",
        )


def test_state_vector_requires_finite_coordinates_and_aligned_semantics() -> None:
    state = StateVector.from_sequence(
        (1.0, 2.0, 0.5),
        labels=("x", "y", "heading"),
        units=("m", "m", "rad"),
        frame_id="world",
    )

    assert state.values == (1.0, 2.0, 0.5)
    assert state.frame_id == "world"
    with pytest.raises(ValueError, match="finite"):
        StateVector(
            values=(inf,),
            labels=("x",),
            units=("m",),
            frame_id="world",
        )
    with pytest.raises(ValueError, match="labels"):
        StateVector(
            values=(1.0, 2.0),
            labels=("x",),
            units=("m", "m"),
            frame_id="world",
        )
    with pytest.raises(ValueError, match="frame_id"):
        StateVector(
            values=(1.0,),
            labels=("x",),
            units=("m",),
            frame_id="",
        )


def test_certificate_and_action_modification_keep_empirical_claim_auditable() -> None:
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
    result = ActionModificationResult(
        reference_action=(0.5, 0.2),
        modified_action=(0.4, 0.1),
        intervened=True,
        intervention_norm=0.2,
        feasible=True,
        certificates=[evaluation],
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
                value=inf,
                satisfied=False,
            ),
            "finite",
        ),
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
            lambda: ActionModificationResult(
                reference_action=(0.0, 0.0),
                modified_action=(0.0, 0.0),
                intervened=False,
                intervention_norm=-0.1,
                feasible=True,
            ),
            "non-negative",
        ),
        (
            lambda: ActionModificationResult(
                reference_action=(0.0, 0.0),
                modified_action=(0.0, 0.0),
                intervened=True,
                intervention_norm=0.0,
                feasible=True,
            ),
            "must agree",
        ),
        (
            lambda: SafetyClaim(kind=ClaimKind.DIAGNOSTIC, statement="  "),
            "must not be empty",
        ),
    ],
)
def test_domain_safety_records_reject_invalid_values(
    factory: Callable[[], object],
    message: str,
) -> None:
    with pytest.raises(ValueError, match=message):
        factory()
