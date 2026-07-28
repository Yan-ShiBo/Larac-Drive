"""Certificate and safety-filter contracts with auditable results."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Generic, Mapping, Protocol, Sequence, TypeVar, runtime_checkable

from .claims import SafetyClaim


StateT = TypeVar("StateT", contravariant=True)
ActionT = TypeVar("ActionT")


@dataclass(frozen=True)
class CertificateEvaluation:
    """Normalized evaluation of one configured certificate condition."""

    name: str
    value: float
    satisfied: bool
    violation: float = 0.0
    claim: SafetyClaim | None = None
    metadata: Mapping[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        if not self.name.strip():
            raise ValueError("CertificateEvaluation.name must not be empty")
        if self.violation < 0.0:
            raise ValueError("CertificateEvaluation.violation must be non-negative")


@dataclass(frozen=True)
class FilterResult(Generic[ActionT]):
    """Action-filter output shared by Python and simulation adapters."""

    reference_action: ActionT
    filtered_action: ActionT
    intervened: bool
    intervention_norm: float
    feasible: bool
    certificates: Sequence[CertificateEvaluation] = ()
    metadata: Mapping[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        if self.intervention_norm < 0.0:
            raise ValueError("FilterResult.intervention_norm must be non-negative")

    @property
    def is_feasible(self) -> bool:
        """Compatibility spelling used by older simulation adapters."""

        return self.feasible

    @property
    def certificate_results(self) -> Sequence[CertificateEvaluation]:
        """Compatibility spelling used by older reporting code."""

        return self.certificates


@runtime_checkable
class Certificate(Protocol[StateT, ActionT]):
    """Evaluate a learned, analytic or formally verified condition."""

    name: str

    def evaluate(
        self,
        state: StateT,
        action: ActionT | None = None,
        context: Mapping[str, Any] | None = None,
    ) -> CertificateEvaluation:
        """Evaluate the configured condition."""


@runtime_checkable
class SafetyFilter(Protocol[StateT, ActionT]):
    """Return an action candidate and diagnostics for one control step."""

    name: str

    def filter(
        self,
        reference_action: ActionT,
        state: StateT,
        context: Mapping[str, Any] | None = None,
    ) -> FilterResult[ActionT]:
        """Filter one reference action."""
