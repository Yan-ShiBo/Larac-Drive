"""Base contracts shared by analytic, learned and formally verified certificates."""

from __future__ import annotations

from dataclasses import dataclass, field
from math import isfinite
from typing import Any, Mapping, Protocol, TypeVar, runtime_checkable

from ..claims import SafetyClaim


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
        if not isinstance(self.name, str) or not self.name.strip():
            raise ValueError("CertificateEvaluation.name must not be empty")
        if not isinstance(self.satisfied, bool):
            raise TypeError("CertificateEvaluation.satisfied must be boolean")
        if not isfinite(self.value):
            raise ValueError("CertificateEvaluation.value must be finite")
        if not isfinite(self.violation) or self.violation < 0.0:
            raise ValueError(
                "CertificateEvaluation.violation must be finite and non-negative"
            )


@runtime_checkable
class Certificate(Protocol[StateT, ActionT]):
    """Evaluate one certificate condition without changing the action."""

    name: str

    def evaluate(
        self,
        state: StateT,
        action: ActionT | None = None,
        context: Mapping[str, Any] | None = None,
    ) -> CertificateEvaluation:
        """Evaluate the configured condition."""
