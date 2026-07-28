"""Optional action-modification contracts with auditable results."""

from __future__ import annotations

from dataclasses import dataclass, field
from math import isfinite
from typing import Any, Generic, Mapping, Protocol, Sequence, TypeVar, runtime_checkable

from .certificates import CertificateEvaluation


StateT = TypeVar("StateT", contravariant=True)
ActionT = TypeVar("ActionT")


@dataclass(frozen=True)
class ActionModificationResult(Generic[ActionT]):
    """Candidate action and diagnostics returned by an action modifier."""

    reference_action: ActionT
    modified_action: ActionT
    intervened: bool
    intervention_norm: float
    feasible: bool
    certificates: Sequence[CertificateEvaluation] = ()
    metadata: Mapping[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        if not isinstance(self.intervened, bool):
            raise TypeError("ActionModificationResult.intervened must be boolean")
        if not isinstance(self.feasible, bool):
            raise TypeError("ActionModificationResult.feasible must be boolean")
        if not isfinite(self.intervention_norm) or self.intervention_norm < 0.0:
            raise ValueError(
                "ActionModificationResult.intervention_norm must be finite "
                "and non-negative"
            )
        if self.intervened != (self.intervention_norm > 0.0):
            raise ValueError(
                "ActionModificationResult.intervened must agree with "
                "intervention_norm"
            )
        object.__setattr__(self, "certificates", tuple(self.certificates))


@runtime_checkable
class ActionModifier(Protocol[StateT, ActionT]):
    """Optionally modify a nominal action and report whether it is feasible."""

    name: str

    def modify(
        self,
        reference_action: ActionT,
        state: StateT,
        context: Mapping[str, Any] | None = None,
    ) -> ActionModificationResult[ActionT]:
        """Return one action candidate and its diagnostics."""
