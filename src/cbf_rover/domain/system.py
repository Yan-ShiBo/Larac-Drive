"""System-model contracts with explicit uncertainty metadata."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Generic, Mapping, Protocol, TypeVar, runtime_checkable


StateT = TypeVar("StateT")
ActionT = TypeVar("ActionT", contravariant=True)


@dataclass(frozen=True)
class Prediction(Generic[StateT]):
    """One model prediction and any uncertainty information it exposes."""

    state: StateT
    metadata: Mapping[str, Any] = field(default_factory=dict)


@runtime_checkable
class DynamicsModel(Protocol[StateT, ActionT]):
    """Predict a future state without prescribing a concrete tensor type."""

    name: str

    def predict(
        self,
        state: StateT,
        action: ActionT,
        context: Mapping[str, Any] | None = None,
    ) -> Prediction[StateT]:
        """Predict the next state or a representative of its distribution."""
