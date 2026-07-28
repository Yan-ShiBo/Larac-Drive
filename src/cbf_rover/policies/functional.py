"""Adapter for using a plain callable as an experiment policy."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Callable, Generic, Mapping, TypeVar


ObservationT = TypeVar("ObservationT")
ActionT = TypeVar("ActionT")


@dataclass
class FunctionalPolicy(Generic[ObservationT, ActionT]):
    """Named policy backed by a dependency-injected callable."""

    name: str
    function: Callable[[ObservationT, Mapping[str, Any] | None], ActionT]

    def act(
        self,
        observation: ObservationT,
        context: Mapping[str, Any] | None = None,
    ) -> ActionT:
        return self.function(observation, context)
