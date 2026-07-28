"""Controller and episode-lifecycle contracts for the research domain."""

from __future__ import annotations

from typing import Any, Mapping, Protocol, TypeVar, runtime_checkable


ObservationT = TypeVar("ObservationT", contravariant=True)
ActionT = TypeVar("ActionT", covariant=True)
ExecutedStateT = TypeVar("ExecutedStateT", contravariant=True)
ExecutedActionT = TypeVar("ExecutedActionT", contravariant=True)


@runtime_checkable
class Controller(Protocol[ObservationT, ActionT]):
    """Produce a nominal action before any optional safety modification."""

    name: str

    def act(
        self,
        observation: ObservationT,
        context: Mapping[str, Any] | None = None,
    ) -> ActionT:
        """Return the nominal action for one control step."""


@runtime_checkable
class EpisodeAware(Protocol):
    """Optional lifecycle hook for stateful controllers and methods."""

    def reset_episode(self, *, seed: int | None = None) -> None:
        """Reset state that must not leak between episodes."""


@runtime_checkable
class ExecutedActionAware(Protocol[ExecutedStateT, ExecutedActionT]):
    """Optional hook for methods whose state advances after action execution."""

    def update_after_step(
        self,
        state: ExecutedStateT,
        executed_action: ExecutedActionT,
        context: Mapping[str, Any] | None = None,
    ) -> None:
        """Advance internal state using the action accepted by the environment."""
