"""Generic episode runner contracts for simulation adapters."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Callable, Generic, Mapping, Protocol, TypeVar, runtime_checkable

from cbf_rover.core import FilterResult, SafetyFilter
from cbf_rover.core.contracts import Policy

from .episode import EpisodeConfig, EpisodeLifecycle, EpisodeOutcome


ObservationT = TypeVar("ObservationT")
StateT = TypeVar("StateT")
ActionT = TypeVar("ActionT")


@dataclass(frozen=True)
class StepTransition(Generic[ObservationT, ActionT]):
    observation: ObservationT
    executed_action: ActionT | None = None
    reward: float = 0.0
    cost: float = 0.0
    terminated: bool = False
    truncated: bool = False
    reason: str = ""
    metadata: Mapping[str, Any] = field(default_factory=dict)


@runtime_checkable
class Environment(Protocol[ObservationT, ActionT]):
    """Minimum simulator API required by :func:`run_episode`."""

    def reset(self, *, seed: int) -> ObservationT:
        """Reset and return the first observation."""

    def step(self, action: ActionT) -> StepTransition[ObservationT, ActionT]:
        """Apply an action and return the resulting transition."""


StepCallback = Callable[
    [
        int,
        ObservationT,
        ActionT,
        FilterResult[ActionT],
        StepTransition[ObservationT, ActionT],
    ],
    None,
]


def run_episode(
    *,
    environment: Environment[ObservationT, ActionT],
    policy: Policy[ObservationT, ActionT],
    safety_filter: SafetyFilter[StateT, ActionT],
    state_adapter: Callable[[ObservationT], StateT],
    config: EpisodeConfig,
    on_step: StepCallback[ObservationT, ActionT] | None = None,
) -> EpisodeOutcome:
    """Run one bounded episode without assuming ROS, Gym or tensor types."""

    lifecycle = EpisodeLifecycle(config)
    observation = environment.reset(seed=config.seed)
    reset = getattr(policy, "reset_episode", None)
    if callable(reset):
        reset(seed=config.seed)
    reset = getattr(safety_filter, "reset_episode", None)
    if callable(reset):
        reset(seed=config.seed)
    lifecycle.start()

    last_transition: StepTransition[ObservationT, ActionT] | None = None
    for _ in range(config.max_steps):
        context = dict(config.context) | {"step_index": lifecycle.step_index}
        reference = policy.act(observation, context)
        state = state_adapter(observation)
        filtered = safety_filter.filter(reference, state, context)
        transition = environment.step(filtered.filtered_action)
        update_after_step = getattr(safety_filter, "update_after_step", None)
        if callable(update_after_step):
            executed_action = (
                transition.executed_action
                if transition.executed_action is not None
                else filtered.filtered_action
            )
            update_after_step(state, executed_action, context)
        step_index = lifecycle.advance()
        if on_step is not None:
            on_step(step_index, observation, reference, filtered, transition)
        observation = transition.observation
        last_transition = transition
        if transition.terminated or transition.truncated:
            lifecycle.terminate(
                transition.reason
                or ("terminated" if transition.terminated else "truncated")
            )
            break

    if lifecycle.phase.value == "running":
        lifecycle.terminate("max_steps")

    metadata = last_transition.metadata if last_transition is not None else {}
    return EpisodeOutcome(
        scenario_id=config.scenario_id,
        seed=config.seed,
        steps=lifecycle.step_index,
        termination_reason=lifecycle.termination_reason,
        success=bool(metadata.get("success", False)),
        metadata=metadata,
    )
