"""Application service for bounded controller episodes."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Callable, Generic, Mapping, Protocol, TypeVar, runtime_checkable

from cbf_rover.domain import (
    ActionModificationResult,
    ActionModifier,
    Controller,
)

from .episode import EpisodeConfig, EpisodeLifecycle, EpisodeOutcome, EpisodePhase


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
        ActionModificationResult[ActionT] | None,
        StepTransition[ObservationT, ActionT],
    ],
    None,
]


class InfeasibleActionError(RuntimeError):
    """Raised before execution when an action modifier reports infeasibility."""

    def __init__(
        self,
        *,
        step_index: int,
        modifier_name: str,
        result: ActionModificationResult[Any],
    ) -> None:
        self.step_index = step_index
        self.modifier_name = modifier_name
        self.result = result
        super().__init__(
            f"action modifier {modifier_name!r} reported infeasibility "
            f"at step {step_index}; environment.step was not called"
        )


def run_episode(
    *,
    environment: Environment[ObservationT, ActionT],
    controller: Controller[ObservationT, ActionT],
    config: EpisodeConfig,
    modifier: ActionModifier[StateT, ActionT] | None = None,
    state_adapter: Callable[[ObservationT], StateT] | None = None,
    on_step: StepCallback[ObservationT, ActionT] | None = None,
) -> EpisodeOutcome:
    """Run one bounded episode, with or without an action modifier.

    Omitting ``modifier`` selects the unmodified nominal-action path: the
    controller action is sent directly to the environment. This is not a
    certificate monitor because no evaluator is connected yet. When a modifier
    is supplied, ``state_adapter`` is required. An infeasible modification
    fails closed before the environment can execute any action.
    """

    if modifier is not None and state_adapter is None:
        raise ValueError("state_adapter is required when modifier is configured")

    lifecycle = EpisodeLifecycle(config)
    reset = getattr(controller, "reset_episode", None)
    if callable(reset):
        reset(seed=config.seed)
    if modifier is not None:
        reset = getattr(modifier, "reset_episode", None)
        if callable(reset):
            reset(seed=config.seed)
    observation = environment.reset(seed=config.seed)
    lifecycle.start()

    last_transition: StepTransition[ObservationT, ActionT] | None = None
    for _ in range(config.max_steps):
        step_index = lifecycle.step_index
        context = dict(config.context) | {"step_index": step_index}
        reference = controller.act(observation, context)
        state: StateT | None = None
        modification: ActionModificationResult[ActionT] | None = None
        command = reference
        if modifier is not None:
            if state_adapter is None:  # Narrowed above; kept explicit for readers.
                raise RuntimeError("state_adapter unexpectedly missing")
            state = state_adapter(observation)
            modification = modifier.modify(reference, state, context)
            if not modification.feasible:
                raise InfeasibleActionError(
                    step_index=step_index,
                    modifier_name=modifier.name,
                    result=modification,
                )
            command = modification.modified_action

        transition = environment.step(command)
        update_after_step = (
            getattr(modifier, "update_after_step", None)
            if modifier is not None
            else None
        )
        if callable(update_after_step) and state is not None:
            executed_action = (
                transition.executed_action
                if transition.executed_action is not None
                else command
            )
            update_after_step(state, executed_action, context)
        lifecycle.advance()
        if on_step is not None:
            on_step(step_index, observation, reference, modification, transition)
        observation = transition.observation
        last_transition = transition
        if transition.terminated or transition.truncated:
            lifecycle.terminate(
                transition.reason
                or ("terminated" if transition.terminated else "truncated")
            )
            break

    if lifecycle.phase is EpisodePhase.RUNNING:
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
