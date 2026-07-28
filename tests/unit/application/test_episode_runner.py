from __future__ import annotations

from collections.abc import Mapping
from typing import Any

import pytest

from cbf_rover.application import (
    EpisodeConfig,
    InfeasibleActionError,
    StepTransition,
    run_episode,
)
from cbf_rover.domain import ActionModificationResult


class RecordingController:
    name = "recording_controller"

    def __init__(self) -> None:
        self.reset_seeds: list[int | None] = []
        self.calls: list[tuple[int, dict[str, Any]]] = []

    def reset_episode(self, *, seed: int | None = None) -> None:
        self.reset_seeds.append(seed)

    def act(
        self,
        observation: int,
        context: Mapping[str, Any] | None = None,
    ) -> int:
        self.calls.append((observation, dict(context or {})))
        return observation * 10


class RecordingModifier:
    name = "recording_modifier"

    def __init__(self, *, feasible: bool) -> None:
        self.feasible = feasible
        self.reset_seeds: list[int | None] = []
        self.calls: list[tuple[int, str, dict[str, Any]]] = []
        self.results: list[ActionModificationResult[int]] = []
        self.executed_updates: list[tuple[str, int, dict[str, Any]]] = []

    def reset_episode(self, *, seed: int | None = None) -> None:
        self.reset_seeds.append(seed)

    def modify(
        self,
        reference_action: int,
        state: str,
        context: Mapping[str, Any] | None = None,
    ) -> ActionModificationResult[int]:
        recorded_context = dict(context or {})
        self.calls.append((reference_action, state, recorded_context))
        result = ActionModificationResult(
            reference_action=reference_action,
            modified_action=reference_action + 1,
            intervened=True,
            intervention_norm=1.0,
            feasible=self.feasible,
        )
        self.results.append(result)
        return result

    def update_after_step(
        self,
        state: str,
        executed_action: int,
        context: Mapping[str, Any] | None = None,
    ) -> None:
        self.executed_updates.append((state, executed_action, dict(context or {})))


class SequenceEnvironment:
    def __init__(self, transitions: list[StepTransition[int, int]]) -> None:
        self.transitions = transitions
        self.reset_seeds: list[int] = []
        self.step_actions: list[int] = []

    def reset(self, *, seed: int) -> int:
        self.reset_seeds.append(seed)
        return 1

    def step(self, action: int) -> StepTransition[int, int]:
        self.step_actions.append(action)
        return self.transitions[len(self.step_actions) - 1]


def _episode_config(*, seed: int = 11) -> EpisodeConfig:
    return EpisodeConfig(
        scenario_id="static_obstacles_v1",
        seed=seed,
        max_steps=3,
        context={"scenario_family": "obstacle_avoidance"},
    )


def test_runner_executes_nominal_action_unchanged_without_modifier() -> None:
    environment = SequenceEnvironment(
        [
            StepTransition(
                observation=2,
                executed_action=10,
                terminated=True,
                reason="goal_reached",
                metadata={"success": True},
            )
        ]
    )
    controller = RecordingController()
    callback_steps: list[int] = []
    callback_modifications: list[ActionModificationResult[int] | None] = []

    outcome = run_episode(
        environment=environment,
        controller=controller,
        config=_episode_config(),
        on_step=lambda step, _observation, _reference, modification, _transition: (
            callback_steps.append(step)
            or callback_modifications.append(modification)
        ),
    )

    assert controller.calls[0][0] == 1
    assert environment.step_actions == [10]
    assert callback_steps == [0]
    assert callback_modifications == [None]
    assert outcome.termination_reason == "goal_reached"
    assert outcome.success is True


def test_runner_executes_feasible_modified_action_and_reports_modification() -> None:
    environment = SequenceEnvironment(
        [
            StepTransition(
                observation=2,
                executed_action=9,
                terminated=True,
                reason="terminated",
            )
        ]
    )
    controller = RecordingController()
    modifier = RecordingModifier(feasible=True)
    callback_modifications: list[ActionModificationResult[int] | None] = []

    outcome = run_episode(
        environment=environment,
        controller=controller,
        config=_episode_config(seed=7),
        modifier=modifier,
        state_adapter=lambda observation: f"state-{observation}",
        on_step=lambda _step, _observation, _reference, modification, _transition: (
            callback_modifications.append(modification)
        ),
    )

    assert environment.step_actions == [11]
    assert modifier.reset_seeds == [7]
    assert modifier.calls[0][:2] == (10, "state-1")
    assert modifier.executed_updates[0][:2] == ("state-1", 9)
    assert callback_modifications == modifier.results
    assert outcome.steps == 1


def test_infeasible_modifier_fails_before_environment_step() -> None:
    environment = SequenceEnvironment([])
    modifier = RecordingModifier(feasible=False)

    with pytest.raises(InfeasibleActionError, match="environment.step") as caught:
        run_episode(
            environment=environment,
            controller=RecordingController(),
            config=_episode_config(),
            modifier=modifier,
            state_adapter=lambda observation: f"state-{observation}",
        )

    assert environment.step_actions == []
    assert caught.value.step_index == 0
    assert caught.value.modifier_name == modifier.name
    assert caught.value.result is modifier.results[0]
    assert caught.value.result.feasible is False


def test_modifier_requires_state_adapter_before_episode_reset() -> None:
    environment = SequenceEnvironment([])
    modifier = RecordingModifier(feasible=True)

    with pytest.raises(ValueError, match="state_adapter"):
        run_episode(
            environment=environment,
            controller=RecordingController(),
            config=_episode_config(),
            modifier=modifier,
        )

    assert environment.reset_seeds == []
    assert environment.step_actions == []
    assert modifier.reset_seeds == []
