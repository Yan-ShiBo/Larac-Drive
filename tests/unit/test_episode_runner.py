from __future__ import annotations

from collections.abc import Mapping
from typing import Any

import pytest

from cbf_rover.core import FilterResult
from cbf_rover.experiments import (
    EpisodeConfig,
    EpisodeLifecycle,
    EpisodePhase,
    StepTransition,
    run_episode,
)


class RecordingPolicy:
    name = "recording_policy"

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
        recorded_context = dict(context or {})
        self.calls.append((observation, recorded_context))
        return observation * 10


class RecordingSafetyFilter:
    name = "recording_filter"

    def __init__(self) -> None:
        self.reset_seeds: list[int | None] = []
        self.filter_calls: list[tuple[int, str, dict[str, Any]]] = []
        self.executed_updates: list[tuple[str, int, dict[str, Any]]] = []

    def reset_episode(self, *, seed: int | None = None) -> None:
        self.reset_seeds.append(seed)

    def filter(
        self,
        reference_action: int,
        state: str,
        context: Mapping[str, Any] | None = None,
    ) -> FilterResult[int]:
        recorded_context = dict(context or {})
        self.filter_calls.append((reference_action, state, recorded_context))
        return FilterResult(
            reference_action=reference_action,
            filtered_action=reference_action + 1,
            intervened=True,
            intervention_norm=1.0,
            feasible=True,
        )

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
        self.actions: list[int] = []

    def reset(self, *, seed: int) -> int:
        self.reset_seeds.append(seed)
        return 1

    def step(self, action: int) -> StepTransition[int, int]:
        self.actions.append(action)
        return self.transitions[len(self.actions) - 1]


def test_episode_lifecycle_allows_only_created_running_terminated_order() -> None:
    lifecycle = EpisodeLifecycle(
        EpisodeConfig(scenario_id="static_obstacles", seed=3, max_steps=2)
    )

    assert lifecycle.phase is EpisodePhase.CREATED
    with pytest.raises(RuntimeError, match="not running"):
        lifecycle.advance()

    lifecycle.start()
    assert lifecycle.advance() == 1
    lifecycle.terminate("goal_reached")

    assert lifecycle.phase is EpisodePhase.TERMINATED
    assert lifecycle.termination_reason == "goal_reached"
    with pytest.raises(RuntimeError, match="cannot start"):
        lifecycle.start()


def test_runner_resets_components_filters_actions_and_reports_termination() -> None:
    environment = SequenceEnvironment(
        [
            StepTransition(observation=2, executed_action=101),
            StepTransition(
                observation=3,
                executed_action=None,
                terminated=True,
                reason="goal_reached",
                metadata={"success": True},
            ),
        ]
    )
    policy = RecordingPolicy()
    safety_filter = RecordingSafetyFilter()
    callback_steps: list[int] = []

    outcome = run_episode(
        environment=environment,
        policy=policy,
        safety_filter=safety_filter,
        state_adapter=lambda observation: f"state-{observation}",
        config=EpisodeConfig(
            scenario_id="static_obstacles",
            seed=11,
            max_steps=5,
            context={"scenario_family": "obstacle_avoidance"},
        ),
        on_step=lambda step_index, *_: callback_steps.append(step_index),
    )

    assert environment.reset_seeds == [11]
    assert policy.reset_seeds == [11]
    assert safety_filter.reset_seeds == [11]
    assert environment.actions == [11, 21]
    assert callback_steps == [1, 2]
    assert [call[2]["step_index"] for call in safety_filter.filter_calls] == [0, 1]
    assert [update[1] for update in safety_filter.executed_updates] == [101, 21]
    assert outcome.steps == 2
    assert outcome.termination_reason == "goal_reached"
    assert outcome.success is True


def test_runner_stops_at_configured_step_bound() -> None:
    environment = SequenceEnvironment(
        [StepTransition(observation=2, executed_action=None)]
    )

    outcome = run_episode(
        environment=environment,
        policy=RecordingPolicy(),
        safety_filter=RecordingSafetyFilter(),
        state_adapter=lambda observation: f"state-{observation}",
        config=EpisodeConfig(
            scenario_id="static_obstacles",
            seed=0,
            max_steps=1,
        ),
    )

    assert outcome.steps == 1
    assert outcome.termination_reason == "max_steps"
    assert outcome.success is False
