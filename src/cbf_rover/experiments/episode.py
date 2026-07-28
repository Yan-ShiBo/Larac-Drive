"""Explicit episode lifecycle with guarded phase transitions."""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Mapping


class EpisodePhase(str, Enum):
    CREATED = "created"
    RUNNING = "running"
    TERMINATED = "terminated"


@dataclass(frozen=True)
class EpisodeConfig:
    scenario_id: str
    seed: int
    max_steps: int
    context: Mapping[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        if not self.scenario_id.strip():
            raise ValueError("scenario_id must not be empty")
        if self.seed < 0:
            raise ValueError("seed must be non-negative")
        if self.max_steps <= 0:
            raise ValueError("max_steps must be positive")


@dataclass(frozen=True)
class EpisodeOutcome:
    scenario_id: str
    seed: int
    steps: int
    termination_reason: str
    success: bool
    metadata: Mapping[str, Any] = field(default_factory=dict)


@dataclass
class EpisodeLifecycle:
    """Mutable phase tracker owned by one runner invocation."""

    config: EpisodeConfig
    phase: EpisodePhase = EpisodePhase.CREATED
    step_index: int = 0
    termination_reason: str = ""

    def start(self) -> None:
        if self.phase is not EpisodePhase.CREATED:
            raise RuntimeError(f"cannot start episode in phase {self.phase.value}")
        self.phase = EpisodePhase.RUNNING

    def advance(self) -> int:
        if self.phase is not EpisodePhase.RUNNING:
            raise RuntimeError("episode is not running")
        self.step_index += 1
        return self.step_index

    def terminate(self, reason: str) -> None:
        if self.phase is not EpisodePhase.RUNNING:
            raise RuntimeError("only a running episode can terminate")
        if not reason.strip():
            raise ValueError("termination reason must not be empty")
        self.termination_reason = reason
        self.phase = EpisodePhase.TERMINATED
