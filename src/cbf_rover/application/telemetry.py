"""Application-level telemetry records independent from storage backends."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Mapping


@dataclass(frozen=True)
class StepRecord:
    """Serializable facts captured for one executed control step."""

    episode_id: str
    step_index: int
    reference_action: Any
    commanded_action: Any
    executed_action: Any | None
    reward: float
    cost: float
    metadata: Mapping[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        if not self.episode_id.strip():
            raise ValueError("episode_id must not be empty")
        if self.step_index < 0:
            raise ValueError("step_index must be non-negative")
