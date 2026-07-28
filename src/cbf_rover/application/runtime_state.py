"""Runtime observation records used while executing an episode."""

from __future__ import annotations

from dataclasses import dataclass, field
from math import isfinite
from typing import Any, Mapping


@dataclass(frozen=True)
class Observation:
    """Opaque sensor payload plus auditable timing and metadata."""

    payload: Any
    timestamp_s: float | None = None
    frame_id: str = ""
    metadata: Mapping[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        if self.timestamp_s is not None and not isfinite(self.timestamp_s):
            raise ValueError("Observation.timestamp_s must be finite")


@dataclass(frozen=True)
class RuntimeState:
    """Method-neutral execution state owned by application services."""

    observation: Observation
    step_index: int
    elapsed_s: float
    metadata: Mapping[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        if self.step_index < 0:
            raise ValueError("RuntimeState.step_index must be non-negative")
        if self.elapsed_s < 0.0 or not isfinite(self.elapsed_s):
            raise ValueError("RuntimeState.elapsed_s must be finite and non-negative")
