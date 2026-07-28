"""Transport-independent action, observation and rover-state records."""

from __future__ import annotations

from dataclasses import dataclass, field
from math import isfinite, sqrt
from typing import Any, Mapping, Sequence


def _finite_tuple(values: Sequence[float], label: str) -> tuple[float, ...]:
    result = tuple(float(value) for value in values)
    if not result:
        raise ValueError(f"{label} must contain at least one value")
    if not all(isfinite(value) for value in result):
        raise ValueError(f"{label} must contain only finite values")
    return result


@dataclass(frozen=True)
class Action:
    """Named, finite action vector independent from NumPy and ROS messages."""

    values: tuple[float, ...]
    labels: tuple[str, ...] = ()

    def __post_init__(self) -> None:
        values = _finite_tuple(self.values, "Action.values")
        object.__setattr__(self, "values", values)
        if self.labels and len(self.labels) != len(values):
            raise ValueError("Action.labels must match Action.values")

    @classmethod
    def from_sequence(
        cls,
        values: Sequence[float],
        *,
        labels: Sequence[str] = (),
    ) -> "Action":
        return cls(tuple(values), tuple(labels))

    def distance_to(self, other: "Action") -> float:
        if len(self.values) != len(other.values):
            raise ValueError("Actions must have equal dimensions")
        return sqrt(sum((left - right) ** 2 for left, right in zip(self.values, other.values)))


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
class RoverState:
    """Method-neutral state record used by experiment runners."""

    observation: Observation
    step_index: int
    elapsed_s: float
    metadata: Mapping[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        if self.step_index < 0:
            raise ValueError("RoverState.step_index must be non-negative")
        if self.elapsed_s < 0.0 or not isfinite(self.elapsed_s):
            raise ValueError("RoverState.elapsed_s must be finite and non-negative")
