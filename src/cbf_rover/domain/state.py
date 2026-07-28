"""Transport-independent mathematical state vectors."""

from __future__ import annotations

from dataclasses import dataclass
from math import isfinite
from typing import Sequence


def _finite_tuple(values: Sequence[float], label: str) -> tuple[float, ...]:
    result = tuple(float(value) for value in values)
    if not result:
        raise ValueError(f"{label} must contain at least one value")
    if not all(isfinite(value) for value in result):
        raise ValueError(f"{label} must contain only finite values")
    return result


@dataclass(frozen=True)
class StateVector:
    """Finite state coordinates with explicit semantics and reference frame."""

    values: tuple[float, ...]
    labels: tuple[str, ...]
    units: tuple[str, ...]
    frame_id: str

    def __post_init__(self) -> None:
        values = _finite_tuple(self.values, "StateVector.values")
        labels = tuple(self.labels)
        units = tuple(self.units)
        object.__setattr__(self, "values", values)
        object.__setattr__(self, "labels", labels)
        object.__setattr__(self, "units", units)
        if len(labels) != len(values):
            raise ValueError("StateVector.labels must match StateVector.values")
        if len(units) != len(values):
            raise ValueError("StateVector.units must match StateVector.values")
        if any(not isinstance(label, str) or not label.strip() for label in labels):
            raise ValueError("StateVector.labels must not contain empty values")
        if len(set(labels)) != len(labels):
            raise ValueError("StateVector.labels must be unique")
        if any(not isinstance(unit, str) or not unit.strip() for unit in units):
            raise ValueError("StateVector.units must not contain empty values")
        if not isinstance(self.frame_id, str) or not self.frame_id.strip():
            raise ValueError("StateVector.frame_id must not be empty")

    @classmethod
    def from_sequence(
        cls,
        values: Sequence[float],
        *,
        labels: Sequence[str],
        units: Sequence[str],
        frame_id: str,
    ) -> "StateVector":
        return cls(tuple(values), tuple(labels), tuple(units), frame_id)
