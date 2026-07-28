"""Transport-independent mathematical action vectors."""

from __future__ import annotations

from dataclasses import dataclass
from math import isfinite, sqrt
from typing import Sequence


def _finite_tuple(values: Sequence[float], label: str) -> tuple[float, ...]:
    result = tuple(float(value) for value in values)
    if not result:
        raise ValueError(f"{label} must contain at least one value")
    if not all(isfinite(value) for value in result):
        raise ValueError(f"{label} must contain only finite values")
    return result


@dataclass(frozen=True)
class ActionVector:
    """Finite action coordinates with explicit semantics and reference frame."""

    values: tuple[float, ...]
    labels: tuple[str, ...]
    units: tuple[str, ...]
    frame_id: str

    def __post_init__(self) -> None:
        values = _finite_tuple(self.values, "ActionVector.values")
        labels = tuple(self.labels)
        units = tuple(self.units)
        object.__setattr__(self, "values", values)
        object.__setattr__(self, "labels", labels)
        object.__setattr__(self, "units", units)
        if len(labels) != len(values):
            raise ValueError("ActionVector.labels must match ActionVector.values")
        if len(units) != len(values):
            raise ValueError("ActionVector.units must match ActionVector.values")
        if any(not isinstance(label, str) or not label.strip() for label in labels):
            raise ValueError("ActionVector.labels must not contain empty values")
        if len(set(labels)) != len(labels):
            raise ValueError("ActionVector.labels must be unique")
        if any(not isinstance(unit, str) or not unit.strip() for unit in units):
            raise ValueError("ActionVector.units must not contain empty values")
        if not isinstance(self.frame_id, str) or not self.frame_id.strip():
            raise ValueError("ActionVector.frame_id must not be empty")

    @classmethod
    def from_sequence(
        cls,
        values: Sequence[float],
        *,
        labels: Sequence[str],
        units: Sequence[str],
        frame_id: str,
    ) -> "ActionVector":
        return cls(tuple(values), tuple(labels), tuple(units), frame_id)

    def distance_to(self, other: "ActionVector") -> float:
        if len(self.values) != len(other.values):
            raise ValueError("Action vectors must have equal dimensions")
        if (
            self.labels != other.labels
            or self.units != other.units
            or self.frame_id != other.frame_id
        ):
            raise ValueError(
                "Action vectors must have identical labels, units, and frame_id"
            )
        return sqrt(
            sum(
                (left - right) ** 2
                for left, right in zip(self.values, other.values)
            )
        )
