"""Explicit evidence labels for obstacle-certificate claims."""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Mapping


class ClaimKind(str, Enum):
    """Strength of the evidence supporting a safety-related result."""

    FORMAL = "formal"
    BOUNDED_MODEL_ERROR = "bounded_model_error"
    EMPIRICAL = "empirical"
    DIAGNOSTIC = "diagnostic"


@dataclass(frozen=True)
class SafetyClaim:
    """Human- and machine-readable statement attached to method outputs."""

    kind: ClaimKind
    statement: str
    assumptions: tuple[str, ...] = ()
    evidence: Mapping[str, str] = field(default_factory=dict)

    def __post_init__(self) -> None:
        if not isinstance(self.kind, ClaimKind):
            raise TypeError("SafetyClaim.kind must be a ClaimKind")
        if not isinstance(self.statement, str) or not self.statement.strip():
            raise ValueError("SafetyClaim.statement must not be empty")
