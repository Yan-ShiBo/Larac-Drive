"""Auditable simulation metrics independent from a logging backend."""

from __future__ import annotations

from dataclasses import dataclass, field
from math import isfinite
from typing import Mapping


@dataclass
class InterventionMetrics:
    evaluated_steps: int = 0
    intervention_count: int = 0
    intervention_norm_sum: float = 0.0
    max_intervention_norm: float = 0.0
    infeasible_count: int = 0
    certificate_violation_count: int = 0

    @property
    def intervention_rate(self) -> float:
        return self.intervention_count / self.evaluated_steps if self.evaluated_steps else 0.0

    @property
    def mean_intervention_norm(self) -> float:
        return (
            self.intervention_norm_sum / self.intervention_count
            if self.intervention_count
            else 0.0
        )

    def record(
        self,
        intervention_norm: float,
        *,
        feasible: bool,
        certificate_violations: int = 0,
    ) -> None:
        if not isfinite(intervention_norm) or intervention_norm < 0.0:
            raise ValueError("intervention_norm must be finite and non-negative")
        if certificate_violations < 0:
            raise ValueError("certificate_violations must be non-negative")
        self.evaluated_steps += 1
        if intervention_norm > 0.0:
            self.intervention_count += 1
            self.intervention_norm_sum += intervention_norm
            self.max_intervention_norm = max(
                self.max_intervention_norm,
                intervention_norm,
            )
        if not feasible:
            self.infeasible_count += 1
        self.certificate_violation_count += certificate_violations


@dataclass(frozen=True)
class EpisodeMetrics:
    scenario_id: str
    seed: int
    steps: int
    success: bool
    collision_count: int
    total_reward: float
    total_cost: float
    min_obstacle_distance_m: float | None
    certificate_violation_count: int
    intervention_count: int
    mean_intervention_norm: float
    termination_reason: str
    extra: Mapping[str, float | int | str | bool] = field(default_factory=dict)
