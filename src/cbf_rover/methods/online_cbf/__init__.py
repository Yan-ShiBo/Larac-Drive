"""Runtime control-barrier action filters.

These filters are separate from the synthesized polynomial q-SBC research
path. They evaluate state-dependent constraints and may modify actions.
"""

from .projection import (
    InfeasibleProjectionError,
    LinearHalfspace2D,
    ProjectionResult,
    project_action_to_halfspaces_and_disk,
)
from .road_filter import (
    BarrierConstraintFact,
    DistanceCBFSpec,
    RoadFilterResult,
    filter_dynamic_road_action,
)

__all__ = [
    "BarrierConstraintFact",
    "DistanceCBFSpec",
    "InfeasibleProjectionError",
    "LinearHalfspace2D",
    "ProjectionResult",
    "RoadFilterResult",
    "filter_dynamic_road_action",
    "project_action_to_halfspaces_and_disk",
]
