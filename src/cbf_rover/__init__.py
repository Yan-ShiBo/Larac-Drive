"""Simulation-first obstacle-certificate research package.

The stable, dependency-light contracts live in :mod:`cbf_rover.core`.
Concrete algorithms are grouped vertically under :mod:`cbf_rover.methods`;
ROS and simulator adapters deliberately live outside this Python package.
"""

from .core import (
    CertificateEvaluation,
    ClaimKind,
    FilterResult,
    SafetyClaim,
    SafetyFilter,
)

__all__ = [
    "CertificateEvaluation",
    "ClaimKind",
    "FilterResult",
    "SafetyClaim",
    "SafetyFilter",
]

__version__ = "0.2.0"
