"""Obstacle-certificate research package.

Dependency-light types live in :mod:`cbf_rover.domain`; use cases live in
:mod:`cbf_rover.application`; infrastructure integrations live in
:mod:`cbf_rover.adapters`; concrete research methods live in
:mod:`cbf_rover.methods`.
"""

from .domain import (
    ActionModificationResult,
    ActionModifier,
    CertificateEvaluation,
    ClaimKind,
    Controller,
    SafetyClaim,
)

__all__ = [
    "ActionModificationResult",
    "ActionModifier",
    "CertificateEvaluation",
    "ClaimKind",
    "Controller",
    "SafetyClaim",
]

__version__ = "0.2.0"
