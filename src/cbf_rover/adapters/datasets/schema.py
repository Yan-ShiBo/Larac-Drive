"""Canonical field names for RGB-D episode and certificate datasets."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class EpisodeFields:
    rgb_images: str = "rgb_images"
    depth_images: str = "depth_images"
    actions: str = "actions"
    rewards: str = "rewards"
    costs: str = "costs"
    dones: str = "dones"


@dataclass(frozen=True)
class CertificateFields:
    latent: str = "latent"
    cbf: str = "cbf"
    clf: str = "clf"
    flag: str = "flag"


@dataclass(frozen=True)
class RawCertificateFrameFields:
    rgb: str = "rgb"
    depth: str = "depth"
    cbf: str = "cbf"
    clf: str = "clf"
    flag: str = "flag"


EPISODE_FIELDS = EpisodeFields()
CERTIFICATE_FIELDS = CertificateFields()
RAW_CERTIFICATE_FRAME_FIELDS = RawCertificateFrameFields()
