"""Server-side training for the empirical CBF-like score."""

from __future__ import annotations

from dataclasses import dataclass
from functools import partial
from pathlib import Path

import torch
import torch.nn.functional as functional

from ..certificates import CBFNetwork
from ._scalar import train_scalar_model
from .common import TrainingSummary


@dataclass(frozen=True)
class CBFTrainingConfig:
    dataset_root: str | Path
    output_dir: str | Path
    epochs: int = 1000
    batch_size: int = 64
    latent_dim: int = 64
    learning_rate: float = 1.0e-4
    test_ratio: float = 0.1
    workers: int = 2
    flag_weight: float = 10.0
    false_positive_threshold: float = -25.0
    false_positive_weight: float = 10.0
    seed: int = 42
    deterministic: bool = False
    device: str = "auto"

    def __post_init__(self) -> None:
        if min(self.epochs, self.batch_size, self.latent_dim) <= 0:
            raise ValueError("epochs, batch_size and latent_dim must be positive")
        if not 0.0 < self.test_ratio < 1.0:
            raise ValueError("test_ratio must be strictly between zero and one")
        if self.flag_weight < 0.0 or self.false_positive_weight < 0.0:
            raise ValueError("CBF loss weights must be non-negative")


def weighted_cbf_loss(
    prediction: torch.Tensor,
    target: torch.Tensor,
    flag: torch.Tensor,
    *,
    flag_weight: float,
    false_positive_threshold: float,
    false_positive_weight: float,
) -> torch.Tensor:
    """Preserve inherited flag and false-positive weighting."""

    target = target.view_as(prediction)
    flag = flag.view_as(prediction)
    base = functional.mse_loss(prediction, target, reduction="none")
    sample_weight = 1.0 + flag_weight * flag
    false_positive = (
        (target < false_positive_threshold)
        & (prediction > false_positive_threshold)
    ).float()
    penalty = (
        false_positive_weight
        * (prediction - target).pow(2)
        * false_positive
    )
    return (base * sample_weight + penalty).mean()


def train_cbf(config: CBFTrainingConfig) -> TrainingSummary:
    return train_scalar_model(
        config=config,
        model=CBFNetwork(input_dim=config.latent_dim),
        model_name="learned_rgbd_cbf",
        target_key="cbf",
        include_flag=True,
        loss_function=partial(
            weighted_cbf_loss,
            flag_weight=config.flag_weight,
            false_positive_threshold=config.false_positive_threshold,
            false_positive_weight=config.false_positive_weight,
        ),
    )
