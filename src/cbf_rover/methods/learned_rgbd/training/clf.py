"""Server-side training for the empirical CLF-like score."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import torch
import torch.nn.functional as functional

from ..certificates import CLFNetwork
from ._scalar import train_scalar_model
from .common import TrainingSummary


@dataclass(frozen=True)
class CLFTrainingConfig:
    dataset_root: str | Path
    output_dir: str | Path
    epochs: int = 1000
    batch_size: int = 128
    latent_dim: int = 64
    learning_rate: float = 1.0e-4
    test_ratio: float = 0.1
    workers: int = 2
    seed: int = 42
    deterministic: bool = False
    device: str = "auto"

    def __post_init__(self) -> None:
        if min(self.epochs, self.batch_size, self.latent_dim) <= 0:
            raise ValueError("epochs, batch_size and latent_dim must be positive")
        if not 0.0 < self.test_ratio < 1.0:
            raise ValueError("test_ratio must be strictly between zero and one")


def _clf_loss(prediction: torch.Tensor, target: torch.Tensor) -> torch.Tensor:
    return functional.mse_loss(prediction, target.view_as(prediction))


def train_clf(config: CLFTrainingConfig) -> TrainingSummary:
    return train_scalar_model(
        config=config,
        model=CLFNetwork(input_dim=config.latent_dim),
        model_name="learned_rgbd_clf",
        target_key="clf",
        include_flag=False,
        loss_function=_clf_loss,
    )
