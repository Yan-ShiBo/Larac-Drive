"""Learned CBF/CLF-like scalar networks for RGB-D latents."""

from __future__ import annotations

import torch
import torch.nn as nn


def _hidden_layers(
    input_dim: int,
    hidden_dims: tuple[int, int, int],
    dropout: float,
) -> list[nn.Module]:
    first, second, third = hidden_dims
    return [
        nn.Linear(input_dim, first),
        nn.ReLU(inplace=True),
        nn.Dropout(dropout),
        nn.Linear(first, second),
        nn.ReLU(inplace=True),
        nn.Dropout(dropout),
        nn.Linear(second, third),
        nn.ReLU(inplace=True),
        nn.Dropout(dropout),
    ]


class CBFNetwork(nn.Module):
    """Empirical CBF-like score; its sign convention is method-specific."""

    def __init__(
        self,
        input_dim: int = 64,
        hidden_dims: tuple[int, int, int] = (256, 128, 64),
        dropout: float = 0.2,
    ) -> None:
        super().__init__()
        self.net = nn.Sequential(
            *_hidden_layers(input_dim, hidden_dims, dropout),
            nn.Linear(hidden_dims[-1], 1),
        )

    def forward(self, latent: torch.Tensor) -> torch.Tensor:
        return self.net(latent)


class CLFNetwork(nn.Module):
    """Empirical CLF-like score scaled to the inherited target range."""

    def __init__(
        self,
        input_dim: int = 64,
        hidden_dims: tuple[int, int, int] = (256, 128, 64),
        dropout: float = 0.2,
        output_scale: float = 99.0,
    ) -> None:
        super().__init__()
        self.output_scale = output_scale
        self.net = nn.Sequential(
            *_hidden_layers(input_dim, hidden_dims, dropout),
            nn.Linear(hidden_dims[-1], 1),
            nn.Sigmoid(),
        )

    def forward(self, latent: torch.Tensor) -> torch.Tensor:
        return self.net(latent) * self.output_scale
