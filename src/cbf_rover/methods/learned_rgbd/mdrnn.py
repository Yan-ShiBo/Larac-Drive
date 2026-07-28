"""Mixture-density recurrent dynamics model for RGB-D latent states."""

from __future__ import annotations

import torch
import torch.nn as nn
import torch.nn.functional as functional
from torch.distributions import Normal


class RecurrentCore(nn.Module):
    def __init__(
        self,
        latent_dim: int,
        action_dim: int,
        hidden_dim: int,
        num_layers: int = 1,
    ) -> None:
        super().__init__()
        self.hidden_dim = hidden_dim
        self.num_layers = num_layers
        self.lstm = nn.LSTM(
            input_size=latent_dim + action_dim,
            hidden_size=hidden_dim,
            num_layers=num_layers,
            batch_first=True,
        )

    def init_hidden(
        self,
        batch_size: int,
        device: torch.device | str,
    ) -> tuple[torch.Tensor, torch.Tensor]:
        hidden = torch.zeros(self.num_layers, batch_size, self.hidden_dim, device=device)
        cell = torch.zeros(self.num_layers, batch_size, self.hidden_dim, device=device)
        return hidden, cell

    def forward(
        self,
        latent: torch.Tensor,
        action: torch.Tensor,
        hidden: tuple[torch.Tensor, torch.Tensor] | None = None,
    ) -> tuple[torch.Tensor, tuple[torch.Tensor, torch.Tensor]]:
        output, next_hidden = self.lstm(torch.cat([latent, action], dim=-1), hidden)
        return output, next_hidden


class MixtureDensityHead(nn.Module):
    def __init__(self, latent_dim: int, hidden_dim: int, components: int) -> None:
        super().__init__()
        self.latent_dim = latent_dim
        self.components = components
        # Attribute names intentionally match inherited checkpoints.
        self.fc_pi = nn.Linear(hidden_dim, components)
        self.fc_mu = nn.Linear(hidden_dim, components * latent_dim)
        self.fc_sigma = nn.Linear(hidden_dim, components * latent_dim)

    def forward(
        self,
        hidden_output: torch.Tensor,
        temperature: float = 1.0,
    ) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
        if temperature <= 0.0:
            raise ValueError("temperature must be positive")
        batch_size, sequence_length, _ = hidden_output.shape
        weights = functional.softmax(self.fc_pi(hidden_output) / temperature, dim=-1)
        means = self.fc_mu(hidden_output).view(
            batch_size,
            sequence_length,
            self.components,
            self.latent_dim,
        )
        scales = self.fc_sigma(hidden_output).exp().view(
            batch_size,
            sequence_length,
            self.components,
            self.latent_dim,
        )
        return weights, means, scales.clamp_min(1.0e-5)


class MDNRNN(nn.Module):
    """Inherited MDN-RNN architecture under the vertical method package."""

    def __init__(
        self,
        latent_dim: int = 64,
        action_dim: int = 2,
        hidden_dim: int = 256,
        num_gaussians: int = 5,
        num_layers: int = 1,
    ) -> None:
        super().__init__()
        self.rnn = RecurrentCore(latent_dim, action_dim, hidden_dim, num_layers)
        self.mdn = MixtureDensityHead(latent_dim, hidden_dim, num_gaussians)

    def forward(
        self,
        latent: torch.Tensor,
        action: torch.Tensor,
        hidden: tuple[torch.Tensor, torch.Tensor] | None = None,
        temperature: float = 1.0,
    ) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor, tuple[torch.Tensor, torch.Tensor]]:
        output, next_hidden = self.rnn(latent, action, hidden)
        weights, means, scales = self.mdn(output, temperature)
        return weights, means, scales, next_hidden


def gaussian_nll_loss(
    weights: torch.Tensor,
    means: torch.Tensor,
    scales: torch.Tensor,
    target: torch.Tensor,
) -> torch.Tensor:
    target = target.unsqueeze(2)
    log_probability = Normal(means, scales).log_prob(target).sum(dim=-1)
    weighted = torch.log(weights.clamp_min(1.0e-8)) + log_probability
    return -torch.logsumexp(weighted, dim=-1).mean()


def expected_latent(weights: torch.Tensor, means: torch.Tensor) -> torch.Tensor:
    return (weights.unsqueeze(-1) * means).sum(dim=2)
