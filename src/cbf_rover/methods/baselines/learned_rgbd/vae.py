"""Dual-branch RGB-D variational autoencoder."""

from __future__ import annotations

from dataclasses import dataclass

import torch
import torch.nn as nn
import torch.nn.functional as functional


@dataclass(frozen=True)
class RGBDVAEConfig:
    rgb_channels: int = 3
    depth_channels: int = 1
    latent_dim: int = 64
    branch_feature_dim: int = 128
    image_size: int = 128

    def __post_init__(self) -> None:
        if min(
            self.rgb_channels,
            self.depth_channels,
            self.latent_dim,
            self.branch_feature_dim,
        ) <= 0:
            raise ValueError("VAE channel and feature dimensions must be positive")
        if self.image_size != 128:
            raise ValueError("the inherited RGB-D VAE supports only image_size=128")


class EncoderBranch(nn.Module):
    def __init__(self, channels: int, feature_dim: int) -> None:
        super().__init__()
        self.net = nn.Sequential(
            nn.Conv2d(channels, 32, kernel_size=4, stride=2, padding=1),
            nn.ReLU(inplace=True),
            nn.Conv2d(32, 64, kernel_size=4, stride=2, padding=1),
            nn.ReLU(inplace=True),
            nn.Conv2d(64, 128, kernel_size=4, stride=2, padding=1),
            nn.ReLU(inplace=True),
            nn.Conv2d(128, feature_dim, kernel_size=4, stride=2, padding=1),
            nn.ReLU(inplace=True),
            nn.AdaptiveAvgPool2d((1, 1)),
        )

    def forward(self, value: torch.Tensor) -> torch.Tensor:
        return self.net(value).flatten(start_dim=1)


class RGBDDecoder(nn.Module):
    def __init__(self, config: RGBDVAEConfig) -> None:
        super().__init__()
        self.fc = nn.Linear(config.latent_dim, 256 * 4 * 4)
        self.shared = nn.Sequential(
            nn.ConvTranspose2d(256, 128, kernel_size=4, stride=2, padding=1),
            nn.ReLU(inplace=True),
            nn.ConvTranspose2d(128, 64, kernel_size=4, stride=2, padding=1),
            nn.ReLU(inplace=True),
            nn.ConvTranspose2d(64, 32, kernel_size=4, stride=2, padding=1),
            nn.ReLU(inplace=True),
            nn.ConvTranspose2d(32, 16, kernel_size=4, stride=2, padding=1),
            nn.ReLU(inplace=True),
        )
        # Attribute names intentionally match inherited checkpoints.
        self.rgb_head = nn.ConvTranspose2d(16, config.rgb_channels, 4, 2, 1)
        self.depth_head = nn.ConvTranspose2d(16, config.depth_channels, 4, 2, 1)

    def forward(self, latent: torch.Tensor) -> tuple[torch.Tensor, torch.Tensor]:
        features = functional.relu(self.fc(latent), inplace=True).view(-1, 256, 4, 4)
        features = self.shared(features)
        return (
            torch.sigmoid(self.rgb_head(features)),
            torch.sigmoid(self.depth_head(features)),
        )


class RGBDVAE(nn.Module):
    def __init__(self, config: RGBDVAEConfig = RGBDVAEConfig()) -> None:
        super().__init__()
        self.config = config
        self.rgb_branch = EncoderBranch(config.rgb_channels, config.branch_feature_dim)
        self.depth_branch = EncoderBranch(config.depth_channels, config.branch_feature_dim)
        self.fusion = nn.Sequential(
            nn.Linear(config.branch_feature_dim * 2, 256),
            nn.ReLU(inplace=True),
        )
        # Attribute names intentionally match inherited checkpoints.
        self.fc_mu = nn.Linear(256, config.latent_dim)
        self.fc_logsigma = nn.Linear(256, config.latent_dim)
        self.decoder = RGBDDecoder(config)
        self.apply(self._initialize)

    @staticmethod
    def _initialize(module: nn.Module) -> None:
        if isinstance(module, (nn.Conv2d, nn.ConvTranspose2d, nn.Linear)):
            nn.init.xavier_uniform_(module.weight)
            if module.bias is not None:
                nn.init.constant_(module.bias, 0.0)

    def encode(self, rgb: torch.Tensor, depth: torch.Tensor) -> tuple[torch.Tensor, torch.Tensor]:
        fused = self.fusion(
            torch.cat([self.rgb_branch(rgb), self.depth_branch(depth)], dim=1)
        )
        return self.fc_mu(fused), self.fc_logsigma(fused)

    @staticmethod
    def reparameterize(mu: torch.Tensor, log_sigma: torch.Tensor) -> torch.Tensor:
        sigma = log_sigma.exp()
        return torch.randn_like(sigma).mul(sigma).add_(mu)

    def forward(
        self,
        rgb: torch.Tensor,
        depth: torch.Tensor,
    ) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor, torch.Tensor, torch.Tensor]:
        mu, log_sigma = self.encode(rgb, depth)
        latent = self.reparameterize(mu, log_sigma)
        rgb_reconstruction, depth_reconstruction = self.decoder(latent)
        return rgb_reconstruction, depth_reconstruction, mu, log_sigma, latent

    @torch.no_grad()
    def encode_to_latent(
        self,
        rgb: torch.Tensor,
        depth: torch.Tensor,
        *,
        use_mu: bool = True,
    ) -> torch.Tensor:
        self.eval()
        mu, log_sigma = self.encode(rgb, depth)
        return mu if use_mu else self.reparameterize(mu, log_sigma)


def rgbd_vae_loss(
    rgb_reconstruction: torch.Tensor,
    depth_reconstruction: torch.Tensor,
    rgb: torch.Tensor,
    depth: torch.Tensor,
    mu: torch.Tensor,
    log_sigma: torch.Tensor,
    depth_weight: float = 1.0,
    kl_weight: float = 1.0,
) -> torch.Tensor:
    rgb_loss = functional.mse_loss(rgb_reconstruction, rgb, reduction="sum")
    depth_loss = functional.mse_loss(depth_reconstruction, depth, reduction="sum")
    divergence = -0.5 * torch.sum(
        1 + 2 * log_sigma - mu.pow(2) - (2 * log_sigma).exp()
    )
    return rgb_loss + depth_weight * depth_loss + kl_weight * divergence
