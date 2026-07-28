"""Server-side adapter for RGB-D VAE training."""

from __future__ import annotations

from dataclasses import asdict, dataclass
from pathlib import Path

import torch
from torch.utils.data import DataLoader

from cbf_rover.adapters.datasets.readers import RGBDFrameDataset

from cbf_rover.methods.baselines.learned_rgbd.checkpoints import (
    load_training_checkpoint,
)
from cbf_rover.methods.baselines.learned_rgbd.vae import (
    RGBDVAE,
    RGBDVAEConfig,
    rgbd_vae_loss,
)
from .common import (
    LossHistory,
    TrainingSummary,
    load_optimizer_state,
    resolve_device,
    seed_everything,
)


@dataclass(frozen=True)
class VAETrainingConfig:
    dataset_dir: str | Path
    output_dir: str | Path
    epochs: int = 200
    batch_size: int = 32
    latent_dim: int = 64
    image_size: int = 128
    learning_rate: float = 1.0e-4
    depth_weight: float = 1.0
    kl_weight: float = 1.0
    holdout_files: int = 20
    seed: int = 42
    deterministic: bool = False
    workers: int = 2
    resume: str | Path | None = None
    device: str = "auto"

    def __post_init__(self) -> None:
        if min(self.epochs, self.batch_size, self.latent_dim, self.image_size) <= 0:
            raise ValueError("epochs, batch_size, latent_dim and image_size must be positive")
        if self.image_size != 128:
            raise ValueError("the inherited RGB-D VAE supports only image_size=128")


def train_vae(config: VAETrainingConfig) -> TrainingSummary:
    """Train the dual-branch VAE and persist restartable checkpoints."""

    seed_everything(config.seed, deterministic=config.deterministic)
    device = resolve_device(config.device)
    output_dir = Path(config.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    train_dataset = RGBDFrameDataset(
        config.dataset_dir,
        train=True,
        holdout_files=config.holdout_files,
    )
    validation_dataset = RGBDFrameDataset(
        config.dataset_dir,
        train=False,
        holdout_files=config.holdout_files,
    )
    train_loader = DataLoader(
        train_dataset,
        batch_size=config.batch_size,
        shuffle=True,
        num_workers=config.workers,
    )
    validation_loader = DataLoader(
        validation_dataset,
        batch_size=config.batch_size,
        shuffle=False,
        num_workers=config.workers,
    )

    model = RGBDVAE(
        RGBDVAEConfig(latent_dim=config.latent_dim, image_size=config.image_size)
    ).to(device)
    optimizer = torch.optim.Adam(model.parameters(), lr=config.learning_rate)
    start_epoch = 0
    checkpoint = None
    if config.resume:
        checkpoint = load_training_checkpoint(config.resume, device)
        model.load_state_dict(checkpoint["state_dict"])
        start_epoch = int(checkpoint.get("epoch", 0))
        if "optimizer" in checkpoint:
            load_optimizer_state(optimizer, checkpoint["optimizer"], device)

    history = LossHistory(output_dir, "RGB-D VAE loss", resume=checkpoint is not None)
    best = history.best_validation_loss
    if best is None and checkpoint is not None:
        best = float(checkpoint.get("best_loss", float("inf")))
    if best is None:
        best = float("inf")

    for epoch in range(start_epoch + 1, config.epochs + 1):
        model.train()
        train_total = 0.0
        for rgb, depth in train_loader:
            rgb, depth = rgb.to(device), depth.to(device)
            optimizer.zero_grad()
            rgb_out, depth_out, mu, log_sigma, _ = model(rgb, depth)
            loss = rgbd_vae_loss(
                rgb_out,
                depth_out,
                rgb,
                depth,
                mu,
                log_sigma,
                config.depth_weight,
                config.kl_weight,
            )
            loss.backward()
            optimizer.step()
            train_total += float(loss.item())
        train_loss = train_total / max(len(train_dataset), 1)

        model.eval()
        validation_total = 0.0
        with torch.no_grad():
            for rgb, depth in validation_loader:
                rgb, depth = rgb.to(device), depth.to(device)
                rgb_out, depth_out, mu, log_sigma, _ = model(rgb, depth)
                validation_total += float(
                    rgbd_vae_loss(
                        rgb_out,
                        depth_out,
                        rgb,
                        depth,
                        mu,
                        log_sigma,
                        config.depth_weight,
                        config.kl_weight,
                    ).item()
                )
        validation_loss = validation_total / max(len(validation_dataset), 1)
        is_best = validation_loss < best
        best = min(best, validation_loss)
        history.append(epoch, train_loss, validation_loss)
        state = {
            "epoch": epoch,
            "state_dict": model.state_dict(),
            "optimizer": optimizer.state_dict(),
            "validation_loss": validation_loss,
            "best_loss": best,
            "config": asdict(config),
        }
        torch.save(state, output_dir / "checkpoint.tar")
        if is_best:
            torch.save(state, output_dir / "best.tar")

    return TrainingSummary("rgbd_vae", output_dir, config.epochs, best)
