"""Server-side adapter for learned latent-dynamics training."""

from __future__ import annotations

from dataclasses import asdict, dataclass
from pathlib import Path

import torch
from torch.utils.data import DataLoader, random_split

from cbf_rover.adapters.datasets.readers import RGBDSequenceDataset

from cbf_rover.methods.baselines.learned_rgbd.checkpoints import (
    load_training_checkpoint,
    state_dict_from_checkpoint,
)
from cbf_rover.methods.baselines.learned_rgbd.mdrnn import (
    MDNRNN,
    gaussian_nll_loss,
)
from cbf_rover.methods.baselines.learned_rgbd.vae import RGBDVAE, RGBDVAEConfig
from .common import LossHistory, TrainingSummary, resolve_device, seed_everything


@dataclass(frozen=True)
class MDRNNTrainingConfig:
    sequence_h5: str | Path
    vae_checkpoint: str | Path
    output_dir: str | Path
    epochs: int = 300
    batch_size: int = 1
    latent_dim: int = 64
    action_dim: int = 2
    hidden_dim: int = 256
    gaussian_components: int = 5
    learning_rate: float = 1.0e-4
    temperature: float = 1.0
    gradient_clip: float = 1.0
    test_ratio: float = 0.1
    seed: int = 42
    deterministic: bool = False
    resume: str | Path | None = None
    device: str = "auto"

    def __post_init__(self) -> None:
        if min(
            self.epochs,
            self.batch_size,
            self.latent_dim,
            self.action_dim,
            self.hidden_dim,
            self.gaussian_components,
        ) <= 0:
            raise ValueError("training and model dimensions must be positive")
        if not 0.0 < self.test_ratio < 1.0:
            raise ValueError("test_ratio must be strictly between zero and one")


def train_mdrnn(config: MDRNNTrainingConfig) -> TrainingSummary:
    seed_everything(config.seed, deterministic=config.deterministic)
    device = resolve_device(config.device)
    output_dir = Path(config.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    vae = RGBDVAE(RGBDVAEConfig(latent_dim=config.latent_dim)).to(device)
    vae.load_state_dict(
        state_dict_from_checkpoint(
            load_training_checkpoint(config.vae_checkpoint, device)
        )
    )
    vae.eval()
    dataset = RGBDSequenceDataset(
        config.sequence_h5,
        encoder=vae,
        device=device,
    )
    if len(dataset) < 2:
        raise ValueError("MDN-RNN training requires at least two sequences")
    validation_length = max(1, int(len(dataset) * config.test_ratio))
    train_length = len(dataset) - validation_length
    generator = torch.Generator().manual_seed(config.seed)
    train_set, validation_set = random_split(
        dataset,
        [train_length, validation_length],
        generator=generator,
    )
    train_loader = DataLoader(train_set, batch_size=config.batch_size, shuffle=True)
    validation_loader = DataLoader(
        validation_set,
        batch_size=config.batch_size,
        shuffle=False,
    )

    model = MDNRNN(
        config.latent_dim,
        config.action_dim,
        config.hidden_dim,
        config.gaussian_components,
    ).to(device)
    if config.resume:
        model.load_state_dict(
            state_dict_from_checkpoint(
                load_training_checkpoint(config.resume, device)
            )
        )
    optimizer = torch.optim.Adam(model.parameters(), lr=config.learning_rate)
    history = LossHistory(output_dir, "RGB-D MDN-RNN loss")
    best = float("inf")

    for epoch in range(1, config.epochs + 1):
        model.train()
        train_total = 0.0
        for latent, next_latent, actions, _rewards, _dones in train_loader:
            latent, next_latent, actions = (
                latent.to(device),
                next_latent.to(device),
                actions.to(device),
            )
            optimizer.zero_grad()
            weights, means, scales, _ = model(
                latent,
                actions,
                temperature=config.temperature,
            )
            loss = gaussian_nll_loss(weights, means, scales, next_latent)
            loss.backward()
            torch.nn.utils.clip_grad_norm_(model.parameters(), config.gradient_clip)
            optimizer.step()
            train_total += float(loss.item())
        train_loss = train_total / max(len(train_loader), 1)

        model.eval()
        validation_total = 0.0
        with torch.no_grad():
            for latent, next_latent, actions, _rewards, _dones in validation_loader:
                weights, means, scales, _ = model(
                    latent.to(device),
                    actions.to(device),
                    temperature=config.temperature,
                )
                validation_total += float(
                    gaussian_nll_loss(
                        weights,
                        means,
                        scales,
                        next_latent.to(device),
                    ).item()
                )
        validation_loss = validation_total / max(len(validation_loader), 1)
        history.append(epoch, train_loss, validation_loss)
        state = {
            "epoch": epoch,
            "state_dict": model.state_dict(),
            "optimizer": optimizer.state_dict(),
            "validation_loss": validation_loss,
            "config": asdict(config),
        }
        torch.save(state, output_dir / "checkpoint.tar")
        if validation_loss < best:
            best = validation_loss
            torch.save(state, output_dir / "best.tar")

    dataset.close()
    return TrainingSummary("rgbd_mdrnn", output_dir, config.epochs, best)
