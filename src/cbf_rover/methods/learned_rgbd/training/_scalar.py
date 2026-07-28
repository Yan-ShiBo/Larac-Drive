"""Shared loop for learned scalar certificate regressors."""

from __future__ import annotations

from dataclasses import asdict
from pathlib import Path
from typing import Any, Callable

import torch
from torch.utils.data import DataLoader, Subset

from cbf_rover.datasets.readers import LatentValueDataset
from cbf_rover.datasets.splits import grouped_sample_indices

from .common import LossHistory, TrainingSummary, resolve_device, seed_everything


LossFunction = Callable[..., torch.Tensor]


def train_scalar_model(
    *,
    config: Any,
    model: torch.nn.Module,
    model_name: str,
    target_key: str,
    include_flag: bool,
    loss_function: LossFunction,
) -> TrainingSummary:
    seed_everything(config.seed, deterministic=config.deterministic)
    device = resolve_device(config.device)
    output_dir = Path(config.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    dataset = LatentValueDataset(
        config.dataset_root,
        target_key=target_key,
        include_flag=include_flag,
    )
    train_indices, validation_indices = grouped_sample_indices(
        dataset.lengths,
        test_ratio=config.test_ratio,
        seed=config.seed,
    )
    train_loader = DataLoader(
        Subset(dataset, train_indices),
        batch_size=config.batch_size,
        shuffle=True,
        num_workers=config.workers,
    )
    validation_loader = DataLoader(
        Subset(dataset, validation_indices),
        batch_size=config.batch_size,
        shuffle=False,
        num_workers=config.workers,
    )
    model = model.to(device)
    optimizer = torch.optim.Adam(model.parameters(), lr=config.learning_rate)
    history = LossHistory(output_dir, f"{model_name} loss")
    best = float("inf")

    for epoch in range(1, config.epochs + 1):
        model.train()
        train_total = 0.0
        for batch in train_loader:
            latent = batch[0].to(device)
            target = batch[1].to(device).unsqueeze(1)
            extra = [value.to(device).unsqueeze(1) for value in batch[2:]]
            optimizer.zero_grad()
            loss = loss_function(model(latent), target, *extra)
            loss.backward()
            optimizer.step()
            train_total += float(loss.item())
        train_loss = train_total / max(len(train_loader), 1)

        model.eval()
        validation_total = 0.0
        with torch.no_grad():
            for batch in validation_loader:
                latent = batch[0].to(device)
                target = batch[1].to(device).unsqueeze(1)
                extra = [value.to(device).unsqueeze(1) for value in batch[2:]]
                validation_total += float(
                    loss_function(model(latent), target, *extra).item()
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

    return TrainingSummary(model_name, output_dir, config.epochs, best)
