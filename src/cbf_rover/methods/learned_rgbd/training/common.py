"""Shared training utilities kept local to the learned RGB-D method."""

from __future__ import annotations

import csv
import random
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import matplotlib
import numpy as np
import torch

matplotlib.use("Agg")
import matplotlib.pyplot as plot


@dataclass(frozen=True)
class TrainingSummary:
    model_name: str
    output_dir: Path
    epochs_completed: int
    best_validation_loss: float


class LossHistory:
    """Persist a restartable CSV history and a compact loss curve."""

    def __init__(self, output_dir: str | Path, title: str, *, resume: bool = False) -> None:
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.title = title
        self.epochs: list[int] = []
        self.train_losses: list[float] = []
        self.validation_losses: list[float] = []
        if resume:
            self._load()

    @property
    def best_validation_loss(self) -> float | None:
        return min(self.validation_losses) if self.validation_losses else None

    def append(self, epoch: int, train_loss: float, validation_loss: float) -> None:
        self.epochs.append(int(epoch))
        self.train_losses.append(float(train_loss))
        self.validation_losses.append(float(validation_loss))
        self._save_csv()
        self._save_plot()

    def _load(self) -> None:
        path = self.output_dir / "loss_history.csv"
        if not path.is_file():
            return
        with path.open("r", newline="", encoding="utf-8") as stream:
            for row in csv.DictReader(stream):
                self.epochs.append(int(row["epoch"]))
                self.train_losses.append(float(row["train_loss"]))
                self.validation_losses.append(float(row["validation_loss"]))

    def _save_csv(self) -> None:
        with (self.output_dir / "loss_history.csv").open(
            "w",
            newline="",
            encoding="utf-8",
        ) as stream:
            writer = csv.writer(stream)
            writer.writerow(["epoch", "train_loss", "validation_loss"])
            writer.writerows(
                zip(self.epochs, self.train_losses, self.validation_losses)
            )

    def _save_plot(self) -> None:
        figure, axis = plot.subplots(figsize=(8, 5))
        axis.plot(self.epochs, self.train_losses, label="train")
        axis.plot(self.epochs, self.validation_losses, label="validation")
        axis.set(title=self.title, xlabel="epoch", ylabel="loss")
        axis.grid(True, linestyle="--", alpha=0.35)
        axis.legend()
        figure.tight_layout()
        figure.savefig(self.output_dir / "loss_curve.png", dpi=160)
        plot.close(figure)


def seed_everything(seed: int, *, deterministic: bool = False) -> None:
    if seed < 0:
        raise ValueError("seed must be non-negative")
    random.seed(seed)
    np.random.seed(seed)
    torch.manual_seed(seed)
    if torch.cuda.is_available():
        torch.cuda.manual_seed_all(seed)
    if deterministic:
        torch.use_deterministic_algorithms(True, warn_only=True)
        if torch.backends.cudnn.is_available():
            torch.backends.cudnn.benchmark = False
            torch.backends.cudnn.deterministic = True


def resolve_device(requested: str) -> torch.device:
    normalized = requested.lower()
    if normalized == "auto":
        return torch.device("cuda" if torch.cuda.is_available() else "cpu")
    device = torch.device(normalized)
    if device.type == "cuda" and not torch.cuda.is_available():
        raise RuntimeError("CUDA was requested but is not available")
    return device


def load_optimizer_state(
    optimizer: torch.optim.Optimizer,
    state: Any,
    device: torch.device,
) -> None:
    optimizer.load_state_dict(state)
    for item in optimizer.state.values():
        for key, value in item.items():
            if torch.is_tensor(value):
                item[key] = value.to(device)


def checkpoint_state_dict(checkpoint: Any) -> Any:
    if isinstance(checkpoint, dict) and "state_dict" in checkpoint:
        return checkpoint["state_dict"]
    return checkpoint
