"""Unified CLI replacing the deleted top-level data/train scripts."""

from __future__ import annotations

import argparse
from collections.abc import Sequence


def _device(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--device", default="auto", help="auto, cpu or cuda[:N]")


def _reproducibility(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--deterministic", action="store_true")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="cbf-rover")
    commands = parser.add_subparsers(dest="command", required=True)

    merge = commands.add_parser("merge-episodes")
    merge.add_argument("--input-dir", required=True)
    merge.add_argument("--output-h5", required=True)
    merge.add_argument("--sequence-length", type=int, default=300)
    merge.add_argument("--stride", type=int, default=50)
    merge.add_argument("--image-size", type=int, choices=(128,), default=128)

    certificate_data = commands.add_parser("create-certificate-dataset")
    certificate_data.add_argument("--input-root", required=True)
    certificate_data.add_argument("--output-root", required=True)
    certificate_data.add_argument("--vae-checkpoint", required=True)
    certificate_data.add_argument("--target", choices=("cbf", "clf"), required=True)
    certificate_data.add_argument("--latent-dim", type=int, default=64)
    certificate_data.add_argument("--batch-size", type=int, default=64)
    _device(certificate_data)

    vae = commands.add_parser("train-vae")
    vae.add_argument("--dataset-dir", required=True)
    vae.add_argument("--output-dir", required=True)
    vae.add_argument("--epochs", type=int, default=200)
    vae.add_argument("--batch-size", type=int, default=32)
    vae.add_argument("--latent-dim", type=int, default=64)
    vae.add_argument("--image-size", type=int, choices=(128,), default=128)
    vae.add_argument("--learning-rate", type=float, default=1.0e-4)
    vae.add_argument("--depth-weight", type=float, default=1.0)
    vae.add_argument("--kl-weight", type=float, default=1.0)
    vae.add_argument("--holdout-files", type=int, default=20)
    vae.add_argument("--workers", type=int, default=2)
    vae.add_argument("--resume")
    _device(vae)
    _reproducibility(vae)

    mdrnn = commands.add_parser("train-mdrnn")
    mdrnn.add_argument("--sequence-h5", required=True)
    mdrnn.add_argument("--vae-checkpoint", required=True)
    mdrnn.add_argument("--output-dir", required=True)
    mdrnn.add_argument("--epochs", type=int, default=300)
    mdrnn.add_argument("--batch-size", type=int, default=1)
    mdrnn.add_argument("--latent-dim", type=int, default=64)
    mdrnn.add_argument("--action-dim", type=int, default=2)
    mdrnn.add_argument("--hidden-dim", type=int, default=256)
    mdrnn.add_argument("--gaussian-components", type=int, default=5)
    mdrnn.add_argument("--learning-rate", type=float, default=1.0e-4)
    mdrnn.add_argument("--temperature", type=float, default=1.0)
    mdrnn.add_argument("--gradient-clip", type=float, default=1.0)
    mdrnn.add_argument("--test-ratio", type=float, default=0.1)
    mdrnn.add_argument("--resume")
    _device(mdrnn)
    _reproducibility(mdrnn)

    for name, batch_size in (("train-cbf", 64), ("train-clf", 128)):
        scalar = commands.add_parser(name)
        scalar.add_argument("--dataset-root", required=True)
        scalar.add_argument("--output-dir", required=True)
        scalar.add_argument("--epochs", type=int, default=1000)
        scalar.add_argument("--batch-size", type=int, default=batch_size)
        scalar.add_argument("--latent-dim", type=int, default=64)
        scalar.add_argument("--learning-rate", type=float, default=1.0e-4)
        scalar.add_argument("--test-ratio", type=float, default=0.1)
        scalar.add_argument("--workers", type=int, default=2)
        if name == "train-cbf":
            scalar.add_argument("--flag-weight", type=float, default=10.0)
            scalar.add_argument(
                "--false-positive-threshold",
                type=float,
                default=-25.0,
            )
            scalar.add_argument(
                "--false-positive-weight",
                type=float,
                default=10.0,
            )
        _device(scalar)
        _reproducibility(scalar)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    values = vars(args).copy()
    command = values.pop("command")

    if command == "merge-episodes":
        from cbf_rover.datasets import EpisodeMergeConfig, merge_rgbd_episodes

        count = merge_rgbd_episodes(EpisodeMergeConfig(**values), progress=print)
        print(f"merged_sequences={count}")
        return 0

    if command == "create-certificate-dataset":
        from cbf_rover.methods.learned_rgbd.training import (
            CertificateDatasetConfig,
            create_certificate_dataset,
        )

        count = create_certificate_dataset(CertificateDatasetConfig(**values))
        print(f"written_rollouts={count}")
        return 0

    if command == "train-vae":
        from cbf_rover.methods.learned_rgbd.training import (
            VAETrainingConfig,
            train_vae,
        )

        summary = train_vae(VAETrainingConfig(**values))
    elif command == "train-mdrnn":
        from cbf_rover.methods.learned_rgbd.training import (
            MDRNNTrainingConfig,
            train_mdrnn,
        )

        summary = train_mdrnn(MDRNNTrainingConfig(**values))
    elif command == "train-cbf":
        from cbf_rover.methods.learned_rgbd.training import (
            CBFTrainingConfig,
            train_cbf,
        )

        summary = train_cbf(CBFTrainingConfig(**values))
    else:
        from cbf_rover.methods.learned_rgbd.training import (
            CLFTrainingConfig,
            train_clf,
        )

        summary = train_clf(CLFTrainingConfig(**values))
    print(
        f"model={summary.model_name} epochs={summary.epochs_completed} "
        f"best_validation_loss={summary.best_validation_loss:.8g} "
        f"output_dir={summary.output_dir}"
    )
    return 0
