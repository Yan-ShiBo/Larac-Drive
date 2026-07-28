"""Create learned-certificate latent datasets from RGB-D rollout frames."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np
import torch

from cbf_rover.datasets.schema import (
    CERTIFICATE_FIELDS,
    RAW_CERTIFICATE_FRAME_FIELDS,
)

from ..checkpoints import load_training_checkpoint, state_dict_from_checkpoint
from ..vae import RGBDVAE, RGBDVAEConfig
from .common import resolve_device


@dataclass(frozen=True)
class CertificateDatasetConfig:
    input_root: str | Path
    output_root: str | Path
    vae_checkpoint: str | Path
    target: str
    latent_dim: int = 64
    batch_size: int = 64
    device: str = "auto"

    def __post_init__(self) -> None:
        if self.target not in (CERTIFICATE_FIELDS.cbf, CERTIFICATE_FIELDS.clf):
            raise ValueError("target must be 'cbf' or 'clf'")
        if self.latent_dim <= 0 or self.batch_size <= 0:
            raise ValueError("latent_dim and batch_size must be positive")


def _encode_rollout(
    vae: RGBDVAE,
    rgb: np.ndarray,
    depth: np.ndarray,
    *,
    device: torch.device,
    batch_size: int,
) -> np.ndarray:
    if rgb.shape[1:] != (128, 128, 3):
        raise ValueError("RGB rollout frames must have shape (N, 128, 128, 3)")
    if depth.ndim == 3:
        depth = depth[:, :, :, np.newaxis]
    if depth.shape[1:] != (128, 128, 1):
        raise ValueError(
            "depth rollout frames must have shape (N, 128, 128) "
            "or (N, 128, 128, 1)"
        )
    if len(rgb) != len(depth):
        raise ValueError("RGB and depth rollout lengths must match")
    latents: list[np.ndarray] = []
    with torch.no_grad():
        for start in range(0, len(rgb), batch_size):
            rgb_tensor = torch.from_numpy(
                rgb[start : start + batch_size].astype(np.float32) / 255.0
            ).permute(0, 3, 1, 2).to(device)
            depth_tensor = torch.from_numpy(
                depth[start : start + batch_size].astype(np.float32)
            ).permute(0, 3, 1, 2).to(device)
            latents.append(
                vae.encode_to_latent(
                    rgb_tensor,
                    depth_tensor,
                    use_mu=True,
                ).cpu().numpy()
            )
    if not latents:
        raise ValueError("cannot encode an empty rollout")
    return np.concatenate(latents, axis=0)


def create_certificate_dataset(config: CertificateDatasetConfig) -> int:
    """Encode every nested NPZ rollout and return the written file count."""

    device = resolve_device(config.device)
    vae = RGBDVAE(RGBDVAEConfig(latent_dim=config.latent_dim)).to(device)
    vae.load_state_dict(
        state_dict_from_checkpoint(
            load_training_checkpoint(config.vae_checkpoint, device)
        )
    )
    vae.eval()

    input_root = Path(config.input_root)
    output_root = Path(config.output_root)
    written = 0
    for source_path in sorted(input_root.glob("*/*.npz")):
        with np.load(source_path) as source:
            rgb = source[RAW_CERTIFICATE_FRAME_FIELDS.rgb]
            depth = source[RAW_CERTIFICATE_FRAME_FIELDS.depth]
            target = source[config.target].astype(np.float32)
            flag = (
                source[RAW_CERTIFICATE_FRAME_FIELDS.flag].astype(np.float32)
                if config.target == CERTIFICATE_FIELDS.cbf
                and RAW_CERTIFICATE_FRAME_FIELDS.flag in source
                else None
            )
        latent = _encode_rollout(
            vae,
            rgb,
            depth,
            device=device,
            batch_size=config.batch_size,
        )
        destination = output_root / source_path.parent.name / source_path.name
        destination.parent.mkdir(parents=True, exist_ok=True)
        if config.target == CERTIFICATE_FIELDS.cbf:
            if flag is None:
                flag = np.zeros_like(target, dtype=np.float32)
            np.savez_compressed(
                destination,
                latent=latent,
                cbf=target,
                flag=flag,
            )
        else:
            np.savez_compressed(destination, latent=latent, clf=target)
        written += 1
    if written == 0:
        raise FileNotFoundError(f"No nested NPZ rollouts found in {input_root}")
    return written
