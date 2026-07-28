"""Torch datasets for the learned RGB-D baseline."""

from __future__ import annotations

from bisect import bisect_right
from pathlib import Path
from typing import Any

import h5py
import numpy as np
import torch
from torch.utils.data import Dataset

from .schema import CERTIFICATE_FIELDS, EPISODE_FIELDS


class RGBDFrameDataset(Dataset):
    """Flatten frames from per-episode HDF5 files for VAE training."""

    def __init__(self, root: str | Path, *, train: bool, holdout_files: int = 20) -> None:
        files = sorted(Path(root).glob("*.h5"))
        if holdout_files <= 0:
            raise ValueError("holdout_files must be positive")
        if len(files) <= holdout_files:
            raise ValueError("dataset requires more files than holdout_files")
        self.files = files[:-holdout_files] if train else files[-holdout_files:]
        self.lengths: list[int] = []
        self.cumulative = [0]
        for path in self.files:
            with h5py.File(path, "r") as source:
                length = int(source[EPISODE_FIELDS.rgb_images].shape[0])
            self.lengths.append(length)
            self.cumulative.append(self.cumulative[-1] + length)

    def __len__(self) -> int:
        return self.cumulative[-1]

    def __getitem__(self, index: int) -> tuple[torch.Tensor, torch.Tensor]:
        if index < 0:
            index += len(self)
        if index < 0 or index >= len(self):
            raise IndexError(index)
        file_index = bisect_right(self.cumulative, index) - 1
        frame_index = index - self.cumulative[file_index]
        with h5py.File(self.files[file_index], "r") as source:
            rgb = source[EPISODE_FIELDS.rgb_images][frame_index].astype(np.float32) / 255.0
            depth = source[EPISODE_FIELDS.depth_images][frame_index].astype(np.float32)
        if depth.ndim == 2:
            depth = depth[:, :, np.newaxis]
        return (
            torch.from_numpy(rgb).permute(2, 0, 1),
            torch.from_numpy(depth).permute(2, 0, 1),
        )


class RGBDSequenceDataset(Dataset):
    """Encode fixed-length RGB-D windows lazily for MDN-RNN training."""

    def __init__(
        self,
        h5_path: str | Path,
        *,
        encoder: Any,
        device: torch.device | str,
    ) -> None:
        self.h5_path = Path(h5_path)
        self.encoder = encoder
        self.device = device
        with h5py.File(self.h5_path, "r") as source:
            self.sequence_count = int(source[EPISODE_FIELDS.rgb_images].shape[0])
        self._source: h5py.File | None = None
        self.encoder.eval()

    def __len__(self) -> int:
        return self.sequence_count

    def __getitem__(
        self,
        index: int,
    ) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor, torch.Tensor, torch.Tensor]:
        if self._source is None:
            self._source = h5py.File(self.h5_path, "r")
        rgb = torch.from_numpy(
            self._source[EPISODE_FIELDS.rgb_images][index].astype(np.float32) / 255.0
        ).permute(0, 3, 1, 2)
        depth_array = self._source[EPISODE_FIELDS.depth_images][index].astype(np.float32)
        if depth_array.ndim == 3:
            depth_array = depth_array[:, :, :, np.newaxis]
        depth = torch.from_numpy(depth_array).permute(0, 3, 1, 2)
        actions = torch.tensor(
            self._source[EPISODE_FIELDS.actions][index],
            dtype=torch.float32,
        )
        rewards = torch.tensor(
            self._source[EPISODE_FIELDS.rewards][index],
            dtype=torch.float32,
        )
        dones = torch.tensor(
            self._source[EPISODE_FIELDS.dones][index],
            dtype=torch.float32,
        )
        with torch.no_grad():
            latent = self.encoder.encode_to_latent(
                rgb.to(self.device),
                depth.to(self.device),
                use_mu=True,
            ).cpu()
        return latent[:-1], latent[1:], actions[:-1], rewards[:-1], dones[:-1]

    def close(self) -> None:
        if self._source is not None:
            self._source.close()
            self._source = None

    def __del__(self) -> None:
        self.close()


class LatentValueDataset(Dataset):
    """Buffered index over NPZ rollout files for learned CBF/CLF training."""

    def __init__(
        self,
        root: str | Path,
        *,
        target_key: str,
        include_flag: bool = False,
    ) -> None:
        if target_key not in (CERTIFICATE_FIELDS.cbf, CERTIFICATE_FIELDS.clf):
            raise ValueError("target_key must be 'cbf' or 'clf'")
        self.target_key = target_key
        self.include_flag = include_flag
        self.files = sorted(Path(root).glob("*/*.npz"))
        if not self.files:
            raise FileNotFoundError(f"No .npz files found in {root}")
        self.lengths: list[int] = []
        self.cumulative = [0]
        for path in self.files:
            with np.load(path) as source:
                length = int(source[target_key].shape[0])
            self.lengths.append(length)
            self.cumulative.append(self.cumulative[-1] + length)

    def __len__(self) -> int:
        return self.cumulative[-1]

    def __getitem__(self, index: int) -> Any:
        if index < 0:
            index += len(self)
        if index < 0 or index >= len(self):
            raise IndexError(index)
        file_index = bisect_right(self.cumulative, index) - 1
        local_index = index - self.cumulative[file_index]
        with np.load(self.files[file_index]) as source:
            latent = torch.tensor(
                source[CERTIFICATE_FIELDS.latent][local_index].astype(np.float32)
            )
            target = torch.tensor(source[self.target_key][local_index].astype(np.float32))
            if self.include_flag:
                flag = torch.tensor(
                    source[CERTIFICATE_FIELDS.flag][local_index].astype(np.float32)
                )
                return latent, target, flag
            return latent, target
