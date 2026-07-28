"""File-level dataset transformations used by server-side CLI commands."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Callable

import h5py

from .schema import EPISODE_FIELDS


@dataclass(frozen=True)
class EpisodeMergeConfig:
    input_dir: str | Path
    output_h5: str | Path
    sequence_length: int = 300
    stride: int = 50
    image_size: int = 128

    def __post_init__(self) -> None:
        if self.sequence_length <= 0 or self.stride <= 0 or self.image_size <= 0:
            raise ValueError("sequence_length, stride and image_size must be positive")
        if self.image_size != 128:
            raise ValueError("RGB-D sequence merge supports only image_size=128")


def merge_rgbd_episodes(
    config: EpisodeMergeConfig,
    *,
    progress: Callable[[str], None] | None = None,
) -> int:
    """Merge per-episode HDF5 files into fixed-length training windows."""

    files = sorted(Path(config.input_dir).glob("*.h5"))
    if not files:
        raise FileNotFoundError(f"No h5 files in {config.input_dir}")

    windows: list[tuple[Path, int]] = []
    for path in files:
        with h5py.File(path, "r") as source:
            required = (
                EPISODE_FIELDS.rgb_images,
                EPISODE_FIELDS.depth_images,
                EPISODE_FIELDS.actions,
                EPISODE_FIELDS.rewards,
                EPISODE_FIELDS.dones,
            )
            missing = [field for field in required if field not in source]
            if missing:
                raise KeyError(f"{path} is missing fields: {', '.join(missing)}")

            shapes = {field: source[field].shape for field in required}
            length = int(shapes[EPISODE_FIELDS.rgb_images][0])
            if any(int(shapes[field][0]) != length for field in required):
                raise ValueError(f"{path} has inconsistent episode field lengths")
            if shapes[EPISODE_FIELDS.rgb_images][1:] != (128, 128, 3):
                raise ValueError(f"{path} RGB frames must have shape (N, 128, 128, 3)")
            if shapes[EPISODE_FIELDS.depth_images][1:] not in (
                (128, 128),
                (128, 128, 1),
            ):
                raise ValueError(
                    f"{path} depth frames must have shape (N, 128, 128) "
                    "or (N, 128, 128, 1)"
                )
            if shapes[EPISODE_FIELDS.actions][1:] != (2,):
                raise ValueError(f"{path} actions must have shape (N, 2)")
            if shapes[EPISODE_FIELDS.rewards][1:] != ():
                raise ValueError(f"{path} rewards must have shape (N,)")
            if shapes[EPISODE_FIELDS.dones][1:] != ():
                raise ValueError(f"{path} dones must have shape (N,)")
        for start in range(0, length - config.sequence_length + 1, config.stride):
            windows.append((path, start))
    if not windows:
        raise ValueError(
            "No usable sequence windows; reduce sequence_length or collect longer episodes"
        )

    output_path = Path(config.output_h5)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with h5py.File(output_path, "w") as output:
        count = len(windows)
        length = config.sequence_length
        size = config.image_size
        rgb = output.create_dataset(
            EPISODE_FIELDS.rgb_images,
            (count, length, size, size, 3),
            dtype="uint8",
        )
        depth = output.create_dataset(
            EPISODE_FIELDS.depth_images,
            (count, length, size, size, 1),
            dtype="float32",
        )
        actions = output.create_dataset(
            EPISODE_FIELDS.actions,
            (count, length, 2),
            dtype="float32",
        )
        rewards = output.create_dataset(
            EPISODE_FIELDS.rewards,
            (count, length),
            dtype="float32",
        )
        dones = output.create_dataset(
            EPISODE_FIELDS.dones,
            (count, length),
            dtype="bool",
        )
        for index, (path, start) in enumerate(windows):
            end = start + length
            with h5py.File(path, "r") as source:
                rgb[index] = source[EPISODE_FIELDS.rgb_images][start:end]
                depth_values = source[EPISODE_FIELDS.depth_images][start:end]
                if depth_values.ndim == 3:
                    depth_values = depth_values[:, :, :, None]
                depth[index] = depth_values
                actions[index] = source[EPISODE_FIELDS.actions][start:end]
                rewards[index] = source[EPISODE_FIELDS.rewards][start:end]
                dones[index] = source[EPISODE_FIELDS.dones][start:end]
            if progress is not None:
                progress(f"{index + 1}/{count} {path.name}[{start}:{end}]")
    return len(windows)
