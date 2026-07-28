"""RGB-D preprocessing owned by the learned RGB-D method."""

from __future__ import annotations

from dataclasses import dataclass

import cv2
import numpy as np
import torch


@dataclass(frozen=True)
class RGBDPreprocessConfig:
    image_size: int = 128
    min_depth_m: float = 0.05
    max_depth_m: float = 2.0
    input_is_bgr: bool = True

    def __post_init__(self) -> None:
        if self.image_size <= 0:
            raise ValueError("image_size must be positive")
        if self.min_depth_m >= self.max_depth_m:
            raise ValueError("min_depth_m must be smaller than max_depth_m")


def preprocess_rgbd_numpy(
    color_image: np.ndarray,
    depth_image: np.ndarray,
    config: RGBDPreprocessConfig = RGBDPreprocessConfig(),
) -> tuple[np.ndarray, np.ndarray]:
    """Resize and normalize one aligned color/depth observation."""

    if color_image is None or depth_image is None:
        raise ValueError("color_image and depth_image must not be None")

    rgb = cv2.resize(color_image, (config.image_size, config.image_size))
    if config.input_is_bgr:
        rgb = cv2.cvtColor(rgb, cv2.COLOR_BGR2RGB)
    rgb = rgb.astype(np.float32) / 255.0

    depth = cv2.resize(depth_image, (config.image_size, config.image_size))
    depth = np.asarray(depth, dtype=np.float32)
    depth = np.nan_to_num(
        depth,
        nan=config.max_depth_m,
        posinf=config.max_depth_m,
        neginf=config.min_depth_m,
    )
    depth = np.clip(depth, config.min_depth_m, config.max_depth_m)
    depth = (depth - config.min_depth_m) / (config.max_depth_m - config.min_depth_m)
    return rgb, depth[:, :, np.newaxis].astype(np.float32)


def preprocess_rgbd_tensors(
    color_image: np.ndarray,
    depth_image: np.ndarray,
    config: RGBDPreprocessConfig = RGBDPreprocessConfig(),
    device: torch.device | str = "cpu",
) -> tuple[torch.Tensor, torch.Tensor]:
    """Convert normalized RGB-D arrays into batched channel-first tensors."""

    rgb, depth = preprocess_rgbd_numpy(color_image, depth_image, config)
    rgb_tensor = torch.from_numpy(rgb).permute(2, 0, 1).unsqueeze(0).to(device)
    depth_tensor = torch.from_numpy(depth).permute(2, 0, 1).unsqueeze(0).to(device)
    return rgb_tensor, depth_tensor
