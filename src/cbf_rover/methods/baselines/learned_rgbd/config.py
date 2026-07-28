"""Configuration for the inherited learned RGB-D baseline."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class LearnedRGBDSafetyConfig:
    """Model dimensions and one-step action-correction hyperparameters."""

    latent_size: int = 64
    action_size: int = 2
    recurrent_hidden_size: int = 256
    gaussian_components: int = 5
    clf_decay: float = 0.95
    cbf_decay: float = 0.70
    safe_cbf_margin: float = -0.05
    cbf_recovery_target: float = -0.02
    optimization_steps: int = 10
    optimization_learning_rate: float = 0.08
    safety_weight: float = 1.0
    action_weight: float = 50.0
    max_action_delta: float = 0.10
    image_size: int = 128
    min_depth_m: float = 0.05
    max_depth_m: float = 2.0
    input_is_bgr: bool = True
    satisfaction_tolerance: float = 1.0e-6

    def __post_init__(self) -> None:
        positive_dimensions = (
            self.latent_size,
            self.action_size,
            self.recurrent_hidden_size,
            self.gaussian_components,
            self.image_size,
        )
        if any(value <= 0 for value in positive_dimensions):
            raise ValueError("model dimensions and image_size must be positive")
        if self.image_size != 128:
            raise ValueError("the inherited RGB-D VAE supports only image_size=128")
        if self.optimization_steps < 0:
            raise ValueError("optimization_steps must be non-negative")
        if self.optimization_learning_rate <= 0.0:
            raise ValueError("optimization_learning_rate must be positive")
        if self.max_action_delta < 0.0:
            raise ValueError("max_action_delta must be non-negative")
        if self.min_depth_m >= self.max_depth_m:
            raise ValueError("min_depth_m must be smaller than max_depth_m")
        if self.satisfaction_tolerance < 0.0:
            raise ValueError("satisfaction_tolerance must be non-negative")
