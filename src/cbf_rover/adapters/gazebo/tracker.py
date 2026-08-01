"""GZ1 world-velocity to differential-drive empirical tracker."""

from __future__ import annotations

from dataclasses import dataclass
from math import atan2, cos, isfinite, pi, sqrt

from cbf_rover.domain.planar import PlanarAction


def wrap_angle(angle_rad: float) -> float:
    value = float(angle_rad)
    if not isfinite(value):
        raise ValueError("angle must be finite")
    return (value + pi) % (2.0 * pi) - pi


@dataclass(frozen=True, slots=True)
class DifferentialDriveCommand:
    linear_mps: float
    angular_radps: float
    unsaturated_linear_mps: float
    unsaturated_angular_radps: float
    saturation_applied: bool
    claim_level: str = "empirical"


@dataclass(frozen=True, slots=True)
class EmpiricalDifferentialTracker:
    heading_gain: float = 2.0
    max_linear_mps: float = 0.5
    max_angular_radps: float = 1.5
    sample_period_s: float = 0.05
    tracker_id: str = "GZ1_EMPIRICAL_DIFFERENTIAL_TRACKER_V1"

    def command(
        self,
        desired_world_velocity: PlanarAction,
        *,
        heading_rad: float,
    ) -> DifferentialDriveCommand:
        heading = float(heading_rad)
        if not isfinite(heading):
            raise ValueError("heading must be finite")
        speed = sqrt(desired_world_velocity.squared_speed_m2ps2)
        if speed <= 1.0e-15:
            return DifferentialDriveCommand(0.0, 0.0, 0.0, 0.0, False)
        desired_heading = atan2(
            desired_world_velocity.u_y_mps,
            desired_world_velocity.u_x_mps,
        )
        error = wrap_angle(desired_heading - heading)
        linear_raw = speed * cos(error)
        angular_raw = self.heading_gain * error
        linear = min(self.max_linear_mps, max(-self.max_linear_mps, linear_raw))
        angular = min(
            self.max_angular_radps,
            max(-self.max_angular_radps, angular_raw),
        )
        return DifferentialDriveCommand(
            linear_mps=linear,
            angular_radps=angular,
            unsaturated_linear_mps=linear_raw,
            unsaturated_angular_radps=angular_raw,
            saturation_applied=linear != linear_raw or angular != angular_raw,
        )
