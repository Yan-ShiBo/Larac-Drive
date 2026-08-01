"""Exact stochastic single-integrator simulator and SAC Gym adapter."""

from __future__ import annotations

from dataclasses import dataclass, field
from math import cos, pi, sin, sqrt
from typing import Any, Literal

import numpy as np

from cbf_rover.domain.planar import (
    CANONICAL_SCENARIO,
    CANONICAL_SYSTEM,
    PlanarAction,
    PlanarState,
    StaticObstacleScenario,
    StochasticSingleIntegrator2D,
)


ResetMode = Literal["initial_disk", "uniform_psi", "mixed"]


def sample_disk(
    rng: np.random.Generator,
    center: tuple[float, float],
    radius: float,
) -> PlanarState:
    angle = float(rng.uniform(-pi, pi))
    radial = float(radius * sqrt(rng.uniform(0.0, 1.0)))
    return PlanarState(
        center[0] + radial * cos(angle),
        center[1] + radial * sin(angle),
    )


def sample_uniform_psi(
    rng: np.random.Generator,
    count: int,
    *,
    scenario: StaticObstacleScenario = CANONICAL_SCENARIO,
) -> np.ndarray:
    """Draw ordered rejection samples from ``Uniform(X minus (Xu union Xg))``."""

    if isinstance(count, bool) or not isinstance(count, int) or count <= 0:
        raise ValueError("count must be a positive integer")
    accepted = np.empty((count, 2), dtype=np.float64)
    written = 0
    while written < count:
        batch_size = max(1024, 2 * (count - written))
        x_values = rng.uniform(
            scenario.domain.x_min,
            scenario.domain.x_max,
            size=batch_size,
        )
        y_values = rng.uniform(
            scenario.domain.y_min,
            scenario.domain.y_max,
            size=batch_size,
        )
        for x_value, y_value in zip(x_values, y_values):
            state = PlanarState(float(x_value), float(y_value))
            if scenario.is_nonterminal_safe(state):
                accepted[written] = state.as_tuple()
                written += 1
                if written == count:
                    break
    return accepted


@dataclass(frozen=True, slots=True)
class ReferenceActionCodec:
    """T2 SAC-only continuous square-to-disk action mapping.

    The final PB1 controller does not use this codec.  Its conservative
    ``0.42 m/s`` radius leaves room for approximation while the certified
    action set remains the S1 ``0.50 m/s`` disk.
    """

    reference_max_speed_mps: float = 0.42
    codec_id: str = "ST1_CONCENTRIC_SQUARE_TO_DISK_042"

    def decode(self, raw_action: Any) -> PlanarAction:
        values = np.asarray(raw_action, dtype=np.float64).reshape(-1)
        if values.shape != (2,) or not np.isfinite(values).all():
            raise ValueError("raw SAC action must contain two finite values")
        if np.any(values < -1.0000001) or np.any(values > 1.0000001):
            raise ValueError("raw SAC action lies outside [-1,1]^2")
        a_value = float(min(1.0, max(-1.0, values[0])))
        b_value = float(min(1.0, max(-1.0, values[1])))
        if a_value == 0.0 and b_value == 0.0:
            return PlanarAction(0.0, 0.0)
        if abs(a_value) > abs(b_value):
            radius = a_value
            angle = (pi / 4.0) * (b_value / a_value)
        else:
            radius = b_value
            angle = (pi / 2.0) - (pi / 4.0) * (a_value / b_value)
        return PlanarAction(
            self.reference_max_speed_mps * radius * cos(angle),
            self.reference_max_speed_mps * radius * sin(angle),
        )


def _distance_to_box(state: PlanarState, box: Any) -> float:
    dx = max(box.x_min - state.x_m, 0.0, state.x_m - box.x_max)
    dy = max(box.y_min - state.y_m, 0.0, state.y_m - box.y_max)
    return sqrt(dx * dx + dy * dy)


@dataclass(slots=True)
class ExactPlanarEnvironment:
    """Stateful exact simulator used by both diagnostics and Gym."""

    scenario: StaticObstacleScenario = CANONICAL_SCENARIO
    system: StochasticSingleIntegrator2D = CANONICAL_SYSTEM
    max_steps: int = 2000
    reset_mode: ResetMode = "initial_disk"
    mixed_initial_probability: float = 0.30
    _rng: np.random.Generator = field(init=False, repr=False)
    _state: PlanarState | None = field(init=False, default=None, repr=False)
    _step_count: int = field(init=False, default=0, repr=False)

    def __post_init__(self) -> None:
        if self.max_steps <= 0:
            raise ValueError("max_steps must be positive")
        if self.reset_mode not in {"initial_disk", "uniform_psi", "mixed"}:
            raise ValueError("unknown reset mode")
        if not 0.0 <= self.mixed_initial_probability <= 1.0:
            raise ValueError("mixed initial probability must lie in [0,1]")
        self._rng = np.random.default_rng(0)
        self._state: PlanarState | None = None
        self._step_count = 0

    @property
    def state(self) -> PlanarState:
        if self._state is None:
            raise RuntimeError("environment must be reset before use")
        return self._state

    @property
    def step_count(self) -> int:
        return self._step_count

    def reset(self, *, seed: int) -> PlanarState:
        self._rng = np.random.default_rng(int(seed))
        mode = self.reset_mode
        if mode == "mixed":
            mode = (
                "initial_disk"
                if self._rng.uniform() < self.mixed_initial_probability
                else "uniform_psi"
            )
        if mode == "initial_disk":
            state = sample_disk(
                self._rng,
                (
                    self.scenario.initial.center_x,
                    self.scenario.initial.center_y,
                ),
                self.scenario.initial.radius,
            )
        else:
            point = sample_uniform_psi(self._rng, 1, scenario=self.scenario)[0]
            state = PlanarState(float(point[0]), float(point[1]))
        self._state = state
        self._step_count = 0
        return state

    def step(
        self,
        action: PlanarAction,
    ) -> tuple[PlanarState, float, bool, bool, dict[str, Any]]:
        previous = self.state
        if not self.system.action_is_admissible(action, tolerance=1.0e-12):
            return previous, -150.0, True, False, {
                "termination_reason": "action_contract_breach",
                "action_contract_breach": True,
                "unsafe_entry": False,
                "model_contract_breach": False,
                "goal_reached": False,
            }
        half_width = self.system.noise.half_width_m
        noise = self._rng.triangular(
            -half_width,
            0.0,
            half_width,
            size=2,
        )
        successor = self.system.successor(previous, action, noise)
        self._state = successor
        self._step_count += 1
        reason = self.scenario.terminal_reason(successor)
        terminated = reason is not None
        truncated = not terminated and self._step_count >= self.max_steps
        if truncated:
            reason = "timeout"

        previous_goal_distance = sqrt(
            self.scenario.goal.squared_distance(previous)
        )
        next_goal_distance = sqrt(self.scenario.goal.squared_distance(successor))
        progress = previous_goal_distance - next_goal_distance
        speed_penalty = 0.03 * action.squared_speed_m2ps2
        clearance = min(
            _distance_to_box(successor, obstacle)
            for obstacle in self.scenario.unsafe_components
        )
        clearance_penalty = 0.0
        if clearance < 0.20:
            clearance_penalty = 2.0 * (0.20 - clearance) ** 2
        boundary_margin = min(
            successor.x_m - self.scenario.domain.x_min,
            self.scenario.domain.x_max - successor.x_m,
            successor.y_m - self.scenario.domain.y_min,
            self.scenario.domain.y_max - successor.y_m,
        )
        boundary_penalty = 0.0
        if boundary_margin < 0.15:
            boundary_penalty = 2.0 * (0.15 - boundary_margin) ** 2
        reward = (
            30.0 * progress
            - 0.01
            - speed_penalty
            - clearance_penalty
            - boundary_penalty
        )
        if reason == "goal_reached":
            reward += 100.0
        elif reason in {"unsafe_entry", "model_contract_breach"}:
            reward -= 150.0
        info = {
            "termination_reason": reason,
            "goal_reached": reason == "goal_reached",
            "unsafe_entry": reason == "unsafe_entry",
            "model_contract_breach": reason == "model_contract_breach",
            "action_contract_breach": False,
            "noise_m": [float(noise[0]), float(noise[1])],
            "state_m": list(successor.as_tuple()),
            "action_mps": list(action.as_tuple()),
        }
        return successor, float(reward), terminated, truncated, info


def build_gymnasium_environment(
    *,
    reset_mode: ResetMode,
    seed: int,
    max_steps: int = 2000,
    codec: ReferenceActionCodec = ReferenceActionCodec(),
) -> Any:
    """Construct the Gym wrapper lazily so pure-math imports stay lightweight."""

    import gymnasium as gym

    class _GymExactEnvironment(gym.Env):
        metadata = {"render_modes": []}

        def __init__(self) -> None:
            super().__init__()
            self.action_space = gym.spaces.Box(
                low=-1.0,
                high=1.0,
                shape=(2,),
                dtype=np.float32,
            )
            self.observation_space = gym.spaces.Box(
                low=-1.0,
                high=1.0,
                shape=(2,),
                dtype=np.float32,
            )
            self.core = ExactPlanarEnvironment(
                max_steps=max_steps,
                reset_mode=reset_mode,
            )
            self.codec = codec
            self._next_seed = int(seed)

        def _observation(self, state: PlanarState) -> np.ndarray:
            xi = self.core.scenario.normalization.normalize(state)
            return np.asarray(xi.as_tuple(), dtype=np.float32)

        def reset(
            self,
            *,
            seed: int | None = None,
            options: dict[str, Any] | None = None,
        ) -> tuple[np.ndarray, dict[str, Any]]:
            del options
            super().reset(seed=seed)
            actual_seed = self._next_seed if seed is None else int(seed)
            self._next_seed = actual_seed + 1
            state = self.core.reset(seed=actual_seed)
            return self._observation(state), {"seed": actual_seed}

        def step(
            self,
            raw_action: Any,
        ) -> tuple[np.ndarray, float, bool, bool, dict[str, Any]]:
            action = self.codec.decode(raw_action)
            state, reward, terminated, truncated, info = self.core.step(action)
            info["raw_action"] = np.asarray(
                raw_action,
                dtype=np.float64,
            ).reshape(2).tolist()
            return (
                self._observation(state),
                reward,
                terminated,
                truncated,
                info,
            )

    environment = _GymExactEnvironment()
    environment.reset(seed=seed)
    return environment
