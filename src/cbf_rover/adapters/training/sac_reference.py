"""ST1/V1 deterministic SAC reference training and checkpoint selection."""

from __future__ import annotations

from dataclasses import asdict, dataclass
import hashlib
import json
from pathlib import Path
import platform
from statistics import median
from typing import Any, Iterable, Mapping, Sequence

import numpy as np

from cbf_rover.adapters.exact.environment import (
    ReferenceActionCodec,
    build_gymnasium_environment,
)
from cbf_rover.domain.planar import CANONICAL_SCENARIO, CANONICAL_SYSTEM
from cbf_rover.methods.polynomial_sbc.polynomial import content_sha256


TRAINING_SEEDS = (11, 23, 47, 89, 131)
EVALUATION_SEEDS = (1009, 2027, 3037, 4051, 5059)


def file_sha256(path: str | Path) -> str:
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for chunk in iter(lambda: stream.read(4 * 1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _write_json(path: Path, payload: Mapping[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.tmp")
    temporary.write_text(
        json.dumps(
            payload,
            indent=2,
            sort_keys=True,
            ensure_ascii=False,
            allow_nan=False,
        )
        + "\n",
        encoding="utf-8",
    )
    temporary.replace(path)


@dataclass(frozen=True, slots=True)
class SACTrainingContract:
    """Recommended finite ST1 training budget; it is not a theorem input."""

    total_timesteps: int = 600_000
    learning_rate: float = 3.0e-4
    buffer_size: int = 1_000_000
    learning_starts: int = 10_000
    batch_size: int = 256
    gamma: float = 0.99
    tau: float = 0.005
    train_freq: int = 1
    gradient_steps: int = 1
    network_width: int = 256
    network_depth: int = 2
    entropy_coefficient: str = "auto"
    reset_mode: str = "mixed"
    mixed_initial_probability: float = 0.30
    reference_action_radius_mps: float = 0.42
    max_steps: int = 2000
    contract_id: str = "ST1_SAC_600K_SYMMETRIC_GLOBAL_V1"

    def payload(self) -> dict[str, Any]:
        result = asdict(self)
        result["training_seeds"] = list(TRAINING_SEEDS)
        result["evaluation_seeds"] = list(EVALUATION_SEEDS)
        result["route_bias"] = None
        result["reward_id"] = "T2_PROGRESS_CLEARANCE_SYMMETRIC_V1"
        result["action_codec_id"] = "ST1_CONCENTRIC_SQUARE_TO_DISK_042"
        return result


def exact_contract_payload() -> dict[str, Any]:
    scenario = CANONICAL_SCENARIO
    system = CANONICAL_SYSTEM
    return {
        "system_id": "M1_STOCHASTIC_SINGLE_INTEGRATOR_2D",
        "dt_s": system.dt_s,
        "action_disk_radius_mps": system.max_speed_mps,
        "noise": {
            "id": system.noise.distribution_id,
            "half_width_m": system.noise.half_width_m,
            "axes_independent": True,
            "time_iid": True,
        },
        "domain": [
            scenario.domain.x_min,
            scenario.domain.x_max,
            scenario.domain.y_min,
            scenario.domain.y_max,
        ],
        "initial": [
            scenario.initial.center_x,
            scenario.initial.center_y,
            scenario.initial.radius,
        ],
        "goal": [
            scenario.goal.center_x,
            scenario.goal.center_y,
            scenario.goal.radius,
        ],
        "unsafe": [
            [box.x_min, box.x_max, box.y_min, box.y_max]
            for box in scenario.unsafe_components
        ],
        "normalization": [
            scenario.normalization.center_x_m,
            scenario.normalization.scale_x_m,
            scenario.normalization.center_y_m,
            scenario.normalization.scale_y_m,
        ],
    }


def train_sac_reference(
    *,
    seed: int,
    output_dir: str | Path,
    device: str,
    contract: SACTrainingContract = SACTrainingContract(),
) -> Path:
    """Train exactly one declared seed and publish a hash-bound checkpoint."""

    if int(seed) not in TRAINING_SEEDS:
        raise ValueError(f"seed must be one of {TRAINING_SEEDS}")
    import gymnasium
    import stable_baselines3
    import torch
    from stable_baselines3 import SAC

    torch.manual_seed(int(seed))
    if torch.cuda.is_available():
        torch.cuda.manual_seed_all(int(seed))
    torch.use_deterministic_algorithms(True, warn_only=True)
    torch.backends.cudnn.benchmark = False
    torch.backends.cudnn.deterministic = True

    destination = Path(output_dir).resolve() / f"seed_{int(seed)}"
    destination.mkdir(parents=True, exist_ok=False)
    codec = ReferenceActionCodec(contract.reference_action_radius_mps)
    environment = build_gymnasium_environment(
        reset_mode="mixed",
        seed=int(seed),
        max_steps=contract.max_steps,
        codec=codec,
    )
    environment.core.mixed_initial_probability = (
        contract.mixed_initial_probability
    )
    model = SAC(
        "MlpPolicy",
        environment,
        learning_rate=contract.learning_rate,
        buffer_size=contract.buffer_size,
        learning_starts=contract.learning_starts,
        batch_size=contract.batch_size,
        tau=contract.tau,
        gamma=contract.gamma,
        train_freq=contract.train_freq,
        gradient_steps=contract.gradient_steps,
        ent_coef=contract.entropy_coefficient,
        policy_kwargs={
            "net_arch": [
                contract.network_width
                for _ in range(contract.network_depth)
            ]
        },
        seed=int(seed),
        device=device,
        verbose=1,
    )
    model.learn(total_timesteps=contract.total_timesteps, progress_bar=False)
    checkpoint_stem = destination / "sac_reference"
    model.save(checkpoint_stem)
    checkpoint = checkpoint_stem.with_suffix(".zip")
    manifest = {
        "schema_version": 1,
        "kind": "sac_reference_checkpoint",
        "seed": int(seed),
        "training_contract": contract.payload(),
        "training_contract_sha256": content_sha256(contract.payload()),
        "exact_contract": exact_contract_payload(),
        "exact_contract_sha256": content_sha256(exact_contract_payload()),
        "checkpoint_file": checkpoint.name,
        "checkpoint_sha256": file_sha256(checkpoint),
        "versions": {
            "python": platform.python_version(),
            "numpy": np.__version__,
            "torch": torch.__version__,
            "gymnasium": gymnasium.__version__,
            "stable_baselines3": stable_baselines3.__version__,
        },
        "claim_level": "reference_only",
    }
    _write_json(destination / "manifest.json", manifest)
    environment.close()
    return destination / "manifest.json"


def load_checkpoint_manifest(path: str | Path) -> dict[str, Any]:
    manifest_path = Path(path).resolve()
    payload = json.loads(manifest_path.read_text(encoding="utf-8"))
    checkpoint = manifest_path.parent / str(payload["checkpoint_file"])
    if file_sha256(checkpoint) != str(payload["checkpoint_sha256"]):
        raise ValueError(f"checkpoint hash mismatch: {checkpoint}")
    if content_sha256(payload["exact_contract"]) != str(
        payload["exact_contract_sha256"]
    ):
        raise ValueError("exact-contract hash mismatch")
    if content_sha256(payload["training_contract"]) != str(
        payload["training_contract_sha256"]
    ):
        raise ValueError("training-contract hash mismatch")
    if payload["exact_contract"] != exact_contract_payload():
        raise ValueError("checkpoint exact contract is not the active contract")
    return payload


def _evaluate_one_checkpoint(
    manifest_path: Path,
    *,
    device: str,
    evaluation_seeds: Sequence[int],
    episodes_per_seed: int,
) -> dict[str, Any]:
    from stable_baselines3 import SAC

    manifest = load_checkpoint_manifest(manifest_path)
    checkpoint = manifest_path.parent / str(manifest["checkpoint_file"])
    model = SAC.load(checkpoint, device=device)
    codec = ReferenceActionCodec()
    episode_rows: list[dict[str, Any]] = []
    for evaluation_seed in evaluation_seeds:
        for episode_index in range(episodes_per_seed):
            episode_seed = int(evaluation_seed) * 10_000 + episode_index
            environment = build_gymnasium_environment(
                reset_mode="initial_disk",
                seed=episode_seed,
                max_steps=2000,
                codec=codec,
            )
            observation, _ = environment.reset(seed=episode_seed)
            last_info: dict[str, Any] = {}
            terminated = False
            truncated = False
            steps = 0
            while not terminated and not truncated:
                raw_action, _ = model.predict(observation, deterministic=True)
                observation, _, terminated, truncated, last_info = (
                    environment.step(raw_action)
                )
                steps += 1
            episode_rows.append(
                {
                    "evaluation_seed": int(evaluation_seed),
                    "episode_index": episode_index,
                    "episode_seed": episode_seed,
                    "steps": steps,
                    "termination_reason": last_info.get(
                        "termination_reason",
                        "timeout",
                    ),
                    "goal_reached": bool(last_info.get("goal_reached", False)),
                    "unsafe_entry": bool(last_info.get("unsafe_entry", False)),
                    "model_contract_breach": bool(
                        last_info.get("model_contract_breach", False)
                    ),
                    "action_contract_breach": bool(
                        last_info.get("action_contract_breach", False)
                    ),
                }
            )
            environment.close()
    successful_steps = [
        int(row["steps"]) for row in episode_rows if row["goal_reached"]
    ]
    total = len(episode_rows)
    hard_failures = sum(
        bool(row["unsafe_entry"])
        or bool(row["model_contract_breach"])
        or bool(row["action_contract_breach"])
        for row in episode_rows
    )
    return {
        "training_seed": int(manifest["seed"]),
        "checkpoint_manifest": str(manifest_path),
        "checkpoint_sha256": str(manifest["checkpoint_sha256"]),
        "episode_count": total,
        "goal_count": len(successful_steps),
        "goal_rate": len(successful_steps) / total,
        "median_successful_steps": (
            float(median(successful_steps)) if successful_steps else None
        ),
        "hard_failure_count": hard_failures,
        "hard_gate_passed": hard_failures == 0,
        "episodes": episode_rows,
    }


def select_sac_checkpoint(
    manifest_paths: Iterable[str | Path],
    *,
    output_path: str | Path,
    device: str = "cpu",
    evaluation_seeds: Sequence[int] = EVALUATION_SEEDS,
    episodes_per_seed: int = 20,
) -> Path:
    """Apply V1 without consulting any PAC or certificate result."""

    paths = sorted(
        (Path(path).resolve() for path in manifest_paths),
        key=lambda path: int(
            json.loads(path.read_text(encoding="utf-8"))["seed"]
        ),
    )
    if len(paths) != len(TRAINING_SEEDS):
        raise ValueError("V1 requires exactly five checkpoint manifests")
    declared_seeds = tuple(
        int(load_checkpoint_manifest(path)["seed"]) for path in paths
    )
    if declared_seeds != TRAINING_SEEDS:
        raise ValueError(
            f"V1 checkpoint seeds must be exactly {TRAINING_SEEDS}"
        )
    evaluations = [
        _evaluate_one_checkpoint(
            path,
            device=device,
            evaluation_seeds=evaluation_seeds,
            episodes_per_seed=episodes_per_seed,
        )
        for path in paths
    ]
    eligible = [
        evaluation
        for evaluation in evaluations
        if evaluation["hard_gate_passed"]
    ]
    selected: dict[str, Any] | None = None
    if eligible:
        selected = min(
            eligible,
            key=lambda evaluation: (
                -float(evaluation["goal_rate"]),
                (
                    float(evaluation["median_successful_steps"])
                    if evaluation["median_successful_steps"] is not None
                    else float("inf")
                ),
                int(evaluation["training_seed"]),
            ),
        )
    payload = {
        "schema_version": 1,
        "kind": "v1_checkpoint_selection",
        "status": "selected" if selected is not None else "not_ready",
        "evaluation_seeds": [int(value) for value in evaluation_seeds],
        "episodes_per_seed": int(episodes_per_seed),
        "ranking": [
            "goal_rate_desc",
            "median_successful_steps_asc",
            "training_seed_asc",
        ],
        "hard_gate": {
            "unsafe_entry": 0,
            "model_contract_breach": 0,
            "action_contract_breach": 0,
        },
        "evaluations": evaluations,
        "selected": selected,
        "forbidden_selection_inputs": [
            "PAC_error",
            "controller_global_validation",
            "q_SBC_feasibility",
            "Gazebo_results",
        ],
    }
    destination = Path(output_path).resolve()
    _write_json(destination, payload)
    return destination


def selected_checkpoint_from_manifest(
    selection_manifest: str | Path,
) -> tuple[Path, dict[str, Any]]:
    selection_path = Path(selection_manifest).resolve()
    payload = json.loads(selection_path.read_text(encoding="utf-8"))
    if payload.get("status") != "selected" or payload.get("selected") is None:
        raise RuntimeError("V1 selection is not_ready")
    checkpoint_manifest = Path(
        payload["selected"]["checkpoint_manifest"]
    ).resolve()
    manifest = load_checkpoint_manifest(checkpoint_manifest)
    return (
        checkpoint_manifest.parent / str(manifest["checkpoint_file"]),
        manifest,
    )


def predict_reference_actions(
    model: Any,
    physical_states: np.ndarray,
    *,
    codec: ReferenceActionCodec = ReferenceActionCodec(),
    batch_size: int = 8192,
) -> np.ndarray:
    values = np.asarray(physical_states, dtype=np.float64)
    if values.ndim != 2 or values.shape[1] != 2:
        raise ValueError("physical_states must have shape (n,2)")
    normalization = CANONICAL_SCENARIO.normalization
    observations = np.empty_like(values, dtype=np.float32)
    observations[:, 0] = (
        values[:, 0] - normalization.center_x_m
    ) / normalization.scale_x_m
    observations[:, 1] = (
        values[:, 1] - normalization.center_y_m
    ) / normalization.scale_y_m
    actions = np.empty_like(values)
    for start in range(0, len(values), batch_size):
        stop = min(start + batch_size, len(values))
        raw_actions, _ = model.predict(
            observations[start:stop],
            deterministic=True,
        )
        raw_batch = np.asarray(raw_actions, dtype=np.float64).reshape(-1, 2)
        for local_index, raw_action in enumerate(raw_batch):
            actions[start + local_index] = codec.decode(raw_action).as_tuple()
    return actions
