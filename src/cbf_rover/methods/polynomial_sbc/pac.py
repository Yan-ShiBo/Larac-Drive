"""PA1/PS1 sparse L-infinity PAC approximation of one selected SAC policy."""

from __future__ import annotations

from dataclasses import dataclass
import json
from math import ceil, log
from pathlib import Path
from typing import Any, Sequence

import numpy as np
from scipy.optimize import linprog

from cbf_rover.adapters.exact.environment import sample_uniform_psi
from cbf_rover.adapters.training.sac_reference import (
    file_sha256,
    predict_reference_actions,
    selected_checkpoint_from_manifest,
)
from cbf_rover.domain.planar import CANONICAL_SCENARIO

from .polynomial import (
    FrozenPolynomialController,
    Power2D,
    SparsePolynomial2D,
    canonical_total_degree_powers,
    content_sha256,
)


@dataclass(frozen=True, slots=True)
class PACStage:
    index: int
    catalog_degree: int
    max_terms_per_axis: int
    require_axis_at_least_terms: int
    require_degree_exactly: int | None
    search_seed: int
    selection_seed: int
    pac_seed: int
    boundary_seed: int


PAC_STAGES = (
    PACStage(1, 3, 6, 6, None, 21_011, 21_012, 21_013, 21_014),
    PACStage(2, 3, 8, 7, None, 22_011, 22_012, 22_013, 22_014),
    PACStage(3, 4, 8, 8, 4, 23_011, 23_012, 23_013, 23_014),
)
SEARCH_SAMPLE_COUNT = 50_000
SELECTION_SAMPLE_COUNT = 20_000
BEAM_WIDTH = 32
EPSILON_PAC = 0.05
ETA_TOTAL = 0.001
ETA_STAGE = ETA_TOTAL / len(PAC_STAGES)
PAC_MAX_ERROR_MPS = 0.05


@dataclass(frozen=True, slots=True)
class AxisCandidate:
    support: tuple[Power2D, ...]
    coefficients: tuple[float, ...]
    search_linf: float
    search_mean: float
    solver_status: int
    solver_message: str

    @property
    def degree(self) -> int:
        return max(sum(power) for power in self.support)

    def rank_key(self) -> tuple[Any, ...]:
        return (
            self.search_linf,
            self.search_mean,
            self.support,
        )


def _write_json(path: Path, payload: dict[str, Any]) -> None:
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


def _normalized_points(physical_points: np.ndarray) -> np.ndarray:
    normalization = CANONICAL_SCENARIO.normalization
    values = np.asarray(physical_points, dtype=np.float64)
    result = np.empty_like(values)
    result[:, 0] = (
        values[:, 0] - normalization.center_x_m
    ) / normalization.scale_x_m
    result[:, 1] = (
        values[:, 1] - normalization.center_y_m
    ) / normalization.scale_y_m
    return result


def _design_matrix(
    normalized_points: np.ndarray,
    support: Sequence[Power2D],
) -> np.ndarray:
    points = np.asarray(normalized_points, dtype=np.float64)
    return np.column_stack(
        [
            points[:, 0] ** x_power * points[:, 1] ** y_power
            for x_power, y_power in support
        ]
    )


def _fit_axis_linf(
    normalized_points: np.ndarray,
    targets: np.ndarray,
    support: tuple[Power2D, ...],
) -> AxisCandidate | None:
    design = _design_matrix(normalized_points, support)
    count = len(targets)
    objective = np.zeros(len(support) + 1, dtype=np.float64)
    objective[-1] = 1.0
    inequality = np.vstack(
        (
            np.column_stack((design, -np.ones(count))),
            np.column_stack((-design, -np.ones(count))),
        )
    )
    upper = np.concatenate((targets, -targets))
    result = linprog(
        objective,
        A_ub=inequality,
        b_ub=upper,
        bounds=[(None, None)] * len(support) + [(0.0, None)],
        method="highs",
        options={"presolve": True},
    )
    if not result.success or result.x is None or not np.isfinite(result.x).all():
        return None
    coefficients = result.x[:-1]
    errors = np.abs(design @ coefficients - targets)
    return AxisCandidate(
        support=support,
        coefficients=tuple(float(value) for value in coefficients),
        search_linf=float(np.max(errors)),
        search_mean=float(np.mean(errors)),
        solver_status=int(result.status),
        solver_message=str(result.message),
    )


def _search_axis_supports(
    normalized_points: np.ndarray,
    targets: np.ndarray,
    stage: PACStage,
) -> list[AxisCandidate]:
    catalog = canonical_total_degree_powers(stage.catalog_degree)
    initial = _fit_axis_linf(normalized_points, targets, catalog)
    if initial is None:
        return []
    beam = [initial]
    eligible: list[AxisCandidate] = []
    for term_count in range(len(catalog) - 1, stage.require_axis_at_least_terms - 1, -1):
        supports = {
            candidate.support[:index] + candidate.support[index + 1 :]
            for candidate in beam
            for index in range(len(candidate.support))
        }
        fitted = [
            candidate
            for support in sorted(supports)
            if (
                candidate := _fit_axis_linf(
                    normalized_points,
                    targets,
                    support,
                )
            )
            is not None
        ]
        fitted.sort(key=AxisCandidate.rank_key)
        beam = fitted[:BEAM_WIDTH]
        if not beam:
            break
        if term_count <= stage.max_terms_per_axis:
            eligible.extend(beam)
    eligible.sort(key=AxisCandidate.rank_key)
    unique: list[AxisCandidate] = []
    observed: set[tuple[Power2D, ...]] = set()
    for candidate in eligible:
        if candidate.support not in observed:
            observed.add(candidate.support)
            unique.append(candidate)
        if len(unique) == BEAM_WIDTH:
            break
    return unique


def _candidate_predictions(
    candidate: AxisCandidate,
    normalized_points: np.ndarray,
) -> np.ndarray:
    return _design_matrix(normalized_points, candidate.support) @ np.asarray(
        candidate.coefficients,
        dtype=np.float64,
    )


def _select_support_pair(
    x_candidates: Sequence[AxisCandidate],
    y_candidates: Sequence[AxisCandidate],
    normalized_points: np.ndarray,
    targets: np.ndarray,
    stage: PACStage,
) -> tuple[AxisCandidate, AxisCandidate, dict[str, float]] | None:
    x_predictions = [
        _candidate_predictions(candidate, normalized_points)
        for candidate in x_candidates
    ]
    y_predictions = [
        _candidate_predictions(candidate, normalized_points)
        for candidate in y_candidates
    ]
    ranked: list[
        tuple[
            tuple[Any, ...],
            AxisCandidate,
            AxisCandidate,
            dict[str, float],
        ]
    ] = []
    for x_index, x_candidate in enumerate(x_candidates):
        x_error = np.abs(x_predictions[x_index] - targets[:, 0])
        for y_index, y_candidate in enumerate(y_candidates):
            if stage.require_degree_exactly is not None and max(
                x_candidate.degree,
                y_candidate.degree,
            ) != stage.require_degree_exactly:
                continue
            point_error = np.maximum(
                x_error,
                np.abs(y_predictions[y_index] - targets[:, 1]),
            )
            linf_error = float(np.max(point_error))
            if linf_error > PAC_MAX_ERROR_MPS:
                continue
            mean_error = float(np.mean(point_error))
            metrics = {
                "selection_linf_mps": linf_error,
                "selection_mean_mps": mean_error,
            }
            rank = (
                linf_error,
                mean_error,
                len(x_candidate.support) + len(y_candidate.support),
                max(x_candidate.degree, y_candidate.degree),
                x_candidate.support,
                y_candidate.support,
            )
            ranked.append((rank, x_candidate, y_candidate, metrics))
    if not ranked:
        return None
    ranked.sort(key=lambda row: row[0])
    _, x_candidate, y_candidate, metrics = ranked[0]
    return x_candidate, y_candidate, metrics


def _fit_joint_linf(
    normalized_points: np.ndarray,
    targets: np.ndarray,
    support_x: tuple[Power2D, ...],
    support_y: tuple[Power2D, ...],
) -> tuple[np.ndarray, np.ndarray, float, str]:
    design_x = _design_matrix(normalized_points, support_x)
    design_y = _design_matrix(normalized_points, support_y)
    count = len(normalized_points)
    x_terms = len(support_x)
    y_terms = len(support_y)
    variables = x_terms + y_terms + 1
    objective = np.zeros(variables, dtype=np.float64)
    objective[-1] = 1.0
    zero_x = np.zeros((count, x_terms), dtype=np.float64)
    zero_y = np.zeros((count, y_terms), dtype=np.float64)
    ones = -np.ones((count, 1), dtype=np.float64)
    inequality = np.vstack(
        (
            np.hstack((design_x, zero_y, ones)),
            np.hstack((-design_x, zero_y, ones)),
            np.hstack((zero_x, design_y, ones)),
            np.hstack((zero_x, -design_y, ones)),
        )
    )
    upper = np.concatenate(
        (
            targets[:, 0],
            -targets[:, 0],
            targets[:, 1],
            -targets[:, 1],
        )
    )
    result = linprog(
        objective,
        A_ub=inequality,
        b_ub=upper,
        bounds=[(None, None)] * (variables - 1) + [(0.0, None)],
        method="highs",
        options={"presolve": True},
    )
    if not result.success or result.x is None or not np.isfinite(result.x).all():
        raise RuntimeError(f"PAC refit failed: {result.status} {result.message}")
    return (
        result.x[:x_terms],
        result.x[x_terms : x_terms + y_terms],
        float(result.x[-1]),
        str(result.message),
    )


def _save_dataset(
    path: Path,
    *,
    states: np.ndarray,
    actions: np.ndarray,
    purpose: str,
    seed: int,
) -> dict[str, Any]:
    path.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(
        path,
        physical_states_m=np.asarray(states, dtype=np.float64),
        reference_actions_mps=np.asarray(actions, dtype=np.float64),
    )
    return {
        "file": path.name,
        "sha256": file_sha256(path),
        "purpose": purpose,
        "seed": int(seed),
        "accepted_point_order": "rejection_sampling_acceptance_order",
        "sample_count": int(len(states)),
        "distribution": "Uniform(Psi)",
    }


def _boundary_stress_points(
    rng: np.random.Generator,
    count: int = 12_000,
) -> np.ndarray:
    scenario = CANONICAL_SCENARIO
    points: list[tuple[float, float]] = []
    edge_count = max(16, count // 24)
    for box in (scenario.domain, *scenario.unsafe_components):
        x_values = np.linspace(box.x_min, box.x_max, edge_count)
        y_values = np.linspace(box.y_min, box.y_max, edge_count)
        points.extend((float(x), box.y_min) for x in x_values)
        points.extend((float(x), box.y_max) for x in x_values)
        points.extend((box.x_min, float(y)) for y in y_values)
        points.extend((box.x_max, float(y)) for y in y_values)
    angles = np.linspace(-np.pi, np.pi, 4 * edge_count, endpoint=False)
    for disk in (scenario.initial, scenario.goal):
        points.extend(
            (
                disk.center_x + disk.radius * float(np.cos(angle)),
                disk.center_y + disk.radius * float(np.sin(angle)),
            )
            for angle in angles
        )
    array = np.asarray(points, dtype=np.float64)
    rng.shuffle(array)
    return array


def fit_pac_stage(
    *,
    selection_manifest: str | Path,
    stage_index: int,
    output_dir: str | Path,
    device: str = "cpu",
    search_sample_count: int = SEARCH_SAMPLE_COUNT,
    selection_sample_count: int = SELECTION_SAMPLE_COUNT,
) -> Path:
    """Run one irreversible PA1 stage and emit a candidate or ``not_ready``."""

    if stage_index < 1 or stage_index > len(PAC_STAGES):
        raise ValueError("stage_index must be 1, 2, or 3")
    stage = PAC_STAGES[stage_index - 1]
    destination = Path(output_dir).resolve()
    destination.mkdir(parents=True, exist_ok=False)
    checkpoint, checkpoint_manifest = selected_checkpoint_from_manifest(
        selection_manifest
    )
    from stable_baselines3 import SAC

    model = SAC.load(checkpoint, device=device)
    datasets: list[dict[str, Any]] = []

    search_rng = np.random.default_rng(stage.search_seed)
    search_states = sample_uniform_psi(search_rng, int(search_sample_count))
    search_actions = predict_reference_actions(model, search_states)
    datasets.append(
        _save_dataset(
            destination / "support_search.npz",
            states=search_states,
            actions=search_actions,
            purpose="support_search",
            seed=stage.search_seed,
        )
    )
    normalized_search = _normalized_points(search_states)
    x_candidates = _search_axis_supports(
        normalized_search,
        search_actions[:, 0],
        stage,
    )
    y_candidates = _search_axis_supports(
        normalized_search,
        search_actions[:, 1],
        stage,
    )

    selection_rng = np.random.default_rng(stage.selection_seed)
    selection_states = sample_uniform_psi(
        selection_rng,
        int(selection_sample_count),
    )
    selection_actions = predict_reference_actions(model, selection_states)
    datasets.append(
        _save_dataset(
            destination / "support_selection.npz",
            states=selection_states,
            actions=selection_actions,
            purpose="support_selection",
            seed=stage.selection_seed,
        )
    )
    selected_pair = _select_support_pair(
        x_candidates,
        y_candidates,
        _normalized_points(selection_states),
        selection_actions,
        stage,
    )
    base_manifest: dict[str, Any] = {
        "schema_version": 1,
        "kind": "pa1_ps1_stage",
        "stage": stage.index,
        "stage_contract": {
            "catalog_degree": stage.catalog_degree,
            "max_terms_per_axis": stage.max_terms_per_axis,
            "require_axis_at_least_terms": stage.require_axis_at_least_terms,
            "require_degree_exactly": stage.require_degree_exactly,
            "beam_width": BEAM_WIDTH,
            "search_sample_count": int(search_sample_count),
            "selection_sample_count": int(selection_sample_count),
            "selection_hard_linf_mps": PAC_MAX_ERROR_MPS,
            "epsilon_pac": EPSILON_PAC,
            "eta_stage": ETA_STAGE,
            "eta_total": ETA_TOTAL,
        },
        "selected_checkpoint_sha256": checkpoint_manifest["checkpoint_sha256"],
        "datasets": datasets,
        "search_candidate_counts": {
            "u_x": len(x_candidates),
            "u_y": len(y_candidates),
        },
        "solver": {
            "name": "scipy.optimize.linprog",
            "method": "highs",
            "objective": "deterministic_L_infinity",
        },
    }
    if selected_pair is None:
        base_manifest["status"] = "not_ready"
        base_manifest["failure_reason"] = "no_support_pair_passed_selection_gate"
        manifest_path = destination / "manifest.json"
        _write_json(manifest_path, base_manifest)
        return manifest_path

    selected_x, selected_y, selection_metrics = selected_pair
    kappa = 1 + len(selected_x.support) + len(selected_y.support)
    pac_count = max(
        5000,
        ceil((2.0 / EPSILON_PAC) * (log(1.0 / ETA_STAGE) + kappa)),
    )
    pac_rng = np.random.default_rng(stage.pac_seed)
    pac_states = sample_uniform_psi(pac_rng, pac_count)
    pac_actions = predict_reference_actions(model, pac_states)
    datasets.append(
        _save_dataset(
            destination / "pac_refit.npz",
            states=pac_states,
            actions=pac_actions,
            purpose="pac_refit",
            seed=stage.pac_seed,
        )
    )
    coefficients_x, coefficients_y, e_star, solver_message = _fit_joint_linf(
        _normalized_points(pac_states),
        pac_actions,
        selected_x.support,
        selected_y.support,
    )
    controller = FrozenPolynomialController(
        controller_id=(
            f"PB1_STAGE_{stage.index}_"
            f"SAC_SEED_{int(checkpoint_manifest['seed'])}"
        ),
        u_x=SparsePolynomial2D(
            selected_x.support,
            tuple(float(value) for value in coefficients_x),
        ),
        u_y=SparsePolynomial2D(
            selected_y.support,
            tuple(float(value) for value in coefficients_y),
        ),
        upstream_checkpoint_sha256=str(
            checkpoint_manifest["checkpoint_sha256"]
        ),
        construction={
            "decision_ids": [
                "K2_SAC_REFERENCE_TO_PAC_POLYNOMIAL",
                "T2_SYMMETRIC_UNBIASED_SAC",
                "V1_SELECT_ONE_SAC_BEFORE_PAC",
                "PB1_NORMALIZED_SPARSE_TOTAL_DEGREE",
                "PA1_INDEPENDENT_UNIFORM_PSI_THREE_WAY",
                "PS1_DETERMINISTIC_LINF_BEAM32",
            ],
            "stage": stage.index,
            "pac_e_star_mps": e_star,
            "pac_epsilon": EPSILON_PAC,
            "pac_eta_stage": ETA_STAGE,
            "pac_eta_total": ETA_TOTAL,
            "pac_sample_count": pac_count,
            "selection_metrics": selection_metrics,
        },
    )
    controller_path = destination / "controller_candidate.json"
    controller.save(controller_path)

    boundary_rng = np.random.default_rng(stage.boundary_seed)
    boundary_states = _boundary_stress_points(boundary_rng)
    boundary_actions = predict_reference_actions(model, boundary_states)
    normalized_boundary = _normalized_points(boundary_states)
    controller_boundary = np.column_stack(
        (
            controller.u_x.evaluate_many(normalized_boundary),
            controller.u_y.evaluate_many(normalized_boundary),
        )
    )
    boundary_error = np.max(
        np.abs(controller_boundary - boundary_actions),
        axis=1,
    )
    datasets.append(
        _save_dataset(
            destination / "boundary_stress.npz",
            states=boundary_states,
            actions=boundary_actions,
            purpose="boundary_stress_diagnostic_only",
            seed=stage.boundary_seed,
        )
    )
    base_manifest.update(
        {
            "status": "candidate" if e_star <= PAC_MAX_ERROR_MPS else "not_ready",
            "failure_reason": (
                None
                if e_star <= PAC_MAX_ERROR_MPS
                else "pac_refit_error_exceeded_gate"
            ),
            "selected_support": {
                "u_x": [list(power) for power in selected_x.support],
                "u_y": [list(power) for power in selected_y.support],
                **selection_metrics,
            },
            "pac": {
                "kappa": kappa,
                "sample_count": pac_count,
                "e_star_mps": e_star,
                "hard_gate_mps": PAC_MAX_ERROR_MPS,
                "solver_message": solver_message,
                "claim": (
                    "With confidence at least 1-eta_total over the declared "
                    "independent PAC draws, the Uniform(Psi) probability that "
                    "the componentwise error exceeds e_star is at most "
                    "epsilon_pac."
                ),
                "not_a_safety_claim": True,
            },
            "controller_candidate": {
                "file": controller_path.name,
                "sha256": file_sha256(controller_path),
                "content_sha256": controller.sha256,
            },
            "boundary_stress_diagnostic": {
                "sample_count": int(len(boundary_states)),
                "linf_mps": float(np.max(boundary_error)),
                "mean_mps": float(np.mean(boundary_error)),
                "used_for_selection": False,
                "used_for_claim": False,
            },
            "datasets": datasets,
        }
    )
    base_manifest["manifest_content_sha256"] = content_sha256(base_manifest)
    manifest_path = destination / "manifest.json"
    _write_json(manifest_path, base_manifest)
    return manifest_path
