"""C2 geometry-driven lower-route piecewise polynomial controller."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any, Mapping

from cbf_rover.adapters.training.sac_reference import file_sha256

from .diagnostics import DX1_DECISION_ID
from .polynomial import (
    FrozenPiecewisePolynomialController,
    PolynomialControllerPiece,
    SparsePolynomial2D,
    content_sha256,
)


C2_DECISION_ID = "C2_FIXED_LOWER_PIECEWISE_POLYNOMIAL_V1"
C2_VALIDATION_ID = "C2V1_PIECEWISE_BERNSTEIN_AND_1000_ROLLOUTS"

C2_CONTRACT: dict[str, Any] = {
    "decision_id": C2_DECISION_ID,
    "route": "lower",
    "partition": {
        "variable": "x_m",
        "breaks_m": [-1.0, 0.8, 3.2, 4.0],
        "runtime_boundary_semantics": "ordered_closed_first_match",
        "formal_boundary_semantics": "validate_both_adjacent_closures",
    },
    "pieces": [
        {
            "piece_id": "lower_entry",
            "physical_x_interval_m": [-1.0, 0.8],
            "u_x": {
                "formula": "-0.05 - 0.25*xi_x",
                "powers": [[0, 0], [1, 0]],
                "coefficients": [-0.05, -0.25],
            },
            "u_y": {
                "formula": "-0.28 - 0.11*xi_y + 0.28*xi_y^2",
                "powers": [[0, 0], [0, 1], [0, 2]],
                "coefficients": [-0.28, -0.11, 0.28],
            },
        },
        {
            "piece_id": "lower_transit",
            "physical_x_interval_m": [0.8, 3.2],
            "u_x": {
                "formula": "0.18 - 0.25*xi_x",
                "powers": [[0, 0], [1, 0]],
                "coefficients": [0.18, -0.25],
            },
            "u_y": {
                "formula": "-0.26 - 0.11*xi_y + 0.26*xi_y^2",
                "powers": [[0, 0], [0, 1], [0, 2]],
                "coefficients": [-0.26, -0.11, 0.26],
            },
        },
        {
            "piece_id": "goal_capture",
            "physical_x_interval_m": [3.2, 4.0],
            "u_x": {
                "formula": "0.42 - 0.525*xi_x",
                "powers": [[0, 0], [1, 0]],
                "coefficients": [0.42, -0.525],
            },
            "u_y": {
                "formula": "-0.20*xi_y",
                "powers": [[0, 1]],
                "coefficients": [-0.20],
            },
        },
    ],
    "fixed_semantics": {
        "state_order": ["x_m", "y_m"],
        "action_order": ["u_x_mps", "u_y_mps"],
        "frame_id": "world",
        "runtime_mode": "monitor_only",
        "post_controller_clipping_allowed": False,
    },
    "validation": {
        "decision_id": C2_VALIDATION_ID,
        "action_disk_margin": 1.0e-6,
        "robust_D1_margin_m": 1.0e-6,
        "rollout_seeds": 10,
        "episodes_per_seed": 100,
        "required_goal_rate": 0.99,
        "required_hard_failures": 0,
    },
    "certificate_feedback_used": False,
}


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


def _verified_dx1_report(path: Path) -> tuple[dict[str, Any], str]:
    payload = json.loads(path.read_text(encoding="utf-8"))
    declared_hash = str(payload.pop("report_content_sha256", "")).lower()
    if content_sha256(payload) != declared_hash:
        raise ValueError("DX1 report content hash mismatch")
    if payload.get("decision_id") != DX1_DECISION_ID:
        raise ValueError("unexpected diagnostic decision id")
    if payload.get("status") != "diagnosed":
        raise ValueError("DX1 report is not complete")
    if not bool(payload.get("all_dense_degrees_failed_old_pb1_gate")):
        raise ValueError("C2 requires the recorded dense PB1 failure")
    checkpoint = payload["selected_checkpoint"]
    if int(checkpoint["training_seed"]) != 11:
        raise ValueError("C2 lower-route evidence must bind V1 seed 11")
    initial = payload["reference_policy_rollouts"]["initial_disk"]
    if int(initial["route_branch_counts"]["lower"]) != int(
        initial["episode_count"]
    ):
        raise ValueError("DX1 initial rollouts did not unanimously use lower route")
    return payload, declared_hash


def _polynomial(payload: Mapping[str, Any]) -> SparsePolynomial2D:
    return SparsePolynomial2D(
        powers=tuple(
            tuple(int(value) for value in power)
            for power in payload["powers"]
        ),
        coefficients=tuple(float(value) for value in payload["coefficients"]),
    )


def build_c2_controller(
    *,
    diagnostic_report: str | Path,
) -> FrozenPiecewisePolynomialController:
    """Construct the fixed C2 coefficients after validating DX1 provenance."""

    diagnostic_path = Path(diagnostic_report).resolve()
    _, diagnostic_content_hash = _verified_dx1_report(diagnostic_path)
    construction_source = {
        "controller_contract": C2_CONTRACT,
        "dx1_report_content_sha256": diagnostic_content_hash,
        "dx1_report_file_sha256": file_sha256(diagnostic_path),
    }
    pieces = tuple(
        PolynomialControllerPiece(
            piece_id=str(piece["piece_id"]),
            x_min_m=float(piece["physical_x_interval_m"][0]),
            x_max_m=float(piece["physical_x_interval_m"][1]),
            u_x=_polynomial(piece["u_x"]),
            u_y=_polynomial(piece["u_y"]),
        )
        for piece in C2_CONTRACT["pieces"]
    )
    return FrozenPiecewisePolynomialController(
        controller_id="CBF_ROVER_C2_LOWER_ROUTE_V1",
        pieces=pieces,
        construction_source_sha256=content_sha256(construction_source),
        construction={
            "decision_ids": [
                DX1_DECISION_ID,
                C2_DECISION_ID,
                C2_VALIDATION_ID,
                "C1_FREEZE_CONTROLLER_THEN_CERTIFY",
            ],
            "diagnostic_report": {
                "file": diagnostic_path.name,
                "file_sha256": file_sha256(diagnostic_path),
                "content_sha256": diagnostic_content_hash,
            },
            "controller_contract": C2_CONTRACT,
            "claim_level": "candidate_pending_C2V1",
        },
    )


def publish_c2_candidate(
    *,
    diagnostic_report: str | Path,
    output_dir: str | Path,
) -> Path:
    """Publish the deterministic candidate and a hash-bound build manifest."""

    destination = Path(output_dir).resolve()
    destination.mkdir(parents=True, exist_ok=False)
    controller = build_c2_controller(diagnostic_report=diagnostic_report)
    candidate_path = destination / "controller_candidate.json"
    controller.save(candidate_path)
    manifest = {
        "schema_version": 1,
        "kind": "c2_piecewise_controller_build",
        "decision_id": C2_DECISION_ID,
        "status": "candidate",
        "claim_level": "pending_formal_controller_validation",
        "controller_candidate": {
            "file": candidate_path.name,
            "file_sha256": file_sha256(candidate_path),
            "content_sha256": controller.sha256,
        },
        "construction_source_sha256": controller.construction_source_sha256,
        "diagnostic_report": controller.construction["diagnostic_report"],
        "contract": C2_CONTRACT,
        "certificate_feedback_used": False,
    }
    manifest["manifest_content_sha256"] = content_sha256(manifest)
    manifest_path = destination / "manifest.json"
    _write_json(manifest_path, manifest)
    return manifest_path
