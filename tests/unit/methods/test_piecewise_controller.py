from __future__ import annotations

import json

import pytest

from cbf_rover.domain.planar import PlanarState
from cbf_rover.methods.polynomial_sbc.diagnostics import DX1_DECISION_ID
from cbf_rover.methods.polynomial_sbc.piecewise_controller import (
    C2_DECISION_ID,
    build_c2_controller,
)
from cbf_rover.methods.polynomial_sbc.polynomial import content_sha256


def _write_valid_dx1_report(path) -> None:
    payload = {
        "schema_version": 1,
        "kind": "pb1_failure_diagnostic",
        "decision_id": DX1_DECISION_ID,
        "status": "diagnosed",
        "all_dense_degrees_failed_old_pb1_gate": True,
        "selected_checkpoint": {
            "training_seed": 11,
            "checkpoint_sha256": "a" * 64,
        },
        "reference_policy_rollouts": {
            "initial_disk": {
                "episode_count": 2,
                "route_branch_counts": {
                    "lower": 2,
                    "upper": 0,
                    "neither": 0,
                },
            }
        },
    }
    payload["report_content_sha256"] = content_sha256(payload)
    path.write_text(
        json.dumps(payload, sort_keys=True, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )


def test_c2_build_binds_verified_dx1_and_fixed_coefficients(tmp_path) -> None:
    diagnostic = tmp_path / "diagnostic_report.json"
    _write_valid_dx1_report(diagnostic)
    controller = build_c2_controller(diagnostic_report=diagnostic)
    assert controller.construction["decision_ids"][1] == C2_DECISION_ID
    assert [piece.piece_id for piece in controller.pieces] == [
        "lower_entry",
        "lower_transit",
        "goal_capture",
    ]
    at_first_switch = controller.act(PlanarState(0.8, 0.0))
    after_first_switch = controller.act(PlanarState(0.800001, 0.0))
    assert at_first_switch.u_x_mps < after_first_switch.u_x_mps
    assert at_first_switch.u_y_mps < after_first_switch.u_y_mps
    assert controller.construction["controller_contract"][
        "certificate_feedback_used"
    ] is False


def test_c2_build_rejects_a_tampered_dx1_report(tmp_path) -> None:
    diagnostic = tmp_path / "diagnostic_report.json"
    _write_valid_dx1_report(diagnostic)
    payload = json.loads(diagnostic.read_text(encoding="utf-8"))
    payload["selected_checkpoint"]["training_seed"] = 23
    diagnostic.write_text(json.dumps(payload), encoding="utf-8")
    with pytest.raises(ValueError, match="content hash mismatch"):
        build_c2_controller(diagnostic_report=diagnostic)
