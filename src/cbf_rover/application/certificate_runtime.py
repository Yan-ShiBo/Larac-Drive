"""Pure construction of audit-ready live q-SBC telemetry."""

from __future__ import annotations

import json
from math import isfinite
from typing import Any

from cbf_rover.domain.planar import PlanarAction, PlanarState
from cbf_rover.methods.polynomial_sbc.certificate import QSBCertificate


def _finite(value: float, name: str) -> float:
    result = float(value)
    if not isfinite(result):
        raise ValueError(f"{name} must be finite")
    return result


def build_qsbc_runtime_facts(
    *,
    certificate: QSBCertificate,
    state: PlanarState,
    nominal_action: PlanarAction,
    stamp_s: float,
    yaw_rad: float,
    command_linear_mps: float,
    command_angular_radps: float,
    tracker_saturation_applied: bool,
    geometry_terminal_reason: str | None,
) -> dict[str, Any]:
    """Build factual monitor-only telemetry without changing any action."""

    validation_status = str(
        certificate.validation.get("status", "unknown")
    )
    if validation_status != "validated":
        raise ValueError(
            "runtime refuses a certificate without independent validation"
        )
    first_slice_ready = bool(
        certificate.validation.get("first_slice_ready", False)
    )
    barrier_value = certificate.evaluate(state)
    if not isfinite(barrier_value):
        raise ValueError("certificate evaluation produced a non-finite value")
    level_gap = 1.0 - barrier_value
    level_relation = (
        "below_unsafe_set_certificate_level"
        if level_gap > 0.0
        else "at_or_above_unsafe_set_certificate_level"
    )
    return {
        "schema_version": 1,
        "kind": "q_sbc_runtime_facts",
        "mode": "monitor_only",
        "runtime_claim_level": "empirical",
        "stamp_s": _finite(stamp_s, "stamp_s"),
        "state": {
            "frame_id": "world",
            "x_m": state.x_m,
            "y_m": state.y_m,
            "yaw_rad": _finite(yaw_rad, "yaw_rad"),
            "geometry_terminal_reason": geometry_terminal_reason,
        },
        "certificate": {
            "certificate_id": certificate.certificate_id,
            "content_sha256": certificate.sha256,
            "controller_content_sha256": (
                certificate.controller_content_sha256
            ),
            "theorem_id": certificate.theorem_id,
            "independent_validation_status": validation_status,
            "first_slice_ready": first_slice_ready,
            "barrier_value_W": barrier_value,
            "unsafe_set_certificate_level": 1.0,
            "unsafe_set_certificate_level_gap": level_gap,
            "certificate_level_relation": level_relation,
            "level_relation_is_not_state_classification": True,
            "q": certificate.q,
            "epsbar": certificate.epsbar,
            "unsafe_before_goal_probability_upper_bound": certificate.q,
            "goal_before_unsafe_probability_lower_bound": (
                certificate.reach_avoid_lower_bound
            ),
            "q_is_not_an_instantaneous_probability": True,
        },
        "control": {
            "nominal_single_integrator_action": {
                "u_x_mps": nominal_action.u_x_mps,
                "u_y_mps": nominal_action.u_y_mps,
            },
            "differential_drive_command": {
                "linear_mps": _finite(
                    command_linear_mps,
                    "command_linear_mps",
                ),
                "angular_radps": _finite(
                    command_angular_radps,
                    "command_angular_radps",
                ),
                "tracker_saturation_applied": bool(
                    tracker_saturation_applied
                ),
                "claim_level": "empirical",
            },
            "certificate_modified_action": False,
            "executed_action_available": False,
        },
    }


def runtime_facts_json(payload: dict[str, Any]) -> str:
    """Serialize one live fact record deterministically for ROS String."""

    return json.dumps(
        payload,
        sort_keys=True,
        separators=(",", ":"),
        ensure_ascii=True,
        allow_nan=False,
    )
