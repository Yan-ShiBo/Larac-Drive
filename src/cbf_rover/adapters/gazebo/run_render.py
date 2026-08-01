"""Render a verified Gazebo measured-state trace at simulated real time."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any, Mapping

from cbf_rover.adapters.exact.differential_road_render import (
    render_differential_road_animation,
)
from cbf_rover.adapters.exact.road_contract import (
    canonical_content_sha256,
    file_sha256,
    generate_seeded_road_scenario,
    obstacle_plan_payload,
)

from .differential_road_contract import GazeboDifferentialRoadContract


def _load_json_object(path: Path) -> dict[str, Any]:
    payload = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise ValueError(f"expected a JSON object: {path}")
    return payload


def _manifest_content_hash_valid(payload: Mapping[str, Any]) -> bool:
    expected = payload.get("manifest_content_sha256")
    content = dict(payload)
    content.pop("manifest_content_sha256", None)
    return isinstance(expected, str) and canonical_content_sha256(content) == expected


def _load_trace(path: Path) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    with path.open("r", encoding="utf-8") as stream:
        for line in stream:
            if not line.strip():
                continue
            payload = json.loads(line)
            if not isinstance(payload, dict):
                raise ValueError(f"trace row is not a JSON object: {path}")
            rows.append(payload)
    if not rows:
        raise ValueError(f"Gazebo trace is empty: {path}")
    return rows


def render_gazebo_differential_road_run(
    *,
    contract: GazeboDifferentialRoadContract,
    run_manifest_path: str | Path,
    output_dir: str | Path,
    display_stride: int | None = None,
) -> Path:
    """Verify a physical run and render its certificate playback."""

    source_manifest_path = Path(run_manifest_path).resolve()
    source_manifest = _load_json_object(source_manifest_path)
    if source_manifest.get("kind") != "dynamic_road_gazebo_classic_cbf_run":
        raise ValueError("unexpected Gazebo source run kind")
    if not _manifest_content_hash_valid(source_manifest):
        raise ValueError("Gazebo source run manifest content hash mismatch")
    if source_manifest["experiment_id"] != contract.experiment_id:
        raise ValueError("Gazebo source run experiment id does not match config")
    if (
        source_manifest["contract"]["config_file_sha256"]
        != contract.config_file_sha256
    ):
        raise ValueError("Gazebo source run config hash does not match config")

    source_directory = source_manifest_path.parent
    trace_path = (
        source_directory / source_manifest["telemetry"]["trace_file"]
    ).resolve()
    if trace_path.parent != source_directory:
        raise ValueError("Gazebo source trace must stay inside its run directory")
    if (
        file_sha256(trace_path)
        != source_manifest["telemetry"]["trace_file_sha256"]
    ):
        raise ValueError("Gazebo source trace file hash mismatch")
    rows = _load_trace(trace_path)
    if rows[-1]["termination_reason"] != source_manifest["termination_reason"]:
        raise ValueError("Gazebo trace termination does not match run manifest")

    seed = int(source_manifest["scenario"]["seed"])
    scenario = generate_seeded_road_scenario(contract.base.base, seed=seed)
    plan = obstacle_plan_payload(scenario=scenario, seed=seed)
    if (
        plan["content_sha256"]
        != source_manifest["scenario"]["obstacle_plan_content_sha256"]
    ):
        raise ValueError("Gazebo scenario projection does not match source run")

    stride = (
        contract.base.display_stride
        if display_stride is None
        else int(display_stride)
    )
    if stride <= 0:
        raise ValueError("display_stride must be positive")
    destination = Path(output_dir).resolve()
    destination.mkdir(parents=True, exist_ok=False)
    animation_path = destination / "gazebo_classic_cbf_1x.gif"
    final_frame_path = destination / "gazebo_classic_cbf_final.png"
    render_differential_road_animation(
        rows,
        scenario=scenario,
        gif_path=animation_path,
        final_frame_path=final_frame_path,
        dt_s=contract.base.system.dt_s,
        display_stride=stride,
        title_backend="GAZEBO CLASSIC 11 + DIFFERENTIAL DRIVE",
        title_claim="EMPIRICAL / NOT q-SBC",
        footer_backend="GAZEBO CLASSIC 11 ODE / MEASURED-STATE CBF",
    )
    report: dict[str, Any] = {
        "schema_version": 1,
        "kind": "dynamic_road_gazebo_classic_cbf_render",
        "experiment_id": contract.experiment_id,
        "status": "rendered",
        "claim_boundary": {
            "playback": (
                "one_times_simulated_time_from_measured_gazebo_trace"
            ),
            "online_distance_cbf": (
                "displayed_from_recorded_measured_state_evaluation"
            ),
            "q_sbc_probability_bound": "not_claimed",
            "ros2_result": "not_claimed",
            "real_vehicle_claim": "forbidden",
        },
        "source": {
            "run_manifest": str(source_manifest_path),
            "run_manifest_file_sha256": file_sha256(source_manifest_path),
            "run_manifest_content_sha256": source_manifest[
                "manifest_content_sha256"
            ],
            "trace_file": str(trace_path),
            "trace_file_sha256": file_sha256(trace_path),
            "config_path": str(contract.config_path),
            "config_file_sha256": contract.config_file_sha256,
            "scenario_seed": seed,
        },
        "render": {
            "playback_rate": "1x_simulated_time",
            "frame_count": len(range(0, len(rows), stride))
            + (1 if (len(rows) - 1) % stride else 0),
            "display_stride": stride,
            "simulated_frame_interval_s": (
                contract.base.system.dt_s * stride
            ),
            "gif_file": animation_path.name,
            "gif_file_sha256": file_sha256(animation_path),
            "final_frame_file": final_frame_path.name,
            "final_frame_file_sha256": file_sha256(final_frame_path),
        },
    }
    report["report_content_sha256"] = canonical_content_sha256(report)
    report_path = destination / "render_manifest.json"
    temporary = report_path.with_name(f".{report_path.name}.tmp")
    temporary.write_text(
        json.dumps(
            report,
            indent=2,
            sort_keys=True,
            ensure_ascii=False,
            allow_nan=False,
        )
        + "\n",
        encoding="utf-8",
    )
    temporary.replace(report_path)
    return report_path
