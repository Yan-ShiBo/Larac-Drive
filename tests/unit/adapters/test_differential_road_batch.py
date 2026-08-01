from __future__ import annotations

import json
from pathlib import Path

import yaml

from cbf_rover.adapters.exact.differential_road_batch import (
    load_differential_road_batch_contract,
    run_differential_road_batch,
)


REPOSITORY_ROOT = Path(__file__).resolve().parents[3]
BATCH_CONFIG_PATH = (
    REPOSITORY_ROOT
    / "configs"
    / "experiments"
    / "dynamic_road_differential_multiseed.yaml"
)
DIFFERENTIAL_CONFIG_PATH = (
    REPOSITORY_ROOT
    / "configs"
    / "experiments"
    / "dynamic_road_differential_v2.yaml"
)


def test_differential_catalogs_are_frozen_and_nonoverlapping() -> None:
    contract = load_differential_road_batch_contract(BATCH_CONFIG_PATH)
    discovery = contract.catalog("discovery")
    validation = contract.catalog("validation")

    assert discovery.start_inclusive == 7320000
    assert discovery.count == 256
    assert not discovery.eligible_for_final_empirical_claim
    assert validation.start_inclusive == 7321000
    assert validation.count == 1024
    assert validation.eligible_for_final_empirical_claim
    assert not discovery.overlaps(validation)
    assert contract.acceptance.required_lane_pattern_count == 32
    assert (
        contract.differential_config_path
        == DIFFERENTIAL_CONFIG_PATH.resolve()
    )


def test_small_differential_batch_publishes_deterministic_rows(
    tmp_path: Path,
) -> None:
    payload = yaml.safe_load(
        BATCH_CONFIG_PATH.read_text(encoding="utf-8")
    )
    payload["bindings"]["differential_config"] = str(
        DIFFERENTIAL_CONFIG_PATH.resolve()
    )
    payload["seed_catalogs"]["discovery"]["count"] = 2
    payload["seed_catalogs"]["validation"]["start_inclusive"] = 7320100
    payload["seed_catalogs"]["validation"]["count"] = 2
    payload["execution"]["progress_interval_episodes"] = 1
    payload["acceptance"]["required_lane_pattern_count"] = 1
    mini_config = tmp_path / "differential_batch.yaml"
    mini_config.write_text(
        yaml.safe_dump(payload, sort_keys=False),
        encoding="utf-8",
    )

    summary_path = run_differential_road_batch(
        batch_config_path=mini_config,
        catalog_name="discovery",
        output_dir=tmp_path / "result",
    )
    summary = json.loads(summary_path.read_text(encoding="utf-8"))
    metrics_lines = (
        summary_path.parent / "episode_metrics.jsonl"
    ).read_text(encoding="utf-8").splitlines()
    progress = json.loads(
        (summary_path.parent / "progress.json").read_text(
            encoding="utf-8"
        )
    )

    assert summary["status"] == "diagnostic_complete"
    assert summary["metrics"]["episode_count"] == 2
    assert summary["metrics"]["failed_episode_count"] == 0
    assert len(metrics_lines) == 2
    assert [json.loads(line)["seed"] for line in metrics_lines] == [
        7320000,
        7320001,
    ]
    assert progress["status"] == "completed"
