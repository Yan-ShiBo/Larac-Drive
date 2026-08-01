from __future__ import annotations

from pathlib import Path

import pytest
import yaml

from cbf_rover.application.scenario_spec import load_scenario_spec


REPOSITORY_ROOT = Path(__file__).resolve().parents[3]
SCENARIO_PATH = (
    REPOSITORY_ROOT / "configs" / "scenarios" / "static_obstacles.yaml"
)


def test_canonical_scenario_resolves_one_shared_gazebo_projection() -> None:
    resolved = load_scenario_spec(SCENARIO_PATH)

    assert resolved.scenario_id == "P1_ROVER_STATIC_OBSTACLES_AABB_V1"
    assert resolved.frame_id == "world"
    assert resolved.total_inflation_radius_m == pytest.approx(0.36)
    assert resolved.gazebo_obstacle_parameters == (
        "box_1,1.5,0,0.34999999999999998,0.34999999999999998,0.80000000000000004",
        "box_2,2.5,0.80000000000000004,0.45000000000000001,0.45000000000000001,1",
        "box_3,2.5,-0.80000000000000004,0.45000000000000001,0.45000000000000001,1",
    )
    assert len(resolved.source_sha256) == 64


def test_scenario_projection_fails_closed_if_geometry_drifts(
    tmp_path: Path,
) -> None:
    document = yaml.safe_load(SCENARIO_PATH.read_text(encoding="utf-8"))
    document["sets"]["unsafe"]["components"][0]["x_bounds_m"][0] = 0.964
    altered = tmp_path / "altered.yaml"
    altered.write_text(
        yaml.safe_dump(document, sort_keys=False),
        encoding="utf-8",
    )

    with pytest.raises(ValueError, match="executable P1/D1 contract"):
        load_scenario_spec(altered)
