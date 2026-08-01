"""Resolve the canonical ScenarioSpec for exact, Gazebo, and RViz consumers.

The YAML document remains the source of truth.  This module deliberately checks
that its mathematical sets still match the executable P1/D1 contract before it
exposes any simulator projection.
"""

from __future__ import annotations

from dataclasses import dataclass
from hashlib import sha256
from math import isclose, isfinite
from pathlib import Path
from typing import Any, Mapping, Sequence

import yaml

from cbf_rover.domain.planar import CANONICAL_SCENARIO


_ABS_TOL = 1.0e-12


def _mapping(value: Any, name: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise ValueError(f"{name} must be a mapping")
    return value


def _sequence(value: Any, name: str, length: int) -> tuple[Any, ...]:
    if not isinstance(value, Sequence) or isinstance(value, (str, bytes)):
        raise ValueError(f"{name} must be a sequence")
    result = tuple(value)
    if len(result) != length:
        raise ValueError(f"{name} must contain exactly {length} entries")
    return result


def _finite_tuple(value: Any, name: str, length: int) -> tuple[float, ...]:
    result = tuple(float(item) for item in _sequence(value, name, length))
    if not all(isfinite(item) for item in result):
        raise ValueError(f"{name} must contain only finite numbers")
    return result


def _expect_close(actual: float, expected: float, name: str) -> None:
    if not isclose(actual, expected, rel_tol=0.0, abs_tol=_ABS_TOL):
        raise ValueError(
            f"{name} disagrees with the executable P1/D1 contract: "
            f"{actual!r} != {expected!r}"
        )


@dataclass(frozen=True, slots=True)
class PhysicalBoxProjection:
    """One physical Gazebo obstacle resolved from the ScenarioSpec."""

    obstacle_id: str
    center_m: tuple[float, float, float]
    size_m: tuple[float, float, float]

    def __post_init__(self) -> None:
        if not self.obstacle_id.strip():
            raise ValueError("obstacle_id must not be empty")
        if any(size <= 0.0 for size in self.size_m):
            raise ValueError(f"{self.obstacle_id} sizes must be positive")
        _expect_close(
            self.center_m[2],
            self.size_m[2] / 2.0,
            f"{self.obstacle_id}.center_z",
        )

    def gazebo_parameter(self) -> str:
        """Return the obstacle-spawner's stable six-field representation."""

        values = (
            self.obstacle_id,
            self.center_m[0],
            self.center_m[1],
            self.size_m[0],
            self.size_m[1],
            self.size_m[2],
        )
        return ",".join(
            str(value) if isinstance(value, str) else format(value, ".17g")
            for value in values
        )


@dataclass(frozen=True, slots=True)
class ResolvedScenarioSpec:
    """Validated ScenarioSpec plus its physical simulator projection."""

    scenario_id: str
    frame_id: str
    source_path: Path
    source_sha256: str
    physical_obstacles: tuple[PhysicalBoxProjection, ...]
    total_inflation_radius_m: float

    @property
    def gazebo_obstacle_parameters(self) -> tuple[str, ...]:
        return tuple(
            obstacle.gazebo_parameter()
            for obstacle in self.physical_obstacles
        )

    def snapshot(self) -> dict[str, Any]:
        return {
            "schema_version": 1,
            "kind": "resolved_scenario_spec",
            "scenario_id": self.scenario_id,
            "frame_id": self.frame_id,
            "source_path": str(self.source_path),
            "source_sha256": self.source_sha256,
            "total_inflation_radius_m": self.total_inflation_radius_m,
            "physical_obstacles": [
                {
                    "id": obstacle.obstacle_id,
                    "center_m": list(obstacle.center_m),
                    "size_m": list(obstacle.size_m),
                    "gazebo_parameter": obstacle.gazebo_parameter(),
                }
                for obstacle in self.physical_obstacles
            ],
        }


def _validate_math_sets(document: Mapping[str, Any]) -> None:
    domain = _mapping(document.get("domain"), "domain")
    x_bounds = _finite_tuple(domain.get("x_bounds_m"), "domain.x_bounds_m", 2)
    y_bounds = _finite_tuple(domain.get("y_bounds_m"), "domain.y_bounds_m", 2)
    expected_domain = CANONICAL_SCENARIO.domain
    for name, actual, expected in (
        ("domain.x_min", x_bounds[0], expected_domain.x_min),
        ("domain.x_max", x_bounds[1], expected_domain.x_max),
        ("domain.y_min", y_bounds[0], expected_domain.y_min),
        ("domain.y_max", y_bounds[1], expected_domain.y_max),
    ):
        _expect_close(actual, expected, name)

    sets = _mapping(document.get("sets"), "sets")
    for set_name, expected_disk in (
        ("initial", CANONICAL_SCENARIO.initial),
        ("goal", CANONICAL_SCENARIO.goal),
    ):
        payload = _mapping(sets.get(set_name), f"sets.{set_name}")
        center = _finite_tuple(
            payload.get("center_m"),
            f"sets.{set_name}.center_m",
            2,
        )
        radius = float(payload.get("radius_m"))
        _expect_close(center[0], expected_disk.center_x, f"{set_name}.center_x")
        _expect_close(center[1], expected_disk.center_y, f"{set_name}.center_y")
        _expect_close(radius, expected_disk.radius, f"{set_name}.radius")

    unsafe = _mapping(sets.get("unsafe"), "sets.unsafe")
    components = _sequence(
        unsafe.get("components"),
        "sets.unsafe.components",
        len(CANONICAL_SCENARIO.unsafe_components),
    )
    for index, (payload_value, expected_box) in enumerate(
        zip(components, CANONICAL_SCENARIO.unsafe_components, strict=True)
    ):
        payload = _mapping(payload_value, f"unsafe[{index}]")
        x_bounds = _finite_tuple(
            payload.get("x_bounds_m"),
            f"unsafe[{index}].x_bounds_m",
            2,
        )
        y_bounds = _finite_tuple(
            payload.get("y_bounds_m"),
            f"unsafe[{index}].y_bounds_m",
            2,
        )
        for name, actual, expected in (
            ("x_min", x_bounds[0], expected_box.x_min),
            ("x_max", x_bounds[1], expected_box.x_max),
            ("y_min", y_bounds[0], expected_box.y_min),
            ("y_max", y_bounds[1], expected_box.y_max),
        ):
            _expect_close(actual, expected, f"unsafe[{index}].{name}")


def _resolve_projection(
    document: Mapping[str, Any],
) -> tuple[tuple[PhysicalBoxProjection, ...], float]:
    projection = _mapping(document.get("projection"), "projection")
    rover_radius = float(projection.get("rover_circumscribed_radius_m"))
    extra_margin = float(projection.get("additional_safety_margin_m"))
    inflation = float(projection.get("total_inflation_radius_m"))
    if not all(isfinite(value) and value >= 0.0 for value in (
        rover_radius,
        extra_margin,
        inflation,
    )):
        raise ValueError("projection radii must be finite and nonnegative")
    _expect_close(
        rover_radius + extra_margin,
        inflation,
        "projection.total_inflation_radius_m",
    )

    source_values = _sequence(
        projection.get("physical_obstacle_source"),
        "projection.physical_obstacle_source",
        len(CANONICAL_SCENARIO.unsafe_components),
    )
    obstacles: list[PhysicalBoxProjection] = []
    for index, (source_value, unsafe_box) in enumerate(
        zip(
            source_values,
            CANONICAL_SCENARIO.unsafe_components,
            strict=True,
        )
    ):
        source = _mapping(
            source_value,
            f"projection.physical_obstacle_source[{index}]",
        )
        obstacle = PhysicalBoxProjection(
            obstacle_id=str(source.get("id", "")),
            center_m=_finite_tuple(
                source.get("center_m"),
                f"physical_obstacle_source[{index}].center_m",
                3,
            ),
            size_m=_finite_tuple(
                source.get("size_m"),
                f"physical_obstacle_source[{index}].size_m",
                3,
            ),
        )
        expected_bounds = (
            obstacle.center_m[0] - obstacle.size_m[0] / 2.0 - inflation,
            obstacle.center_m[0] + obstacle.size_m[0] / 2.0 + inflation,
            obstacle.center_m[1] - obstacle.size_m[1] / 2.0 - inflation,
            obstacle.center_m[1] + obstacle.size_m[1] / 2.0 + inflation,
        )
        for name, actual, expected in (
            ("x_min", expected_bounds[0], unsafe_box.x_min),
            ("x_max", expected_bounds[1], unsafe_box.x_max),
            ("y_min", expected_bounds[2], unsafe_box.y_min),
            ("y_max", expected_bounds[3], unsafe_box.y_max),
        ):
            _expect_close(
                actual,
                expected,
                f"{obstacle.obstacle_id}.inflated_{name}",
            )
        obstacles.append(obstacle)
    return tuple(obstacles), inflation


def load_scenario_spec(path: str | Path) -> ResolvedScenarioSpec:
    """Load and fail-closed validate the canonical scenario YAML."""

    source_path = Path(path).expanduser().resolve()
    source_bytes = source_path.read_bytes()
    document = _mapping(
        yaml.safe_load(source_bytes.decode("utf-8")),
        "ScenarioSpec",
    )
    if int(document.get("schema_version", -1)) != 3:
        raise ValueError("unsupported ScenarioSpec schema_version")
    if document.get("kind") != "scenario_spec":
        raise ValueError("ScenarioSpec kind must be 'scenario_spec'")
    scenario = _mapping(document.get("scenario"), "scenario")
    scenario_id = str(scenario.get("id", ""))
    frame_id = str(scenario.get("frame_id", ""))
    if scenario_id != "P1_ROVER_STATIC_OBSTACLES_AABB_V1":
        raise ValueError("unexpected canonical scenario identity")
    if frame_id != "world" or scenario.get("units") != "SI":
        raise ValueError("the first slice requires world-frame SI geometry")

    _validate_math_sets(document)
    obstacles, inflation = _resolve_projection(document)
    return ResolvedScenarioSpec(
        scenario_id=scenario_id,
        frame_id=frame_id,
        source_path=source_path,
        source_sha256=sha256(source_bytes).hexdigest(),
        physical_obstacles=obstacles,
        total_inflation_radius_m=inflation,
    )
