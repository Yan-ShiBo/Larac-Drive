from __future__ import annotations

from pathlib import Path


REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
SCAN_ROOTS = (
    REPOSITORY_ROOT / "src" / "cbf_rover",
    REPOSITORY_ROOT / "simulation",
)
TEXT_SUFFIXES = {
    ".cfg",
    ".md",
    ".py",
    ".sdf",
    ".toml",
    ".txt",
    ".world",
    ".xacro",
    ".xml",
    ".yaml",
    ".yml",
}
DEPRECATED_PYTHON_PATHS = {
    "cbf_rover.certificates",
    "cbf_rover.cli",
    "cbf_rover.core",
    "cbf_rover.data",
    "cbf_rover.datasets",
    "cbf_rover.evaluation",
    "cbf_rover.experiments",
    "cbf_rover.methods.learned_rgbd",
    "cbf_rover.perception",
    "cbf_rover.policies",
    "cbf_rover.safety_filters",
    "cbf_rover.simulation",
    "cbf_rover.training",
    "cbf_rover.world_models",
}
DEPRECATED_ROS_NAMES = {
    "bringup_pkg",
    "cbf_rover_gazebo",
    "cbf_rover_ros",
    "gazebo_robot",
    "robot_model",
    "rover_hardware",
    "yzh_gazebo_sim",
}


def _source_text_files() -> list[Path]:
    files: list[Path] = []
    for root in SCAN_ROOTS:
        assert root.is_dir(), root
        files.extend(
            path
            for path in root.rglob("*")
            if path.is_file() and path.suffix.casefold() in TEXT_SUFFIXES
        )
    return sorted(files)


def test_source_and_simulation_do_not_reference_deprecated_module_or_ros_paths() -> None:
    violations: list[str] = []
    forbidden = DEPRECATED_PYTHON_PATHS | DEPRECATED_ROS_NAMES
    for path in _source_text_files():
        text = path.read_text(encoding="utf-8").casefold()
        for old_path in forbidden:
            if old_path in text:
                violations.append(
                    f"{path.relative_to(REPOSITORY_ROOT)} contains {old_path!r}"
                )

    assert violations == []
