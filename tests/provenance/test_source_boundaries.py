from __future__ import annotations

import re
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
OLD_ROS_NAMES = {
    "bringup_pkg",
    "gazebo_robot",
    "robot_model",
    "ros2_humble_ws",
    "rover_hardware",
    "yzh_gazebo_sim",
    "yzh_ground",
}
OLD_PYTHON_IMPORT = re.compile(
    r"\bcbf_rover\.(?:"
    r"certificates|data|evaluation|perception|safety_filters|simulation|training|"
    r"world_models"
    r")(?=\.|\b)"
)


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


def test_source_and_simulation_do_not_hardcode_inherited_package_names() -> None:
    violations: list[str] = []
    for path in _source_text_files():
        text = path.read_text(encoding="utf-8")
        folded = text.casefold()
        for old_name in OLD_ROS_NAMES:
            if old_name in folded:
                violations.append(
                    f"{path.relative_to(REPOSITORY_ROOT)} contains {old_name!r}"
                )
        if match := OLD_PYTHON_IMPORT.search(text):
            violations.append(
                f"{path.relative_to(REPOSITORY_ROOT)} contains {match.group(0)!r}"
            )

    assert violations == []
