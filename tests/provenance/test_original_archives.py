from __future__ import annotations

import hashlib
from pathlib import Path


REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
ORIGINAL_ARCHIVE_ROOT = REPOSITORY_ROOT / "接手的原始文件"
EXPECTED_ARCHIVES = {
    "rgbd_safe_method.7z": (
        "AB2F0A0ECEB97D78CB825E04ADEAC2BEB64D27CD3F53A937FF4475708E3893B0"
    ),
    "yzh-skills.7z": (
        "72DE126DE7A22EF0E6E6B82838710D2DD169822620E1B27FB32D0F886E2ECA21"
    ),
    "yzh1-杨子恒的所有原始代码.7z": (
        "405C1A810D72915F14BA759D282E79CC1D8C2D585EE787A591BE029E0FE66A07"
    ),
}


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(4 * 1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest().upper()


def test_original_archive_directory_contains_only_fixed_original_files() -> None:
    assert ORIGINAL_ARCHIVE_ROOT.is_dir()
    actual_entries = {entry.name: entry for entry in ORIGINAL_ARCHIVE_ROOT.iterdir()}

    assert set(actual_entries) == set(EXPECTED_ARCHIVES)
    assert all(entry.is_file() for entry in actual_entries.values())


def test_original_archive_hashes_match_handover_baseline() -> None:
    for name, expected_hash in EXPECTED_ARCHIVES.items():
        archive_path = ORIGINAL_ARCHIVE_ROOT / name
        assert archive_path.is_file(), name
        assert _sha256(archive_path) == expected_hash, name
