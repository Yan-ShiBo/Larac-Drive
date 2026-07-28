from __future__ import annotations

import os
from pathlib import Path


REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
ORIGINAL_ARCHIVE_ROOT = REPOSITORY_ROOT / "接手的原始文件"
MODEL_MANIFEST = REPOSITORY_ROOT / "artifacts" / "manifests" / "models.yaml"
MODEL_WEIGHT_SUFFIXES = (
    ".pt",
    ".pth",
    ".tar",
    ".ckpt",
    ".onnx",
    ".safetensors",
)
EXPECTED_MISSING_SLOTS = {
    "reference_policy",
    "rgbd_vae",
    "rgbd_mdrnn",
    "learned_cbf",
    "learned_clf",
}


def _repository_weight_files() -> list[Path]:
    matches: list[Path] = []
    for directory, directory_names, file_names in os.walk(
        REPOSITORY_ROOT,
        topdown=True,
    ):
        current = Path(directory)
        if current == REPOSITORY_ROOT:
            directory_names[:] = [
                name
                for name in directory_names
                if name not in {".git", ORIGINAL_ARCHIVE_ROOT.name}
            ]
        for name in file_names:
            if name.casefold().endswith(MODEL_WEIGHT_SUFFIXES):
                matches.append((current / name).relative_to(REPOSITORY_ROOT))
    return sorted(matches)


def _manifest_slot_statuses(text: str) -> dict[str, str]:
    statuses: dict[str, str] = {}
    in_slots = False
    current_id: str | None = None
    for line in text.splitlines():
        if line == "slots:":
            in_slots = True
            continue
        if not in_slots:
            continue
        if line.startswith("  - id: "):
            current_id = line.removeprefix("  - id: ").strip()
            continue
        if current_id is not None and line.startswith("    status: "):
            assert current_id not in statuses, f"duplicate slot id: {current_id}"
            statuses[current_id] = line.removeprefix("    status: ").strip()
    return statuses


def test_repository_has_no_embedded_model_weight_files() -> None:
    assert _repository_weight_files() == []


def test_model_manifest_has_exactly_five_missing_external_artifact_slots() -> None:
    text = MODEL_MANIFEST.read_text(encoding="utf-8")
    statuses = _manifest_slot_statuses(text)

    assert set(statuses) == EXPECTED_MISSING_SLOTS
    assert set(statuses.values()) == {"missing"}
    assert "  repository_contains_weights: false" in text
    assert "  storage_root_from_environment: CBF_ROVER_ARTIFACT_ROOT" in text
