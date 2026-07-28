"""Manifest-verified checkpoint loading for the learned RGB-D method."""

from __future__ import annotations

import hashlib
import re
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import torch
import yaml


_SLOT_TO_FIELD = {
    "rgbd_vae": "vae",
    "rgbd_mdrnn": "mdrnn",
    "learned_cbf": "cbf",
    "learned_clf": "clf",
}
_SHA256_PATTERN = re.compile(r"[0-9a-f]{64}")


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(4 * 1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _verified_checkpoint_from_slot(
    *,
    root: Path,
    slot_id: str,
    slot: dict[str, Any],
) -> "VerifiedCheckpoint":
    if slot.get("status") != "verified":
        raise ValueError(f"artifact slot is not verified: {slot_id}")

    uri = slot.get("uri")
    if not isinstance(uri, str) or not uri.startswith("artifact://"):
        raise ValueError(f"artifact slot has an invalid URI: {slot_id}")
    relative_value = uri.removeprefix("artifact://")
    relative_path = Path(relative_value)
    if (
        not relative_value
        or "?" in relative_value
        or "#" in relative_value
        or relative_path.is_absolute()
    ):
        raise ValueError(f"artifact URI must be a plain relative path: {slot_id}")
    candidate = (root / relative_path).resolve(strict=True)
    try:
        candidate.relative_to(root)
    except ValueError as exc:
        raise ValueError(f"artifact URI escapes root: {slot_id}") from exc

    expected_hash = slot.get("sha256")
    if not isinstance(expected_hash, str) or not _SHA256_PATTERN.fullmatch(
        expected_hash
    ):
        raise ValueError(f"artifact slot has an invalid SHA-256: {slot_id}")
    expected_bytes = slot.get("bytes")
    if (
        not isinstance(expected_bytes, int)
        or isinstance(expected_bytes, bool)
        or expected_bytes < 0
    ):
        raise ValueError(f"artifact slot has invalid byte count: {slot_id}")

    provenance = slot.get("provenance")
    compatibility = slot.get("compatibility")
    if not isinstance(provenance, dict) or not all(
        provenance.get(key)
        for key in (
            "source_type",
            "training_run_id",
            "source_revision",
            "created_at_utc",
            "dataset_manifest_uri",
        )
    ):
        raise ValueError(f"artifact slot has incomplete provenance: {slot_id}")
    if not isinstance(compatibility, dict) or not all(
        compatibility.get(key)
        for key in (
            "framework",
            "framework_version",
            "architecture",
            "loader",
            "input_contract",
            "output_contract",
        )
    ):
        raise ValueError(f"artifact slot has incomplete compatibility: {slot_id}")

    checkpoint = VerifiedCheckpoint(
        slot_id=slot_id,
        artifact_root=root,
        path=candidate,
        sha256=expected_hash,
        bytes=expected_bytes,
    )
    checkpoint.validate()
    return checkpoint


@dataclass(frozen=True)
class VerifiedCheckpoint:
    """One manifest slot bound to an immutable external file identity."""

    slot_id: str
    artifact_root: Path
    path: Path
    sha256: str
    bytes: int

    def validate(self) -> None:
        root = self.artifact_root.resolve(strict=True)
        candidate = self.path.resolve(strict=True)
        if not root.is_dir():
            raise NotADirectoryError(f"artifact root is not a directory: {root}")
        try:
            candidate.relative_to(root)
        except ValueError as exc:
            raise ValueError(
                f"checkpoint path escapes artifact root: {candidate}"
            ) from exc
        if not candidate.is_file():
            raise FileNotFoundError(f"checkpoint is not a file: {candidate}")
        if candidate.stat().st_size != self.bytes:
            raise ValueError(f"checkpoint byte count mismatch: {self.slot_id}")
        if _sha256(candidate) != self.sha256:
            raise ValueError(f"checkpoint SHA-256 mismatch: {self.slot_id}")


@dataclass(frozen=True)
class LearnedRGBDCheckpointSet:
    """Four verified external checkpoints required by the learned method."""

    manifest_path: Path
    artifact_root: Path
    vae: VerifiedCheckpoint
    mdrnn: VerifiedCheckpoint
    cbf: VerifiedCheckpoint
    clf: VerifiedCheckpoint

    @classmethod
    def from_manifest(
        cls,
        *,
        manifest_path: str | Path,
        artifact_root: str | Path,
    ) -> "LearnedRGBDCheckpointSet":
        """Resolve four verified slots from the versioned artifact manifest."""

        root = Path(artifact_root).expanduser().resolve(strict=True)
        if not root.is_dir():
            raise NotADirectoryError(f"artifact root is not a directory: {root}")
        manifest = Path(manifest_path).expanduser().resolve(strict=True)
        if not manifest.is_file():
            raise FileNotFoundError(f"artifact manifest is not a file: {manifest}")

        with manifest.open("r", encoding="utf-8") as stream:
            document = yaml.safe_load(stream)
        if not isinstance(document, dict):
            raise ValueError("artifact manifest must contain a YAML mapping")
        artifact_set = document.get("artifact_set")
        if not isinstance(artifact_set, dict):
            raise ValueError("artifact manifest is missing artifact_set")
        if artifact_set.get("status") != "ready":
            raise ValueError("artifact_set status must be 'ready'")

        raw_slots = document.get("slots")
        if not isinstance(raw_slots, list):
            raise ValueError("artifact manifest slots must be a list")
        slots_by_id: dict[str, dict[str, Any]] = {}
        for slot in raw_slots:
            if not isinstance(slot, dict) or not isinstance(slot.get("id"), str):
                raise ValueError(
                    "every artifact manifest slot must be a mapping with an id"
                )
            slot_id = slot["id"]
            if slot_id in slots_by_id:
                raise ValueError(f"artifact manifest has duplicate slot: {slot_id}")
            slots_by_id[slot_id] = slot

        resolved_required: dict[str, VerifiedCheckpoint] = {}
        for slot_id, slot in slots_by_id.items():
            required = slot.get("required")
            if not isinstance(required, bool):
                raise ValueError(
                    f"artifact slot required flag must be boolean: {slot_id}"
                )
            if required:
                resolved_required[slot_id] = _verified_checkpoint_from_slot(
                    root=root,
                    slot_id=slot_id,
                    slot=slot,
                )

        verified: dict[str, VerifiedCheckpoint] = {}
        for slot_id, field_name in _SLOT_TO_FIELD.items():
            checkpoint = resolved_required.get(slot_id)
            if checkpoint is None:
                raise ValueError(f"artifact manifest is missing slot: {slot_id}")
            verified[field_name] = checkpoint

        checkpoint_set = cls(
            manifest_path=manifest,
            artifact_root=root,
            vae=verified["vae"],
            mdrnn=verified["mdrnn"],
            cbf=verified["cbf"],
            clf=verified["clf"],
        )
        checkpoint_set.validate()
        return checkpoint_set

    def validate(self) -> None:
        for checkpoint in (self.vae, self.mdrnn, self.cbf, self.clf):
            checkpoint.validate()


def _safe_torch_load(
    path: Path,
    device: torch.device | str,
) -> Any:
    try:
        return torch.load(path, map_location=device, weights_only=True)
    except TypeError as exc:
        raise RuntimeError(
            "this project requires a PyTorch release with weights_only=True support"
        ) from exc


def load_checkpoint(
    checkpoint: VerifiedCheckpoint,
    device: torch.device | str,
) -> Any:
    """Reverify identity and load tensor-only content from one manifest slot."""

    if not isinstance(checkpoint, VerifiedCheckpoint):
        raise TypeError("inference checkpoints must come from a verified manifest")
    checkpoint.validate()
    return _safe_torch_load(checkpoint.path, device)


def load_training_checkpoint(
    path: str | Path,
    device: torch.device | str,
) -> Any:
    """Safely load an operator-selected checkpoint for a server training job."""

    resolved = Path(path).expanduser().resolve(strict=True)
    if not resolved.is_file():
        raise FileNotFoundError(f"training checkpoint is not a file: {resolved}")
    return _safe_torch_load(resolved, device)


def state_dict_from_checkpoint(checkpoint: Any) -> Any:
    """Extract common nested state-dict layouts used by inherited training."""

    if isinstance(checkpoint, dict):
        for key in ("state_dict", "model_state_dict", "model"):
            if key in checkpoint:
                return checkpoint[key]
    return checkpoint
