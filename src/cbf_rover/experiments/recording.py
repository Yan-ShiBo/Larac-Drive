"""Append-only JSON Lines recording for reproducible experiment traces."""

from __future__ import annotations

import json
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any, Mapping


@dataclass(frozen=True)
class StepRecord:
    episode_id: str
    step_index: int
    reference_action: Any
    filtered_action: Any
    reward: float
    cost: float
    metadata: Mapping[str, Any]


class JsonlRecorder:
    """Write one self-contained JSON object per line."""

    def __init__(self, output_path: str | Path) -> None:
        self.output_path = Path(output_path)
        self.output_path.parent.mkdir(parents=True, exist_ok=True)

    def append(self, record: StepRecord) -> None:
        payload = asdict(record)
        with self.output_path.open("a", encoding="utf-8") as stream:
            stream.write(json.dumps(payload, ensure_ascii=False, default=_json_default))
            stream.write("\n")


def _json_default(value: Any) -> Any:
    if hasattr(value, "tolist"):
        return value.tolist()
    if hasattr(value, "item"):
        return value.item()
    raise TypeError(f"Cannot serialize {type(value).__name__}")
