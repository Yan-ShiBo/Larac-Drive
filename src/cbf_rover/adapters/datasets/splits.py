"""Leakage-resistant grouped split helpers."""

from __future__ import annotations

import random
from collections.abc import Sequence


def grouped_sample_indices(
    group_lengths: Sequence[int],
    *,
    test_ratio: float,
    seed: int,
) -> tuple[list[int], list[int]]:
    if len(group_lengths) < 2:
        raise ValueError("grouped splitting requires at least two source groups")
    if not 0.0 < test_ratio < 1.0:
        raise ValueError("test_ratio must be strictly between zero and one")
    if any(length <= 0 for length in group_lengths):
        raise ValueError("all group lengths must be positive")

    group_ids = list(range(len(group_lengths)))
    random.Random(seed).shuffle(group_ids)
    test_group_count = min(
        len(group_ids) - 1,
        max(1, round(len(group_ids) * test_ratio)),
    )
    test_groups = set(group_ids[:test_group_count])
    train_indices: list[int] = []
    test_indices: list[int] = []
    offset = 0
    for group_id, length in enumerate(group_lengths):
        target = test_indices if group_id in test_groups else train_indices
        target.extend(range(offset, offset + length))
        offset += length
    return train_indices, test_indices
