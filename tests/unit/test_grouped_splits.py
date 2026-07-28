from __future__ import annotations

import pytest

from cbf_rover.datasets import grouped_sample_indices


def test_grouped_split_keeps_each_episode_wholly_in_one_partition() -> None:
    lengths = [2, 3, 4, 5]
    train, test = grouped_sample_indices(lengths, test_ratio=0.25, seed=42)
    train_set = set(train)
    test_set = set(test)

    assert train_set
    assert test_set
    assert train_set.isdisjoint(test_set)
    assert train_set | test_set == set(range(14))

    episode_ranges = (
        set(range(0, 2)),
        set(range(2, 5)),
        set(range(5, 9)),
        set(range(9, 14)),
    )
    assert all(group <= train_set or group <= test_set for group in episode_ranges)


def test_grouped_split_is_reproducible_for_a_fixed_seed() -> None:
    first = grouped_sample_indices([5, 5, 5], test_ratio=0.34, seed=7)
    second = grouped_sample_indices([5, 5, 5], test_ratio=0.34, seed=7)

    assert first == second


@pytest.mark.parametrize(
    ("lengths", "ratio"),
    [
        ([5], 0.2),
        ([5, 5], 0.0),
        ([5, 5], 1.0),
        ([5, 0], 0.2),
    ],
)
def test_grouped_split_rejects_inputs_that_can_leak_or_empty_a_group(
    lengths: list[int],
    ratio: float,
) -> None:
    with pytest.raises(ValueError):
        grouped_sample_indices(lengths, test_ratio=ratio, seed=42)
