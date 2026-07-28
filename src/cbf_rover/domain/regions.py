"""Set-membership contract used by systems and certificate methods."""

from __future__ import annotations

from typing import Protocol, TypeVar, runtime_checkable


StateT = TypeVar("StateT", contravariant=True)


@runtime_checkable
class Region(Protocol[StateT]):
    """A named state-space set with a membership predicate."""

    name: str

    def contains(self, state: StateT) -> bool:
        """Return whether ``state`` belongs to this region."""
