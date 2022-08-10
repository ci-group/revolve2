"""Functions for combining populations in EA algorithms."""

from ._generational import generational
from ._steady_state import steady_state

__all__ = [
    "generational",
    "steady_state",
]
