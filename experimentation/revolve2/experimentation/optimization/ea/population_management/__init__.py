"""Functions for combining populations in EA algorithms."""

from ._generational import generational
from ._steady_state import steady_state
from ._steady_state_mip import steady_state_morphological_innovation_protection

__all__ = [
    "generational",
    "steady_state",
    "steady_state_morphological_innovation_protection",
]
