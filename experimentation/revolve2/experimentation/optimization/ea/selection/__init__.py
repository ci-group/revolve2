"""Functions for selecting individuals from populations in EA algorithms."""

from ._multiple_unique import multiple_unique
from ._topn import topn
from ._tournament import tournament
from ._pareto_frontier import pareto_frontier

__all__ = ["multiple_unique", "topn", "tournament"]
