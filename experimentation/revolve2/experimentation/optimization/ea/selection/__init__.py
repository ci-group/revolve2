"""Functions for selecting individuals from populations in EA algorithms."""

from ._multiple_unique import multiple_unique
from ._pareto_frontier import pareto_frontier
from ._topn import topn
from ._tournament import tournament

__all__ = ["multiple_unique", "pareto_frontier", "topn", "tournament"]
