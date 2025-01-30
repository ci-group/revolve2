"""Functions for selecting individuals from populations in EA algorithms."""

from revolve2.experimentation.optimization.ea.selection._multiple_unique import multiple_unique
from revolve2.experimentation.optimization.ea.selection._pareto_frontier import pareto_frontier
from revolve2.experimentation.optimization.ea.selection._topn import topn
from revolve2.experimentation.optimization.ea.selection._tournament import tournament
from revolve2.experimentation.optimization.ea.selection._roulette import roulette

__all__ = ["multiple_unique", "pareto_frontier", "topn", "tournament", "roulette"]
