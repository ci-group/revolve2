"""Populations for evolutionary algorithms."""

from ._pop_list import PopList
from ._multiple_unique import multiple_unique
from ._topn import topn
from ._tournament import tournament

__all__ = ["PopList", "multiple_unique", "topn", "tournament"]
