"""Functions for combining populations in EA algorithms."""

from ._comma import comma
from ._comma_with_selection import comma_with_selection
from ._plus import plus
from ._plus_with_selection import plus_with_selection

__all__ = ["comma", "comma_with_selection", "plus", "plus_with_selection"]
