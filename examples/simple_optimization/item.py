"""An item that could be in a knapsack."""

from dataclasses import dataclass


@dataclass
class Item:
    """An item that could be in a knapsack."""

    weight: float
    value: float
