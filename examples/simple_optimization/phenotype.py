"""Knapsack phenotype."""

from dataclasses import dataclass
from typing import List


@dataclass
class Phenotype(List[bool]):
    """
    Phenotype of a knapsack.

    A bitstring with each bit representing if an item is in the knapsack.
    """

    items: List[bool]
