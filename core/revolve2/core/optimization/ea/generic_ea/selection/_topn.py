from typing import List, TypeVar

from ._argsort import argsort
from ._supports_lt import SupportsLt

Genotype = TypeVar("Genotype")
Fitness = TypeVar("Fitness", bound=SupportsLt)


def topn(n: int, genotypes: List[Genotype], fitnesses: List[Fitness]) -> List[int]:
    """
    Get indices of the top n genotypes sorted by their fitness.

    :param n: The number of genotypes to select.
    :param genotypes: The genotypes. Ignored, but argument kept for function signature compatibility with other selection functions/
    :param fitnesses: Fitnesses of the genotypes.
    :returns: Indices of the selected genotypes.
    """
    assert len(fitnesses) >= n

    return argsort(fitnesses)[::-1][:n]
