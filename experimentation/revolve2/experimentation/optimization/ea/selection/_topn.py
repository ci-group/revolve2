from typing import TypeVar

from ._argsort import argsort
from ._supports_lt import SupportsLt

Genotype = TypeVar("Genotype")
Fitness = TypeVar("Fitness", bound=SupportsLt)

# Force n to be passed as a keyword argument
def topn(fitnesses: list[Fitness], *, n: int) -> list[int]:
    """
    Get indices of the top n genotypes sorted by their fitness.

    :param n: The number of genotypes to select.
    :param fitnesses: Fitnesses of the genotypes.
    :returns: Indices of the selected genotypes.
    """
    assert len(fitnesses) >= n

    return argsort(fitnesses)[::-1][:n]
