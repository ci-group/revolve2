from typing import List, TypeVar

from ._argsort import argsort

Genotype = TypeVar("Genotype")
Fitness = TypeVar("Fitness")


def comma(
    population_size: int,
    new_fitnesses: List[Fitness],
) -> List[int]:
    """
    Get the indices of the top `population_size` fitnesses.

    Also known as mu,lambda.

    :param population_size: Number of indices to select.
    :param new_fitnesses: The fitnesses.
    :returns: List of indices of selected fitnesses.
    """
    assert len(new_fitnesses) >= population_size

    return argsort(new_fitnesses)[::-1][:population_size]
