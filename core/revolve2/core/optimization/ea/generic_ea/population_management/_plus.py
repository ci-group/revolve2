from typing import List, Tuple, TypeVar

from ._argsort import argsort

Genotype = TypeVar("Genotype")
Fitness = TypeVar("Fitness")


def plus(
    old_fitnesses: List[Fitness],
    new_fitnesses: List[Fitness],
) -> Tuple[List[int], List[int]]:
    """
    Get the indices of the top `len(old_fitnesses)` fitnesses from the combined set of old and new individuals.

    Also known as mu+lambda.

    :param old_fitnesses: Fitnesses of the original individuals.
    :param new_fitnesses: Fitnesses of the new individuals.
    :returns: Tuple of list of indices of selected old fitnesses and list of indices of selected new fitnesses.
    """
    population_size = len(old_fitnesses)

    all = old_fitnesses + new_fitnesses
    selection = argsort(all)[::-1][:population_size]

    selected_old = [s for s in selection if s < len(old_fitnesses)]
    selected_new = [
        s - len(old_fitnesses) for s in selection if s >= len(old_fitnesses)
    ]

    return selected_old, selected_new
