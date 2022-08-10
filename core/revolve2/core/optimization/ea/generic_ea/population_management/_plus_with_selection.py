from typing import Callable, List, Tuple, TypeVar

from ..selection import multiple_unique

Genotype = TypeVar("Genotype")
Fitness = TypeVar("Fitness")


def plus_with_selection(
    old_individuals: List[Genotype],
    old_fitnesses: List[Fitness],
    new_individuals: List[Genotype],
    new_fitnesses: List[Fitness],
    selection_function: Callable[[List[Genotype], List[Fitness]], int],
) -> Tuple[List[int], List[int]]:
    """
    Select `len(old_individuals)` unique individuals from the combined set of old and new individuals and their fitnesses using the provided selection function.

    :param old_individuals: Original individuals.
    :param old_fitnesses: Fitnesses of the original individuals.
    :param new_individuals: New individuals.
    :param new_fitnesses: Fitnesses of the new individuals.
    :param selection_function: Function selecting a single individual from the combined set of old and new individuals based using their genotype as well as their fitnesses. ([Genotype], [Fitness]) -> index.
    :returns: Tuple of list of indices of selected old individuals and list of indices of selected new individuals.
    """
    assert len(old_individuals) == len(old_fitnesses)
    assert len(new_individuals) == len(new_fitnesses)

    population_size = len(old_individuals)
    all_individuals = old_individuals + new_individuals
    all_fitnesses = old_fitnesses + new_fitnesses

    selection = multiple_unique(
        all_individuals, all_fitnesses, population_size, selection_function
    )

    selected_old = [s for s in selection if s < len(old_individuals)]
    selected_new = [
        s - len(old_individuals) for s in selection if s >= len(old_individuals)
    ]

    return selected_old, selected_new
