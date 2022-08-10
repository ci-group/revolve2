from typing import Callable, List, TypeVar

from .. import selection

Genotype = TypeVar("Genotype")
Fitness = TypeVar("Fitness")


def comma_with_selection(
    population_size: int,
    individuals: List[Genotype],
    fitnesses: List[Fitness],
    selection_function: Callable[[List[Genotype], List[Fitness]], int],
) -> List[int]:
    """
    Get the indices of `population_size` unique individuals from a set of individuals and their fitnesses using the provided selection function.

    :param population_size: Number of indices to select.
    :param individuals: The individuals.
    :param fitnesses: Fitnesses of the individuals.
    :param selection_function: Function selecting a single individual from the individuals based on its fitness. ([Genotype], [Fitness]) -> index.
    :returns: List of indices of selected individuals.
    """
    assert len(individuals) >= population_size

    return selection.multiple_unique(
        individuals, fitnesses, population_size, selection_function
    )
