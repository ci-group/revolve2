from typing import Callable, List, TypeVar

from .. import selection

Genotype = TypeVar("Genotype")
Fitness = TypeVar("Fitness")


def generational(
    old_individuals: List[Genotype],
    old_fitnesses: List[Fitness],
    new_individuals: List[Genotype],
    new_fitnesses: List[Fitness],
    selection_function: Callable[[List[Genotype], List[Fitness]], int],
) -> List[int]:
    """
    Select n unique individuals from only the new individuals using the provided selection function.

    N is the number of old individuals.
    # TODO this is not actually generational, see issue https://github.com/ci-group/revolve2/issues/142
    Also known as mu,lambda.

    :param old_individuals: Original individuals.
    :param old_fitnesses: Fitnesses of the original individuals.
    :param new_individuals: New individuals.
    :paramnew_fitnesses: Fitnesses of the new individuals.
    :param selection_function: Function selecting a single individual from the new individuals based on its fitness. ([Genotype], [Fitness]) -> index.
    :returns: List of indices of selected individuals from the list of new individuals.
    """
    population_size = len(old_individuals)
    selection_pool = new_individuals

    assert len(selection_pool) >= population_size

    return selection.multiple_unique(
        new_individuals, new_fitnesses, population_size, selection_function
    )
