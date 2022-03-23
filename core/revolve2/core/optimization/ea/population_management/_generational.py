from typing import Callable, List, TypeVar, Tuple

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
    Select n unique individuals from only the new individuals
    using the provided selection function,
    where n is the number of old individuals.
    Also known as mu,lambda.
    """

    population_size = len(old_individuals)
    selection_pool = new_individuals

    assert len(selection_pool) >= population_size

    return selection.multiple_unique(
        new_individuals, new_fitnesses, population_size, selection_function
    )
