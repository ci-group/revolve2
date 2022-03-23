from typing import Callable, List, Tuple, TypeVar

from ..selection import multiple_unique

Genotype = TypeVar("Genotype")
Fitness = TypeVar("Fitness")


def steady_state(
    old_individuals: List[Genotype],
    old_fitnesses: List[Fitness],
    new_individuals: List[Genotype],
    new_fitnesses: List[Fitness],
    selection_function: Callable[[List[Genotype], List[Fitness]], int],
) -> Tuple[List[int], List[int]]:
    """
    Select n unique individuals from the combined set of old and new individuals
    using the provided selection function,
    where n is the number of old individuals.
    Also known as mu+lambda.
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
