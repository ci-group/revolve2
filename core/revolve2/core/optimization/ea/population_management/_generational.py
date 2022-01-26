from typing import Callable, List, TypeVar

from .. import selection

Individual = TypeVar("Individual")


def generational(
    old_individuals: List[Individual],
    new_individuals: List[Individual],
    selection_function: Callable[[List[Individual]], Individual],
) -> List[Individual]:
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
        new_individuals, population_size, selection_function
    )
