from typing import Callable, List, TypeVar

from .. import selection

Individual = TypeVar("Individual")


def steady_state(
    old_individuals: List[Individual],
    new_individuals: List[Individual],
    selection_function: Callable[[List[Individual]], Individual],
) -> List[Individual]:
    """
    Select n unique individuals from the combined set of old and new individuals
    using the provided selection function,
    where n is the number of old individuals.
    Also known as mu+lambda.
    """

    population_size = len(old_individuals)
    selection_pool = old_individuals + new_individuals

    return selection.multiple_unique(
        selection_pool, population_size, selection_function
    )
