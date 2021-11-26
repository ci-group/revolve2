from typing import Callable, List, TypeVar

Individual = TypeVar("Individual")


def multiple_unique(
    population: List[Individual],
    selection_size: int,
    selection_function: Callable[[List[Individual]], Individual],
) -> List[Individual]:
    """
    Perform selection on population of distinct group, it can be used in the
    form parent selection or survival selection.
    It never selects the same individual more than once
    :param population: list of individuals where to select from
    :param selection_size: amount of individuals to select
    :param selection_function:
    """
    if len(population) < selection_size:
        assert (
            len(population) >= selection_size
        ), f"Population size({len(population)}) cannot be smaller than selection size({selection_size})"

    selected_individuals = []
    for _ in range(selection_size):
        new_individual = False
        while new_individual is False:
            selected_individual = selection_function(population)
            if selected_individual not in selected_individuals:
                selected_individuals.append(selected_individual)
                new_individual = True
    return selected_individuals
