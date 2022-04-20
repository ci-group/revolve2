from typing import Callable, List, TypeVar

Genotype = TypeVar("Genotype")
Fitness = TypeVar("Fitness")  # TODO bounds


def multiple_unique(
    population: List[Genotype],
    fitnesses: List[Fitness],
    selection_size: int,
    selection_function: Callable[[List[Genotype], List[Fitness]], int],
) -> List[int]:
    """
    Perform selection on population of distinct group, it can be used in the
    form parent selection or survival selection.
    It never selects the same individual more than once
    :param population: list of individuals where to select from
    :param selection_size: amount of individuals to select
    :param selection_function:
    """
    assert len(population) == len(fitnesses)
    assert selection_size < len(population)

    selected_individuals = []
    for _ in range(selection_size):
        new_individual = False
        while new_individual is False:
            selected_individual = selection_function(population, fitnesses)
            if selected_individual not in selected_individuals:
                selected_individuals.append(selected_individual)
                new_individual = True
    return selected_individuals
