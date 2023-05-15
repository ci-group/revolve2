from typing import Callable, List, Tuple, TypeVar, cast

Individual = TypeVar("Individual")
Fitness = TypeVar("Fitness")  # TODO bounds


def multiple_unique(
    selection_size: int,
    population: List[Individual],
    fitnesses: List[Fitness],
    selection_function: Callable[[List[Individual], List[Fitness]], int],
) -> Tuple[int, ...]:  # TODO return type can be know by selection size
    """
    Select multiple distinct individuals from a population using the provided selection function.

    :param selection_size: Amount of of individuals to select.
    :param population: List of individuals to select from.
    :param fitnesses: Fitnesses of the population.
    :param selection_function: Function that select a single individual from a population. ([Individual], [Fitness]) -> index.
    :returns: Indices of the selected individuals.
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
    return cast(Tuple[int, int], tuple(selected_individuals))