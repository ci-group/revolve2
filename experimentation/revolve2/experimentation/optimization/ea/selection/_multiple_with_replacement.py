from typing import Callable, TypeVar

import numpy as np
import numpy.typing as npt

TIndividual = TypeVar("TIndividual")
TFitness = TypeVar("TFitness")


def multiple_with_replacement(
    selection_size: int,
    population: list[TIndividual],
    fitnesses: list[TFitness],
    selection_function: Callable[[list[TIndividual], list[TFitness]], int],
) -> npt.NDArray[np.float_]:
    """
    Select multiple individuals from a population using the provided selection function.

    :param selection_size: Amount of of individuals to select.
    :param population: List of individuals to select from.
    :param fitnesses: Fitnesses of the population.
    :param selection_function: Function that select a single individual from a population. ([TIndividual], [TFitness]) -> index.
    :returns: Indices of the selected individuals.
    """
    assert len(population) == len(fitnesses)
    assert selection_size <= len(population)

    selected_individuals = []

    while len(selected_individuals) < selection_size:
        selected_individual = selection_function(population, fitnesses)
        selected_individuals.append(selected_individual)

    return np.array(selected_individuals)
