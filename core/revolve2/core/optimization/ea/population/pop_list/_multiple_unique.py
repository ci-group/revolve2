from typing import Callable, List, TypeVar

from .._measures import Measures
from .._serializable import Serializable
from ._pop_list import PopList

TIndividual = TypeVar("TIndividual", bound=Serializable)
TMeasures = TypeVar("TMeasures", bound=Measures)


def multiple_unique(
    population: PopList[TIndividual, TMeasures],
    selection_size: int,
    selection_function: Callable[[PopList[TIndividual, TMeasures]], int],
) -> List[int]:
    """
    Select multiple unique individuals from a population using the provided selection function.

    :param population: The population to select from.
    :param selection_size: The number of individuals to select.
    :param selection_function: The function to use to select an individual.
    :returns: Indices of the selected individuals in the population.
    """
    assert selection_size < len(population.individuals)

    selected_individuals = []
    for _ in range(selection_size):
        new_individual = False
        while new_individual is False:
            selected_individual = selection_function(population)
            if selected_individual not in selected_individuals:
                selected_individuals.append(selected_individual)
                new_individual = True
    return selected_individuals
