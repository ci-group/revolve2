from typing import List, TypeVar, Callable, Tuple
from ._pop_list import PopList

TIndividual = TypeVar("TIndividual")
TMeasures = TypeVar("TMeasures")


def multiple_unique(
    population: PopList[TIndividual, TMeasures],
    selection_size: int,
    selection_function: Callable[[PopList[TIndividual, TMeasures]], int],
) -> List[int]:
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
