from typing import TypeVar, List, Tuple
import numpy as np
from ._pop_list import PopList

TIndividual = TypeVar("TIndividual")
TMeasures = TypeVar("TMeasures")


def topn(
    original_population: PopList[TIndividual, TMeasures],
    offspring_population: PopList[TIndividual, TMeasures],
    measure: str,
    n: int,
) -> Tuple[List[int], List[int]]:
    indices = np.argsort(
        [i.measures[measure] for i in original_population.individuals]
        + [i.measures[measure] for i in offspring_population.individuals]
    )[: -1 - n : -1]
    return [i for i in indices if i < len(original_population.individuals)], [
        i - len(original_population.individuals)
        for i in indices
        if i >= len(original_population.individuals)
    ]
