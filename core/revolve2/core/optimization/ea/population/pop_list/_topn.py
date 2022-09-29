from typing import TypeVar, List, Tuple
import numpy as np
from ._pop_list import PopList

TIndividual = TypeVar("TIndividual")


def topn(
    original_population: PopList[TIndividual],
    offspring_population: PopList[TIndividual],
    measure: str,
    n: int,
) -> Tuple[List[int], List[int]]:
    indices = np.argsort(
        [i.measures[measure] for i in original_population.individuals]
        + [i.measures[measure] for i in offspring_population.individuals]
    )
    return [i for i in indices if i < len(original_population.individuals)], [
        i - len(original_population.individuals)
        for i in indices
        if i >= len(original_population.individuals)
    ]
