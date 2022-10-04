from typing import List, Tuple, TypeVar

import numpy as np

from .._db_serializable import DbSerializable
from .._measures import Measures
from ._pop_list import PopList

TIndividual = TypeVar("TIndividual", bound=DbSerializable)
TMeasures = TypeVar("TMeasures", bound=Measures)


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
