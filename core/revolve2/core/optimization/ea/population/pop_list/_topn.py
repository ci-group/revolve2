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
    """
    Select the top n individuals from two combined populations based on one of their measures.

    :param original_population: The first population to consider.
    :param offspring_population: The second population to consider.
    :param measure: The measure to rank by.
    :param n: The number of individual to select.
    :returns: Indices of the selected individuals in their respective populations.
    """
    indices = np.argsort(
        [i.measures[measure] for i in original_population.individuals]
        + [i.measures[measure] for i in offspring_population.individuals]
    )[: -1 - n : -1]
    return [i for i in indices if i < len(original_population.individuals)], [
        i - len(original_population.individuals)
        for i in indices
        if i >= len(original_population.individuals)
    ]
