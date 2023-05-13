"""topn function."""

from typing import List, Tuple

import numpy as np
from population import Population


def topn(
    original_population: Population,
    offspring_population: Population,
    n: int,
) -> Tuple[List[int], List[int]]:
    """
    Select the top n individuals from two combined populations based on one of their measures.

    :param original_population: The first population to consider.
    :param offspring_population: The second population to consider.
    :param n: The number of individual to select.
    :returns: Indices of the selected individuals in their respective populations. Original, offspring.
    """
    measures = [i.fitness for i in original_population.individuals] + [
        i.fitness for i in offspring_population.individuals
    ]

    indices = np.argsort(measures)[: -1 - n : -1]
    return [i for i in indices if i < len(original_population.individuals)], [
        i - len(original_population.individuals)
        for i in indices
        if i >= len(original_population.individuals)
    ]
