from __future__ import annotations

from typing import List, TypeVar
from ._pop_list import PopList
import numpy as np

TIndividual = TypeVar("TIndividual")


def tournament(
    population: PopList[TIndividual],
    measure: str,
    rng: np.random.Generator,
    k: int,
) -> int:
    """
    Perform tournament selection and return the index of the best individual.

    :param rng: Random number generator.
    :param fitnesses: List of finesses of individuals that joint the tournament.
    :param k: Amount of individuals to participate in tournament.
    :returns: The index of te individual that won the tournament.
    """
    assert len(population.individuals) >= k

    fitnesses = [i.measures[measure] for i in population.individuals]

    participant_indices = rng.choice(range(len(fitnesses)), size=k)
    return max(participant_indices, key=lambda i: fitnesses[i])