from __future__ import annotations

from typing import TypeVar

import numpy as np
from revolve2.core.database import Serializable

from .._serializable_measures import SerializableMeasures
from ._pop_list import PopList

TIndividual = TypeVar("TIndividual", bound=Serializable)
TMeasures = TypeVar("TMeasures", bound=SerializableMeasures)


def tournament(
    population: PopList[TIndividual, TMeasures],
    measure: str,
    rng: np.random.Generator,
    k: int,
) -> int:
    """
    Perform tournament selection and return the index of the best individual.

    :param population: The population containing the individuals.
    :param measure: The measure to use for selection.
    :param rng: Random number generator.
    :param k: Amount of individuals to participate in tournament.
    :returns: The index of te individual that won the tournament.
    """
    assert len(population) >= k

    fitnesses = [i.measures[measure] for i in population]

    participant_indices = rng.choice(range(len(fitnesses)), size=k)
    return max(participant_indices, key=lambda i: fitnesses[i])  # type: ignore # TODO
