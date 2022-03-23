from __future__ import annotations
from random import Random
from typing import List, TypeVar, Protocol

TSupportsLessThan = TypeVar("TSupportsLessThan", bound="SupportsLessThan")


class SupportsLessThan(Protocol):
    def __lt__(self: TSupportsLessThan, other: TSupportsLessThan) -> bool:
        pass


Fitness = TypeVar("Fitness", bound="SupportsLessThan")


def tournament(rng: Random, fitnesses: List[Fitness], k: int) -> int:
    """
    Perform tournament selection and return the index of the best individual

    :param rng: random number generator
    :param population: list of individuals where to select from
    :param k: amount of individuals to participate in tournament
    """
    assert len(fitnesses) >= k

    participant_indices = rng.choices(population=range(len(fitnesses)), k=k)
    return max(participant_indices, key=lambda i: fitnesses[i])
