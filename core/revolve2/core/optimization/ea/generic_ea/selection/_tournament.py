from __future__ import annotations

from random import Random
from typing import List, Protocol, TypeVar

TSupportsLessThan = TypeVar("TSupportsLessThan", bound="SupportsLessThan")


class SupportsLessThan(Protocol):
    """Interface for types supporting the < operator."""

    def __lt__(self: TSupportsLessThan, other: TSupportsLessThan) -> bool:
        """
        Compare two objects using the < operator.

        :param other: The object to compare this with.
        """
        pass


Fitness = TypeVar("Fitness", bound="SupportsLessThan")


def tournament(rng: Random, fitnesses: List[Fitness], k: int) -> int:
    """
    Perform tournament selection and return the index of the best individual.

    :param rng: Random number generator.
    :param fitnesses: List of finesses of individuals that joint the tournamente.
    :param k: Amount of individuals to participate in tournament.
    :returns: The index of te individual that won the tournament.
    """
    assert len(fitnesses) >= k

    participant_indices = rng.choices(population=range(len(fitnesses)), k=k)
    return max(participant_indices, key=lambda i: fitnesses[i])
