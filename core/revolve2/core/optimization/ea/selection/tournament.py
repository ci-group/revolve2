from random import Random
from typing import List, Tuple, TypeVar

Individual = TypeVar("Individual")
Fitness = TypeVar("Fitness")


def tournament(
    rng: Random, population: List[Tuple[Individual, Fitness]], k
) -> Tuple[Individual, Fitness]:
    """
    Perform tournament selection and return best individual

    :param rng: random number generator
    :param population: list of individuals where to select from
    :param k: amount of individuals to participate in tournament
    """
    assert len(population) >= k

    participants = rng.choices(population=population, k=k)
    return max(participants, key=lambda p: p[1])
