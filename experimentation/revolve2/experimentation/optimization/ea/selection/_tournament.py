from typing import TypeVar

import numpy as np

Fitness = TypeVar("Fitness")


def tournament(rng: np.random.Generator, fitnesses: list[Fitness], k: int) -> int:
    """
    Perform tournament selection and return the index of the best individual.

    :param rng: Random number generator.
    :param fitnesses: List of finesses of individuals that joint the tournamente.
    :param k: Amount of individuals to participate in tournament.
    :returns: The index of te individual that won the tournament.
    """
    assert len(fitnesses) >= k

    participant_indices = rng.choice(range(len(fitnesses)), size=k)
    return max(participant_indices, key=lambda i: fitnesses[i])  # type: ignore[no-any-return]
