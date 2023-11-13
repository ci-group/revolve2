from typing import Callable, TypeVar

import numpy as np
import numpy.typing as npt

Genotype = TypeVar("Genotype")
Fitness = TypeVar("Fitness")
Ages = TypeVar("Ages")


def steady_state_innovation_protection(
    old_genotypes: list[Genotype],
    old_fitnesses: list[Fitness],
    old_ages: list[Ages],
    new_genotypes: list[Genotype],
    new_fitnesses: list[Fitness],
    new_ages: list[Ages],
    selection_fun: Callable[
        [list[Genotype], list[Fitness], list[Ages], int], npt.NDArray[np.int_]
    ],
) -> tuple[list[int], list[int]]:
    """
    Select `len(old_genotypes)` individuals using the provided selection function from combined set of old and new individuals.

    :param old_genotypes: Genotypes of the individuals in the parent population.
    :param old_fitnesses: Fitnesses of the individuals in the parent population.
    :param old_ages: Ages of the individuals in the parent population.
    :param new_genotypes: Genotypes of the individuals from the offspring.
    :param new_fitnesses: Fitnesses of the individuals from the offspring.
    :param new_ages: Ages of the individuals from the offspring .
    :param selection_fun: Function that selects n individuals from a population based on their genotype and fitness. (genotypes, fitnesses, ages, n) -> indices
    :returns: (indices of selected individuals from parent population, indices of selected individuals from offspring).
    """
    assert len(old_genotypes) == len(old_fitnesses) == len(old_ages)
    assert len(new_genotypes) == len(new_fitnesses) == len(new_ages)

    population_size = len(old_genotypes)

    selection = selection_fun(
        old_genotypes + new_genotypes, old_fitnesses + new_fitnesses, old_ages + new_ages, population_size
    )

    selected_old = [s for s in selection if s < len(old_fitnesses)]
    selected_new = [
        s - len(old_fitnesses) for s in selection if s >= len(old_fitnesses)
    ]

    return selected_old, selected_new
