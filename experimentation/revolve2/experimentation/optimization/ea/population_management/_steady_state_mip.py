from typing import Callable, TypeVar

import numpy as np
import numpy.typing as npt

TGenotype = TypeVar("TGenotype")
TValues = TypeVar("TValues")


def steady_state_morphological_innovation_protection(
    old_genotypes: list[TGenotype],
    old_fitnesses: list[float],
    old_ages: list[int],
    new_genotypes: list[TGenotype],
    new_fitnesses: list[float],
    new_ages: list[int],
    selection_fun: Callable[
        [list[list[TValues]], list[bool], int], npt.NDArray[np.int_]
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

    selection = selection_fun([old_fitnesses + new_fitnesses, old_ages + new_ages], [False, True], population_size)

    selected_old = [s for s in selection if s < len(old_fitnesses)]
    selected_new = [
        s - len(old_fitnesses) for s in selection if s >= len(old_fitnesses)
    ]

    return selected_old, selected_new