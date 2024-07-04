from typing import Callable, TypeVar

import numpy as np
import numpy.typing as npt

Genotype = TypeVar("Genotype")
Fitness = TypeVar("Fitness")


def steady_state(
    old_genotypes: list[Genotype],
    old_fitnesses: list[Fitness],
    new_genotypes: list[Genotype],
    new_fitnesses: list[Fitness],
    selection_funct_morphological_innovation_protectionion: Callable[
        [int, list[Genotype], list[Fitness]], npt.NDArray[np.float_]
    ],
) -> tuple[list[int], list[int]]:
    """
    Select `len(old_genotypes)` individuals using the provided selection function from combined set of old and new individuals.

    :param old_genotypes: Genotypes of the individuals in the parent population.
    :param old_fitnesses: Fitnesses of the individuals in the parent population.
    :param new_genotypes: Genotypes of the individuals from the offspring.
    :param new_fitnesses: Fitnesses of the individuals from the offspring.
    :param selection_function: Function that selects n individuals from a population based on their genotype and fitness. (n, genotypes, fitnesses) -> indices
    :returns: (indices of selected individuals from parent population, indices of selected individuals from offspring).
    """
    assert len(old_genotypes) == len(old_fitnesses)
    assert len(new_genotypes) == len(new_fitnesses)

    population_size = len(old_genotypes)

    selection = selection_function(
        population_size, old_genotypes + new_genotypes, old_fitnesses + new_fitnesses
    )

    selected_old = [s for s in selection if s < len(old_fitnesses)]
    selected_new = [
        s - len(old_fitnesses) for s in selection if s >= len(old_fitnesses)
    ]

    return selected_old, selected_new
