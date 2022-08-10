from typing import Callable, List, Tuple, TypeVar

Genotype = TypeVar("Genotype")
Fitness = TypeVar("Fitness")


def steady_state(
    old_genotypes: List[Genotype],
    old_fitnesses: List[Fitness],
    new_genotypes: List[Genotype],
    new_fitnesses: List[Fitness],
    selection_fun: Callable[[int, List[Genotype], List[Fitness]], List[int]],
) -> Tuple[List[int], List[int]]:
    """
    Select `len(old_genotypes)` individuals using the provided selection function from combined set of old and new individuals.

    :param old_genotypes: Genotypes of the individuals in the parent population.
    :param old_fitnesses: Fitnesses of the individuals in the parent population.
    :param new_genotypes: Genotypes of the individuals from the offspring.
    :param new_fitnesses: Fitnesses of the individuals from the offspring.
    :param selection_fun: Function that selects n individuals from a population based on their genotype and fitness. (n, genotypes, fitnesses) -> indices
    :returns: (indices of selected individuals from parent population, indices of selected individuals from offspring).
    """
    assert len(old_genotypes) == len(old_fitnesses)
    assert len(new_genotypes) == len(new_fitnesses)

    population_size = len(old_genotypes)

    selection = selection_fun(
        population_size, old_genotypes + new_genotypes, old_fitnesses + new_fitnesses
    )

    selected_old = [s for s in selection if s < len(old_fitnesses)]
    selected_new = [
        s - len(old_fitnesses) for s in selection if s >= len(old_fitnesses)
    ]

    return selected_old, selected_new
