from typing import Callable, List, Tuple, TypeVar

Genotype = TypeVar("Genotype")
Fitness = TypeVar("Fitness")


def generational(
    old_genotypes: List[Genotype],
    old_fitnesses: List[Fitness],
    new_genotypes: List[Genotype],
    new_fitnesses: List[Fitness],
    selection_fun: Callable[[int, List[Genotype], List[Fitness]], List[int]],
) -> Tuple[List[int], List[int]]:
    """
    Select `len(old_genotypes)` individuals using the provided selection function from only the offspring(`new_genotypes`).

    :param old_genotypes: Genotypes of the individuals in the parent population. Ignored and only here for function signature compatibility with `steady_state`.
    :param old_fitnesses: Fitnesses of the individuals in the parent population. Ignored and only here for function signature compatibility with `steady_state`.
    :param new_genotypes: Genotypes of the individuals from the offspring.
    :param new_fitnesses: Fitnesses of the individuals from the offspring.
    :param selection_fun: Function that selects n individuals from a population based on their genotype and fitness. (n, genotypes, fitnesses) -> indices
    :returns: (always empty list of indices of selected old individuals, indices of selected individuals from offspring).
    """
    population_size = len(old_genotypes)
    assert len(new_genotypes) == len(new_fitnesses)
    assert len(new_fitnesses) >= population_size

    return [], selection_fun(population_size, new_genotypes, new_fitnesses)
