"""de_offspring function."""

from typing import List

import numpy as np
import numpy.typing as npt
from genotype import Genotype
from population import Population


def de_offspring(
    population: Population,
    rng: np.random.Generator,
    differential_weight: float,
    crossover_probability: float,
) -> List[Genotype]:
    """
    Create offspring from a population using the differential evolution algorithm.

    :param population: The population to create offspring from.
    :param rng: Random number generator.
    :param differential_weight: Weight for the DE difference vector.
    :param crossover_probability: Probability for each bit to cross over.
    :returns: List of created offspring genotypes.
    """
    num_params = len(population.individuals[0].genotype.parameters)

    offspring_genotypes: List[Genotype] = []

    for individual_i, individual in enumerate(population.individuals):
        sub_pop = population.individuals[individual_i:]
        selection = rng.choice(range(len(sub_pop)), 3)

        x: npt.NDArray[np.float_] = np.array(individual.genotype.parameters)
        a = np.array(sub_pop[selection[0]].genotype.parameters)
        b = np.array(sub_pop[selection[1]].genotype.parameters)
        c = np.array(sub_pop[selection[2]].genotype.parameters)

        y = a + differential_weight * (b - c)

        crossover_mask = rng.binomial(n=1, p=crossover_probability, size=num_params)

        child = crossover_mask * x + (1.0 - crossover_mask) * y

        offspring_genotypes.append(Genotype(tuple(float(v) for v in child)))

    return offspring_genotypes
