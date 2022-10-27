from typing import List, TypeVar

import numpy as np
import numpy.typing as npt
from revolve2.core.database.std import Rng
from revolve2.core.optimization.ea.population import Parameters, SerializableMeasures
from revolve2.core.optimization.ea.population.pop_list import PopList

TMeasures = TypeVar("TMeasures", bound=SerializableMeasures)


def de_offspring(
    population: PopList[Parameters, TMeasures],
    rng: Rng,
    differential_weight: float,
    crossover_probability: float,
) -> List[Parameters]:
    """
    Create offspring from a population using the differential evolution algorithm.

    :param population: The population to create offspring from.
    :param rng: Random number generator.
    :param differential_weight: Weight for the DE difference vector.
    :param crossover_probability: Probability for each bit to cross over.
    :returns: List of created offspring genotypes.
    """
    num_params = len(population[0].genotype)

    offspring_genotypes: List[Parameters] = []

    for individual_i, individual in enumerate(population):
        sub_pop = population[individual_i:]
        selection = rng.rng.choice(range(len(sub_pop)), 3)

        x: npt.NDArray[np.float_] = np.array(individual.genotype)
        a = np.array(sub_pop[selection[0]].genotype)
        b = np.array(sub_pop[selection[1]].genotype)
        c = np.array(sub_pop[selection[2]].genotype)

        y = a + differential_weight * (b - c)

        crossover_mask = rng.rng.binomial(n=1, p=crossover_probability, size=num_params)

        child = crossover_mask * x + (1.0 - crossover_mask) * y

        offspring_genotypes.append(Parameters([float(v) for v in child]))

    return offspring_genotypes
