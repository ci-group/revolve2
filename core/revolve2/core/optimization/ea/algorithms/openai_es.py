"""
OpenAI-ES algorithm.

Call `make_perturbed_parameters` with a set of `Parameters`, evaluate the resulting `Parameters` and pass them to `step` to create the new mean.
"""

from typing import List, TypeVar

import numpy as np
import numpy.typing as npt
from revolve2.core.database.std import Rng
from revolve2.core.optimization.ea.population import Parameters, SerializableMeasures
from revolve2.core.optimization.ea.population.pop_list import PopList

TMeasures = TypeVar("TMeasures", bound=SerializableMeasures)


def make_perturbed_parameters(
    mean: Parameters, rng: Rng, population_size: int, standard_deviation: float
) -> List[Parameters]:
    """
    Perturb the provided mean `Parameters` `population_size` times using a normal distribution.

    :param mean: The Parameters to perturb.
    :param rng: Random number generator used for the normal distribution.
    :param population_size: Number of parameters to generate.
    :param standard_deviation: Standard deviation of the normal distribution.
    :returns: The generated parameters.
    """
    pertubations = rng.rng.standard_normal((population_size, len(mean)))
    return [Parameters(a) for a in standard_deviation * pertubations + mean]


def step(
    mean: Parameters,
    population: PopList[Parameters, TMeasures],
    weight_measure: str,
    learning_rate: float,
    standard_deviation: float,
) -> Parameters:
    """
    Move the mean `Parameters` in the direction of the weighted sum of the set of perturbed `Parameters`, weighted by their `weight_measure` measure.

    :param mean: Original mean `Parameters`.
    :param population: The population to combine.
    :param weight_measure: The measure to use for the weighted sum.
    :param learning_rate: How much to move.
    :param standard_deviation: Standard deviation of the normal distribution used to create the pertubations.
    :returns: The combined Parameters.
    """
    weight_measures: npt.NDArray[np.float_] = np.array(
        [i.measures[weight_measure] for i in population]
    )
    fitnesses_gaussian = (weight_measures - np.mean(weight_measures)) / np.std(
        weight_measures
    )
    return Parameters(
        np.array(mean)
        + learning_rate
        / (len(population) * standard_deviation)
        * np.dot(np.array([i.genotype for i in population]).T, fitnesses_gaussian)
    )
