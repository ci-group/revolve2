"""Functions for CPPNWIN genotypes for a modular robot CPG brain."""

import multineat
from revolve2.core.modular_robot import Body

from .._genotype import Genotype
from .._random_v1 import random_v1 as base_random_v1
from ._brain_cpg_network_neighbour_v1 import BrainCpgNetworkNeighbourV1


def random_v1(
    innov_db: multineat.InnovationDatabase,
    rng: multineat.RNG,
    multineat_params: multineat.Parameters,
    output_activation_func: multineat.ActivationFunction,
    num_initial_mutations: int,
) -> Genotype:
    """
    Create a CPPNWIN genotype for a modular robot CPG brain.

    :param innov_db: Multineat innovation database. See Multineat library.
    :param rng: Random number generator.
    :param multineat_params: Multineat parameters. See Multineat library.
    :param output_activation_func: Activation function for the output layer. See Multineat library.
    :param num_initial_mutations: The number of times to mutate to create a random network.
    :returns: The created genotype.
    """
    assert multineat_params.MutateOutputActivationFunction == False
    # other activation functions could work too, but this has been tested.
    # if you want another one, make sure it's output is between -1 and 1.
    assert output_activation_func == multineat.ActivationFunction.SIGNED_SINE

    return base_random_v1(
        innov_db,
        rng,
        multineat_params,
        output_activation_func,
        7,  # bias(always 1), x1, y1, z1, x2, y2, z2
        1,  # weight
        num_initial_mutations,
    )


def develop_v1(genotype: Genotype, body: Body) -> BrainCpgNetworkNeighbourV1:
    """
    Develop a CPPNWIN genotype into a `BrainCpgNetworkNeighbourV1` brain.

    It is important that the genotype was created using a compatible function.

    :param genotype: The genotype to create the brain from.
    :param body: # TODO This parameter is not used.
    :returns: The create brain.
    """
    return BrainCpgNetworkNeighbourV1(genotype.genotype)
