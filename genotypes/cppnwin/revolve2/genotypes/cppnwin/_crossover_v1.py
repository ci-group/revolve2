import multineat

from ._genotype import Genotype


def crossover_v1(
    parent1: Genotype,
    parent2: Genotype,
    multineat_params: multineat.Parameters,
    rng: multineat.RNG,
    mate_average: bool,
    interspecies_crossover: bool,
) -> Genotype:
    """
    Perform crossover between two CPPNWIN genotypes.

    Both genotypes will not be altered.

    :param parent1: Genotype 1.
    :param parent2: Genotype 2.
    :param multineat_params: Multineat parameters. See Multineat library.
    :param rng: Random number generator.
    :param mate_average: If the weights of matching connections between the parents should be averaged instead of choosing the value from one of the parents. See NEAT algorithm.
    :param interspecies_crossover: TODO description. Choose `False` if you don't know what this means. See Multineat library algorithm.
    :returns: Genotype that is the result of crossover.
    """
    new_genotype = parent1.genotype.MateWithConstraints(
        parent2.genotype,
        mate_average,
        interspecies_crossover,
        rng,
        multineat_params,
    )
    return Genotype(new_genotype)
