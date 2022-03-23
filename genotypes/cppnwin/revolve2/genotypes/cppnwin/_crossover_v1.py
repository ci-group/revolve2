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
    new_genotype = parent1.genotype.Mate(
        parent2.genotype,
        mate_average,
        interspecies_crossover,
        rng,
        multineat_params,
    )
    return Genotype(new_genotype)
