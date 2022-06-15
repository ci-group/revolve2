import multineat

from ._genotype import Genotype


def mutate_v1(
    genotype: Genotype,
    multineat_params: multineat.Parameters,
    innov_db: multineat.InnovationDatabase,
    rng: multineat.RNG,
) -> Genotype:
    new_genotype = genotype.genotype.MutateWithConstraints(
        False,
        multineat.SearchMode.BLENDED,
        innov_db,
        multineat_params,
        rng,
    )
    return Genotype(new_genotype)
