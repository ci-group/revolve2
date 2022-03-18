from ._genotype import Genotype
import multineat


def mutate_v1(
    genotype: Genotype,
    multineat_params: multineat.Parameters,
    innov_db: multineat.InnovationDatabase,
    rng: multineat.RNG,
) -> Genotype:
    new_genotype = multineat.Genome(genotype.genotype)
    new_genotype.Mutate(
        False,
        multineat.SearchMode.COMPLEXIFYING,
        innov_db,
        multineat_params,
        rng,
    )
    return Genotype(new_genotype)
