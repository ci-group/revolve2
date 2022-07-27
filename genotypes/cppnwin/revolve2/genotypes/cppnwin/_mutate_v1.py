import multineat

from ._genotype import Genotype


def mutate_v1(
    genotype: Genotype,
    multineat_params: multineat.Parameters,
    innov_db: multineat.InnovationDatabase,
    rng: multineat.RNG,
) -> Genotype:
    """
    Mutate a CPPNWIN genotype.

    The genotype will not be changed; a mutated copy will be returned.

    :param genotype: The genotype to mutate. This object is not altered.
    :param multineat_params: Multineat parameters. See Multineat library.
    :param innov_db: Multineat innovation database. See Multineat library.
    :param rng: Random number generator.
    :returns: A mutated copy of the provided genotype.
    """
    new_genotype = genotype.genotype.MutateWithConstraints(
        False,
        multineat.SearchMode.BLENDED,
        innov_db,
        multineat_params,
        rng,
    )
    return Genotype(new_genotype)
