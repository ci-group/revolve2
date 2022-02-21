import multineat


def random_multineat_genotype(
    innov_db: multineat.InnovationDatabase,
    rng: multineat.RNG,
    multineat_params: multineat.Parameters,
    output_activation_func: multineat.ActivationFunction,
    num_inputs: int,
    num_outputs: int,
    num_initial_mutations: int,
) -> multineat.Genome:
    genotype = multineat.Genome(
        0,  # ID
        num_inputs,
        0,  # n_hidden
        num_outputs,
        False,  # FS_NEAT
        output_activation_func,  # output activation type
        multineat.ActivationFunction.TANH,  # hidden activation type
        0,  # seed_type
        multineat_params,
        0,  # number of hidden layers
    )

    for _ in range(num_initial_mutations):
        genotype.Mutate(
            False,
            multineat.SearchMode.COMPLEXIFYING,
            innov_db,
            multineat_params,
            rng,
        )

    return genotype
