from __future__ import annotations

from dataclasses import dataclass

import multineat


@dataclass
class BodybrainBase:
    _genotype: multineat.Genome

    @classmethod
    def random(
        cls,
        innov_db: multineat.InnovationDatabase,
        rng: multineat.RNG,
        multineat_params: multineat.Parameters,
        output_activation_func: multineat.ActivationFunction,
        num_inputs: int,
        num_outputs: int,
        num_initial_mutations: int,
    ) -> BodybrainBase:
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

        return cls(genotype)
