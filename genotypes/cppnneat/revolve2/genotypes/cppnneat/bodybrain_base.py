from __future__ import annotations

from dataclasses import dataclass
from typing import Generic, TypeVar

import multineat
from revolve2.core.modular_robot.body import Body

Child = TypeVar("Child")


@dataclass
class BodybrainBase(Generic[Child]):
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

    def mutate(
        self,
        multineat_params: multineat.Parameters,
        innov_db: multineat.InnovationDatabase,
        rng: multineat.RNG,
    ) -> Child:
        new_genotype = multineat.Genome(self._genotype)
        new_genotype.Mutate(
            False,
            multineat.SearchMode.COMPLEXIFYING,
            innov_db,
            multineat_params,
            rng,
        )
        return type(self)(new_genotype)

    @classmethod
    def crossover(
        cls,
        parent1: BodybrainBase,
        parent2: BodybrainBase,
        multineat_params: multineat.Parameters,
        rng: multineat.RNG,
        mate_average: bool,
        interspecies_crossover: bool,
    ) -> Child:
        new_genotype = parent1._genotype.Mate(
            parent2._genotype,
            mate_average,
            interspecies_crossover,
            rng,
            multineat_params,
        )
        return cls(new_genotype)
