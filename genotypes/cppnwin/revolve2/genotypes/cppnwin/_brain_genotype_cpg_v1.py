from __future__ import annotations

from dataclasses import dataclass
from typing import cast

import multineat

from revolve2.core.modular_robot import Body as ModularRobotBody
from revolve2.core.modular_robot import Brain as ModularRobotBrain
from revolve2.core.optimization.ea.modular_robot import BrainGenotype
from revolve2.serialization import Serializable, SerializeError, StaticData

from ._brain_cpg_v1 import BrainCpgV1
from ._random_multineat_genotype import random_multineat_genotype


@dataclass
class BrainGenotypeCpgV1(BrainGenotype, Serializable):
    _genotype: multineat.Genome

    @classmethod
    def random(
        cls,
        innov_db: multineat.InnovationDatabase,
        rng: multineat.RNG,
        multineat_params: multineat.Parameters,
        output_activation_func: multineat.ActivationFunction,
        num_initial_mutations: int,
    ) -> BrainGenotypeCpgV1:
        assert multineat_params.MutateOutputActivationFunction == False
        # other activation functions could work too, but this has been tested.
        # if you want another one, make sure it's output is between -1 and 1.
        assert output_activation_func == multineat.ActivationFunction.SIGNED_SINE

        return cls(
            random_multineat_genotype(
                innov_db,
                rng,
                multineat_params,
                output_activation_func,
                7,  # bias(always 1), x1, y1, z1, x2, y2, z2
                1,  # weight
                num_initial_mutations,
            )
        )

    def mutate(
        self,
        multineat_params: multineat.Parameters,
        innov_db: multineat.InnovationDatabase,
        rng: multineat.RNG,
    ) -> BrainGenotypeCpgV1:
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
        parent1: BrainGenotypeCpgV1,
        parent2: BrainGenotypeCpgV1,
        multineat_params: multineat.Parameters,
        rng: multineat.RNG,
        mate_average: bool,
        interspecies_crossover: bool,
    ) -> BrainGenotypeCpgV1:
        new_genotype = parent1._genotype.Mate(
            parent2._genotype,
            mate_average,
            interspecies_crossover,
            rng,
            multineat_params,
        )
        return cls(new_genotype)

    def develop(self, body: ModularRobotBody) -> ModularRobotBrain:
        return BrainCpgV1(self._genotype)

    def serialize(self) -> StaticData:
        return cast(str, self._genotype.Serialize())  # TODO missing multineat typing

    @classmethod
    def deserialize(cls, data: StaticData) -> BrainGenotypeCpgV1:
        genotype = multineat.Genome()
        if type(data) != str:
            raise SerializeError()
        genotype.Deserialize(data)
        return cls(genotype)
