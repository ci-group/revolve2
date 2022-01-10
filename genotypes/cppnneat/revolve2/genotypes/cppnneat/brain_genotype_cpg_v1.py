from __future__ import annotations

from typing import cast

import multineat
from revolve2.core.database import StaticData
from revolve2.core.database.serialize import Serializable, SerializeError
from revolve2.core.modular_robot import Body as ModularRobotBody
from revolve2.core.modular_robot import Brain as ModularRobotBrain
from revolve2.core.optimization.ea.modular_robot import BrainGenotype

from .bodybrain_base import BodybrainBase
from .brain_cpg_v1 import BrainCpgV1


class BrainGenotypeCpgV1(
    BrainGenotype, BodybrainBase["BrainGenotypeCpgV1"], Serializable
):
    @classmethod
    def random(
        cls,
        innov_db: multineat.InnovationDatabase,
        rng: multineat.RNG,
        multineat_params: multineat.Parameters,
        output_activation_func: multineat.ActivationFunction,
        num_initial_mutations: int,
    ) -> BodybrainBase[BrainGenotypeCpgV1]:
        assert multineat_params.MutateOutputActivationFunction == False
        # other activation functions could work too, but this has been tested.
        # if you want another one, make sure it's output is between -1 and 1.
        assert output_activation_func == multineat.ActivationFunction.SIGNED_SINE

        return super(BrainGenotypeCpgV1, cls).random(
            innov_db,
            rng,
            multineat_params,
            output_activation_func,
            7,  # bias(always 1), x1, y1, z1, x2, y2, z2
            1,  # weight
            num_initial_mutations,
        )

    def develop(self, body: ModularRobotBody) -> ModularRobotBrain:
        return BrainCpgV1(self._genotype)

    def serialize(self) -> StaticData:
        return cast(str, self._genotype.Serialize())  # TODO missing multineat typing

    @classmethod
    def deserialize(cls, data: StaticData) -> Serializable:
        genotype = multineat.Genome()
        if type(data) != str:
            raise SerializeError()
        genotype.Deserialize(data)
        return cls(genotype)
